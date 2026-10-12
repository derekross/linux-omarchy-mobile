// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2015 Broadcom Corporation
 *
 * The Broadcom GPS SPI driver: the BCM4775x GNSS receiver's SSI link,
 * exposed as /dev/ttyBCM for Broadcom's lhd daemon, plus the /dev/bbd_*
 * nodes lhd opens next to it (bbd.c).
 *
 * Based on Google's gs201 bcm_gps_spi.c (google-modules/gps/broadcom/
 * bcm47765). Changes:
 * - GPIO descriptors and the three supplies from the device tree; the
 *   supplies are on only while /dev/ttyBCM is open (lhd), or for the
 *   debugfs probe, and the chip goes back to standby first;
 * - 8-bit SPI words throughout (Google's switched to 32-bit words for
 *   DMA-sized transfers, which relies on its SPI controller's byte swap);
 * - the host_req interrupt is re-armed after resume only if /dev/ttyBCM is
 *   open; Google's re-armed it unconditionally;
 * - no sensor-hub (SSP), coredump or 1 Hz statistics code;
 * - a debugfs "probe" file that powers the chip, wakes its MCU and reads
 *   two host interface status registers, for bring-up.
 */

#include <linux/circ_buf.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/poll.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>

#include "bbd.h"
#include "bcm_gps_spi.h"

/* Full duplex, 2-byte packet lengths, no flow control: as Google's */
#define SSI_LEN		2

/* Time from the supplies to nstandby, and from nstandby to the MCU */
#define BCM_POWER_ON_DELAY_US	2000
#define BCM_WAKE_TIMEOUT_MS	100

/*
 * Bring-up evidence for correlating with lhd's own log: open/close of
 * /dev/ttyBCM, power, nstandby, wake handshake failures and the first SSI
 * frames of each session. On by default while the chip is being brought up.
 */
static bool events = true;
module_param(events, bool, 0644);
MODULE_PARM_DESC(events, "Log ttyBCM open/close, power, nstandby and the first frames (default: on)");

static unsigned int event_frames = 16;
module_param(event_frames, uint, 0644);
MODULE_PARM_DESC(event_frames, "SSI frames logged per session when events=1 (default 16)");

static bool ssi_dump;
module_param(ssi_dump, bool, 0644);
MODULE_PARM_DESC(ssi_dump, "Hex-dump every SSI transfer from the next open (heavy; default: off)");

#define bcm_event(priv, fmt, ...)					\
	do {								\
		if (events)						\
			dev_info(&(priv)->spi->dev, fmt, ##__VA_ARGS__);\
	} while (0)

static const char * const bcm_supply_names[BCM_NUM_SUPPLIES] = {
	"vddcore", "vddrf", "vddaux",
};

static void bcm_on_packet_received(struct bcm_spi_priv *priv,
				   unsigned char *data, unsigned int size);

/* Power */

static int bcm_power_on(struct bcm_spi_priv *priv)
{
	int ret = 0;

	mutex_lock(&priv->power_lock);
	if (!priv->powered) {
		gpiod_set_raw_value_cansleep(priv->nstandby, 0);
		gpiod_set_value_cansleep(priv->mcu_req, 0);
		ret = regulator_bulk_enable(BCM_NUM_SUPPLIES, priv->supplies);
		if (!ret) {
			usleep_range(BCM_POWER_ON_DELAY_US,
				     BCM_POWER_ON_DELAY_US + 500);
			priv->powered = true;
			bcm_event(priv, "power on (supplies enabled, standby)\n");
		} else {
			dev_err(&priv->spi->dev, "supplies: %d\n", ret);
		}
	}
	mutex_unlock(&priv->power_lock);

	return ret;
}

static void bcm_power_off(struct bcm_spi_priv *priv)
{
	mutex_lock(&priv->power_lock);
	if (priv->powered) {
		gpiod_set_value_cansleep(priv->mcu_req, 0);
		gpiod_set_raw_value_cansleep(priv->nstandby, 0);
		regulator_bulk_disable(BCM_NUM_SUPPLIES, priv->supplies);
		priv->powered = false;
		bcm_event(priv, "power off\n");
	}
	mutex_unlock(&priv->power_lock);
}

/* sysfs: the lines lhd drives itself (GpioNStdbyPath in lhd.conf/gps.xml) */

static ssize_t nstandby_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct bcm_spi_priv *priv = spi_get_drvdata(to_spi_device(dev));

	return sysfs_emit(buf, "%d\n",
			  gpiod_get_raw_value_cansleep(priv->nstandby));
}

static ssize_t nstandby_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct bcm_spi_priv *priv = spi_get_drvdata(to_spi_device(dev));
	bool on = buf[0] != '0';
	int ret;

	bcm_event(priv, "nstandby <- %d by %s[%d] (ttyBCM %s, %s)\n", on,
		  current->comm, task_pid_nr(current),
		  priv->busy ? "open" : "closed",
		  priv->powered ? "powered" : "unpowered");

	/*
	 * Stock keeps the supplies on all the time; here they follow use.
	 * Raising nstandby powers the chip if /dev/ttyBCM is not open yet,
	 * so the order lhd uses doesn't matter.
	 */
	if (on) {
		ret = bcm_power_on(priv);
		if (ret)
			return ret;
	}

	mutex_lock(&priv->power_lock);
	/* Raw level, as Google's: 1 runs the chip. */
	gpiod_set_raw_value_cansleep(priv->nstandby, on);
	mutex_unlock(&priv->power_lock);

	/* Back to standby with nobody holding the link: power off too. */
	if (!on && !priv->busy)
		bcm_power_off(priv);

	return count;
}
static DEVICE_ATTR_RW(nstandby);

static ssize_t sspmcureq_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct bcm_spi_priv *priv = spi_get_drvdata(to_spi_device(dev));

	return sysfs_emit(buf, "%d\n",
			  gpiod_get_value_cansleep(priv->mcu_req));
}

static ssize_t sspmcureq_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct bcm_spi_priv *priv = spi_get_drvdata(to_spi_device(dev));

	gpiod_set_value_cansleep(priv->mcu_req, buf[0] != '0');

	return count;
}
static DEVICE_ATTR_RW(sspmcureq);

static struct attribute *bcm_spi_attrs[] = {
	&dev_attr_nstandby.attr,
	&dev_attr_sspmcureq.attr,
	NULL
};
ATTRIBUTE_GROUPS(bcm_spi);

/* SSI framing */

static const unsigned long rx_buffer_block_size[4] = {
	32, 256, 1024 * 2, 1024 * 16
};

static unsigned long bcm_ssi_chk_pzc(struct bcm_spi_priv *priv,
				     unsigned char stat_byte)
{
	unsigned long blk_bytes = rx_buffer_block_size[(stat_byte &
			HSI_F_MOSI_CTRL_SZE_MASK) >> HSI_F_MOSI_CTRL_SZE_SHIFT];
	unsigned long blk_counts = (stat_byte & HSI_F_MOSI_CTRL_CNT_MASK) >>
			HSI_F_MOSI_CTRL_CNT_SHIFT;

	priv->rx_buffer_avail_bytes = blk_bytes * blk_counts;

	if (priv->dbg.pzc && (stat_byte & HSI_F_MOSI_CTRL_PE_MASK))
		dev_info(&priv->spi->dev, "PZC: rx stat 0x%02x (PE) avail %lu\n",
			 stat_byte, priv->rx_buffer_avail_bytes);

	return priv->rx_buffer_avail_bytes;
}

static void pk_log(struct bcm_spi_priv *priv, const char *dir,
		   const void *data, int len)
{
	if (likely(!priv->dbg.ssi))
		return;

	print_hex_dump(KERN_INFO, dir, DUMP_PREFIX_NONE, 32, 1, data, len,
		       false);
}

int bcm_spi_sync(struct bcm_spi_priv *priv, void *tx_buf, void *rx_buf,
		 int len)
{
	struct spi_transfer xfer = {
		.tx_buf = tx_buf,
		.rx_buf = rx_buf,
		.len = len,
		.bits_per_word = 8,
	};
	int ret;

	pk_log(priv, "w ", tx_buf, len);
	ret = spi_sync_transfer(priv->spi, &xfer, 1);
	pk_log(priv, "r ", rx_buf, len);
	if (ret)
		dev_err_ratelimited(&priv->spi->dev, "spi_sync error for cmd 0x%x: %d\n",
				    ((struct bcm_ssi_tx_frame *)tx_buf)->cmd, ret);

	return ret;
}

static unsigned short bcm_ssi_get_len(unsigned char ctrl_byte,
				      unsigned char *data)
{
	if (ctrl_byte & SSI_PCKT_2B_LENGTH)
		return data[0] | (data[1] << 8);

	return data[0];
}

static void bcm_ssi_set_len(unsigned char ctrl_byte, unsigned char *data,
			    unsigned short len)
{
	if (ctrl_byte & SSI_PCKT_2B_LENGTH) {
		data[0] = len & 0xff;
		data[1] = (len >> 8) & 0xff;
	} else {
		data[0] = len;
	}
}

static int bcm_ssi_tx(struct bcm_spi_priv *priv, int length)
{
	struct bcm_ssi_tx_frame *tx = priv->tx_buf;
	struct bcm_ssi_rx_frame *rx = priv->rx_buf;
	struct bcm_spi_strm_protocol *strm = &priv->tx_strm;
	unsigned short frame_data_size = strm->frame_len - strm->ctrl_len;
	unsigned short m_write = length;
	unsigned short bytes_to_write = length;
	unsigned short n_read;
	unsigned char *data_p;
	int ret;

	tx->cmd = strm->ctrl_byte;
	if (strm->pckt_len)
		bcm_ssi_set_len(strm->ctrl_byte, tx->data, m_write);

	ret = bcm_spi_sync(priv, tx, rx, bytes_to_write + strm->ctrl_len);
	if (ret) {
		priv->ssi_tx_fail++;
		return ret;
	}

	/* Full duplex: what the chip sent while we wrote */
	data_p = rx->data + strm->pckt_len;
	n_read = bcm_ssi_get_len(strm->ctrl_byte, rx->data);
	if (n_read > frame_data_size) {
		dev_err(&priv->spi->dev, "FD n_read %u overflows frame %u\n",
			n_read, frame_data_size);
		n_read = frame_data_size;
	}
	if (m_write < n_read) {
		bcm_on_packet_received(priv, data_p, m_write);
		n_read -= m_write;
		bytes_to_write -= m_write;
		data_p += m_write + strm->fc_len;
	} else {
		bytes_to_write = n_read;
	}
	if (bytes_to_write)
		bcm_on_packet_received(priv, data_p, bytes_to_write);

	bcm_ssi_chk_pzc(priv, rx->status);

	if (events && priv->ev_frames < event_frames) {
		priv->ev_frames++;
		dev_info(&priv->spi->dev, "tx frame %u bytes, rx status 0x%02x, FD in %u\n",
			 m_write, rx->status, bytes_to_write);
	}

	return 0;
}

static int bcm_ssi_rx(struct bcm_spi_priv *priv, size_t *length)
{
	struct bcm_ssi_tx_frame *tx = priv->tx_buf;
	struct bcm_ssi_rx_frame *rx = priv->rx_buf;
	struct circ_buf *rd_circ = &priv->read_buf;
	struct bcm_spi_strm_protocol *strm = &priv->rx_strm;
	unsigned short ctrl_len = strm->pckt_len + 1;	/* + status byte */
	unsigned short payload_len;

	bcm_ssi_set_len(strm->ctrl_byte, tx->data, 0);
	tx->cmd = strm->ctrl_byte;
	rx->status = 0;
	if (bcm_spi_sync(priv, tx, rx, ctrl_len))
		return -EIO;

	bcm_ssi_chk_pzc(priv, rx->status);

	payload_len = bcm_ssi_get_len(strm->ctrl_byte, rx->data);
	if (!payload_len) {
		if (events)
			dev_info_ratelimited(&priv->spi->dev,
					     "host_req but RX length 0 (status 0x%02x)\n",
					     rx->status);
		return -EAGAIN;
	}

	*length = min_t(size_t, strm->frame_len - ctrl_len, payload_len);

	/* Slow down if lhd is not keeping up (half the ring is full) */
	if (CIRC_CNT(rd_circ->head, rd_circ->tail, BCM_SPI_READ_BUF_SIZE) >
	    BCM_SPI_READ_BUF_SIZE / 2) {
		msleep(DELAY_FOR_SYSTEM_OVERLOADED_MS);
		if (*length >= READ_SIZE_FOR_SYSTEM_OVERLOADED)
			*length = READ_SIZE_FOR_SYSTEM_OVERLOADED;
	}

	memset(tx->data, 0, *length + ctrl_len - 1);
	if (bcm_spi_sync(priv, tx, rx, *length + ctrl_len))
		return -EIO;

	payload_len = bcm_ssi_get_len(strm->ctrl_byte, rx->data);
	if (payload_len < *length)
		*length = payload_len;

	if (events && priv->ev_frames < event_frames) {
		priv->ev_frames++;
		dev_info(&priv->spi->dev, "rx frame %zu bytes, status 0x%02x\n",
			 *length, rx->status);
	}

	return 0;
}

static void bcm_check_overrun(struct bcm_spi_priv *priv, size_t avail)
{
	unsigned long now = jiffies;

	if (!avail)
		return;

	if (time_before(now, priv->last_tick + msecs_to_jiffies(100))) {
		priv->skip_count++;
		return;
	}

	if (priv->skip_count)
		dev_err(&priv->spi->dev, "%lu messages skipped\n",
			priv->skip_count);
	dev_err(&priv->spi->dev, "input overrun by %zu bytes\n", avail);
	priv->skip_count = 0;
	priv->last_tick = now;
}

static void bcm_on_packet_received(struct bcm_spi_priv *priv,
				   unsigned char *data, unsigned int size)
{
	struct circ_buf *rd_circ = &priv->read_buf;
	size_t written = 0, avail = size;

	mutex_lock(&priv->rlock);
	while (avail > 0 && CIRC_SPACE(rd_circ->head, rd_circ->tail,
				       BCM_SPI_READ_BUF_SIZE)) {
		size_t space_to_end = CIRC_SPACE_TO_END(rd_circ->head,
				rd_circ->tail, BCM_SPI_READ_BUF_SIZE);
		size_t copied = min(space_to_end, avail);

		memcpy(rd_circ->buf + rd_circ->head, data + written, copied);
		avail -= copied;
		written += copied;
		rd_circ->head = (rd_circ->head + copied) &
				(BCM_SPI_READ_BUF_SIZE - 1);
	}
	priv->packet_received += size;
	mutex_unlock(&priv->rlock);

	wake_up(&priv->poll_wait);
	bcm_check_overrun(priv, avail);
}

/* Wake the chip's MCU: raise mcu_req until it answers on mcu_resp. */
static bool bcm477x_hello(struct bcm_spi_priv *priv)
{
	int count = 0;

	gpiod_set_value_cansleep(priv->mcu_req, 1);
	while (!gpiod_get_value_cansleep(priv->mcu_resp)) {
		if (count++ > BCM_WAKE_TIMEOUT_MS) {
			gpiod_set_value_cansleep(priv->mcu_req, 0);
			dev_err_ratelimited(&priv->spi->dev,
					    "MCU_RESP does not answer MCU_REQ (nstandby %d)\n",
					    gpiod_get_raw_value_cansleep(priv->nstandby));
			return false;
		}

		usleep_range(1000, 2000);
		if (gpiod_get_value_cansleep(priv->mcu_resp))
			break;

		/* Toggle the request every 20 tries, as Google's */
		if (count % 20 == 0) {
			gpiod_set_value_cansleep(priv->mcu_req, 0);
			usleep_range(1000, 2000);
			gpiod_set_value_cansleep(priv->mcu_req, 1);
			usleep_range(1000, 2000);
		}
	}

	return true;
}

static void bcm477x_bye(struct bcm_spi_priv *priv)
{
	gpiod_set_value_cansleep(priv->mcu_req, 0);
}

static void bcm_irq_enable(struct bcm_spi_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->irq_lock, flags);
	if (!atomic_xchg(&priv->irq_enabled, 1))
		enable_irq(priv->irq);
	spin_unlock_irqrestore(&priv->irq_lock, flags);
}

static void bcm_irq_disable(struct bcm_spi_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->irq_lock, flags);
	if (atomic_xchg(&priv->irq_enabled, 0))
		disable_irq_nosync(priv->irq);
	spin_unlock_irqrestore(&priv->irq_lock, flags);
}

static void bcm_rxtx_work_func(struct work_struct *work)
{
	struct bcm_spi_priv *priv = container_of(work, struct bcm_spi_priv,
						 rxtx_work);
	struct circ_buf *rd_circ = &priv->read_buf;
	struct circ_buf *wr_circ = &priv->write_buf;
	struct bcm_spi_strm_protocol *strm = &priv->tx_strm;
	unsigned short rx_pckt_len = priv->rx_strm.pckt_len;
	int wait_for_pzc = 0;

	if (!bcm477x_hello(priv))
		goto out;

	do {
		size_t avail = 0, written = 0;
		int ret = 0;

		/* Read first */
		if (gpiod_get_value_cansleep(priv->host_req) || wait_for_pzc) {
			wait_for_pzc = 0;
			ret = bcm_ssi_rx(priv, &avail);
			if (ret == -EIO)
				break;
			if (!ret)
				bcm_on_packet_received(priv,
					priv->rx_buf->data + rx_pckt_len,
					avail);
		}

		/* Then write */
		avail = CIRC_CNT(wr_circ->head, wr_circ->tail,
				 BCM_SPI_WRITE_BUF_SIZE);
		if (!avail)
			continue;

		mutex_lock(&priv->wlock);
		if (avail > strm->frame_len - strm->ctrl_len)
			avail = strm->frame_len - strm->ctrl_len;

		/*
		 * Don't send more than the chip's receive ring can take
		 * (its "PZC" status). When it is full, read and retry.
		 */
		if (avail > priv->rx_buffer_avail_bytes) {
			if (priv->rx_buffer_avail_bytes)
				priv->ssi_tx_pzc_retries++;
			else
				priv->ssi_tx_pzc_retry_delays++;

			if (!priv->rx_buffer_avail_bytes && priv->dbg.rng) {
				u32 regval32[3];

				bcm_ireg_read(priv, "RNGDMA_RX",
					      HSI_RNGDMA_RX_SW_ADDR_OFFSET,
					      regval32, 3);
			}
			avail = priv->rx_buffer_avail_bytes;
			usleep_range(1000, 2000);
			wait_for_pzc = 1;
		}

		while (avail > 0) {
			size_t cnt_to_end = CIRC_CNT_TO_END(wr_circ->head,
					wr_circ->tail, BCM_SPI_WRITE_BUF_SIZE);
			size_t copied = min(cnt_to_end, avail);

			memcpy(priv->tx_buf->data + strm->pckt_len + written,
			       wr_circ->buf + wr_circ->tail, copied);
			avail -= copied;
			written += copied;
			wr_circ->tail = (wr_circ->tail + copied) &
					(BCM_SPI_WRITE_BUF_SIZE - 1);
		}

		if (written)
			ret = bcm_ssi_tx(priv, written);
		mutex_unlock(&priv->wlock);
		if (ret)
			break;

		/* Wake lhd only once the ring is 1/8 full while looping */
		if (CIRC_CNT(rd_circ->head, rd_circ->tail,
			     BCM_SPI_READ_BUF_SIZE) > BCM_SPI_READ_BUF_SIZE / 8)
			wake_up(&priv->poll_wait);
	} while (!atomic_read(&priv->suspending) &&
		 (gpiod_get_value_cansleep(priv->host_req) ||
		  CIRC_CNT(wr_circ->head, wr_circ->tail,
			   BCM_SPI_WRITE_BUF_SIZE)));

	bcm477x_bye(priv);
out:
	wake_up(&priv->poll_wait);

	if (!atomic_read(&priv->suspending) && priv->busy)
		bcm_irq_enable(priv);
}

static irqreturn_t bcm_irq_handler(int irq, void *data)
{
	struct bcm_spi_priv *priv = data;

	/* Level interrupt: off until the worker has drained the chip */
	spin_lock(&priv->irq_lock);
	if (atomic_xchg(&priv->irq_enabled, 0))
		disable_irq_nosync(priv->irq);
	spin_unlock(&priv->irq_lock);

	if (!atomic_read(&priv->suspending))
		queue_work(priv->serial_wq, &priv->rxtx_work);

	return IRQ_HANDLED;
}

/* /dev/ttyBCM */

static void bcm_ssi_init_strm(struct bcm_spi_priv *priv)
{
	struct bcm_spi_strm_protocol *strm;
	unsigned char len_mask = SSI_LEN == 2 ? SSI_PCKT_2B_LENGTH :
						SSI_PCKT_1B_LENGTH;

	strm = &priv->tx_strm;
	strm->pckt_len = SSI_LEN;
	strm->fc_len = 0;
	strm->ctrl_len = strm->pckt_len + strm->fc_len + 1;	/* + cmd */
	strm->frame_len = SSI_LEN == 2 ? MAX_SPI_FRAME_LEN :
					 MAX_SPI_DREG_FRAME_LEN;
	strm->ctrl_byte = SSI_MODE_FULL_DUPLEX | SSI_MODE_STREAM | len_mask |
			  SSI_WRITE_TRANS | SSI_FLOW_CONTROL_DISABLED;

	strm = &priv->rx_strm;
	strm->pckt_len = SSI_LEN;
	strm->fc_len = 0;
	strm->ctrl_len = strm->pckt_len + strm->fc_len + 1;	/* + status */
	strm->frame_len = SSI_LEN == 2 ? MAX_SPI_FRAME_LEN :
					 MAX_SPI_DREG_FRAME_LEN;
	/*
	 * Google's picks SSI_READ_TRANS here (its "SSI_MODE ==
	 * SSI_MODE_FULL_DUPLEX" test compares 1 with 0x40), so the read
	 * command byte stock sends is 0x70. Keep that.
	 */
	strm->ctrl_byte = SSI_MODE_FULL_DUPLEX | SSI_MODE_STREAM | len_mask |
			  SSI_READ_TRANS;
}

static int bcm_spi_open(struct inode *inode, struct file *filp)
{
	struct bcm_spi_priv *priv = container_of(filp->private_data,
						 struct bcm_spi_priv, misc);
	int ret;

	if (priv->busy)
		return -EBUSY;

	ret = bcm_power_on(priv);
	if (ret)
		return ret;

	priv->busy = true;
	priv->read_buf.head = priv->read_buf.tail = 0;
	priv->write_buf.head = priv->write_buf.tail = 0;
	priv->packet_received = 0;
	priv->ssi_tx_fail = 0;
	priv->ssi_tx_pzc_retries = 0;
	priv->ssi_tx_pzc_retry_delays = 0;
	priv->rx_buffer_avail_bytes = HSI_PZC_MAX_RX_BUFFER;
	priv->ev_frames = 0;
	priv->dbg.ssi = ssi_dump;
	bcm_ssi_init_strm(priv);
	bcm_event(priv, "ttyBCM opened by %s[%d]\n", current->comm,
		  task_pid_nr(current));

	filp->private_data = priv;
	bcm_irq_enable(priv);
	enable_irq_wake(priv->irq);

	return 0;
}

static int bcm_spi_release(struct inode *inode, struct file *filp)
{
	struct bcm_spi_priv *priv = filp->private_data;

	bcm_event(priv, "ttyBCM closed by %s[%d] after %lu bytes in, %d tx failures\n",
		  current->comm, task_pid_nr(current), priv->packet_received,
		  priv->ssi_tx_fail);
	disable_irq_wake(priv->irq);
	bcm_irq_disable(priv);
	priv->busy = false;
	flush_workqueue(priv->serial_wq);
	bcm_power_off(priv);

	return 0;
}

static ssize_t bcm_spi_read(struct file *filp, char __user *buf,
			    size_t size, loff_t *ppos)
{
	struct bcm_spi_priv *priv = filp->private_data;
	struct circ_buf *circ = &priv->read_buf;
	size_t rd_size = 0;

	mutex_lock(&priv->rlock);
	while (size > 0 && CIRC_CNT(circ->head, circ->tail,
				    BCM_SPI_READ_BUF_SIZE)) {
		size_t cnt_to_end = CIRC_CNT_TO_END(circ->head, circ->tail,
						    BCM_SPI_READ_BUF_SIZE);
		size_t copied = min(cnt_to_end, size);

		if (copy_to_user(buf + rd_size, circ->buf + circ->tail,
				 copied)) {
			mutex_unlock(&priv->rlock);
			return -EFAULT;
		}
		size -= copied;
		rd_size += copied;
		circ->tail = (circ->tail + copied) &
			     (BCM_SPI_READ_BUF_SIZE - 1);
	}
	mutex_unlock(&priv->rlock);

	return rd_size;
}

static ssize_t bcm_spi_write(struct file *filp, const char __user *buf,
			     size_t size, loff_t *ppos)
{
	struct bcm_spi_priv *priv = filp->private_data;
	struct circ_buf *circ = &priv->write_buf;
	size_t wr_size = 0;

	mutex_lock(&priv->wlock);
	while (size > 0 && CIRC_SPACE(circ->head, circ->tail,
				      BCM_SPI_WRITE_BUF_SIZE)) {
		size_t space_to_end = CIRC_SPACE_TO_END(circ->head,
				circ->tail, BCM_SPI_WRITE_BUF_SIZE);
		size_t copied = min(space_to_end, size);

		if (copy_from_user(circ->buf + circ->head, buf + wr_size,
				   copied)) {
			mutex_unlock(&priv->wlock);
			return -EFAULT;
		}
		size -= copied;
		wr_size += copied;
		circ->head = (circ->head + copied) &
			     (BCM_SPI_WRITE_BUF_SIZE - 1);
	}
	mutex_unlock(&priv->wlock);

	if (!atomic_read(&priv->suspending))
		queue_work(priv->serial_wq, &priv->rxtx_work);

	return wr_size;
}

static __poll_t bcm_spi_poll(struct file *filp, poll_table *wait)
{
	struct bcm_spi_priv *priv = filp->private_data;
	struct circ_buf *rd = &priv->read_buf;
	struct circ_buf *wr = &priv->write_buf;
	__poll_t mask = 0;

	poll_wait(filp, &priv->poll_wait, wait);

	if (CIRC_CNT(rd->head, rd->tail, BCM_SPI_READ_BUF_SIZE))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (CIRC_SPACE(wr->head, wr->tail, BCM_SPI_WRITE_BUF_SIZE))
		mask |= EPOLLOUT | EPOLLWRNORM;

	return mask;
}

static const struct file_operations bcm_spi_fops = {
	.owner		= THIS_MODULE,
	.open		= bcm_spi_open,
	.release	= bcm_spi_release,
	.read		= bcm_spi_read,
	.write		= bcm_spi_write,
	.poll		= bcm_spi_poll,
	.llseek		= noop_llseek,
};

/*
 * debugfs "probe": with /dev/ttyBCM closed, power the chip, take it out of
 * standby, wake its MCU and read HSI_STATUS and HSI_ERROR_STATUS through an
 * SSI debug frame. Then everything goes back off. Bring-up only.
 */
static int bcm_probe_show(struct seq_file *s, void *unused)
{
	struct bcm_spi_priv *priv = s->private;
	u8 status = 0xff, error = 0xff;
	bool awake;
	int ret;

	if (priv->busy)
		return -EBUSY;

	ret = bcm_power_on(priv);
	if (ret)
		return ret;

	gpiod_set_raw_value_cansleep(priv->nstandby, 1);
	msleep(20);
	awake = bcm477x_hello(priv);
	if (awake) {
		mutex_lock(&priv->wlock);
		bcm_dreg_read(priv, "HSI_STATUS", HSI_STATUS, &status, 1);
		bcm_dreg_read(priv, "HSI_ERROR_STATUS", HSI_ERROR_STATUS,
			      &error, 1);
		mutex_unlock(&priv->wlock);
	}
	bcm477x_bye(priv);
	bcm_power_off(priv);

	seq_printf(s, "mcu_resp: %s\nHSI_STATUS: 0x%02x\nHSI_ERROR_STATUS: 0x%02x\n",
		   awake ? "answered" : "no answer", status, error);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(bcm_probe);

static int bcm_pins_show(struct seq_file *s, void *unused)
{
	struct bcm_spi_priv *priv = s->private;

	seq_printf(s, "powered: %d\nnstandby: %d\nmcu_req: %d\nmcu_resp: %d\nhost_req: %d\nirq_enabled: %d\n",
		   priv->powered,
		   gpiod_get_raw_value_cansleep(priv->nstandby),
		   gpiod_get_value_cansleep(priv->mcu_req),
		   gpiod_get_value_cansleep(priv->mcu_resp),
		   gpiod_get_value_cansleep(priv->host_req),
		   atomic_read(&priv->irq_enabled));
	seq_printf(s, "packets: %lu\ntx_fail: %d\npzc_retries: %d\npzc_delays: %d\n",
		   priv->packet_received, priv->ssi_tx_fail,
		   priv->ssi_tx_pzc_retries, priv->ssi_tx_pzc_retry_delays);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(bcm_pins);

/* SPI driver */

static int bcm_spi_suspend(struct device *dev)
{
	struct bcm_spi_priv *priv = spi_get_drvdata(to_spi_device(dev));

	atomic_set(&priv->suspending, 1);
	bcm_irq_disable(priv);
	flush_workqueue(priv->serial_wq);

	return 0;
}

static int bcm_spi_resume(struct device *dev)
{
	struct bcm_spi_priv *priv = spi_get_drvdata(to_spi_device(dev));

	atomic_set(&priv->suspending, 0);
	if (priv->busy)
		bcm_irq_enable(priv);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(bcm_spi_pm_ops, bcm_spi_suspend,
				bcm_spi_resume);

static int bcm_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct bcm_spi_priv *priv;
	int i, ret;

	priv = kvzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->spi = spi;
	spi_set_drvdata(spi, priv);

	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		goto err_free;

	/* All lines start low: chip in standby, no wake request. */
	priv->nstandby = devm_gpiod_get(dev, "nstandby", GPIOD_ASIS);
	if (IS_ERR(priv->nstandby)) {
		ret = PTR_ERR(priv->nstandby);
		goto err_free;
	}
	ret = gpiod_direction_output_raw(priv->nstandby, 0);
	if (ret)
		goto err_free;

	priv->mcu_req = devm_gpiod_get(dev, "mcu-req", GPIOD_OUT_LOW);
	if (IS_ERR(priv->mcu_req)) {
		ret = PTR_ERR(priv->mcu_req);
		goto err_free;
	}

	priv->mcu_resp = devm_gpiod_get(dev, "mcu-resp", GPIOD_IN);
	if (IS_ERR(priv->mcu_resp)) {
		ret = PTR_ERR(priv->mcu_resp);
		goto err_free;
	}

	priv->host_req = devm_gpiod_get(dev, "host-req", GPIOD_IN);
	if (IS_ERR(priv->host_req)) {
		ret = PTR_ERR(priv->host_req);
		goto err_free;
	}

	priv->irq = spi->irq > 0 ? spi->irq : gpiod_to_irq(priv->host_req);
	if (priv->irq < 0) {
		ret = priv->irq;
		goto err_free;
	}

	for (i = 0; i < BCM_NUM_SUPPLIES; i++)
		priv->supplies[i].supply = bcm_supply_names[i];
	ret = devm_regulator_bulk_get(dev, BCM_NUM_SUPPLIES, priv->supplies);
	if (ret)
		goto err_free;

	priv->tx_buf = devm_kzalloc(dev, sizeof(*priv->tx_buf), GFP_KERNEL);
	priv->rx_buf = devm_kzalloc(dev, sizeof(*priv->rx_buf), GFP_KERNEL);
	if (!priv->tx_buf || !priv->rx_buf) {
		ret = -ENOMEM;
		goto err_free;
	}

	init_waitqueue_head(&priv->poll_wait);
	priv->read_buf.buf = priv->_read_buf;
	priv->write_buf.buf = priv->_write_buf;
	mutex_init(&priv->rlock);
	mutex_init(&priv->wlock);
	mutex_init(&priv->power_lock);
	spin_lock_init(&priv->irq_lock);
	atomic_set(&priv->irq_enabled, 0);
	atomic_set(&priv->suspending, 0);
	INIT_WORK(&priv->rxtx_work, bcm_rxtx_work_func);

	priv->serial_wq = alloc_workqueue("bcm477x_wq",
					  WQ_HIGHPRI | WQ_UNBOUND |
					  WQ_MEM_RECLAIM, 1);
	if (!priv->serial_wq) {
		ret = -ENOMEM;
		goto err_free;
	}

	ret = request_irq(priv->irq, bcm_irq_handler,
			  IRQF_TRIGGER_HIGH | IRQF_NO_AUTOEN, "ttyBCM", priv);
	if (ret)
		goto err_wq;

	priv->bbd = bbd_init(dev, &priv->dbg);
	if (IS_ERR(priv->bbd)) {
		ret = PTR_ERR(priv->bbd);
		goto err_irq;
	}

	priv->misc.minor = MISC_DYNAMIC_MINOR;
	priv->misc.name = "ttyBCM";
	priv->misc.fops = &bcm_spi_fops;
	priv->misc.parent = dev;
	ret = misc_register(&priv->misc);
	if (ret)
		goto err_bbd;

	priv->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("probe", 0400, priv->debugfs, priv,
			    &bcm_probe_fops);
	debugfs_create_file("pins", 0400, priv->debugfs, priv,
			    &bcm_pins_fops);

	dev_info(dev, "BCM4775x GNSS on SPI, irq %d; supplies off until /dev/ttyBCM opens\n",
		 priv->irq);

	return 0;

err_bbd:
	bbd_exit(priv->bbd);
err_irq:
	free_irq(priv->irq, priv);
err_wq:
	destroy_workqueue(priv->serial_wq);
err_free:
	kvfree(priv);
	return dev_err_probe(dev, ret, "probe failed\n");
}

static void bcm_spi_teardown(struct bcm_spi_priv *priv)
{
	atomic_set(&priv->suspending, 1);
	bcm_irq_disable(priv);
	flush_workqueue(priv->serial_wq);
	bcm_power_off(priv);
}

static void bcm_spi_remove(struct spi_device *spi)
{
	struct bcm_spi_priv *priv = spi_get_drvdata(spi);

	debugfs_remove_recursive(priv->debugfs);
	misc_deregister(&priv->misc);
	bcm_spi_teardown(priv);
	bbd_exit(priv->bbd);
	free_irq(priv->irq, priv);
	destroy_workqueue(priv->serial_wq);
	kvfree(priv);
}

static void bcm_spi_shutdown(struct spi_device *spi)
{
	bcm_spi_teardown(spi_get_drvdata(spi));
}

static const struct of_device_id bcm_spi_of_match[] = {
	{ .compatible = "brcm,bcm4775-gnss" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm_spi_of_match);

static const struct spi_device_id bcm_spi_id[] = {
	{ "bcm4775-gnss" },
	{ }
};
MODULE_DEVICE_TABLE(spi, bcm_spi_id);

static struct spi_driver bcm_spi_driver = {
	.driver = {
		.name = "bcm4775x-gnss",
		.of_match_table = bcm_spi_of_match,
		.pm = pm_sleep_ptr(&bcm_spi_pm_ops),
		.dev_groups = bcm_spi_groups,
	},
	.id_table = bcm_spi_id,
	.probe = bcm_spi_probe,
	.remove = bcm_spi_remove,
	.shutdown = bcm_spi_shutdown,
};
module_spi_driver(bcm_spi_driver);

MODULE_DESCRIPTION("Broadcom BCM4775x GNSS SPI/SSI bridge for lhd");
MODULE_LICENSE("GPL");
