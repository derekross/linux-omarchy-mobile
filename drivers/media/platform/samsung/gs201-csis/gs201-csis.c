// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google gs201 (Tensor G2) CSIS camera receiver and CSIS WDMA
 *
 * The gs201 camera block has eight MIPI CSI-2 receiver links (CSIS, a
 * newer version of the receiver in imx-mipi-csis), each behind a Samsung
 * DC-PHY slave, and a DMA block whose "CSIS WDMA" contexts write a link's
 * frames straight to memory. Stock uses that path for its RAW output; the
 * rest of the ISP (PDP, 3AA, ...) is not involved and not driven here.
 *
 * One device instance drives one link and one WDMA context, as one media
 * device: sensor -> CSIS subdev -> WDMA video node. Raw Bayer only, packed
 * as on the CSI-2 bus. Several links may share a WDMA context (the
 * cameras on this phone all use context 0), and the routing registers in
 * the CSIS sysreg and the DMA block's common page are shared by all of
 * them, so only one instance streams at a time.
 *
 * Links run D-PHY or C-PHY, from the bus type of the link's endpoint.
 *
 * Register offsets come from Google's GPL gs201 device tree (the lwis CSIS
 * and DMA nodes and their interrupt bit names) and match imx-mipi-csis for
 * the link. The meaning of the WDMA registers is documented in
 * gs201-csis-regs.h, with what is known and what is inferred.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <dt-bindings/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>

#include <media/mipi-csi2.h>
#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>

#include "gs201-csis-regs.h"
#include "zumapro-csis-core.h"

#define CSIS_DRIVER_NAME	"gs201-csis"

#define CSIS_PAD_SINK		0
#define CSIS_PAD_SOURCE		1
#define CSIS_PADS_NUM		2

#define CSIS_DEF_WIDTH		640
#define CSIS_DEF_HEIGHT		480
#define CSIS_MAX_WIDTH		8192
#define CSIS_MAX_HEIGHT		8192
#define CSIS_STRIDE_ALIGN	64	/* bytes; the stride known to work [I] */
#define CSIS_STRIDE_ALIGN_MAX	4096

/* Lane rate limits of the DC-PHY in D-PHY mode */
#define CSIS_MIN_LANE_RATE	80000000ULL
#define CSIS_MAX_LANE_RATE	6500000000ULL

/*
 * zumapro (Tensor G4) has the same CSI-2 link (version 0x08010000 against
 * gs201's 0x05040000, same layout) but a different DMA path: the ISP front
 * end's CSIS core, whose image logical channel (LoCH) takes a link's data
 * type on one virtual channel and writes it to memory through its Bayer
 * WDMA (zumapro-csis-core.h). Without the core's registers in the device
 * tree the link runs alone (as link_only), counting frames.
 */
struct gs201_csis_soc {
	bool wdma;		/* gs201's CSIS WDMA, routing and CSIS sysreg */
	unsigned int num_links;
	u32 cmn_ctrl_extra;	/* CMN_CTRL bits the SoC's camera software sets */
	unsigned int pixel_mode;	/* the link channel's, unless overridden */
	unsigned int dma_bits;
	/*
	 * DBG_OPTION_SUITE bit 23: gs201's camera software sets it, and the link
	 * then hands out unpacked 16-bit words, which gs201's WDMA expects.
	 * zumapro's clears it: its CSIS core unpacks the packed CSI-2 stream.
	 */
	bool dbg_bit23;
	/* PHY_CMN_CTRL lane enables: gs201 sets them; zumapro never writes it */
	bool phy_cmn_ctrl;
};

struct gs201_csis_format {
	u32 code;
	u32 fourcc;
	u8 data_type;
	u8 bpp;
	u8 wdma_fmt;
};

#define CSIS_FMT(_code, _fourcc, _dt, _bpp, _wdma) { \
	.code = MEDIA_BUS_FMT_##_code, .fourcc = V4L2_PIX_FMT_##_fourcc, \
	.data_type = _dt, .bpp = _bpp, .wdma_fmt = _wdma }

static const struct gs201_csis_format gs201_csis_formats[] = {
	CSIS_FMT(SRGGB10_1X10, SRGGB10P, MIPI_CSI2_DT_RAW10, 10, CSIS_WDMA_FMT_RAW10),
	CSIS_FMT(SGRBG10_1X10, SGRBG10P, MIPI_CSI2_DT_RAW10, 10, CSIS_WDMA_FMT_RAW10),
	CSIS_FMT(SGBRG10_1X10, SGBRG10P, MIPI_CSI2_DT_RAW10, 10, CSIS_WDMA_FMT_RAW10),
	CSIS_FMT(SBGGR10_1X10, SBGGR10P, MIPI_CSI2_DT_RAW10, 10, CSIS_WDMA_FMT_RAW10),
	CSIS_FMT(SRGGB8_1X8, SRGGB8, MIPI_CSI2_DT_RAW8, 8, CSIS_WDMA_FMT_RAW8),
	CSIS_FMT(SGRBG8_1X8, SGRBG8, MIPI_CSI2_DT_RAW8, 8, CSIS_WDMA_FMT_RAW8),
	CSIS_FMT(SGBRG8_1X8, SGBRG8, MIPI_CSI2_DT_RAW8, 8, CSIS_WDMA_FMT_RAW8),
	CSIS_FMT(SBGGR8_1X8, SBGGR8, MIPI_CSI2_DT_RAW8, 8, CSIS_WDMA_FMT_RAW8),
};

struct gs201_csis_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	dma_addr_t addr;
};

struct gs201_csis_stats {
	u32 frame_start;
	u32 frame_end;
	u32 link_errors;
	u32 last_link_err;
	u32 wdma_done;
	u32 wdma_errors;
	u32 last_wdma_err;
	u32 dropped;
	u32 missed_fe;
	u32 link_irqs;
	u32 core_irqs;		/* zumapro: core interrupts this link handled */
	u32 core_last_agg;
	u32 core_last_proc;
	u32 wdma_started_at;	/* the link frame end the WDMA started at */
	u32 wdma_restarts;	/* wdma_recover restarts */
	u32 wdma_lines;		/* line ends since the channel's last frame start */
	/* bring-up: how often each error bit was set, per interrupt */
	u32 int0_bits[32];
	u32 int1_bits[8];
	u32 wdma_bits[32];
	/* bring-up: frame syncs per link VC bit, and per WDMA channel */
	u32 link_fs[32];
	u32 link_fe[32];
	u32 wdma_fs[4];
	u32 wdma_fe[4];
	u32 wdma_le[4];
};

struct gs201_csis {
	const struct gs201_csis_soc *soc;
	void __iomem *core;	/* zumapro: the CSIS core page, or NULL */
	unsigned int loch;	/* zumapro: the image LoCH this link writes through */
	void *sq_buf;		/* zumapro: the LoCH's status queue */
	dma_addr_t sq_dma;
	struct clk *fe_clk;	/* zumapro: the ISP front end's NOC clock */
	unsigned long fe_rate_old;
	void *scratch;		/* zumapro: frames with no buffer go here */
	dma_addr_t scratch_dma;
	size_t scratch_size;
	struct device *dev;
	void __iomem *link;
	void __iomem *dma;
	void __iomem *wdma;
	void __iomem *ebuf_regs;	/* optional, shared */
	struct phy *phy;
	struct clk *cam_clk;		/* optional: ACPM CAM DVFS domain */
	struct clk *int_clk;		/* optional: ACPM INT (bus) DVFS domain */
	unsigned long cam_rate_old;
	unsigned long int_rate_old;
	u32 sc_con_saved[CSIS_SYSREG_SC_CON_NUM];
	u32 ebuf_in_saved;
	u32 link_sel_saved;
	u32 ebuf_ctrl_saved;
	unsigned int link_id;
	unsigned int ctx_id;
	unsigned int ebuf;
	unsigned int wdma_ch;
	bool dma_on;
	unsigned int wdma_arm;		/* link frame ends left before the WDMA starts */
	bool cphy;			/* the link's endpoint is C-PHY */
	bool lrte_epd;			/* the sensor sends LRTE packet delimiters */
	struct regmap *sysreg;

	struct v4l2_device v4l2_dev;
	struct media_device mdev;
	struct v4l2_async_notifier notifier;
	struct v4l2_subdev sd;
	struct media_pad pads[CSIS_PADS_NUM];
	struct v4l2_mbus_config_mipi_csi2 bus;

	struct {
		struct v4l2_subdev *sd;
		struct media_pad *pad;
	} source;

	struct video_device vdev;
	struct media_pad vdev_pad;
	struct vb2_queue queue;
	struct mutex vlock;		/* video node and queue */
	struct v4l2_pix_format pix;
	const struct gs201_csis_format *fmt;

	spinlock_t slock;		/* buffers, stats */
	struct list_head pending;
	struct gs201_csis_buffer *active;
	struct gs201_csis_buffer *next;
	u32 sequence;
	struct gs201_csis_stats stats;
	bool wdma_err_logged;

	struct dentry *debugfs;
};

/*
 * The capture path has run on the phone (CAMERA-PLAN.md CAM4, CAM5), so
 * streaming is allowed by default; the camera modules' blacklist is the
 * bring-up gate. allow_stream=0 refuses any stream.
 */
static bool allow_stream = true;
module_param(allow_stream, bool, 0644);
MODULE_PARM_DESC(allow_stream,
		 "Allow the link and WDMA to be started (default on; 0 refuses)");

/*
 * CAM3b: run the PHY, the link and the sensor but leave the WDMA (and its
 * routing) off, so the first test can't write memory. No buffer completes.
 */
static bool link_only;
module_param(link_only, bool, 0644);
MODULE_PARM_DESC(link_only, "Start the link without the WDMA (bring-up, no DMA)");

/*
 * CAM3b.4 experiment: stock sets bit (link + 8 * PDP channel) in CSIS
 * sysreg 0x488; with no PDP that is bit link + 24 [HAL, I for RAW-only].
 */
static bool sysreg_link_sel;
module_param(sysreg_link_sel, bool, 0644);
MODULE_PARM_DESC(sysreg_link_sel, "Set bit (link + 24) of CSIS sysreg 0x488 at stream on (bring-up)");

/*
 * The camera block boots with its DVFS domain at its lowest level (67 MHz),
 * where a four-lane link at 1.5 Gbps per lane overflows and reports ECC and
 * CRC errors; at 400 MHz it runs clean. The WDMA needs the bus (INT) domain
 * at 400 MHz too, or it misses frame ends under load. Both are voted while
 * streaming and restored at stop. 0 leaves a domain alone (bring-up).
 */
static unsigned int cam_khz = 400000;
module_param(cam_khz, uint, 0644);
MODULE_PARM_DESC(cam_khz, "CAM DVFS rate in kHz while streaming (0: leave it)");

/*
 * The main camera's C-PHY link (three trios, about 1.6 Gsps) overflows its
 * image FIFO at a CAM vote of 400 MHz; 533 MHz runs it clean (measured on
 * this phone model by another bring-up). C-PHY links vote this instead.
 */
static unsigned int cam_khz_cphy = 533000;
module_param(cam_khz_cphy, uint, 0644);
MODULE_PARM_DESC(cam_khz_cphy, "CAM DVFS rate in kHz while a C-PHY link streams (0: leave it)");

static unsigned int int_khz = 400000;
module_param(int_khz, uint, 0644);
MODULE_PARM_DESC(int_khz, "INT (bus) DVFS rate in kHz while the WDMA runs (0: leave it)");

/*
 * Bring-up, for the full-resolution LASTADDR errors (CAMERA-PLAN.md
 * CAM5.2c): the line stride alignment, and the channel control value at
 * start. Stock starts a channel with 0xf83 (bits 11:7 = 31, meaning not
 * known); 0x3 (enable and pointer update only) is the other value seen to
 * work. The 2016-pixel modes' 2560-byte lines are a multiple of 512, the
 * 4032-pixel modes' 5056-byte lines are not.
 */
static unsigned int stride_align = CSIS_STRIDE_ALIGN;
module_param(stride_align, uint, 0644);
MODULE_PARM_DESC(stride_align, "WDMA line stride alignment in bytes, a power of 2 (bring-up)");

static unsigned int wdma_ctrl = CSIS_WDMA_CTRL_ON;
module_param(wdma_ctrl, uint, 0644);
MODULE_PARM_DESC(wdma_ctrl, "WDMA channel control value at start (bring-up; bit 0 is forced)");

/*
 * Bring-up: the LRTE configuration register's value (0x600), 0 for the
 * driver's choice (stock's 0x7fff, or the delimiter enable with both spacer
 * fields at their reset value for google,lrte-packet-delimiters). The
 * register resets to 0x7fff7fff; writing 0x7fff clears the upper spacer
 * field, which may matter to a C-PHY sensor [I: the reset value isn't
 * read on this phone yet].
 */
static unsigned int lrte_config;
module_param(lrte_config, uint, 0644);
MODULE_PARM_DESC(lrte_config, "LRTE configuration register value (bring-up; 0: the driver's)");

/*
 * Bring-up, for a sensor with a second stream (the Pixel 7 Pro's S5KGN1
 * sends RAW10 lines on VC 1 too): ch0_vc picks the VC channel 0 (and so
 * the WDMA) takes; ch1_vc, when not 15, gives channel 1 that VC with
 * channel 0's data type and size instead of parking it (its WDMA channel
 * stays off).
 */
static unsigned int ch0_vc;
module_param(ch0_vc, uint, 0644);
MODULE_PARM_DESC(ch0_vc, "Virtual channel of link channel 0 (bring-up; default 0)");

static unsigned int ch1_vc = CSIS_ISPCFG_VC_PARKED;
module_param(ch1_vc, uint, 0644);
MODULE_PARM_DESC(ch1_vc, "Virtual channel of link channel 1 (bring-up; 15: parked, no data type)");

/*
 * Bring-up: let the WDMA write up to this many lines past the frame
 * height, into buffers that much larger, to measure how many lines a
 * sensor really sends on the captured channel (a frame shorter than the
 * programmed height lands without an error; a longer one stops at the
 * last address).
 */
static unsigned int extra_lines;
module_param(extra_lines, uint, 0644);
MODULE_PARM_DESC(extra_lines, "WDMA lines and buffer space past the frame height (bring-up; default 0)");

/*
 * Bring-up: also take the WDMA's line-end interrupts (all four channels)
 * and count them, to see which channel the data of a stream arrives on.
 * About one interrupt a line: for short tests only.
 */
static bool wdma_line_irq;
module_param(wdma_line_irq, bool, 0644);
MODULE_PARM_DESC(wdma_line_irq, "Count WDMA line ends per channel (bring-up; many interrupts)");

/*
 * Bring-up: the link channels' pixel mode (13:12 of the channel config;
 * stock uses quad, 2), to try how a C-PHY link hands its lines to the
 * WDMA.
 */
/* zumapro bring-up: the CSIS core's memory format and the link's parallel mode */
static int zp_mem_fmt = -1;
module_param(zp_mem_fmt, int, 0644);
MODULE_PARM_DESC(zp_mem_fmt, "zumapro Bayer WDMA memory format (bring-up; default RAW10 packed 4, 2 RAW16)");

/*
 * zumapro: ABL leaves the front end's NOC clock at a sixth of SHARED3
 * (111 MHz); the main camera's C-PHY link (18.8 Gbps) then overruns the
 * front end (link ERR_DESKEW_OVER, short frames). While a link streams, the
 * INT DVFS domain (the clock's rail) goes up first, then the clock.
 */
static unsigned int fe_khz = 331776;
module_param(fe_khz, uint, 0644);
MODULE_PARM_DESC(fe_khz, "zumapro ISP-FE NOC clock in kHz while streaming (0: leave it)");

static unsigned int fe_int_khz = 533000;
module_param(fe_int_khz, uint, 0644);
MODULE_PARM_DESC(fe_int_khz, "zumapro INT DVFS rate in kHz while streaming, before the ISP-FE clock (0: leave it)");

static int cmn_ctrl_extra = -1;
module_param(cmn_ctrl_extra, int, 0644);
MODULE_PARM_DESC(cmn_ctrl_extra, "Extra link CMN_CTRL bits (bring-up; default the SoC's)");

static int zp_dt = -1;
module_param(zp_dt, int, 0644);
MODULE_PARM_DESC(zp_dt, "zumapro LoCH data type code (bring-up; default from the CSI-2 data type)");

static int parallel_mode = -1;
module_param(parallel_mode, int, 0644);
MODULE_PARM_DESC(parallel_mode, "Link channel parallel mode, ISP_CONFIG bits 15:14 (bring-up; default left 0)");

static int pixel_mode = -1;
module_param(pixel_mode, int, 0644);
MODULE_PARM_DESC(pixel_mode, "Link channel pixel mode, 0-3 (bring-up; default the SoC's: gs201 2, quad; zumapro 3)");

/*
 * Bring-up: start the WDMA channel only at the link's Nth frame end after
 * the stream starts (0: at once, before the sensor streams, as before), so
 * its first frame start is a whole frame's. The main camera's S5KGN1 got
 * a frame start, a few line ends and then LASTADDR in its first frame, and
 * the channel took almost no frame start after that.
 */
static unsigned int wdma_late;
module_param(wdma_late, uint, 0644);
MODULE_PARM_DESC(wdma_late, "Enable the WDMA channel at the link's Nth frame end (bring-up; 0: at start)");

/*
 * Bring-up: after a WDMA error (LASTADDR, LASTDATA, OVERLAP), switch the
 * channel off at once, fail the buffer it was writing and switch it on
 * again at the next link frame end, instead of leaving it to run on. A
 * channel that hit LASTADDR took almost no frame start afterwards, so one
 * error ended the stream; with this, each frame is a fresh try and the
 * run tells how many frames land.
 */
static bool wdma_recover;
module_param(wdma_recover, bool, 0644);
MODULE_PARM_DESC(wdma_recover, "Restart the WDMA channel at the next link frame end after an error (bring-up)");

/*
 * Bring-up, to match the order and values of a set-up known to capture
 * this phone's main camera: wdma_after_link programs the DMA block and
 * the channel after the link is set up and enabled, just before the
 * sensor streams (default: before the link); data_ctrl_clear clears bit 0
 * of the context's data control first (its value is logged either way);
 * qurgent=0 leaves the DMA common QURGENT register as found.
 */
static bool wdma_after_link;
module_param(wdma_after_link, bool, 0644);
MODULE_PARM_DESC(wdma_after_link, "Program the WDMA after the link starts, before the sensor (bring-up)");

static bool data_ctrl_clear;
module_param(data_ctrl_clear, bool, 0644);
MODULE_PARM_DESC(data_ctrl_clear, "Clear bit 0 of the WDMA context's data control at start (bring-up)");

static bool qurgent = true;
module_param(qurgent, bool, 0644);
MODULE_PARM_DESC(qurgent, "Write the DMA common QURGENT enable at start (bring-up; default on)");

/*
 * Bring-up: keep the bits of the common control and the channel
 * configurations that this driver doesn't name as the link's soft reset
 * left them (read-modify-write, as the set-up known to capture the main
 * camera does) instead of writing them as 0. The values after the reset
 * are logged either way.
 */
static bool link_rmw;
module_param(link_rmw, bool, 0644);
MODULE_PARM_DESC(link_rmw, "Keep unnamed link control bits as the soft reset left them (bring-up)");

/*
 * The link's INT0 (error) interrupt mask. ERR_WRONG_CONFIG is left out:
 * it flags a packet no channel takes, which the sensors send by design
 * (an embedded-data line a frame on the Pixel 9a's front and ultrawide
 * cameras, about 500 PDAF packets a frame on the main one, some 16,000
 * interrupts a second at 30 fps). Bring-up: CSIS_INT0_ERRORS takes all.
 */
static unsigned int int0_mask = CSIS_INT0_ERRORS & ~CSIS_INT0_ERR_WRONG_CONFIG;
module_param(int0_mask, uint, 0644);
MODULE_PARM_DESC(int0_mask, "Link INT0 (error) interrupt mask (bring-up; default all errors but WRONG_CONFIG)");

/*
 * Bring-up: hand buffers that a WDMA error ended (wdma_recover) or that
 * missed their frame end to userspace as done, not as errors, so what
 * the WDMA wrote before the error can be looked at.
 */
static bool keep_failed;
module_param(keep_failed, bool, 0644);
MODULE_PARM_DESC(keep_failed, "Return failed buffers as done, to look at their data (bring-up)");

/*
 * Bring-up: the value written to the CSIS sysreg's SC_CON0-2 ("PDP mux",
 * 3 bits each; at 0 the WDMA gets syncs but no pixel data).
 */
static unsigned int sc_con = CSIS_SYSREG_SC_CON_PASS;
module_param(sc_con, uint, 0644);
MODULE_PARM_DESC(sc_con, "CSIS sysreg SC_CON0-2 value, 1-7 (bring-up; default 1)");

/*
 * Bring-up: program all 32 address slots with the buffer's address and
 * mark them all valid, instead of slot 0 only. If a second frame start
 * (another virtual channel's) moves the channel to its next slot in the
 * middle of a frame, that slot then points at the same buffer instead of
 * at nothing.
 */
static bool all_slots;
module_param(all_slots, bool, 0644);
MODULE_PARM_DESC(all_slots, "Write the buffer address to all 32 WDMA slots (bring-up)");

/*
 * Bring-up: bits to set in CSIS sysreg 0x488 at stream on (stock sets
 * bit link + 8 x PDP channel there; on gs101 these were per-link data
 * enables). Restored at stop.
 */
static unsigned int link_sel_bits;
module_param(link_sel_bits, uint, 0644);
MODULE_PARM_DESC(link_sel_bits, "Bits to set in CSIS sysreg 0x488 while streaming (bring-up)");

/*
 * Bring-up: program the WDMA for only this many rows (0: the format's),
 * to see whether a short frame lands where a whole one doesn't.
 */
static unsigned int wdma_rows;
module_param(wdma_rows, uint, 0644);
MODULE_PARM_DESC(wdma_rows, "WDMA frame height in rows, if smaller than the format's (bring-up)");

/*
 * Bring-up: the WDMA channel's format code instead of the pixel format's
 * (e.g. 6, ten bits in 16; size the buffer with extra_lines, as the
 * stride stays the packed one). 0: the pixel format's.
 */
static unsigned int wdma_fmt;
module_param(wdma_fmt, uint, 0644);
MODULE_PARM_DESC(wdma_fmt, "WDMA format code override (bring-up; 0: the pixel format's)");

/*
 * Bring-up: the WDMA context's bandwidth limit registers (0x560-0x568,
 * "dma_bw_limit_0-2" in the stock HAL's register table). The C-PHY cameras
 * land frames only while each line takes long enough that the WDMA writes
 * at most about 400 MB/s; their values are logged at start, and
 * bw_limitN=V writes V (default: left as found).
 */
static int bw_limit0 = -1, bw_limit1 = -1, bw_limit2 = -1;
module_param(bw_limit0, int, 0644);
MODULE_PARM_DESC(bw_limit0, "WDMA context bandwidth limit register 0 value (bring-up; -1: leave)");
module_param(bw_limit1, int, 0644);
MODULE_PARM_DESC(bw_limit1, "WDMA context bandwidth limit register 1 value (bring-up; -1: leave)");
module_param(bw_limit2, int, 0644);
MODULE_PARM_DESC(bw_limit2, "WDMA context bandwidth limit register 2 value (bring-up; -1: leave)");

/*
 * Bring-up: write the DMA common page's per-link configuration word at
 * 0x48 (named for link 1 in the stock HAL's register table; stock writes
 * 0x100) at start. 0 leaves it as found, and the value found is logged.
 */
static bool cmn_cfg_csis1 = true;
module_param(cmn_cfg_csis1, bool, 0644);
MODULE_PARM_DESC(cmn_cfg_csis1, "Write 0x100 to the DMA common register 0x48 at start (bring-up; default on)");

static bool cached_buffers = true;
module_param(cached_buffers, bool, 0644);
MODULE_PARM_DESC(cached_buffers, "Allocate capture buffers cached (non-coherent, synced at dequeue)");

/* The stock RAW path writes WDMA channel 2; which one we need is a CAM4 test */
static unsigned int wdma_channel;
module_param(wdma_channel, uint, 0444);
MODULE_PARM_DESC(wdma_channel, "CSIS WDMA channel within the context, 0-3 (bring-up)");

/*
 * The instance that streams: the WDMA context, its interrupt line, the
 * DMA block's common page and the CSIS sysreg routing are shared by every
 * link, so one at a time.
 */
static DEFINE_MUTEX(gs201_csis_owner_lock);
static struct gs201_csis *gs201_csis_owner;

static int gs201_csis_claim(struct gs201_csis *csis)
{
	int ret = 0;

	mutex_lock(&gs201_csis_owner_lock);
	if (gs201_csis_owner && gs201_csis_owner != csis) {
		dev_info(csis->dev, "link %u busy: link %u is streaming\n",
			 csis->link_id, gs201_csis_owner->link_id);
		ret = -EBUSY;
	} else {
		WRITE_ONCE(gs201_csis_owner, csis);
	}
	mutex_unlock(&gs201_csis_owner_lock);

	return ret;
}

static void gs201_csis_release(struct gs201_csis *csis)
{
	mutex_lock(&gs201_csis_owner_lock);
	if (gs201_csis_owner == csis)
		WRITE_ONCE(gs201_csis_owner, NULL);
	mutex_unlock(&gs201_csis_owner_lock);
}

static const struct gs201_csis_format *gs201_csis_find_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(gs201_csis_formats); i++)
		if (gs201_csis_formats[i].code == code)
			return &gs201_csis_formats[i];

	return NULL;
}

static const struct gs201_csis_format *gs201_csis_find_fourcc(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(gs201_csis_formats); i++)
		if (gs201_csis_formats[i].fourcc == fourcc)
			return &gs201_csis_formats[i];

	return NULL;
}

static inline u32 link_read(struct gs201_csis *csis, u32 reg)
{
	return readl(csis->link + reg);
}

static inline void link_write(struct gs201_csis *csis, u32 reg, u32 val)
{
	writel(val, csis->link + reg);
}

static inline u32 wdma_read(struct gs201_csis *csis, u32 reg)
{
	return readl(csis->wdma + reg);
}

static inline void wdma_write(struct gs201_csis *csis, u32 reg, u32 val)
{
	writel(val, csis->wdma + reg);
}

/* -----------------------------------------------------------------------------
 * Hardware: link (the stock camera software's order: debug option, interrupt
 * sources, channel config and size, line-rate default, shadow update, PHY
 * common control, then the common control with enable last)
 */

static void gs201_csis_link_start(struct gs201_csis *csis,
				  const struct v4l2_mbus_framefmt *mf,
				  const struct gs201_csis_format *f,
				  unsigned int lanes)
{
	u32 cmn_keep = 0, cfg_keep[4] = { };
	unsigned int ch;

	link_write(csis, CSIS_CMN_CTRL,
		   link_read(csis, CSIS_CMN_CTRL) | CSIS_CMN_CTRL_SW_RESET);
	usleep_range(1000, 1500);

	dev_info(csis->dev, "link %u after soft reset: CMN_CTRL 0x%08x ISP_CONFIG 0x%08x 0x%08x 0x%08x 0x%08x DBG 0x%08x\n",
		 csis->link_id, link_read(csis, CSIS_CMN_CTRL),
		 link_read(csis, CSIS_ISP_CONFIG_CH(0)), link_read(csis, CSIS_ISP_CONFIG_CH(1)),
		 link_read(csis, CSIS_ISP_CONFIG_CH(2)), link_read(csis, CSIS_ISP_CONFIG_CH(3)),
		 link_read(csis, CSIS_DBG_OPTION_SUITE));
	if (link_rmw) {
		cmn_keep = link_read(csis, CSIS_CMN_CTRL) &
			   ~(CSIS_CMN_CTRL_CSI_EN | CSIS_CMN_CTRL_SW_RESET |
			     CSIS_CMN_CTRL_LANE_NUMBER | CSIS_CMN_CTRL_INTERLEAVE |
			     CSIS_CMN_CTRL_F13 | CSIS_CMN_CTRL_CPHY);
		for (ch = 0; ch < 4; ch++)
			cfg_keep[ch] = link_read(csis, CSIS_ISP_CONFIG_CH(ch)) &
				       ~(CSIS_ISPCFG_DATAFORMAT | CSIS_ISPCFG_PIXEL_MODE |
					 CSIS_ISPCFG_VC);
	}

	if (csis->soc->dbg_bit23)
		link_write(csis, CSIS_DBG_OPTION_SUITE,
			   link_read(csis, CSIS_DBG_OPTION_SUITE) | CSIS_DBG_OPTION_BIT23);
	else
		link_write(csis, CSIS_DBG_OPTION_SUITE, 0);
	link_write(csis, CSIS_INT_SRC0, ~0u);
	link_write(csis, CSIS_INT_SRC1, ~0u);

	/*
	 * Channel 0 takes VC 0; stock parks channels 1-3 on VC 1-3 with no
	 * data type, but a sensor that sends a second stream on VC 1 then cuts
	 * channel 0's output to 8 bits a pixel, so they go to VC 15.
	 */
	for (ch = 0; ch < 4; ch++) {
		bool used = ch == 0 || (ch == 1 && ch1_vc != CSIS_ISPCFG_VC_PARKED);
		u32 dt = used ? f->data_type : CSIS_ISPCFG_DT_UNUSED;
		u32 vc = ch == 0 ? ch0_vc : ch == 1 ? ch1_vc : CSIS_ISPCFG_VC_PARKED;

		link_write(csis, CSIS_ISP_CONFIG_CH(ch), cfg_keep[ch] |
			   FIELD_PREP(CSIS_ISPCFG_DATAFORMAT, dt) |
			   FIELD_PREP(CSIS_ISPCFG_PIXEL_MODE,
				      (pixel_mode >= 0 ? pixel_mode : csis->soc->pixel_mode) & 3) |
			   (parallel_mode >= 0 ? FIELD_PREP(GENMASK(15, 14), parallel_mode & 3) : 0) |
			   FIELD_PREP(CSIS_ISPCFG_VC, vc));
		link_write(csis, CSIS_ISP_RESOL_CH(ch), !used ? 0 :
			   FIELD_PREP(CSIS_ISP_RESOL_V, mf->height) |
			   FIELD_PREP(CSIS_ISP_RESOL_H, mf->width));
	}

	/*
	 * A C-PHY sensor that ends its packets with LRTE delimiters needs
	 * them enabled here, with the spacer fields at their reset value;
	 * otherwise stock's value.
	 */
	link_write(csis, CSIS_LRTE_CONFIG, lrte_config ? lrte_config :
		   csis->lrte_epd ? CSIS_LRTE_EPD_EN | CSIS_LRTE_SPACERS_RESET :
		   CSIS_LRTE_DEFAULT);
	link_write(csis, CSIS_UPD_SDW, CSIS_UPD_SDW_ALL);
	if (csis->soc->phy_cmn_ctrl)
		link_write(csis, CSIS_PHY_CMN_CTRL, CSIS_PHY_CMN_CTRL_ENABLE);

	link_write(csis, CSIS_INT_MSK0, int0_mask & CSIS_INT0_ERRORS);
	link_write(csis, CSIS_INT_MSK1, CSIS_INT1_FRAME_START |
		   CSIS_INT1_FRAME_END | CSIS_INT1_ERRORS);

	link_write(csis, CSIS_CMN_CTRL, cmn_keep | CSIS_CMN_CTRL_CSI_EN |
		   FIELD_PREP(CSIS_CMN_CTRL_LANE_NUMBER, lanes - 1) |
		   FIELD_PREP(CSIS_CMN_CTRL_INTERLEAVE, 3) |
		   FIELD_PREP(CSIS_CMN_CTRL_F13, 2) |
		   (cmn_ctrl_extra >= 0 ? cmn_ctrl_extra : csis->soc->cmn_ctrl_extra) |
		   (csis->cphy ? CSIS_CMN_CTRL_CPHY : 0));
}

/* -----------------------------------------------------------------------------
 * Hardware: the zumapro CSIS core's image LoCH
 */

static inline u32 core_read(struct gs201_csis *csis, u32 reg)
{
	return readl(csis->core + reg);
}

static inline void core_write(struct gs201_csis *csis, u32 reg, u32 val)
{
	writel(val, csis->core + reg);
}

static inline u32 img_read(struct gs201_csis *csis, u32 reg)
{
	return core_read(csis, ZP_IMG(csis->loch, reg));
}

static inline void img_write(struct gs201_csis *csis, u32 reg, u32 val)
{
	core_write(csis, ZP_IMG(csis->loch, reg), val);
}

static u32 zp_core_dt(u8 data_type)
{
	switch (data_type) {
	case MIPI_CSI2_DT_RAW8:
		return ZP_DT_RAW8;
	case MIPI_CSI2_DT_RAW12:
		return ZP_DT_RAW12;
	case MIPI_CSI2_DT_RAW14:
		return ZP_DT_RAW14;
	default:
		return ZP_DT_RAW10;
	}
}

static void zp_core_set_addr(struct gs201_csis *csis, dma_addr_t addr)
{
	/* one batch slot; the configuration takes effect at the next frame */
	img_write(csis, ZP_IMG_BATCH_ADDR_LSB(0),
		  lower_32_bits(addr) & ZP_IMG_ADDR_LSB_MASK);
	img_write(csis, ZP_IMG_BATCH_ADDR_MSB(0),
		  upper_32_bits(addr) & ZP_IMG_ADDR_MSB_MASK);
	img_write(csis, ZP_IMG_WDMA_CONFIG_ENABLE, 1);
}

/* The LoCH takes the link's channel 0 (VC 0) data type, as the HAL routes it. */
static void zp_core_route(struct gs201_csis *csis)
{
	u32 val;

	/* a LoCH a previous stream left running or in error starts clean */
	img_write(csis, ZP_IMG_WDMA_CONFIG_ENABLE, 0);
	img_write(csis, ZP_IMG_WDMA_CONTEXT_ENABLE, 0);
	/*
	 * A stream that stopped cleanly leaves the LoCH bound to its link
	 * (EN_STATUS active), and a disabled LoCH ignores the abort, so the
	 * next stream on another link got no interrupt at all. Abort it
	 * enabled, with its DMA off.
	 */
	if (img_read(csis, ZP_IMG_LOCH_EN_STATUS) & ZP_IMG_LOCH_EN_STATUS_ACTIVE)
		core_write(csis, ZP_LOCH_EN_SET, BIT(csis->loch));
	core_write(csis, ZP_LOCH_ABORT, BIT(csis->loch));
	if (readl_poll_timeout(csis->core + ZP_IMG(csis->loch, ZP_IMG_ABORT_DONE_ISR),
			       val, val & ZP_IMG_ABORT_DONE, 100, 20000))
		dev_dbg(csis->dev, "LoCH %u: no abort-done at start\n", csis->loch);
	core_write(csis, ZP_LOCH_EN_CLR, BIT(csis->loch));
	img_write(csis, ZP_IMG_ABORT_DONE_ISR, val);
	img_write(csis, ZP_IMG_PROC_ISR, img_read(csis, ZP_IMG_PROC_ISR));
	img_write(csis, ZP_IMG_ERR_ISR, img_read(csis, ZP_IMG_ERR_ISR));
	core_write(csis, ZP_LOCH_INT_ISR, BIT(csis->loch));

	core_write(csis, ZP_FRAME_ID_CLR, BIT(csis->loch));
	img_write(csis, ZP_IMG_AUTO_REC_SYNC_INTV0, 0);
	img_write(csis, ZP_IMG_AUTO_REC_SYNC_INTV1, 0);
	img_write(csis, ZP_IMG_BATCH_INT_MSK_SOF, 1);
	img_write(csis, ZP_IMG_BATCH_INT_MSK_EOF, 1);
	img_write(csis, ZP_IMG_BATCH_INT_MSK_DMA_DONE, 1);
	img_write(csis, ZP_IMG_BATCH_INT_MSK_FRAME_DONE, 1);
	img_write(csis, ZP_IMG_CTL,
		  FIELD_PREP(ZP_IMG_CTL_LINK, csis->link_id) |
		  FIELD_PREP(ZP_IMG_CTL_LINK_CH, 0) |
		  FIELD_PREP(ZP_IMG_CTL_DT, zp_dt >= 0 ? zp_dt :
			     zp_core_dt(csis->fmt->data_type)));
	img_write(csis, ZP_IMG_BATCH_CTL, 0);
	img_write(csis, ZP_IMG_ISP_RESOL,
		  FIELD_PREP(ZP_IMG_ISP_RESOL_H, csis->pix.height) |
		  FIELD_PREP(ZP_IMG_ISP_RESOL_W, csis->pix.width));
	img_write(csis, ZP_IMG_PROC_IER, ZP_IMG_PROC_SOF | ZP_IMG_PROC_EOF |
		  ZP_IMG_PROC_SHADOW_UPDATE | ZP_IMG_PROC_DMA_DONE);
}

static void zp_core_unroute(struct gs201_csis *csis)
{
	core_write(csis, ZP_LOCH_EN_CLR, BIT(csis->loch));
	img_write(csis, ZP_IMG_CTL, 0);
}

static void zp_core_wdma_start(struct gs201_csis *csis)
{
	u32 fmt = zp_mem_fmt >= 0 ? zp_mem_fmt :
		  csis->fmt->bpp == 8 ? ZP_MEM_FMT_8BIT : ZP_MEM_FMT_RAW10_PACKED;

	img_write(csis, ZP_IMG_BAYER_MEM_FORMAT, fmt);
	img_write(csis, ZP_IMG_BAYER_STRIDE, csis->pix.bytesperline);
	img_write(csis, ZP_IMG_BAYER_CROP_EN, 0);
	img_write(csis, ZP_IMG_BAYER_ENABLE, BIT(0));
	img_write(csis, ZP_IMG_SQ_CONFIG_ID_LSB, 0);
	img_write(csis, ZP_IMG_SQ_CONFIG_ID_MSB, 0);
	img_write(csis, ZP_IMG_SQ_BASE_ADDR_LSB, lower_32_bits(csis->sq_dma));
	img_write(csis, ZP_IMG_SQ_BASE_ADDR_MSB, upper_32_bits(csis->sq_dma));

	img_write(csis, ZP_IMG_PROC_ISR, img_read(csis, ZP_IMG_PROC_ISR));
	img_write(csis, ZP_IMG_ERR_ISR, img_read(csis, ZP_IMG_ERR_ISR));
	img_write(csis, ZP_IMG_WDMA_CONFIG_MISS, 1);
	img_write(csis, ZP_IMG_PROC_IMR, ZP_IMG_PROC_SOF | ZP_IMG_PROC_EOF |
		  ZP_IMG_PROC_DMA_DONE | ZP_IMG_PROC_FRAME_DONE);
	img_write(csis, ZP_IMG_ERR_IMR, ZP_IMG_ERRORS);
	core_write(csis, ZP_LOCH_INT_ISR, BIT(csis->loch));
	core_write(csis, ZP_LOCH_INT_IMR,
		   core_read(csis, ZP_LOCH_INT_IMR) | BIT(csis->loch));

	img_write(csis, ZP_IMG_WDMA_CONFIG_ENABLE, 1);
	img_write(csis, ZP_IMG_WDMA_CONTEXT_ENABLE, 1);
	core_write(csis, ZP_LOCH_EN_SET, BIT(csis->loch));

	dev_info(csis->dev, "LoCH %u on: link %u, CTL 0x%08x RESOL 0x%08x, %u bytes a line, format %u, slot 0 0x%x:%08x; LOCH_EN 0x%08x\n",
		 csis->loch, csis->link_id, img_read(csis, ZP_IMG_CTL),
		 img_read(csis, ZP_IMG_ISP_RESOL), csis->pix.bytesperline, fmt,
		 img_read(csis, ZP_IMG_BATCH_ADDR_MSB(0)),
		 img_read(csis, ZP_IMG_BATCH_ADDR_LSB(0)),
		 core_read(csis, ZP_LOCH_EN));
}

static void zp_core_wdma_stop(struct gs201_csis *csis)
{
	u32 val;

	img_write(csis, ZP_IMG_WDMA_CONFIG_ENABLE, 0);
	img_write(csis, ZP_IMG_WDMA_CONTEXT_ENABLE, 0);
	core_write(csis, ZP_LOCH_EN_CLR, BIT(csis->loch));
	core_write(csis, ZP_LOCH_ABORT, BIT(csis->loch));
	if (readl_poll_timeout(csis->core + ZP_IMG(csis->loch, ZP_IMG_ABORT_DONE_ISR),
			       val, val & ZP_IMG_ABORT_DONE, 100, 50000))
		dev_dbg(csis->dev, "LoCH %u: no abort-done\n", csis->loch);
	img_write(csis, ZP_IMG_ABORT_DONE_ISR, val);
	core_write(csis, ZP_LOCH_INT_IMR,
		   core_read(csis, ZP_LOCH_INT_IMR) & ~BIT(csis->loch));
	img_write(csis, ZP_IMG_PROC_IMR, 0);
	img_write(csis, ZP_IMG_ERR_IMR, 0);
	img_write(csis, ZP_IMG_PROC_ISR, img_read(csis, ZP_IMG_PROC_ISR));
	img_write(csis, ZP_IMG_ERR_ISR, img_read(csis, ZP_IMG_ERR_ISR));
	core_write(csis, ZP_LOCH_INT_ISR, BIT(csis->loch));
}

static void gs201_csis_link_stop(struct gs201_csis *csis)
{
	link_write(csis, CSIS_INT_MSK0, 0);
	link_write(csis, CSIS_INT_MSK1, 0);
	link_write(csis, CSIS_CMN_CTRL,
		   link_read(csis, CSIS_CMN_CTRL) & ~CSIS_CMN_CTRL_CSI_EN);
	if (csis->soc->phy_cmn_ctrl)
		link_write(csis, CSIS_PHY_CMN_CTRL, 0);
}

/* -----------------------------------------------------------------------------
 * Hardware: WDMA
 */

static inline void wdma_ch_write(struct gs201_csis *csis, u32 reg, u32 val)
{
	wdma_write(csis, CSIS_WDMA_CH(csis->wdma_ch, reg), val);
}

static void gs201_csis_wdma_set_addr(struct gs201_csis *csis, dma_addr_t addr)
{
	unsigned int n;

	/* One slot, rewritten each frame start for the frame after [I] */
	wdma_ch_write(csis, CSIS_WDMA_ADDR(1), lower_32_bits(addr));
	if (all_slots)
		for (n = 2; n <= 32; n++)
			wdma_ch_write(csis, CSIS_WDMA_ADDR(n), lower_32_bits(addr));
}

/*
 * link -> WDMA slot -> CSIS WDMA context, with the EBUF bypassed. The
 * registers are shared by all links; their previous values come back at
 * stop.
 */
static int gs201_csis_route(struct gs201_csis *csis)
{
	unsigned int i;
	int ret;

	for (i = 0; i < CSIS_SYSREG_SC_CON_NUM; i++) {
		ret = regmap_read(csis->sysreg, CSIS_SYSREG_SC_CON(i),
				  &csis->sc_con_saved[i]);
		if (ret)
			return ret;
	}
	ret = regmap_read(csis->sysreg, CSIS_SYSREG_EBUF_IN(csis->ebuf),
			  &csis->ebuf_in_saved);
	if (ret)
		return ret;

	for (i = 0; i < CSIS_SYSREG_SC_CON_NUM; i++) {
		ret = regmap_write(csis->sysreg, CSIS_SYSREG_SC_CON(i),
				   sc_con & 7 ?: CSIS_SYSREG_SC_CON_PASS);
		if (ret)
			goto err_restore;
	}
	ret = regmap_write(csis->sysreg, CSIS_SYSREG_EBUF_IN(csis->ebuf),
			   csis->link_id);
	if (ret)
		goto err_restore;

	if (csis->ebuf_regs) {
		csis->ebuf_ctrl_saved = readl(csis->ebuf_regs + CSIS_EBUF_CTRL);
		writel(csis->ebuf_ctrl_saved | CSIS_EBUF_CTRL_BYPASS,
		       csis->ebuf_regs + CSIS_EBUF_CTRL);
	}

	return 0;

err_restore:
	for (i = 0; i < CSIS_SYSREG_SC_CON_NUM; i++)
		regmap_write(csis->sysreg, CSIS_SYSREG_SC_CON(i),
			     csis->sc_con_saved[i]);
	return ret;
}

static void gs201_csis_unroute(struct gs201_csis *csis)
{
	unsigned int i;

	if (csis->ebuf_regs)
		writel(csis->ebuf_ctrl_saved, csis->ebuf_regs + CSIS_EBUF_CTRL);
	regmap_write(csis->sysreg, CSIS_SYSREG_EBUF_IN(csis->ebuf),
		     csis->ebuf_in_saved);
	for (i = 0; i < CSIS_SYSREG_SC_CON_NUM; i++)
		regmap_write(csis->sysreg, CSIS_SYSREG_SC_CON(i),
			     csis->sc_con_saved[i]);
}

static void gs201_csis_wdma_start(struct gs201_csis *csis)
{
	u32 val;

	writel(CSIS_DMA_CMN_CLK_CTRL_VAL, csis->dma + CSIS_DMA_CMN_CLK_CTRL);
	writel(CSIS_DMA_CMN_CTRL_SW_RESET, csis->dma + CSIS_DMA_CMN_CTRL);
	if (readl_poll_timeout(csis->dma + CSIS_DMA_CMN_CTRL, val,
			       !(val & CSIS_DMA_CMN_CTRL_SW_RESET), 10, 1000))
		dev_warn(csis->dev, "DMA block: reset didn't complete\n");
	writel(CSIS_DMA_CMN_CTRL_VAL, csis->dma + CSIS_DMA_CMN_CTRL);
	writel(1, csis->dma + CSIS_DMA_CMN_DEBUG_EN);
	val = readl(csis->dma + CSIS_DMA_CMN_CFG_CSIS1);
	if (cmn_cfg_csis1)
		writel(CSIS_DMA_CMN_CFG_CSIS1_VAL, csis->dma + CSIS_DMA_CMN_CFG_CSIS1);
	dev_info(csis->dev, "DMA common 0x48 was 0x%08x, now 0x%08x; WDMA %s\n",
		 val, readl(csis->dma + CSIS_DMA_CMN_CFG_CSIS1),
		 wdma_late ? "starts at a link frame end" : "starts now");
	if (qurgent)
		writel(CSIS_DMA_CMN_QURGENT_EN_VAL, csis->dma + CSIS_DMA_CMN_QURGENT_EN);
	val = wdma_read(csis, CSIS_WDMA_DATA_CTRL);
	if (data_ctrl_clear)
		wdma_write(csis, CSIS_WDMA_DATA_CTRL, val & ~BIT(0));
	dev_info(csis->dev, "WDMA data control was 0x%08x, now 0x%08x; QURGENT 0x%08x\n",
		 val, wdma_read(csis, CSIS_WDMA_DATA_CTRL),
		 readl(csis->dma + CSIS_DMA_CMN_QURGENT_EN));

	dev_info(csis->dev, "WDMA bandwidth limits 0x%08x 0x%08x 0x%08x\n",
		 wdma_read(csis, CSIS_WDMA_BW_LIMIT(0)), wdma_read(csis, CSIS_WDMA_BW_LIMIT(1)),
		 wdma_read(csis, CSIS_WDMA_BW_LIMIT(2)));
	if (bw_limit0 >= 0)
		wdma_write(csis, CSIS_WDMA_BW_LIMIT(0), bw_limit0);
	if (bw_limit1 >= 0)
		wdma_write(csis, CSIS_WDMA_BW_LIMIT(1), bw_limit1);
	if (bw_limit2 >= 0)
		wdma_write(csis, CSIS_WDMA_BW_LIMIT(2), bw_limit2);

	wdma_ch_write(csis, CSIS_WDMA_FMT,
		      FIELD_PREP(CSIS_WDMA_FMT_CODE, wdma_fmt ?: csis->fmt->wdma_fmt));
	wdma_ch_write(csis, CSIS_WDMA_SKIP, 0);
	/* the buffer was sized with the extra lines when the format was set */
	wdma_ch_write(csis, CSIS_WDMA_RESOL,
		      FIELD_PREP(CSIS_WDMA_RESOL_V, wdma_rows &&
				 wdma_rows < csis->pix.sizeimage / csis->pix.bytesperline ?
				 wdma_rows : csis->pix.sizeimage / csis->pix.bytesperline) |
		      FIELD_PREP(CSIS_WDMA_RESOL_H, csis->pix.width));
	wdma_ch_write(csis, CSIS_WDMA_STRIDE, csis->pix.bytesperline);
	wdma_ch_write(csis, CSIS_WDMA_FCNT_SEQ, all_slots ? ~0u : BIT(0));
	wdma_write(csis, CSIS_WDMA_SBWC_CTRL, 0);

	wdma_write(csis, CSIS_WDMA_INT_SRC, ~0u);
	/*
	 * Frame start and end of all four channels (only this one's drive
	 * the buffers; the others are counted), line ends when asked.
	 */
	wdma_write(csis, CSIS_WDMA_INT_MSK,
		   GENMASK(11, 4) | CSIS_WDMA_INT_ERRORS |
		   (wdma_line_irq ? GENMASK(3, 0) : 0));

	if (wdma_late)
		WRITE_ONCE(csis->wdma_arm, wdma_late);
	else
		wdma_ch_write(csis, CSIS_WDMA_CTRL, wdma_ctrl | CSIS_WDMA_CTRL_EN);
}

static void gs201_csis_wdma_stop(struct gs201_csis *csis)
{
	u32 val;

	wdma_ch_write(csis, CSIS_WDMA_CTRL, CSIS_WDMA_CTRL_OFF);
	if (readl_poll_timeout(csis->wdma + CSIS_WDMA_INT_SRC, val,
			       val & CSIS_WDMA_INT_ABORT_DONE, 100, 50000))
		dev_dbg(csis->dev, "WDMA %u: no abort-done\n", csis->ctx_id);
	wdma_write(csis, CSIS_WDMA_INT_MSK, 0);
	wdma_write(csis, CSIS_WDMA_INT_SRC, ~0u);

	/* The stock reset/flush sequence */
	wdma_write(csis, CSIS_WDMA_DATA_CTRL, 0);
	wdma_ch_write(csis, CSIS_WDMA_CTRL, CSIS_WDMA_CTRL_EN);
	wdma_ch_write(csis, CSIS_WDMA_FMT, CSIS_WDMA_FMT_RAW10);
	wdma_ch_write(csis, CSIS_WDMA_CTRL, 0);
}

/* -----------------------------------------------------------------------------
 * Interrupts
 */

static irqreturn_t gs201_csis_link_irq(int irq, void *data)
{
	struct gs201_csis *csis = data;
	unsigned long fs = 0, fe = 0;
	unsigned int bit;
	u32 src0, src1;

	src0 = link_read(csis, CSIS_INT_SRC0);
	src1 = link_read(csis, CSIS_INT_SRC1);
	/*
	 * INT1's frame start and end bits summarise the per-VC FS/FE
	 * sources: clear those too, or the summary stays set and the
	 * interrupt fires again at once (about 170000 times a second while
	 * streaming 60 fps).
	 */
	if (src1 & CSIS_INT1_FRAME_START) {
		fs = link_read(csis, CSIS_INT_SRC_FS);
		link_write(csis, CSIS_INT_SRC_FS, fs);
	}
	if (src1 & CSIS_INT1_FRAME_END) {
		fe = link_read(csis, CSIS_INT_SRC_FE);
		link_write(csis, CSIS_INT_SRC_FE, fe);
	}
	link_write(csis, CSIS_INT_SRC0, src0);
	link_write(csis, CSIS_INT_SRC1, src1);

	if (src1 & CSIS_INT1_FRAME_START) {
		struct v4l2_event ev = {
			.type = V4L2_EVENT_FRAME_SYNC,
			.u.frame_sync.frame_sequence = csis->stats.frame_start,
		};

		v4l2_event_queue(csis->sd.devnode, &ev);
	}

	spin_lock(&csis->slock);
	csis->stats.link_irqs++;
	for_each_set_bit(bit, &fs, 32)
		csis->stats.link_fs[bit]++;
	for_each_set_bit(bit, &fe, 32)
		csis->stats.link_fe[bit]++;
	if (src1 & CSIS_INT1_FRAME_START)
		csis->stats.frame_start++;
	if (src1 & CSIS_INT1_FRAME_END) {
		csis->stats.frame_end++;
		if (csis->wdma_arm && !--csis->wdma_arm) {
			wdma_ch_write(csis, CSIS_WDMA_CTRL, wdma_ctrl | CSIS_WDMA_CTRL_EN);
			csis->stats.wdma_started_at = csis->stats.frame_end;
		}
	}
	if ((src0 & CSIS_INT0_ERRORS) || (src1 & CSIS_INT1_ERRORS)) {
		unsigned long b0 = src0 & CSIS_INT0_ERRORS;
		unsigned long b1 = src1 & CSIS_INT1_ERRORS;

		csis->stats.link_errors++;
		csis->stats.last_link_err = src0 | (src1 & CSIS_INT1_ERRORS) << 24;
		for_each_set_bit(bit, &b0, 32)
			csis->stats.int0_bits[bit]++;
		for_each_set_bit(bit, &b1, 8)
			csis->stats.int1_bits[bit]++;
	}
	spin_unlock(&csis->slock);

	if (src0 & CSIS_INT0_ERRORS)
		dev_dbg_ratelimited(csis->dev, "link %u error 0x%08x\n",
				    csis->link_id, src0);

	return (src0 | src1) ? IRQ_HANDLED : IRQ_NONE;
}

static void gs201_csis_next_buffer(struct gs201_csis *csis)
{
	csis->next = list_first_entry_or_null(&csis->pending,
					      struct gs201_csis_buffer, list);
	if (csis->next) {
		list_del(&csis->next->list);
		if (csis->dma_on && csis->core)
			zp_core_set_addr(csis, csis->next->addr);
		else if (csis->dma_on)
			gs201_csis_wdma_set_addr(csis, csis->next->addr);
	}
}

static irqreturn_t gs201_csis_wdma_irq(int irq, void *data)
{
	struct gs201_csis *csis = data;
	struct gs201_csis_buffer *done = NULL, *failed = NULL;
	unsigned int ch;
	u32 src;

	/* The line and the context are shared: only the streaming link reads */
	if (READ_ONCE(gs201_csis_owner) != csis)
		return IRQ_NONE;

	src = wdma_read(csis, CSIS_WDMA_INT_SRC);
	wdma_write(csis, CSIS_WDMA_INT_SRC, src);
	if (!src)
		return IRQ_NONE;

	spin_lock(&csis->slock);

	if (src & CSIS_WDMA_INT_FS(csis->wdma_ch))
		csis->stats.wdma_lines = 0;
	csis->stats.wdma_lines += !!(src & CSIS_WDMA_INT_LINE_END(csis->wdma_ch));
	for (ch = 0; ch < 4; ch++) {
		csis->stats.wdma_fs[ch] += !!(src & CSIS_WDMA_INT_FS(ch));
		csis->stats.wdma_fe[ch] += !!(src & CSIS_WDMA_INT_FE(ch));
		csis->stats.wdma_le[ch] += !!(src & CSIS_WDMA_INT_LINE_END(ch));
	}

	if (src & CSIS_WDMA_INT_ERRORS) {
		unsigned long be = src & CSIS_WDMA_INT_ERRORS;
		unsigned int bit;

		csis->stats.wdma_errors++;
		csis->stats.last_wdma_err = src;
		for_each_set_bit(bit, &be, 32)
			csis->stats.wdma_bits[bit]++;
		if (!csis->wdma_err_logged) {
			u32 ch = csis->wdma_ch;

			csis->wdma_err_logged = true;
			dev_warn(csis->dev, "WDMA error 0x%08x: CTRL 0x%08x FMT 0x%08x ADDR1 0x%08x RESOL 0x%08x STRIDE 0x%08x, %ux%u, %u bytes a line, %u a frame; link last packet 0x%08x, FRM_CNT 0x%08x, LRTE 0x%08x\n",
				 src,
				 wdma_read(csis, CSIS_WDMA_CH(ch, CSIS_WDMA_CTRL)),
				 wdma_read(csis, CSIS_WDMA_CH(ch, CSIS_WDMA_FMT)),
				 wdma_read(csis, CSIS_WDMA_CH(ch, CSIS_WDMA_ADDR(1))),
				 wdma_read(csis, CSIS_WDMA_CH(ch, CSIS_WDMA_RESOL)),
				 wdma_read(csis, CSIS_WDMA_CH(ch, CSIS_WDMA_STRIDE)),
				 csis->pix.width, csis->pix.height,
				 csis->pix.bytesperline, csis->pix.sizeimage,
				 link_read(csis, CSIS_DBG_LAST_PKT),
				 link_read(csis, CSIS_FRM_CNT_CH(0)),
				 link_read(csis, CSIS_LRTE_CONFIG));
			dev_warn(csis->dev, "  WDMA debug: frames 0x%08x, lines/data 0x%08x\n",
				 readl(csis->dma + CSIS_DMA_CMN_DBG_VCNT(csis->ctx_id)),
				 readl(csis->dma + CSIS_DMA_CMN_DBG_HDCNT(csis->ctx_id)));
			dev_warn(csis->dev, "  so far: WDMA FS %u/%u/%u/%u FE %u/%u/%u/%u lines %u/%u/%u/%u (ch 0-3); link FS VC0 %u VC1 %u, FE VC0 %u VC1 %u\n",
				 csis->stats.wdma_fs[0], csis->stats.wdma_fs[1],
				 csis->stats.wdma_fs[2], csis->stats.wdma_fs[3],
				 csis->stats.wdma_fe[0], csis->stats.wdma_fe[1],
				 csis->stats.wdma_fe[2], csis->stats.wdma_fe[3],
				 csis->stats.wdma_le[0], csis->stats.wdma_le[1],
				 csis->stats.wdma_le[2], csis->stats.wdma_le[3],
				 csis->stats.link_fs[0], csis->stats.link_fs[1],
				 csis->stats.link_fe[0], csis->stats.link_fe[1]);
		}
	}

	/*
	 * Frame start latched the address of the buffer being written: it
	 * becomes active, and the next one is programmed for the frame after.
	 * An active buffer still there missed its frame end: give it back as
	 * failed rather than lose it.
	 */
	if (src & CSIS_WDMA_INT_FS(csis->wdma_ch)) {
		if (csis->active && csis->next) {
			failed = csis->active;
			csis->active = NULL;
			csis->stats.missed_fe++;
		}
		if (csis->next) {
			csis->active = csis->next;
			csis->next = NULL;
			gs201_csis_next_buffer(csis);
		} else {
			csis->stats.dropped++;
		}
	}

	if ((src & CSIS_WDMA_INT_FE(csis->wdma_ch)) && csis->active) {
		done = csis->active;
		csis->active = NULL;
		csis->stats.wdma_done++;
	}

	if (wdma_recover && (src & CSIS_WDMA_INT_ERRORS) && !csis->wdma_arm) {
		wdma_ch_write(csis, CSIS_WDMA_CTRL, wdma_ctrl & ~CSIS_WDMA_CTRL_EN);
		if (!failed && !done && csis->active) {
			failed = csis->active;
			csis->active = NULL;
		}
		csis->wdma_arm = 1;
		csis->stats.wdma_restarts++;
		if (csis->stats.wdma_restarts <= 8)
			dev_info(csis->dev, "WDMA error 0x%08x at link frame %u, %u line ends into the frame: restart at the next frame end\n",
				 src, csis->stats.frame_start, csis->stats.wdma_lines);
	}

	spin_unlock(&csis->slock);

	if (failed) {
		failed->vb.vb2_buf.timestamp = ktime_get_ns();
		failed->vb.sequence = csis->sequence++;
		failed->vb.field = V4L2_FIELD_NONE;
		vb2_buffer_done(&failed->vb.vb2_buf, keep_failed ?
				VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR);
	}

	if (done) {
		done->vb.vb2_buf.timestamp = ktime_get_ns();
		done->vb.sequence = csis->sequence++;
		done->vb.field = V4L2_FIELD_NONE;
		vb2_buffer_done(&done->vb.vb2_buf,
				(src & CSIS_WDMA_INT_ERRORS) && !keep_failed ?
				VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	}

	return IRQ_HANDLED;
}

/*
 * zumapro: the CSIS core's interrupt is shared by every LoCH. Start of frame
 * latched the slot address: that buffer becomes active and the next is
 * programmed; DMA done completes it.
 */
static irqreturn_t zp_core_irq(int irq, void *data)
{
	struct gs201_csis *csis = data;
	struct gs201_csis_buffer *done = NULL, *failed = NULL;
	u32 agg, proc, err;

	if (READ_ONCE(gs201_csis_owner) != csis)
		return IRQ_NONE;

	agg = core_read(csis, ZP_LOCH_INT_ISR);
	csis->stats.core_last_agg = agg;
	if (!(agg & BIT(csis->loch)))
		return IRQ_NONE;
	proc = img_read(csis, ZP_IMG_PROC_ISR);
	csis->stats.core_irqs++;
	csis->stats.core_last_proc = proc;
	err = img_read(csis, ZP_IMG_ERR_ISR) & ZP_IMG_ERRORS;
	img_write(csis, ZP_IMG_PROC_ISR, proc);
	if (err)
		img_write(csis, ZP_IMG_ERR_ISR, err);
	core_write(csis, ZP_LOCH_INT_ISR, BIT(csis->loch));

	spin_lock(&csis->slock);
	csis->stats.wdma_fs[0] += !!(proc & ZP_IMG_PROC_SOF);
	csis->stats.wdma_fe[0] += !!(proc & ZP_IMG_PROC_EOF);
	csis->stats.wdma_le[0] += !!(proc & ZP_IMG_PROC_FRAME_DONE);
	if (err) {
		unsigned long be = err;
		unsigned int bit;

		csis->stats.wdma_errors++;
		csis->stats.last_wdma_err = err;
		for_each_set_bit(bit, &be, 32)
			csis->stats.wdma_bits[bit]++;
		if (!csis->wdma_err_logged) {
			csis->wdma_err_logged = true;
			dev_warn(csis->dev, "LoCH %u error 0x%04x (proc 0x%02x): CTL 0x%08x RESOL 0x%08x STRIDE %u, slot 0 0x%x:%08x, activity 0x%08x, WDMA debug 0x%08x, AXI response errors 0x%08x, config miss 0x%x\n",
				 csis->loch, err, proc, img_read(csis, ZP_IMG_CTL),
				 img_read(csis, ZP_IMG_ISP_RESOL),
				 img_read(csis, ZP_IMG_BAYER_STRIDE),
				 img_read(csis, ZP_IMG_BATCH_ADDR_MSB(0)),
				 img_read(csis, ZP_IMG_BATCH_ADDR_LSB(0)),
				 img_read(csis, ZP_IMG_ACTIVITY_STATUS),
				 img_read(csis, ZP_IMG_WDMA_DBG_INFO),
				 img_read(csis, ZP_IMG_AXI_RESP_ERR_STATUS),
				 img_read(csis, ZP_IMG_WDMA_CONFIG_MISS));
		}
	}

	/*
	 * Each frame start latches the slot programmed before it, so the slot
	 * is reprogrammed only here, never mid-frame (a configuration written
	 * during a frame is a "late config" and stops the LoCH): the next
	 * queued buffer, or the scratch buffer.
	 */
	if (proc & ZP_IMG_PROC_SOF) {
		if (csis->active) {
			failed = csis->active;
			csis->active = NULL;
			csis->stats.missed_fe++;
		}
		csis->active = csis->next;
		csis->next = NULL;
		if (!list_empty(&csis->pending)) {
			gs201_csis_next_buffer(csis);
		} else {
			zp_core_set_addr(csis, csis->scratch_dma);
			csis->stats.dropped++;
		}
	}

	if ((proc & ZP_IMG_PROC_DMA_DONE) && csis->active) {
		done = csis->active;
		csis->active = NULL;
		csis->stats.wdma_done++;
	}
	spin_unlock(&csis->slock);

	if (failed) {
		failed->vb.vb2_buf.timestamp = ktime_get_ns();
		failed->vb.sequence = csis->sequence++;
		failed->vb.field = V4L2_FIELD_NONE;
		vb2_buffer_done(&failed->vb.vb2_buf, keep_failed ?
				VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR);
	}
	if (done) {
		done->vb.vb2_buf.timestamp = ktime_get_ns();
		done->vb.sequence = csis->sequence++;
		done->vb.field = V4L2_FIELD_NONE;
		vb2_buffer_done(&done->vb.vb2_buf, err && !keep_failed ?
				VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	}

	return IRQ_HANDLED;
}

/* -----------------------------------------------------------------------------
 * Streaming
 */

static void gs201_csis_dvfs_vote(struct gs201_csis *csis, struct clk *clk,
				 const char *name, unsigned int khz,
				 unsigned long *old)
{
	int ret;

	*old = 0;
	if (!clk || !khz)
		return;

	*old = clk_get_rate(clk);
	ret = clk_set_rate(clk, (unsigned long)khz * 1000);
	if (ret) {
		dev_warn(csis->dev, "%s DVFS to %u kHz: %d\n", name, khz, ret);
		*old = 0;
	}
}

static void gs201_csis_dvfs_restore(struct clk *clk, unsigned long *old)
{
	if (clk && *old)
		clk_set_rate(clk, *old);
	*old = 0;
}

static int gs201_csis_phy_on(struct gs201_csis *csis, unsigned int lanes,
			     const struct gs201_csis_format *fmt)
{
	union phy_configure_opts opts = { };
	s64 link_freq;
	int ret;

	link_freq = v4l2_get_link_freq(csis->source.pad, fmt->bpp, lanes * 2);
	if (link_freq < 0) {
		dev_err(csis->dev, "no link frequency: %lld\n", link_freq);
		return link_freq;
	}

	/*
	 * D-PHY: twice the link frequency per lane. C-PHY: the trio's symbol
	 * rate, which the sensor drivers here report as twice their link
	 * frequency too [I: only the PHY's settle choice depends on it].
	 */
	opts.mipi_dphy.lanes = lanes;
	opts.mipi_dphy.hs_clk_rate = link_freq * 2;
	if (opts.mipi_dphy.hs_clk_rate < CSIS_MIN_LANE_RATE ||
	    opts.mipi_dphy.hs_clk_rate > CSIS_MAX_LANE_RATE)
		return -ERANGE;

	ret = phy_set_mode_ext(csis->phy, PHY_MODE_MIPI_DPHY,
			       csis->cphy ? PHY_TYPE_CPHY : PHY_TYPE_DPHY);
	if (ret)
		return ret;

	ret = phy_configure(csis->phy, &opts);
	if (ret)
		return ret;

	return phy_power_on(csis->phy);
}

static int gs201_csis_enable_streams(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     u32 pad, u64 streams_mask)
{
	struct gs201_csis *csis = container_of(sd, struct gs201_csis, sd);
	const struct gs201_csis_format *f;
	const struct v4l2_mbus_framefmt *mf;
	int lanes, ret;

	if (!allow_stream) {
		dev_err(csis->dev, "streaming disabled (allow_stream=0)\n");
		return -EPERM;
	}

	mf = v4l2_subdev_state_get_format(state, CSIS_PAD_SINK);
	f = gs201_csis_find_code(mf->code);
	if (!f)
		return -EPIPE;

	lanes = v4l2_get_active_data_lanes(csis->source.pad,
					   csis->bus.num_data_lanes);
	if (lanes < 0)
		return lanes;

	if (csis->sysreg) {
		unsigned int sel = 0, rst = 0;

		regmap_read(csis->sysreg, CSIS_SYSREG_LINK_SEL, &sel);
		regmap_read(csis->sysreg, CSIS_SYSREG_PHY_RESET, &rst);
		dev_info(csis->dev, "sysreg 0x488 = 0x%08x, 0x500 = 0x%08x before start\n",
			 sel, rst);
		if (sysreg_link_sel)
			regmap_update_bits(csis->sysreg, CSIS_SYSREG_LINK_SEL,
					   BIT(csis->link_id + 24),
					   BIT(csis->link_id + 24));
		csis->link_sel_saved = sel;
		if (link_sel_bits)
			regmap_update_bits(csis->sysreg, CSIS_SYSREG_LINK_SEL,
					   link_sel_bits, link_sel_bits);
	}

	gs201_csis_dvfs_vote(csis, csis->cam_clk, "CAM",
			     csis->cphy ? cam_khz_cphy : cam_khz,
			     &csis->cam_rate_old);

	ret = gs201_csis_phy_on(csis, lanes, f);
	if (ret)
		goto err_dvfs;

	gs201_csis_link_start(csis, mf, f, lanes);
	if (csis->dma_on && wdma_after_link && !csis->core)
		gs201_csis_wdma_start(csis);

	ret = v4l2_subdev_enable_streams(csis->source.sd,
					 csis->source.pad->index, BIT(0));
	if (ret)
		goto err_link;

	/* Bring-up: what each part looks like right after the start. */
	if (csis->sysreg) {
		unsigned int sel = 0, rst = 0;

		regmap_read(csis->sysreg, CSIS_SYSREG_LINK_SEL, &sel);
		regmap_read(csis->sysreg, CSIS_SYSREG_PHY_RESET, &rst);
		dev_info(csis->dev, "sysreg 0x488 = 0x%08x, 0x500 = 0x%08x after start\n",
			 sel, rst);
	}
	dev_info(csis->dev, "link %u on: %ux%u DT 0x%02x, %d %s; CMN_CTRL 0x%08x PHY_CMN 0x%08x INT_MSK0/1 0x%08x/0x%08x SRC0/1 0x%08x/0x%08x FRM_CNT 0x%08x; CAM %lu Hz, INT %lu Hz\n",
		 csis->link_id, mf->width, mf->height, f->data_type, lanes,
		 csis->cphy ? "C-PHY trios" : "lanes",
		 link_read(csis, CSIS_CMN_CTRL), link_read(csis, CSIS_PHY_CMN_CTRL),
		 link_read(csis, CSIS_INT_MSK0), link_read(csis, CSIS_INT_MSK1),
		 link_read(csis, CSIS_INT_SRC0), link_read(csis, CSIS_INT_SRC1),
		 link_read(csis, CSIS_FRM_CNT_CH(0)),
		 csis->cam_clk ? clk_get_rate(csis->cam_clk) : 0,
		 csis->int_clk ? clk_get_rate(csis->int_clk) : 0);

	return 0;

err_link:
	gs201_csis_link_stop(csis);
	phy_power_off(csis->phy);
err_dvfs:
	gs201_csis_dvfs_restore(csis->cam_clk, &csis->cam_rate_old);
	return ret;
}

static int gs201_csis_disable_streams(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      u32 pad, u64 streams_mask)
{
	struct gs201_csis *csis = container_of(sd, struct gs201_csis, sd);

	/* Bring-up: raw state before stopping, whatever the interrupts did. */
	dev_info(csis->dev, "link %u off: FRM_CNT 0x%08x SRC0/1 0x%08x/0x%08x FS/FE 0x%08x/0x%08x CMN_CTRL 0x%08x, %u link interrupts\n",
		 csis->link_id, link_read(csis, CSIS_FRM_CNT_CH(0)),
		 link_read(csis, CSIS_INT_SRC0), link_read(csis, CSIS_INT_SRC1),
		 link_read(csis, CSIS_INT_SRC_FS), link_read(csis, CSIS_INT_SRC_FE),
		 link_read(csis, CSIS_CMN_CTRL), csis->stats.link_irqs);

	v4l2_subdev_disable_streams(csis->source.sd, csis->source.pad->index,
				    BIT(0));
	if (link_sel_bits && csis->sysreg)
		regmap_write(csis->sysreg, CSIS_SYSREG_LINK_SEL, csis->link_sel_saved);
	gs201_csis_link_stop(csis);
	phy_power_off(csis->phy);
	gs201_csis_dvfs_restore(csis->cam_clk, &csis->cam_rate_old);

	return 0;
}

/* -----------------------------------------------------------------------------
 * Subdev
 */

static int gs201_csis_enum_mbus_code(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad == CSIS_PAD_SOURCE) {
		if (code->index)
			return -EINVAL;
		code->code = v4l2_subdev_state_get_format(state, CSIS_PAD_SOURCE)->code;
		return 0;
	}

	if (code->index >= ARRAY_SIZE(gs201_csis_formats))
		return -EINVAL;

	code->code = gs201_csis_formats[code->index].code;

	return 0;
}

static int gs201_csis_set_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_format *sdf)
{
	const struct gs201_csis_format *f;
	struct v4l2_mbus_framefmt *mf;

	if (sdf->pad == CSIS_PAD_SOURCE)
		return v4l2_subdev_get_fmt(sd, state, sdf);

	f = gs201_csis_find_code(sdf->format.code);
	if (!f)
		f = &gs201_csis_formats[0];

	mf = v4l2_subdev_state_get_format(state, CSIS_PAD_SINK);
	*mf = sdf->format;
	mf->code = f->code;
	mf->field = V4L2_FIELD_NONE;
	/* No alignment: libcamera drops sensor modes whose size changes here */
	mf->width = clamp_t(u32, mf->width, 1, CSIS_MAX_WIDTH);
	mf->height = clamp_t(u32, mf->height, 1, CSIS_MAX_HEIGHT);
	sdf->format = *mf;

	*v4l2_subdev_state_get_format(state, CSIS_PAD_SOURCE) = *mf;

	return 0;
}

static int gs201_csis_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				     struct v4l2_mbus_frame_desc *fd)
{
	const struct gs201_csis_format *f;
	struct v4l2_subdev_state *state;
	u32 code;

	if (pad != CSIS_PAD_SOURCE)
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	code = v4l2_subdev_state_get_format(state, CSIS_PAD_SOURCE)->code;
	v4l2_subdev_unlock_state(state);

	f = gs201_csis_find_code(code);
	if (!f)
		return -EPIPE;

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_PARALLEL;
	fd->num_entries = 1;
	fd->entry[0].pixelcode = code;

	return 0;
}

static int gs201_csis_init_state(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format sdf = {
		.pad = CSIS_PAD_SINK,
		.format = {
			.code = gs201_csis_formats[0].code,
			.width = CSIS_DEF_WIDTH,
			.height = CSIS_DEF_HEIGHT,
			.colorspace = V4L2_COLORSPACE_RAW,
		},
	};

	return gs201_csis_set_fmt(sd, state, &sdf);
}

static const struct v4l2_subdev_pad_ops gs201_csis_pad_ops = {
	.enum_mbus_code = gs201_csis_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = gs201_csis_set_fmt,
	.get_frame_desc = gs201_csis_get_frame_desc,
	.enable_streams = gs201_csis_enable_streams,
	.disable_streams = gs201_csis_disable_streams,
};

static const struct v4l2_subdev_video_ops gs201_csis_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static int gs201_csis_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				      struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_FRAME_SYNC)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub, 0, NULL);
}

static const struct v4l2_subdev_core_ops gs201_csis_core_ops = {
	.subscribe_event = gs201_csis_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops gs201_csis_subdev_ops = {
	.core = &gs201_csis_core_ops,
	.video = &gs201_csis_video_ops,
	.pad = &gs201_csis_pad_ops,
};

static const struct v4l2_subdev_internal_ops gs201_csis_internal_ops = {
	.init_state = gs201_csis_init_state,
};

static int gs201_csis_subdev_link_validate(struct media_link *link)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(link->sink->entity);
	int ret;

	ret = v4l2_subdev_link_validate(link);
	if (ret)
		dev_info(sd->dev, "link %s:%u -> %s:%u invalid: %d (formats differ?)\n",
			 link->source->entity->name, link->source->index,
			 link->sink->entity->name, link->sink->index, ret);

	return ret;
}

static const struct media_entity_operations gs201_csis_entity_ops = {
	.link_validate = gs201_csis_subdev_link_validate,
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
};

/* -----------------------------------------------------------------------------
 * Video node
 */

static void gs201_csis_fill_pix(struct v4l2_pix_format *pix,
				const struct gs201_csis_format *f)
{
	unsigned int align = stride_align;

	if (!is_power_of_2(align) || align < CSIS_STRIDE_ALIGN ||
	    align > CSIS_STRIDE_ALIGN_MAX)
		align = CSIS_STRIDE_ALIGN;

	pix->width = clamp_t(u32, ALIGN(pix->width, 4), 4, CSIS_MAX_WIDTH);
	pix->height = clamp_t(u32, pix->height, 1, CSIS_MAX_HEIGHT);
	pix->pixelformat = f->fourcc;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = ALIGN(DIV_ROUND_UP(pix->width * f->bpp, 8),
				  align);
	pix->sizeimage = pix->bytesperline *
			 (pix->height + min(extra_lines, CSIS_MAX_HEIGHT));
	pix->colorspace = V4L2_COLORSPACE_RAW;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int gs201_csis_querycap(struct file *file, void *fh,
			       struct v4l2_capability *cap)
{
	strscpy(cap->driver, CSIS_DRIVER_NAME, sizeof(cap->driver));
	strscpy(cap->card, "gs201 CSIS WDMA", sizeof(cap->card));

	return 0;
}

static int gs201_csis_enum_fmt(struct file *file, void *fh,
			       struct v4l2_fmtdesc *f)
{
	unsigned int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(gs201_csis_formats); i++) {
		if (f->mbus_code && gs201_csis_formats[i].code != f->mbus_code)
			continue;
		if (n++ == f->index) {
			f->pixelformat = gs201_csis_formats[i].fourcc;
			return 0;
		}
	}

	return -EINVAL;
}

static int gs201_csis_enum_framesizes(struct file *file, void *fh,
				      struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || !gs201_csis_find_fourcc(fsize->pixel_format))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = 4;
	fsize->stepwise.max_width = CSIS_MAX_WIDTH;
	fsize->stepwise.step_width = 4;
	fsize->stepwise.min_height = 1;
	fsize->stepwise.max_height = CSIS_MAX_HEIGHT;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int gs201_csis_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct gs201_csis *csis = video_drvdata(file);

	f->fmt.pix = csis->pix;

	return 0;
}

static int gs201_csis_try_fmt(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	const struct gs201_csis_format *fmt;

	fmt = gs201_csis_find_fourcc(f->fmt.pix.pixelformat);
	if (!fmt)
		fmt = &gs201_csis_formats[0];

	gs201_csis_fill_pix(&f->fmt.pix, fmt);

	return 0;
}

static int gs201_csis_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct gs201_csis *csis = video_drvdata(file);

	if (vb2_is_busy(&csis->queue))
		return -EBUSY;

	gs201_csis_try_fmt(file, fh, f);
	csis->pix = f->fmt.pix;
	csis->fmt = gs201_csis_find_fourcc(csis->pix.pixelformat);

	return 0;
}

static const struct v4l2_ioctl_ops gs201_csis_ioctl_ops = {
	.vidioc_querycap = gs201_csis_querycap,
	.vidioc_enum_fmt_vid_cap = gs201_csis_enum_fmt,
	.vidioc_enum_framesizes = gs201_csis_enum_framesizes,
	.vidioc_g_fmt_vid_cap = gs201_csis_g_fmt,
	.vidioc_s_fmt_vid_cap = gs201_csis_s_fmt,
	.vidioc_try_fmt_vid_cap = gs201_csis_try_fmt,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations gs201_csis_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

static int gs201_csis_link_validate(struct media_link *link)
{
	struct video_device *vdev =
		media_entity_to_video_device(link->sink->entity);
	struct gs201_csis *csis = video_get_drvdata(vdev);
	struct v4l2_subdev_format sdf = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = CSIS_PAD_SOURCE,
	};
	int ret;

	ret = v4l2_subdev_call_state_active(&csis->sd, pad, get_fmt, &sdf);
	if (ret)
		return ret;

	if (sdf.format.width != csis->pix.width ||
	    sdf.format.height != csis->pix.height ||
	    sdf.format.code != csis->fmt->code) {
		dev_info(csis->dev, "video node format %ux%u 0x%x differs from CSIS source %ux%u 0x%x\n",
			 csis->pix.width, csis->pix.height, csis->fmt->code,
			 sdf.format.width, sdf.format.height, sdf.format.code);
		return -EPIPE;
	}

	return 0;
}

static const struct media_entity_operations gs201_csis_vdev_entity_ops = {
	.link_validate = gs201_csis_link_validate,
};

/* -----------------------------------------------------------------------------
 * vb2
 */

static int gs201_csis_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
				  unsigned int *nplanes, unsigned int sizes[],
				  struct device *alloc_devs[])
{
	struct gs201_csis *csis = vb2_get_drv_priv(vq);

	/*
	 * Cached buffers, synced for the CPU at dequeue: libcamera's
	 * software ISP reads every frame with the CPU for its statistics
	 * (and debayers on the CPU without a GPU), which through the
	 * default uncached mapping costs tens of milliseconds a frame.
	 * REQBUFS sets the queue's coherency from the caller's flags just
	 * before calling here, and libcamera passes none.
	 */
	if (cached_buffers)
		vq->non_coherent_mem = 1;

	if (*nplanes)
		return sizes[0] < csis->pix.sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = csis->pix.sizeimage;

	return 0;
}

static int gs201_csis_buf_prepare(struct vb2_buffer *vb)
{
	struct gs201_csis *csis = vb2_get_drv_priv(vb->vb2_queue);
	struct gs201_csis_buffer *buf =
		container_of(to_vb2_v4l2_buffer(vb), struct gs201_csis_buffer, vb);

	if (vb2_plane_size(vb, 0) < csis->pix.sizeimage)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, csis->pix.sizeimage);
	buf->addr = vb2_dma_contig_plane_dma_addr(vb, 0);

	/* The WDMA takes 32-bit addresses [I]: refuse anything above. */
	if (upper_32_bits(buf->addr + csis->pix.sizeimage - 1))
		return -EINVAL;

	return 0;
}

static void gs201_csis_buf_queue(struct vb2_buffer *vb)
{
	struct gs201_csis *csis = vb2_get_drv_priv(vb->vb2_queue);
	struct gs201_csis_buffer *buf =
		container_of(to_vb2_v4l2_buffer(vb), struct gs201_csis_buffer, vb);
	unsigned long flags;

	spin_lock_irqsave(&csis->slock, flags);
	list_add_tail(&buf->list, &csis->pending);
	/* zumapro programs the slot at frame start only */
	if (!csis->next && vb2_start_streaming_called(vb->vb2_queue) &&
	    !csis->core)
		gs201_csis_next_buffer(csis);
	spin_unlock_irqrestore(&csis->slock, flags);
}

static void gs201_csis_return_all(struct gs201_csis *csis,
				  enum vb2_buffer_state state)
{
	struct gs201_csis_buffer *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&csis->slock, flags);
	if (csis->active)
		list_add_tail(&csis->active->list, &csis->pending);
	if (csis->next)
		list_add_tail(&csis->next->list, &csis->pending);
	csis->active = NULL;
	csis->next = NULL;
	list_for_each_entry_safe(buf, tmp, &csis->pending, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&csis->slock, flags);
}

static int gs201_csis_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct gs201_csis *csis = vb2_get_drv_priv(vq);
	unsigned long flags;
	int ret;

	ret = gs201_csis_claim(csis);
	if (ret)
		goto err_return;

	ret = video_device_pipeline_alloc_start(&csis->vdev);
	if (ret)
		goto err_release;

	csis->sequence = 0;
	memset(&csis->stats, 0, sizeof(csis->stats));
	csis->wdma_err_logged = false;
	csis->dma_on = !link_only && (csis->soc->wdma || csis->core);

	spin_lock_irqsave(&csis->slock, flags);
	if (!csis->next)
		gs201_csis_next_buffer(csis);
	spin_unlock_irqrestore(&csis->slock, flags);

	if (csis->dma_on && csis->core) {
		gs201_csis_dvfs_vote(csis, csis->int_clk, "INT", fe_int_khz,
				     &csis->int_rate_old);
		gs201_csis_dvfs_vote(csis, csis->fe_clk, "ISP-FE", fe_khz,
				     &csis->fe_rate_old);
		csis->scratch_size = csis->pix.sizeimage;
		csis->scratch = dma_alloc_coherent(csis->dev, csis->scratch_size,
						   &csis->scratch_dma, GFP_KERNEL);
		if (!csis->scratch) {
			gs201_csis_dvfs_restore(csis->fe_clk, &csis->fe_rate_old);
			gs201_csis_dvfs_restore(csis->int_clk, &csis->int_rate_old);
			ret = -ENOMEM;
			goto err_pipeline;
		}
		if (!csis->next)
			zp_core_set_addr(csis, csis->scratch_dma);
		zp_core_route(csis);
		zp_core_wdma_start(csis);
	} else if (csis->dma_on) {
		ret = gs201_csis_route(csis);
		if (ret)
			goto err_pipeline;

		gs201_csis_dvfs_vote(csis, csis->int_clk, "INT", int_khz, &csis->int_rate_old);
		if (!wdma_after_link)
			gs201_csis_wdma_start(csis);
	}

	ret = v4l2_subdev_enable_streams(&csis->sd, CSIS_PAD_SOURCE, BIT(0));
	if (ret)
		goto err_wdma;

	return 0;

err_wdma:
	if (csis->dma_on && csis->core) {
		zp_core_wdma_stop(csis);
		zp_core_unroute(csis);
		dma_free_coherent(csis->dev, csis->scratch_size, csis->scratch,
				  csis->scratch_dma);
		csis->scratch = NULL;
		gs201_csis_dvfs_restore(csis->fe_clk, &csis->fe_rate_old);
		gs201_csis_dvfs_restore(csis->int_clk, &csis->int_rate_old);
	} else if (csis->dma_on) {
		gs201_csis_wdma_stop(csis);
		gs201_csis_dvfs_restore(csis->int_clk, &csis->int_rate_old);
		gs201_csis_unroute(csis);
	}
err_pipeline:
	video_device_pipeline_stop(&csis->vdev);
err_release:
	gs201_csis_release(csis);
err_return:
	gs201_csis_return_all(csis, VB2_BUF_STATE_QUEUED);
	return ret;
}

/* Bring-up: the error bits seen and how often, to debugfs or the log */
static void gs201_csis_print_bits(struct seq_file *s, struct device *dev,
				  const char *name, const u32 *count,
				  unsigned int n)
{
	char buf[160];
	int len = 0;
	unsigned int i;

	for (i = 0; i < n; i++)
		if (count[i])
			len += scnprintf(buf + len, sizeof(buf) - len, " b%u x%u",
					 i, count[i]);
	if (!len)
		return;
	if (s)
		seq_printf(s, "%s_bits%s\n", name, buf);
	else
		dev_info(dev, "%s:%s\n", name, buf);
}

static void gs201_csis_stop_streaming(struct vb2_queue *vq)
{
	struct gs201_csis *csis = vb2_get_drv_priv(vq);
	unsigned long flags;

	spin_lock_irqsave(&csis->slock, flags);
	csis->wdma_arm = 0;
	spin_unlock_irqrestore(&csis->slock, flags);
	v4l2_subdev_disable_streams(&csis->sd, CSIS_PAD_SOURCE, BIT(0));
	if (csis->dma_on && csis->core) {
		dev_info(csis->dev, "LoCH %u at stop: frame %u, done frame %u, activity 0x%08x, WDMA debug 0x%08x, AXI response errors 0x%08x\n",
			 csis->loch, img_read(csis, ZP_IMG_FRAME_ID),
			 img_read(csis, ZP_IMG_DONE_FRAME_ID),
			 img_read(csis, ZP_IMG_ACTIVITY_STATUS),
			 img_read(csis, ZP_IMG_WDMA_DBG_INFO),
			 img_read(csis, ZP_IMG_AXI_RESP_ERR_STATUS));
		zp_core_wdma_stop(csis);
		zp_core_unroute(csis);
		dma_free_coherent(csis->dev, csis->scratch_size, csis->scratch,
				  csis->scratch_dma);
		csis->scratch = NULL;
		gs201_csis_dvfs_restore(csis->fe_clk, &csis->fe_rate_old);
		gs201_csis_dvfs_restore(csis->int_clk, &csis->int_rate_old);
	} else if (csis->dma_on) {
		gs201_csis_wdma_stop(csis);
		gs201_csis_dvfs_restore(csis->int_clk, &csis->int_rate_old);
		gs201_csis_unroute(csis);
	}
	video_device_pipeline_stop(&csis->vdev);
	gs201_csis_return_all(csis, VB2_BUF_STATE_ERROR);
	gs201_csis_release(csis);

	dev_info(csis->dev, "stopped: FS %u FE %u link errors %u (last 0x%08x), WDMA done %u errors %u (last 0x%08x), dropped %u, missed frame ends %u\n",
		 csis->stats.frame_start, csis->stats.frame_end,
		 csis->stats.link_errors, csis->stats.last_link_err,
		 csis->stats.wdma_done, csis->stats.wdma_errors,
		 csis->stats.last_wdma_err, csis->stats.dropped,
		 csis->stats.missed_fe);
	if (csis->dma_on && csis->dma)
		dev_info(csis->dev, "WDMA debug at stop: frames 0x%08x, lines/data 0x%08x\n",
			 readl(csis->dma + CSIS_DMA_CMN_DBG_VCNT(csis->ctx_id)),
			 readl(csis->dma + CSIS_DMA_CMN_DBG_HDCNT(csis->ctx_id)));
	if (wdma_late || wdma_recover)
		dev_info(csis->dev, "WDMA started at link frame end %u, %u restarts\n",
			 csis->stats.wdma_started_at, csis->stats.wdma_restarts);
	gs201_csis_print_bits(NULL, csis->dev, "INT0 error bits", csis->stats.int0_bits, 32);
	gs201_csis_print_bits(NULL, csis->dev, "INT1 error bits", csis->stats.int1_bits, 8);
	gs201_csis_print_bits(NULL, csis->dev, "WDMA error bits", csis->stats.wdma_bits, 32);
	gs201_csis_print_bits(NULL, csis->dev, "link FS by VC", csis->stats.link_fs, 32);
	gs201_csis_print_bits(NULL, csis->dev, "link FE by VC", csis->stats.link_fe, 32);
	gs201_csis_print_bits(NULL, csis->dev, "WDMA FS by channel", csis->stats.wdma_fs, 4);
	gs201_csis_print_bits(NULL, csis->dev, "WDMA FE by channel", csis->stats.wdma_fe, 4);
	gs201_csis_print_bits(NULL, csis->dev, "WDMA line ends by channel", csis->stats.wdma_le, 4);
}

static const struct vb2_ops gs201_csis_vb2_ops = {
	.queue_setup = gs201_csis_queue_setup,
	.buf_prepare = gs201_csis_buf_prepare,
	.buf_queue = gs201_csis_buf_queue,
	.start_streaming = gs201_csis_start_streaming,
	.stop_streaming = gs201_csis_stop_streaming,
};

/* -----------------------------------------------------------------------------
 * Debugfs: counters, and the link and WDMA registers while streaming
 */

static int gs201_csis_stats_show(struct seq_file *s, void *unused)
{
	struct gs201_csis *csis = s->private;
	struct gs201_csis_stats st;
	unsigned long flags;

	spin_lock_irqsave(&csis->slock, flags);
	st = csis->stats;
	spin_unlock_irqrestore(&csis->slock, flags);

	seq_printf(s, "frame_start %u\nframe_end %u\nlink_errors %u\nlast_link_err 0x%08x\n",
		   st.frame_start, st.frame_end, st.link_errors, st.last_link_err);
	seq_printf(s, "wdma_done %u\nwdma_errors %u\nlast_wdma_err 0x%08x\ndropped %u\nmissed_fe %u\nlink_irqs %u\n",
		   st.wdma_done, st.wdma_errors, st.last_wdma_err, st.dropped,
		   st.missed_fe, st.link_irqs);
	seq_printf(s, "core_irqs %u\ncore_last_agg 0x%08x\ncore_last_proc 0x%08x\n",
		   st.core_irqs, st.core_last_agg, st.core_last_proc);
	gs201_csis_print_bits(s, NULL, "int0", st.int0_bits, 32);
	gs201_csis_print_bits(s, NULL, "int1", st.int1_bits, 8);
	gs201_csis_print_bits(s, NULL, "wdma", st.wdma_bits, 32);
	gs201_csis_print_bits(s, NULL, "link_fs_vc", st.link_fs, 32);
	gs201_csis_print_bits(s, NULL, "link_fe_vc", st.link_fe, 32);
	gs201_csis_print_bits(s, NULL, "wdma_fs_ch", st.wdma_fs, 4);
	gs201_csis_print_bits(s, NULL, "wdma_fe_ch", st.wdma_fe, 4);
	gs201_csis_print_bits(s, NULL, "wdma_lines_ch", st.wdma_le, 4);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(gs201_csis_stats);

/* Registers are read only while streaming, when the block is known to run. */
static int gs201_csis_regs_show(struct seq_file *s, void *unused)
{
	static const u32 link_regs[] = {
		CSIS_VERSION, CSIS_CMN_CTRL, CSIS_UPD_SDW, CSIS_INT_MSK0,
		CSIS_INT_SRC0, CSIS_INT_MSK1, CSIS_INT_SRC1,
		CSIS_ISP_CONFIG_CH(0), CSIS_ISP_RESOL_CH(0), CSIS_FRM_CNT_CH(0),
		CSIS_PHY_CMN_CTRL,
	};
	static const u32 wdma_regs[] = {
		CSIS_WDMA_DATA_CTRL, CSIS_WDMA_INT_MSK, CSIS_WDMA_INT_SRC,
		CSIS_WDMA_FRO_FRAME_NUM,
	};
	static const u32 ch_regs[] = {
		CSIS_WDMA_CTRL, CSIS_WDMA_FMT, CSIS_WDMA_ADDR(1),
		CSIS_WDMA_RESOL, CSIS_WDMA_STRIDE, CSIS_WDMA_FCNT_SEQ,
	};
	struct gs201_csis *csis = s->private;
	unsigned int i;

	mutex_lock(&csis->vlock);
	if (!vb2_is_streaming(&csis->queue)) {
		mutex_unlock(&csis->vlock);
		seq_puts(s, "not streaming\n");
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(link_regs); i++)
		seq_printf(s, "link 0x%03x = 0x%08x\n", link_regs[i],
			   link_read(csis, link_regs[i]));
	for (i = 0; csis->wdma && i < ARRAY_SIZE(wdma_regs); i++)
		seq_printf(s, "wdma 0x%03x = 0x%08x\n", wdma_regs[i],
			   wdma_read(csis, wdma_regs[i]));
	for (i = 0; csis->wdma && i < ARRAY_SIZE(ch_regs); i++) {
		u32 r = CSIS_WDMA_CH(csis->wdma_ch, ch_regs[i]);

		seq_printf(s, "wdma 0x%03x = 0x%08x\n", r, wdma_read(csis, r));
	}
	mutex_unlock(&csis->vlock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(gs201_csis_regs);

/*
 * While streaming: every link's control, frame counter and interrupt
 * sources (to see which link the sensor's data reaches), this link's
 * register windows, and the CSIS sysreg's routing page.
 */
static int gs201_csis_scan_show(struct seq_file *s, void *unused)
{
	static const struct { u32 start, end; } win[] = {
		{ 0x000, 0x07c }, { 0x500, 0x50c }, { 0x600, 0x60c },
		{ 0x680, 0x6a0 }, { 0x700, 0x73c },
	};
	/*
	 * Only the CSIS sysreg registers this driver programs: reading the
	 * whole 0x400-0x50c range hit a word that aborts the SoC.
	 */
	static const u32 sysreg_regs[] = {
		0x408, 0x40c, 0x410, 0x430, 0x488, 0x500,
	};
	struct gs201_csis *csis = s->private;
	unsigned int i;
	u32 off, val;

	mutex_lock(&csis->vlock);
	if (!vb2_is_streaming(&csis->queue)) {
		mutex_unlock(&csis->vlock);
		seq_puts(s, "not streaming\n");
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(win); i++)
		for (off = win[i].start; off <= win[i].end; off += 4)
			seq_printf(s, "link%u+0x%03x = 0x%08x\n", csis->link_id,
				   off, link_read(csis, off));

	for (i = 0; csis->sysreg && i < ARRAY_SIZE(sysreg_regs); i++) {
		if (regmap_read(csis->sysreg, sysreg_regs[i], &val))
			continue;
		seq_printf(s, "sysreg+0x%03x = 0x%08x\n", sysreg_regs[i], val);
	}
	mutex_unlock(&csis->vlock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(gs201_csis_scan);

/* -----------------------------------------------------------------------------
 * Async: the sensor
 */

static int gs201_csis_notify_bound(struct v4l2_async_notifier *n,
				   struct v4l2_subdev *sd,
				   struct v4l2_async_connection *asc)
{
	struct gs201_csis *csis = container_of(n, struct gs201_csis, notifier);
	struct fwnode_handle *ep, *remote_ep;
	int src, ret;

	/*
	 * Link only the sensor endpoint that faces this link's own endpoint:
	 * a sensor may have endpoints to several links (only one of them
	 * enabled), and helpers that walk all of its endpoints would add one
	 * link per endpoint.
	 */
	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(csis->dev), 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!ep)
		return -ENOTCONN;
	remote_ep = fwnode_graph_get_remote_endpoint(ep);
	fwnode_handle_put(ep);
	if (!remote_ep)
		return -ENOTCONN;

	src = media_entity_get_fwnode_pad(&sd->entity, remote_ep,
					  MEDIA_PAD_FL_SOURCE);
	fwnode_handle_put(remote_ep);
	if (src < 0)
		return src;

	ret = media_create_pad_link(&sd->entity, src, &csis->sd.entity,
				    CSIS_PAD_SINK, MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		return ret;

	csis->source.sd = sd;
	csis->source.pad = &sd->entity.pads[src];

	return 0;
}

static int gs201_csis_notify_complete(struct v4l2_async_notifier *n)
{
	struct gs201_csis *csis = container_of(n, struct gs201_csis, notifier);

	return v4l2_device_register_subdev_nodes(&csis->v4l2_dev);
}

static const struct v4l2_async_notifier_operations gs201_csis_notify_ops = {
	.bound = gs201_csis_notify_bound,
	.complete = gs201_csis_notify_complete,
};

static int gs201_csis_parse_dt(struct gs201_csis *csis)
{
	struct v4l2_fwnode_endpoint vep = { .bus_type = V4L2_MBUS_UNKNOWN };
	struct v4l2_async_connection *asc;
	struct fwnode_handle *ep;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(csis->dev), 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!ep)
		return -ENOTCONN;

	ret = v4l2_fwnode_endpoint_parse(ep, &vep);
	if (ret)
		goto out;

	if (vep.bus_type != V4L2_MBUS_CSI2_DPHY &&
	    vep.bus_type != V4L2_MBUS_CSI2_CPHY) {
		ret = -EINVAL;
		goto out;
	}
	csis->cphy = vep.bus_type == V4L2_MBUS_CSI2_CPHY;
	csis->bus = vep.bus.mipi_csi2;
	if (!csis->bus.num_data_lanes) {
		ret = -EINVAL;
		goto out;
	}

	v4l2_async_nf_init(&csis->notifier, &csis->v4l2_dev);
	asc = v4l2_async_nf_add_fwnode_remote(&csis->notifier, ep,
					      struct v4l2_async_connection);
	if (IS_ERR(asc)) {
		ret = PTR_ERR(asc);
		goto out;
	}
	csis->notifier.ops = &gs201_csis_notify_ops;

	ret = v4l2_async_nf_register(&csis->notifier);
out:
	fwnode_handle_put(ep);
	return ret;
}

/* -----------------------------------------------------------------------------
 * Probe
 */

static int gs201_csis_register(struct gs201_csis *csis)
{
	struct v4l2_subdev *sd = &csis->sd;
	struct video_device *vdev = &csis->vdev;
	struct vb2_queue *q = &csis->queue;
	int ret;

	csis->mdev.dev = csis->dev;
	strscpy(csis->mdev.model, "gs201 CSIS", sizeof(csis->mdev.model));
	strscpy(csis->mdev.driver_name, CSIS_DRIVER_NAME,
		sizeof(csis->mdev.driver_name));
	media_device_init(&csis->mdev);

	csis->v4l2_dev.mdev = &csis->mdev;
	ret = v4l2_device_register(csis->dev, &csis->v4l2_dev);
	if (ret)
		goto err_mdev;

	v4l2_subdev_init(sd, &gs201_csis_subdev_ops);
	sd->internal_ops = &gs201_csis_internal_ops;
	sd->owner = THIS_MODULE;
	sd->dev = csis->dev;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &gs201_csis_entity_ops;
	sd->fwnode = dev_fwnode(csis->dev);
	snprintf(sd->name, sizeof(sd->name), "gs201-csis link %u", csis->link_id);
	v4l2_set_subdevdata(sd, csis);

	csis->pads[CSIS_PAD_SINK].flags = MEDIA_PAD_FL_SINK |
					  MEDIA_PAD_FL_MUST_CONNECT;
	csis->pads[CSIS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE |
					    MEDIA_PAD_FL_MUST_CONNECT;
	ret = media_entity_pads_init(&sd->entity, CSIS_PADS_NUM, csis->pads);
	if (ret)
		goto err_v4l2;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto err_entity;

	ret = v4l2_device_register_subdev(&csis->v4l2_dev, sd);
	if (ret)
		goto err_finalize;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = csis;
	q->buf_struct_size = sizeof(struct gs201_csis_buffer);
	q->ops = &gs201_csis_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	/*
	 * A full camera pool falls back to the page allocator, where a
	 * high-order request fails with a warning: report -ENOMEM only.
	 * (Not DMA_ATTR_NO_WARN: the non-coherent allocation the queue uses
	 * for cached buffers refuses any attribute but single pages.)
	 */
	q->gfp_flags = __GFP_NOWARN;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &csis->vlock;
	q->dev = csis->dev;
	q->min_queued_buffers = 2;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_subdev;

	snprintf(vdev->name, sizeof(vdev->name), "gs201-csis-wdma%u", csis->ctx_id);
	vdev->fops = &gs201_csis_fops;
	vdev->ioctl_ops = &gs201_csis_ioctl_ops;
	vdev->v4l2_dev = &csis->v4l2_dev;
	vdev->queue = q;
	vdev->lock = &csis->vlock;
	vdev->release = video_device_release_empty;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_IO_MC;
	vdev->entity.ops = &gs201_csis_vdev_entity_ops;
	csis->vdev_pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	ret = media_entity_pads_init(&vdev->entity, 1, &csis->vdev_pad);
	if (ret)
		goto err_subdev;
	video_set_drvdata(vdev, csis);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_vdev_entity;

	ret = media_create_pad_link(&sd->entity, CSIS_PAD_SOURCE,
				    &vdev->entity, 0,
				    MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		goto err_vdev;

	ret = gs201_csis_parse_dt(csis);
	if (ret)
		goto err_vdev;

	ret = media_device_register(&csis->mdev);
	if (ret)
		goto err_notifier;

	return 0;

err_notifier:
	v4l2_async_nf_unregister(&csis->notifier);
	v4l2_async_nf_cleanup(&csis->notifier);
err_vdev:
	video_unregister_device(vdev);
err_vdev_entity:
	media_entity_cleanup(&vdev->entity);
err_subdev:
	v4l2_device_unregister_subdev(sd);
err_finalize:
	v4l2_subdev_cleanup(sd);
err_entity:
	media_entity_cleanup(&sd->entity);
err_v4l2:
	v4l2_device_unregister(&csis->v4l2_dev);
err_mdev:
	media_device_cleanup(&csis->mdev);
	return ret;
}

static int gs201_csis_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gs201_csis *csis;
	struct resource *res;
	int irq, ret;

	csis = devm_kzalloc(dev, sizeof(*csis), GFP_KERNEL);
	if (!csis)
		return -ENOMEM;

	csis->dev = dev;
	mutex_init(&csis->vlock);
	spin_lock_init(&csis->slock);
	INIT_LIST_HEAD(&csis->pending);
	platform_set_drvdata(pdev, csis);

	csis->soc = of_device_get_match_data(dev);
	if (!csis->soc)
		return -ENODEV;

	csis->link = devm_platform_ioremap_resource_byname(pdev, "link");
	if (IS_ERR(csis->link))
		return PTR_ERR(csis->link);

	if (!csis->soc->wdma) {
		/* zumapro: the CSIS core page, optional (without it, link only) */
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "core");
		if (res) {
			csis->core = devm_ioremap(dev, res->start, resource_size(res));
			if (!csis->core)
				return -ENOMEM;
		}
		device_property_read_u32(dev, "google,image-loch", &csis->loch);
		csis->fe_clk = devm_clk_get_optional(dev, "fe");
		if (IS_ERR(csis->fe_clk))
			return dev_err_probe(dev, PTR_ERR(csis->fe_clk), "ISP-FE clock\n");
		if (csis->loch > 6)
			return dev_err_probe(dev, -EINVAL, "bad google,image-loch\n");
		goto no_wdma;
	}

	/* The DMA common page is shared by all instances: map, don't claim. */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dma");
	if (!res)
		return -EINVAL;
	csis->dma = devm_ioremap(dev, res->start, resource_size(res));
	if (!csis->dma)
		return -ENOMEM;

	/* So may the WDMA context: map it shared too (one stream at a time). */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "wdma");
	if (!res)
		return -EINVAL;
	csis->wdma = devm_ioremap(dev, res->start, resource_size(res));
	if (!csis->wdma)
		return -ENOMEM;

	/* The EBUF control is shared by all links too; optional. */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ebuf");
	if (res) {
		csis->ebuf_regs = devm_ioremap(dev, res->start, resource_size(res));
		if (!csis->ebuf_regs)
			return -ENOMEM;
	}

no_wdma:
	csis->cam_clk = devm_clk_get_optional(dev, "cam");
	if (IS_ERR(csis->cam_clk))
		return dev_err_probe(dev, PTR_ERR(csis->cam_clk), "CAM clock\n");
	csis->int_clk = devm_clk_get_optional(dev, "int");
	if (IS_ERR(csis->int_clk))
		return dev_err_probe(dev, PTR_ERR(csis->int_clk), "INT clock\n");

	ret = device_property_read_u32(dev, "google,csis-link", &csis->link_id);
	if (ret || csis->link_id >= csis->soc->num_links)
		return dev_err_probe(dev, -EINVAL, "bad google,csis-link\n");
	ret = device_property_read_u32(dev, "google,wdma-context", &csis->ctx_id);
	if (csis->soc->wdma && (ret || csis->ctx_id > 3))
		return dev_err_probe(dev, -EINVAL, "bad google,wdma-context\n");

	csis->ebuf = csis->ctx_id;
	device_property_read_u32(dev, "google,ebuf", &csis->ebuf);
	if (csis->ebuf > 3)
		return dev_err_probe(dev, -EINVAL, "bad google,ebuf\n");
	csis->wdma_ch = wdma_channel;
	csis->lrte_epd = device_property_read_bool(dev,
						   "google,lrte-packet-delimiters");

	if (csis->soc->wdma) {
		csis->sysreg = syscon_regmap_lookup_by_phandle(dev->of_node,
							       "google,csis-sysreg");
		if (IS_ERR(csis->sysreg))
			return dev_err_probe(dev, PTR_ERR(csis->sysreg),
					     "no CSIS sysreg\n");
	}

	csis->phy = devm_phy_get(dev, NULL);
	if (IS_ERR(csis->phy))
		return dev_err_probe(dev, PTR_ERR(csis->phy), "no PHY\n");

	ret = of_reserved_mem_device_init(dev);
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "memory region\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(csis->soc->dma_bits));
	if (ret)
		goto err_mem;

	if (csis->core) {
		/* the LoCH writes a status record per frame: give it a page */
		csis->sq_buf = dmam_alloc_coherent(dev, PAGE_SIZE, &csis->sq_dma,
						   GFP_KERNEL);
		if (!csis->sq_buf) {
			ret = -ENOMEM;
			goto err_mem;
		}
		irq = platform_get_irq_byname(pdev, "core");
		if (irq < 0) {
			ret = irq;
			goto err_mem;
		}
		ret = devm_request_irq(dev, irq, zp_core_irq, IRQF_SHARED,
				       dev_name(dev), csis);
		if (ret)
			goto err_mem;
	}

	irq = platform_get_irq_byname(pdev, "link");
	if (irq < 0) {
		ret = irq;
		goto err_mem;
	}
	ret = devm_request_irq(dev, irq, gs201_csis_link_irq, 0,
			       dev_name(dev), csis);
	if (ret)
		goto err_mem;

	if (csis->soc->wdma) {
		irq = platform_get_irq_byname(pdev, "wdma");
		if (irq < 0) {
			ret = irq;
			goto err_mem;
		}
		ret = devm_request_irq(dev, irq, gs201_csis_wdma_irq, IRQF_SHARED,
				       dev_name(dev), csis);
		if (ret)
			goto err_mem;
	}

	csis->fmt = &gs201_csis_formats[0];
	csis->pix.width = CSIS_DEF_WIDTH;
	csis->pix.height = CSIS_DEF_HEIGHT;
	gs201_csis_fill_pix(&csis->pix, csis->fmt);

	ret = gs201_csis_register(csis);
	if (ret)
		goto err_mem;

	csis->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("stats", 0400, csis->debugfs, csis,
			    &gs201_csis_stats_fops);
	debugfs_create_file("regs", 0400, csis->debugfs, csis,
			    &gs201_csis_regs_fops);
	debugfs_create_file("scan", 0400, csis->debugfs, csis,
			    &gs201_csis_scan_fops);

	dev_info(dev, "link %u -> WDMA context %u, %u %s%s\n",
		 csis->link_id, csis->ctx_id, csis->bus.num_data_lanes,
		 csis->cphy ? "C-PHY trios" : "D-PHY data lanes",
		 csis->lrte_epd ? ", LRTE packet delimiters" : "");

	return 0;

err_mem:
	of_reserved_mem_device_release(dev);
	return ret;
}

static void gs201_csis_remove(struct platform_device *pdev)
{
	struct gs201_csis *csis = platform_get_drvdata(pdev);

	debugfs_remove_recursive(csis->debugfs);
	media_device_unregister(&csis->mdev);
	v4l2_async_nf_unregister(&csis->notifier);
	v4l2_async_nf_cleanup(&csis->notifier);
	video_unregister_device(&csis->vdev);
	media_entity_cleanup(&csis->vdev.entity);
	v4l2_device_unregister_subdev(&csis->sd);
	v4l2_subdev_cleanup(&csis->sd);
	media_entity_cleanup(&csis->sd.entity);
	v4l2_device_unregister(&csis->v4l2_dev);
	media_device_cleanup(&csis->mdev);
	of_reserved_mem_device_release(&pdev->dev);
}

static const struct gs201_csis_soc gs201_csis_gs201 = {
	.wdma = true,
	.num_links = 8,
	.pixel_mode = CSIS_ISPCFG_PIXEL_MODE_QUAD,
	.dma_bits = CSIS_WDMA_ADDR_BITS,
	.dbg_bit23 = true,
	.phy_cmn_ctrl = true,
};

/* Links 0-11 have contexts and interrupts; stock wires sensors to 0-8. */
static const struct gs201_csis_soc gs201_csis_zumapro = {
	.num_links = 12,
	/* the zumapro HAL's CMN_CTRL bits 15:12 are 5, gs201's 4 */
	.cmn_ctrl_extra = ZP_LINK_CMN_CTRL_F12,
	.pixel_mode = ZP_LINK_PIXEL_MODE,
	.dma_bits = ZP_ADDR_BITS,
};

static const struct of_device_id gs201_csis_of_match[] = {
	{ .compatible = "google,gs201-csis", .data = &gs201_csis_gs201 },
	{ .compatible = "google,zumapro-csis", .data = &gs201_csis_zumapro },
	{ }
};
MODULE_DEVICE_TABLE(of, gs201_csis_of_match);

static struct platform_driver gs201_csis_driver = {
	.probe = gs201_csis_probe,
	.remove = gs201_csis_remove,
	.driver = {
		.name = CSIS_DRIVER_NAME,
		.of_match_table = gs201_csis_of_match,
	},
};
module_platform_driver(gs201_csis_driver);

MODULE_DESCRIPTION("Google gs201 CSIS camera receiver and WDMA driver");
MODULE_LICENSE("GPL");
