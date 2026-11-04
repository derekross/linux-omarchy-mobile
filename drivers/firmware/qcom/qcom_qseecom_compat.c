// SPDX-License-Identifier: GPL-2.0-only
/*
 * Downstream-compatible /dev/qseecom for a vendor HAL in a chroot
 *
 * Qualcomm phones of the SM8150 generation run their trusted applications
 * (TAs) in QSEE, and Android's vendor HALs reach them through Qualcomm's
 * libQSEEComAPI, which drives the downstream kernel's /dev/qseecom ioctls.
 * On the OnePlus 7T Pro the Goodix optical fingerprint HAL does exactly
 * that: it loads its TA, shares buffers with it and calls it, while
 * qseecomd serves the TA's requests for files and secure storage
 * (listeners). Mainline only has an in-kernel QSEECOM client (uefisecapp)
 * and no userspace interface, so this driver provides the small subset of
 * the downstream ABI those two programs use, on top of the SCM calls in
 * qcom_scm.c. It is not meant for upstream: its ABI is the vendor's.
 *
 * Shared memory comes from the SoC's no-map qseecom carve-out (memory-region
 * in the device tree), handed to userspace as dma-bufs through one ioctl of
 * our own (QSEECOM_COMPAT_IOCTL_ALLOC); a preload shim in the chroot turns
 * the HAL's ion allocations into that ioctl. The memory is mapped
 * write-combined on both sides (kernel and userspace) and has no linear-map
 * alias, so no cache maintenance is done around SCM calls.
 *
 * The stock device tree of the 7T Pro sets qcom,commonlib64-loaded-by-uefi
 * and qcom,appsbl-qseecom-support, so the bootloader has already loaded the
 * common library (cmnlib64) and told QSEE where the app region is: this
 * driver never loads cmnlib and never sends the app-region notification. It
 * also sets qcom,qsee-reentrancy-support = <2>: requests that block on a
 * busy listener come back as "blocked" and are continued here.
 *
 * Safety gates (module parameters, fixed at load):
 *  - app_allowlist: the TAs that may be loaded or bound, by the name the
 *    client gives. Empty by default, so nothing loads until the human
 *    names the TA. QSEE takes a TA's identity from its signed image, not
 *    from that name, so after a load QSEE is asked for the TA's signed name
 *    and the TA is shut down unless QSEE knows it by the id the load
 *    returned: the list holds for images, not just for names.
 *  - app_signed_names: client:signed pairs, for a TA whose signed name
 *    differs from the name its client loads it by (the OnePlus 7T Pro's
 *    Goodix TA is loaded as gfp9558 and signed as fingerprint). Without a
 *    pair the two are the same. A pair doesn't allow anything by itself.
 *  - listener_allowlist: the listener IDs that may register. Empty by
 *    default.
 *  - allow_rpmb: the RPMB listener stays refused, even when allowlisted,
 *    unless this is set. An RPMB write can't be undone, and the project's
 *    safety rules make every RPMB write the human's call.
 *
 * All sequences of SCM calls (a send with its listener round trips, an app
 * load, a listener registration) run under one mutex: QSEE isn't re-entrant
 * for us. Mainline's own QSEECOM client (uefisecapp) is only created on
 * allowlisted machines, and the OnePlus 7T Pro isn't one, so nothing else
 * issues QSEECOM calls concurrently on this phone. On a machine where
 * uefisecapp runs, its calls are serialized per call only, not per
 * sequence.
 *
 * The ioctl numbers and layouts, and the SCM calls the OnePlus 7T Pro's
 * fingerprint stack makes (version, app start, app send, listener
 * registration and response), were checked against OnePlus's
 * libQSEEComAPI (OxygenOS 10.0.14) and QSEE on that phone. The app
 * shutdown, listener deregistration and continue-blocked-request calls
 * are written from the downstream ABI and not confirmed on hardware;
 * they are marked VERIFY below. A listener whose deregistration fails
 * keeps its buffer until reboot, as QSEE may still write to it.
 */

#include <linux/dma-buf.h>
#include <linux/err.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/fs.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/jiffies.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include "qcom_scm.h"	/* QCOM_SCM_ARGS() */

/* --- Safety gates and tunables --- */

static char *app_allowlist = "";
module_param(app_allowlist, charp, 0444);
MODULE_PARM_DESC(app_allowlist,
		 "Comma-separated TA names that may be loaded or bound (default: none)");

static char *app_signed_names = "";
module_param(app_signed_names, charp, 0444);
MODULE_PARM_DESC(app_signed_names,
		 "Comma-separated client:signed TA name pairs, where QSEE knows a TA by another name than its client loads it by (default: none)");

static char *listener_allowlist = "";
module_param(listener_allowlist, charp, 0444);
MODULE_PARM_DESC(listener_allowlist,
		 "Comma-separated listener IDs that may register (default: none)");

static bool allow_rpmb;
module_param(allow_rpmb, bool, 0444);
MODULE_PARM_DESC(allow_rpmb,
		 "Let the RPMB listener register (irreversible writes; default: no)");

static unsigned int listener_timeout_ms = 10000;
module_param(listener_timeout_ms, uint, 0444);
MODULE_PARM_DESC(listener_timeout_ms,
		 "How long a request waits for its listener's answer (ms)");

static unsigned int listener_register_cmd = 1;
module_param(listener_register_cmd, uint, 0444);
MODULE_PARM_DESC(listener_register_cmd,
		 "TZ command that registers a listener: 1 (plain) or 6 (smcinvoke variant)");

/* The RPMB service listener ID of libQSEEComAPI/qseecomd. */
#define QSC_LISTENER_RPMB		0x2000

/* --- Downstream userspace ABI (uapi/linux/qseecom.h of the msm kernels) --- */

/*
 * The layouts below are what libQSEEComAPI from OnePlus's vendor image
 * uses: arm64 userspace, pointers 8 bytes, lengths 32 bits (the size is
 * encoded in the ioctl numbers it issues).
 */
#define QSC_MAX_APP_NAME	64
#define QSC_MAX_ION_FD		4

struct qsc_register_listener_req {
	__u32 listener_id;
	__s32 ifd_data_fd;
	__u64 virt_sb_base;		/* void * */
	__u32 sb_size;
	__u32 __pad;
};

struct qsc_send_cmd_req {
	__u64 cmd_req_buf;		/* void * */
	__u32 cmd_req_len;
	__u32 __pad0;
	__u64 resp_buf;			/* void * */
	__u32 resp_len;
	__u32 __pad1;
};

struct qsc_ion_fd_info {
	__s32 fd;
	__u32 cmd_buf_offset;
};

struct qsc_send_modfd_cmd_req {
	__u64 cmd_req_buf;		/* void * */
	__u32 cmd_req_len;
	__u32 __pad0;
	__u64 resp_buf;			/* void * */
	__u32 resp_len;
	struct qsc_ion_fd_info ifd_data[QSC_MAX_ION_FD];
	__u32 __pad1;
};

struct qsc_send_resp_req {
	__u64 resp_buf;			/* void * */
	__u32 resp_len;
	__u32 __pad;
};

struct qsc_load_img_req {
	__u32 mdt_len;
	__u32 img_len;
	__s32 ifd_data_fd;
	char img_name[QSC_MAX_APP_NAME];
	__u32 app_arch;
	__u32 app_id;			/* out */
};

struct qsc_set_sb_mem_param_req {
	__s32 ifd_data_fd;
	__u32 __pad0;
	__u64 virt_sb_base;		/* void * */
	__u32 sb_len;
	__u32 __pad1;
};

struct qsc_qseos_version_req {
	__u32 qseos_version;		/* out */
};

struct qsc_app_load_query {
	char app_name[QSC_MAX_APP_NAME];
	__u32 app_id;			/* out */
	__u32 app_arch;
};

struct qsc_send_modfd_listener_resp {
	__u64 resp_buf_ptr;		/* void * */
	__u32 resp_len;
	struct qsc_ion_fd_info ifd_data[QSC_MAX_ION_FD];
	__u32 __pad;
};

/* Ours, for the ion shim: a dma-buf from the carve-out. */
struct qseecom_compat_alloc {
	__u64 len;			/* in, rounded up to 4 KiB */
	__u32 flags;			/* in, must be 0 */
	__s32 fd;			/* out, O_CLOEXEC | O_RDWR dma-buf */
};

static_assert(sizeof(struct qsc_register_listener_req) == 24);
static_assert(sizeof(struct qsc_send_cmd_req) == 32);
static_assert(sizeof(struct qsc_send_modfd_cmd_req) == 64);
static_assert(sizeof(struct qsc_send_resp_req) == 16);
static_assert(sizeof(struct qsc_load_img_req) == 84);
static_assert(sizeof(struct qsc_set_sb_mem_param_req) == 24);
static_assert(sizeof(struct qsc_qseos_version_req) == 4);
static_assert(sizeof(struct qsc_app_load_query) == 72);
static_assert(sizeof(struct qsc_send_modfd_listener_resp) == 48);
static_assert(sizeof(struct qseecom_compat_alloc) == 16);

/*
 * Checked against the ioctl codes OnePlus's libQSEEComAPI.so (OxygenOS
 * 10.0.14, arm64) issues: register is 1 (24 bytes), unregister 2, send 3
 * (32 bytes); every size below matches. It also issues 13, 16, 17, 18 and
 * 24 (external ELF, service commands, disk-encryption keys), which this
 * driver refuses.
 */
#define QSC_IOC_MAGIC			0x97
#define QSC_IOCTL_REGISTER_LISTENER_REQ	_IOWR(QSC_IOC_MAGIC, 1, struct qsc_register_listener_req)
#define QSC_IOCTL_UNREGISTER_LISTENER_REQ _IO(QSC_IOC_MAGIC, 2)
#define QSC_IOCTL_SEND_CMD_REQ		_IOWR(QSC_IOC_MAGIC, 3, struct qsc_send_cmd_req)
#define QSC_IOCTL_SEND_MODFD_CMD_REQ	_IOWR(QSC_IOC_MAGIC, 4, struct qsc_send_modfd_cmd_req)
#define QSC_IOCTL_RECEIVE_REQ		_IO(QSC_IOC_MAGIC, 5)
#define QSC_IOCTL_SEND_RESP_REQ		_IO(QSC_IOC_MAGIC, 6)
#define QSC_IOCTL_LOAD_APP_REQ		_IOWR(QSC_IOC_MAGIC, 7, struct qsc_load_img_req)
#define QSC_IOCTL_SET_MEM_PARAM_REQ	_IOWR(QSC_IOC_MAGIC, 8, struct qsc_set_sb_mem_param_req)
#define QSC_IOCTL_UNLOAD_APP_REQ	_IO(QSC_IOC_MAGIC, 9)
#define QSC_IOCTL_GET_QSEOS_VERSION_REQ	_IOWR(QSC_IOC_MAGIC, 10, struct qsc_qseos_version_req)
#define QSC_IOCTL_PERF_ENABLE_REQ	_IO(QSC_IOC_MAGIC, 11)
#define QSC_IOCTL_PERF_DISABLE_REQ	_IO(QSC_IOC_MAGIC, 12)
#define QSC_IOCTL_APP_LOADED_QUERY_REQ	_IOWR(QSC_IOC_MAGIC, 15, struct qsc_app_load_query)
#define QSC_IOCTL_SEND_MODFD_RESP \
	_IOWR(QSC_IOC_MAGIC, 21, struct qsc_send_modfd_listener_resp)
#define QSC_IOCTL_SET_BUS_SCALING_REQ	_IOWR(QSC_IOC_MAGIC, 23, int)
#define QSC_IOCTL_SEND_MODFD_CMD_64_REQ	_IOWR(QSC_IOC_MAGIC, 35, struct qsc_send_modfd_cmd_req)
#define QSC_IOCTL_SEND_MODFD_RESP_64 \
	_IOWR(QSC_IOC_MAGIC, 36, struct qsc_send_modfd_listener_resp)
#define QSEECOM_COMPAT_IOCTL_ALLOC	_IOWR(QSC_IOC_MAGIC, 0xc0, struct qseecom_compat_alloc)

/* --- TZ syscall IDs (owner/service/command of the downstream TZ IDs) --- */

#define QSC_OWNER_TZ_APPS		48
#define QSC_OWNER_QSEE_OS		50
#define QSC_SVC_APP_ID_PLACEHOLDER	0
#define QSC_SVC_APP_MGR			1
#define QSC_SVC_LISTENER		2

/* TZ_APP_QSAPP_SEND_DATA_ID, as mainline's qcom_scm_qseecom_app_send */
#define QSC_CMD_APP_SEND		1
/* TZ_OS_APP_START_ID, args mdt_len, img_len, image address */
#define QSC_CMD_APP_START		1
/* VERIFY: TZ_OS_APP_SHUTDOWN_ID, arg app_id */
#define QSC_CMD_APP_SHUTDOWN		2
/* TZ_OS_REGISTER_LISTENER_ID (1) / ..._SMCINVOKE_ID (6) */
#define QSC_CMD_LISTENER_REGISTER	1
#define QSC_CMD_LISTENER_REGISTER_SMCI	6
/* VERIFY: TZ_OS_DEREGISTER_LISTENER_ID, arg listener_id */
#define QSC_CMD_LISTENER_DEREGISTER	2
/* TZ_OS_LISTENER_RESPONSE_HANDLER_ID, args listener_id, status */
#define QSC_CMD_LISTENER_RESPONSE	3
/* VERIFY: TZ_OS_CONTINUE_BLOCKED_REQUEST_ID, arg blocked session */
#define QSC_CMD_CONTINUE_BLOCKED	4

/* Results, as in qcom_scm.c */
#define QSC_RESULT_SUCCESS		0
#define QSC_RESULT_INCOMPLETE		1
#define QSC_RESULT_BLOCKED_ON_LISTENER	2
#define QSC_RESULT_FAILURE		0xffffffffU
#define QSC_RES_APP_ID			0xee01
#define QSC_RES_LISTENER_ID		0xee02

/* downstream's qseos_version as userspace sees it */
#define QSC_QSEOS_VERSION_14		0x14

/* A runaway exchange with QSEE ends here rather than looping forever. */
#define QSC_MAX_ROUND_TRIPS		1024

/* --- State --- */

struct qsc_buf {
	struct qsc *qsc;
	phys_addr_t phys;
	void *kva;
	size_t len;
};

/* A shared buffer as one client sees it: a dma-buf of ours and its user VA. */
struct qsc_sb {
	struct dma_buf *dmabuf;		/* reference held, or NULL */
	struct qsc_buf *buf;
	u64 user_base;
	size_t len;
};

enum qsc_listener_state {
	QSC_L_IDLE,		/* no request */
	QSC_L_PENDING,		/* TZ has a request; RECEIVE_REQ not yet back */
	QSC_L_SERVING,		/* the listener has it */
	QSC_L_DONE,		/* the listener has answered */
};

struct qsc_listener {
	struct kref ref;
	struct list_head node;		/* qsc->listeners, under lock */
	struct qsc *qsc;
	u32 id;
	struct qsc_sb sb;		/* fixed while registered */
	enum qsc_listener_state state;	/* under qsc->lock */
	bool aborted;			/* under qsc->lock */
	wait_queue_head_t rcv_wq;
	wait_queue_head_t rsp_wq;
};

struct qsc_app {
	struct list_head node;		/* qsc->apps, under cmd_mutex */
	char name[QSC_MAX_APP_NAME];
	u32 app_id;
	unsigned int users;
	bool ours;			/* loaded by this driver: we may shut it down */
};

struct qsc {
	struct device *dev;
	struct miscdevice misc;
	struct gen_pool *pool;
	phys_addr_t base;
	void *kva;
	size_t size;

	struct mutex cmd_mutex;		/* one QSEE sequence at a time; apps */
	struct list_head apps;
	spinlock_t lock;		/* listeners and their state */
	struct list_head listeners;
	/*
	 * Listeners QSEE failed to deregister, under cmd_mutex. Never freed:
	 * each holds its buffer's dma-buf until reboot (qsc_unregister_listener).
	 */
	struct list_head kept;
};

/* One open /dev/qseecom */
struct qsc_client {
	struct qsc *qsc;
	struct mutex lock;		/* the fields below */
	struct qsc_sb sb;		/* SET_MEM_PARAM */
	struct qsc_app *app;		/* LOAD_APP or a bound APP_LOADED_QUERY */
	struct qsc_listener *listener;	/* REGISTER_LISTENER */
};

/* --- Allowlists --- */

static bool qsc_token_next(const char **list, char *tok, size_t size)
{
	const char *s = *list, *e;
	size_t n;

	while (*s == ',' || *s == ' ')
		s++;
	if (!*s)
		return false;
	e = strchrnul(s, ',');
	n = e - s;
	while (n && s[n - 1] == ' ')
		n--;
	*list = e;
	if (n >= size)
		n = size - 1;	/* too long to match anything real */
	memcpy(tok, s, n);
	tok[n] = '\0';
	return true;
}

static bool qsc_app_allowed(const char *name)
{
	const char *list = app_allowlist ?: "";
	char tok[QSC_MAX_APP_NAME];

	while (qsc_token_next(&list, tok, sizeof(tok)))
		if (!strcmp(tok, name))
			return true;
	return false;
}

/*
 * The name QSEE knows the TA @name by: its pair in app_signed_names, or
 * @name itself.
 */
static void qsc_app_signed_name(const char *name, char *signed_name)
{
	const char *list = app_signed_names ?: "";
	char tok[2 * QSC_MAX_APP_NAME];
	char *sep;

	while (qsc_token_next(&list, tok, sizeof(tok))) {
		sep = strchr(tok, ':');
		if (!sep || sep == tok || !sep[1])
			continue;
		*sep = '\0';
		if (!strcmp(tok, name) && strlen(sep + 1) < QSC_MAX_APP_NAME) {
			strscpy(signed_name, sep + 1, QSC_MAX_APP_NAME);
			return;
		}
	}
	strscpy(signed_name, name, QSC_MAX_APP_NAME);
}

static bool qsc_listener_allowed(u32 id)
{
	const char *list = listener_allowlist ?: "";
	char tok[16];
	u32 v;

	if (id == QSC_LISTENER_RPMB && !allow_rpmb)
		return false;
	while (qsc_token_next(&list, tok, sizeof(tok)))
		if (!kstrtou32(tok, 0, &v) && v == id)
			return true;
	return false;
}

/* --- dma-bufs from the carve-out --- */

static struct sg_table *qsc_dmabuf_map(struct dma_buf_attachment *attach,
				       enum dma_data_direction dir)
{
	/* CPU mmap and this driver only; no device attaches to these. */
	return ERR_PTR(-EOPNOTSUPP);
}

static void qsc_dmabuf_unmap(struct dma_buf_attachment *attach,
			     struct sg_table *sgt, enum dma_data_direction dir)
{
}

static void qsc_dmabuf_release(struct dma_buf *dmabuf)
{
	struct qsc_buf *buf = dmabuf->priv;

	memset(buf->kva, 0, buf->len);
	gen_pool_free(buf->qsc->pool, (unsigned long)buf->kva, buf->len);
	kfree(buf);
}

static int qsc_dmabuf_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct qsc_buf *buf = dmabuf->priv;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;

	if (off >= buf->len || size > buf->len - off)
		return -EINVAL;

	vm_flags_set(vma, VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start,
			       PHYS_PFN(buf->phys + off), size,
			       vma->vm_page_prot);
}

/* Write-combined on both sides: nothing to flush or invalidate. */
static int qsc_dmabuf_begin_cpu(struct dma_buf *dmabuf,
				enum dma_data_direction dir)
{
	return 0;
}

static int qsc_dmabuf_end_cpu(struct dma_buf *dmabuf,
			      enum dma_data_direction dir)
{
	return 0;
}

static const struct dma_buf_ops qsc_dmabuf_ops = {
	.map_dma_buf = qsc_dmabuf_map,
	.unmap_dma_buf = qsc_dmabuf_unmap,
	.release = qsc_dmabuf_release,
	.mmap = qsc_dmabuf_mmap,
	.begin_cpu_access = qsc_dmabuf_begin_cpu,
	.end_cpu_access = qsc_dmabuf_end_cpu,
};

static int qsc_alloc(struct qsc *qsc, struct qseecom_compat_alloc __user *uarg)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp);
	struct qseecom_compat_alloc req;
	struct dma_buf *dmabuf;
	struct qsc_buf *buf;
	unsigned long kva;
	size_t len;
	int fd;

	if (copy_from_user(&req, uarg, sizeof(req)))
		return -EFAULT;
	if (req.flags || !req.len || req.len > qsc->size)
		return -EINVAL;
	len = PAGE_ALIGN(req.len);

	buf = kzalloc_obj(*buf);
	if (!buf)
		return -ENOMEM;
	kva = gen_pool_alloc(qsc->pool, len);
	if (!kva) {
		kfree(buf);
		return -ENOMEM;
	}
	buf->qsc = qsc;
	buf->kva = (void *)kva;
	buf->phys = gen_pool_virt_to_phys(qsc->pool, kva);
	buf->len = len;
	memset(buf->kva, 0, len);

	exp.ops = &qsc_dmabuf_ops;
	exp.size = len;
	exp.flags = O_RDWR;
	exp.priv = buf;
	dmabuf = dma_buf_export(&exp);
	if (IS_ERR(dmabuf)) {
		gen_pool_free(qsc->pool, kva, len);
		kfree(buf);
		return PTR_ERR(dmabuf);
	}

	fd = dma_buf_fd(dmabuf, O_CLOEXEC);
	if (fd < 0) {
		dma_buf_put(dmabuf);	/* releases buf */
		return fd;
	}
	req.fd = fd;
	req.len = len;
	if (copy_to_user(uarg, &req, sizeof(req))) {
		/* The fd is installed; userspace owns it now. */
		return -EFAULT;
	}
	return 0;
}

/* A dma-buf of ours by fd, with a reference; anything else is refused. */
static struct dma_buf *qsc_dmabuf_get(struct qsc *qsc, int fd)
{
	struct dma_buf *dmabuf = dma_buf_get(fd);

	if (IS_ERR(dmabuf))
		return dmabuf;
	if (dmabuf->ops != &qsc_dmabuf_ops) {
		dma_buf_put(dmabuf);
		dev_warn_ratelimited(qsc->dev, "fd %d is not a qseecom buffer\n", fd);
		return ERR_PTR(-EINVAL);
	}
	return dmabuf;
}

static int qsc_sb_set(struct qsc *qsc, struct qsc_sb *sb, int fd,
		      u64 user_base, u32 len)
{
	struct dma_buf *dmabuf;
	struct qsc_buf *buf;

	if (!len || !user_base)
		return -EINVAL;
	dmabuf = qsc_dmabuf_get(qsc, fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);
	buf = dmabuf->priv;
	if (len > buf->len) {
		dma_buf_put(dmabuf);
		return -EINVAL;
	}
	sb->dmabuf = dmabuf;
	sb->buf = buf;
	sb->user_base = user_base;
	sb->len = len;
	return 0;
}

static void qsc_sb_clear(struct qsc_sb *sb)
{
	if (sb->dmabuf)
		dma_buf_put(sb->dmabuf);
	memset(sb, 0, sizeof(*sb));
}

/* Offset of [uptr, uptr + len) inside the shared buffer, or -EINVAL. */
static long qsc_sb_offset(const struct qsc_sb *sb, u64 uptr, u32 len)
{
	u64 off;

	if (!sb->dmabuf || !len || uptr < sb->user_base)
		return -EINVAL;
	off = uptr - sb->user_base;
	if (off >= sb->len || len > sb->len - off)
		return -EINVAL;
	return off;
}

/* --- QSEE calls and the listener round trips --- */

static int qsc_call(u32 owner, u32 svc, u32 cmd, u32 arginfo,
		    const u64 *args, unsigned int nargs,
		    struct qcom_scm_qseecom_raw_res *res)
{
	return qcom_scm_qseecom_raw_call(owner, svc, cmd, arginfo, args, nargs,
					 res);
}

static void qsc_listener_free(struct kref *ref)
{
	struct qsc_listener *l = container_of(ref, struct qsc_listener, ref);

	qsc_sb_clear(&l->sb);
	kfree(l);
}

static void qsc_listener_put(struct qsc_listener *l)
{
	kref_put(&l->ref, qsc_listener_free);
}

static struct qsc_listener *qsc_listener_find(struct qsc *qsc, u32 id)
{
	struct qsc_listener *l;

	spin_lock(&qsc->lock);
	list_for_each_entry(l, &qsc->listeners, node) {
		if (l->id == id && !l->aborted) {
			kref_get(&l->ref);
			spin_unlock(&qsc->lock);
			return l;
		}
	}
	spin_unlock(&qsc->lock);
	return NULL;
}

/*
 * Hand TZ's request to listener @id and wait for its answer. Returns the
 * status to report back to TZ: success only if the listener answered.
 */
static u32 qsc_listener_serve(struct qsc *qsc, u32 id)
{
	struct qsc_listener *l = qsc_listener_find(qsc, id);
	u32 status = QSC_RESULT_FAILURE;
	long left;

	if (!l) {
		dev_warn_ratelimited(qsc->dev,
				     "request for listener 0x%x, which isn't registered\n",
				     id);
		return status;
	}

	spin_lock(&qsc->lock);
	l->state = QSC_L_PENDING;
	spin_unlock(&qsc->lock);
	wake_up_interruptible(&l->rcv_wq);

	left = wait_event_interruptible_timeout(l->rsp_wq,
						READ_ONCE(l->state) == QSC_L_DONE ||
						READ_ONCE(l->aborted),
						msecs_to_jiffies(listener_timeout_ms));

	spin_lock(&qsc->lock);
	if (l->state == QSC_L_DONE)
		status = QSC_RESULT_SUCCESS;
	l->state = QSC_L_IDLE;	/* a late answer is refused */
	spin_unlock(&qsc->lock);

	if (status != QSC_RESULT_SUCCESS)
		dev_warn_ratelimited(qsc->dev, "listener 0x%x: %s, answering failure\n",
				     id, left == 0 ? "timed out" :
				     left < 0 ? "interrupted" : "gone");
	qsc_listener_put(l);
	return status;
}

/*
 * Finish an exchange that QSEE answered with @res: serve listeners while it
 * is incomplete and continue it while it is blocked. Leaves the final result
 * in @res. Called with cmd_mutex held.
 */
static int qsc_finish(struct qsc *qsc, struct qcom_scm_qseecom_raw_res *res)
{
	unsigned int trips;
	u64 args[2];
	int ret;

	lockdep_assert_held(&qsc->cmd_mutex);

	for (trips = 0; trips < QSC_MAX_ROUND_TRIPS; trips++) {
		switch ((u32)res->result) {
		case QSC_RESULT_SUCCESS:
			return 0;
		case QSC_RESULT_INCOMPLETE:
			if (res->resp_type != QSC_RES_LISTENER_ID)
				dev_dbg(qsc->dev, "incomplete with type 0x%llx\n",
					res->resp_type);
			args[0] = (u32)res->data;		/* listener */
			args[1] = qsc_listener_serve(qsc, args[0]);
			dev_dbg(qsc->dev, "listener 0x%llx served: %lld\n",
				args[0], (long long)args[1]);
			ret = qsc_call(QSC_OWNER_QSEE_OS, QSC_SVC_LISTENER,
				       QSC_CMD_LISTENER_RESPONSE,
				       QCOM_SCM_ARGS(2, QCOM_SCM_VAL, QCOM_SCM_VAL),
				       args, 2, res);
			break;
		case QSC_RESULT_BLOCKED_ON_LISTENER:
			/*
			 * Reentrancy phase 2: data names the busy listener,
			 * resp_type the blocked session (VERIFY). All our
			 * exchanges run under cmd_mutex, so the listener is
			 * free by now; continue the session.
			 */
			args[0] = res->resp_type;
			ret = qsc_call(QSC_OWNER_QSEE_OS, QSC_SVC_LISTENER,
				       QSC_CMD_CONTINUE_BLOCKED,
				       QCOM_SCM_ARGS(1, QCOM_SCM_VAL),
				       args, 1, res);
			break;
		default:
			dev_dbg(qsc->dev, "QSEE result 0x%llx\n", res->result);
			return -EIO;
		}
		if (ret)
			return ret;
	}
	dev_err(qsc->dev, "QSEE exchange did not finish after %u round trips\n",
		trips);
	return -EIO;
}

/* --- Apps --- */

static struct qsc_app *qsc_app_find(struct qsc *qsc, const char *name)
{
	struct qsc_app *app;

	list_for_each_entry(app, &qsc->apps, node)
		if (!strcmp(app->name, name))
			return app;
	return NULL;
}

static struct qsc_app *qsc_app_find_id(struct qsc *qsc, u32 app_id)
{
	struct qsc_app *app;

	list_for_each_entry(app, &qsc->apps, node)
		if (app->app_id == app_id)
			return app;
	return NULL;
}

/* Shut down the TA @app_id, which this driver loaded. cmd_mutex held. */
static int qsc_app_shutdown(struct qsc *qsc, u32 app_id)
{
	struct qcom_scm_qseecom_raw_res res = {};
	u64 args[1];
	int ret;

	lockdep_assert_held(&qsc->cmd_mutex);

	/* VERIFY: app shutdown arg (app_id) */
	args[0] = app_id;
	ret = qsc_call(QSC_OWNER_QSEE_OS, QSC_SVC_APP_MGR, QSC_CMD_APP_SHUTDOWN,
		       QCOM_SCM_ARGS(1, QCOM_SCM_VAL), args, 1, &res);
	if (!ret)
		ret = qsc_finish(qsc, &res);
	return ret;
}

/*
 * APP_START takes no name: QSEE takes the TA's identity from the signed
 * image and returns its app id, whatever the caller called it. So an
 * allowlisted name says nothing about the image until QSEE, asked for the
 * allowlisted TA's signed name (@signed_name), answers with the id the
 * load returned. Shut down a TA that fails this, so that nothing outside
 * app_allowlist stays loaded. cmd_mutex held.
 */
static int qsc_app_verify(struct qsc *qsc, const char *name,
			  const char *signed_name, u32 app_id)
{
	struct qsc_app *other;
	u32 named_id;
	int ret;

	lockdep_assert_held(&qsc->cmd_mutex);

	ret = qcom_scm_qseecom_app_get_id(signed_name, &named_id);
	if (!ret && named_id == app_id)
		return 0;

	if (ret)
		dev_err(qsc->dev, "TA %s: the image loaded as app id %u, but the lookup of %s failed (%d): refused\n",
			name, app_id, signed_name, ret);
	else
		dev_err(qsc->dev, "TA %s: the image loaded as app id %u, but %s is app id %u: refused\n",
			name, app_id, signed_name, named_id);

	/*
	 * QSEE answered with an app this driver already holds under another
	 * name: another client uses it, so it stays.
	 */
	other = qsc_app_find_id(qsc, app_id);
	if (other) {
		dev_err(qsc->dev, "app id %u is bound here as %s: left loaded\n",
			app_id, other->name);
		return -EPERM;
	}

	ret = qsc_app_shutdown(qsc, app_id);
	if (ret)
		dev_err(qsc->dev, "shutting down app id %u failed: %d\n",
			app_id, ret);
	else
		dev_info(qsc->dev, "shut down app id %u\n", app_id);
	return -EPERM;
}

/*
 * Bind @name, loading it from @img if QSEE doesn't know its signed name.
 * cmd_mutex held.
 */
static int qsc_app_get(struct qsc *qsc, const char *name, struct qsc_buf *img,
		       u32 mdt_len, u32 img_len, struct qsc_app **out)
{
	struct qcom_scm_qseecom_raw_res res = {};
	char signed_name[QSC_MAX_APP_NAME];
	struct qsc_app *app;
	bool ours = false;
	u32 app_id;
	u64 args[3];
	int ret;

	lockdep_assert_held(&qsc->cmd_mutex);

	app = qsc_app_find(qsc, name);
	if (app) {
		app->users++;
		*out = app;
		return 0;
	}

	qsc_app_signed_name(name, signed_name);
	ret = qcom_scm_qseecom_app_get_id(signed_name, &app_id);
	if (ret == -ENOENT && img) {
		args[0] = mdt_len;
		args[1] = img_len;
		args[2] = img->phys;
		ret = qsc_call(QSC_OWNER_QSEE_OS, QSC_SVC_APP_MGR,
			       QSC_CMD_APP_START,
			       QCOM_SCM_ARGS(3, QCOM_SCM_VAL, QCOM_SCM_VAL,
					     QCOM_SCM_VAL),
			       args, 3, &res);
		if (!ret)
			ret = qsc_finish(qsc, &res);
		if (!ret && res.resp_type != QSC_RES_APP_ID)
			ret = -EIO;
		if (ret) {
			dev_err(qsc->dev, "loading TA %s failed: %d\n", name, ret);
			return ret;
		}
		app_id = res.data;
		ret = qsc_app_verify(qsc, name, signed_name, app_id);
		if (ret)
			return ret;
		ours = true;
		dev_info(qsc->dev, "loaded TA %s (signed as %s), app id %u\n",
			 name, signed_name, app_id);
	} else if (ret) {
		return ret;
	}

	app = kzalloc_obj(*app);
	if (!app)
		return -ENOMEM;	/* a loaded TA stays loaded until reboot */
	strscpy(app->name, name, sizeof(app->name));
	app->app_id = app_id;
	app->users = 1;
	app->ours = ours;
	list_add(&app->node, &qsc->apps);
	*out = app;
	return 0;
}

static void qsc_app_put(struct qsc *qsc, struct qsc_app *app)
{
	int ret;

	lockdep_assert_held(&qsc->cmd_mutex);

	if (--app->users)
		return;
	if (app->ours) {
		ret = qsc_app_shutdown(qsc, app->app_id);
		if (ret)
			dev_warn(qsc->dev, "unloading TA %s failed: %d\n",
				 app->name, ret);
		else
			dev_info(qsc->dev, "unloaded TA %s\n", app->name);
	}
	list_del(&app->node);
	kfree(app);
}

static int qsc_copy_name(char *dst, const char *src)
{
	size_t n = strnlen(src, QSC_MAX_APP_NAME);

	if (!n || n == QSC_MAX_APP_NAME)
		return -EINVAL;
	memcpy(dst, src, n);
	dst[n] = '\0';
	return 0;
}

static int qsc_load_app(struct qsc_client *c, void __user *uarg)
{
	struct qsc *qsc = c->qsc;
	char name[QSC_MAX_APP_NAME];
	struct qsc_load_img_req req;
	struct dma_buf *dmabuf;
	struct qsc_buf *img;
	struct qsc_app *app;
	int ret;

	if (copy_from_user(&req, uarg, sizeof(req)))
		return -EFAULT;
	ret = qsc_copy_name(name, req.img_name);
	if (ret)
		return ret;
	if (c->app || c->listener)
		return -EBUSY;
	if (!qsc_app_allowed(name)) {
		dev_warn(qsc->dev, "TA %s is not in app_allowlist: refused\n", name);
		return -EPERM;
	}

	dmabuf = qsc_dmabuf_get(qsc, req.ifd_data_fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);
	img = dmabuf->priv;
	/*
	 * img_len is the whole image, the .mdt included (libQSEEComAPI loads
	 * a split app as mdt_len 7464, img_len 6519388 for its nine files of
	 * 6519388 bytes in all), as downstream passes both to TZ.
	 */
	if (!req.mdt_len || req.mdt_len > req.img_len || req.img_len > img->len) {
		dev_warn(qsc->dev, "TA %s: mdt %u, image %u in a buffer of %zu: refused\n",
			 name, req.mdt_len, req.img_len, img->len);
		dma_buf_put(dmabuf);
		return -EINVAL;
	}

	mutex_lock(&qsc->cmd_mutex);
	ret = qsc_app_get(qsc, name, img, req.mdt_len, req.img_len, &app);
	mutex_unlock(&qsc->cmd_mutex);
	dma_buf_put(dmabuf);
	if (ret)
		return ret;

	c->app = app;
	req.app_id = app->app_id;
	if (copy_to_user(uarg, &req, sizeof(req)))
		return -EFAULT;	/* bound; UNLOAD or close releases it */
	return 0;
}

static void qsc_unbind_app(struct qsc_client *c)
{
	if (!c->app)
		return;
	mutex_lock(&c->qsc->cmd_mutex);
	qsc_app_put(c->qsc, c->app);
	mutex_unlock(&c->qsc->cmd_mutex);
	c->app = NULL;
}

/*
 * libQSEEComAPI reads -EEXIST as "already loaded" and 0 as "not loaded".
 * A TA outside app_allowlist is reported as not loaded and not bound.
 */
static int qsc_app_loaded_query(struct qsc_client *c, void __user *uarg)
{
	struct qsc *qsc = c->qsc;
	struct qsc_app_load_query q;
	char name[QSC_MAX_APP_NAME];
	struct qsc_app *app;
	int ret;

	if (copy_from_user(&q, uarg, sizeof(q)))
		return -EFAULT;
	ret = qsc_copy_name(name, q.app_name);
	if (ret)
		return ret;
	if (!qsc_app_allowed(name)) {
		dev_warn(qsc->dev, "query for TA %s, not in app_allowlist\n", name);
		return 0;
	}
	if (c->app || c->listener)
		return -EBUSY;

	mutex_lock(&qsc->cmd_mutex);
	ret = qsc_app_get(qsc, name, NULL, 0, 0, &app);
	mutex_unlock(&qsc->cmd_mutex);
	if (ret == -ENOENT)
		return 0;
	if (ret)
		return ret;

	c->app = app;
	q.app_id = app->app_id;
	if (copy_to_user(uarg, &q, sizeof(q)))
		return -EFAULT;
	return -EEXIST;
}

/* --- Commands --- */

/*
 * Write the physical address of each fd's buffer into the request at its
 * offset (@wide: 64 bits, else 32), or zero them again (@clear).
 */
static int qsc_fixup_fds(struct qsc *qsc, void *base, u32 len,
			 const struct qsc_ion_fd_info *ifd, bool wide,
			 bool clear)
{
	size_t w = wide ? sizeof(u64) : sizeof(u32);
	unsigned int i;

	for (i = 0; i < QSC_MAX_ION_FD; i++) {
		struct dma_buf *dmabuf;
		u64 phys = 0;

		if (ifd[i].fd <= 0)
			continue;
		if (ifd[i].cmd_buf_offset > len || w > len - ifd[i].cmd_buf_offset)
			return -EINVAL;
		if (!clear) {
			dmabuf = qsc_dmabuf_get(qsc, ifd[i].fd);
			if (IS_ERR(dmabuf))
				return PTR_ERR(dmabuf);
			phys = ((struct qsc_buf *)dmabuf->priv)->phys;
			dma_buf_put(dmabuf);
			if (!wide && upper_32_bits(phys))
				return -EINVAL;
		}
		if (wide) {
			memcpy(base + ifd[i].cmd_buf_offset, &phys, sizeof(u64));
		} else {
			u32 phys32 = lower_32_bits(phys);

			memcpy(base + ifd[i].cmd_buf_offset, &phys32, sizeof(u32));
		}
	}
	return 0;
}

static int qsc_send(struct qsc_client *c, u64 ureq, u32 req_len, u64 ursp,
		    u32 rsp_len, const struct qsc_ion_fd_info *ifd, bool wide)
{
	struct qcom_scm_qseecom_raw_res res = {};
	struct qsc *qsc = c->qsc;
	long req_off, rsp_off;
	void *req_kva;
	u64 args[5];
	int ret, ret2;

	if (!c->app)
		return -EINVAL;
	req_off = qsc_sb_offset(&c->sb, ureq, req_len);
	rsp_off = qsc_sb_offset(&c->sb, ursp, rsp_len);
	if (req_off < 0 || rsp_off < 0)
		return -EINVAL;
	req_kva = c->sb.buf->kva + req_off;

	mutex_lock(&qsc->cmd_mutex);
	if (ifd) {
		ret = qsc_fixup_fds(qsc, req_kva, req_len, ifd, wide, false);
		if (ret)
			goto out;
	}

	args[0] = c->app->app_id;
	args[1] = c->sb.buf->phys + req_off;
	args[2] = req_len;
	args[3] = c->sb.buf->phys + rsp_off;
	args[4] = rsp_len;
	/* The same call as mainline's app send */
	ret = qsc_call(QSC_OWNER_TZ_APPS, QSC_SVC_APP_ID_PLACEHOLDER,
		       QSC_CMD_APP_SEND,
		       QCOM_SCM_ARGS(5, QCOM_SCM_VAL, QCOM_SCM_RW, QCOM_SCM_VAL,
				     QCOM_SCM_RW, QCOM_SCM_VAL),
		       args, 5, &res);
	if (!ret)
		ret = qsc_finish(qsc, &res);

	if (ifd) {
		ret2 = qsc_fixup_fds(qsc, req_kva, req_len, ifd, wide, true);
		if (!ret)
			ret = ret2;
	}
out:
	mutex_unlock(&qsc->cmd_mutex);
	return ret;
}

static int qsc_send_cmd(struct qsc_client *c, void __user *uarg)
{
	struct qsc_send_cmd_req req;

	if (copy_from_user(&req, uarg, sizeof(req)))
		return -EFAULT;
	return qsc_send(c, req.cmd_req_buf, req.cmd_req_len, req.resp_buf,
			req.resp_len, NULL, false);
}

static int qsc_send_modfd_cmd(struct qsc_client *c, void __user *uarg,
			      bool wide)
{
	struct qsc_send_modfd_cmd_req req;

	if (copy_from_user(&req, uarg, sizeof(req)))
		return -EFAULT;
	return qsc_send(c, req.cmd_req_buf, req.cmd_req_len, req.resp_buf,
			req.resp_len, req.ifd_data, wide);
}

static int qsc_set_mem_param(struct qsc_client *c, void __user *uarg)
{
	struct qsc_set_sb_mem_param_req req;
	struct qsc_sb sb = {};
	int ret;

	if (copy_from_user(&req, uarg, sizeof(req)))
		return -EFAULT;
	if (c->listener)
		return -EBUSY;
	ret = qsc_sb_set(c->qsc, &sb, req.ifd_data_fd, req.virt_sb_base,
			 req.sb_len);
	if (ret)
		return ret;
	qsc_sb_clear(&c->sb);
	c->sb = sb;
	return 0;
}

/* --- Listeners --- */

static int qsc_register_listener(struct qsc_client *c, void __user *uarg)
{
	struct qcom_scm_qseecom_raw_res res = {};
	struct qsc_register_listener_req req;
	struct qsc *qsc = c->qsc;
	struct qsc_listener *l, *other;
	u64 args[3];
	u32 cmd;
	int ret;

	if (copy_from_user(&req, uarg, sizeof(req)))
		return -EFAULT;
	if (c->app || c->listener)
		return -EBUSY;
	if (!qsc_listener_allowed(req.listener_id)) {
		dev_warn(qsc->dev, "listener 0x%x refused (%s)\n", req.listener_id,
			 req.listener_id == QSC_LISTENER_RPMB && !allow_rpmb ?
			 "RPMB needs allow_rpmb" : "not in listener_allowlist");
		return -EPERM;
	}
	if (listener_register_cmd != QSC_CMD_LISTENER_REGISTER &&
	    listener_register_cmd != QSC_CMD_LISTENER_REGISTER_SMCI)
		return -EINVAL;
	cmd = listener_register_cmd;

	l = kzalloc_obj(*l);
	if (!l)
		return -ENOMEM;
	kref_init(&l->ref);
	l->qsc = qsc;
	l->id = req.listener_id;
	init_waitqueue_head(&l->rcv_wq);
	init_waitqueue_head(&l->rsp_wq);
	ret = qsc_sb_set(qsc, &l->sb, req.ifd_data_fd, req.virt_sb_base,
			 req.sb_size);
	if (ret) {
		dev_warn(qsc->dev, "listener 0x%x: shared buffer refused: %d\n",
			 req.listener_id, ret);
		kfree(l);
		return ret;
	}

	mutex_lock(&qsc->cmd_mutex);
	other = qsc_listener_find(qsc, l->id);
	if (other) {
		qsc_listener_put(other);
		ret = -EEXIST;
		goto err;
	}
	/*
	 * QSEE refuses a listener buffer that is not a whole number of
	 * pages (result -15 for the RPMB listener's 0x6400 and QISL's 500
	 * bytes; 0x5000 and 0x7e000 register). The dma-buf is page-rounded,
	 * so register the rounded length, which stays inside it.
	 */
	args[0] = l->id;
	args[1] = l->sb.buf->phys;
	args[2] = min_t(size_t, PAGE_ALIGN(l->sb.len), l->sb.buf->len);
	ret = qsc_call(QSC_OWNER_QSEE_OS, QSC_SVC_LISTENER, cmd,
		       QCOM_SCM_ARGS(3, QCOM_SCM_VAL, QCOM_SCM_RW, QCOM_SCM_VAL),
		       args, 3, &res);
	if (!ret && (u32)res.result != QSC_RESULT_SUCCESS)
		ret = -EIO;
	if (ret) {
		dev_warn(qsc->dev, "listener 0x%x: register failed: %d (QSEE result 0x%llx, %zu bytes at %pa)\n",
			 l->id, ret, res.result, (size_t)args[2], &l->sb.buf->phys);
		goto err;
	}

	spin_lock(&qsc->lock);
	list_add(&l->node, &qsc->listeners);
	c->listener = l;
	spin_unlock(&qsc->lock);
	mutex_unlock(&qsc->cmd_mutex);

	dev_info(qsc->dev, "listener 0x%x registered\n", l->id);
	return 0;

err:
	mutex_unlock(&qsc->cmd_mutex);
	qsc_listener_put(l);
	return ret;
}

static void qsc_unregister_listener(struct qsc_client *c)
{
	struct qcom_scm_qseecom_raw_res res = {};
	struct qsc_listener *l = c->listener;
	struct qsc *qsc = c->qsc;
	u64 args[1];
	int ret;

	if (!l)
		return;

	/* Wake a waiting request first: it answers TZ with a failure. */
	spin_lock(&qsc->lock);
	l->aborted = true;
	spin_unlock(&qsc->lock);
	wake_up_interruptible(&l->rcv_wq);
	wake_up_interruptible(&l->rsp_wq);

	mutex_lock(&qsc->cmd_mutex);
	spin_lock(&qsc->lock);
	list_del(&l->node);
	c->listener = NULL;
	spin_unlock(&qsc->lock);
	/* VERIFY: deregister arg (id) */
	args[0] = l->id;
	ret = qsc_call(QSC_OWNER_QSEE_OS, QSC_SVC_LISTENER,
		       QSC_CMD_LISTENER_DEREGISTER,
		       QCOM_SCM_ARGS(1, QCOM_SCM_VAL), args, 1, &res);
	if (ret || (u32)res.result != QSC_RESULT_SUCCESS) {
		/*
		 * QSEE may still hold the buffer as this listener's and write
		 * into it. Keep the listener, and with it its reference to the
		 * buffer's dma-buf, until reboot: the client's reference moves
		 * to qsc->kept and is never dropped, so the pages never go back
		 * to the pool. The dma-buf holds a reference to this module
		 * (dma_buf_export() takes one for its exporter), so the module
		 * can't be unloaded, nor the pool destroyed, while it is kept;
		 * unbinding the device is suppressed.
		 */
		list_add(&l->node, &qsc->kept);
		mutex_unlock(&qsc->cmd_mutex);
		dev_err(qsc->dev, "listener 0x%x: deregister failed (%d, QSEE result 0x%llx): its buffer (%zu bytes at %pa) is kept until reboot\n",
			l->id, ret, res.result, l->sb.buf->len, &l->sb.buf->phys);
		return;
	}
	mutex_unlock(&qsc->cmd_mutex);
	dev_info(qsc->dev, "listener 0x%x unregistered\n", l->id);

	qsc_listener_put(l);
}

/* A reference to the client's listener, without the client lock. */
static struct qsc_listener *qsc_client_listener(struct qsc_client *c)
{
	struct qsc_listener *l;

	spin_lock(&c->qsc->lock);
	l = READ_ONCE(c->listener);
	if (l)
		kref_get(&l->ref);
	spin_unlock(&c->qsc->lock);
	return l;
}

static int qsc_receive_req(struct qsc_client *c)
{
	struct qsc_listener *l = qsc_client_listener(c);
	struct qsc *qsc = c->qsc;
	int ret;

	if (!l)
		return -EINVAL;
	ret = wait_event_interruptible(l->rcv_wq,
				       READ_ONCE(l->state) == QSC_L_PENDING ||
				       READ_ONCE(l->aborted));
	if (!ret) {
		spin_lock(&qsc->lock);
		if (l->aborted)
			ret = -ENODEV;
		else if (l->state == QSC_L_PENDING)
			l->state = QSC_L_SERVING;
		else
			ret = -EAGAIN;
		spin_unlock(&qsc->lock);
	}
	qsc_listener_put(l);
	return ret;
}

static int qsc_send_resp(struct qsc_client *c, void __user *uarg, bool modfd,
			 bool wide)
{
	struct qsc_listener *l = qsc_client_listener(c);
	struct qsc_send_modfd_listener_resp req;
	struct qsc *qsc = c->qsc;
	long off;
	int ret = 0;

	if (!l)
		return -EINVAL;

	if (modfd) {
		if (copy_from_user(&req, uarg, sizeof(req))) {
			ret = -EFAULT;
			goto out;
		}
		off = qsc_sb_offset(&l->sb, req.resp_buf_ptr, req.resp_len);
		if (off < 0) {
			ret = off;
			goto out;
		}
		if (READ_ONCE(l->state) != QSC_L_SERVING) {
			ret = -EINVAL;
			goto out;
		}
		ret = qsc_fixup_fds(qsc, l->sb.buf->kva + off, req.resp_len,
				    req.ifd_data, wide, false);
		if (ret)
			goto out;
	}

	spin_lock(&qsc->lock);
	if (l->state == QSC_L_SERVING)
		l->state = QSC_L_DONE;
	else
		ret = -EINVAL;	/* nothing asked, or the request timed out */
	spin_unlock(&qsc->lock);
	if (!ret)
		wake_up_interruptible(&l->rsp_wq);
out:
	qsc_listener_put(l);
	return ret;
}

/* --- File operations --- */

static int qsc_open(struct inode *inode, struct file *file)
{
	struct qsc *qsc = container_of(file->private_data, struct qsc, misc);
	struct qsc_client *c;

	c = kzalloc_obj(*c);
	if (!c)
		return -ENOMEM;
	c->qsc = qsc;
	mutex_init(&c->lock);
	file->private_data = c;
	return 0;
}

static int qsc_release(struct inode *inode, struct file *file)
{
	struct qsc_client *c = file->private_data;

	qsc_unregister_listener(c);
	qsc_unbind_app(c);
	qsc_sb_clear(&c->sb);
	mutex_destroy(&c->lock);
	kfree(c);
	return 0;
}

static long qsc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct qsc_client *c = file->private_data;
	void __user *uarg = (void __user *)arg;
	u32 version;
	long ret;

	/* These wait for TZ or answer it; they must not take the client lock. */
	switch (cmd) {
	case QSC_IOCTL_RECEIVE_REQ:
		return qsc_receive_req(c);
	case QSC_IOCTL_SEND_RESP_REQ:
		return qsc_send_resp(c, NULL, false, false);
	case QSC_IOCTL_SEND_MODFD_RESP:
		return qsc_send_resp(c, uarg, true, false);
	case QSC_IOCTL_SEND_MODFD_RESP_64:
		return qsc_send_resp(c, uarg, true, true);
	case QSEECOM_COMPAT_IOCTL_ALLOC:
		return qsc_alloc(c->qsc, uarg);
	case QSC_IOCTL_GET_QSEOS_VERSION_REQ:
		/*
		 * Not QSEE's own version (0x1400000 on SM8150) but downstream's
		 * QSEOS_VERSION_13 (0x13) or _14 (0x14), which libQSEEComAPI
		 * compares against: with anything but 0x14 its
		 * QSEECom_receive_req() returns success without copying the
		 * request out of the shared buffer, so every listener saw an
		 * all-zero request (OxygenOS 10.0.14's library, disassembled).
		 * QSEE that new is QSEOS 14 to downstream's version check.
		 */
		ret = qcom_scm_qseecom_get_version(&version);
		if (ret)
			return ret;
		return put_user(QSC_QSEOS_VERSION_14, (__u32 __user *)uarg);
	case QSC_IOCTL_PERF_ENABLE_REQ:
	case QSC_IOCTL_PERF_DISABLE_REQ:
	case QSC_IOCTL_SET_BUS_SCALING_REQ:
		return 0;	/* no crypto clock or bus votes here */
	}

	mutex_lock(&c->lock);
	switch (cmd) {
	case QSC_IOCTL_SEND_CMD_REQ:
		ret = qsc_send_cmd(c, uarg);
		break;
	case QSC_IOCTL_SEND_MODFD_CMD_REQ:
		ret = qsc_send_modfd_cmd(c, uarg, false);
		break;
	case QSC_IOCTL_SEND_MODFD_CMD_64_REQ:
		ret = qsc_send_modfd_cmd(c, uarg, true);
		break;
	case QSC_IOCTL_REGISTER_LISTENER_REQ:
		ret = qsc_register_listener(c, uarg);
		break;
	case QSC_IOCTL_UNREGISTER_LISTENER_REQ:
		ret = c->listener ? 0 : -EINVAL;
		qsc_unregister_listener(c);
		break;
	case QSC_IOCTL_LOAD_APP_REQ:
		ret = qsc_load_app(c, uarg);
		break;
	case QSC_IOCTL_UNLOAD_APP_REQ:
		ret = c->app ? 0 : -EINVAL;
		qsc_unbind_app(c);
		break;
	case QSC_IOCTL_APP_LOADED_QUERY_REQ:
		ret = qsc_app_loaded_query(c, uarg);
		break;
	case QSC_IOCTL_SET_MEM_PARAM_REQ:
		ret = qsc_set_mem_param(c, uarg);
		break;
	default:
		dev_info_ratelimited(c->qsc->dev,
				     "unsupported ioctl type 0x%x nr %u (%s)\n",
				     _IOC_TYPE(cmd), _IOC_NR(cmd), current->comm);
		ret = -ENOTTY;
		break;
	}
	mutex_unlock(&c->lock);
	return ret;
}

static const struct file_operations qsc_fops = {
	.owner = THIS_MODULE,
	.open = qsc_open,
	.release = qsc_release,
	/* No compat_ioctl: the layouts are those of 64-bit userspace. */
	.unlocked_ioctl = qsc_ioctl,
};

/* --- Platform device --- */

static void qsc_misc_deregister(void *data)
{
	misc_deregister(data);
}

static int qsc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource res;
	u32 version;
	struct qsc *qsc;
	int ret;

	if (!qcom_scm_is_available())
		return -EPROBE_DEFER;

	ret = qcom_scm_qseecom_get_version(&version);
	if (ret)
		return dev_err_probe(dev, ret, "no QSEECOM in this firmware\n");

	qsc = devm_kzalloc(dev, sizeof(*qsc), GFP_KERNEL);
	if (!qsc)
		return -ENOMEM;
	qsc->dev = dev;
	mutex_init(&qsc->cmd_mutex);
	spin_lock_init(&qsc->lock);
	INIT_LIST_HEAD(&qsc->apps);
	INIT_LIST_HEAD(&qsc->listeners);
	INIT_LIST_HEAD(&qsc->kept);

	ret = of_reserved_mem_region_to_resource(dev->of_node, 0, &res);
	if (ret)
		return dev_err_probe(dev, ret, "no memory-region\n");
	qsc->base = res.start;
	qsc->size = resource_size(&res);
	if (!PAGE_ALIGNED(qsc->base) || !PAGE_ALIGNED(qsc->size))
		return dev_err_probe(dev, -EINVAL, "carve-out is not page aligned\n");

	qsc->kva = devm_memremap(dev, qsc->base, qsc->size, MEMREMAP_WC);
	if (IS_ERR(qsc->kva))
		return dev_err_probe(dev, PTR_ERR(qsc->kva),
				     "failed to map the carve-out\n");

	qsc->pool = devm_gen_pool_create(dev, PAGE_SHIFT, -1, "qseecom");
	if (IS_ERR(qsc->pool))
		return PTR_ERR(qsc->pool);
	ret = gen_pool_add_virt(qsc->pool, (unsigned long)qsc->kva, qsc->base,
				qsc->size, -1);
	if (ret)
		return ret;

	qsc->misc.minor = MISC_DYNAMIC_MINOR;
	qsc->misc.name = "qseecom";
	qsc->misc.mode = 0600;
	qsc->misc.fops = &qsc_fops;
	qsc->misc.parent = dev;
	ret = misc_register(&qsc->misc);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, qsc_misc_deregister, &qsc->misc);
	if (ret)
		return ret;

	dev_info(dev, "QSEECOM 0x%x, %zu KiB shared memory at %pa; apps \"%s\" (signed names \"%s\"), listeners \"%s\"%s\n",
		 version, qsc->size / SZ_1K, &qsc->base, app_allowlist ?: "",
		 app_signed_names ?: "", listener_allowlist ?: "",
		 allow_rpmb ? ", RPMB allowed" : "");
	return 0;
}

static const struct of_device_id qsc_of_match[] = {
	{ .compatible = "omamobile,qseecom-compat" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, qsc_of_match);

static struct platform_driver qsc_driver = {
	.probe = qsc_probe,
	.driver = {
		.name = "qcom-qseecom-compat",
		.of_match_table = qsc_of_match,
		/* Open files and exported dma-bufs outlive an unbind. */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(qsc_driver);

MODULE_DESCRIPTION("Downstream-compatible /dev/qseecom for a vendor HAL");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("DMA_BUF");
