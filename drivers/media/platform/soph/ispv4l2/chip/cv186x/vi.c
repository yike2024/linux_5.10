#include <vi.h>
#include <proc/vi_dbg_proc.h>
#include <proc/vi_proc.h>
#include <proc/vi_isp_proc.h>
#include <vip/vi_perf_chk.h>
#include <vi_ext.h>

#include <linux/cvi_buffer.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>
#if (KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE)
#include <linux/dma-map-ops.h>
#endif

#define VI_MEM_BM_ION

#ifdef VI_MEM_BM_ION
#include "ion/ion.h"
#include "ion/cvitek/cvitek_ion_alloc.h"
#endif

#include <linux/cif_uapi.h>
#include <linux/sns_v4l2_uapi.h>
#include "vi_sys.h"
#include "cmdq.h"
#include "isp_dev.h"

/*******************************************************
 *  MACRO defines
 ******************************************************/
#define DMA_SETUP_2(raw_num, id)					\
	do {								\
		bufaddr = _mempool_get_addr();				\
		ispblk_dma_setaddr(ictx, id, bufaddr);			\
		bufsize = ispblk_dma_buf_get_size(ictx, raw_num, id);	\
		_mempool_pop(bufsize);					\
	} while (0)

#define DMA_SETUP(raw_num, id)						\
	do {								\
		bufaddr = _mempool_get_addr();				\
		bufsize = ispblk_dma_config(ictx, raw_num, id, bufaddr);\
		_mempool_pop(bufsize);					\
	} while (0)

#define VI_PROFILE
#define VI_MAX_LIST_NUM	(0x80)

/* In practical application, it is necessary to drop the frame of AE convergence process.
 * But in vi-vpss online & vpss-vc sbm scenario, there is no way to drop the frame.
 * Use cover with black to avoid this problem.
 */
#ifndef PORTING_TEST
// #define COVER_WITH_BLACK
#endif
/*******************************************************
 *  Global variables
 ******************************************************/
u32 vi_log_lv = VI_ERR | VI_WARN | VI_NOTICE | VI_INFO | VI_DBG;
module_param(vi_log_lv, int, 0644);

#ifdef PORTING_TEST //test only
static bool usr_trigger;
int stop_stream_en;
static int count;
module_param(stop_stream_en, int, 0644);
#endif

struct cvi_vi_ctx *gViCtx;
static struct cvi_vi_dev *gvidev;
struct cvi_overflow_info *gOverflowInfo;
extern uint8_t g_w_bit[ISP_PRERAW_MAX], g_h_bit[ISP_PRERAW_MAX];
static uintptr_t sys_base_address;

const char * const clk_sys_name[] = {
	"clk_sys_0", "clk_sys_1", "clk_sys_2",
	"clk_sys_3", "clk_sys_4", "clk_sys_5"
};
const char * const clk_isp_name[] = {
	"clk_csi_be", "clk_raw", "clk_isp_top"
};
const char * const clk_mac_name[] = {
	"clk_csi_mac0", "clk_csi_mac1", "clk_csi_mac2",
	"clk_csi_mac3", "clk_csi_mac4", "clk_csi_mac5",
	"clk_csi_mac_vi0", "clk_csi_mac_vi1"
};

#define VIP_ALIGNMENT 0x40
#define VI_CMDQ_BUF_SIZE (0x20000)

#define ISP_MAX_WIDTH  7680
#define ISP_MAX_HEIGHT 4320
#define ISP_DEFAULT_WIDTH  1920
#define ISP_DEFAULT_HEIGHT 1080

enum {
	ISP_OUT_FMT_NV21,
	ISP_OUT_FMT_YUYV,
	ISP_OUT_FMT_MAX,
};

static struct cvi_vip_fmt cvi_vip_formats[ISP_OUT_FMT_MAX] = {
	{
	.fourcc      = V4L2_PIX_FMT_NV21,
	.bit_depth   = { 8, 8, 0 },
	.buffers     = 1,
	.plane_sub_h = 1,
	.plane_sub_v = 1,
	},
	{
	.fourcc      = V4L2_PIX_FMT_YUYV,
	.bit_depth   = { 16 },
	.buffers     = 1,
	.plane_sub_h = 1,
	.plane_sub_v = 1,
	},
};

/*******************************************************git
 *  Internal APIs
 ******************************************************/

#if (KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE)
static void legacy_timer_emu_func(struct timer_list *t)
{
	struct legacy_timer_emu *lt = from_timer(lt, t, t);

	lt->function(lt->data);
}
#endif //(KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE)

static void check_size(u32 *width, u32 *height)
{
	if (*width > ISP_MAX_WIDTH || *height > ISP_MAX_HEIGHT
		|| *width == 0 || *height == 0) {
		vi_pr(VI_INFO, "size(%u*%u) is invalid! use default(640*480)\n",
			*width, *height);
		*width = ISP_DEFAULT_WIDTH;
		*height = ISP_DEFAULT_HEIGHT;
	}
}

static int vi_snsr_i2c_write(struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	struct isp_i2c_data *i2c)
{
	struct cvi_isp_device *dev =
		container_of(vdev, struct cvi_isp_device, vi_dev);
	uint8_t chn_id = vi_get_dev_num_by_raw(&vdev->ctx, raw_num);
	struct v4l2_subdev *sensor_sd = dev->sensors[chn_id].sd;
	struct i2c_client *client = v4l2_get_subdevdata(sensor_sd);
	struct i2c_adapter *adap;
	struct i2c_msg msg;
	u8 tx[4];
	int ret, retry = 5, idx = 0;

	if (!client) {
		vi_pr(VI_ERR, "NULL i2c_client dev:%d!\n", raw_num);
		return -1;
	}

	/* Get i2c adapter */
	adap = client->adapter;

	/* Config reg addr */
	if (i2c->addr_bytes == 1) {
		tx[idx++] = i2c->reg_addr & 0xff;
	} else {
		tx[idx++] = i2c->reg_addr >> 8;
		tx[idx++] = i2c->reg_addr & 0xff;
	}

	/* Config data */
	if (i2c->data_bytes == 1) {
		tx[idx++] = i2c->data & 0xff;
	} else {
		tx[idx++] = i2c->data >> 8;
		tx[idx++] = i2c->data & 0xff;
	}

	/* send the i2c with retry */
	msg.addr = client->addr;
	msg.buf = tx;
	msg.len = idx;
	msg.flags = 0;

	while (retry--) {
		ret = i2c_transfer(adap, &msg, 1);
		if (ret == 1) {
			dev_dbg(&client->dev, "0x%x = 0x%x\n",
				i2c->reg_addr, i2c->data);
			break;
		} else if (ret == -EAGAIN) {
			dev_dbg(&client->dev, "retry 0x%x = 0x%x\n",
					i2c->reg_addr, i2c->data);
		} else {
			dev_err(&client->dev, "fail to send 0x%x, 0x%x, %d\n",
						i2c->reg_addr, i2c->data, ret);
		}
	}

	return ret == 1 ? 0 : -EIO;
}

void sys_set_reset_vi(union vi_sys_reset reset)
{
	_reg_write(sys_base_address + VI_SYS_VI_RESETS, reset.raw);
}

union vi_sys_reset sys_get_reset_vi(void)
{
	union vi_sys_reset reset;

	reset.raw = _reg_read(sys_base_address + VI_SYS_VI_RESETS);
	return reset;
}

void v4l2_sys_reg_write_mask(uintptr_t address, u32 mask, u32 data)
{
	u32 value;
	uintptr_t addr = sys_base_address + address;

	value = readl_relaxed((void __iomem *)addr) & ~mask;
	value |= (data & mask);
	writel(value, (void __iomem *)addr);
}

#ifdef VI_MEM_BM_ION
#define MAX_VB2_BUF_NUM 72
static struct mem_mapping vb2_buf_meminfo[MAX_VB2_BUF_NUM];
static int vb2_buf_mem_index;

void *v4l2_vb_dma_alloc(struct device *dev, size_t size,
		dma_addr_t *dma_handle, gfp_t gfp, unsigned long attrs)
{
	int32_t dmabuf_fd = 0, ret = 0;
	struct dma_buf *dmabuf;
	struct ion_buffer *ionbuf;
	void *vmap_addr = NULL;
	int index = 0;
	struct mem_mapping *mem_info = NULL;
	char name[16];
	uint8_t *owner_name = NULL;

	mutex_lock(&gvidev->v4l2_vb_lock);
	index = vb2_buf_mem_index++;
	mutex_unlock(&gvidev->v4l2_vb_lock);

	if (index > MAX_VB2_BUF_NUM -1) {
		vi_pr(VI_ERR, "vb2 buffer requset outof(%d)\n", MAX_VB2_BUF_NUM);
		return NULL;
	}
	sprintf(name, "v4l2_vb_%02d", index);

	mem_info = &vb2_buf_meminfo[index];

	//vpp heap
	dmabuf_fd = bm_ion_alloc(0x1, size, 1);
	if (dmabuf_fd < 0) {
		vi_pr(VI_INFO, "bm_ion_alloc len=0x%lx failed\n", size);
		return NULL;
	}

	dmabuf = dma_buf_get(dmabuf_fd);
	if (!dmabuf) {
		vi_pr(VI_INFO, "allocated get dmabuf failed\n");
		return NULL;
	}

	ionbuf = (struct ion_buffer *)dmabuf->priv;
	owner_name = vmalloc(64);
	strncpy(owner_name, name, sizeof(name));
	ionbuf->name = owner_name;

	ret = dma_buf_begin_cpu_access(dmabuf, DMA_TO_DEVICE);
	if (ret < 0) {
		vi_pr(VI_INFO, "cvi_ion_alloc() dma_buf_begin_cpu_access failed\n");
		dma_buf_put(dmabuf);
		return NULL;
	}

	vmap_addr = ionbuf->vaddr;
	if (IS_ERR(vmap_addr)) {
		ret = -EINVAL;
		return NULL;
	}

	*dma_handle = ionbuf->paddr;

	mem_info->dmabuf = (void *)dmabuf;
	mem_info->dmabuf_fd = dmabuf_fd;
	mem_info->vir_addr = vmap_addr;
	mem_info->phy_addr = ionbuf->paddr;
	mem_info->fd_pid = current->pid;
	mem_info->size = size;
	mem_info->files = current->files;

	vi_pr(VI_INFO, "dma_fd[%d]:%d alloc size:%ld, phy_addr:0x%llx\n",
		index, dmabuf_fd, size, *dma_handle);

	return vmap_addr;
}

void v4l2_vb_dma_free(struct device *dev, size_t size, void *vaddr,
			dma_addr_t dma_handle, unsigned long attrs)
{
	struct mem_mapping *mem_info = NULL;
	struct dma_buf *dmabuf;
	struct ion_buffer *ionbuf;
	int32_t dmabuf_fd;
	int index = 0;
	int i;

	for (i = 0; i < MAX_VB2_BUF_NUM; i++) {
		mem_info = &vb2_buf_meminfo[i];
		if (mem_info->phy_addr == dma_handle) {
			dmabuf_fd = mem_info->dmabuf_fd;
			dmabuf = mem_info->dmabuf;
			ionbuf = (struct ion_buffer *)dmabuf->priv;
			index = i;
			break;
		}
	}

	if (i == MAX_VB2_BUF_NUM) {
		vi_pr(VI_ERR, "no mem info match! (0x%llx)\n", dma_handle);
		return;
	}

	if (ionbuf->name) {
		vfree(ionbuf->name);
		ionbuf->name = NULL;
	}

	dma_buf_end_cpu_access(dmabuf, DMA_TO_DEVICE);
	dma_buf_put(dmabuf);

	__close_fd(mem_info->files, mem_info->dmabuf_fd);

	memset(mem_info, 0, sizeof(struct mem_mapping));

	mutex_lock(&gvidev->v4l2_vb_lock);
	vb2_buf_mem_index--;
	vi_pr(VI_INFO, "g_index:%d, dma_fd[%d]:%d\n", vb2_buf_mem_index, index, dmabuf_fd);
	mutex_unlock(&gvidev->v4l2_vb_lock);
}

static int v4l2_vb_dma_mmap(struct device *dev, struct vm_area_struct *vma,
			void *cpu_addr, dma_addr_t dma_addr, size_t size, unsigned long attrs)
{
	unsigned long vm_start = vma->vm_start;
	unsigned int vm_size = vma->vm_end - vma->vm_start;
	void *pos = phys_to_virt(dma_addr);

	vi_pr(VI_DBG, "mmap size(%ld) vm_size(%d) phys(0x%llx)\n", size, vm_size, dma_addr);

	while (vm_size > 0) {
		if (remap_pfn_range(vma, vm_start, virt_to_pfn(pos), PAGE_SIZE, vma->vm_page_prot))
			return -EAGAIN;
		vm_start += PAGE_SIZE;
		pos += PAGE_SIZE;
		vm_size -= PAGE_SIZE;
	}

	return 0;
}
#endif

int32_t vi_ion_cache_invalidate(uint64_t addr_p, void *addr_v, uint32_t u32Len)
{
#if (KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE) && defined(__riscv)
	arch_sync_dma_for_device(addr_p, u32Len, DMA_FROM_DEVICE);
#else
	__dma_map_area(phys_to_virt(addr_p), u32Len, DMA_FROM_DEVICE);
#endif

	/*	*/
	smp_mb();
	return 0;
}

int32_t vi_ion_cache_flush(uint64_t addr_p, void *addr_v, uint32_t u32Len)
{
#if (KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE) && defined(__riscv)
	arch_sync_dma_for_device(addr_p, u32Len, DMA_TO_DEVICE);
#else
	__dma_map_area(phys_to_virt(addr_p), u32Len, DMA_TO_DEVICE);
#endif

	/*	*/
	smp_mb();
	return 0;
}

static int32_t _vi_dma_alloc(struct cvi_vi_dev *vdev,
	struct mem_mapping *mem_info, uint64_t *addr_p,
	uint32_t u32Len, uint32_t is_cached, uint8_t *name)
{
#ifdef VI_MEM_BM_ION
	int32_t dmabuf_fd = 0, ret = 0;
	struct dma_buf *dmabuf;
	struct ion_buffer *ionbuf;
	void *vmap_addr = NULL;
	uint32_t size = PAGE_ALIGN(u32Len);
	uint8_t *owner_name = NULL;
	uint8_t name_size = 32;

	//vpp heap
	dmabuf_fd = bm_ion_alloc(0x1, size, is_cached);
	if (dmabuf_fd < 0) {
		vi_pr(VI_INFO, "bm_ion_alloc len=0x%x failed\n", size);
		return -ENOMEM;
	}

	dmabuf = dma_buf_get(dmabuf_fd);
	if (!dmabuf) {
		vi_pr(VI_INFO, "allocated get dmabuf failed\n");
		return -ENOMEM;
	}

	ionbuf = (struct ion_buffer *)dmabuf->priv;
	owner_name = vmalloc(name_size);
	if (name)
		strncpy(owner_name, name, name_size);
	else
		strncpy(owner_name, "anonymous", name_size);

	ionbuf->name = owner_name;

	ret = dma_buf_begin_cpu_access(dmabuf, DMA_TO_DEVICE);
	if (ret < 0) {
		vi_pr(VI_INFO, "cvi_ion_alloc() dma_buf_begin_cpu_access failed\n");
		dma_buf_put(dmabuf);
		return ret;
	}

	vmap_addr = ionbuf->vaddr;
	if (IS_ERR(vmap_addr)) {
		ret = -EINVAL;
		return ret;
	}

	*addr_p = ionbuf->paddr;

	mem_info->dmabuf = (void *)dmabuf;
	mem_info->dmabuf_fd = dmabuf_fd;
	mem_info->vir_addr = vmap_addr;
	mem_info->phy_addr = ionbuf->paddr;
	mem_info->fd_pid = current->pid;
	mem_info->size = size;
	mem_info->files = current->files;

#else
	dma_addr_t dma_handle;
	void *addr_v = NULL;
	uint32_t size = PAGE_ALIGN(u32Len);

	if (u32Len <= 0) {
		vi_pr(VI_ERR, "illegal alloc size(%d) !!\n", u32Len);
		return -1;
	}

	vi_pr(VI_INFO, "size:%d, align:%d\n", u32Len, size);

	addr_v = dma_alloc_coherent(vdev->dev, size,
				&dma_handle, GFP_DMA | GFP_KERNEL);

	if (addr_v == NULL) {
		pr_err("dma_alloc_coherent allocated len=%d failed\n", size);
		return -ENOMEM;
	}

	*addr_p = dma_handle;

	mem_info->dmabuf = NULL;
	mem_info->dmabuf_fd = -1;
	mem_info->vir_addr = addr_v;
	mem_info->phy_addr = *addr_p;
	mem_info->fd_pid = current->pid;
	mem_info->size = size;
#endif
	vi_pr(VI_INFO, "phy_addr=0x%llx\n", mem_info->phy_addr);

	return 0;
}

static void _vi_dma_free(struct cvi_vi_dev *videv, struct mem_mapping *mem_info)
{
#ifdef VI_MEM_BM_ION
	struct dma_buf *dmabuf;
	struct ion_buffer *ionbuf;

	if (!mem_info->phy_addr) {
		vi_pr(VI_DBG, "NULL mem to free!\n");
		return;
	}

	dmabuf = (struct dma_buf *)(mem_info->dmabuf);
	ionbuf = (struct ion_buffer *)dmabuf->priv;

	if (ionbuf->name) {
		vfree(ionbuf->name);
		ionbuf->name = NULL;
	}

	dma_buf_end_cpu_access(dmabuf, DMA_TO_DEVICE);
	dma_buf_put(dmabuf);

	__close_fd(mem_info->files, mem_info->dmabuf_fd);
#else
	if (mem_info->phy_addr && mem_info->size)
		dma_free_coherent(videv->dev, mem_info->size,
				mem_info->vir_addr, mem_info->phy_addr);
#endif
	vi_pr(VI_INFO, "phy_addr=0x%llx\n", mem_info->phy_addr);

	memset(mem_info, 0, sizeof(struct mem_mapping));
}

/**
 * _mempool_ion_alloc -alloc memory for vi dma
 *
 */
static int vi_mempool_alloc_ion(struct cvi_vi_dev *videv)
{
	int32_t ret = 0;
	uint64_t addr_p = 0;
	u32 size = 0;

	vi_pr(VI_INFO, "+\n");

	size = get_dma_buf_size(videv);

	ret = _vi_dma_alloc(videv, &vi_dma_buf_info, &addr_p,
					size, 1, "VI_DMA_BUF");
	if (ret < 0) {
		vi_pr(VI_ERR, "dma alloc failed\n");
		return ret;
	}

	isp_mempool.base = addr_p;
	isp_mempool.size = size;

	vi_pr(VI_INFO, "-\n");

	return ret;
}

static void vi_mempool_free_ion(struct cvi_vi_dev *videv)
{
	_vi_dma_free(videv, &vi_dma_buf_info);
	isp_mempool.base = 0;
	isp_mempool.size = 0;
}

/**
 * _mempool_reset - reset the byteused and assigned buffer for each dma
 *
 */
static void _vi_mempool_reset(void)
{
	u8 i = 0;

	isp_mempool.byteused = 0;

	memset(isp_bufpool, 0x0, (sizeof(struct _membuf) * ISP_PRERAW_MAX));

	for (i = 0; i < ISP_PRERAW_MAX; i++) {
		spin_lock_init(&isp_bufpool[i].pre_fe_sts_lock);
		spin_lock_init(&isp_bufpool[i].pre_be_sts_lock);
		spin_lock_init(&isp_bufpool[i].post_sts_lock);
	}
}

/**
 * _mempool_get_addr - get mempool's latest address.
 *
 * @return: the latest address of the mempool.
 */
static uint64_t _mempool_get_addr(void)
{
	return isp_mempool.base + isp_mempool.byteused;
}

/**
 * _mempool_pop - acquire a buffer-space from mempool.
 *
 * @param size: the space acquired.
 * @return: negative if no enough space; o/w, the address of the buffer needed.
 */
static int64_t _mempool_pop(uint32_t size)
{
	int64_t addr;

	size = VI_ALIGN(size);

	if ((isp_mempool.byteused + size) > isp_mempool.size) {
		vi_pr(VI_ERR, "reserved_memory(0x%x) is not enough. byteused(0x%x) alloc_size(0x%x)\n",
				isp_mempool.size, isp_mempool.byteused, size);
		return -EINVAL;
	}

	addr = isp_mempool.base + isp_mempool.byteused;
	isp_mempool.byteused += size;

	return addr;
}

static int copy_vb2buf_to_dump(struct vb2_buffer *vb2_buf)
{
	uint64_t src_addr = 0;
	void *src_vir = NULL;
	void *dst_vir = NULL;

	if (!vb2_buf) {
		vi_pr(VI_ERR, "NULL vb2_buffer!\n");
		return -1;
	}

	if (!yuv_dump_vb_info.phy_addr) {
		vi_pr(VI_ERR, "yuv dump has not alloc yet!\n");
		return -1;
	}

	src_addr = vb2_dma_contig_plane_dma_addr(vb2_buf, 0);
	src_vir = phys_to_virt(src_addr);
	dst_vir = phys_to_virt(yuv_dump_vb_info.phy_addr);

	memcpy(dst_vir, src_vir, yuv_dump_vb_info.size);

	return 0;
}

static int _vi_set_dev_bind_info(struct cvi_vi_dev *videv, VI_DEV ViDev, u8 mac_num)
{
	struct isp_ctx *ctx = &videv->ctx;
	u8 raw_num = 0;
	int ret = -1;

	if (mac_num >= VI_MAX_DEV_NUM) {
		vi_pr(VI_ERR, "invalid mac(%d)\n", mac_num);
		return ret;
	}

	raw_num = mac_num;

	if (ctx->isp_bind_info[raw_num].is_bind) {
		vi_pr(VI_ERR, "mac(%d) is binded before\n", mac_num);
		return ret;
	}

	if ((gViCtx->devAttr[ViDev].enInputDataType == VI_DATA_TYPE_YUV ||
	     gViCtx->devAttr[ViDev].enInputDataType == VI_DATA_TYPE_YUV_EARLY) &&
	      (gViCtx->devAttr[ViDev].enWorkMode > 0 &&
	       gViCtx->devAttr[ViDev].enIntfMode >= VI_MODE_BT656 &&
	       gViCtx->devAttr[ViDev].enIntfMode <= VI_MODE_BT1120_INTERLEAVED)) { // bt_demux
		if (!(raw_num >= ISP_PRERAW_LITE0 && raw_num <= ISP_PRERAW_LITE1)) {
			vi_pr(VI_ERR, "invalid bind mac(%d) from vi dev(%d)\n", mac_num, ViDev);
			return ret;
		}
	} else {
		if (raw_num > ISP_PRERAW5) {
			vi_pr(VI_ERR, "invalid bind mac(%d) from vi dev(%d)\n", mac_num, ViDev);
			return ret;
		}
	}

	vi_pr(VI_INFO, "dev(%d) bind to mac(%d)\n", ViDev, mac_num);
	ctx->isp_bind_info[raw_num].is_bind = true;
	ctx->isp_bind_info[raw_num].bind_dev_num = ViDev;
	ret = 0;

	return ret;
}

static inline CVI_S32 check_vi_pipe_valid(VI_PIPE pipe)
{
	if (pipe < 0 || pipe > (VI_MAX_PIPE_NUM - 1)) {
		vi_pr(VI_ERR, "pipe num expect 0~%d, but now %d Caller is %p\n",
			VI_MAX_PIPE_NUM - 1, pipe, __builtin_return_address(0));
		return CVI_FAILURE;
	}

	return CVI_SUCCESS;
}

CVI_S32 vi_get_pipe_frame(struct cvi_vi_dev *videv, VI_PIPE ViPipe,
	VIDEO_FRAME_INFO_S *pstFrameInfo, CVI_S32 s32MilliSec)
{
	CVI_S32 ret;
	struct cvi_vip_isp_raw_blk dump[2];
	CVI_U32 u32BlkSize, dev_frm_w, dev_frm_h, frm_w, frm_h, raw_num;
	CVI_U64 u64PhyAddr = 0;
	int dev_id = ViPipe, frm_num = 1, i = 0;
	struct vi_rect rawdump_crop;
	struct isp_ctx *ctx = &videv->ctx;

	ret = check_vi_pipe_valid(ViPipe);
	if (ret != CVI_SUCCESS)
		return ret;

	if (atomic_read(&videv->isp_streamoff) == 1) {
		vi_pr(VI_ERR, "StartStream first\n");
		return CVI_FAILURE;
	}

	if (gViCtx->dumpAttr[ViPipe].bEnable == CVI_FALSE) {
		vi_pr(VI_ERR, "SetPipeDumpAttr first\n");
		return CVI_FAILURE;
	}

	if (gViCtx->dumpAttr[ViPipe].enDumpType == VI_DUMP_TYPE_YUV ||
		gViCtx->dumpAttr[ViPipe].enDumpType == VI_DUMP_TYPE_IR) {
		vi_pr(VI_ERR, "IR or yuv raw dump is not supported.\n");
		return CVI_FAILURE;
	}

	memset(dump, 0, sizeof(dump));
	raw_num = vi_get_raw_num_by_dev(ctx, ViPipe);
	dump[0].raw_dump.raw_num = raw_num;

	dev_frm_w = ctx->isp_pipe_cfg[raw_num].crop.w;
	dev_frm_h = ctx->isp_pipe_cfg[raw_num].crop.h;

	memset(&rawdump_crop, 0, sizeof(rawdump_crop));
	if ((pstFrameInfo[0].stVFrame.s16OffsetTop != 0) ||
		(pstFrameInfo[0].stVFrame.s16OffsetBottom != 0) ||
		(pstFrameInfo[0].stVFrame.s16OffsetLeft != 0) ||
		(pstFrameInfo[0].stVFrame.s16OffsetRight != 0)) {
		rawdump_crop.x = pstFrameInfo[0].stVFrame.s16OffsetLeft;
		rawdump_crop.y = pstFrameInfo[0].stVFrame.s16OffsetTop;
		rawdump_crop.w = dev_frm_w - rawdump_crop.x - pstFrameInfo[0].stVFrame.s16OffsetRight;
		rawdump_crop.h = dev_frm_h - rawdump_crop.y - pstFrameInfo[0].stVFrame.s16OffsetBottom;

		vi_pr(VI_INFO, "set rawdump crop x(%d), y(%d), w(%d), h(%d)\n",
			rawdump_crop.x, rawdump_crop.y, rawdump_crop.w, rawdump_crop.h);

		frm_w = rawdump_crop.w;
		frm_h = rawdump_crop.h;
	} else {
		frm_w = dev_frm_w;
		frm_h = dev_frm_h;
	}
	ctx->isp_pipe_cfg[raw_num].rawdump_crop = rawdump_crop;
	ctx->isp_pipe_cfg[raw_num].rawdump_crop_se = rawdump_crop;

	u32BlkSize = VI_GetRawBufferSize(frm_w, frm_h,
					PIXEL_FORMAT_RGB_BAYER_12BPP,
					gViCtx->pipeAttr[ViPipe].enCompressMode,
					16, gViCtx->isTile);
	vi_pr(VI_INFO, "frm_w(%d), frm_h(%d), u32BlkSize: %d\n",
		frm_w, frm_h, u32BlkSize);

	ret = _vi_dma_alloc(videv, &raw_dump_vb_info[0],
		&u64PhyAddr, u32BlkSize, 1, "VI_RAW_DUMP_L");
	if (ret < 0) {
		vi_pr(VI_ERR, "raw dump dma alloc failed!\n");
		return ret;
	}
	dump[0].raw_dump.phy_addr = u64PhyAddr;

	if (ctx->isp_pipe_cfg[raw_num].is_hdr_on) {
		frm_num = 2;
		ret = _vi_dma_alloc(videv, &raw_dump_vb_info[1],
			&u64PhyAddr, u32BlkSize, 1, "VI_RAW_DUMP_S");
		if (ret < 0) {
			vi_pr(VI_ERR, "raw dump dma alloc failed!\n");
			return ret;
		}
		dump[1].raw_dump.phy_addr = u64PhyAddr;
	}

	if (s32MilliSec >= 0)
		dump[0].time_out = dump[1].time_out = s32MilliSec;

	ret = isp_raw_dump(videv, &dump[0]);
	if (ret != CVI_SUCCESS) {
		vi_pr(VI_ERR, "_isp_raw_dump fail\n");
		return CVI_FAILURE;
	}

	if (dump[0].is_timeout) {
		vi_pr(VI_ERR, "Get pipe frame time out(%d)\n", s32MilliSec);
		goto free_dma;
	}

	if (dump[0].is_sig_int) {
		vi_pr(VI_ERR, "Get pipe frame signal interrupt\n");
		goto free_dma;
	}


	for (; i < frm_num; i++) {
		pstFrameInfo[i].stVFrame.u64PhyAddr[0]  = dump[i].raw_dump.phy_addr;
		pstFrameInfo[i].stVFrame.u32Length[0]   = u32BlkSize;
		pstFrameInfo[i].stVFrame.enBayerFormat  = gViCtx->devAttr[dev_id].enBayerFormat;
		pstFrameInfo[i].stVFrame.enCompressMode = gViCtx->pipeAttr[dev_id].enCompressMode;
		pstFrameInfo[i].stVFrame.u32Width       = dump[i].src_w;
		pstFrameInfo[i].stVFrame.u32Height      = dump[i].src_h;
		pstFrameInfo[i].stVFrame.s16OffsetLeft  = dump[i].crop_x;
		pstFrameInfo[i].stVFrame.s16OffsetTop   = dump[i].crop_y;
		pstFrameInfo[i].stVFrame.u32TimeRef     = dump[i].frm_num;
	}

	vi_pr(VI_INFO, "raw dump success\n");

	return CVI_SUCCESS;

free_dma:
	_vi_dma_free(videv, &raw_dump_vb_info[0]);
	if (ctx->isp_pipe_cfg[raw_num].is_hdr_on)
		_vi_dma_free(videv, &raw_dump_vb_info[1]);
	return CVI_FAILURE;
}

CVI_S32 vi_release_pipe_frame(struct cvi_vi_dev *videv,
	const enum cvi_isp_raw raw_num)
{
	u8 i = 0;
	CVI_S32 ret = CVI_SUCCESS;

	ret = check_vi_pipe_valid(raw_num);
	if (ret != CVI_SUCCESS)
		return ret;

	for (i = 0; i < ISP_PRERAW_MAX; i++) {
		free_isp_byr(i);
	}

	_vi_dma_free(videv, &raw_dump_vb_info[0]);
	if (videv->ctx.isp_pipe_cfg[raw_num].is_hdr_on)
		_vi_dma_free(videv, &raw_dump_vb_info[1]);

	return CVI_SUCCESS;
}

CVI_S32 vi_get_chn_frame(struct cvi_vi_dev *videv,
	VI_PIPE ViPipe, VIDEO_FRAME_INFO_S *pstFrameInfo, CVI_S32 s32MilliSec)
{
	struct isp_ctx *ctx = &videv->ctx;
	u32 width = gViCtx->chnStatus[ViPipe].stSize.u32Width;
	u32 height = gViCtx->chnStatus[ViPipe].stSize.u32Height;
	u8	raw_num = vi_get_raw_num_by_dev(ctx, ViPipe);
	u32 size = videv->pipe[raw_num].sizeimage[0] + videv->pipe[raw_num].sizeimage[1];
	u64 u64PhyAddr = 0;
	CVI_S32 ret = 0;
	struct timespec64 ts;

	ktime_get_ts64(&ts);

	memset(pstFrameInfo, 0, sizeof(*pstFrameInfo));
	pstFrameInfo->stVFrame.enPixelFormat = PIXEL_FORMAT_NV21;
	pstFrameInfo->stVFrame.u32Width = width;
	pstFrameInfo->stVFrame.u32Height = height;
	pstFrameInfo->stVFrame.u32TimeRef = videv->postraw_frame_number[raw_num];
	pstFrameInfo->stVFrame.u64PTS = ts.tv_sec * 1000 + ts.tv_nsec / 1000 / 1000;
	pstFrameInfo->stVFrame.u32Stride[0] = width;
	pstFrameInfo->stVFrame.u32Stride[1] = width;
	pstFrameInfo->stVFrame.u32Stride[2] = width;
	pstFrameInfo->stVFrame.u32Length[0] = videv->pipe[raw_num].sizeimage[0];
	pstFrameInfo->stVFrame.u32Length[1] = videv->pipe[raw_num].sizeimage[1];
	pstFrameInfo->stVFrame.u32Length[2] = 0;

	ret = _vi_dma_alloc(videv, &yuv_dump_vb_info, &u64PhyAddr, size, 1, "VI_YUV_DUMP");
	if (ret < 0) {
		vi_pr(VI_ERR, "yuv dump dma alloc failed!\n");
		return ret;
	}

	atomic_set(&videv->isp_dump_yuv[raw_num], 1);

	ret = wait_event_interruptible_timeout(
		videv->yuv_dump_wait_q[raw_num],
		atomic_read(&videv->isp_dump_yuv[raw_num]) == 0,
		msecs_to_jiffies(s32MilliSec));

	if (!ret) {
		vi_pr(VI_ERR, "dev_%d wait dump_yuv timeout(%d ms)\n", ViPipe, s32MilliSec);
		return -ETIME;
	}

	pstFrameInfo->stVFrame.u64PhyAddr[0] = u64PhyAddr;

	vi_pr(VI_INFO, "pixfmt(%d), w(%d), h(%d), pts(%lld), addr(0x%llx), stride(%d)\n",
			pstFrameInfo->stVFrame.enPixelFormat, pstFrameInfo->stVFrame.u32Width,
			pstFrameInfo->stVFrame.u32Height, pstFrameInfo->stVFrame.u64PTS,
			pstFrameInfo->stVFrame.u64PhyAddr[0], pstFrameInfo->stVFrame.u32Stride[0]);

	return ret;
}

CVI_S32 vi_release_chn_frame(struct cvi_vi_dev *videv)
{
	_vi_dma_free(videv, &yuv_dump_vb_info);

	return CVI_SUCCESS;
}

/**
 * block num 16 [0~15]
 *
 * line_spliter, preraw_fe:
 * left input [0~7], right input [8~15]
 * left output [0~7], right output [8~15]
 *
 * preraw_be, postraw:
 * left input [0~8], right input [7~15]
 * left output [0~7], right output [8~15]
 */
static void _isp_tile_calc_size(struct isp_ctx *ictx)
{
	struct _isp_cfg *cfg_raw0 = &ictx->isp_pipe_cfg[ISP_PRERAW0];
	struct _isp_cfg *cfg_raw1 = &ictx->isp_pipe_cfg[ISP_PRERAW1];
	uint32_t guard = (ictx->img_width % 16 == 0)
			? (ictx->img_width / 16)
			: (ictx->img_width / 16) + 1;
	uint32_t left_width = guard * (16 / 2 + 1);

	ictx->tile_cfg.l_in.start  = 0;
	ictx->tile_cfg.l_in.end    = cfg_raw0->crop.x + left_width - 1;
	ictx->tile_cfg.l_out.start = 0;
	ictx->tile_cfg.l_out.end   = left_width - guard - 1;
	ictx->tile_cfg.r_in.start  = cfg_raw0->crop.x + left_width - guard - guard;
	ictx->tile_cfg.r_in.end    = cfg_raw0->max_width - 1;
	ictx->tile_cfg.r_out.start = left_width - guard;
	ictx->tile_cfg.r_out.end   = ictx->img_width - 1;

	vi_pr(VI_INFO, "tile cfg: img_width(%d) left_width(%d) guard(%d)\n",
		ictx->img_width, left_width, guard);
	vi_pr(VI_INFO, "tile cfg: Left in(%d %d) out(%d %d)\n",
		ictx->tile_cfg.l_in.start, ictx->tile_cfg.l_in.end,
		ictx->tile_cfg.l_out.start, ictx->tile_cfg.l_out.end);
	vi_pr(VI_INFO, "tile cfg: Right in(%d %d) out(%d %d)\n",
		ictx->tile_cfg.r_in.start, ictx->tile_cfg.r_in.end,
		ictx->tile_cfg.r_out.start, ictx->tile_cfg.r_out.end);

	memcpy(cfg_raw1, cfg_raw0, sizeof(struct _isp_cfg));
	cfg_raw0->csibdg_width = cfg_raw0->crop.x + ictx->tile_cfg.l_out.end + 1;
	cfg_raw1->csibdg_width = cfg_raw0->max_width - cfg_raw0->csibdg_width;

	cfg_raw0->crop.x       = cfg_raw0->crop.x;
	cfg_raw0->crop.y       = cfg_raw0->crop.y;
	cfg_raw0->crop.w       = cfg_raw0->csibdg_width - cfg_raw0->crop.x;
	cfg_raw0->crop.h       = cfg_raw0->crop.h;
	cfg_raw0->post_img_w   = cfg_raw0->crop.w;
	cfg_raw0->post_img_h   = cfg_raw0->crop.h;

	cfg_raw1->crop.x       = 0;
	cfg_raw1->crop.y       = cfg_raw1->crop.y;
	cfg_raw1->crop.w       = cfg_raw1->crop.w - ictx->tile_cfg.l_out.end - 1;
	cfg_raw1->crop.h       = cfg_raw1->crop.h;
	cfg_raw1->post_img_w   = cfg_raw1->crop.w;
	cfg_raw1->post_img_h   = cfg_raw1->crop.h;

	if (ictx->is_hdr_on) {
		memcpy(&cfg_raw0->crop_se, &cfg_raw0->crop, sizeof(struct vi_rect));
		memcpy(&cfg_raw1->crop_se, &cfg_raw1->crop, sizeof(struct vi_rect));
	}

	cfg_raw0->is_work_on_r_tile = false;
	cfg_raw1->is_work_on_r_tile = true;

	ictx->img_width  = cfg_raw0->crop.w;
	ictx->img_height = cfg_raw0->crop.h;

	vi_pr(VI_INFO, "csibdg max, w:h=%d:%d\n", cfg_raw0->max_width, cfg_raw0->max_height);
	vi_pr(VI_INFO, "csibdg_%d, w:h=%d:%d, crop_x:y:w:h=%d:%d:%d:%d\n",
		ISP_PRERAW0, cfg_raw0->csibdg_width, cfg_raw0->csibdg_height,
		cfg_raw0->crop.x, cfg_raw0->crop.y, cfg_raw0->crop.w, cfg_raw0->crop.h);
	vi_pr(VI_INFO, "csibdg_%d, w:h=%d:%d, crop_x:y:w:h=%d:%d:%d:%d\n",
		ISP_PRERAW1, cfg_raw1->csibdg_width, cfg_raw1->csibdg_height,
		cfg_raw1->crop.x, cfg_raw1->crop.y, cfg_raw1->crop.w, cfg_raw1->crop.h);
}

void _isp_snr_cfg_enq(struct cvi_isp_snr_update *snr_node, const enum cvi_isp_raw raw_num)
{
	unsigned long flags;
	struct _isp_snr_i2c_node *n, *q;

	if (snr_node == NULL)
		return;

	spin_lock_irqsave(&snr_node_lock[raw_num], flags);

	if (snr_node->snr_cfg_node.snsr.need_update) {
		n = kmalloc(sizeof(*n), GFP_ATOMIC);
		if (n == NULL) {
			vi_pr(VI_ERR, "SNR cfg node alloc size(%zu) fail\n", sizeof(*n));
			spin_unlock_irqrestore(&snr_node_lock[raw_num], flags);
			return;
		}
		memcpy(&n->n, &snr_node->snr_cfg_node.snsr, sizeof(struct snsr_regs_s));

		while (!list_empty(&isp_snr_i2c_queue[raw_num].list)
			&& (isp_snr_i2c_queue[raw_num].num_rdy >= (VI_MAX_LIST_NUM - 1))) {
			q = list_first_entry(&isp_snr_i2c_queue[raw_num].list, struct _isp_snr_i2c_node, list);
			list_del_init(&q->list);
			--isp_snr_i2c_queue[raw_num].num_rdy;
			kfree(q);
		}
		list_add_tail(&n->list, &isp_snr_i2c_queue[raw_num].list);
		++isp_snr_i2c_queue[raw_num].num_rdy;
	}

	spin_unlock_irqrestore(&snr_node_lock[raw_num], flags);
}

void cvi_isp_rdy_buf_queue(struct cvi_vi_dev *vdev, struct cvi_isp_buf *b, const u8 chn_num)
{
	unsigned long flags;

	if (b == NULL) {
		vi_pr(VI_ERR, "queue buf is NULL !!\n");
		return;
	}

	spin_lock_irqsave(&vdev->qbuf_lock, flags);
	list_add_tail(&b->list, &vdev->qbuf_list[chn_num]);
	++vdev->qbuf_num[chn_num];
	spin_unlock_irqrestore(&vdev->qbuf_lock, flags);
}

struct cvi_isp_buf *cvi_isp_rdy_buf_next(struct cvi_vi_dev *vdev, const u8 chn_num)
{
	unsigned long flags;
	struct cvi_isp_buf *b = NULL;

	spin_lock_irqsave(&vdev->qbuf_lock, flags);
	if (!list_empty(&vdev->qbuf_list[chn_num]))
		b = list_first_entry(&vdev->qbuf_list[chn_num], struct cvi_isp_buf, list);
	spin_unlock_irqrestore(&vdev->qbuf_lock, flags);

	return b;
}

int cvi_isp_rdy_buf_empty(struct cvi_vi_dev *vdev, const u8 chn_num)
{
	unsigned long flags;
	int empty = 0;

	spin_lock_irqsave(&vdev->qbuf_lock, flags);
	empty = (vdev->qbuf_num[chn_num] == 0);
	spin_unlock_irqrestore(&vdev->qbuf_lock, flags);

	return empty;
}

void cvi_isp_rdy_buf_pop(struct cvi_vi_dev *vdev, const u8 chn_num)
{
	unsigned long flags;

	if (vdev->postraw_frame_number[chn_num] <= gViCtx->bypass_frm[chn_num]) {
		return;
	}

	spin_lock_irqsave(&vdev->qbuf_lock, flags);
	vdev->qbuf_num[chn_num]--;
	spin_unlock_irqrestore(&vdev->qbuf_lock, flags);
}

void _vi_yuv_dma_setup(struct isp_ctx *ctx, const enum cvi_isp_raw raw_num)
{
	struct _membuf *pool = &isp_bufpool[raw_num];
	struct isp_buffer *b;
	struct isp_ctx *ictx = ctx;
	uint64_t bufaddr = 0;
	uint32_t bufsize = 0;
	uint32_t base = 0, dma = 0;
	uint32_t rgbmap_le = 0;
	uint8_t i = 0, j = 0;
	uint8_t total_chn = ctx->isp_pipe_cfg[raw_num].muxMode + 1;

	if (ctx->isp_pipe_cfg[raw_num].is_bt_demux)
		base = csibdg_lite_dma_find_hwid(raw_num, ISP_FE_CH0);
	else
		base = csibdg_dma_find_hwid(raw_num, ISP_FE_CH0);

	for (i = 0; i < total_chn; i++) {
		dma = base + i;

		for (j = 0; j < OFFLINE_YUV_BUF_NUM; j++) {
			b = vmalloc(sizeof(*b));
			if (b == NULL) {
				vi_pr(VI_ERR, "yuv_buf isp_buf_%d vmalloc size(%zu) fail\n", j, sizeof(*b));
				return;
			}
			memset(b, 0, sizeof(*b));
			b->raw_num = raw_num;
			b->chn_num = i;
			bufsize = ispblk_dma_yuv_bypass_config(ctx, dma, 0, raw_num);
			pool->yuv_yuyv[i][j] = b->addr = _mempool_pop(bufsize);

			if (j == 0)
				ispblk_dma_setaddr(ctx, dma, b->addr);

			isp_buf_queue(&pre_fe_out_q[raw_num][i], b);
		}
	}

	if (ctx->isp_pipe_cfg[raw_num].yuv_scene_mode == ISP_YUV_SCENE_ISP &&
	    !ctx->isp_pipe_cfg[raw_num].is_bt_demux) {
		if (ctx->is_3dnr_on) {
			// rgbmap_le
			rgbmap_le = rgbmap_dma_find_hwid(raw_num, ISP_RAW_PATH_LE);
			DMA_SETUP_2(raw_num, rgbmap_le);
			isp_bufpool[raw_num].rgbmap_le[0] = bufaddr;
			ispblk_rgbmap_dma_config(ctx, raw_num, rgbmap_le);

			//Slice buffer mode use ring buffer
			if (!(_is_fe_be_online(ctx) && ctx->is_rgbmap_sbm_on)) {
				if (!_is_all_online(ctx)) {
					for (i = 1; i < RGBMAP_BUF_IDX; i++)
						isp_bufpool[raw_num].rgbmap_le[i] = _mempool_pop(bufsize);
				}
			}
		}
	}
}

static void _isp_preraw_fe_dma_dump(struct isp_ctx *ictx, enum cvi_isp_raw raw_num)
{
	u8 i = 0;
	u8 pre_fe_buf_num = OFFLINE_RAW_BUF_NUM;
	char str[64] = "PRERAW_FE";

	//for ai isp, need more buffer
	if (ictx->isp_pipe_cfg[raw_num].is_raw_ai_isp) {
		pre_fe_buf_num += BNR_AI_ISP_BUF_NUM;
	}

	vi_pr(VI_INFO, "***************%s_%d************************\n", str, raw_num);
	for (i = 0; i < pre_fe_buf_num; i++)
		vi_pr(VI_INFO, "pre_fe_le(0x%llx)\n", isp_bufpool[raw_num].pre_fe_le[i]);

	for (i = 0; i < RGBMAP_BUF_IDX; i++)
		vi_pr(VI_INFO, "rgbmap_le(0x%llx)\n", isp_bufpool[raw_num].rgbmap_le[i]);

	if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
		for (i = 0; i < pre_fe_buf_num; i++)
			vi_pr(VI_INFO, "pre_fe_se(0x%llx)\n", isp_bufpool[raw_num].pre_fe_se[i]);

		for (i = 0; i < RGBMAP_BUF_IDX; i++)
			vi_pr(VI_INFO, "rgbmap_se(0x%llx)\n", isp_bufpool[raw_num].rgbmap_se[i]);
	}

	if (ictx->isp_pipe_cfg[raw_num].is_yuv_sensor &&
		!ictx->isp_pipe_cfg[raw_num].is_offline_scaler) {
		for (i = 0; i < ISP_FE_CHN_MAX; i++) {
			vi_pr(VI_DBG, "yuyv_yuv(0x%llx), yuyv_yuv(0x%llx)\n",
				isp_bufpool[raw_num].yuv_yuyv[i][0], isp_bufpool[raw_num].yuv_yuyv[i][1]);
		}
	}
	vi_pr(VI_DBG, "*************************************************\n");
}

static void _isp_preraw_be_dma_dump(struct isp_ctx *ictx)
{
	u8 i = 0;
	char str[64] = "PRERAW_BE";
	enum cvi_isp_raw raw_num = ISP_PRERAW0;
	u8 first_raw_num = vi_get_first_raw_num(ictx);

	vi_pr(VI_INFO, "***************%s************************\n", str);
	vi_pr(VI_INFO, "be_rdma_le(0x%llx)\n", isp_bufpool[first_raw_num].pre_fe_le[0]);
	for (i = 0; i < OFFLINE_PRE_BE_BUF_NUM; i++)
		vi_pr(VI_DBG, "be_wdma_le(0x%llx)\n", isp_bufpool[first_raw_num].pre_be_le[i]);

	if (ictx->is_hdr_on) {
		vi_pr(VI_INFO, "be_rdma_se(0x%llx)\n", isp_bufpool[first_raw_num].pre_fe_se[0]);
		for (i = 0; i < OFFLINE_PRE_BE_BUF_NUM; i++)
			vi_pr(VI_DBG, "be_wdma_se(0x%llx)\n", isp_bufpool[first_raw_num].pre_be_se[i]);
	}

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;

		vi_pr(VI_DBG, "***********************raw_num(%d)**********************\n", raw_num);
		vi_pr(VI_DBG, "af(0x%llx, 0x%llx)\n",
				isp_bufpool[raw_num].sts_mem[0].af.phy_addr,
				isp_bufpool[raw_num].sts_mem[1].af.phy_addr);

		for (i = 0; i < OFFLINE_PRE_BE_BUF_NUM; i++) {
			if (ictx->isp_pipe_cfg[raw_num].is_rgbir_sensor) {
				vi_pr(VI_DBG, "ir_le(0x%llx)\n", isp_bufpool[raw_num].ir_le[i]);
				if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
					vi_pr(VI_DBG, "ir_se(0x%llx)\n", isp_bufpool[raw_num].ir_se[i]);
				}
			}
		}
	}
	vi_pr(VI_DBG, "*************************************************\n");
}

static void _isp_rawtop_dma_dump(struct isp_ctx *ictx)
{
	char str[64] = "RAW_TOP";
	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	vi_pr(VI_INFO, "***************%s************************\n", str);
	vi_pr(VI_DBG, "rawtop_rdma_le(0x%llx)\n", isp_bufpool[raw_num].pre_be_le[0]);

	if (ictx->is_hdr_on)
		vi_pr(VI_DBG, "rawtop_rdma_se(0x%llx)\n", isp_bufpool[raw_num].pre_be_se[0]);

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;

		vi_pr(VI_INFO, "***********************raw_num(%d)**********************\n", raw_num);
		vi_pr(VI_DBG, "lsc(0x%llx)\n", isp_bufpool[raw_num].lsc);

		vi_pr(VI_DBG, "ae_le(0x%llx, 0x%llx)\n",
				isp_bufpool[raw_num].sts_mem[0].ae_le.phy_addr,
				isp_bufpool[raw_num].sts_mem[1].ae_le.phy_addr);
		vi_pr(VI_DBG, "gms(0x%llx, 0x%llx)\n",
				isp_bufpool[raw_num].sts_mem[0].gms.phy_addr,
				isp_bufpool[raw_num].sts_mem[1].gms.phy_addr);
		vi_pr(VI_DBG, "awb(0x%llx, 0x%llx)\n",
				isp_bufpool[raw_num].sts_mem[0].awb.phy_addr,
				isp_bufpool[raw_num].sts_mem[1].awb.phy_addr);
		vi_pr(VI_DBG, "lmap_le(0x%llx)\n", isp_bufpool[raw_num].lmap_le);

		if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
			vi_pr(VI_DBG, "ae_se(0x%llx, 0x%llx)\n",
					isp_bufpool[raw_num].sts_mem[0].ae_se.phy_addr,
					isp_bufpool[raw_num].sts_mem[1].ae_se.phy_addr);
			vi_pr(VI_DBG, "lmap_se(0x%llx)\n", isp_bufpool[raw_num].lmap_se);
		}
	}

	vi_pr(VI_DBG, "*************************************************\n");
}

static void _isp_rgbtop_dma_dump(struct isp_ctx *ictx)
{
	char str[64] = "RGB_TOP";
	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	vi_pr(VI_INFO, "***************%s************************\n", str);
	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;

		vi_pr(VI_INFO, "***********************raw_num(%d)**********************\n", raw_num);
		vi_pr(VI_DBG, "hist_edge_v(0x%llx, 0x%llx)\n",
				isp_bufpool[raw_num].sts_mem[0].hist_edge_v.phy_addr,
				isp_bufpool[raw_num].sts_mem[1].hist_edge_v.phy_addr);
		vi_pr(VI_DBG, "manr(0x%llx, 0x%llx), manr_rtile(0x%llx, 0x%llx)\n",
				isp_bufpool[raw_num].manr[0],
				isp_bufpool[raw_num].manr[1],
				isp_bufpool[raw_num].manr_rtile[0],
				isp_bufpool[raw_num].manr_rtile[1]);
		vi_pr(VI_DBG, "tdnr(0x%llx, 0x%llx, 0x%llx), tdnr_rtile(0x%llx, 0x%llx, 0x%llx)\n",
				isp_bufpool[raw_num].tdnr[0],
				isp_bufpool[raw_num].tdnr[1],
				isp_bufpool[raw_num].tdnr[2],
				isp_bufpool[raw_num].tdnr_rtile[0],
				isp_bufpool[raw_num].tdnr_rtile[1],
				isp_bufpool[raw_num].tdnr_rtile[2]);
	}

	vi_pr(VI_DBG, "*************************************************\n");
}

static void _isp_yuvtop_dma_dump(struct isp_ctx *ictx)
{
	char str[64] = "YUV_TOP";
	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	uint64_t dci_bufaddr = 0;
	uint64_t ldci_bufaddr = 0;

	vi_pr(VI_INFO, "***************%s************************\n", str);
	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;

		vi_pr(VI_INFO, "***********************raw_num(%d)**********************\n", raw_num);
		vi_pr(VI_DBG, "dci(0x%llx, 0x%llx)\n",
				isp_bufpool[raw_num].sts_mem[0].dci.phy_addr,
				isp_bufpool[raw_num].sts_mem[1].dci.phy_addr);
		vi_pr(VI_DBG, "ldci(0x%llx)\n", isp_bufpool[raw_num].ldci);

		// show wasted buf size for 256B-aligned ldci bufaddr
		dci_bufaddr = isp_bufpool[raw_num].sts_mem[1].dci.phy_addr;
		ldci_bufaddr = isp_bufpool[raw_num].ldci;
		vi_pr(VI_DBG, "ldci wasted_bufsize_for_alignment(%d)\n",
			(uint32_t)(ldci_bufaddr - (dci_bufaddr + 0x200)));
	}
	vi_pr(VI_INFO, "*************************************************\n");
	vi_pr(VI_INFO, "VI total reserved memory(0x%x)\n", isp_mempool.byteused);
	vi_pr(VI_INFO, "*************************************************\n");
}

static void _isp_manr_dma_setup(struct isp_ctx *ictx, enum cvi_isp_raw raw_num)
{
	uint64_t bufaddr = 0;
	uint32_t bufsize = 0;

	if (!ictx->is_3dnr_on)
		return;

	DMA_SETUP_2(raw_num, ISP_BLK_ID_DMA_CTL_MMAP_IIR_R);
	isp_bufpool[raw_num].manr[0] = bufaddr;
	ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_MMAP_IIR_W, isp_bufpool[raw_num].manr[0]);

	isp_bufpool[raw_num].sts_mem[0].mmap.phy_addr =
		isp_bufpool[raw_num].sts_mem[1].mmap.phy_addr = bufaddr;
	isp_bufpool[raw_num].sts_mem[0].mmap.size =
		isp_bufpool[raw_num].sts_mem[1].mmap.size = bufsize;

	if (ictx->isp_pipe_cfg[raw_num].is_tile) { //iir right tile
		bufsize = ispblk_dma_buf_get_size(ictx, raw_num + 1, ISP_BLK_ID_DMA_CTL_MMAP_IIR_R);
		isp_bufpool[raw_num].manr_rtile[0] = _mempool_pop(bufsize);
		isp_bufpool[raw_num].sts_mem[0].mmap.size =
			isp_bufpool[raw_num].sts_mem[1].mmap.size += bufsize;
	}

	if (ictx->isp_pipe_cfg[raw_num].is_tnr_ai_isp) {
		DMA_SETUP_2(raw_num, ISP_BLK_ID_DMA_CTL_MMAP_AI_ISP);
		isp_bufpool[raw_num].manr[1] = bufaddr;
	}

	if (_is_all_online(ictx)) {
		ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_MMAP_PRE_LE_R,
					isp_bufpool[raw_num].rgbmap_le[0]);
	} else {
		ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_MMAP_CUR_LE_R,
					isp_bufpool[raw_num].rgbmap_le[0]);
		if (_is_fe_be_online(ictx) && ictx->is_rgbmap_sbm_on)
			ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_MMAP_PRE_LE_R,
						isp_bufpool[raw_num].rgbmap_le[0]);
		else
			ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_MMAP_PRE_LE_R,
						isp_bufpool[raw_num].rgbmap_le[1]);
	}

	if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
		ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_MMAP_CUR_SE_R,
					isp_bufpool[raw_num].rgbmap_se[0]);
		if (_is_fe_be_online(ictx) && ictx->is_rgbmap_sbm_on)
			ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_MMAP_PRE_SE_R,
						isp_bufpool[raw_num].rgbmap_se[0]);
		else
			ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_MMAP_PRE_SE_R,
						isp_bufpool[raw_num].rgbmap_se[1]);
	}

	// tnr motion
	DMA_SETUP_2(raw_num, ISP_BLK_ID_DMA_CTL_TNR_ST_MO);
	isp_bufpool[raw_num].tdnr[0] = bufaddr;
	ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_TNR_LD_MO, bufaddr);

	if (ictx->isp_pipe_cfg[raw_num].is_tile) { //tnr motion right tile
		bufsize = ispblk_dma_buf_get_size(ictx, raw_num + 1, ISP_BLK_ID_DMA_CTL_TNR_ST_MO);
		isp_bufpool[raw_num].tdnr_rtile[0] = bufaddr;
	}

	// tnr y/uv
	if (ictx->is_fbc_on) {
		u64 bufaddr_tmp = 0;

		bufaddr_tmp = _mempool_get_addr();
		//ring buffer constraint. reg_base is 4k byte-align
		bufaddr = VI_4K_ALIGN(bufaddr_tmp);
		ispblk_dma_config(ictx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_LD_Y, bufaddr);
		bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_LD_Y);
		_mempool_pop(bufsize + (u32)(bufaddr - bufaddr_tmp));

		ispblk_dma_config(ictx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_ST_Y, bufaddr);
		isp_bufpool[raw_num].tdnr[1] = bufaddr;

		bufaddr_tmp = _mempool_get_addr();
		//ring buffer constraint. reg_base is 4k byte-align
		bufaddr = VI_4K_ALIGN(bufaddr_tmp);
		ispblk_dma_config(ictx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_LD_C, bufaddr);
		bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_LD_C);
		_mempool_pop(bufsize + (u32)(bufaddr - bufaddr_tmp));

		ispblk_dma_config(ictx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_ST_C, bufaddr);
		isp_bufpool[raw_num].tdnr[2] = bufaddr;
	} else {
		DMA_SETUP_2(raw_num, ISP_BLK_ID_DMA_CTL_TNR_LD_Y);
		ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_TNR_ST_Y, bufaddr);
		isp_bufpool[raw_num].tdnr[1] = bufaddr;

		DMA_SETUP_2(raw_num, ISP_BLK_ID_DMA_CTL_TNR_LD_C);
		ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_TNR_ST_C, bufaddr);
		isp_bufpool[raw_num].tdnr[2] = bufaddr;

		if (ictx->isp_pipe_cfg[raw_num].is_tile) {
			bufsize = ispblk_dma_buf_get_size(ictx, raw_num + 1,
					ISP_BLK_ID_DMA_CTL_TNR_LD_Y);
			isp_bufpool[raw_num].tdnr_rtile[1] = _mempool_pop(bufsize);

			bufsize = ispblk_dma_buf_get_size(ictx, raw_num + 1,
					ISP_BLK_ID_DMA_CTL_TNR_ST_C);
			isp_bufpool[raw_num].tdnr_rtile[2] = _mempool_pop(bufsize);
		}

		if (ictx->isp_pipe_cfg[raw_num].is_tnr_ai_isp) {
			DMA_SETUP_2(raw_num, ISP_BLK_ID_DMA_CTL_AI_ISP_RDMA_Y);
			isp_bufpool[raw_num].tnr_ai_isp[0] = bufaddr;

			DMA_SETUP_2(raw_num, ISP_BLK_ID_DMA_CTL_AI_ISP_RDMA_U);
			isp_bufpool[raw_num].tnr_ai_isp[1] = bufaddr;

			DMA_SETUP_2(raw_num, ISP_BLK_ID_DMA_CTL_AI_ISP_RDMA_V);
			isp_bufpool[raw_num].tnr_ai_isp[2] = bufaddr;
		}
	}
}

void _isp_splt_dma_setup(struct isp_ctx *ictx, enum cvi_isp_raw raw_num)
{
	uint64_t dma_addr = 0;
	uint64_t bufaddr = 0;
	uint32_t bufsize = 0;
	uint32_t splt_fe0_le = ISP_BLK_ID_DMA_CTL_SPLT_FE0_WDMA_LE;
	uint32_t splt_fe0_se = ISP_BLK_ID_DMA_CTL_SPLT_FE0_WDMA_SE;
	uint32_t splt_fe1_le = ISP_BLK_ID_DMA_CTL_SPLT_FE1_WDMA_LE;
	uint32_t splt_fe1_se = ISP_BLK_ID_DMA_CTL_SPLT_FE1_WDMA_SE;
	uint8_t  i = 0;
	struct isp_buffer *b;
	enum cvi_isp_fe_chn_num chn_num = ISP_FE_CH0;

	//raw ai isp move to FE, abandon splt config
	return;

	if (raw_num != ISP_PRERAW0)
		return;

	if (ictx->isp_pipe_cfg[raw_num].is_raw_ai_isp) {
		chn_num = ISP_FE_CH0;
		for (i = 0; i < OFFLINE_SPLT_BUF_NUM; i++) {
			DMA_SETUP_2(raw_num, splt_fe0_le);
			b = vmalloc(sizeof(*b));
			if (b == NULL) {
				vi_pr(VI_ERR, "splt_fe%d_le isp_buf_%d vmalloc size(%zu) fail\n",
						raw_num, i, sizeof(*b));
				return;
			}
			memset(b, 0, sizeof(*b));
			b->addr = bufaddr;
			b->raw_num = raw_num;
			b->chn_num = chn_num;
			b->ir_idx = i;
			isp_bufpool[raw_num].splt_le[i] = b->addr;
			isp_buf_queue(&splt_out_q[raw_num][chn_num], b);
		}
		ispblk_dma_config(ictx, raw_num, splt_fe0_le, isp_bufpool[raw_num].splt_le[0]);

		if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
			chn_num = ISP_FE_CH1;
			for (i = 0; i < OFFLINE_SPLT_BUF_NUM; i++) {
				DMA_SETUP_2(raw_num, splt_fe0_se);
				b = vmalloc(sizeof(*b));
				if (b == NULL) {
					vi_pr(VI_ERR, "splt_fe%d_se isp_buf_%d vmalloc size(%zu) fail\n",
							raw_num, i, sizeof(*b));
					return;
				}
				memset(b, 0, sizeof(*b));
				b->addr = bufaddr;
				b->raw_num = raw_num;
				b->chn_num = chn_num;
				b->ir_idx = i;
				isp_bufpool[raw_num].splt_se[i] = b->addr;
				isp_buf_queue(&splt_out_q[raw_num][chn_num], b);
			}
			ispblk_dma_config(ictx, raw_num, splt_fe0_se, isp_bufpool[raw_num].splt_se[0]);
		}

		if (ictx->isp_pipe_cfg[raw_num].is_tile) {
			dma_addr = isp_bufpool[raw_num].splt_le[0];
			dma_addr += (ictx->isp_pipe_cfg[raw_num].csibdg_width * 3) / 2;
			ispblk_dma_config(ictx, raw_num + 1, splt_fe1_le, dma_addr);

			if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
				dma_addr = isp_bufpool[raw_num].splt_se[0];
				dma_addr += (ictx->isp_pipe_cfg[raw_num].csibdg_width * 3) / 2;
				ispblk_dma_config(ictx, raw_num + 1, splt_fe1_se, dma_addr);
			}
		}
	}
}

void _isp_pre_fe_dma_setup(struct isp_ctx *ictx, enum cvi_isp_raw raw_num)
{
	uint64_t bufaddr = 0;
	uint32_t bufsize = 0;
	uint8_t  i = 0;
	uint8_t  pre_fe_buf_num = OFFLINE_RAW_BUF_NUM;
	struct isp_buffer *b;

	u32 raw_le, raw_se;
	u32 rgbmap_le, rgbmap_se;

	if (ictx->isp_pipe_cfg[raw_num].is_yuv_sensor) { //YUV sensor
		if (!ictx->isp_pipe_cfg[raw_num].is_offline_scaler) //Online mode to scaler
			_vi_yuv_dma_setup(ictx, raw_num);

		goto EXIT;
	}

	raw_le = csibdg_dma_find_hwid(raw_num, ISP_FE_CH0);
	raw_se = csibdg_dma_find_hwid(raw_num, ISP_FE_CH1);
	rgbmap_le = rgbmap_dma_find_hwid(raw_num, ISP_RAW_PATH_LE);
	rgbmap_se = rgbmap_dma_find_hwid(raw_num, ISP_RAW_PATH_SE);

	if (_is_be_post_online(ictx) && !ictx->isp_pipe_cfg[raw_num].is_raw_replay_be) { //fe->dram->be->post
		//for ai isp, need more buffer
		if (ictx->isp_pipe_cfg[raw_num].is_raw_ai_isp) {
			pre_fe_buf_num += BNR_AI_ISP_BUF_NUM;
		}

		for (i = 0; i < pre_fe_buf_num; i++) {
			DMA_SETUP_2(raw_num, raw_le);
			b = vmalloc(sizeof(*b));
			if (b == NULL) {
				vi_pr(VI_ERR, "raw_le isp_buf_%d vmalloc size(%zu) fail\n", i, sizeof(*b));
				return;
			}
			memset(b, 0, sizeof(*b));
			b->addr = bufaddr;
			b->raw_num = raw_num;
			b->chn_num = ISP_FE_CH0;
			b->ir_idx = i;
			isp_bufpool[raw_num].pre_fe_le[i] = b->addr;
			isp_buf_queue(&pre_fe_out_q[raw_num][ISP_FE_CH0], b);
		}
		ispblk_dma_config(ictx, raw_num, raw_le, isp_bufpool[raw_num].pre_fe_le[0]);
		if (ictx->isp_pipe_cfg[raw_num].is_tile) {
			raw_le = csibdg_dma_find_hwid(raw_num + 1, ISP_FE_CH0);
			bufaddr = isp_bufpool[raw_num].pre_fe_le[0] +
					3 * UPPER(ictx->isp_pipe_cfg[raw_num].crop.w, 1);
			ispblk_dma_config(ictx, raw_num + 1, raw_le, bufaddr);
		}

		if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
			for (i = 0; i < pre_fe_buf_num; i++) {
				DMA_SETUP_2(raw_num, raw_se);
				b = vmalloc(sizeof(*b));
				if (b == NULL) {
					vi_pr(VI_ERR, "raw_se isp_buf_%d vmalloc size(%zu) fail\n", i, sizeof(*b));
					return;
				}
				memset(b, 0, sizeof(*b));
				b->addr = bufaddr;
				b->raw_num = raw_num;
				b->chn_num = ISP_FE_CH1;
				b->ir_idx = i;
				isp_bufpool[raw_num].pre_fe_se[i] = b->addr;
				isp_buf_queue(&pre_fe_out_q[raw_num][ISP_FE_CH1], b);
			}
			ispblk_dma_config(ictx, raw_num, raw_se, isp_bufpool[raw_num].pre_fe_se[0]);
			if (ictx->isp_pipe_cfg[raw_num].is_tile) {
				raw_se = csibdg_dma_find_hwid(raw_num + 1, ISP_FE_CH1);
				bufaddr = isp_bufpool[raw_num].pre_fe_se[0] +
						3 * UPPER(ictx->isp_pipe_cfg[raw_num].crop_se.w, 1);
				ispblk_dma_config(ictx, raw_num + 1, raw_se, bufaddr);
			}
		}
	}

	if (ictx->is_3dnr_on) {
		// rgbmap_le
		DMA_SETUP_2(raw_num, rgbmap_le);
		isp_bufpool[raw_num].rgbmap_le[0] = bufaddr;
		ispblk_rgbmap_dma_config(ictx, raw_num, rgbmap_le);
		if (ictx->isp_pipe_cfg[raw_num].is_tile) {
			u32 grid_size = (1 << g_w_bit[raw_num]);
			u32 w = ictx->isp_pipe_cfg[raw_num].crop.w;

			rgbmap_le = rgbmap_dma_find_hwid(raw_num + 1, ISP_RAW_PATH_LE);
			bufaddr += ((w + grid_size - 1) / grid_size) * 6;
			ispblk_dma_setaddr(ictx, rgbmap_le, bufaddr);
			ispblk_rgbmap_dma_config(ictx, raw_num + 1, rgbmap_le);
		}
		//Slice buffer mode use ring buffer
		if (!(_is_fe_be_online(ictx) && ictx->is_rgbmap_sbm_on)) {
			if (!_is_all_online(ictx)) {
				for (i = 1; i < RGBMAP_BUF_IDX; i++)
					isp_bufpool[raw_num].rgbmap_le[i] = _mempool_pop(bufsize);
			}
		}

		if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
			// rgbmap se
			DMA_SETUP_2(raw_num, rgbmap_se);
			isp_bufpool[raw_num].rgbmap_se[0] = bufaddr;
			ispblk_rgbmap_dma_config(ictx, raw_num, rgbmap_se);
			if (ictx->isp_pipe_cfg[raw_num].is_tile) {
				u32 grid_size = (1 << g_w_bit[raw_num]);
				u32 w = ictx->isp_pipe_cfg[raw_num].crop.w;

				rgbmap_se = rgbmap_dma_find_hwid(raw_num + 1, ISP_RAW_PATH_SE);
				bufaddr += ((w + grid_size - 1) / grid_size) * 6;
				ispblk_dma_setaddr(ictx, rgbmap_se, bufaddr);
				ispblk_rgbmap_dma_config(ictx, raw_num + 1, rgbmap_se);
			}

			//Slice buffer mode use ring buffer
			if (!(_is_fe_be_online(ictx) && ictx->is_rgbmap_sbm_on)) {
				for (i = 1; i < RGBMAP_BUF_IDX; i++)
					isp_bufpool[raw_num].rgbmap_se[i] = _mempool_pop(bufsize);
			}
		}
	}

EXIT:
	_isp_preraw_fe_dma_dump(ictx, raw_num);
}

void _isp_pre_be_dma_setup(struct isp_ctx *ictx)
{
	uint64_t bufaddr = 0;
	uint32_t bufsize = 0;
	uint8_t  buf_num = 0;
	struct isp_buffer *b;

	enum cvi_isp_raw raw_num = ISP_PRERAW0;
	u8 first_raw_num = vi_get_first_raw_num(ictx);
	u8 cfg_dma = false;

	//RGB path
	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;
		if (!ictx->isp_pipe_cfg[raw_num].is_yuv_sensor) {
			cfg_dma = true;
			break;
		}
	}

	if (cfg_dma == false)
		goto EXIT;

	if (ictx->is_offline_be && !ictx->isp_pipe_cfg[first_raw_num].is_raw_replay_be) {
		//apply pre_fe_le buffer
		ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_PRE_RAW_VI_SEL_LE,
					isp_bufpool[first_raw_num].pre_fe_le[0]);

		if (ictx->is_hdr_on) {
			//apply pre_fe_se buffer
			ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_PRE_RAW_VI_SEL_SE,
						isp_bufpool[first_raw_num].pre_fe_se[0]);
		}
	}

	if (_is_fe_be_online(ictx)) { //fe->be->dram->post
		if (ictx->is_slice_buf_on) {
			DMA_SETUP_2(first_raw_num, ISP_BLK_ID_DMA_CTL_PRE_RAW_BE_LE);
			isp_bufpool[first_raw_num].pre_be_le[0] = bufaddr;

			if (ictx->isp_pipe_cfg[first_raw_num].is_hdr_on) {
				DMA_SETUP_2(first_raw_num, ISP_BLK_ID_DMA_CTL_PRE_RAW_BE_SE);
				isp_bufpool[first_raw_num].pre_be_se[0] = bufaddr;
			}
		} else {
			for (buf_num = 0; buf_num < OFFLINE_PRE_BE_BUF_NUM; buf_num++) {
				DMA_SETUP_2(first_raw_num, ISP_BLK_ID_DMA_CTL_PRE_RAW_BE_LE);
				b = vmalloc(sizeof(*b));
				if (b == NULL) {
					vi_pr(VI_ERR, "be_wdma_le isp_buf_%d vmalloc size(%zu) fail\n",
						buf_num, sizeof(*b));
					return;
				}
				memset(b, 0, sizeof(*b));
				b->addr = bufaddr;
				b->raw_num = first_raw_num;
				b->ir_idx = buf_num;
				isp_bufpool[first_raw_num].pre_be_le[buf_num] = b->addr;
				isp_buf_queue(&pre_be_out_q[ISP_RAW_PATH_LE], b);
			}

			ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_PRE_RAW_BE_LE,
						isp_bufpool[first_raw_num].pre_be_le[0]);

			if (ictx->isp_pipe_cfg[first_raw_num].is_hdr_on) {
				for (buf_num = 0; buf_num < OFFLINE_PRE_BE_BUF_NUM; buf_num++) {
					DMA_SETUP_2(first_raw_num, ISP_BLK_ID_DMA_CTL_PRE_RAW_BE_SE);
					b = vmalloc(sizeof(*b));
					if (b == NULL) {
						vi_pr(VI_ERR, "be_wdma_se isp_buf_%d vmalloc size(%zu) fail\n",
								buf_num, sizeof(*b));
						return;
					}
					memset(b, 0, sizeof(*b));
					b->addr = bufaddr;
					b->raw_num = first_raw_num;
					b->ir_idx = buf_num;
					isp_bufpool[first_raw_num].pre_be_se[buf_num] = b->addr;
					isp_buf_queue(&pre_be_out_q[ISP_RAW_PATH_SE], b);
				}

				ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_PRE_RAW_BE_SE,
							isp_bufpool[first_raw_num].pre_be_se[0]);
			}
		}
	}

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		//Be out dma
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;
		if (ictx->isp_pipe_cfg[raw_num].is_yuv_sensor)
			continue;

		// af
		DMA_SETUP(raw_num, ISP_BLK_ID_DMA_CTL_AF_W);
		isp_bufpool[raw_num].sts_mem[0].af.phy_addr = bufaddr;
		isp_bufpool[raw_num].sts_mem[0].af.size = bufsize;
		isp_bufpool[raw_num].sts_mem[1].af.phy_addr = _mempool_pop(bufsize);
		isp_bufpool[raw_num].sts_mem[1].af.size = bufsize;

		// rgbir
		if (ictx->isp_pipe_cfg[raw_num].is_rgbir_sensor) {
			for (buf_num = 0; buf_num < OFFLINE_PRE_BE_BUF_NUM; buf_num++) {
				DMA_SETUP(raw_num, ISP_BLK_ID_DMA_CTL_RGBIR_LE);
				isp_bufpool[raw_num].ir_le[buf_num] = bufaddr;
			}
			ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_RGBIR_LE, isp_bufpool[raw_num].ir_le[0]);

			if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
				for (buf_num = 0; buf_num < OFFLINE_PRE_BE_BUF_NUM; buf_num++) {
					DMA_SETUP(raw_num, ISP_BLK_ID_DMA_CTL_RGBIR_SE);
					isp_bufpool[raw_num].ir_se[buf_num] = bufaddr;
				}
				ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_RGBIR_SE, isp_bufpool[raw_num].ir_se[0]);
			}
		}
	}
EXIT:
	_isp_preraw_be_dma_dump(ictx);
}

void _isp_rawtop_dma_setup(struct isp_ctx *ictx)
{
	uint64_t bufaddr = 0;
	uint32_t bufsize = 0;

	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	u8 cfg_dma = false;

	//RGB path
	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;
		if (!ictx->isp_pipe_cfg[raw_num].is_yuv_sensor) {
			cfg_dma = true;
			break;
		}
	}

	if (cfg_dma == false) //YUV sensor only
		goto EXIT;

	if (_is_fe_be_online(ictx)) { //fe->be->dram->post
		u8 first_raw_num = vi_get_first_raw_num(ictx);

		ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_RAW_RDMA0,
					isp_bufpool[first_raw_num].pre_be_le[0]);
		if (ictx->is_hdr_on) {
			ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_RAW_RDMA1,
						isp_bufpool[first_raw_num].pre_be_se[0]);
		}
	}

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;
		if (ictx->isp_pipe_cfg[raw_num].is_yuv_sensor) //YUV sensor
			continue;

		// lsc
		DMA_SETUP(raw_num, ISP_BLK_ID_DMA_CTL_LSC_LE);
		isp_bufpool[raw_num].lsc = bufaddr;

		// left tile and right tile use the same lsc addr
		if (ictx->isp_pipe_cfg[raw_num].is_tile) {
			isp_bufpool[raw_num + 1].lsc = bufaddr;
		}

		// gms
		bufaddr = _mempool_get_addr();
		ispblk_dma_config(ictx, raw_num, ISP_BLK_ID_DMA_CTL_GMS, bufaddr);
		bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_GMS);
		_mempool_pop(bufsize);
		isp_bufpool[raw_num].sts_mem[0].gms.phy_addr = bufaddr;
		isp_bufpool[raw_num].sts_mem[0].gms.size = bufsize;
		isp_bufpool[raw_num].sts_mem[1].gms.phy_addr = _mempool_pop(bufsize);
		isp_bufpool[raw_num].sts_mem[1].gms.size = bufsize;

		// lmap_le
		DMA_SETUP_2(raw_num, ISP_BLK_ID_DMA_CTL_LMAP_LE);
		isp_bufpool[raw_num].lmap_le = bufaddr;

		// ae le
		DMA_SETUP(raw_num, ISP_BLK_ID_DMA_CTL_AE_HIST_LE);
		isp_bufpool[raw_num].sts_mem[0].ae_le.phy_addr = bufaddr;
		isp_bufpool[raw_num].sts_mem[0].ae_le.size = bufsize;
		isp_bufpool[raw_num].sts_mem[1].ae_le.phy_addr = _mempool_pop(bufsize);
		isp_bufpool[raw_num].sts_mem[1].ae_le.size = bufsize;

		if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
			// lsc_se
			ispblk_dma_config(ictx, raw_num, ISP_BLK_ID_DMA_CTL_LSC_SE, isp_bufpool[raw_num].lsc);

			// lmap_se
			DMA_SETUP_2(raw_num, ISP_BLK_ID_DMA_CTL_LMAP_SE);
			isp_bufpool[raw_num].lmap_se = bufaddr;

			// ae se
			DMA_SETUP(raw_num, ISP_BLK_ID_DMA_CTL_AE_HIST_SE);
			isp_bufpool[raw_num].sts_mem[0].ae_se.phy_addr = bufaddr;
			isp_bufpool[raw_num].sts_mem[0].ae_se.size = bufsize;
			isp_bufpool[raw_num].sts_mem[1].ae_se.phy_addr = _mempool_pop(bufsize);
			isp_bufpool[raw_num].sts_mem[1].ae_se.size = bufsize;
		}
	}

EXIT:
	_isp_rawtop_dma_dump(ictx);
}

void _isp_rgbtop_dma_setup(struct isp_ctx *ictx)
{
	uint64_t bufaddr = 0;
	uint32_t bufsize = 0;

	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	u8 cfg_dma = false;

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;

		cfg_dma = true;
		break;
	}

	if (cfg_dma == false) //No pipe need to setup
		goto EXIT;

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;
		if (ictx->isp_pipe_cfg[raw_num].is_yuv_sensor &&
			ictx->isp_pipe_cfg[raw_num].yuv_scene_mode != ISP_YUV_SCENE_ISP)
			continue;

		if (!ictx->isp_pipe_cfg[raw_num].is_yuv_sensor) {
			// hist_edge_v
			DMA_SETUP_2(raw_num, ISP_BLK_ID_DMA_CTL_HIST_EDGE_V);
			isp_bufpool[raw_num].sts_mem[0].hist_edge_v.phy_addr = bufaddr;
			isp_bufpool[raw_num].sts_mem[0].hist_edge_v.size = bufsize;
			isp_bufpool[raw_num].sts_mem[1].hist_edge_v.phy_addr = _mempool_pop(bufsize);
			isp_bufpool[raw_num].sts_mem[1].hist_edge_v.size = bufsize;

			// ltm dma
			ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_LTM_LE, isp_bufpool[raw_num].lmap_le);

			if (ictx->isp_pipe_cfg[raw_num].is_hdr_on)
				ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_LTM_SE, isp_bufpool[raw_num].lmap_se);
			else
				ispblk_dma_setaddr(ictx, ISP_BLK_ID_DMA_CTL_LTM_SE, isp_bufpool[raw_num].lmap_le);
		}

		// manr
		if (ictx->is_3dnr_on) {
			_isp_manr_dma_setup(ictx, raw_num);
		}
	}
EXIT:
	_isp_rgbtop_dma_dump(ictx);
}

void _isp_yuvtop_dma_setup(struct isp_ctx *ictx)
{
	uint64_t bufaddr = 0;
	uint64_t tmp_bufaddr = 0;
	uint32_t bufsize = 0;

	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	u8 cfg_dma = false;

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;
		if (!ictx->isp_pipe_cfg[raw_num].is_yuv_sensor ||
			ictx->isp_pipe_cfg[raw_num].yuv_scene_mode == ISP_YUV_SCENE_ISP) {
			cfg_dma = true;
			break;
		}
	}

	if (cfg_dma == false)
		goto EXIT;

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;
		if (ictx->isp_pipe_cfg[raw_num].is_yuv_sensor &&
			ictx->isp_pipe_cfg[raw_num].yuv_scene_mode != ISP_YUV_SCENE_ISP)
			continue;

		// dci
		DMA_SETUP(raw_num, ISP_BLK_ID_DMA_CTL_DCI);
		isp_bufpool[raw_num].sts_mem[0].dci.phy_addr = bufaddr;
		isp_bufpool[raw_num].sts_mem[0].dci.size = bufsize;
		isp_bufpool[raw_num].sts_mem[1].dci.phy_addr = _mempool_pop(bufsize);
		isp_bufpool[raw_num].sts_mem[1].dci.size = bufsize;

		//TODO use the same buffer or not
		if (ictx->isp_pipe_cfg[raw_num].is_tile) {
			// dci
			isp_bufpool[raw_num + 1].sts_mem[0].dci.phy_addr = isp_bufpool[raw_num].sts_mem[0].dci.phy_addr;
			isp_bufpool[raw_num + 1].sts_mem[0].dci.size     = isp_bufpool[raw_num].sts_mem[0].dci.size;
			isp_bufpool[raw_num + 1].sts_mem[1].dci.phy_addr = isp_bufpool[raw_num].sts_mem[1].dci.phy_addr;
			isp_bufpool[raw_num + 1].sts_mem[1].dci.size     = isp_bufpool[raw_num].sts_mem[1].dci.size;
		}

		// ldci
		//DMA_SETUP(ISP_BLK_ID_DMA_CTL_LDCI_W);
		//addr 256B alignment workaround
		tmp_bufaddr = _mempool_get_addr();
		bufaddr = VI_256_ALIGN(tmp_bufaddr);
		bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_LDCI_W);
		ispblk_dma_config(ictx, raw_num, ISP_BLK_ID_DMA_CTL_LDCI_W, bufaddr);
		_mempool_pop(bufsize + (uint32_t)(bufaddr - tmp_bufaddr));

		isp_bufpool[raw_num].ldci = bufaddr;
		ispblk_dma_config(ictx, raw_num, ISP_BLK_ID_DMA_CTL_LDCI_R, isp_bufpool[raw_num].ldci);
		if (ictx->isp_pipe_cfg[raw_num].is_tile) {
			tmp_bufaddr = _mempool_get_addr();
			bufaddr = VI_256_ALIGN(tmp_bufaddr);
			_mempool_pop(bufsize + (uint32_t)(bufaddr - tmp_bufaddr));
			isp_bufpool[raw_num + 1].ldci = bufaddr;
		}
	}

	if (cfg_dma) {
		if (ictx->isp_pipe_cfg[ISP_PRERAW0].is_offline_scaler ||
			(ictx->is_multi_sensor && ictx->isp_pipe_cfg[ISP_PRERAW1].is_offline_scaler)) {
			//SW workaround. Need to set y/uv dma_disable = 1 before csibdg enable
			if (_is_be_post_online(ictx) && !ictx->isp_pipe_cfg[ISP_PRERAW0].is_raw_replay_be) {
				ispblk_dma_enable(ictx, ISP_BLK_ID_DMA_CTL_YUV_CROP_Y, 1, 1);
				ispblk_dma_enable(ictx, ISP_BLK_ID_DMA_CTL_YUV_CROP_C, 1, 1);
			} else {
				ispblk_dma_enable(ictx, ISP_BLK_ID_DMA_CTL_YUV_CROP_Y, 1, 0);
				ispblk_dma_enable(ictx, ISP_BLK_ID_DMA_CTL_YUV_CROP_C, 1, 0);
			}
		} else {
			ispblk_dma_enable(ictx, ISP_BLK_ID_DMA_CTL_YUV_CROP_Y, 0, 0);
			ispblk_dma_enable(ictx, ISP_BLK_ID_DMA_CTL_YUV_CROP_C, 0, 0);
		}
	}

EXIT:
	_isp_yuvtop_dma_dump(ictx);
}

void _isp_cmdq_dma_setup(struct isp_ctx *ictx)
{
	uint64_t bufaddr = 0;
	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		bufaddr = _mempool_get_addr();
		ictx->isp_pipe_cfg[raw_num].cmdq_buf.buf_size = VI_CMDQ_BUF_SIZE;
		ictx->isp_pipe_cfg[raw_num].cmdq_buf.phy_addr = bufaddr;
		ictx->isp_pipe_cfg[raw_num].cmdq_buf.vir_addr = phys_to_virt(bufaddr);
		_mempool_pop(VI_CMDQ_BUF_SIZE);
	}
}

void _vi_dma_setup(struct isp_ctx *ictx)
{
	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;
		_isp_splt_dma_setup(ictx, raw_num);
		_isp_pre_fe_dma_setup(ictx, raw_num);
	}

	_isp_pre_be_dma_setup(ictx);
	_isp_rawtop_dma_setup(ictx);
	_isp_rgbtop_dma_setup(ictx);
	_isp_yuvtop_dma_setup(ictx);
	_isp_cmdq_dma_setup(ictx);
}

void _vi_dma_set_sw_mode(struct isp_ctx *ctx)
{
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_SPLT_FE0_WDMA_LE, true);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_SPLT_FE0_WDMA_SE, true);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_SPLT_FE0_RDMA_LE, ctx->is_tile ? true : false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_SPLT_FE0_RDMA_SE, ctx->is_tile ? true : false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_SPLT_FE1_WDMA_LE, true);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_SPLT_FE1_WDMA_SE, true);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_SPLT_FE1_RDMA_LE, ctx->is_tile ? true : false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_SPLT_FE1_RDMA_SE, ctx->is_tile ? true : false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_PRE_RAW_VI_SEL_LE, ctx->is_tile ? true : false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_PRE_RAW_VI_SEL_SE, ctx->is_tile ? true : false);
	ispblk_rgbmap_dma_mode(ctx, ISP_BLK_ID_DMA_CTL_FE0_RGBMAP_LE);
	ispblk_rgbmap_dma_mode(ctx, ISP_BLK_ID_DMA_CTL_FE0_RGBMAP_SE);
	ispblk_rgbmap_dma_mode(ctx, ISP_BLK_ID_DMA_CTL_FE1_RGBMAP_LE);
	ispblk_rgbmap_dma_mode(ctx, ISP_BLK_ID_DMA_CTL_FE1_RGBMAP_SE);
	ispblk_rgbmap_dma_mode(ctx, ISP_BLK_ID_DMA_CTL_FE2_RGBMAP_LE);
	ispblk_rgbmap_dma_mode(ctx, ISP_BLK_ID_DMA_CTL_FE2_RGBMAP_SE);
	ispblk_rgbmap_dma_mode(ctx, ISP_BLK_ID_DMA_CTL_FE3_RGBMAP_LE);
	ispblk_rgbmap_dma_mode(ctx, ISP_BLK_ID_DMA_CTL_FE3_RGBMAP_SE);
	ispblk_rgbmap_dma_mode(ctx, ISP_BLK_ID_DMA_CTL_FE4_RGBMAP_LE);
	ispblk_rgbmap_dma_mode(ctx, ISP_BLK_ID_DMA_CTL_FE4_RGBMAP_SE);
	ispblk_rgbmap_dma_mode(ctx, ISP_BLK_ID_DMA_CTL_FE5_RGBMAP_LE);
	ispblk_rgbmap_dma_mode(ctx, ISP_BLK_ID_DMA_CTL_FE5_RGBMAP_SE);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_PRE_RAW_BE_LE, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_PRE_RAW_BE_SE, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_LMAP_LE, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_LMAP_SE, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_CSI0_BDG0, ctx->is_tile ? true : false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_CSI0_BDG1, ctx->is_tile ? true : false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_CSI0_BDG2, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_CSI0_BDG3, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_CSI1_BDG0, ctx->is_tile ? true : false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_CSI1_BDG1, ctx->is_tile ? true : false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_CSI1_BDG2, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_CSI1_BDG3, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_CSI2_BDG0, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_CSI2_BDG1, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_CSI3_BDG0, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_CSI3_BDG1, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_CSI4_BDG0, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_CSI4_BDG1, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_CSI5_BDG0, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_CSI5_BDG1, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_BT0_LITE0, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_BT0_LITE1, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_BT0_LITE2, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_BT0_LITE3, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_BT1_LITE0, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_BT1_LITE1, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_BT1_LITE2, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_BT1_LITE3, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_RGBIR_LE, ctx->is_tile ? true : false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_RGBIR_SE, ctx->is_tile ? true : false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_AF_W, true);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_LSC_LE, true);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_LSC_SE, true);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_GMS, true);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_AE_HIST_LE, true);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_AE_HIST_SE, true);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_RAW_RDMA0, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_RAW_RDMA1, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_HIST_EDGE_V, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_YUV_CROP_Y, ctx->is_tile ? true : false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_YUV_CROP_C, ctx->is_tile ? true : false);
	ispblk_mmap_dma_mode(ctx, ISP_BLK_ID_DMA_CTL_MMAP_PRE_LE_R);
	ispblk_mmap_dma_mode(ctx, ISP_BLK_ID_DMA_CTL_MMAP_PRE_SE_R);
	ispblk_mmap_dma_mode(ctx, ISP_BLK_ID_DMA_CTL_MMAP_CUR_LE_R);
	ispblk_mmap_dma_mode(ctx, ISP_BLK_ID_DMA_CTL_MMAP_CUR_SE_R);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_MMAP_IIR_R, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_MMAP_IIR_W, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_MMAP_AI_ISP, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_TNR_ST_MO, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_TNR_LD_MO, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_TNR_ST_Y, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_TNR_ST_C, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_TNR_LD_Y, ctx->is_fbc_on ? true : false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_TNR_LD_C, ctx->is_fbc_on ? true : false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_AI_ISP_RDMA_Y, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_AI_ISP_RDMA_U, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_AI_ISP_RDMA_V, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_DCI, true);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_LTM_LE, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_LTM_SE, false);
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_LDCI_W, true); //ldci_iir_w
	ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_LDCI_R, true); //ldci_iir_r
}

void _vi_yuv_get_dma_size(struct isp_ctx *ctx, const enum cvi_isp_raw raw_num)
{
	uint32_t bufsize = 0;
	uint32_t base = 0, dma = 0;
	uint32_t rgbmap_le = 0;
	uint8_t i = 0, j = 0;
	uint8_t chn_str = ctx->raw_chnstr_num[raw_num];
	uint8_t total_chn = chn_str + ctx->isp_pipe_cfg[raw_num].muxMode + 1;

	if (ctx->isp_pipe_cfg[raw_num].is_bt_demux)
		base = csibdg_lite_dma_find_hwid(raw_num, ISP_FE_CH0);
	else
		base = csibdg_dma_find_hwid(raw_num, ISP_FE_CH0);

	for (i = chn_str; i < total_chn; i++) {
		dma = base + i - chn_str;

		for (j = 0; j < OFFLINE_YUV_BUF_NUM; j++) {
			bufsize = ispblk_dma_yuv_bypass_config(ctx, dma, 0, raw_num);
			_mempool_pop(bufsize);
		}
	}

	if (ctx->isp_pipe_cfg[raw_num].yuv_scene_mode == ISP_YUV_SCENE_ISP &&
	    !ctx->isp_pipe_cfg[raw_num].is_bt_demux) {
		if (ctx->is_3dnr_on) {
			// rgbmap le
			rgbmap_le = rgbmap_dma_find_hwid(raw_num, ISP_RAW_PATH_LE);
			bufsize = ispblk_dma_buf_get_size(ctx, raw_num, rgbmap_le);
			_mempool_pop(bufsize);

			//Slice buffer mode use ring buffer
			if (!(_is_fe_be_online(ctx) && ctx->is_rgbmap_sbm_on)) {
				if (!_is_all_online(ctx)) {
					for (i = 1; i < RGBMAP_BUF_IDX; i++)
						_mempool_pop(bufsize);
				}
			}
		}
	}
}

void _vi_splt_get_dma_size(struct isp_ctx *ictx, enum cvi_isp_raw raw_num)
{
	uint32_t bufsize = 0;
	uint32_t splt_le = 0, splt_se = 0;
	uint8_t  i = 0;

	//raw ai isp move to FE, abandon splt config
	return;

	if (raw_num != ISP_PRERAW0)
		return;

	if (ictx->isp_pipe_cfg[raw_num].is_raw_ai_isp) {
		splt_le = ISP_BLK_ID_DMA_CTL_SPLT_FE0_WDMA_LE;
		splt_se = ISP_BLK_ID_DMA_CTL_SPLT_FE0_WDMA_SE;

		for (i = 0; i < OFFLINE_SPLT_BUF_NUM; i++) {
			bufsize = ispblk_dma_buf_get_size(ictx, raw_num, splt_le);
			_mempool_pop(bufsize);
		}

		if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
			for (i = 0; i < OFFLINE_SPLT_BUF_NUM; i++) {
				bufsize = ispblk_dma_buf_get_size(ictx, raw_num, splt_se);
				_mempool_pop(bufsize);
			}
		}
	}
}

void _vi_pre_fe_get_dma_size(struct isp_ctx *ictx, enum cvi_isp_raw raw_num)
{
	uint32_t bufsize = 0;
	uint8_t  i = 0;
	uint8_t  pre_fe_buf_num = OFFLINE_RAW_BUF_NUM;
	u32 raw_le, raw_se;
	u32 rgbmap_le, rgbmap_se;

	raw_le = csibdg_dma_find_hwid(raw_num, ISP_FE_CH0);
	raw_se = csibdg_dma_find_hwid(raw_num, ISP_FE_CH1);
	rgbmap_le = rgbmap_dma_find_hwid(raw_num, ISP_RAW_PATH_LE);
	rgbmap_se = rgbmap_dma_find_hwid(raw_num, ISP_RAW_PATH_SE);

	if (ictx->isp_pipe_cfg[raw_num].is_yuv_sensor) { //YUV sensor
		if (!ictx->isp_pipe_cfg[raw_num].is_offline_scaler) //Online mode to scaler
			_vi_yuv_get_dma_size(ictx, raw_num);

		goto EXIT;
	}

	if (_is_be_post_online(ictx)) { //fe->dram->be->post
		//for ai isp, need more buffer
		if (ictx->isp_pipe_cfg[raw_num].is_raw_ai_isp) {
			pre_fe_buf_num += BNR_AI_ISP_BUF_NUM;
		}

		for (i = 0; i < pre_fe_buf_num; i++) {
			bufsize = ispblk_dma_buf_get_size(ictx, raw_num, raw_le);
			_mempool_pop(bufsize);
		}

		if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
			for (i = 0; i < pre_fe_buf_num; i++) {
				bufsize = ispblk_dma_buf_get_size(ictx, raw_num, raw_se);
				_mempool_pop(bufsize);
			}
		}
	}

	// rgbmap le
	bufsize = ispblk_dma_buf_get_size(ictx, raw_num, rgbmap_le);
	_mempool_pop(bufsize);

	//Slice buffer mode use ring buffer
	if (!(_is_fe_be_online(ictx) && ictx->is_rgbmap_sbm_on)) {
		if (!_is_all_online(ictx)) {
			for (i = 1; i < RGBMAP_BUF_IDX; i++)
				_mempool_pop(bufsize);
		}
	}

	if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
		// rgbmap se
		bufsize = ispblk_dma_buf_get_size(ictx, raw_num, rgbmap_se);
		_mempool_pop(bufsize);

		//Slice buffer mode use ring buffer
		if (!(_is_fe_be_online(ictx) && ictx->is_rgbmap_sbm_on)) {
			for (i = 1; i < RGBMAP_BUF_IDX; i++)
				_mempool_pop(bufsize);
		}
	}
EXIT:
	return;
}

void _vi_pre_be_get_dma_size(struct isp_ctx *ictx)
{
	uint32_t bufsize = 0;
	uint8_t  buf_num = 0;

	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	u8 cfg_dma = false;

	//RGB path
	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;
		if (!ictx->isp_pipe_cfg[raw_num].is_yuv_sensor) {
			cfg_dma = true;
			break;
		}
	}

	if (cfg_dma == false)
		goto EXIT;

	if (_is_fe_be_online(ictx)) { //fe->be->dram->post
		raw_num = vi_get_first_raw_num(ictx);

		if (ictx->is_slice_buf_on) {
			bufsize = ispblk_dma_buf_get_size(ictx, raw_num,
					ISP_BLK_ID_DMA_CTL_PRE_RAW_BE_LE);
			_mempool_pop(bufsize);

			if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
				bufsize = ispblk_dma_buf_get_size(ictx, raw_num,
						ISP_BLK_ID_DMA_CTL_PRE_RAW_BE_SE);
				_mempool_pop(bufsize);
			}
		} else {
			for (buf_num = 0; buf_num < OFFLINE_PRE_BE_BUF_NUM; buf_num++) {
				bufsize = ispblk_dma_buf_get_size(ictx, raw_num,
						ISP_BLK_ID_DMA_CTL_PRE_RAW_BE_LE);
				_mempool_pop(bufsize);
			}

			if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
				for (buf_num = 0; buf_num < OFFLINE_PRE_BE_BUF_NUM; buf_num++) {
					bufsize = ispblk_dma_buf_get_size(ictx, raw_num,
							ISP_BLK_ID_DMA_CTL_PRE_RAW_BE_SE);
					_mempool_pop(bufsize);
				}
			}
		}
	}

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		//Be out dma
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;
		if (ictx->isp_pipe_cfg[raw_num].is_yuv_sensor)
			continue;

		// af
		bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_AF_W);
		_mempool_pop(bufsize);
		_mempool_pop(bufsize);

		// rgbir
		if (ictx->isp_pipe_cfg[raw_num].is_rgbir_sensor) {
			for (buf_num = 0; buf_num < OFFLINE_PRE_BE_BUF_NUM; buf_num++) {
				bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_RGBIR_LE);
				_mempool_pop(bufsize);
			}

			if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
				for (buf_num = 0; buf_num < OFFLINE_PRE_BE_BUF_NUM; buf_num++) {
					bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_RGBIR_SE);
					_mempool_pop(bufsize);
				}
			}
		}
	}
EXIT:
	return;
}

void _vi_rawtop_get_dma_size(struct isp_ctx *ictx)
{
	uint32_t bufsize = 0;

	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	u8 cfg_dma = false;

	//RGB path
	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;
		if (!ictx->isp_pipe_cfg[raw_num].is_yuv_sensor) {
			cfg_dma = true;
			break;
		}
	}

	if (cfg_dma == false) //YUV sensor only
		goto EXIT;

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;
		if (ictx->isp_pipe_cfg[raw_num].is_yuv_sensor) //YUV sensor
			continue;

		// lsc
		bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_LSC_LE);
		_mempool_pop(bufsize);

		// gms
		bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_GMS);
		_mempool_pop(bufsize);
		_mempool_pop(bufsize);

		// lmap_le
		bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_LMAP_LE);
		_mempool_pop(bufsize);

		// ae le
		bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_AE_HIST_LE);
		_mempool_pop(bufsize);
		_mempool_pop(bufsize);

		if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
			// lmap_se
			bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_LMAP_SE);
			_mempool_pop(bufsize);

			// ae se
			bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_AE_HIST_SE);
			_mempool_pop(bufsize);
			_mempool_pop(bufsize);
		}
	}
EXIT:
	return;
}

void _vi_rgbtop_get_dma_size(struct isp_ctx *ictx)
{
	uint32_t bufsize = 0;

	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	u8 cfg_dma = false;

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;

		cfg_dma = true;
		break;
	}

	if (cfg_dma == false) //No pipe need to setup
		goto EXIT;

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;
		if (ictx->isp_pipe_cfg[raw_num].is_yuv_sensor &&
			ictx->isp_pipe_cfg[raw_num].yuv_scene_mode != ISP_YUV_SCENE_ISP) //YUV sensor
			continue;

		if (!ictx->isp_pipe_cfg[raw_num].is_yuv_sensor) {
			// hist_edge_v
			bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_HIST_EDGE_V);
			_mempool_pop(bufsize);
			_mempool_pop(bufsize);
		}

		// manr
		if (ictx->is_3dnr_on) {
			uint64_t bufaddr = 0;
			uint64_t tmp_bufaddr = 0;

			// MANR M + H
			bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_MMAP_IIR_R);
			_mempool_pop(bufsize);
			if (ictx->isp_pipe_cfg[raw_num].is_tile) {
				// MANR M + H right tile
				bufsize = ispblk_dma_buf_get_size(ictx, raw_num + 1, ISP_BLK_ID_DMA_CTL_MMAP_IIR_R);
				_mempool_pop(bufsize);
			}

			if (ictx->isp_pipe_cfg[raw_num].is_tnr_ai_isp) {
				bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_MMAP_AI_ISP);
				_mempool_pop(bufsize);
			}

			// TNR motion
			bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_ST_MO);
			_mempool_pop(bufsize);

			if (ictx->isp_pipe_cfg[raw_num].is_tile) {
				// TNR motion right tile
				bufsize = ispblk_dma_buf_get_size(ictx, raw_num + 1, ISP_BLK_ID_DMA_CTL_TNR_ST_MO);
				_mempool_pop(bufsize);
			}

			if (ictx->is_fbc_on) {
				// TNR Y
				tmp_bufaddr = _mempool_get_addr();
				//ring buffer constraint. reg_base is 4k byte-align
				bufaddr = VI_4K_ALIGN(tmp_bufaddr);
				bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_LD_Y);
				_mempool_pop(bufsize + (uint32_t)(bufaddr - tmp_bufaddr));

				// TNR UV
				tmp_bufaddr = _mempool_get_addr();
				//ring buffer constraint. reg_base is 4k byte-align
				bufaddr = VI_4K_ALIGN(tmp_bufaddr);
				bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_LD_C);
				_mempool_pop(bufsize + (uint32_t)(bufaddr - tmp_bufaddr));
			} else {
				// TNR Y
				bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_LD_Y);
				_mempool_pop(bufsize);

				// TNR UV
				bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_LD_C);
				_mempool_pop(bufsize);
				if (ictx->isp_pipe_cfg[raw_num].is_tile) {
					// TNR Y right tile
					bufsize = ispblk_dma_buf_get_size(ictx, raw_num + 1,
							ISP_BLK_ID_DMA_CTL_TNR_LD_Y);
					_mempool_pop(bufsize);

					// TNR UV right tile
					bufsize = ispblk_dma_buf_get_size(ictx, raw_num + 1,
							ISP_BLK_ID_DMA_CTL_TNR_LD_C);
					_mempool_pop(bufsize);
				}

				if (ictx->isp_pipe_cfg[raw_num].is_tnr_ai_isp) {
					bufsize = ispblk_dma_buf_get_size(ictx, raw_num,
							ISP_BLK_ID_DMA_CTL_AI_ISP_RDMA_Y);
					_mempool_pop(bufsize);

					bufsize = ispblk_dma_buf_get_size(ictx, raw_num,
							ISP_BLK_ID_DMA_CTL_AI_ISP_RDMA_U);
					_mempool_pop(bufsize);

					bufsize = ispblk_dma_buf_get_size(ictx, raw_num,
							ISP_BLK_ID_DMA_CTL_AI_ISP_RDMA_V);
					_mempool_pop(bufsize);
				}
			}
		}
	}
EXIT:
	return;
}

void _vi_yuvtop_get_dma_size(struct isp_ctx *ictx)
{
	uint32_t bufsize = 0;
	uint64_t bufaddr = 0;
	uint64_t tmp_bufaddr = 0;

	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	u8 cfg_dma = false;

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;
		if (!ictx->isp_pipe_cfg[raw_num].is_yuv_sensor ||
			ictx->isp_pipe_cfg[raw_num].yuv_scene_mode == ISP_YUV_SCENE_ISP) {
			cfg_dma = true;
			break;
		}
	}

	if (cfg_dma == false) //No pipe need get_dma_size
		goto EXIT;

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;
		if (ictx->isp_pipe_cfg[raw_num].is_yuv_sensor &&
			ictx->isp_pipe_cfg[raw_num].yuv_scene_mode != ISP_YUV_SCENE_ISP) //YUV sensor
			continue;

		// dci
		bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_DCI);
		_mempool_pop(bufsize);
		tmp_bufaddr = _mempool_pop(bufsize);

		// ldci
		bufaddr = VI_256_ALIGN(tmp_bufaddr);
		bufsize = ispblk_dma_buf_get_size(ictx, raw_num, ISP_BLK_ID_DMA_CTL_LDCI_W);
		_mempool_pop(bufsize + (uint32_t)(bufaddr - tmp_bufaddr));

		vi_pr(VI_INFO, "ldci bufsize: total(%d), used(%d), wasted_for_alignment(%d)\n",
			bufsize + (uint32_t)(bufaddr - tmp_bufaddr),
			bufsize,
			(uint32_t)(bufaddr - tmp_bufaddr));

		if (ictx->isp_pipe_cfg[raw_num].is_tile) { //ldci right tile
			tmp_bufaddr = _mempool_get_addr();
			bufaddr = VI_256_ALIGN(tmp_bufaddr);
			_mempool_pop(bufsize + (uint32_t)(bufaddr - tmp_bufaddr));

			vi_pr(VI_INFO, "ldci right tile bufsize: total(%d), used(%d), wasted_for_alignment(%d)\n",
				bufsize + (uint32_t)(bufaddr - tmp_bufaddr),
				bufsize,
				(uint32_t)(bufaddr - tmp_bufaddr));
		}
	}
EXIT:
	return;
}

void _vi_cmdq_get_dma_size(struct isp_ctx *ictx)
{
	uint32_t bufsize = 0;
	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		bufsize += VI_CMDQ_BUF_SIZE;
	}

	_mempool_pop(bufsize);
}

void _vi_get_dma_buf_size(struct isp_ctx *ictx)
{
	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;
		if (_is_right_tile(ictx, raw_num))
			continue;
		_vi_splt_get_dma_size(ictx, raw_num);
		_vi_pre_fe_get_dma_size(ictx, raw_num);
	}

	_vi_pre_be_get_dma_size(ictx);
	_vi_rawtop_get_dma_size(ictx);
	_vi_rgbtop_get_dma_size(ictx);
	_vi_yuvtop_get_dma_size(ictx);
	_vi_cmdq_get_dma_size(ictx);
}

static void _vi_preraw_be_init(struct cvi_vi_dev *vdev)
{
	struct isp_ctx *ctx = &vdev->ctx;
	uint32_t bps[40] = {((0 << 12) | 0), ((0 << 12) | 20), ((0 << 12) | 40), ((0 << 12) | 60)};
	enum cvi_isp_raw raw_num = ISP_PRERAW0;
	u8 cfg_be = false;

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ctx->isp_pipe_enable[raw_num])
			continue;

		if (!ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) {
			cfg_be = true;
			break;
		}
	}

	if (cfg_be) { //RGB sensor
		// preraw_vi_sel
		ispblk_preraw_vi_sel_config(ctx);
		// preraw_be_top
		ispblk_preraw_be_config(ctx, raw_num);
		// preraw_be wdma ctrl
		ispblk_pre_wdma_ctrl_config(ctx, raw_num);

		//ispblk_blc_set_offset(ctx, ISP_BLC_ID_BE_LE, 511, 511, 511, 511);
		ispblk_blc_set_gain(ctx, ISP_BLC_ID_BE_LE, 0x40f, 0x419, 0x419, 0x405);
		//ispblk_blc_set_2ndoffset(ctx, ISP_BLC_ID_BE_LE, 511, 511, 511, 511);
		ispblk_blc_enable(ctx, ISP_BLC_ID_BE_LE, false, false);

		if (ctx->is_hdr_on) {
			ispblk_blc_set_offset(ctx, ISP_BLC_ID_BE_SE, 511, 511, 511, 511);
			ispblk_blc_set_gain(ctx, ISP_BLC_ID_BE_SE, 0x800, 0x800, 0x800, 0x800);
			ispblk_blc_set_2ndoffset(ctx, ISP_BLC_ID_BE_SE, 511, 511, 511, 511);
			ispblk_blc_enable(ctx, ISP_BLC_ID_BE_SE, false, false);
		}

		if (ctx->isp_pipe_cfg[raw_num].is_rgbir_sensor) {
			ispblk_rgbir_config(ctx, ISP_RAW_PATH_LE, true);
			ispblk_rgbir_config(ctx, ISP_RAW_PATH_SE, ctx->is_hdr_on);
		} else {
			ispblk_rgbir_config(ctx, ISP_RAW_PATH_LE, false);
			ispblk_rgbir_config(ctx, ISP_RAW_PATH_SE, false);
		}

		ispblk_dpc_set_static(ctx, ISP_RAW_PATH_LE, 0, bps, 4);
		ispblk_dpc_config(ctx, ISP_RAW_PATH_LE, false, 0);
		ispblk_dpc_config(ctx, ISP_RAW_PATH_SE, ctx->is_hdr_on, 0);

		ispblk_af_config(ctx, true);

		if (_is_fe_be_online(ctx) && ctx->is_slice_buf_on)
			ispblk_slice_buf_config(&vdev->ctx, raw_num, true);
		else
			ispblk_slice_buf_config(&vdev->ctx, raw_num, false);
	}
}

static void _isp_rawtop_init(struct cvi_vi_dev *vdev)
{
	struct isp_ctx *ictx = &vdev->ctx;
	u8 first_raw_num = vi_get_first_raw_num(ictx);

	// raw_top
	ispblk_rawtop_config(ictx, first_raw_num);
	// raw_rdma ctrl
	ispblk_raw_rdma_ctrl_config(ictx, first_raw_num, ISP_BLK_ID_RAW_RDMA0,
					ictx->is_offline_postraw);
	ispblk_raw_rdma_ctrl_config(ictx, first_raw_num, ISP_BLK_ID_RAW_RDMA1,
					ictx->is_offline_postraw && ictx->is_hdr_on);

	ispblk_bnr_config(ictx, ISP_BLK_ID_BNR0, ISP_BNR_OUT_B_DELAY, false, 0, 0);

	ispblk_lsc_config(ictx, ISP_BLK_ID_LSC0, false);

	ispblk_cfa_config(ictx, ISP_BLK_ID_CFA0);
	ispblk_rgbcac_config(ictx, ISP_BLK_ID_RGBCAC0, true, 0);
	ispblk_lcac_config(ictx, ISP_BLK_ID_LCAC0, false, 0);
	ispblk_gms_config(ictx, true);

	ispblk_wbg_config(ictx, ISP_WBG_ID_RAW_TOP_LE, 0x400, 0x400, 0x400);
	ispblk_wbg_enable(ictx, ISP_WBG_ID_RAW_TOP_LE, false, false);

	ispblk_lmap_config(ictx, ISP_BLK_ID_LMAP0, true);

	ispblk_aehist_config(ictx, ISP_BLK_ID_AE_HIST0, true);

	if (ictx->is_hdr_on) {
		ispblk_bnr_config(ictx, ISP_BLK_ID_BNR1, ISP_BNR_OUT_B_DELAY, false, 0, 0);
		ispblk_lsc_config(ictx, ISP_BLK_ID_LSC1, false);
		ispblk_cfa_config(ictx, ISP_BLK_ID_CFA1);
		ispblk_rgbcac_config(ictx, ISP_BLK_ID_RGBCAC1, true, 0);
		ispblk_lcac_config(ictx, ISP_BLK_ID_LCAC1, false, 0);
		ispblk_wbg_config(ictx, ISP_WBG_ID_RAW_TOP_SE, 0x400, 0x400, 0x400);
		ispblk_wbg_enable(ictx, ISP_WBG_ID_RAW_TOP_SE, false, false);
		ispblk_lmap_config(ictx, ISP_BLK_ID_LMAP1, true);
		ispblk_aehist_config(ictx, ISP_BLK_ID_AE_HIST1, true);
	} else {
		ispblk_bnr_config(ictx, ISP_BLK_ID_BNR1, ISP_BNR_OUT_B_DELAY, false, 0, 0);
		ispblk_lsc_config(ictx, ISP_BLK_ID_LSC1, false);
		ispblk_rgbcac_config(ictx, ISP_BLK_ID_RGBCAC1, false, 0);
		ispblk_lcac_config(ictx, ISP_BLK_ID_LCAC1, false, 0);
		ispblk_lmap_config(ictx, ISP_BLK_ID_LMAP1, false);
		ispblk_aehist_config(ictx, ISP_BLK_ID_AE_HIST1, false);
	}
}

static void _isp_rgbtop_init(struct cvi_vi_dev *vdev)
{
	struct isp_ctx *ictx = &vdev->ctx;
	u8 first_raw_num = vi_get_first_raw_num(ictx);

	ispblk_rgbtop_config(ictx, first_raw_num);

	ispblk_hist_v_config(ictx, true, 0);

	//ispblk_awb_config(ictx, ISP_BLK_ID_AWB2, true, ISP_AWB_LE);

	ispblk_ccm_config(ictx, ISP_BLK_ID_CCM0, false, &ccm_hw_cfg);
	ispblk_dhz_config(ictx, false);

	ispblk_ygamma_config(ictx, false, ictx->gamma_tbl_idx, ygamma_data, 0, 0);
	ispblk_ygamma_enable(ictx, false);

	ispblk_gamma_config(ictx, false, ictx->gamma_tbl_idx, gamma_data, 0);
	ispblk_gamma_enable(ictx, false);

	//ispblk_clut_config(ictx, false, c_lut_r_lut, c_lut_g_lut, c_lut_b_lut);
	ispblk_rgbdither_config(ictx, false, false, false, false);
	ispblk_csc_config(ictx);

	ispblk_manr_config(ictx, ictx->is_3dnr_on);

	if (ictx->is_hdr_on) {
		ispblk_ccm_config(ictx, ISP_BLK_ID_CCM1, false, &ccm_hw_cfg);
		ispblk_fusion_config(ictx, true, true, ISP_FS_OUT_FS);
		ispblk_ltm_b_lut(ictx, 0, ltm_b_lut);
		ispblk_ltm_d_lut(ictx, 0, ltm_d_lut);
		ispblk_ltm_g_lut(ictx, 0, ltm_g_lut);
		ispblk_ltm_config(ictx, true, true, true, true);
	} else {
		ispblk_fusion_config(ictx, !_is_all_online(ictx), true, ISP_FS_OUT_LONG);
		ispblk_ltm_config(ictx, false, false, false, false);
	}
}

static void _isp_yuvtop_init(struct cvi_vi_dev *vdev)
{
	struct isp_ctx *ictx = &vdev->ctx;
	u8 first_raw_num = vi_get_first_raw_num(ictx);

	ispblk_yuvtop_config(ictx, first_raw_num);

	ispblk_yuvdither_config(ictx, 0, false, true, true, true);
	ispblk_yuvdither_config(ictx, 1, false, true, true, true);

	ispblk_tnr_config(ictx, ictx->is_3dnr_on, 0);
	if (ictx->is_3dnr_on && ictx->is_fbc_on) {
		ispblk_fbce_config(ictx, true);
		ispblk_fbcd_config(ictx, true);
		ispblk_fbc_ring_buf_config(ictx, true);
	} else {
		ispblk_fbce_config(ictx, false);
		ispblk_fbcd_config(ictx, false);
		ispblk_fbc_ring_buf_config(ictx, false);
	}

	ispblk_ynr_config(ictx, ISP_YNR_OUT_Y_DELAY, 128);
	ispblk_cnr_config(ictx, false, false, 255, 0);
	ispblk_pre_ee_config(ictx, false);
	ispblk_ee_config(ictx, false);
#ifdef COVER_WITH_BLACK
	memset(ycur_data, 0, sizeof(ycur_data));
	ispblk_ycur_config(ictx, false, 0, ycur_data);
	ispblk_ycur_enable(ictx, true, 0);
#else
	ispblk_ycur_config(ictx, false, 0, ycur_data);
	ispblk_ycur_enable(ictx, false, 0);
#endif
	ispblk_dci_config(ictx, true, ictx->gamma_tbl_idx, dci_map_lut_50, 0);
	ispblk_ldci_config(ictx, false, 0);

	ispblk_ca_config(ictx, false, 1);
	ispblk_ca_lite_config(ictx, false);

	ispblk_crop_enable(ictx, ISP_BLK_ID_YUV_CROP_Y, false);
	ispblk_crop_enable(ictx, ISP_BLK_ID_YUV_CROP_C, false);
}

static u32 _is_drop_next_frame(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	const enum cvi_isp_fe_chn_num chn_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	uint32_t start_drop_num = ctx->isp_pipe_cfg[raw_num].drop_ref_frm_num;
	uint32_t end_drop_num = start_drop_num + ctx->isp_pipe_cfg[raw_num].drop_frm_cnt;
	u32 frm_num = 0;
	u8 dev_num = 0;

	if (ctx->isp_pipe_cfg[raw_num].is_drop_next_frame) {
		//for tuning_dis, shoudn't trigger preraw;
		if ((ctx->is_multi_sensor) && (!ctx->isp_pipe_cfg[raw_num].is_yuv_sensor)) {
			dev_num = vi_get_dev_num_by_raw(ctx, raw_num);
			if ((tuning_dis[0] > 0) && ((tuning_dis[0] - 1) != dev_num)) {
				vi_pr(VI_DBG, "input buf is not equal to current tuning number\n");
				return 1;
			}
		}

		//if sof_num in [start_sof, end_sof), shoudn't trigger preraw;
		frm_num = vdev->pre_fe_sof_cnt[raw_num][ISP_FE_CH0];

		if ((start_drop_num != 0) && (frm_num >= start_drop_num) && (frm_num < end_drop_num))
			return 1;
	}

	return 0;
}

static void _set_drop_frm_info(
	const struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	struct isp_i2c_data *i2c_data)
{
	struct isp_ctx *ctx = (struct isp_ctx *)(&vdev->ctx);

	ctx->isp_pipe_cfg[raw_num].drop_frm_cnt = i2c_data->drop_frame_cnt;

	ctx->isp_pipe_cfg[raw_num].is_drop_next_frame = true;
	ctx->isp_pipe_cfg[raw_num].drop_ref_frm_num = vdev->pre_fe_sof_cnt[raw_num][ISP_FE_CH0];

	vi_pr(VI_DBG, "raw_%d, drop_ref_frm_num=%d, drop frame=%d\n", raw_num,
				ctx->isp_pipe_cfg[raw_num].drop_ref_frm_num,
				i2c_data->drop_frame_cnt);
}

static void _clear_drop_frm_info(
	const struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num)
{
	struct isp_ctx *ctx = (struct isp_ctx *)(&vdev->ctx);

	ctx->isp_pipe_cfg[raw_num].drop_frm_cnt = 0;
	ctx->isp_pipe_cfg[raw_num].drop_ref_frm_num = 0;
	ctx->isp_pipe_cfg[raw_num].is_drop_next_frame = false;
}

static void _snr_i2c_update(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	struct _isp_snr_i2c_node **_i2c_n,
	const u16 _i2c_num,
	int is_vblank_update)
{
	struct _isp_snr_i2c_node *node;
	struct _isp_snr_i2c_node *next_node;
	struct isp_i2c_data *i2c_data;
	struct isp_i2c_data *next_i2c_data;
	unsigned long flags;
	u16 i = 0, j = 0;
	// 0: Delete Node, 1: Postpone Node, 2: Do nothing
	u16 del_node = 0;
	uint32_t dev_mask = 0;
	// uint32_t cmd = burst_i2c_en ? CVI_SNS_I2C_BURST_QUEUE : CVI_SNS_I2C_WRITE;
	CVI_BOOL no_update = CVI_TRUE;
	u32 fe_frm_num = 0;

	if (vdev->ctx.isp_pipe_cfg[raw_num].is_raw_replay_be || vdev->ctx.isp_pipe_cfg[raw_num].is_patgen_en)
		return;

	//dual sensor case, fe/be frm_num wouldn't be the same when dump frame of rotation.
	if (_is_be_post_online(&vdev->ctx))
		fe_frm_num = vdev->pre_fe_frm_num[raw_num][ISP_FE_CH0] - vdev->dump_frame_number[raw_num];
	else
		fe_frm_num = vdev->pre_fe_frm_num[raw_num][ISP_FE_CH0];

	for (j = 0; j < _i2c_num; j++) {
		node = _i2c_n[j];
		no_update = CVI_TRUE;

		vi_pr(VI_DBG, "raw_num=%d, i2c_num=%d, j=%d, magic_num=%d, fe_frm_num=%d, v_blank_update=%d\n",
				raw_num, _i2c_num, j, node->n.magic_num, fe_frm_num, is_vblank_update);

		//magic num set by ISP team. fire i2c when magic num same as last fe frm num.
		if (((node->n.magic_num == fe_frm_num ||
			 (node->n.magic_num < fe_frm_num && (j + 1) >= _i2c_num)) && (!is_vblank_update)) ||
			 ((node->n.magic_num_vblank  == fe_frm_num ||
			 (node->n.magic_num_vblank  < fe_frm_num && (j + 1) >= _i2c_num)) && (is_vblank_update))) {

			if ((node->n.magic_num != fe_frm_num && !is_vblank_update) ||
				(node->n.magic_num_vblank != fe_frm_num && is_vblank_update)) {
				vi_pr(VI_WARN, "exception handle, send delayed i2c data.\n");
			}

			for (i = 0; i < node->n.regs_num; i++) {
				i2c_data = &node->n.i2c_data[i];

				vi_pr(VI_DBG, "i2cdata[%d]:i2c_addr=0x%x write:0x%x needvblank:%d needupdate:%d\n", i,
				i2c_data->reg_addr, i2c_data->data, i2c_data->vblank_update, i2c_data->update);

				if (i2c_data->update && (i2c_data->dly_frm_num == 0)) {
					if ((i2c_data->vblank_update && is_vblank_update)
					|| (!i2c_data->vblank_update && !is_vblank_update)) {
					vi_snsr_i2c_write(vdev, raw_num, i2c_data);
						i2c_data->update = 0;
						if (burst_i2c_en)
							dev_mask |= BIT(i2c_data->i2c_dev);
						if (i2c_data->drop_frame)
							_set_drop_frm_info(vdev, raw_num, i2c_data);
					} else {
						no_update = CVI_FALSE;
					}
				} else if (i2c_data->update && !(i2c_data->dly_frm_num == 0)) {
					vi_pr(VI_DBG, "addr=0x%x, dly_frm=%d\n",
							i2c_data->reg_addr, i2c_data->dly_frm_num);
					i2c_data->dly_frm_num--;
					del_node = 1;
				}
			}

		} else if ((node->n.magic_num < fe_frm_num && !is_vblank_update) ||
					(node->n.magic_num_vblank < fe_frm_num && is_vblank_update)) {

			if ((j + 1) < _i2c_num) {

				next_node = _i2c_n[j + 1];

				for (i = 0; i < next_node->n.regs_num; i++) {
					next_i2c_data = &next_node->n.i2c_data[i];
					i2c_data = &node->n.i2c_data[i];

					if (i2c_data->update && next_i2c_data->update == 0) {
						next_i2c_data->update = i2c_data->update;
						vi_pr(VI_WARN, "exception handle, i2c node merge, addr: 0x%x\n",
							i2c_data->reg_addr);
					}
				}

				del_node = 0;
			} else {
				// impossible case
			}
		} else {
			del_node = 2;
		}

		if (del_node == 0 && no_update) {
			vi_pr(VI_DBG, "i2c node %d del node and free\n", j);
			spin_lock_irqsave(&snr_node_lock[raw_num], flags);
			list_del_init(&node->list);
			--isp_snr_i2c_queue[raw_num].num_rdy;
			kfree(node);
			spin_unlock_irqrestore(&snr_node_lock[raw_num], flags);
		} else if (del_node == 1) {
			if (is_vblank_update)
				node->n.magic_num_vblank++;
			else
				node->n.magic_num++;
			vi_pr(VI_DBG, "postpone i2c node\n");
		}
	}

	// while (dev_mask) {
	// 	uint32_t tmp = ffs(dev_mask) - 1;

	// 	vip_sys_cmm_cb_i2c(CVI_SNS_I2C_BURST_FIRE, (void *)&tmp);
	// 	dev_mask &= ~BIT(tmp);
	// }
}

static void _isp_snr_cfg_deq_and_fire(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	int needvblank)
{
	struct list_head *pos, *temp;
	struct _isp_snr_i2c_node *i2c_n[VI_MAX_LIST_NUM], *i2c_n_temp[0];
	unsigned long flags;
	u16 i2c_num = 0;
	int i;

	spin_lock_irqsave(&snr_node_lock[raw_num], flags);
	if (needvblank == 0) {
		list_for_each_safe(pos, temp, &isp_snr_i2c_queue[raw_num].list) {
			i2c_n[i2c_num] = list_entry(pos, struct _isp_snr_i2c_node, list);
			i2c_num++;
		}
	} else {
		list_for_each_safe(pos, temp, &isp_snr_i2c_queue[raw_num].list) {
			i2c_n_temp[0] = list_entry(pos, struct _isp_snr_i2c_node, list);
			for (i = 0; i < i2c_n_temp[0]->n.regs_num; i++) {
				if (i2c_n_temp[0]->n.i2c_data[i].vblank_update &&
				    i2c_n_temp[0]->n.i2c_data[i].update) {
					i2c_n[i2c_num] = i2c_n_temp[0];
					i2c_num++;
					break;
				}
			}
		}
	}

	spin_unlock_irqrestore(&snr_node_lock[raw_num], flags);

	_snr_i2c_update(vdev, raw_num, i2c_n, i2c_num, needvblank);
}

static inline void _vi_clear_mmap_fbc_ring_base(struct cvi_vi_dev *vdev, const enum cvi_isp_raw raw_num)
{
	struct isp_ctx *ctx = &vdev->ctx;

	//Clear mmap previous ring base to start addr after first frame done.
	if (ctx->is_3dnr_on && (ctx->isp_pipe_cfg[raw_num].first_frm_cnt == 1)) {
		manr_clear_prv_ring_base(ctx, raw_num);

		if (ctx->is_fbc_on) {
			ispblk_fbc_chg_to_sw_mode(&vdev->ctx, raw_num);
			ispblk_fbc_clear_fbcd_ring_base(&vdev->ctx, raw_num);
		}
	}
}

static inline void _vi_wake_up_preraw_th(struct cvi_vi_dev *vdev, const enum cvi_isp_raw raw_num)
{
	unsigned long flags;
	struct _isp_raw_num_n *n;

	n = kzalloc(sizeof(*n), GFP_ATOMIC);
	if (n == NULL) {
		vi_pr(VI_ERR, "pre_raw_num_q kmalloc size(%zu) fail\n", sizeof(*n));
		return;
	}
	n->raw_num = raw_num;

	spin_lock_irqsave(&raw_num_lock, flags);
	list_add_tail(&n->list, &pre_raw_num_q.list);
	spin_unlock_irqrestore(&raw_num_lock, flags);

	vdev->vi_th[E_VI_TH_PRERAW].flag = 1;
	wake_up(&vdev->vi_th[E_VI_TH_PRERAW].wq);
}

static void _usr_pic_timer_handler(unsigned long data)
{
	struct cvi_vi_dev *vdev = (struct cvi_vi_dev *)usr_pic_timer.data;
	struct isp_ctx *ctx = &vdev->ctx;
	struct isp_buffer *b = NULL;
	enum cvi_isp_raw raw_num = ISP_PRERAW0;
	s8 s8Ret = ISP_SUCCESS;

	if (!(ctx->isp_pipe_cfg[raw_num].is_raw_replay_fe || ctx->isp_pipe_cfg[raw_num].is_raw_replay_be))
		goto EXIT;

	if (!ctx->is_ctrl_inited)
		goto EXIT;

	if (atomic_read(&vdev->isp_streamoff) == 1)
		goto EXIT;

#ifdef PORTING_TEST
	if (!usr_trigger)
		goto EXIT;
	usr_trigger = stop_stream_en ? false : true;
#endif

	if (ctx->isp_pipe_cfg[raw_num].is_raw_replay_fe) {
		if (atomic_read(&vdev->pre_fe_state[raw_num][ISP_FE_CH0]) != ISP_STATE_IDLE)
			goto EXIT;

		if (atomic_read(&vdev->pre_fe_state[raw_num][ISP_FE_CH1]) != ISP_STATE_IDLE &&
			ctx->isp_pipe_cfg[raw_num].is_hdr_on)
			goto EXIT;

		if (ctx->isp_pipe_cfg[raw_num].is_tile) {
			//reconfig rdma for tile dma
			uint32_t splt_fe0_le = ISP_BLK_ID_DMA_CTL_SPLT_FE0_RDMA_LE;
			uint32_t splt_fe0_se = ISP_BLK_ID_DMA_CTL_SPLT_FE0_RDMA_SE;
			uint32_t splt_fe1_le = ISP_BLK_ID_DMA_CTL_SPLT_FE1_RDMA_LE;
			uint32_t splt_fe1_se = ISP_BLK_ID_DMA_CTL_SPLT_FE1_RDMA_SE;
			uint64_t dma_addr = 0;

			if (atomic_read(&vdev->postraw_state) != ISP_STATE_IDLE)
				goto EXIT;

			if (atomic_read(&vdev->pre_fe_state[raw_num + 1][ISP_FE_CH0]) != ISP_STATE_IDLE)
				goto EXIT;

			if (atomic_read(&vdev->pre_fe_state[raw_num + 1][ISP_FE_CH1]) != ISP_STATE_IDLE &&
				ctx->isp_pipe_cfg[raw_num].is_hdr_on)
				goto EXIT;

			dma_addr = ispblk_dma_getaddr(ctx, splt_fe0_le);
			ispblk_dma_config(ctx, raw_num, splt_fe0_le, dma_addr);
			if (ctx->is_dpcm_on)
				dma_addr += (ctx->isp_pipe_cfg[raw_num].csibdg_width * 3) / 4;
			else
				dma_addr += (ctx->isp_pipe_cfg[raw_num].csibdg_width * 3) / 2;
			ispblk_dma_config(ctx, raw_num + 1, splt_fe1_le, dma_addr);

			if (ctx->isp_pipe_cfg[raw_num].is_hdr_on) {
				dma_addr = ispblk_dma_getaddr(ctx, splt_fe0_se);
				ispblk_dma_config(ctx, raw_num, splt_fe0_se, dma_addr);
				if (ctx->is_dpcm_on)
					dma_addr += (ctx->isp_pipe_cfg[raw_num].csibdg_width * 3) / 4;
				else
					dma_addr += (ctx->isp_pipe_cfg[raw_num].csibdg_width * 3) / 2;
				ispblk_dma_config(ctx, raw_num + 1, splt_fe1_se, dma_addr);
			}
		}

		tasklet_hi_schedule(&vdev->job_work);

		s8Ret = _pre_hw_enque(vdev, raw_num, ISP_FE_CH0);
		if (ctx->isp_pipe_cfg[raw_num].is_hdr_on)
			s8Ret |= _pre_hw_enque(vdev, raw_num, ISP_FE_CH1);

		if (ctx->isp_pipe_cfg[raw_num].is_tile) {
			s8Ret |= _pre_hw_enque(vdev, raw_num + 1, ISP_FE_CH0);
			if (ctx->isp_pipe_cfg[raw_num].is_hdr_on)
				s8Ret |= _pre_hw_enque(vdev, raw_num + 1, ISP_FE_CH1);
		}
		if (s8Ret == ISP_SUCCESS)
			_splt_hw_enque(vdev, raw_num);

	} else if (ctx->isp_pipe_cfg[raw_num].is_raw_replay_be) {
		if (atomic_read(&vdev->pre_be_state[ISP_BE_CH0]) != ISP_STATE_IDLE)
			goto EXIT;

		// maybe no used now, a2 used be offline mode for raw replay
		if (_is_fe_be_online(ctx)) {
			if (ctx->isp_pipe_cfg[raw_num].is_hdr_on) {
				b = isp_buf_next(&pre_be_out_q[ISP_RAW_PATH_SE]);
				if (!b) {
					vi_pr(VI_DBG, "pre_be chn_num_%d outbuf is empty\n", ISP_FE_CH1);
					return;
				}

				ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_PRE_RAW_BE_SE, b->addr);
			}

			if (atomic_read(&vdev->isp_raw_dump_en[raw_num]) == 1) { // raw_dump flow
				_isp_fe_be_raw_dump_cfg(vdev, raw_num, 0);
				atomic_set(&vdev->isp_raw_dump_en[raw_num], 3);
			}
		}

		_vi_clear_mmap_fbc_ring_base(vdev, raw_num);

		vi_tuning_gamma_ips_update(ctx, raw_num);
		vi_tuning_clut_update(ctx, raw_num);
		vi_tuning_dci_update(ctx, raw_num);
		vi_tuning_drc_update(ctx, raw_num);

		_post_rgbmap_update(ctx, raw_num, vdev->pre_be_frm_num[raw_num][ISP_BE_CH0]);

		_pre_hw_enque(vdev, raw_num, ISP_BE_CH0);

		_vi_wake_up_preraw_th(vdev, raw_num);
	}

EXIT:
#if (KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE)
	mod_timer(&usr_pic_timer.t, jiffies + vdev->usr_pic_delay);
#else
	mod_timer(&usr_pic_timer, jiffies + vdev->usr_pic_delay);
#endif
}

void usr_pic_time_remove(void)
{
#if (KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE)
	if (timer_pending(&usr_pic_timer.t)) {
		del_timer_sync(&usr_pic_timer.t);
		timer_setup(&usr_pic_timer.t, legacy_timer_emu_func, 0);
#else
	if (timer_pending(&usr_pic_timer)) {
		del_timer_sync(&usr_pic_timer);
		init_timer(&usr_pic_timer);
#endif
	}
}

int usr_pic_timer_init(struct cvi_vi_dev *vdev)
{
#ifdef PORTING_TEST
	usr_trigger = true;
#endif
	usr_pic_time_remove();
	usr_pic_timer.function = _usr_pic_timer_handler;
	usr_pic_timer.data = (uintptr_t)vdev;
#if (KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE)
	usr_pic_timer.t.expires = jiffies + vdev->usr_pic_delay;
	add_timer(&usr_pic_timer.t);
#else
	usr_pic_timer.expires = jiffies + vdev->usr_pic_delay;
	add_timer(&usr_pic_timer);
#endif

	return 0;
}

void _isp_v4l2_event_queue(
	struct video_device *vfd, const u32 type, const u32 frame_num)
{
	struct v4l2_event event = {
		.type = type,
		.u.frame_sync.frame_sequence = frame_num,
	};

	v4l2_event_queue(vfd, &event);
}

void vi_event_queue(struct cvi_vi_dev *vdev, const u32 type, const u32 frm_num)
{
	unsigned long flags;
	struct vi_event_k *ev_k;
#if (KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE)
	struct timespec64 ts;
#endif

	if (type >= VI_EVENT_MAX) {
		vi_pr(VI_ERR, "event queue type(%d) error\n", type);
		return;
	}

	ev_k = kzalloc(sizeof(*ev_k), GFP_ATOMIC);
	if (ev_k == NULL) {
		vi_pr(VI_ERR, "event queue kzalloc size(%zu) fail\n", sizeof(*ev_k));
		return;
	}

	spin_lock_irqsave(&event_lock, flags);
	ev_k->ev.type = type;
	ev_k->ev.frame_sequence = frm_num;

#if (KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE)
	ts = ktime_to_timespec64(ktime_get());
	ev_k->ev.timestamp.tv_sec = ts.tv_sec;
	ev_k->ev.timestamp.tv_nsec = ts.tv_nsec;
#else
	ev_k->ev.timestamp = ktime_to_timeval(ktime_get());
#endif
	list_add_tail(&ev_k->list, &event_q.list);
	spin_unlock_irqrestore(&event_lock, flags);

	wake_up(&vdev->isp_event_wait_q);

	_isp_v4l2_event_queue(&vdev->vnode[0].vdev, type, frm_num);
}

void cvi_isp_dqbuf_list(struct cvi_vi_dev *vdev, const u32 frm_num, const u8 chn_id)
{
	unsigned long flags;
	struct _isp_dqbuf_n *n;

	n = kzalloc(sizeof(struct _isp_dqbuf_n), GFP_ATOMIC);
	if (n == NULL) {
		vi_pr(VI_ERR, "DQbuf kmalloc size(%zu) fail\n", sizeof(struct _isp_dqbuf_n));
		return;
	}
	n->chn_id	= chn_id;
	n->frm_num	= frm_num;
	n->timestamp	= ktime_to_timespec64(ktime_get());

	spin_lock_irqsave(&dq_lock, flags);
	list_add_tail(&n->list, &dqbuf_q[chn_id].list);
	spin_unlock_irqrestore(&dq_lock, flags);
}

static void _isp_yuv_bypass_buf_enq(struct cvi_vi_dev *vdev, const enum cvi_isp_raw raw_num, const u8 buf_chn)
{
	struct isp_ctx *ctx = &vdev->ctx;
	struct cvi_isp_buf *b = NULL;
	struct vb2_buffer *vb2_buf;
	enum ISP_BLK_ID_T dmaid;
	u64 tmp_addr = 0, i = 0;
	u8 hw_chn_num = buf_chn - ctx->raw_chnstr_num[raw_num];

	cvi_isp_rdy_buf_pop(vdev, buf_chn);
	b = cvi_isp_rdy_buf_next(vdev, buf_chn);
	if (b == NULL)
		return;

	vb2_buf = &b->buf.vb2_buf;

	tmp_addr = vb2_dma_contig_plane_dma_addr(vb2_buf, 0);

	vi_pr(VI_DBG, "update chn[%d] yuv bypass outbuf: 0x%llx\n", buf_chn, tmp_addr);

	if (ctx->isp_pipe_cfg[raw_num].is_bt_demux)
		dmaid = csibdg_lite_dma_find_hwid(raw_num, hw_chn_num);
	else
		dmaid = csibdg_dma_find_hwid(raw_num, hw_chn_num);

	if (ctx->isp_pipe_cfg[raw_num].is_422_to_420) {
		for (i = 0; i < 2; i++) {
			tmp_addr = vb2_dma_contig_plane_dma_addr(vb2_buf, i);
			if (vdev->pre_fe_frm_num[raw_num][hw_chn_num] == 0)
				ispblk_dma_yuv_bypass_config(ctx, dmaid + i, tmp_addr, raw_num);
			else
				ispblk_dma_setaddr(ctx, dmaid + i, tmp_addr);
		}
	} else {
		if (vdev->pre_fe_frm_num[raw_num][hw_chn_num] == 0)
			ispblk_dma_yuv_bypass_config(ctx, dmaid, tmp_addr, raw_num);
		else
			ispblk_dma_setaddr(ctx, dmaid, tmp_addr);
	}
}

static void _isp_yuv_bypass_trigger(struct cvi_vi_dev *vdev, const enum cvi_isp_raw raw_num, const u8 hw_chn_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	u8 buf_chn;

	if (atomic_read(&vdev->isp_streamoff) == 0) {
		if (atomic_cmpxchg(&vdev->pre_fe_state[raw_num][hw_chn_num],
					ISP_STATE_IDLE, ISP_STATE_RUNNING) ==
					ISP_STATE_RUNNING) {
			vi_pr(VI_DBG, "fe_%d chn_num_%d is running\n", raw_num, hw_chn_num);
			return;
		}
		buf_chn = ctx->raw_chnstr_num[raw_num] + hw_chn_num;

		_isp_yuv_bypass_buf_enq(vdev, raw_num, buf_chn);
		isp_pre_trig(ctx, raw_num, hw_chn_num);
	}
}

void _vi_postraw_ctrl_setup(struct cvi_vi_dev *vdev)
{
	struct isp_ctx *ctx = &vdev->ctx;
	enum cvi_isp_raw raw_num;
	u8 cfg_post = false;

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ctx->isp_pipe_enable[raw_num])
			continue;

		if (!ctx->isp_pipe_cfg[raw_num].is_yuv_sensor ||
			ctx->isp_pipe_cfg[raw_num].yuv_scene_mode == ISP_YUV_SCENE_ISP) {
			cfg_post = true;
			break;
		}
	}

	if (cfg_post) {
		_isp_rawtop_init(vdev);
		_isp_rgbtop_init(vdev);
		_isp_yuvtop_init(vdev);
	}

	ispblk_isptop_config(ctx);
}

void _vi_pre_fe_ctrl_setup(struct cvi_vi_dev *vdev, enum cvi_isp_raw raw_num)
{
	struct isp_ctx *ictx = &vdev->ctx;
	u32 blc_le_id, blc_se_id, rgbmap_le_id, rgbmap_se_id, wbg_le_id, wbg_se_id;
	struct cif_yuv_swap_s swap = {0};

	if (ictx->isp_pipe_cfg[raw_num].is_yuv_sensor) {//YUV sensor
		if (ictx->isp_pipe_cfg[raw_num].is_422_to_420) {//uyvy to yuyv to 420
			swap.devno = raw_num;
			swap.yc_swap = 1;
			swap.uv_swap = 1;
			// _vi_call_cb(E_MODULE_CIF, CVI_MIPI_SET_YUV_SWAP, &swap);
		}

		if (ictx->isp_pipe_cfg[raw_num].is_bt_demux)
			ispblk_csibdg_lite_config(ictx, raw_num);
		else {
			ispblk_csibdg_yuv_bypass_config(ictx, raw_num);
			if (ictx->isp_pipe_cfg[raw_num].yuv_scene_mode == ISP_YUV_SCENE_ISP) {
				switch (raw_num) {
				case ISP_PRERAW0:
					rgbmap_le_id = ISP_RGBMAP_ID_FE0_LE;
					rgbmap_se_id = ISP_RGBMAP_ID_FE0_SE;
					break;
				case ISP_PRERAW1:
					rgbmap_le_id = ISP_RGBMAP_ID_FE1_LE;
					rgbmap_se_id = ISP_RGBMAP_ID_FE1_SE;
					break;
				case ISP_PRERAW2:
					rgbmap_le_id = ISP_RGBMAP_ID_FE2_LE;
					rgbmap_se_id = ISP_RGBMAP_ID_FE2_SE;
					break;
				case ISP_PRERAW3:
					rgbmap_le_id = ISP_RGBMAP_ID_FE3_LE;
					rgbmap_se_id = ISP_RGBMAP_ID_FE3_SE;
					break;
				case ISP_PRERAW4:
					rgbmap_le_id = ISP_RGBMAP_ID_FE4_LE;
					rgbmap_se_id = ISP_RGBMAP_ID_FE4_SE;
					break;
				case ISP_PRERAW5:
					rgbmap_le_id = ISP_RGBMAP_ID_FE5_LE;
					rgbmap_se_id = ISP_RGBMAP_ID_FE5_SE;
					break;
				default:
					break;
				}
				ispblk_rgbmap_config(ictx, rgbmap_le_id, ictx->is_3dnr_on);
				ispblk_rgbmap_config(ictx, rgbmap_se_id, false);

				ispblk_preraw_fe_config(ictx, raw_num);
			}
		}

		if (ictx->isp_pipe_cfg[raw_num].is_offline_scaler) { //vi vpss offline mode
			uint8_t chn_str = ictx->raw_chnstr_num[raw_num];
			uint8_t total_chn = chn_str + ictx->isp_pipe_cfg[raw_num].muxMode + 1;

			for (; chn_str < total_chn; chn_str++)
				_isp_yuv_bypass_buf_enq(vdev, raw_num, chn_str);
		}
	} else { //RGB sensor
		switch (raw_num) {
		case ISP_PRERAW0:
			blc_le_id    = ISP_BLC_ID_FE0_LE;
			blc_se_id    = ISP_BLC_ID_FE0_SE;
			wbg_le_id    = ISP_WBG_ID_FE0_LE;
			wbg_se_id    = ISP_WBG_ID_FE0_SE;
			rgbmap_le_id = ISP_RGBMAP_ID_FE0_LE;
			rgbmap_se_id = ISP_RGBMAP_ID_FE0_SE;
			break;
		case ISP_PRERAW1:
			blc_le_id    = ISP_BLC_ID_FE1_LE;
			blc_se_id    = ISP_BLC_ID_FE1_SE;
			wbg_le_id    = ISP_WBG_ID_FE1_LE;
			wbg_se_id    = ISP_WBG_ID_FE1_SE;
			rgbmap_le_id = ISP_RGBMAP_ID_FE1_LE;
			rgbmap_se_id = ISP_RGBMAP_ID_FE1_SE;
			break;
		case ISP_PRERAW2:
			blc_le_id    = ISP_BLC_ID_FE2_LE;
			blc_se_id    = ISP_BLC_ID_FE2_SE;
			wbg_le_id    = ISP_WBG_ID_FE2_LE;
			wbg_se_id    = ISP_WBG_ID_FE2_SE;
			rgbmap_le_id = ISP_RGBMAP_ID_FE2_LE;
			rgbmap_se_id = ISP_RGBMAP_ID_FE2_SE;
			break;
		case ISP_PRERAW3:
			blc_le_id    = ISP_BLC_ID_FE3_LE;
			blc_se_id    = ISP_BLC_ID_FE3_SE;
			wbg_le_id    = ISP_WBG_ID_FE3_LE;
			wbg_se_id    = ISP_WBG_ID_FE3_SE;
			rgbmap_le_id = ISP_RGBMAP_ID_FE3_LE;
			rgbmap_se_id = ISP_RGBMAP_ID_FE3_SE;
			break;
		case ISP_PRERAW4:
			blc_le_id    = ISP_BLC_ID_FE4_LE;
			blc_se_id    = ISP_BLC_ID_FE4_SE;
			wbg_le_id    = ISP_WBG_ID_FE4_LE;
			wbg_se_id    = ISP_WBG_ID_FE4_SE;
			rgbmap_le_id = ISP_RGBMAP_ID_FE4_LE;
			rgbmap_se_id = ISP_RGBMAP_ID_FE4_SE;
			break;
		case ISP_PRERAW5:
			blc_le_id    = ISP_BLC_ID_FE5_LE;
			blc_se_id    = ISP_BLC_ID_FE5_SE;
			wbg_le_id    = ISP_WBG_ID_FE5_LE;
			wbg_se_id    = ISP_WBG_ID_FE5_SE;
			rgbmap_le_id = ISP_RGBMAP_ID_FE5_LE;
			rgbmap_se_id = ISP_RGBMAP_ID_FE5_SE;
			break;
		default:
			break;
		}

		ispblk_preraw_fe_config(ictx, raw_num);
		ispblk_csibdg_config(ictx, raw_num);
		ispblk_csibdg_crop_update(ictx, raw_num, true);

		ispblk_blc_set_gain(ictx, blc_le_id, 0x40f, 0x419, 0x419, 0x405);
		ispblk_blc_enable(ictx, blc_le_id, false, false);

		ispblk_wbg_config(ictx, wbg_le_id, 0x400, 0x400, 0x400);
		ispblk_wbg_enable(ictx, wbg_le_id, false, false);

		ispblk_rgbmap_config(ictx, rgbmap_le_id, ictx->is_3dnr_on);

		if (ictx->isp_pipe_cfg[raw_num].is_hdr_on) {
			ispblk_blc_set_gain(ictx, blc_se_id, 0x40f, 0x419, 0x419, 0x405);
			ispblk_blc_enable(ictx, blc_se_id, false, false);

			ispblk_rgbmap_config(ictx, rgbmap_se_id, ictx->is_3dnr_on);

			ispblk_wbg_config(ictx, wbg_se_id, 0x400, 0x400, 0x400);
			ispblk_wbg_enable(ictx, wbg_se_id, false, false);
		} else {
			ispblk_rgbmap_config(ictx, rgbmap_se_id, false);
		}
	}
}

void _vi_splt_ctrl_setup(struct cvi_vi_dev *vdev, enum cvi_isp_raw raw_num)
{
	struct isp_ctx *ctx = &vdev->ctx;

	if (raw_num != ISP_PRERAW0)
		return;

	if (ctx->isp_pipe_cfg[raw_num].is_raw_replay_fe ||
	    ctx->isp_pipe_cfg[raw_num].is_tile ||
	    line_spliter_en) {
		ispblk_splt_config(ctx, raw_num, true);
	} else {
		ispblk_splt_config(ctx, raw_num, false);
	}

	if (ctx->isp_pipe_cfg[raw_num].is_raw_replay_fe) {
		ispblk_splt_wdma_ctrl_config(ctx, ISP_BLK_ID_SPLT_FE0_WDMA, false);
		ispblk_splt_wdma_ctrl_config(ctx, ISP_BLK_ID_SPLT_FE1_WDMA, false);

		ispblk_splt_rdma_ctrl_config(ctx, ISP_BLK_ID_SPLT_FE0_RDMA_LE, true);
		ispblk_splt_rdma_ctrl_config(ctx, ISP_BLK_ID_SPLT_FE0_RDMA_SE,
						ctx->isp_pipe_cfg[raw_num].is_hdr_on);

		ispblk_splt_rdma_ctrl_config(ctx, ISP_BLK_ID_SPLT_FE1_RDMA_LE,
						ctx->isp_pipe_cfg[raw_num].is_tile);
		ispblk_splt_rdma_ctrl_config(ctx, ISP_BLK_ID_SPLT_FE1_RDMA_SE,
						ctx->isp_pipe_cfg[raw_num].is_tile &&
						ctx->isp_pipe_cfg[raw_num].is_hdr_on);
	} else {
		ispblk_splt_wdma_ctrl_config(ctx, ISP_BLK_ID_SPLT_FE0_WDMA, false);
		ispblk_splt_wdma_ctrl_config(ctx, ISP_BLK_ID_SPLT_FE1_WDMA, false);

		ispblk_splt_rdma_ctrl_config(ctx, ISP_BLK_ID_SPLT_FE0_RDMA_LE, false);
		ispblk_splt_rdma_ctrl_config(ctx, ISP_BLK_ID_SPLT_FE0_RDMA_SE, false);

		ispblk_splt_rdma_ctrl_config(ctx, ISP_BLK_ID_SPLT_FE1_RDMA_LE, false);
		ispblk_splt_rdma_ctrl_config(ctx, ISP_BLK_ID_SPLT_FE1_RDMA_SE, false);
	}
}

void _vi_ctrl_init(enum cvi_isp_raw raw_num, struct cvi_vi_dev *vdev)
{
	struct isp_ctx *ictx = &vdev->ctx;
	bool is_ctrl_inited = true;
	uint8_t first_raw_num = vi_get_first_raw_num(ictx);

	if (ictx->is_ctrl_inited)
		return;

	if (vdev->snr_info[raw_num].snr_fmt.img_size[0].active_w != 0) { //MW config snr_info flow
		ictx->isp_pipe_cfg[raw_num].csibdg_width  = vdev->snr_info[raw_num].snr_fmt.img_size[0].width;
		ictx->isp_pipe_cfg[raw_num].csibdg_height = vdev->snr_info[raw_num].snr_fmt.img_size[0].height;
		ictx->isp_pipe_cfg[raw_num].max_width     = vdev->snr_info[raw_num].snr_fmt.img_size[0].max_width;
		ictx->isp_pipe_cfg[raw_num].max_height    = vdev->snr_info[raw_num].snr_fmt.img_size[0].max_height;

		ictx->isp_pipe_cfg[raw_num].crop.w = vdev->snr_info[raw_num].snr_fmt.img_size[0].active_w;
		ictx->isp_pipe_cfg[raw_num].crop.h = vdev->snr_info[raw_num].snr_fmt.img_size[0].active_h;
		ictx->isp_pipe_cfg[raw_num].crop.x = vdev->snr_info[raw_num].snr_fmt.img_size[0].start_x;
		ictx->isp_pipe_cfg[raw_num].crop.y = vdev->snr_info[raw_num].snr_fmt.img_size[0].start_y;

		if (vdev->snr_info[raw_num].snr_fmt.frm_num > 1) { //HDR
			ictx->isp_pipe_cfg[raw_num].crop_se.w = ictx->isp_pipe_cfg[raw_num].crop.w;
			ictx->isp_pipe_cfg[raw_num].crop_se.h = ictx->isp_pipe_cfg[raw_num].crop.h;
			ictx->isp_pipe_cfg[raw_num].crop_se.x = ictx->isp_pipe_cfg[raw_num].crop.x;
			ictx->isp_pipe_cfg[raw_num].crop_se.y = ictx->isp_pipe_cfg[raw_num].crop.y;
			ictx->isp_pipe_cfg[raw_num].is_hdr_on = true;
			vdev->ctx.is_hdr_on = true;
			vi_pr(VI_INFO, "raw%d hdr on, se_w_h(%d:%d)\n", raw_num,
				ictx->isp_pipe_cfg[raw_num].crop_se.w,
				ictx->isp_pipe_cfg[raw_num].crop_se.h);
		}

		ictx->rgb_color_mode[raw_num] = vdev->snr_info[raw_num].color_mode;

		if ((ictx->rgb_color_mode[raw_num] >= ISP_BAYER_TYPE_GRGBI) &&
		    (ictx->rgb_color_mode[raw_num] <= ISP_BAYER_TYPE_IGBGR)) {
			ictx->isp_pipe_cfg[raw_num].is_rgbir_sensor = true;
		}
		vi_pr(VI_INFO, "csibdg_w_h(%d:%d) max_w_h(%d:%d) crop_w_h(%d:%d)\n",
			ictx->isp_pipe_cfg[raw_num].csibdg_width, ictx->isp_pipe_cfg[raw_num].csibdg_height,
			ictx->isp_pipe_cfg[raw_num].max_width, ictx->isp_pipe_cfg[raw_num].max_height,
			ictx->isp_pipe_cfg[raw_num].crop.w, ictx->isp_pipe_cfg[raw_num].crop.h);
	}

	if (ictx->isp_pipe_cfg[raw_num].is_patgen_en ||
	    ictx->isp_pipe_cfg[raw_num].is_raw_replay_fe ||
	    ictx->isp_pipe_cfg[raw_num].is_raw_replay_be) {
		ictx->isp_pipe_cfg[raw_num].crop.w = vdev->usr_crop.width;
		ictx->isp_pipe_cfg[raw_num].crop.h = vdev->usr_crop.height;
		ictx->isp_pipe_cfg[raw_num].crop.x = vdev->usr_crop.left;
		ictx->isp_pipe_cfg[raw_num].crop.y = vdev->usr_crop.top;
		ictx->isp_pipe_cfg[raw_num].crop_se.w = vdev->usr_crop.width;
		ictx->isp_pipe_cfg[raw_num].crop_se.h = vdev->usr_crop.height;
		ictx->isp_pipe_cfg[raw_num].crop_se.x = vdev->usr_crop.left;
		ictx->isp_pipe_cfg[raw_num].crop_se.y = vdev->usr_crop.top;

		ictx->isp_pipe_cfg[raw_num].csibdg_width  = vdev->usr_fmt.width;
		ictx->isp_pipe_cfg[raw_num].csibdg_height = vdev->usr_fmt.height;
		ictx->isp_pipe_cfg[raw_num].max_width     = vdev->usr_fmt.width;
		ictx->isp_pipe_cfg[raw_num].max_height    = vdev->usr_fmt.height;

		ictx->rgb_color_mode[raw_num] = vdev->usr_fmt.code;

		ictx->isp_pipe_cfg[raw_num].is_hdr_on = false;
		vdev->ctx.is_hdr_on = false;
		vi_pr(VI_INFO, "csi_bdg=%d:%d, post_crop=%d:%d:%d:%d\n",
				vdev->usr_fmt.width, vdev->usr_fmt.height,
				vdev->usr_crop.width, vdev->usr_crop.height,
				vdev->usr_crop.left, vdev->usr_crop.top);
	}

	ictx->isp_pipe_cfg[raw_num].post_img_w = ictx->isp_pipe_cfg[raw_num].crop.w;
	ictx->isp_pipe_cfg[raw_num].post_img_h = ictx->isp_pipe_cfg[raw_num].crop.h;

	/* use csibdg crop */
	ictx->crop_x = 0;
	ictx->crop_y = 0;
	ictx->crop_se_x = 0;
	ictx->crop_se_y = 0;

	if (!ictx->isp_pipe_cfg[raw_num].is_yuv_sensor) {
		//Postraw out size
		ictx->img_width = ictx->isp_pipe_cfg[first_raw_num].crop.w;
		ictx->img_height = ictx->isp_pipe_cfg[first_raw_num].crop.h;
	}

	if ((ictx->rgb_color_mode[raw_num] >= ISP_BAYER_TYPE_GRGBI) &&
	    (ictx->rgb_color_mode[raw_num] <= ISP_BAYER_TYPE_IGBGR)) {
		ictx->isp_pipe_cfg[raw_num].is_rgbir_sensor = true;
	}

	vi_pr(VI_INFO, "raw_num(%d) color_mode(%d) csibdg_w_h(%d:%d) max_w_h(%d:%d) crop_x_y_w_h(%d:%d:%d:%d)\n",
		raw_num, ictx->rgb_color_mode[raw_num],
		ictx->isp_pipe_cfg[raw_num].csibdg_width, ictx->isp_pipe_cfg[raw_num].csibdg_height,
		ictx->isp_pipe_cfg[raw_num].max_width, ictx->isp_pipe_cfg[raw_num].max_height,
		ictx->isp_pipe_cfg[raw_num].crop.x, ictx->isp_pipe_cfg[raw_num].crop.y,
		ictx->isp_pipe_cfg[raw_num].crop.w, ictx->isp_pipe_cfg[raw_num].crop.h);
	if (raw_num == first_raw_num) {
		if (_is_fe_be_online(ictx) && ictx->is_slice_buf_on)
			vi_calculate_slice_buf_setting(ictx, raw_num);

		isp_init(ictx);
	}

	ictx->isp_pipe_cfg[raw_num].is_ctrl_inited = true;
	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ictx->isp_pipe_enable[raw_num])
			continue;

		if (!ictx->isp_pipe_cfg[raw_num].is_ctrl_inited)
			is_ctrl_inited = false;
	}

	if (is_ctrl_inited)
		ictx->is_ctrl_inited = true;
}

void _vi_scene_ctrl(struct cvi_vi_dev *vdev)
{
	struct isp_ctx *ctx = &vdev->ctx;
	enum cvi_isp_raw raw_num = ISP_PRERAW0;
	uint8_t rgb_snsr_num = 0;

	if (ctx->is_ctrl_inited) {
		return;
	}

	if (gViCtx->total_dev_num >= 2) { //Multi sensor scenario
		ctx->is_multi_sensor = true;

		for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
			if (!ctx->isp_pipe_enable[raw_num])
				continue;
			if (!ctx->isp_pipe_cfg[raw_num].is_yuv_sensor)
				rgb_snsr_num++;
		}

		if (rgb_snsr_num == 1) { //rgb+yuv
			ctx->is_offline_be = false;
			ctx->is_offline_postraw = true;
			ctx->is_slice_buf_on = true;
			/*
			// if 422to420, chnAttr must be NV21
			if (ctx->isp_pipe_cfg[ISP_PRERAW1].muxMode == VI_WORK_MODE_1Multiplex &&
				gViCtx->chnAttr[ctx->rawb_chnstr_num].enPixelFormat == PIXEL_FORMAT_NV21)
				ctx->isp_pipe_cfg[ISP_PRERAW1].is_422_to_420  = true;
			*/
			RGBMAP_BUF_IDX = 2;
		} else {
			ctx->is_offline_be = true;
			ctx->is_offline_postraw = false;
			ctx->is_slice_buf_on = false;
			ctx->is_fbc_on = false;
			/*
			if (ctx->isp_pipe_cfg[ISP_PRERAW2].is_yuv_sensor &&
				ctx->isp_pipe_cfg[ISP_PRERAW2].muxMode == VI_WORK_MODE_1Multiplex &&
				gViCtx->chnAttr[ctx->rawb_chnstr_num].enPixelFormat == PIXEL_FORMAT_NV21)
				ctx->isp_pipe_cfg[ISP_PRERAW2].is_422_to_420  = true;
			*/
			//Only single sensor with non-tile can use two rgbmap buf, two sensors need 3 rgbmap
			RGBMAP_BUF_IDX = 3;
		}
	} else { //Single sensor
		raw_num = vi_get_first_raw_num(ctx);
		ctx->is_multi_sensor = false;

		//for 585 wdr need use 3 rgbmap, fe ch0 after sof
		RGBMAP_BUF_IDX = 3;

#ifndef PORTING_TEST

		if (!ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) { // rgb sensor
			if ((vdev->snr_info[raw_num].snr_fmt.img_size[0].active_w > 4608) || tile_en) {
				ctx->is_tile = true;
				ctx->is_fbc_on = false;
				ctx->isp_pipe_cfg[raw_num].is_tile = true;
				ctx->isp_pipe_cfg[raw_num + 1].is_tile = true;
			} else if (vdev->snr_info[raw_num].snr_fmt.img_size[0].active_w <= 4608 &&
				   vdev->snr_info[raw_num].snr_fmt.img_size[0].active_h <= 2160) {
				ctx->is_slice_buf_on = false;
			}

			if (ctx->isp_pipe_cfg[raw_num].is_raw_replay_fe ||
			    ctx->isp_pipe_cfg[raw_num].is_tile) { // splt->dram->fe0~1->dram->be->post
				ctx->is_offline_be = true;
				ctx->is_offline_postraw = false;
				ctx->is_slice_buf_on = false;
				RGBMAP_BUF_IDX = 3;
			} else if (ctx->isp_pipe_cfg[raw_num].is_raw_replay_be) { // dram->be->post
				ctx->is_offline_be = true;
				ctx->is_offline_postraw = false;
				ctx->is_slice_buf_on = false;
				ctx->rgbmap_prebuf_idx = 0;
				RGBMAP_BUF_IDX = 3;
			} else { // fe->be->dram/slice->post
				//Only single sensor with non-tile can use two rgbmap buf
				RGBMAP_BUF_IDX = 2;
			}
		} else { // yuv sensor
			ctx->isp_pipe_cfg[raw_num].is_offline_scaler = true;
			ctx->isp_pipe_cfg[raw_num].is_422_to_420  = false;

			if (ctx->isp_pipe_cfg[raw_num].muxMode > 0 &&
			    ctx->isp_pipe_cfg[raw_num].infMode >= VI_MODE_BT656 &&
			    ctx->isp_pipe_cfg[raw_num].infMode <= VI_MODE_BT1120_INTERLEAVED) {
				ctx->isp_pipe_cfg[raw_num].is_bt_demux = true;
				if (ctx->isp_pipe_cfg[raw_num].yuv_scene_mode == ISP_YUV_SCENE_ISP) {
					vi_pr(VI_WARN, "bt_demux sensor switch scene_mode to YUV_SCENE_ONLINE\n");
					ctx->isp_pipe_cfg[raw_num].yuv_scene_mode = ISP_YUV_SCENE_ONLINE;
				}
			}

			ctx->is_offline_be = true;
			ctx->is_offline_postraw = false;
			ctx->is_slice_buf_on = false;
		}
#else
		if (!ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) { // rgb sensor
			if ((ctx->isp_pipe_cfg[raw_num].is_raw_replay_fe &&
			     vdev->usr_fmt.width > 4608) ||
			    (!ctx->isp_pipe_cfg[raw_num].is_raw_replay_fe &&
			     vdev->snr_info[raw_num].snr_fmt.img_size[0].active_w > 4608) ||
			    (tile_en)) {
				ctx->is_tile = true;
				ctx->is_fbc_on = false;
				ctx->isp_pipe_cfg[raw_num].is_tile = true;
				ctx->isp_pipe_cfg[raw_num + 1].is_tile = true;
			}
		}
#endif
		if (!ctx->is_3dnr_on)
			ctx->isp_pipe_cfg[raw_num].is_tnr_ai_isp = false;

		if (ctx->isp_pipe_cfg[raw_num].is_tnr_ai_isp)
			ctx->is_fbc_on = false; //send uncompressed data to AI

		vi_pr(VI_INFO, "is_3dnr_on[%d], is_fbc_on[%d], is_tile[%d]\n",
				ctx->is_3dnr_on, ctx->is_fbc_on, ctx->is_tile);
	}

	if (line_spliter_en) {
		if (ctx->is_multi_sensor) {
			vi_pr(VI_WARN, "force line spliter not support multi sensor now!\n");
			line_spliter_en = 0;
		} else if (ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) {
			vi_pr(VI_WARN, "force line spliter not support yuv sensor now!\n");
			line_spliter_en = 0;
		} else if (ctx->isp_pipe_cfg[raw_num].is_raw_replay_fe ||
			   ctx->isp_pipe_cfg[raw_num].is_raw_replay_be ||
			   ctx->isp_pipe_cfg[raw_num].is_raw_ai_isp ||
			   ctx->isp_pipe_cfg[raw_num].is_tile) {
			vi_pr(VI_WARN, "raw_raplay/ai_isp_raw/tile mode is already on line spliter mode!\n");
			line_spliter_en = 0;
		}

		// force line spiliter mode always go through splt->fe0->dram->be->post
		if (line_spliter_en) {
			ctx->is_offline_be = true;
			ctx->is_offline_postraw = false;
			ctx->is_slice_buf_on = false;
			RGBMAP_BUF_IDX = 2;
		}
	}

	if (!sbm_en)
		ctx->is_slice_buf_on = false;

	if (rgbmap_sbm_en && ctx->is_slice_buf_on)
		ctx->is_rgbmap_sbm_on = true;

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!vdev->ctx.isp_pipe_enable[raw_num])
			continue;

		ctx->raw_chnstr_num[raw_num] = ctx->total_chn_num;

		if (!ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) {
			ctx->total_chn_num++;

			vi_pr(VI_INFO, "raw_num=%d, rgb, chnstr_num=%d\n",
					raw_num, ctx->raw_chnstr_num[raw_num]);
		} else {
			ctx->total_chn_num += ctx->isp_pipe_cfg[raw_num].muxMode + 1;

			vi_pr(VI_INFO, "raw_num=%d, yuv, chnstr_num=%d, chn_num=%d\n",
					raw_num, ctx->raw_chnstr_num[raw_num],
					ctx->isp_pipe_cfg[raw_num].muxMode + 1);
		}
	}

	vi_pr(VI_INFO, "Total_chn_num=%d, is_multi_sensor[%d]\n", ctx->total_chn_num, ctx->is_multi_sensor);
}

static void _vi_suspend(struct cvi_vi_dev *vdev)
{
	struct cvi_vi_ctx *pviProcCtx = NULL;
	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	pviProcCtx = (struct cvi_vi_ctx *)(vdev->shared_mem);

	if (pviProcCtx->vi_stt == VI_SUSPEND) {
		for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++)
			isp_streaming(&vdev->ctx, false, raw_num);
		_vi_sw_init(vdev);
#ifndef FPGA_PORTING
		_vi_clk_ctrl(vdev, false);
#endif
	}
}

static int _vi_resume(struct cvi_vi_dev *vdev)
{
	struct cvi_vi_ctx *pviProcCtx = NULL;

	pviProcCtx = (struct cvi_vi_ctx *)(vdev->shared_mem);

	if (pviProcCtx->vi_stt == VI_SUSPEND) {
		pviProcCtx->vi_stt = VI_RUNNING;
	}

	return 0;
}

void _viBWCalSet(struct cvi_vi_dev *vdev)
{
	struct isp_ctx *ctx = &vdev->ctx;
	struct cvi_vi_ctx *pviProcCtx = NULL;
	void __iomem *bw_limiter;
	u64 bwladdr[ISP_BW_LIMIT_MAX] = {0x0A074020, 0x0A072020, 0x0A078020}; // rdma, wdma0, wdma1
	u32 data_size[ISP_BW_LIMIT_MAX] = {0}, BW[ISP_PRERAW_MAX][ISP_BW_LIMIT_MAX] = {0};
	u32 total_bw = 0, bwlwin = 0, bwltxn = 0, margin = 125, fps = 25;
	u32 def_bwltxn = 4, def_fps = 25;
	u32 width, height;
	u8 i, yuv_chn_num;
	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	pviProcCtx = (struct cvi_vi_ctx *)(vdev->shared_mem);

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!ctx->isp_pipe_enable[raw_num])
			continue;

		fps = pviProcCtx->devAttr[raw_num].snrFps ?
			pviProcCtx->devAttr[raw_num].snrFps :
			def_fps;
		width = ctx->isp_pipe_cfg[raw_num].crop.w;
		height = ctx->isp_pipe_cfg[raw_num].crop.h;

		if (!ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) {//RGB sensor
			if (!ctx->isp_pipe_cfg[raw_num].is_hdr_on) {
				data_size[ISP_BW_LIMIT_RDMA] = (411 * width * height) / 128 + 50052;
				data_size[ISP_BW_LIMIT_WDMA0] = (396 * width * height) / 128 + 8160;
				data_size[ISP_BW_LIMIT_WDMA1] = (391 * width * height) / 128 + 21536;
			} else {
				data_size[ISP_BW_LIMIT_RDMA] = (630 * width * height) / 128 + 50052;
				data_size[ISP_BW_LIMIT_WDMA0] = (792 * width * height) / 128 + 8160;
				data_size[ISP_BW_LIMIT_WDMA1] = (394 * width * height) / 128 + 38496;
			}
		} else { //YUV sensor
			yuv_chn_num = ctx->isp_pipe_cfg[raw_num].muxMode + 1;
			data_size[ISP_BW_LIMIT_RDMA] = (192 * yuv_chn_num * width * height) / 128;
			data_size[ISP_BW_LIMIT_WDMA0] = (192 * yuv_chn_num * width * height) / 128;
			data_size[ISP_BW_LIMIT_WDMA1] = 0;
		}

		for (i = 0; i < ISP_BW_LIMIT_MAX; ++i) {
			BW[raw_num][i] = (fps * data_size[i]) / 1000000 + 1;
		}
	}

	// TODO
	// just restrain RDMA now, WDMA0/WDMA1 wait for Brian
	// for (i = 0; i < BW_LIMIT_MAX; ++i) {
	for (i = 0; i < 1; ++i) {
		total_bw = 0;

		for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
			total_bw += BW[raw_num][i];
		}

		for (bwltxn = def_bwltxn; bwltxn > 1; --bwltxn) {
			bwlwin = bwltxn * 256000 / ((((total_bw * 33) / 10) * margin) / 100);
			if (bwlwin <= 1024)
				break;
		}

		bw_limiter = ioremap(bwladdr[i], 0x4);
		iowrite32(((bwltxn << 10) | bwlwin), bw_limiter);
		vi_pr(VI_INFO, "isp %s bw_limiter=0x%x, BW=%d, bwltxn=%d, bwlwin=%d\n",
				(i == 0) ? "rdma" : ((i == 1) ? "wdma0" : "wdma1"),
				ioread32(bw_limiter), total_bw, bwltxn, bwlwin);
		iounmap(bw_limiter);
	}
}

int vi_start_streaming(struct cvi_vi_dev *vdev)
{
	enum cvi_isp_raw raw_num = ISP_PRERAW0;
	int rc = 0;

	vi_pr(VI_INFO, "+\n");

	if (_vi_resume(vdev) != 0) {
		vi_pr(VI_ERR, "vi resume failed\n");
		return -1;
	}

	_vi_mempool_reset();
	rc = vi_tuning_buf_setup(&vdev->ctx);
	if (rc < 0) {
		vi_pr(VI_ERR, "vi alloc tuning buf fail!\n");
		return -1;
	}
	vi_tuning_buf_clear();

	//SW workaround to disable csibdg enable first due to csibdg enable is on as default.
	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++)
		isp_streaming(&vdev->ctx, false, raw_num);

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (vdev->ctx.isp_pipe_cfg[raw_num].is_raw_ai_isp) {
			vi_pr(VI_INFO, "raw_num_%d is raw_ai_isp\n", raw_num);
			++vdev->tpu_thread_num;
			if (vdev->tpu_thread_num == 1) {
				vdev->tpu_thd_bind[raw_num] = E_VI_TH_RUN_TPU1;
			} else if (vdev->tpu_thread_num == 2) {
				vdev->tpu_thd_bind[raw_num] = E_VI_TH_RUN_TPU2;
			} else {
				vi_pr(VI_ERR, "only support 2 tpu thread\n");
				break;
			}

			if (vi_create_thread(vdev, vdev->tpu_thd_bind[raw_num])) {
				vi_pr(VI_ERR, "Failed to create tpu thread\n");
			}
		}
	}

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		atomic_set(&vdev->post_dq_flag[raw_num], 1);
		if (!vdev->ctx.isp_pipe_enable[raw_num])
			continue;
		vi_pr(VI_INFO, "pipe_enable_%d, hdr(%d),stagger_vsync(%d)\n", raw_num,
				vdev->ctx.isp_pipe_cfg[raw_num].is_hdr_on,
				vdev->ctx.isp_pipe_cfg[raw_num].is_stagger_vsync);

		if (raw_num == ISP_PRERAW0) {
			_vi_splt_ctrl_setup(vdev, raw_num);
			if (vdev->ctx.isp_pipe_cfg[raw_num].is_tile) {
				vdev->ctx.rgb_color_mode[raw_num + 1] =
						vdev->ctx.rgb_color_mode[raw_num];
				vdev->ctx.isp_pipe_cfg[raw_num + 1].is_stagger_vsync =
						vdev->ctx.isp_pipe_cfg[raw_num].is_stagger_vsync;
			}
		}
		if (!vdev->ctx.isp_pipe_cfg[raw_num].is_raw_replay_be)
			_vi_pre_fe_ctrl_setup(vdev, raw_num);
		if (!vdev->ctx.is_multi_sensor) { //only single sensor maybe break
			if (_is_all_online(&vdev->ctx) ||
				(_is_fe_be_online(&vdev->ctx) && vdev->ctx.is_slice_buf_on)) {
				vi_pr(VI_INFO, "on-the-fly mode or slice_buffer is on\n");
				break;
			}
		}

		if (!vdev->ctx.isp_pipe_cfg[raw_num].is_raw_replay_fe &&
		    !vdev->ctx.isp_pipe_cfg[raw_num].is_raw_replay_be) {
			if (!vdev->ctx.isp_pipe_cfg[raw_num].is_yuv_sensor) { //RGB sensor
				isp_pre_trig(&vdev->ctx, raw_num, ISP_FE_CH0);
				if (vdev->ctx.isp_pipe_cfg[raw_num].is_hdr_on)
					isp_pre_trig(&vdev->ctx, raw_num, ISP_FE_CH1);
			} else { //YUV sensor
				u8 chn_str = 0;
				u8 total_chn = vdev->ctx.isp_pipe_cfg[raw_num].muxMode + 1;

				for (; chn_str < total_chn; chn_str++)
					isp_pre_trig(&vdev->ctx, raw_num, chn_str);
			}
		}
	}

	_vi_preraw_be_init(vdev);
	_vi_postraw_ctrl_setup(vdev);
	_vi_dma_setup(&vdev->ctx);
	_vi_dma_set_sw_mode(&vdev->ctx);

	vi_pr(VI_INFO, "ISP scene path, be_off=%d, post_off=%d, slice_buff_on=%d, 3dnr_on=%d, fbc_on=%d\n",
			vdev->ctx.is_offline_be, vdev->ctx.is_offline_postraw, vdev->ctx.is_slice_buf_on,
			vdev->ctx.is_3dnr_on, vdev->ctx.is_fbc_on);

	raw_num = vi_get_first_raw_num(&vdev->ctx);
	if (_is_fe_be_online(&vdev->ctx)) {
		if (vdev->ctx.isp_pipe_cfg[raw_num].is_offline_scaler) { //offline mode
			_postraw_outbuf_enq(vdev, raw_num);

			isp_post_trig(&vdev->ctx, raw_num);

			if (!vdev->ctx.isp_pipe_cfg[raw_num].is_raw_replay_be) {
				if (!vdev->ctx.isp_pipe_cfg[raw_num].is_yuv_sensor) { //RGB sensor
					isp_pre_trig(&vdev->ctx, raw_num, ISP_FE_CH0);
					if (vdev->ctx.isp_pipe_cfg[raw_num].is_hdr_on)
						isp_pre_trig(&vdev->ctx, raw_num, ISP_FE_CH1);
				}
			}

			atomic_set(&vdev->pre_be_state[ISP_BE_CH0], ISP_STATE_RUNNING);
		}
	} else if (_is_be_post_online(&vdev->ctx)) {
		if (!vdev->ctx.isp_pipe_cfg[raw_num].is_raw_replay_be &&
		    !vdev->ctx.isp_pipe_cfg[raw_num].is_raw_replay_fe) { //not rawreplay
			if (vdev->ctx.isp_pipe_cfg[raw_num].is_tile ||
			    line_spliter_en) {
				isp_splt_trig(&vdev->ctx, raw_num);
			}
		}
	}

#ifndef PORTING_TEST
	// _viBWCalSet(vdev);
#endif

#ifdef PORTING_TEST
	vi_ip_test_cases_init(&vdev->ctx);
#endif

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!vdev->ctx.isp_pipe_enable[raw_num])
			continue;
		isp_streaming(&vdev->ctx, true, raw_num);
	}

	atomic_set(&vdev->isp_streamoff, 0);

	vi_pr(VI_DBG, "-\n");

	return rc;
}

/* abort streaming and wait for last buffer */
int vi_stop_streaming(struct cvi_vi_dev *vdev)
{
	// struct cvi_isp_buf *cvi_vb, *tmp;
	// struct vb2_buffer *vb2_buf;
	struct _isp_dqbuf_n *n = NULL;
	struct vi_event_k   *ev_k = NULL;
	unsigned long flags;
	struct isp_buffer *isp_b;
	struct _isp_snr_i2c_node *i2c_n;
	struct _isp_raw_num_n    *raw_n;
	enum cvi_isp_raw raw_num = ISP_PRERAW0;
	u8 i = 0, j = 0, count = 10;
	u8 rc = 0;

	vi_pr(VI_INFO, "+\n");

	atomic_set(&vdev->isp_streamoff, 1);

	// disable load-from-dram at streamoff
	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++)
		vdev->ctx.isp_pipe_cfg[raw_num].is_raw_replay_be = false;

	usr_pic_time_remove();

	// wait to make sure hw stopped.
	while (--count > 0) {
		if (atomic_read(&vdev->postraw_state) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW0][ISP_FE_CH0]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW0][ISP_FE_CH1]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW0][ISP_FE_CH2]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW0][ISP_FE_CH3]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW1][ISP_FE_CH0]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW1][ISP_FE_CH1]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW1][ISP_FE_CH2]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW1][ISP_FE_CH3]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW2][ISP_FE_CH0]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW2][ISP_FE_CH1]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW3][ISP_FE_CH0]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW3][ISP_FE_CH1]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW4][ISP_FE_CH0]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW4][ISP_FE_CH1]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW5][ISP_FE_CH0]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW5][ISP_FE_CH1]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE0][ISP_FE_CH0]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE0][ISP_FE_CH1]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE0][ISP_FE_CH2]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE0][ISP_FE_CH3]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE1][ISP_FE_CH0]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE1][ISP_FE_CH1]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE1][ISP_FE_CH2]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE1][ISP_FE_CH3]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_be_state[ISP_BE_CH0]) == ISP_STATE_IDLE &&
			atomic_read(&vdev->pre_be_state[ISP_BE_CH1]) == ISP_STATE_IDLE)
			break;
		vi_pr(VI_WARN, "wait count(%d)\n", count);
#ifdef FPGA_PORTING
		msleep(200);
#else
		msleep(20);
#endif
	}

	if (count == 0) {
		vi_pr(VI_ERR, "isp status fe_0(ch0:%d, ch1:%d, ch2:%d, ch3:%d)\n",
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW0][ISP_FE_CH0]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW0][ISP_FE_CH1]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW0][ISP_FE_CH2]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW0][ISP_FE_CH3]));
		vi_pr(VI_ERR, "isp status fe_1(ch0:%d, ch1:%d, ch2:%d, ch3:%d)\n",
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW1][ISP_FE_CH0]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW1][ISP_FE_CH1]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW1][ISP_FE_CH2]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW1][ISP_FE_CH3]));
		vi_pr(VI_ERR, "isp status fe_2(ch0:%d, ch1:%d) fe_3(ch0:%d, ch1:%d)\n",
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW2][ISP_FE_CH0]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW2][ISP_FE_CH1]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW3][ISP_FE_CH0]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW3][ISP_FE_CH1]));
		vi_pr(VI_ERR, "isp status fe_4(ch0:%d, ch1:%d) fe_5(ch0:%d, ch1:%d)\n",
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW4][ISP_FE_CH0]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW4][ISP_FE_CH1]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW5][ISP_FE_CH0]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW5][ISP_FE_CH1]));
		vi_pr(VI_ERR, "isp status fe_lite0(ch0:%d, ch1:%d, ch2:%d, ch3:%d)\n",
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE0][ISP_FE_CH0]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE0][ISP_FE_CH1]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE0][ISP_FE_CH2]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE0][ISP_FE_CH3]));
		vi_pr(VI_ERR, "isp status fe_lite1(ch0:%d, ch1:%d, ch2:%d, ch3:%d)\n",
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE1][ISP_FE_CH0]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE1][ISP_FE_CH1]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE1][ISP_FE_CH2]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE1][ISP_FE_CH3]));
		vi_pr(VI_ERR, "isp status be(ch0:%d, ch1:%d) postraw(%d)\n",
				atomic_read(&vdev->pre_be_state[ISP_BE_CH0]),
				atomic_read(&vdev->pre_be_state[ISP_BE_CH1]),
				atomic_read(&vdev->postraw_state));
	}

#if 0
	for (i = 0; i < 2; i++) {
		/*
		 * Release all the buffers enqueued to driver
		 * when streamoff is issued
		 */
		spin_lock_irqsave(&vdev->rdy_lock, flags);
		list_for_each_entry_safe(cvi_vb, tmp, &(vdev->rdy_queue[i]), list) {
			vfree(cvi_vb);
		}
		vdev->num_rdy[i] = 0;
		INIT_LIST_HEAD(&vdev->rdy_queue[i]);
		spin_unlock_irqrestore(&vdev->rdy_lock, flags);
	}
	for (i = 0; i < VI_MAX_CHN_NUM; i++) {
		/*
		 * Release all the buffers enqueued to driver
		 * when streamoff is issued
		 */
		spin_lock_irqsave(&vdev->qbuf_lock, flags);
		list_for_each_entry_safe(cvi_vb, tmp, &(vdev->qbuf_list[i]), list) {
			vb2_buf = &(cvi_vb->buf.vb2_buf);
			if (vb2_buf->state == VB2_BUF_STATE_ERROR)
				continue;
			vb2_buffer_done(vb2_buf, VB2_BUF_STATE_ERROR);
		}
		vdev->qbuf_num[i] = 0;
		INIT_LIST_HEAD(&vdev->qbuf_list[i]);
		spin_unlock_irqrestore(&vdev->qbuf_lock, flags);
	}
#endif

	spin_lock_irqsave(&event_lock, flags);
	while (!list_empty(&event_q.list)) {
		ev_k = list_first_entry(&event_q.list, struct vi_event_k, list);
		list_del_init(&ev_k->list);
		kfree(ev_k);
	}
	spin_unlock_irqrestore(&event_lock, flags);

	for (i = 0; i < ISP_SPLT_MAX; i++) {
		for (j = 0; j < ISP_SPLT_CHN_MAX; j++) {
			while ((isp_b = isp_buf_remove(&splt_out_q[i][j])) != NULL)
				vfree(isp_b);
			while ((isp_b = isp_buf_remove(&pre_fe_in_q[i][j])) != NULL)
				vfree(isp_b);
		}
	}

	for (i = 0; i < ISP_PRERAW_MAX; i++) {
		spin_lock_irqsave(&vdev->qbuf_lock, flags);
		vdev->qbuf_num[i] = 0;
		INIT_LIST_HEAD(&vdev->qbuf_list[i]);
		vdev->dqbuf_num[i] = 0;
		INIT_LIST_HEAD(&vdev->dqbuf_list[i]);
		spin_unlock_irqrestore(&vdev->qbuf_lock, flags);

		spin_lock_irqsave(&dq_lock, flags);
		while (!list_empty(&dqbuf_q[i].list)) {
			n = list_first_entry(&dqbuf_q[i].list, struct _isp_dqbuf_n, list);
			list_del_init(&n->list);
			kfree(n);
		}
		spin_unlock_irqrestore(&dq_lock, flags);

		for (j = 0; j < ISP_FE_CHN_MAX; j++) {
			while ((isp_b = isp_buf_remove(&pre_fe_out_q[i][j])) != NULL)
				vfree(isp_b);
		}

		while ((isp_b = isp_buf_remove(&raw_dump_b_dq[i])) != NULL)
			vfree(isp_b);
		while ((isp_b = isp_buf_remove(&raw_dump_b_se_dq[i])) != NULL)
			vfree(isp_b);
		while ((isp_b = isp_buf_remove(&raw_dump_b_q[i])) != NULL)
			vfree(isp_b);
		while ((isp_b = isp_buf_remove(&raw_dump_b_se_q[i])) != NULL)
			vfree(isp_b);

		spin_lock_irqsave(&snr_node_lock[i], flags);
		while (!list_empty(&isp_snr_i2c_queue[i].list)) {
			i2c_n = list_first_entry(&isp_snr_i2c_queue[i].list, struct _isp_snr_i2c_node, list);
			list_del_init(&i2c_n->list);
			kfree(i2c_n);
		}
		isp_snr_i2c_queue[i].num_rdy = 0;
		spin_unlock_irqrestore(&snr_node_lock[i], flags);

		while ((isp_b = isp_buf_remove(&pre_be_in_se_q[i])) != NULL)
			vfree(isp_b);
	}

	while ((isp_b = isp_buf_remove(&pre_be_in_q)) != NULL)
		vfree(isp_b);

	for (i = 0; i < ISP_RAW_PATH_MAX; i++) {
		while ((isp_b = isp_buf_remove(&pre_be_out_q[i])) != NULL)
			vfree(isp_b);
		while ((isp_b = isp_buf_remove(&postraw_in_q[i])) != NULL)
			vfree(isp_b);
	}

	spin_lock_irqsave(&raw_num_lock, flags);
	while (!list_empty(&pre_raw_num_q.list)) {
		raw_n = list_first_entry(&pre_raw_num_q.list, struct _isp_raw_num_n, list);
		list_del_init(&raw_n->list);
		kfree(raw_n);
	}
	spin_unlock_irqrestore(&raw_num_lock, flags);

	for (i = 0; i < ISP_PRERAW_MAX; i++) {
		kfree(isp_bufpool[i].fswdr_rpt);
		isp_bufpool[i].fswdr_rpt = 0;
	}

	// reset at stop for next run.
	isp_reset(&vdev->ctx);
	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++)
		isp_streaming(&vdev->ctx, false, raw_num);

#ifdef PORTING_TEST
	vi_ip_test_cases_uninit(&vdev->ctx);
#endif
	_vi_suspend(vdev);

	vi_tuning_buf_release();

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (vdev->tpu_thd_bind[raw_num]) {
			vi_destory_thread(vdev, vdev->tpu_thd_bind[raw_num]);
			vdev->tpu_thread_num--;
		}
	}

	vi_pr(VI_INFO, "-\n");

	return rc;
}

static uint64_t bnr_ai_isp_lauch_tpu(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	enum cvi_isp_fe_chn_num chn_num,
	uint64_t input_addr)
{
	struct isp_ctx *ctx = &vdev->ctx;
	struct isp_buffer *b = NULL;
	uint64_t output_addr = input_addr;
	uint8_t dev_num = vi_get_dev_num_by_raw(&vdev->ctx, raw_num);
	bool is_lauch_sucess = false;
	int timeout = 80;
	int ret = 0;

	b = isp_buf_last(&pre_fe_out_q[raw_num][chn_num]);
	if (!b) {
		vi_pr(VI_ERR, "pre_fe_out_%d_%d is NULL!\n", raw_num, chn_num);
		goto exit;
	}

	output_addr = b->addr;
	b->addr = input_addr;

	++ctx->isp_pipe_cfg[raw_num].bnr_ai_isp_frm_cnt;

	ai_isp_cfg_info[raw_num].ai_bnr_addr_pool[0] = input_addr;
	ai_isp_cfg_info[raw_num].ai_bnr_addr_pool[1] = output_addr;

	atomic_set(&vdev->ai_isp_int_flag[raw_num], 1);

	wake_up(&vdev->ai_isp_wait_q[raw_num]);

	ret = wait_event_timeout(
		vdev->ai_isp_wait_q[raw_num],
		atomic_read(&vdev->ai_isp_int_flag[raw_num]) == 2,
		msecs_to_jiffies(timeout));

	if (!ret) {
		vi_pr(VI_WARN, "dev_%d wait process timeout(%d ms) at frame(%d)\n",
			dev_num, timeout, ctx->isp_pipe_cfg[raw_num].bnr_ai_isp_frm_cnt);
		goto exit;
	}

	is_lauch_sucess = true;

exit:
	if (!is_lauch_sucess) {
		b->addr = output_addr;
		output_addr = input_addr;
	}

	return output_addr;
}

static void ai_isp_handle_process(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	struct isp_buffer *b = NULL;
	enum ai_isp_type ai_isp_type = AI_ISP_TYPE_BUTT;
	struct isp_queue *be_w_q = NULL;
	struct isp_queue *be_r_q = NULL;
	enum cvi_isp_fe_chn_num chn_num = ISP_FE_CH0;
	enum cvi_isp_fe_chn_num chn_max = chn_num;

	ai_isp_type = atomic_read(&vdev->ai_isp_type);
	switch (ai_isp_type) {
	case AI_ISP_TYPE_BNR:
		if (ctx->isp_pipe_cfg[raw_num].is_hdr_on)
			chn_max = ISP_FE_CH1;
		else
			chn_max = ISP_FE_CH0;

		reinit_completion(&vdev->tpu_done[raw_num]);
		while (chn_num <= chn_max) {
			be_w_q = &bnr_ai_isp_q[raw_num][chn_num];
			b = isp_buf_remove(be_w_q);
			if (!b) {
				vi_pr(VI_ERR, "tpu_in_%d_%d is NULL!\n", raw_num, chn_num);
				complete(&vdev->tpu_done[raw_num]);
				return;
			}

			// to avoid i2c_data send delay
			--vdev->pre_fe_frm_num[raw_num][chn_num];

			mutex_lock(&vdev->ai_isp_lock);
			_pre_hw_enque(vdev, raw_num, chn_num);
			mutex_unlock(&vdev->ai_isp_lock);

			if (atomic_read(&vdev->bnr_run_tpu[raw_num])) {
				b->addr = bnr_ai_isp_lauch_tpu(vdev, raw_num, chn_num, b->addr);
			}

			++vdev->pre_fe_frm_num[raw_num][chn_num];

			be_r_q = (chn_num == ISP_FE_CH0) ? &pre_be_in_q : &pre_be_in_se_q[raw_num];
			isp_buf_queue(be_r_q, b);
			chn_num++;
		}
		complete(&vdev->tpu_done[raw_num]);

		tasklet_hi_schedule(&vdev->job_work);

		break;
	default:
		vi_pr(VI_WARN, "unknown ai_isp_type (%d)!\n", ai_isp_type);
		break;
	}
}

static int ai_isp_resolve_cfg(struct cvi_vi_dev *vdev, ai_isp_cfg_t cfg)
{
	struct isp_ctx *ctx = &vdev->ctx;
	int timeout = 200;  //ms
	int ret;
	u8 raw_num = vi_get_raw_num_by_dev(ctx, cfg.ViPipe);

	switch (cfg.ai_isp_type) {
	case AI_ISP_TYPE_BNR:
	{
		switch (cfg.ai_isp_cfg_type) {
		case AI_ISP_PIPE_LOAD:
		{
			ctx->isp_pipe_cfg[raw_num].is_raw_ai_isp = true;
			break;
		}
		case AI_ISP_PIPE_UNLOAD:
		{
			//nothing to do
			break;
		}
		case AI_ISP_CFG_INIT:
		{
			if (!ctx->is_offline_be || ctx->is_offline_postraw) {
				vi_pr(VI_WARN, "bnr ai isp need (be offline)&(post online)\n");
				return -EPERM;
			}
			if (!ctx->isp_pipe_cfg[raw_num].is_raw_ai_isp) {
				vi_pr(VI_WARN, "need to enable raw_ai_isp before streaming!\n");
				return -EPERM;
			}

			break;
		}
		case AI_ISP_CFG_DEINIT:
		{
			ret = wait_for_completion_timeout(&vdev->tpu_done[raw_num],
					msecs_to_jiffies(timeout));
			if (ret == 0) {
				vi_pr(VI_WARN, "wait raw_%d tpu_done timeout(%d)!\n",
						raw_num, timeout);
			}

			break;
		}
		case AI_ISP_CFG_ENABLE:
		{
			ctx->isp_pipe_cfg[raw_num].bnr_ai_isp_frm_cnt = 0;
			atomic_set(&vdev->bnr_run_tpu[raw_num], 1);
			break;
		}
		case AI_ISP_CFG_DISABLE:
		{
			atomic_set(&vdev->bnr_run_tpu[raw_num], 0);

			atomic_set(&vdev->ai_isp_int_flag[raw_num], 2);
			wake_up(&vdev->ai_isp_wait_q[raw_num]);

			ret = wait_for_completion_timeout(&vdev->tpu_done[raw_num],
					msecs_to_jiffies(timeout));
			if (ret == 0) {
				vi_pr(VI_WARN, "wait raw_%d tpu_done timeout(%d)!\n",
						raw_num, timeout);
			}

			break;
		}
		default:
			break;
		}
		break;
	}

	default:
		break;
	}

	return 0;
}

static inline void _vi_wake_up_tpu_th(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	const enum ai_isp_type type)
{
	enum E_VI_TH tpu_th_id = vdev->tpu_thd_bind[raw_num];

	if (tpu_th_id < E_VI_TH_RUN_TPU1 || tpu_th_id > E_VI_TH_RUN_TPU2) {
		vi_pr(VI_ERR, "invalid thread_id(%d) for raw_%d\n",
			tpu_th_id, raw_num);
		return;
	}

	atomic_set(&vdev->ai_isp_type, type);
	vdev->vi_th[tpu_th_id].flag = raw_num + 1;
	wake_up(&vdev->vi_th[tpu_th_id].wq);
}

static int _vi_run_tpu_thread1(void *arg)
{
	struct cvi_vi_dev *vdev = (struct cvi_vi_dev *)arg;
	enum cvi_isp_raw raw_num = ISP_PRERAW0;
	enum E_VI_TH th_id = E_VI_TH_RUN_TPU1;

	while (1) {
		wait_event(vdev->vi_th[th_id].wq, vdev->vi_th[th_id].flag != 0
			|| kthread_should_stop());

		if (vdev->vi_th[th_id].flag != 0) {
			raw_num = vdev->vi_th[th_id].flag - 1;
			vdev->vi_th[th_id].flag = 0;
		}

		if (kthread_should_stop()) {
			pr_info("%s exit\n", vdev->vi_th[th_id].th_name);
			atomic_set(&vdev->vi_th[th_id].thread_exit, 1);
			do_exit(1);
		}

		ai_isp_handle_process(vdev, raw_num);
	}

	return 0;
}

static int _vi_run_tpu_thread2(void *arg)
{
	struct cvi_vi_dev *vdev = (struct cvi_vi_dev *)arg;
	enum cvi_isp_raw raw_num = ISP_PRERAW0;
	enum E_VI_TH th_id = E_VI_TH_RUN_TPU2;

	while (1) {
		wait_event(vdev->vi_th[th_id].wq, vdev->vi_th[th_id].flag != 0
			|| kthread_should_stop());

		if (vdev->vi_th[th_id].flag != 0) {
			raw_num = vdev->vi_th[th_id].flag - 1;
			vdev->vi_th[th_id].flag = 0;
		}

		if (kthread_should_stop()) {
			pr_info("%s exit\n", vdev->vi_th[th_id].th_name);
			atomic_set(&vdev->vi_th[th_id].thread_exit, 1);
			do_exit(1);
		}

		ai_isp_handle_process(vdev, raw_num);
	}

	return 0;
}

static void vi_dq_vb_buffer(struct cvi_vi_dev *videv, const u8 chn_id)
{
	struct cvi_isp_buf *cvi_vb, *tmp;
	struct vb2_buffer *vb2_buf;
	unsigned long flags;

	spin_lock_irqsave(&videv->qbuf_lock, flags);
	list_for_each_entry_safe(cvi_vb, tmp, &(videv->qbuf_list[chn_id]), list) {
		vb2_buf = &(cvi_vb->buf.vb2_buf);
		if (vb2_buf->state == VB2_BUF_STATE_ERROR)
			continue;
		vb2_buffer_done(vb2_buf, VB2_BUF_STATE_ERROR);
	}

	list_for_each_entry_safe(cvi_vb, tmp, &(videv->dqbuf_list[chn_id]), list) {
		vb2_buf = &(cvi_vb->buf.vb2_buf);
		if (vb2_buf->state == VB2_BUF_STATE_ERROR)
			continue;
		vb2_buffer_done(vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&videv->qbuf_lock, flags);
}

static int _pre_be_outbuf_enque(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	const u8 hw_chn_num)
{
	struct isp_ctx *ctx = &vdev->ctx;

	if (!ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) { //RGB sensor
		struct isp_queue *be_out_q = &pre_be_out_q[hw_chn_num];
		enum ISP_BLK_ID_T pre_be_dma = (hw_chn_num == ISP_BE_CH0) ?
						ISP_BLK_ID_DMA_CTL_PRE_RAW_BE_LE :
						ISP_BLK_ID_DMA_CTL_PRE_RAW_BE_SE;
		struct isp_buffer *b = NULL;

		b = isp_buf_next(be_out_q);
		if (!b) {
			vi_pr(VI_DBG, "pre_be chn_num_%d outbuf is empty\n", hw_chn_num);
			return 0;
		}

		ispblk_dma_setaddr(ctx, pre_be_dma, b->addr);

		if (ctx->isp_pipe_cfg[b->raw_num].is_rgbir_sensor) {
			isp_bufpool[b->raw_num].pre_be_ir_busy_idx = b->ir_idx;

			if (hw_chn_num == ISP_BE_CH0)
				ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_RGBIR_LE,
							isp_bufpool[raw_num].ir_le[b->ir_idx]);
			else
				ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_RGBIR_SE,
							isp_bufpool[raw_num].ir_se[b->ir_idx]);
		}
	} else if (ctx->is_multi_sensor &&
		   ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) { //RGB+YUV sensor
		enum ISP_BLK_ID_T pre_fe_dma;
		struct isp_queue *fe_out_q = &pre_fe_out_q[raw_num][hw_chn_num];
		struct isp_buffer *b = NULL;

		b = isp_buf_next(fe_out_q);
		if (!b) {
			vi_pr(VI_DBG, "pre_fe_%d buf_chn_num_%d outbuf is empty\n", raw_num, hw_chn_num);
			return 0;
		}

		if (ctx->isp_pipe_cfg[raw_num].is_bt_demux)
			pre_fe_dma = csibdg_lite_dma_find_hwid(raw_num, hw_chn_num);
		else
			pre_fe_dma = csibdg_dma_find_hwid(raw_num, hw_chn_num);

		ispblk_dma_setaddr(ctx, pre_fe_dma, b->addr);
	}

	return 1;
}

static int _pre_fe_outbuf_enque(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	const enum cvi_isp_fe_chn_num chn_num)
{
	struct isp_ctx *ctx = &vdev->ctx;

	if (!ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) { //RGB sensor
		struct isp_queue *fe_out_q = &pre_fe_out_q[raw_num][chn_num];
		enum ISP_BLK_ID_T pre_fe_dma;
		struct isp_buffer *b = NULL;
		bool trigger = false;
		uint32_t dmaid_le, dmaid_se;

		dmaid_le = csibdg_dma_find_hwid(raw_num, ISP_FE_CH0);
		dmaid_se = csibdg_dma_find_hwid(raw_num, ISP_FE_CH1);

		if (atomic_read(&vdev->isp_raw_dump_en[raw_num]) == 1) {//raw_dump flow
			if (ctx->isp_pipe_cfg[raw_num].is_hdr_on) {
				trigger = vdev->pre_fe_frm_num[raw_num][ISP_FE_CH0] ==
						vdev->pre_fe_frm_num[raw_num][ISP_FE_CH1];
			} else {
				trigger = true;
			}

			if (trigger) {
				struct isp_queue *fe_out_q = &raw_dump_b_q[raw_num];

				vi_pr(VI_DBG, "pre_fe raw_dump cfg start\n");

				b = isp_buf_next(fe_out_q);
				if (b == NULL) {
					vi_pr(VI_ERR, "Pre_fe_%d LE raw_dump outbuf is empty\n", raw_num);
					return 0;
				}

				ispblk_dma_setaddr(ctx, dmaid_le, b->addr);

				if (ctx->isp_pipe_cfg[raw_num].is_hdr_on) {
					struct isp_buffer *b_se = NULL;
					struct isp_queue *fe_out_q_se = &raw_dump_b_se_q[raw_num];

					b_se = isp_buf_next(fe_out_q_se);
					if (b_se == NULL) {
						vi_pr(VI_ERR, "Pre_fe_%d SE raw_dump outbuf is empty\n", raw_num);
						return 0;
					}

					ispblk_dma_setaddr(ctx, dmaid_se, b_se->addr);
				}

				atomic_set(&vdev->isp_raw_dump_en[raw_num], 2);
			}
		} else {
			//TODO maybe we can combine code, reduce code;
			if (_is_right_tile(ctx, raw_num)) { //raw_num = ISP_PRERAW1
				//TODO we suggest that fe0 done at first
				u64 buffaddr = ispblk_dma_getaddr(ctx, csibdg_dma_find_hwid(ISP_PRERAW0, chn_num));

				fe_out_q = &pre_fe_out_q[ISP_PRERAW0][chn_num];

				b = isp_buf_next(fe_out_q);
				if (!b) {
					vi_pr(VI_DBG, "pre_fe_%d chn_num_%d outbuf is empty\n", raw_num, chn_num);
					return 0;
				}

				if (buffaddr != b->addr) {
					vi_pr(VI_DBG, "pre_fe0/fe1 addr mismatch buffer=0x%llx, b->addr=0x%llx\n",
						buffaddr, b->addr);
					return 0;
				}

				pre_fe_dma = (chn_num == ISP_FE_CH0) ? dmaid_le : dmaid_se;

				ispblk_dma_setaddr(ctx, pre_fe_dma,
					buffaddr + 3 * UPPER(ctx->isp_pipe_cfg[ISP_PRERAW0].csibdg_width, 1));
			} else {
				b = isp_buf_next(fe_out_q);
				if (!b) {
					vi_pr(VI_INFO, "pre_fe_%d chn_num_%d outbuf is empty\n", raw_num, chn_num);
					return 0;
				}

				pre_fe_dma = (chn_num == ISP_FE_CH0) ? dmaid_le : dmaid_se;
				ispblk_dma_setaddr(ctx, pre_fe_dma, b->addr);
			}
		}
	} else if (ctx->is_multi_sensor &&
		   ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) { //RGB+YUV sensor
		enum ISP_BLK_ID_T dmaid;
		struct isp_queue *fe_out_q = &pre_fe_out_q[raw_num][chn_num];
		struct isp_buffer *b = NULL;

		b = isp_buf_next(fe_out_q);
		if (!b) {
			vi_pr(VI_DBG, "pre_fe_%d buf_chn_num_%d outbuf is empty\n", raw_num, chn_num);
			return 0;
		}

		if (ctx->isp_pipe_cfg[raw_num].is_bt_demux)
			dmaid = csibdg_lite_dma_find_hwid(raw_num, chn_num);
		else
			dmaid = csibdg_dma_find_hwid(raw_num, chn_num);

		ispblk_dma_setaddr(ctx, dmaid, b->addr);
	}

	return 1;
}

static int _postraw_inbuf_enq_check(
	struct cvi_vi_dev *vdev,
	enum cvi_isp_raw *raw_num,
	enum cvi_isp_fe_chn_num *chn_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	struct isp_queue *in_q = NULL, *in_se_q = NULL;
	struct isp_buffer *b = NULL, *b_se = NULL;
	int ret = 0;

	// LE
	if (_is_fe_be_online(ctx)) { //fe->be->dram->post
		in_q = &postraw_in_q[ISP_RAW_PATH_LE];
	} else if (_is_be_post_online(ctx)) { //fe->dram->be->post
		in_q = &pre_be_in_q;
	}

	b = isp_buf_next(in_q);
	if (b == NULL) {
		if (_is_fe_be_online(ctx)) //fe->be->dram->post
			vi_pr(VI_DBG, "Postraw input buf is empty\n");
		else if (_is_be_post_online(ctx)) //fe->dram->be->post
			vi_pr(VI_DBG, "Pre_be input buf is empty\n");
		ret = 1;
		return ret;
	}

	*raw_num = b->raw_num;
	*chn_num = b->chn_num;

	vdev->ctx.isp_pipe_cfg[b->raw_num].crop.x = b->crop_le.x;
	vdev->ctx.isp_pipe_cfg[b->raw_num].crop.y = b->crop_le.y;
	vdev->ctx.isp_pipe_cfg[b->raw_num].crop.w = vdev->ctx.img_width =
							ctx->isp_pipe_cfg[b->raw_num].post_img_w;
	vdev->ctx.isp_pipe_cfg[b->raw_num].crop.h = vdev->ctx.img_height =
							ctx->isp_pipe_cfg[b->raw_num].post_img_h;

	//YUV sensor, offline return error, online than config rawtop read dma.
	if (ctx->isp_pipe_cfg[b->raw_num].is_yuv_sensor) {
		if (ctx->isp_pipe_cfg[b->raw_num].is_offline_scaler) {
			ret = 1;
		} else {
			ispblk_dma_yuv_bypass_config(ctx, ISP_BLK_ID_DMA_CTL_RAW_RDMA0, b->addr, b->raw_num);
		}

		return ret;
	}

	// SE
	if (_is_fe_be_online(ctx)) { //fe->be->dram->post
		in_se_q = &postraw_in_q[ISP_RAW_PATH_SE];
	} else if (_is_be_post_online(ctx)) { //fe->dram->be->post
		in_se_q = &pre_be_in_se_q[b->raw_num];
	}

	if (ctx->isp_pipe_cfg[b->raw_num].is_hdr_on) {
		b_se = isp_buf_next(in_se_q);
		if (b_se == NULL) {
			if (_is_fe_be_online(ctx)) //fe->be->dram->post
				vi_pr(VI_DBG, "Postraw se input buf is empty\n");
			else if (_is_be_post_online(ctx)) //fe->dram->be->post
				vi_pr(VI_DBG, "Pre_be se input buf is empty\n");
			ret = 1;
			return ret;
		}
	}

	vdev->ctx.isp_pipe_cfg[b->raw_num].rgbmap_i.w_bit = b->rgbmap_i.w_bit;
	vdev->ctx.isp_pipe_cfg[b->raw_num].rgbmap_i.h_bit = b->rgbmap_i.h_bit;

	vdev->ctx.isp_pipe_cfg[b->raw_num].lmap_i.w_bit = b->lmap_i.w_bit;
	vdev->ctx.isp_pipe_cfg[b->raw_num].lmap_i.h_bit = b->lmap_i.h_bit;

	if (_is_fe_be_online(ctx)) { //fe->be->dram->post
		ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_RAW_RDMA0, b->addr);
		if (ctx->isp_pipe_cfg[b->raw_num].is_hdr_on) {
			vdev->ctx.isp_pipe_cfg[b->raw_num].crop_se.x = b_se->crop_se.x;
			vdev->ctx.isp_pipe_cfg[b->raw_num].crop_se.y = b_se->crop_se.y;
			vdev->ctx.isp_pipe_cfg[b->raw_num].crop_se.w = vdev->ctx.img_width;
			vdev->ctx.isp_pipe_cfg[b->raw_num].crop_se.h = vdev->ctx.img_height;

			ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_RAW_RDMA1, b_se->addr);
		}
	} else if (_is_be_post_online(ctx)) { //fe->dram->be->post
		u64 addr_le, addr_se;

		addr_le = b->addr;
		if (ctx->isp_pipe_cfg[b->raw_num].is_hdr_on)
			addr_se = b_se->addr;

		if (ctx->isp_pipe_cfg[b->raw_num].is_tile) {
			ctx->img_width = ctx->tile_cfg.l_in.end - ctx->isp_pipe_cfg[b->raw_num].crop.x + 1;
			if (++vdev->postraw_proc_num == 2) {
				ctx->is_work_on_r_tile = true;
				ctx->img_width = ctx->tile_cfg.r_out.end - ctx->tile_cfg.r_in.start + 1;
				addr_le += (ctx->tile_cfg.r_in.start - ctx->isp_pipe_cfg[b->raw_num].crop.x) * 3 / 2;
				if (ctx->isp_pipe_cfg[b->raw_num].is_hdr_on)
					addr_se += (ctx->tile_cfg.r_in.start
							- ctx->isp_pipe_cfg[b->raw_num].crop.x) * 3 / 2;
			}
		}

		vi_pr(VI_DBG, "is_right_tile %d, update addr_le 0x%llx\n", ctx->is_work_on_r_tile, addr_le);
		ispblk_dma_config(ctx, b->raw_num, ISP_BLK_ID_DMA_CTL_PRE_RAW_VI_SEL_LE, addr_le);
		if (ctx->isp_pipe_cfg[b->raw_num].is_hdr_on)
			ispblk_dma_config(ctx, b->raw_num, ISP_BLK_ID_DMA_CTL_PRE_RAW_VI_SEL_SE, addr_se);
	}

	if (ctx->isp_pipe_cfg[b->raw_num].is_tnr_ai_isp) {
		// TODO, This is for test.
		// Normally, it is up to the TPU playback.
		if (ctx->isp_pipe_cfg[b->raw_num].first_frm_cnt == 1)
			ctx->isp_pipe_cfg[b->raw_num].is_tnr_ai_isp_rdy = true;
	}

	return ret;
}

static void _postraw_outbuf_enque(struct cvi_vi_dev *vdev, const enum cvi_isp_raw raw_num)
{
	struct vb2_buffer *vb2_buf;
	struct cvi_isp_buf *b = NULL;
	struct isp_ctx *ctx = &vdev->ctx;
	uint64_t dma_offset = 0;
	uint64_t dma_addr[2];

	if (!atomic_read(&vdev->is_streaming[raw_num]))
		return;

	//Get the buffer for postraw output buffer
	b = cvi_isp_rdy_buf_next(vdev, raw_num);
	if (b == NULL)
		return;

	vb2_buf = &b->buf.vb2_buf;

	switch (vdev->fmt[raw_num]->buffers) {
	case 1:
		dma_addr[0] = vb2_dma_contig_plane_dma_addr(vb2_buf, 0);
		dma_addr[1] = dma_addr[0] + vdev->pipe[raw_num].sizeimage[0];
		break;
	case 2:
		dma_addr[0] = vb2_dma_contig_plane_dma_addr(vb2_buf, 0);
		dma_addr[1] = vb2_dma_contig_plane_dma_addr(vb2_buf, 1);
		break;
	default:
		vi_pr(VI_WARN, "unsupport planes:%d\n", vdev->fmt[raw_num]->buffers);
		break;
	}

	vi_pr(VI_DBG, "dma_addr:0x%llx-0x%llx\n", dma_addr[0], dma_addr[1]);

	if (ctx->isp_pipe_cfg[raw_num].is_tile) {
		if (ctx->is_work_on_r_tile) {
			dma_offset = ctx->tile_cfg.r_out.start;
			if (ctx->isp_pipe_cfg[raw_num].is_postout_crop) {
				//only start_x less than half of the image crop width, then update offset
				dma_offset = (ctx->isp_pipe_cfg[raw_num].postout_crop.x < dma_offset) ?
				dma_offset - ctx->isp_pipe_cfg[raw_num].postout_crop.x : 0;
			}

			dma_addr[0] += dma_offset;
			dma_addr[1] += dma_offset;
		}
		ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_YUV_CROP_Y, dma_addr[0]);
		ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_YUV_CROP_C, dma_addr[1]);
	} else {
		ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_YUV_CROP_Y, dma_addr[0]);
		ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_YUV_CROP_C, dma_addr[1]);
	}
}

static u8 _postraw_outbuf_empty(struct cvi_vi_dev *vdev, const enum cvi_isp_raw raw_num)
{
	u8 ret = 0;

	if (cvi_isp_rdy_buf_empty(vdev, raw_num)) {
		vi_pr(VI_DBG, "postraw chn_%d output buffer is empty\n", raw_num);
		ret = 1;
	}

	return ret;
}

void _postraw_outbuf_enq(struct cvi_vi_dev *vdev, const enum cvi_isp_raw raw_num)
{
	if (!(vdev->ctx.isp_pipe_cfg[raw_num].is_tile && vdev->ctx.is_work_on_r_tile))
		cvi_isp_rdy_buf_pop(vdev, raw_num);
	_postraw_outbuf_enque(vdev, raw_num);
}

/*
 * for postraw offline only.
 *  trig preraw if there is output buffer in preraw output.
 */
s8 _pre_hw_enque(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	const enum cvi_isp_fe_chn_num chn_num)
{
	struct isp_ctx *ctx = &vdev->ctx;

	//ISP frame error handling
	if (atomic_read(&vdev->isp_err_handle_flag) == 1) {
		vi_pr(VI_DBG, "wait err_handling done\n");
		return -ISP_ERROR;
	}

	if (atomic_read(&vdev->isp_streamoff) == 0) {
		if (_is_drop_next_frame(vdev, raw_num, chn_num)) {
			vi_pr(VI_DBG, "Pre_fe_%d chn_num_%d drop_frame_num %d\n",
					raw_num, chn_num, vdev->drop_frame_number[raw_num]);
			return -ISP_DROP_FRM;
		}

		if (_is_fe_be_online(ctx) && !ctx->is_slice_buf_on) { //fe->be->dram->post
			if (!ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) { //RGB sensor
				if (atomic_cmpxchg(&vdev->pre_be_state[chn_num],
							ISP_STATE_IDLE, ISP_STATE_RUNNING) ==
							ISP_STATE_RUNNING) {
					vi_pr(VI_DBG, "Pre_be chn_num_%d is running\n", chn_num);
					return -ISP_RUNNING;
				}
			} else { //YUV sensor
				if (atomic_cmpxchg(&vdev->pre_fe_state[raw_num][chn_num],
							ISP_STATE_IDLE, ISP_STATE_RUNNING) ==
							ISP_STATE_RUNNING) {
					vi_pr(VI_DBG, "Pre_fe_%d chn_num_%d is running\n", raw_num, chn_num);
					return -ISP_RUNNING;
				}
			}

			// only if fe->be->dram
			if (_pre_be_outbuf_enque(vdev, raw_num, chn_num)) {
				if (atomic_read(&vdev->isp_raw_dump_en[raw_num]) == 1) //raw_dump flow
					_isp_fe_be_raw_dump_cfg(vdev, raw_num, chn_num);
				isp_pre_trig(ctx, raw_num, chn_num);
			} else {
				if (!ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) //RGB sensor
					atomic_set(&vdev->pre_be_state[chn_num], ISP_STATE_IDLE);
				else  //YUV sensor
					atomic_set(&vdev->pre_fe_state[raw_num][chn_num], ISP_STATE_IDLE);
				return -ISP_NO_BUFFER;
			}
		} else if (_is_be_post_online(ctx)) { //fe->dram->be->post
			if (ctx->isp_pipe_cfg[raw_num].is_raw_replay_be) {
				if (atomic_read(&vdev->isp_streamon) == 0) {
					vi_pr(VI_DBG, "VI not ready\n");
					return -ISP_STOP;
				}

				if (atomic_cmpxchg(&vdev->postraw_state,
							ISP_STATE_IDLE, ISP_STATE_RUNNING) ==
							ISP_STATE_RUNNING) {
					vi_pr(VI_DBG, "Postraw is running\n");
					return -ISP_RUNNING;
				}

				if (ctx->isp_pipe_cfg[raw_num].is_offline_scaler) { //Scaler onffline mode
					if (_postraw_outbuf_empty(vdev, raw_num)) {
						atomic_set(&vdev->postraw_state, ISP_STATE_IDLE);
						return -ISP_NO_BUFFER;
					}

					_postraw_outbuf_enq(vdev, raw_num);
				}

				if (atomic_read(&vdev->isp_raw_dump_en[raw_num]) == 1) //raw_dump flow
					_isp_fe_be_raw_dump_cfg(vdev, raw_num, chn_num);

				isp_pre_trig(ctx, raw_num, chn_num);
			} else {
				if (atomic_cmpxchg(&vdev->pre_fe_state[raw_num][chn_num],
							ISP_STATE_IDLE, ISP_STATE_RUNNING) ==
							ISP_STATE_RUNNING) {
					vi_pr(VI_DBG, "Pre_fe_%d chn_num_%d is running\n", raw_num, chn_num);
					return -ISP_RUNNING;
				}

				// only if fe->dram
				if (_pre_fe_outbuf_enque(vdev, raw_num, chn_num)) {
					isp_pre_trig(ctx, raw_num, chn_num);
				} else {
					atomic_set(&vdev->pre_fe_state[raw_num][chn_num], ISP_STATE_IDLE);
					return -ISP_NO_BUFFER;
				}
			}
		}
	}
	return ISP_SUCCESS;
}

static inline void _swap_post_sts_buf(struct isp_ctx *ctx, const enum cvi_isp_raw raw_num)
{
	struct _membuf *pool;
	unsigned long flags;
	uint8_t idx;

	pool = &isp_bufpool[raw_num];

	spin_lock_irqsave(&pool->post_sts_lock, flags);
	if (pool->post_sts_in_use == 1) {
		spin_unlock_irqrestore(&pool->post_sts_lock, flags);
		return;
	}
	pool->post_sts_busy_idx ^= 1;
	spin_unlock_irqrestore(&pool->post_sts_lock, flags);

	if (_is_be_post_online(ctx))
		idx = pool->post_sts_busy_idx ^ 1;
	else
		idx = pool->post_sts_busy_idx;

	//gms dma
	ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_GMS, pool->sts_mem[idx].gms.phy_addr);

	//ae le dma
	ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_AE_HIST_LE, pool->sts_mem[idx].ae_le.phy_addr);
	if (ctx->isp_pipe_cfg[raw_num].is_hdr_on) {
		//ae se dma
		ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_AE_HIST_SE, pool->sts_mem[idx].ae_se.phy_addr);
	}

	//dci dma is fixed size
	ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_DCI, pool->sts_mem[idx].dci.phy_addr);
	//hist edge v dma is fixed size
	ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_HIST_EDGE_V, pool->sts_mem[idx].hist_edge_v.phy_addr);
}

static inline void _post_rgbmap_update(struct isp_ctx *ctx, const enum cvi_isp_raw raw_num, const u32 frm_num)
{
	uint64_t cur_le_r, cur_se_r, pre_le_r, pre_se_r;
	uint16_t w_bit = ctx->isp_pipe_cfg[raw_num].rgbmap_i.w_bit;
	uint8_t cur_idx = (frm_num - ctx->rgbmap_prebuf_idx) % RGBMAP_BUF_IDX;
	uint8_t pre_idx = (frm_num - 1 + RGBMAP_BUF_IDX - ctx->rgbmap_prebuf_idx) % RGBMAP_BUF_IDX;

	cur_le_r = isp_bufpool[raw_num].rgbmap_le[cur_idx];
	if (frm_num <= ctx->rgbmap_prebuf_idx)
		pre_le_r = isp_bufpool[raw_num].rgbmap_le[0];
	else
		pre_le_r = isp_bufpool[raw_num].rgbmap_le[pre_idx];

	if (ctx->isp_pipe_cfg[raw_num].is_tile && ctx->is_work_on_r_tile) {
		cur_le_r += UPPER(ctx->tile_cfg.r_in.start - ctx->isp_pipe_cfg[raw_num].crop.x, w_bit) * 6;
		pre_le_r += UPPER(ctx->tile_cfg.r_in.start - ctx->isp_pipe_cfg[raw_num].crop.x, w_bit) * 6;
		ispblk_dma_config(ctx, raw_num + 1, ISP_BLK_ID_DMA_CTL_MMAP_CUR_LE_R, cur_le_r);
		ispblk_dma_config(ctx, raw_num + 1, ISP_BLK_ID_DMA_CTL_MMAP_PRE_LE_R, pre_le_r);
	} else {
		ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_MMAP_CUR_LE_R, cur_le_r);
		ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_MMAP_PRE_LE_R, pre_le_r);
	}

	vi_pr(VI_DBG, "is_tile(%d), is_right(%d), cur_le_r(0x%llx), pre_le_r(0x%llx)\n",
		ctx->isp_pipe_cfg[raw_num].is_tile, ctx->is_work_on_r_tile, cur_le_r, pre_le_r);

	if (ctx->isp_pipe_cfg[raw_num].is_hdr_on) {
		cur_se_r = isp_bufpool[raw_num].rgbmap_se[cur_idx];
		if (frm_num <= ctx->rgbmap_prebuf_idx)
			pre_se_r = isp_bufpool[raw_num].rgbmap_se[0];
		else
			pre_se_r = isp_bufpool[raw_num].rgbmap_se[pre_idx];

		if (ctx->isp_pipe_cfg[raw_num].is_tile && ctx->is_work_on_r_tile) {
			cur_se_r += UPPER(ctx->tile_cfg.r_in.start - ctx->isp_pipe_cfg[raw_num].crop.x, w_bit) * 6;
			pre_se_r += UPPER(ctx->tile_cfg.r_in.start - ctx->isp_pipe_cfg[raw_num].crop.x, w_bit) * 6;
			ispblk_dma_config(ctx, raw_num + 1, ISP_BLK_ID_DMA_CTL_MMAP_CUR_SE_R, cur_se_r);
			ispblk_dma_config(ctx, raw_num + 1, ISP_BLK_ID_DMA_CTL_MMAP_PRE_SE_R, pre_se_r);
		} else {
			ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_MMAP_CUR_SE_R, cur_se_r);
			ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_MMAP_PRE_SE_R, pre_se_r);
		}
	}
}

static inline void _post_lmap_update(struct isp_ctx *ctx, const enum cvi_isp_raw raw_num)
{
	uint64_t lmap_le = isp_bufpool[raw_num].lmap_le;
	uint64_t lmap_se = isp_bufpool[raw_num].lmap_se;

	ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_LMAP_LE, lmap_le);
	ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_LTM_LE, lmap_le);

	if (ctx->isp_pipe_cfg[raw_num].is_hdr_on) {
		ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_LTM_SE, lmap_se);
		ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_LMAP_SE, lmap_se);
	} else {
		ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_LTM_SE, lmap_le);
		ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_LMAP_SE, lmap_le);
	}
}

static inline void _post_mlsc_update(struct isp_ctx *ctx, const enum cvi_isp_raw raw_num)
{
	uint64_t lsc = isp_bufpool[raw_num].lsc;

	ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_LSC_LE, lsc);

	if (ctx->isp_pipe_cfg[raw_num].is_hdr_on)
		ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_LSC_SE, lsc);
}

static inline void _post_ldci_update(struct isp_ctx *ctx, const enum cvi_isp_raw raw_num)
{
	uint64_t ldci_wdma = isp_bufpool[raw_num].ldci;
	uint64_t ldci_rdma = isp_bufpool[raw_num].ldci;

	if (ctx->isp_pipe_cfg[raw_num].is_tile && ctx->is_work_on_r_tile) {
		ldci_wdma = isp_bufpool[raw_num + 1].ldci;
		ldci_rdma = isp_bufpool[raw_num + 1].ldci;
	}

	ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_LDCI_W, ldci_wdma);
	ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_LDCI_R, ldci_rdma);
}

static inline void _post_3dnr_update(struct isp_ctx *ctx, const enum cvi_isp_raw raw_num)
{
	uint64_t manr_addr = isp_bufpool[raw_num].manr[0];
	uint64_t ai_isp_y_addr, ai_isp_u_addr, ai_isp_v_addr, ai_isp_manr_addr;
	uint64_t r_mo_addr, r_y_addr, r_uv_addr;
	uint64_t w_mo_addr, w_y_addr, w_uv_addr;
	uint32_t size;
	void *pSrc = NULL, *pDst = NULL;

	r_mo_addr = w_mo_addr = isp_bufpool[raw_num].tdnr[0];
	r_y_addr  = w_y_addr  = isp_bufpool[raw_num].tdnr[1];
	r_uv_addr = w_uv_addr = isp_bufpool[raw_num].tdnr[2];

	if (ctx->is_3dnr_on) {
		if (ctx->is_fbc_on) {
			//3dnr y
			ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_ST_Y, r_y_addr);
			ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_LD_Y, w_y_addr);

			//3dnr uv
			ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_ST_C, r_uv_addr);
			ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_LD_C, w_uv_addr);
		} else {
			if (ctx->isp_pipe_cfg[raw_num].is_tile) {
				if (ctx->is_work_on_r_tile) { //Right tile
					w_y_addr  = r_y_addr  = isp_bufpool[raw_num].tdnr_rtile[1];
					w_uv_addr = r_uv_addr = isp_bufpool[raw_num].tdnr_rtile[2];
				}
			}

			//3dnr y
			ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_ST_Y, r_y_addr);
			ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_LD_Y, w_y_addr);

			//3dnr uv
			ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_ST_C, r_uv_addr);
			ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_LD_C, w_uv_addr);
		}

		if (ctx->isp_pipe_cfg[raw_num].is_tile) {
			if (ctx->is_work_on_r_tile) {
				r_mo_addr = w_mo_addr =  isp_bufpool[raw_num].tdnr_rtile[0];
				manr_addr = isp_bufpool[raw_num].manr_rtile[0];
			}
		}
		//3dnr mo
		ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_ST_MO, r_mo_addr);
		ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_TNR_LD_MO, w_mo_addr);

		//manr
		ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_MMAP_IIR_R, manr_addr);
		ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_MMAP_IIR_W, manr_addr);

		//ai_isp
		if (ctx->isp_pipe_cfg[raw_num].is_tnr_ai_isp &&
		    ctx->isp_pipe_cfg[raw_num].is_tnr_ai_isp_rdy) {
			// TODO, This is for test.
			// Normally, should use the dma address returned by TPU.
			ai_isp_y_addr    = isp_bufpool[raw_num].tnr_ai_isp[0];
			ai_isp_u_addr    = isp_bufpool[raw_num].tnr_ai_isp[1];
			ai_isp_v_addr    = isp_bufpool[raw_num].tnr_ai_isp[2];
			ai_isp_manr_addr = isp_bufpool[raw_num].manr[1];

			pSrc = phys_to_virt(r_y_addr);
			pDst = phys_to_virt(ai_isp_y_addr);
			size = ispblk_dma_buf_get_size(ctx, raw_num, ISP_BLK_ID_DMA_CTL_AI_ISP_RDMA_Y);
			memcpy(pDst, pSrc, size);

			pSrc = phys_to_virt(r_uv_addr);
			pDst = phys_to_virt(ai_isp_u_addr);
			size = ispblk_dma_buf_get_size(ctx, raw_num, ISP_BLK_ID_DMA_CTL_AI_ISP_RDMA_U);
			memcpy(pDst, pSrc, size);

			pSrc += size;
			pDst = phys_to_virt(ai_isp_v_addr);
			size = ispblk_dma_buf_get_size(ctx, raw_num, ISP_BLK_ID_DMA_CTL_AI_ISP_RDMA_V);
			memcpy(pDst, pSrc, size);

			pSrc = phys_to_virt(manr_addr);
			pDst = phys_to_virt(ai_isp_manr_addr);
			size = ispblk_dma_buf_get_size(ctx, raw_num, ISP_BLK_ID_DMA_CTL_MMAP_AI_ISP);
			memcpy(pDst, pSrc, size);

			ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_AI_ISP_RDMA_Y, ai_isp_y_addr);
			ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_AI_ISP_RDMA_U, ai_isp_u_addr);
			ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_AI_ISP_RDMA_V, ai_isp_v_addr);
			ispblk_dma_setaddr(ctx, ISP_BLK_ID_DMA_CTL_MMAP_AI_ISP, ai_isp_manr_addr);
		}
	}
}

static inline void _post_yuv_crop_update(struct isp_ctx *ctx, const enum cvi_isp_raw raw_num)
{
	if (ctx->isp_pipe_cfg[raw_num].is_offline_scaler) {
		if (ctx->isp_pipe_cfg[raw_num].is_tnr_ai_isp &&
		    !ctx->isp_pipe_cfg[raw_num].is_tnr_ai_isp_rdy) {
			ispblk_dma_enable(ctx, ISP_BLK_ID_DMA_CTL_YUV_CROP_Y, true, true);
			ispblk_dma_enable(ctx, ISP_BLK_ID_DMA_CTL_YUV_CROP_C, true, true);
		} else {
			ispblk_dma_enable(ctx, ISP_BLK_ID_DMA_CTL_YUV_CROP_Y, true, false);
			ispblk_dma_enable(ctx, ISP_BLK_ID_DMA_CTL_YUV_CROP_C, true, false);
		}
		if (ctx->isp_pipe_cfg[raw_num].is_postout_crop) {
			struct vi_rect crop = ctx->isp_pipe_cfg[raw_num].postout_crop;

			ispblk_crop_config(ctx, ISP_BLK_ID_YUV_CROP_Y, ctx->isp_pipe_cfg[raw_num].crop);
			crop.x >>= 1;
			crop.y >>= 1;
			crop.w >>= 1;
			crop.h >>= 1;
			ispblk_crop_config(ctx, ISP_BLK_ID_YUV_CROP_C, crop);
		} else {
			ispblk_crop_enable(ctx, ISP_BLK_ID_DMA_CTL_YUV_CROP_Y, false);
			ispblk_crop_enable(ctx, ISP_BLK_ID_DMA_CTL_YUV_CROP_C, false);
		}
	} else {
		ispblk_dma_enable(ctx, ISP_BLK_ID_DMA_CTL_YUV_CROP_Y, false, false);
		ispblk_dma_enable(ctx, ISP_BLK_ID_DMA_CTL_YUV_CROP_C, false, false);
	}

	if (ctx->isp_pipe_cfg[raw_num].is_tile) {
		struct vi_rect crop = {0};

		if (!ctx->is_work_on_r_tile) { //left tile
			crop.w = ctx->tile_cfg.l_out.end - ctx->tile_cfg.l_out.start + 1;
			crop.h = ctx->img_height;

			if (ctx->isp_pipe_cfg[raw_num].is_postout_crop) {
				if (ctx->isp_pipe_cfg[raw_num].postout_crop.x >= ctx->tile_cfg.r_out.start) {
					//it's means left tile dma disable
					ispblk_dma_enable(ctx, ISP_BLK_ID_DMA_CTL_YUV_CROP_Y, true, true);
					ispblk_dma_enable(ctx, ISP_BLK_ID_DMA_CTL_YUV_CROP_C, true, true);
				} else {
					crop.x = ctx->isp_pipe_cfg[raw_num].postout_crop.x;
					crop.y = ctx->isp_pipe_cfg[raw_num].postout_crop.y;
					crop.w = (crop.x + ctx->isp_pipe_cfg[raw_num].postout_crop.w
							> ctx->tile_cfg.r_out.start)
							? ctx->tile_cfg.r_out.start - crop.x
							: ctx->isp_pipe_cfg[raw_num].postout_crop.w;
					crop.h = ctx->isp_pipe_cfg[raw_num].postout_crop.h;
				}
			}

			vi_pr(VI_DBG, "left tile y crop x=%d y=%d w=%d h=%d",
				crop.x, crop.y, crop.w, crop.h);
			ispblk_crop_config(ctx, ISP_BLK_ID_YUV_CROP_Y, crop);
			crop.x >>= 1;
			crop.w >>= 1;
			if (ctx->isp_pipe_cfg[raw_num].is_offline_scaler) {//offline2sc
				crop.y >>= 1;
				crop.h >>= 1;
			}
			ispblk_crop_config(ctx, ISP_BLK_ID_YUV_CROP_C, crop);

			vi_pr(VI_DBG, "left tile uv crop x=%d y=%d w=%d h=%d",
				crop.x, crop.y, crop.w, crop.h);
		} else { //right tile
			crop.h = ctx->img_height;
			if (!ctx->isp_pipe_cfg[raw_num].is_offline_scaler) {
				crop.x = 0;
				crop.w = ctx->img_width;
			} else { //offline2sc
				crop.x = ctx->tile_cfg.r_out.start - ctx->tile_cfg.r_in.start;
				crop.w = ctx->tile_cfg.r_out.end - ctx->tile_cfg.r_out.start + 1;
			}

			if (ctx->isp_pipe_cfg[raw_num].is_postout_crop) {
				if (ctx->isp_pipe_cfg[raw_num].postout_crop.x +
					ctx->isp_pipe_cfg[raw_num].postout_crop.w <= ctx->tile_cfg.r_out.start) {
					//it's means right tile dma disable
					ispblk_dma_enable(ctx, ISP_BLK_ID_DMA_CTL_YUV_CROP_Y, true, true);
					ispblk_dma_enable(ctx, ISP_BLK_ID_DMA_CTL_YUV_CROP_C, true, true);
				} else {
					crop.x = (ctx->isp_pipe_cfg[raw_num].postout_crop.x < ctx->tile_cfg.r_out.start)
						? crop.x
						: (crop.x + ctx->isp_pipe_cfg[raw_num].postout_crop.x
						   - ctx->tile_cfg.r_out.start);
					crop.y = ctx->isp_pipe_cfg[raw_num].postout_crop.y;
					crop.w = (ctx->isp_pipe_cfg[raw_num].postout_crop.x < ctx->tile_cfg.r_out.start)
						? (ctx->isp_pipe_cfg[raw_num].postout_crop.x
						  + ctx->isp_pipe_cfg[raw_num].postout_crop.w
						  - ctx->tile_cfg.r_out.start)
						: ctx->isp_pipe_cfg[raw_num].postout_crop.w;
					crop.h = ctx->isp_pipe_cfg[raw_num].postout_crop.h;
				}
			}

			vi_pr(VI_DBG, "right tile y crop x=%d y=%d w=%d h=%d",
				crop.x, crop.y, crop.w, crop.h);
			ispblk_crop_config(ctx, ISP_BLK_ID_YUV_CROP_Y, crop);
			crop.x >>= 1;
			crop.w >>= 1;
			if (ctx->isp_pipe_cfg[raw_num].is_offline_scaler) {//offline2sc
				crop.y >>= 1;
				crop.h >>= 1;
			}
			ispblk_crop_config(ctx, ISP_BLK_ID_YUV_CROP_C, crop);

			vi_pr(VI_DBG, "right tile uv crop x=%d y=%d w=%d h=%d",
				crop.x, crop.y, crop.w, crop.h);
		}
	}
}

static inline void _post_dma_update(struct cvi_vi_dev *vdev, const enum cvi_isp_raw raw_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	if (!ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) { //RGB Sensor
		//Update rgbmap dma addr
		_post_rgbmap_update(ctx, raw_num, vdev->pre_fe_frm_num[raw_num][ISP_FE_CH0]);

		//update lmap dma
		_post_lmap_update(ctx, raw_num);

		//update mlsc dma
		_post_mlsc_update(ctx, raw_num);

		//update ldci dma
		_post_ldci_update(ctx, raw_num);

		//update 3dnr dma
		_post_3dnr_update(ctx, raw_num);

		//update yuv_crop dma
		_post_yuv_crop_update(ctx, raw_num);
	} else {
		//YUV Sensor
		if (ctx->isp_pipe_cfg[raw_num].yuv_scene_mode == ISP_YUV_SCENE_ISP && ctx->is_3dnr_on) {
			//Update rgbmap dma addr
			_post_rgbmap_update(ctx, raw_num, vdev->pre_fe_frm_num[raw_num][ISP_FE_CH0]);
			//update 3dnr dma
			_post_3dnr_update(ctx, raw_num);
		}
	}
}

static u32 _is_fisrt_frm_after_drop(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	uint32_t first_frm_num_after_drop = ctx->isp_pipe_cfg[raw_num].isp_reset_frm;
	u32 frm_num = 0;

	frm_num = vdev->pre_fe_frm_num[raw_num][ISP_FE_CH0];

	if ((first_frm_num_after_drop != 0) && (frm_num == first_frm_num_after_drop)) {
		vi_pr(VI_DBG, "reset isp frm_num[%d]\n", frm_num);
		ctx->isp_pipe_cfg[raw_num].isp_reset_frm = 0;
		return 1;
	} else
		return 0;
}

static inline void _post_ctrl_update(struct cvi_vi_dev *vdev, const enum cvi_isp_raw raw_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	enum cvi_isp_raw dst_raw = (ctx->isp_pipe_cfg[raw_num].is_tile && ctx->is_work_on_r_tile)
					? raw_num + 1 : raw_num;

	ispblk_post_cfg_update(ctx, raw_num);

	// ispblk_fusion_hdr_cfg(ctx, raw_num);

	_vi_clear_mmap_fbc_ring_base(vdev, raw_num);

	if (ctx->is_3dnr_on) {
		ispblk_tnr_post_chg(ctx, raw_num);

		//To set apply the prev frm or not for manr/3dnr
		if (vdev->preraw_first_frm[dst_raw] ||
		    _is_fisrt_frm_after_drop(vdev, dst_raw)) {
			vdev->preraw_first_frm[dst_raw] = false;
			isp_first_frm_reset(ctx, 1);
		} else {
			isp_first_frm_reset(ctx, 0);
		}
	}
}

static uint8_t _pre_be_sts_in_use_chk(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	const enum cvi_isp_be_chn_num chn_num)
{
	unsigned long flags;
	static uint8_t be_in_use;

	if (chn_num == ISP_BE_CH0) {
		spin_lock_irqsave(&isp_bufpool[raw_num].pre_be_sts_lock, flags);
		if (isp_bufpool[raw_num].pre_be_sts_in_use == 1) {
			be_in_use = 1;
		} else {
			be_in_use = 0;
			isp_bufpool[raw_num].pre_be_sts_busy_idx ^= 1;
		}
		spin_unlock_irqrestore(&isp_bufpool[raw_num].pre_be_sts_lock, flags);
	}

	return be_in_use;
}

static inline void _swap_pre_be_sts_buf(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	const enum cvi_isp_be_chn_num chn_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	struct _membuf *pool;
	uint8_t idx;

	if (_pre_be_sts_in_use_chk(vdev, raw_num, chn_num) == 0) {
		pool = &isp_bufpool[raw_num];
		if (_is_be_post_online(ctx))
			idx = isp_bufpool[raw_num].pre_be_sts_busy_idx ^ 1;
		else
			idx = isp_bufpool[raw_num].pre_be_sts_busy_idx;

		if (chn_num == ISP_BE_CH0) {
			//af dma
			ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_AF_W, pool->sts_mem[idx].af.phy_addr);
		}
	}
}

static inline void _pre_be_dma_update(struct cvi_vi_dev *vdev, const enum cvi_isp_raw raw_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	uint64_t ir_le, ir_se;
	uint8_t idx = 0;

	if (ctx->isp_pipe_cfg[raw_num].is_rgbir_sensor && _is_be_post_online(ctx)) {
		idx = isp_bufpool[raw_num].pre_be_ir_busy_idx;
		ir_le = isp_bufpool[raw_num].ir_le[idx];
		ir_se = isp_bufpool[raw_num].ir_se[idx];

		if (ctx->isp_pipe_cfg[raw_num].is_tile && ctx->is_work_on_r_tile) {
			ir_le += 3 * UPPER((ctx->tile_cfg.r_in.start
						- ctx->isp_pipe_cfg[raw_num].crop.x + 1) >> 1, 1);
			ispblk_dma_config(ctx, raw_num + 1, ISP_BLK_ID_DMA_CTL_RGBIR_LE, ir_le);
		} else {
			ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_RGBIR_LE, ir_le);
		}

		if (ctx->isp_pipe_cfg[raw_num].is_hdr_on) {
			if (ctx->isp_pipe_cfg[raw_num].is_tile && ctx->is_work_on_r_tile) {
				ir_se += 3 * UPPER((ctx->tile_cfg.r_in.start
							- ctx->isp_pipe_cfg[raw_num].crop.x + 1) >> 1, 1);
				ispblk_dma_config(ctx, raw_num + 1, ISP_BLK_ID_DMA_CTL_RGBIR_SE, ir_se);
			} else {
				ispblk_dma_config(ctx, raw_num, ISP_BLK_ID_DMA_CTL_RGBIR_SE, ir_se);
			}
		}
	}
}

static inline void _pre_be_ctrl_update(struct cvi_vi_dev *vdev, const enum cvi_isp_raw raw_num)
{
	struct isp_ctx *ctx = &vdev->ctx;

	ispblk_pre_be_cfg_update(ctx, raw_num);
}

/*
 * update cfg by raw_num when post_num==0, maybe will update by raw_num and chn_num.
 */
static void _postraw_update_cfg_from_user(struct cvi_vi_dev *vdev, enum cvi_isp_raw raw_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	enum cvi_isp_raw raw = ISP_PRERAW0;
	int chn_num = 0;

	//only update [raw_num][chn0]
	for (; raw < raw_num; raw++) {
		if (!vdev->ctx.isp_pipe_enable[raw])
			continue;
		if (!ctx->isp_pipe_cfg[raw].is_yuv_sensor) {
			chn_num++;
		} else {
			chn_num += ctx->isp_pipe_cfg[raw].muxMode + 1;
		}
	}

	//can't update crop when postraw work on r_tile
	if (ctx->isp_pipe_cfg[raw_num].is_tile && vdev->postraw_proc_num == 2)
		return;

	if (!gViCtx->chnCrop[chn_num].bEnable) {
		ctx->isp_pipe_cfg[raw_num].is_postout_crop = false;
		return;
	}

	ctx->isp_pipe_cfg[raw_num].postout_crop.x = gViCtx->chnCrop[chn_num].stCropRect.s32X;
	ctx->isp_pipe_cfg[raw_num].postout_crop.y = gViCtx->chnCrop[chn_num].stCropRect.s32Y;
	ctx->isp_pipe_cfg[raw_num].postout_crop.w = gViCtx->chnCrop[chn_num].stCropRect.u32Width;
	ctx->isp_pipe_cfg[raw_num].postout_crop.h = gViCtx->chnCrop[chn_num].stCropRect.u32Height;
	ctx->isp_pipe_cfg[raw_num].is_postout_crop = true;
}

/*
 * - postraw offline -
 *  trig postraw if there is in/out buffer for postraw
 * - postraw online -
 *  trig preraw if there is output buffer for postraw
 */
static void _post_hw_enque(
	struct cvi_vi_dev *vdev)
{
	struct isp_ctx *ctx = &vdev->ctx;
	enum cvi_isp_raw raw_num = ISP_PRERAW0;
	enum cvi_isp_fe_chn_num chn_num = ISP_FE_CH0;

	if (atomic_read(&vdev->isp_streamoff) == 1 && !ctx->is_slice_buf_on) {
		vi_pr(VI_DBG, "stop streaming\n");
		return;
	}

	if (atomic_read(&vdev->isp_err_handle_flag) == 1) {
		vi_pr(VI_DBG, "wait err_handing done\n");
		return;
	}

	if (_is_fe_be_online(ctx) && !ctx->is_slice_buf_on) { //fe->be->dram->post
		if (atomic_cmpxchg(&vdev->postraw_state,
					ISP_STATE_IDLE, ISP_STATE_RUNNING) ==
					ISP_STATE_RUNNING) {
			vi_pr(VI_DBG, "Postraw is running\n");
			return;
		}

		if (_postraw_inbuf_enq_check(vdev, &raw_num, &chn_num)) {
			atomic_set(&vdev->postraw_state, ISP_STATE_IDLE);
			return;
		}

		{ //Scaler offline mode
			if (_postraw_outbuf_empty(vdev, raw_num)) {
				atomic_set(&vdev->postraw_state, ISP_STATE_IDLE);
				return;
			}

			_postraw_outbuf_enq(vdev, raw_num);
		}

		ispblk_post_yuv_cfg_update(ctx, raw_num);

		if (ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) { //YUV sensor
			if (ctx->isp_pipe_cfg[raw_num].yuv_scene_mode == ISP_YUV_SCENE_ISP) {
				postraw_tuning_update(&vdev->ctx, raw_num);

				//Update postraw dma size/addr
				_post_dma_update(vdev, raw_num);
			}
		} else { //RGB sensor
			postraw_tuning_update(&vdev->ctx, raw_num);

			//Update postraw size/ctrl flow
			_post_ctrl_update(vdev, raw_num);
			//Update postraw dma size/addr
			_post_dma_update(vdev, raw_num);
			//Update postraw stt gms/ae/hist_edge_v dma size/addr
			_swap_post_sts_buf(ctx, raw_num);
		}

		vdev->offline_raw_num = raw_num;

		ctx->cam_id = raw_num;

		isp_post_trig(ctx, raw_num);

		vi_record_post_trigger(vdev, raw_num);
	} else if (_is_be_post_online(ctx)) { //fe->dram->be->post
		if (atomic_cmpxchg(&vdev->pre_be_state[ISP_BE_CH0],
					ISP_STATE_IDLE, ISP_STATE_RUNNING) ==
					ISP_STATE_RUNNING) {
			vi_pr(VI_DBG, "Pre_be ch_num_%d is running\n", ISP_BE_CH0);
			return;
		}

		if (atomic_cmpxchg(&vdev->postraw_state,
					ISP_STATE_IDLE, ISP_STATE_RUNNING) ==
					ISP_STATE_RUNNING) {
			atomic_set(&vdev->pre_be_state[ISP_BE_CH0], ISP_STATE_IDLE);
			vi_pr(VI_DBG, "Postraw is running\n");
			return;
		}

		if (_postraw_inbuf_enq_check(vdev, &raw_num, &chn_num)) {
			atomic_set(&vdev->pre_be_state[ISP_BE_CH0], ISP_STATE_IDLE);
			atomic_set(&vdev->postraw_state, ISP_STATE_IDLE);
			return;
		}

		{ //Scaler offline mode
			if (_postraw_outbuf_empty(vdev, raw_num)) {
				atomic_set(&vdev->pre_be_state[ISP_BE_CH0], ISP_STATE_IDLE);
				atomic_set(&vdev->postraw_state, ISP_STATE_IDLE);
				return;
			}

			//only update crop, offline2sc
			_postraw_update_cfg_from_user(vdev, raw_num);
			_postraw_outbuf_enq(vdev, raw_num);
		}

		ispblk_post_yuv_cfg_update(ctx, raw_num);

		if (ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) { //YUV sensor
			if (ctx->isp_pipe_cfg[raw_num].yuv_scene_mode == ISP_YUV_SCENE_ISP) {
				postraw_tuning_update(&vdev->ctx, raw_num);

				//Update postraw dma size/addr
				_post_dma_update(vdev, raw_num);
			}
		} else { //RGB sensor
			pre_be_tuning_update(&vdev->ctx, raw_num);

			//Update pre be size/ctrl flow
			_pre_be_ctrl_update(vdev, raw_num);
			//Update pre be dma size/addr
			_pre_be_dma_update(vdev, raw_num);
			//Update pre be sts size/addr
			_swap_pre_be_sts_buf(vdev, raw_num, ISP_BE_CH0);

			postraw_tuning_update(&vdev->ctx, raw_num);

			//Update postraw size/ctrl flow
			_post_ctrl_update(vdev, raw_num);
			//Update postraw dma size/addr
			_post_dma_update(vdev, raw_num);
			//Update postraw sts awb/dci/hist_edge_v dma size/addr
			_swap_post_sts_buf(ctx, raw_num);
		}

		vdev->offline_raw_num = raw_num;

		ctx->cam_id = raw_num;

		isp_post_trig(ctx, raw_num);

	}
}

static void _splt_hw_enque(struct cvi_vi_dev *vdev, const enum cvi_isp_raw hw_raw_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	enum cvi_isp_raw raw_num = hw_raw_num;

	if (atomic_read(&vdev->isp_streamoff) == 1) {
		vi_pr(VI_DBG, "stop streaming\n");
		return;
	}

	if (atomic_read(&vdev->isp_err_handle_flag) == 1) {
		vi_pr(VI_DBG, "wait err_handing done\n");
		return;
	}

	if (_is_right_tile(ctx, raw_num))
		raw_num = ISP_PRERAW0;

	if (ctx->isp_pipe_cfg[raw_num].is_raw_replay_fe) {
		if (ctx->isp_pipe_cfg[raw_num].is_tile) {
			if (vdev->pre_fe_frm_num[raw_num][ISP_FE_CH0] !=
			    vdev->pre_fe_frm_num[raw_num + 1][ISP_FE_CH0])
				return;

			if (ctx->isp_pipe_cfg[raw_num].is_hdr_on &&
			    (vdev->pre_fe_frm_num[raw_num][ISP_FE_CH0] !=
			     vdev->pre_fe_frm_num[raw_num + 1][ISP_FE_CH1]))
				return;
		} else {
			if (ctx->isp_pipe_cfg[raw_num].is_hdr_on &&
			    (vdev->pre_fe_frm_num[raw_num][ISP_FE_CH0] !=
			     vdev->pre_fe_frm_num[raw_num][ISP_FE_CH1]))
				return;
		}

		isp_splt_trig(ctx, raw_num);
	} else if (line_spliter_en) {
		isp_splt_trig(ctx, raw_num);

	}
}

static void _pre_fe_rgbmap_update(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	const enum cvi_isp_fe_chn_num chn_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	uint8_t rgbmap_idx = (vdev->pre_fe_frm_num[raw_num][chn_num]) % RGBMAP_BUF_IDX;
	uint32_t dma_id = 0;
	uint64_t buffaddr = 0;

	if (chn_num == ISP_FE_CH0) {
		dma_id = rgbmap_dma_find_hwid(raw_num, ISP_RAW_PATH_LE);
		buffaddr = isp_bufpool[raw_num].rgbmap_le[rgbmap_idx];

		ispblk_dma_setaddr(ctx, dma_id, isp_bufpool[raw_num].rgbmap_le[rgbmap_idx]);
		vi_pr(VI_DBG, "dmaid=%d buffaddr=0x%llx\n", dma_id, buffaddr);

		if (ctx->isp_pipe_cfg[raw_num].is_tile) {
			u32 grid_size = (1 << g_w_bit[raw_num]);
			u32 w = ctx->isp_pipe_cfg[raw_num].crop.w;

			dma_id = rgbmap_dma_find_hwid(raw_num + 1, ISP_RAW_PATH_LE);
			buffaddr += ((w + grid_size - 1) / grid_size) * 6;
			ispblk_dma_setaddr(ctx, dma_id, buffaddr);
			vi_pr(VI_DBG, "dmaid=%d buffaddr=0x%llx\n", dma_id, buffaddr);
		}
	} else if (chn_num == ISP_FE_CH1) {
		dma_id = rgbmap_dma_find_hwid(raw_num, ISP_RAW_PATH_SE);
		buffaddr = isp_bufpool[raw_num].rgbmap_se[rgbmap_idx];

		ispblk_dma_setaddr(ctx, dma_id, isp_bufpool[raw_num].rgbmap_se[rgbmap_idx]);
		if (ctx->isp_pipe_cfg[raw_num].is_tile) {
			u32 grid_size = (1 << g_w_bit[raw_num]);
			u32 w = ctx->isp_pipe_cfg[raw_num].crop.w;

			dma_id = rgbmap_dma_find_hwid(raw_num + 1, ISP_RAW_PATH_SE);
			buffaddr += ((w + grid_size - 1) / grid_size) * 6;
			ispblk_dma_setaddr(ctx, dma_id, buffaddr);
		}
	}
}

void vi_destory_thread(struct cvi_vi_dev *vdev, enum E_VI_TH th_id)
{
	if (th_id < 0 || th_id >= E_VI_TH_MAX) {
		pr_err("No such thread_id(%d)\n", th_id);
		return;
	}

	if (vdev->vi_th[th_id].w_thread != NULL) {
		int ret;

		ret = kthread_stop(vdev->vi_th[th_id].w_thread);
		if (!ret) {
			while (atomic_read(&vdev->vi_th[th_id].thread_exit) == 0) {
				pr_info("wait for %s exit\n", vdev->vi_th[th_id].th_name);
				usleep_range(5 * 1000, 10 * 1000);
			}
		}
		vdev->vi_th[th_id].w_thread = NULL;
	}
}

int vi_create_thread(struct cvi_vi_dev *vdev, enum E_VI_TH th_id)
{
	struct sched_param param;
	int rc = 0;

	if (th_id < 0 || th_id >= E_VI_TH_MAX) {
		pr_err("No such thread_id(%d)\n", th_id);
		return -1;
	}

	param.sched_priority = MAX_USER_RT_PRIO - 10;

	if (vdev->vi_th[th_id].w_thread == NULL) {
		switch (th_id) {
		case E_VI_TH_PRERAW:
			memcpy(vdev->vi_th[th_id].th_name, "cvitask_isp_pre",
				sizeof(vdev->vi_th[th_id].th_name));
			vdev->vi_th[th_id].th_handler = _vi_preraw_thread;
			break;
		case E_VI_TH_VBLANK_HANDLER:
			memcpy(vdev->vi_th[th_id].th_name, "cvitask_isp_blank",
				sizeof(vdev->vi_th[th_id].th_name));
			vdev->vi_th[th_id].th_handler = _vi_vblank_handler_thread;
			break;
		case E_VI_TH_ERR_HANDLER:
			memcpy(vdev->vi_th[th_id].th_name, "cvitask_isp_err",
				sizeof(vdev->vi_th[th_id].th_name));
			vdev->vi_th[th_id].th_handler = _vi_err_handler_thread;
			break;
		case E_VI_TH_EVENT_HANDLER:
			memcpy(vdev->vi_th[th_id].th_name, "vi_event_handler",
				sizeof(vdev->vi_th[th_id].th_name));
			vdev->vi_th[th_id].th_handler = _vi_event_handler_thread;
			break;
		case E_VI_TH_RUN_TPU1:
			memcpy(vdev->vi_th[th_id].th_name, "ai_isp_tpu1",
				sizeof(vdev->vi_th[th_id].th_name));
			vdev->vi_th[th_id].th_handler = _vi_run_tpu_thread1;
			break;
		case E_VI_TH_RUN_TPU2:
			memcpy(vdev->vi_th[th_id].th_name, "ai_isp_tpu2",
				sizeof(vdev->vi_th[th_id].th_name));
			vdev->vi_th[th_id].th_handler = _vi_run_tpu_thread2;
			break;
		default:
			pr_err("No such thread(%d)\n", th_id);
			return -1;
		}
		vdev->vi_th[th_id].w_thread = kthread_create(vdev->vi_th[th_id].th_handler,
								(void *)vdev,
								vdev->vi_th[th_id].th_name);
		if (IS_ERR(vdev->vi_th[th_id].w_thread)) {
			pr_err("Unable to start %s.\n", vdev->vi_th[th_id].th_name);
			return -1;
		}

		sched_setscheduler(vdev->vi_th[th_id].w_thread, SCHED_FIFO, &param);

		vdev->vi_th[th_id].flag = 0;
		atomic_set(&vdev->vi_th[th_id].thread_exit, 0);
		init_waitqueue_head(&vdev->vi_th[th_id].wq);
		wake_up_process(vdev->vi_th[th_id].w_thread);
	} else {
		pr_err("thread(%s) already exist\n", vdev->vi_th[th_id].th_name);
	}

	return rc;
}

static void _vi_sw_init(struct cvi_vi_dev *vdev)
{
	struct isp_ctx *ctx = &vdev->ctx;
	struct cvi_vi_ctx *pviProcCtx = NULL;
	u8 i = 0, j = 0;

	pviProcCtx = (struct cvi_vi_ctx *)(vdev->shared_mem);

#if (KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE)
	timer_setup(&usr_pic_timer.t, legacy_timer_emu_func, 0);
#else
	init_timer(&usr_pic_timer);
#endif
	ctx->is_tile            = false;
	ctx->is_offline_be      = true;
	ctx->is_offline_postraw = false;
	ctx->is_3dnr_on         = true;
	ctx->is_dpcm_on         = false;
	ctx->is_hdr_on          = false;
	ctx->is_multi_sensor    = false;
	ctx->is_sublvds_path    = false;
	ctx->is_fbc_on          = true;
	ctx->is_ctrl_inited     = false;
	ctx->is_slice_buf_on    = false;
	ctx->is_rgbmap_sbm_on   = false;
	ctx->is_3dnr_old2new    = false;
	ctx->rgbmap_prebuf_idx  = 1;
	ctx->cam_id             = 0;
	ctx->total_chn_num      = 0;
	ctx->gamma_tbl_idx      = 0;
	vdev->postraw_proc_num  = 0;
	vdev->tpu_thread_num    = 0;
	vdev->usr_pic_delay     = 0;
	vdev->isp_source        = CVI_ISP_SOURCE_DEV;

	if (pviProcCtx->vi_stt != VI_SUSPEND)
		memset(vdev->snr_info, 0, sizeof(struct cvi_isp_snr_info) * ISP_PRERAW_MAX);

	for (i = 0; i < ISP_RAW_PATH_MAX; i++) {
		vdev->usr_pic_phy_addr[i] = 0;
	}

	for (i = 0; i < ISP_PRERAW_MAX; i++) {
		vdev->preraw_first_frm[i]     = true;
		vdev->postraw_frame_number[i] = 0;
		vdev->drop_frame_number[i]    = 0;
		vdev->dump_frame_number[i]    = 0;
		vdev->isp_int_flag[i]         = false;
		ctx->mmap_grid_size[i]        = 3;
		ctx->isp_pipe_enable[i]       = false;
		ctx->raw_chnstr_num[i]        = 0;
		ctx->isp_bind_info[i].is_bind = false;
		ctx->isp_bind_info[i].bind_dev_num = ISP_PRERAW_MAX;
		ctx->isp_pipe_cfg[i].is_raw_ai_isp = false;

		memset(&ctx->isp_pipe_cfg[i], 0, sizeof(struct _isp_cfg));
		ctx->isp_pipe_cfg[i].is_offline_scaler = true;
		ctx->isp_pipe_cfg[i].tnr_mode = (ctx->is_3dnr_on) ?
						ISP_TNR_TYPE_OLD_MODE :
						ISP_TNR_TYPE_BYPASS_MODE;

		for (j = 0; j < ISP_FE_CHN_MAX; j++) {
			vdev->pre_fe_sof_cnt[i][j] = 0;
			vdev->pre_fe_frm_num[i][j] = 0;

			atomic_set(&vdev->pre_fe_state[i][j], ISP_STATE_IDLE);
		}

		for (j = 0; j < ISP_BE_CHN_MAX; j++) {
			vdev->pre_be_frm_num[i][j] = 0;
		}

		spin_lock_init(&snr_node_lock[i]);

		atomic_set(&vdev->isp_raw_dump_en[i], 0);
		atomic_set(&vdev->isp_smooth_raw_dump_en[i], 0);
	}

	for (i = 0; i < ISP_SPLT_MAX; i++) {
		for (j = 0; j < ISP_SPLT_CHN_MAX; j++) {
			vdev->splt_wdma_frm_num[i][j] = 0;
			vdev->splt_rdma_frm_num[i][j] = 0;

			INIT_LIST_HEAD(&splt_out_q[i][j].rdy_queue);
			INIT_LIST_HEAD(&pre_fe_in_q[i][j].rdy_queue);
			splt_out_q[i][j].num_rdy        = 0;
			splt_out_q[i][j].raw_num        = i;
			pre_fe_in_q[i][j].num_rdy       = 0;
			pre_fe_in_q[i][j].raw_num       = i;

			atomic_set(&vdev->splt_state[i][j], ISP_STATE_IDLE);
		}
	}

	for (i = 0; i < ISP_PRERAW_MAX; i++) {
		for (j = 0; j < ISP_FE_CHN_MAX; j++) {
			INIT_LIST_HEAD(&pre_fe_out_q[i][j].rdy_queue);
			pre_fe_out_q[i][j].num_rdy      = 0;
			pre_fe_out_q[i][j].raw_num      = i;
		}

		INIT_LIST_HEAD(&raw_dump_b_dq[i].rdy_queue);
		INIT_LIST_HEAD(&raw_dump_b_se_dq[i].rdy_queue);
		raw_dump_b_dq[i].num_rdy     = 0;
		raw_dump_b_dq[i].raw_num     = i;
		raw_dump_b_se_dq[i].num_rdy  = 0;
		raw_dump_b_se_dq[i].raw_num  = i;

		INIT_LIST_HEAD(&raw_dump_b_q[i].rdy_queue);
		INIT_LIST_HEAD(&raw_dump_b_se_q[i].rdy_queue);
		raw_dump_b_q[i].num_rdy      = 0;
		raw_dump_b_q[i].raw_num      = i;
		raw_dump_b_se_q[i].num_rdy   = 0;
		raw_dump_b_se_q[i].raw_num   = i;

		INIT_LIST_HEAD(&isp_snr_i2c_queue[i].list);
		isp_snr_i2c_queue[i].num_rdy = 0;

		INIT_LIST_HEAD(&pre_be_in_se_q[i].rdy_queue);
		pre_be_in_se_q[i].num_rdy    = 0;

		// for ai isp init
		for (j = 0; j < ISP_FE_CHN_MAX; j++) {
			INIT_LIST_HEAD(&bnr_ai_isp_q[i][j].rdy_queue);
			bnr_ai_isp_q[i][j].num_rdy = 0;
			bnr_ai_isp_q[i][j].raw_num = i;
		}
		atomic_set(&vdev->bnr_run_tpu[i], 0);
		atomic_set(&vdev->ai_isp_int_flag[i], 0);
		init_completion(&vdev->tpu_done[i]);
		init_waitqueue_head(&vdev->ai_isp_wait_q[i]);
	}

	INIT_LIST_HEAD(&pre_raw_num_q.list);
	INIT_LIST_HEAD(&event_q.list);

	INIT_LIST_HEAD(&pre_be_in_q.rdy_queue);
	pre_be_in_q.num_rdy     = 0;

	for (i = 0; i < ISP_RAW_PATH_MAX; i++) {
		INIT_LIST_HEAD(&pre_be_out_q[i].rdy_queue);
		INIT_LIST_HEAD(&postraw_in_q[i].rdy_queue);
		pre_be_out_q[i].num_rdy     = 0;
		postraw_in_q[i].num_rdy     = 0;
	}

	for (i = 0; i < VI_MAX_CHN_NUM; i++) {
		INIT_LIST_HEAD(&dqbuf_q[i].list);
		init_waitqueue_head(&vdev->isp_dq_wait_q[i]);

		init_waitqueue_head(&vdev->yuv_dump_wait_q[i]);
	}

	atomic_set(&vdev->pre_be_state[ISP_BE_CH0], ISP_STATE_IDLE);
	atomic_set(&vdev->pre_be_state[ISP_BE_CH1], ISP_STATE_IDLE);
	atomic_set(&vdev->postraw_state, ISP_STATE_IDLE);
	atomic_set(&vdev->isp_streamoff, 1);
	atomic_set(&vdev->isp_err_handle_flag, 0);
	atomic_set(&vdev->ol_sc_frm_done, 1);
	atomic_set(&vdev->isp_dbg_flag, 0);
	atomic_set(&vdev->ctx.is_post_done, 0);

	atomic_set(&vdev->ai_isp_type, AI_ISP_TYPE_BUTT);
	mutex_init(&vdev->ai_isp_lock);

	spin_lock_init(&buf_lock);
	spin_lock_init(&raw_num_lock);
	spin_lock_init(&dq_lock);
	spin_lock_init(&event_lock);
	spin_lock_init(&vdev->qbuf_lock);

	init_waitqueue_head(&vdev->isp_event_wait_q);
	init_waitqueue_head(&vdev->isp_dbg_wait_q);

	vi_tuning_sw_init();
}

static void _vi_init_param(struct cvi_vi_dev *vdev)
{
	struct isp_ctx *ctx = &vdev->ctx;
	int enable_dev_num = vdev->num_dev;
	uint8_t i = 0;

	atomic_set(&dev_open_cnt, 0);

	memset(ctx, 0, sizeof(*ctx));

	ctx->phys_regs = isp_get_phys_reg_bases();
	ctx->cam_id    = 0;

	for (i = 0; i < ISP_PRERAW_MAX; i++) {
		ctx->rgb_color_mode[i]                  = ISP_BAYER_TYPE_GB;
		ctx->isp_pipe_cfg[i].is_patgen_en       = false;
		ctx->isp_pipe_cfg[i].is_raw_replay_be   = false;
		ctx->isp_pipe_cfg[i].is_yuv_sensor = false;
		ctx->isp_pipe_cfg[i].is_drop_next_frame = false;
		ctx->isp_pipe_cfg[i].isp_reset_frm      = 0;
		ctx->isp_pipe_cfg[i].is_422_to_420      = false;
		ctx->isp_pipe_cfg[i].max_height         = 0;
		ctx->isp_pipe_cfg[i].max_width          = 0;
		ctx->isp_pipe_cfg[i].csibdg_width       = 0;
		ctx->isp_pipe_cfg[i].csibdg_height      = 0;
		ctx->isp_pipe_cfg[i].yuv_scene_mode     = ISP_YUV_SCENE_BYPASS;

		gViCtx->isDevEnable[i] = false;

		INIT_LIST_HEAD(&raw_dump_b_dq[i].rdy_queue);
		INIT_LIST_HEAD(&raw_dump_b_se_dq[i].rdy_queue);
		raw_dump_b_dq[i].num_rdy         = 0;
		raw_dump_b_dq[i].raw_num         = i;
		raw_dump_b_se_dq[i].num_rdy      = 0;
		raw_dump_b_se_dq[i].raw_num      = i;

		INIT_LIST_HEAD(&raw_dump_b_q[i].rdy_queue);
		INIT_LIST_HEAD(&raw_dump_b_se_q[i].rdy_queue);
		raw_dump_b_q[i].num_rdy          = 0;
		raw_dump_b_q[i].raw_num          = i;
		raw_dump_b_se_q[i].num_rdy       = 0;
		raw_dump_b_se_q[i].raw_num       = i;

		INIT_LIST_HEAD(&isp_snr_i2c_queue[i].list);
		isp_snr_i2c_queue[i].num_rdy     = 0;

		INIT_LIST_HEAD(&pre_be_in_se_q[i].rdy_queue);
		pre_be_in_se_q[i].num_rdy        = 0;
	}

	INIT_LIST_HEAD(&pre_raw_num_q.list);

	memset(&vdev->usr_crop, 0, sizeof(vdev->usr_crop));

	for (i = 0; i < VI_MAX_CHN_NUM; i++) {
		INIT_LIST_HEAD(&vdev->qbuf_list[i]);
		vdev->qbuf_num[i] = 0;
		INIT_LIST_HEAD(&vdev->dqbuf_list[i]);
		vdev->dqbuf_num[i] = 0;
	}

	for (i = 0; i < ISP_PRERAW_MAX; i++) {
		vdev->isp_int_flag[i] = false;
		init_waitqueue_head(&vdev->isp_int_wait_q[i]);
	}

	//ToDo sync_task_ext
	for (i = 0; i < enable_dev_num; i++)
		sync_task_init(i);

	tasklet_init(&vdev->job_work, isp_post_tasklet, (unsigned long)vdev);

	atomic_set(&vdev->isp_streamon, 0);
}

int vi_mac_clk_ctrl(struct cvi_vi_dev *vdev, u8 mac_num, u8 enable)
{
	int rc = 0;

	if (mac_num >= ARRAY_SIZE(vdev->clk_mac))
		return rc;

	if (vdev->clk_mac[mac_num]) {
		if (enable) {
			if (clk_prepare_enable(vdev->clk_mac[mac_num])) {
				vi_pr(VI_ERR, "Failed to prepare and enable clk_mac(%d)\n", mac_num);
				rc = -EAGAIN;
				goto EXIT;
			}
		} else {
			if (__clk_is_enabled(vdev->clk_mac[mac_num]))
				clk_disable_unprepare(vdev->clk_mac[mac_num]);
			else
				clk_unprepare(vdev->clk_mac[mac_num]);
		}
	} else {
		vi_pr(VI_ERR, "clk_mac(%d) is null\n", mac_num);
		rc = -EAGAIN;
		goto EXIT;
	}

	vi_pr(VI_INFO, "clk_mac(%d) enalbe[%d]\n", mac_num, enable);
EXIT:
	return rc;
}

#ifndef FPGA_PORTING
static int _vi_clk_ctrl(struct cvi_vi_dev *vdev, u8 enable)
{
	u8 i = 0;
	int rc = 0;

	for (i = 0; i < ARRAY_SIZE(vdev->clk_sys); ++i) {
		if (vdev->clk_sys[i]) {
			if (enable) {
				if (clk_prepare_enable(vdev->clk_sys[i])) {
					vi_pr(VI_ERR, "Failed to prepare and enable clk_sys(%d)\n", i);
					rc = -EAGAIN;
					goto EXIT;
				}
			} else {
				if (__clk_is_enabled(vdev->clk_sys[i]))
					clk_disable_unprepare(vdev->clk_sys[i]);
				else
					clk_unprepare(vdev->clk_sys[i]);
			}
		} else {
			vi_pr(VI_ERR, "clk_sys(%d) is null\n", i);
			rc = -EAGAIN;
			goto EXIT;
		}
	}

	for (i = 0; i < ARRAY_SIZE(vdev->clk_isp); ++i) {
		if (vdev->clk_isp[i]) {
			if (enable) {
				if (clk_prepare_enable(vdev->clk_isp[i])) {
					vi_pr(VI_ERR, "Failed to enable clk_isp(%d)\n", i);
					rc = -EAGAIN;
					goto EXIT;
				}
			} else {
				if (__clk_is_enabled(vdev->clk_isp[i]))
					clk_disable_unprepare(vdev->clk_isp[i]);
				else
					clk_unprepare(vdev->clk_isp[i]);
			}
		} else {
			vi_pr(VI_ERR, "clk_isp(%d) is null\n", i);
			rc = -EAGAIN;
			goto EXIT;
		}
	}

	//Set axi_isp_top_clk_en
	v4l2_sys_reg_write_mask(VI_SYS_REG_CLK_AXI_ISP_TOP_EN,
			VI_SYS_REG_CLK_AXI_ISP_TOP_EN_MASK,
			enable << VI_SYS_REG_CLK_AXI_ISP_TOP_EN_OFFSET);

EXIT:
	return rc;
}
#endif

void _vi_sdk_release(struct cvi_vi_dev *vdev)
{
	u8 i = 0;
	struct isp_ctx *ctx = &vdev->ctx;

	// vi_disable_chn(0);

	for (i = 0; i < VI_MAX_CHN_NUM; i++) {
		memset(&gViCtx->chnAttr[i], 0, sizeof(VI_CHN_ATTR_S));
		memset(&gViCtx->chnStatus[i], 0, sizeof(VI_CHN_STATUS_S));
		gViCtx->blk_size[i] = 0;
		gViCtx->bypass_frm[i] = 0;
	}

	for (i = 0; i < VI_MAX_PIPE_NUM; i++)
		gViCtx->isPipeCreated[i] = false;

	for (i = 0; i < VI_MAX_DEV_NUM; i++)
		gViCtx->isDevEnable[i] = false;

	for (i = 0; i < ISP_PRERAW_MAX; i++) {
		memset(&ctx->isp_pipe_cfg[i], 0, sizeof(struct _isp_cfg));
		ctx->isp_pipe_cfg[i].is_offline_scaler = true;
	}
}

static int _vi_create_proc(struct cvi_vi_dev *vdev)
{
	struct file *filp = NULL;
	int ret = 0;

	// make sure /proc/v4l2 exist
	filp = filp_open("/proc/v4l2", O_RDONLY, 0);
	if (IS_ERR(filp)) {
		proc_mkdir("v4l2", NULL);
	} else {
		filp_close(filp, 0);
	}

	/* vi proc setup */
	vdev->shared_mem = kzalloc(VI_SHARE_MEM_SIZE, GFP_ATOMIC);
	if (!vdev->shared_mem) {
		pr_err("shared_mem alloc size(%d) failed\n", VI_SHARE_MEM_SIZE);
		return -ENOMEM;
	}

	if (vi_proc_init(vdev, vdev->shared_mem) < 0) {
		pr_err("vi proc init failed\n");
		return -EAGAIN;
	}

	if (vi_dbg_proc_init(vdev) < 0) {
		pr_err("vi_dbg proc init failed\n");
		return -EAGAIN;
	}

	if (isp_proc_init(vdev) < 0) {
		pr_err("isp proc init failed\n");
		return -EAGAIN;
	}

	return ret;
}

static void _vi_destroy_proc(struct cvi_vi_dev *vdev)
{
	vi_proc_remove();
	vi_dbg_proc_remove();
	kfree(vdev->shared_mem);
	vdev->shared_mem = NULL;

	isp_proc_remove();
}

static int _vi_event_handler_thread(void *arg)
{
	struct cvi_vi_dev *vdev = (struct cvi_vi_dev *)arg;
#ifdef FPGA_PORTING
	u32 timeout = 100000;//ms
#else
	u32 timeout = 2000;//ms
#endif
	int ret = 0, flag = 0;
	enum E_VI_TH th_id = E_VI_TH_EVENT_HANDLER;

	while (1) {
		ret = wait_event_timeout(vdev->vi_th[th_id].wq,
					vdev->vi_th[th_id].flag != 0 || kthread_should_stop(),
					msecs_to_jiffies(timeout) - 1);

		if (vdev->vi_th[th_id].flag != 0) {
			flag = vdev->vi_th[th_id].flag - 1;
			vdev->vi_th[th_id].flag = 0;
		}

		if (kthread_should_stop()) {
			pr_info("%s exit\n", vdev->vi_th[th_id].th_name);
			atomic_set(&vdev->vi_th[th_id].thread_exit, 1);
			do_exit(1);
		}

		if (!ret) {
			vi_pr(VI_INFO, "raw_%d vi_event_handler timeout(%d)ms\n", flag, timeout);
			if (++gViCtx->timeout_cnt >= 2) {
				atomic_set(&vdev->isp_dbg_flag, 1);
				wake_up(&vdev->isp_dbg_wait_q);
				gViCtx->timeout_cnt = 0;
			}
			continue;
		} else {
			u8 raw_num = flag;
			struct cvi_isp_buf *b = NULL;
			int index;
			u32 frm_num = vdev->postraw_frame_number[raw_num];
			bool is_bypass = frm_num > gViCtx->bypass_frm[raw_num] ? 0 : 1;
			unsigned long flags;

			spin_lock_irqsave(&vdev->qbuf_lock, flags);
			if (!list_empty(&vdev->qbuf_list[raw_num])) {
				b = list_first_entry(&vdev->qbuf_list[raw_num], struct cvi_isp_buf, list);
				index = b->buf.vb2_buf.index;

				if (atomic_read(&vdev->isp_dump_yuv[raw_num])){
					struct vb2_buffer *vb2_buf = NULL;
					vi_pr(VI_INFO, "start to dump yuv\n");
					vb2_buf = &(b->buf.vb2_buf);
					copy_vb2buf_to_dump(vb2_buf);
					atomic_set(&vdev->isp_dump_yuv[raw_num], 0);
					wake_up(&vdev->yuv_dump_wait_q[raw_num]);
					vi_pr(VI_INFO, "dump yuv success!\n");
				}

				if (is_bypass) {
					vi_pr(VI_INFO, "raw_%d bypass frm_%d\n", raw_num, frm_num);
					//memory write workaround, warm up all vb buffer
					if (frm_num <= vdev->qbuf_num[raw_num])
						list_move_tail(&b->list, &vdev->qbuf_list[raw_num]);
				} else {
					list_move_tail(&b->list, &vdev->dqbuf_list[raw_num]);
				}

				vi_pr(VI_DBG, "raw_%d index:%d\n", raw_num, index);

				if (list_is_singular(&vdev->qbuf_list[raw_num])) {
					b = list_first_entry(&vdev->dqbuf_list[raw_num], struct cvi_isp_buf, list);
					index = b->buf.vb2_buf.index;
					list_move(&b->list, &vdev->qbuf_list[raw_num]);
					vdev->qbuf_num[raw_num]++;
					vi_pr(VI_DBG, "raw_%d qbuf run out, take vb(%d) to it's head!\n", raw_num, index);
				}
			} else {
				vi_pr(VI_INFO, "raw_%d qbuf is empty!\n", raw_num);
				spin_unlock_irqrestore(&vdev->qbuf_lock, flags);
				continue;
			}
			spin_unlock_irqrestore(&vdev->qbuf_lock, flags);

			if (is_bypass) {
				struct _isp_dqbuf_n *n = NULL;
				spin_lock_irqsave(&dq_lock, flags);
				if (!list_empty(&dqbuf_q[raw_num].list)) {
					n = list_first_entry(&dqbuf_q[raw_num].list, struct _isp_dqbuf_n, list);
					list_del_init(&n->list);
					kfree(n);
				}
				spin_unlock_irqrestore(&dq_lock, flags);
			}

			if (atomic_read(&vdev->post_dq_flag[raw_num]) == 1 && !is_bypass) {
				wake_up(&vdev->isp_dq_wait_q[raw_num]);
				atomic_set(&vdev->post_dq_flag[raw_num], 0);
			}
		}
	}

	return 0;
}

/*******************************************************
 *  Irq handlers
 ******************************************************/

static void _vi_record_debug_info(struct isp_ctx *ctx)
{
	uint8_t i = 0;
	uintptr_t isptop = ctx->phys_regs[ISP_BLK_ID_ISPTOP];
	uintptr_t preraw_fe = ctx->phys_regs[ISP_BLK_ID_PRE_RAW_FE0];
	uintptr_t preraw_be = ctx->phys_regs[ISP_BLK_ID_PRE_RAW_BE];
	uintptr_t yuvtop = ctx->phys_regs[ISP_BLK_ID_YUVTOP];
	uintptr_t rgbtop = ctx->phys_regs[ISP_BLK_ID_RGBTOP];
	uintptr_t rawtop = ctx->phys_regs[ISP_BLK_ID_RAWTOP];
	uintptr_t rdma28 = ctx->phys_regs[ISP_BLK_ID_DMA_CTL_RAW_RDMA0];
	struct cvi_vi_info *vi_info = NULL;

	if (gOverflowInfo != NULL)
		return;

	gOverflowInfo = kzalloc(sizeof(struct cvi_overflow_info), GFP_ATOMIC);
	if (gOverflowInfo == NULL) {
		vi_pr(VI_ERR, "gOverflowInfo kzalloc size(%zu) fail\n", sizeof(struct cvi_overflow_info));
		return;
	}

	vi_info = &gOverflowInfo->vi_info;

	//isp_top
	vi_info->isp_top.blk_idle = ISP_RD_REG(isptop, REG_ISP_TOP_T, BLK_IDLE);
	for (i = 0; i <= 6; i++) {
		//Debug
		ISP_WR_BITS(isptop, REG_ISP_TOP_T, DUMMY, DBUS_SEL, i);
		vi_info->isp_top.dbus_sel[i].r_0 = ISP_RD_REG(isptop, REG_ISP_TOP_T, DBUS0); //0x0A070040
		vi_info->isp_top.dbus_sel[i].r_4 = ISP_RD_REG(isptop, REG_ISP_TOP_T, DBUS1); //0x0A070044
		vi_info->isp_top.dbus_sel[i].r_8 = ISP_RD_REG(isptop, REG_ISP_TOP_T, DBUS2); //0x0A070048
		vi_info->isp_top.dbus_sel[i].r_c = ISP_RD_REG(isptop, REG_ISP_TOP_T, DBUS3); //0x0A07004C
	}

	//pre_raw_fe
	vi_info->preraw_fe.preraw_info = ISP_RD_REG(preraw_fe, REG_PRE_RAW_FE_T, PRE_RAW_INFO);
	vi_info->preraw_fe.fe_idle_info = ISP_RD_REG(preraw_fe, REG_PRE_RAW_FE_T, FE_IDLE_INFO);

	//pre_raw_be
	vi_info->preraw_be.preraw_be_info = ISP_RD_REG(preraw_be, REG_PRE_RAW_BE_T, BE_INFO);
	vi_info->preraw_be.be_dma_idle_info = ISP_RD_REG(preraw_be, REG_PRE_RAW_BE_T, BE_DMA_IDLE_INFO);
	vi_info->preraw_be.ip_idle_info = ISP_RD_REG(preraw_be, REG_PRE_RAW_BE_T, BE_IP_IDLE_INFO);
	vi_info->preraw_be.stvalid_status = ISP_RD_REG(preraw_be, REG_PRE_RAW_BE_T, TVALID_STATUS);
	vi_info->preraw_be.stready_status = ISP_RD_REG(preraw_be, REG_PRE_RAW_BE_T, TREADY_STATUS);

	//rawtop
	vi_info->rawtop.stvalid_status = ISP_RD_REG(rawtop, REG_RAW_TOP_T, STVALID_STATUS);
	vi_info->rawtop.stready_status = ISP_RD_REG(rawtop, REG_RAW_TOP_T, STREADY_STATUS);
	vi_info->rawtop.dma_idle = ISP_RD_REG(rawtop, REG_RAW_TOP_T, DMA_IDLE);

#if 0
	ISP_WR_BITS(isptop, REG_RAW_TOP_T, DEBUG_SELECT, RAW_TOP_DEBUG_SELECT, 0);
	vi_pr(VI_INFO, "RAW_TOP, debug_select(h2c)=0x0, debug(h28)=0x%x\n",
		ISP_RD_REG(rawtop, REG_RAW_TOP_T, DEBUG));

	ISP_WR_BITS(isptop, REG_RAW_TOP_T, DEBUG_SELECT, RAW_TOP_DEBUG_SELECT, 4);
	vi_pr(VI_INFO, "RAW_TOP, debug_select(h2c)=0x4, debug(h28)=0x%x\n",
		ISP_RD_REG(rawtop, REG_RAW_TOP_T, DEBUG));
#endif

	//rgbtop
	vi_info->rgbtop.ip_stvalid_status = ISP_RD_REG(rgbtop, REG_ISP_RGB_TOP_T, DBG_IP_S_VLD);
	vi_info->rgbtop.ip_stready_status = ISP_RD_REG(rgbtop, REG_ISP_RGB_TOP_T, DBG_IP_S_RDY);
	vi_info->rgbtop.dmi_stvalid_status = ISP_RD_REG(rgbtop, REG_ISP_RGB_TOP_T, DBG_DMI_VLD);
	vi_info->rgbtop.dmi_stready_status = ISP_RD_REG(rgbtop, REG_ISP_RGB_TOP_T, DBG_DMI_RDY);
	vi_info->rgbtop.xcnt_rpt = ISP_RD_BITS(rgbtop, REG_ISP_RGB_TOP_T, PATGEN4, XCNT_RPT);
	vi_info->rgbtop.ycnt_rpt = ISP_RD_BITS(rgbtop, REG_ISP_RGB_TOP_T, PATGEN4, YCNT_RPT);

	//yuvtop
	vi_info->yuvtop.debug_state = ISP_RD_REG(yuvtop, REG_YUV_TOP_T, YUV_DEBUG_STATE);
	vi_info->yuvtop.stvalid_status = ISP_RD_REG(yuvtop, REG_YUV_TOP_T, STVALID_STATUS);
	vi_info->yuvtop.stready_status = ISP_RD_REG(yuvtop, REG_YUV_TOP_T, STREADY_STATUS);
	vi_info->yuvtop.xcnt_rpt = ISP_RD_BITS(yuvtop, REG_YUV_TOP_T, PATGEN4, XCNT_RPT);
	vi_info->yuvtop.ycnt_rpt = ISP_RD_BITS(yuvtop, REG_YUV_TOP_T, PATGEN4, YCNT_RPT);

	//rdma28
	ISP_WR_BITS(rdma28, REG_ISP_DMA_CTL_T, SYS_CONTROL, DBG_SEL, 0x1);
	vi_info->rdma28[0].dbg_sel = 0x1;
	vi_info->rdma28[0].status = ISP_RD_REG(rdma28, REG_ISP_DMA_CTL_T, DMA_STATUS);
	ISP_WR_BITS(rdma28, REG_ISP_DMA_CTL_T, SYS_CONTROL, DBG_SEL, 0x2);
	vi_info->rdma28[1].dbg_sel = 0x2;
	vi_info->rdma28[1].status = ISP_RD_REG(rdma28, REG_ISP_DMA_CTL_T, DMA_STATUS);
	vi_info->enable = true;
}

static void _vi_show_debug_info(void)
{
	struct cvi_vi_info *vi_info = NULL;
	uint8_t i = 0;

	if (gOverflowInfo == NULL)
		return;

	vi_info = &gOverflowInfo->vi_info;

	if (vi_info->enable) {
		vi_info->enable = false;
		pr_info("ISP_TOP, blk_idle(h38)=0x%x\n", vi_info->isp_top.blk_idle);
		for (i = 0; i <= 6; i++) {
			pr_info("dbus_sel=%d, r_0=0x%x, r_4=0x%x, r_8=0x%x, r_c=0x%x\n",
				i,
				vi_info->isp_top.dbus_sel[i].r_0,
				vi_info->isp_top.dbus_sel[i].r_4,
				vi_info->isp_top.dbus_sel[i].r_8,
				vi_info->isp_top.dbus_sel[i].r_c);
		}
		pr_info("PRE_RAW_FE0, preraw_info(h34)=0x%x, fe_idle_info(h50)=0x%x\n",
			vi_info->preraw_fe.preraw_info,
			vi_info->preraw_fe.fe_idle_info);
		pr_info("PRE_RAW_BE, preraw_be_info(h14)=0x%x, be_dma_idle_info(h18)=0x%x, ip_idle_info(h1c)=0x%x\n",
			vi_info->preraw_be.preraw_be_info,
			vi_info->preraw_be.be_dma_idle_info,
			vi_info->preraw_be.ip_idle_info);
		pr_info("PRE_RAW_BE, stvalid_status(h28)=0x%x, stready_status(h2c)=0x%x\n",
			vi_info->preraw_be.stvalid_status,
			vi_info->preraw_be.stready_status);
		pr_info("RAW_TOP, stvalid_status(h40)=0x%x, stready_status(h44)=0x%x, dma_idle(h60)=0x%x\n",
			vi_info->rawtop.stvalid_status,
			vi_info->rawtop.stready_status,
			vi_info->rawtop.dma_idle);
		pr_info("RGB_TOP, ip_stvalid_status(h50)=0x%x, ip_stready_status(h54)=0x%x\n",
			vi_info->rgbtop.ip_stvalid_status,
			vi_info->rgbtop.ip_stready_status);
		pr_info("RGB_TOP, dmi_stvalid_status(h58)=0x%x, dmi_stready_status(h5c)=0x%x\n",
			vi_info->rgbtop.dmi_stvalid_status,
			vi_info->rgbtop.dmi_stready_status);
		pr_info("RGB_TOP xcnt_rpt=0x%x, ycnt_rpt=0x%x\n",
			vi_info->rgbtop.xcnt_rpt,
			vi_info->rgbtop.ycnt_rpt);
		pr_info("YUV_TOP debug_state(h18)=0x%x, stvalid_status(h6c)=0x%x, stready_status(h70)=0x%x\n",
			vi_info->yuvtop.debug_state,
			vi_info->yuvtop.stvalid_status,
			vi_info->yuvtop.stready_status);
		pr_info("YUV_TOP xcnt_rpt=0x%x, ycnt_rpt=0x%x\n",
			vi_info->yuvtop.xcnt_rpt,
			vi_info->yuvtop.ycnt_rpt);
		pr_info("rdma28, dbg_sel(h000)=0x%x, status(h014)=0x%x\n",
			vi_info->rdma28[0].dbg_sel,
			vi_info->rdma28[0].status);
		pr_info("rdma28, dbg_sel(h000)=0x%x, status(h014)=0x%x\n",
			vi_info->rdma28[1].dbg_sel,
			vi_info->rdma28[1].status);
	}

	kfree(gOverflowInfo);
	gOverflowInfo = NULL;
}

void _vi_err_handler(struct cvi_vi_dev *vdev, const enum cvi_isp_raw err_raw_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	int i, j, count = 10;
	bool fe_idle, be_idle, post_idle;

	//Stop pre/postraw trigger go
	atomic_set(&vdev->isp_err_handle_flag, 1);

	//step 1 : set frm vld = 0
	isp_frm_err_handler(ctx, err_raw_num, 1);

	//step 2 : wait to make sure post and the other fe is done.
	while (--count > 0) {
		if (_is_be_post_online(ctx)) {
			if (ctx->is_multi_sensor) {
				fe_idle = be_idle = post_idle = true;

				for (i = ISP_PRERAW0; i < ISP_PRERAW_MAX; i++) {
					if (!vdev->ctx.isp_pipe_enable[i])
						continue;
					if (i == err_raw_num)
						continue;
					for (j = ISP_FE_CH0; j < ISP_FE_CHN_MAX; j++) {
						if (!(atomic_read(&vdev->pre_fe_state[i][j]) == ISP_STATE_IDLE)) {
							fe_idle = false;
							break;
						}
					}
				}

				if (!(atomic_read(&vdev->pre_be_state[ISP_BE_CH0]) == ISP_STATE_IDLE &&
					atomic_read(&vdev->pre_be_state[ISP_BE_CH1]) == ISP_STATE_IDLE))
					be_idle = false;

				if (!(atomic_read(&vdev->postraw_state) == ISP_STATE_IDLE))
					post_idle = false;

				if ((fe_idle == true) && (be_idle == true) && (post_idle == true))
					break;

				vi_pr(VI_WARN, "wait fe/be/post idle count(%d) for be_post_online\n", count);
			} else {
				if (atomic_read(&vdev->postraw_state) == ISP_STATE_IDLE &&
				    atomic_read(&vdev->pre_be_state[ISP_BE_CH0]) == ISP_STATE_IDLE &&
				    atomic_read(&vdev->pre_be_state[ISP_BE_CH1]) == ISP_STATE_IDLE)
					break;

				vi_pr(VI_WARN, "wait be/post idle count(%d) for be_post_online\n", count);
			}
		} else if (_is_fe_be_online(ctx) && !ctx->is_slice_buf_on) {
			if (atomic_read(&vdev->postraw_state) == ISP_STATE_IDLE)
				break;

			vi_pr(VI_WARN, "wait post idle(%d) count(%d) for fe_be online single\n",
					atomic_read(&vdev->postraw_state), count);
		} else {
			break;
		}

		usleep_range(5 * 1000, 10 * 1000);
	}

	//If fe/be/post not done;
	if (count == 0) {
		vi_pr(VI_ERR, "isp status fe_0(ch0:%d, ch1:%d, ch2:%d, ch3:%d)\n",
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW0][ISP_FE_CH0]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW0][ISP_FE_CH1]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW0][ISP_FE_CH2]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW0][ISP_FE_CH3]));
		vi_pr(VI_ERR, "isp status fe_1(ch0:%d, ch1:%d, ch2:%d, ch3:%d)\n",
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW1][ISP_FE_CH0]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW1][ISP_FE_CH1]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW1][ISP_FE_CH2]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW1][ISP_FE_CH3]));
		vi_pr(VI_ERR, "isp status fe_2(ch0:%d, ch1:%d) fe_3(ch0:%d, ch1:%d)\n",
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW2][ISP_FE_CH0]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW2][ISP_FE_CH1]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW3][ISP_FE_CH0]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW3][ISP_FE_CH1]));
		vi_pr(VI_ERR, "isp status fe_4(ch0:%d, ch1:%d) fe_5(ch0:%d, ch1:%d)\n",
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW4][ISP_FE_CH0]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW4][ISP_FE_CH1]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW5][ISP_FE_CH0]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW5][ISP_FE_CH1]));
		vi_pr(VI_ERR, "isp status fe_lite0(ch0:%d, ch1:%d, ch2:%d, ch3:%d)\n",
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE0][ISP_FE_CH0]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE0][ISP_FE_CH1]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE0][ISP_FE_CH2]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE0][ISP_FE_CH3]));
		vi_pr(VI_ERR, "isp status fe_lite1(ch0:%d, ch1:%d, ch2:%d, ch3:%d)\n",
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE1][ISP_FE_CH0]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE1][ISP_FE_CH1]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE1][ISP_FE_CH2]),
				atomic_read(&vdev->pre_fe_state[ISP_PRERAW_LITE1][ISP_FE_CH3]));
		vi_pr(VI_ERR, "isp status be(ch0:%d, ch1:%d) postraw(%d)\n",
				atomic_read(&vdev->pre_be_state[ISP_BE_CH0]),
				atomic_read(&vdev->pre_be_state[ISP_BE_CH1]),
				atomic_read(&vdev->postraw_state));
		return;
	}

	//step 3 : set csibdg sw abort and wait abort done
	if (isp_frm_err_handler(ctx, err_raw_num, 3) < 0)
		return;

	//step 4 : isp sw reset and vip reset pull up
	isp_frm_err_handler(ctx, err_raw_num, 4);

	//step 5 : isp sw reset and vip reset pull down
	isp_frm_err_handler(ctx, err_raw_num, 5);

	//step 6 : wait ISP idle
	if (isp_frm_err_handler(ctx, err_raw_num, 6) < 0)
		return;

	//step 7 : reset sw state to idle
	if (_is_be_post_online(ctx)) {
		atomic_set(&vdev->pre_fe_state[err_raw_num][ISP_FE_CH0], ISP_STATE_IDLE);
		atomic_set(&vdev->pre_fe_state[err_raw_num][ISP_FE_CH1], ISP_STATE_IDLE);
		atomic_set(&vdev->pre_fe_state[err_raw_num][ISP_FE_CH2], ISP_STATE_IDLE);
		atomic_set(&vdev->pre_fe_state[err_raw_num][ISP_FE_CH3], ISP_STATE_IDLE);
	} else if (_is_fe_be_online(ctx) && !ctx->is_slice_buf_on) {
		atomic_set(&vdev->pre_be_state[ISP_BE_CH0], ISP_STATE_IDLE);
		atomic_set(&vdev->pre_be_state[ISP_BE_CH1], ISP_STATE_IDLE);
	} else if (_is_fe_be_online(ctx) && ctx->is_slice_buf_on) { //slice buffer on
		atomic_set(&vdev->pre_fe_state[err_raw_num][ISP_FE_CH0], ISP_STATE_IDLE);
		atomic_set(&vdev->pre_fe_state[err_raw_num][ISP_FE_CH1], ISP_STATE_IDLE);
		atomic_set(&vdev->pre_fe_state[err_raw_num][ISP_FE_CH2], ISP_STATE_IDLE);
		atomic_set(&vdev->pre_fe_state[err_raw_num][ISP_FE_CH3], ISP_STATE_IDLE);
		atomic_set(&vdev->pre_be_state[ISP_BE_CH0], ISP_STATE_IDLE);
		atomic_set(&vdev->pre_be_state[ISP_BE_CH1], ISP_STATE_IDLE);
	}

	//step 8 : set fbcd dma to hw mode if fbc is on
	if (ctx->is_fbc_on) {
		ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_TNR_ST_Y, false);
		ispblk_dma_set_sw_mode(ctx, ISP_BLK_ID_DMA_CTL_TNR_ST_C, false);
	}

	//step 9 : reset first frame count
	vdev->ctx.isp_pipe_cfg[err_raw_num].first_frm_cnt = 0;

	//step 10 : show overflow info
	_vi_show_debug_info();

	//Let postraw trigger go
	atomic_set(&vdev->isp_err_handle_flag, 0);
}

static int _vi_err_handler_thread(void *arg)
{
	struct cvi_vi_dev *vdev = (struct cvi_vi_dev *)arg;
	enum cvi_isp_raw err_raw_num;
	enum E_VI_TH th_id = E_VI_TH_ERR_HANDLER;

	while (1) {
		wait_event(vdev->vi_th[th_id].wq, vdev->vi_th[th_id].flag != 0 || kthread_should_stop());

		if (vdev->vi_th[th_id].flag != 0) {
			err_raw_num = vdev->vi_th[th_id].flag - 1;
			vdev->vi_th[th_id].flag = 0;
		}

		if (kthread_should_stop()) {
			pr_info("%s exit\n", vdev->vi_th[th_id].th_name);
			atomic_set(&vdev->vi_th[th_id].thread_exit, 1);
			do_exit(1);
		}

		_vi_err_handler(vdev, err_raw_num);
	}

	return 0;
}

static inline void vi_err_wake_up_th(struct cvi_vi_dev *vdev, enum cvi_isp_raw err_raw)
{
	vdev->vi_th[E_VI_TH_ERR_HANDLER].flag = err_raw + 1;

	wake_up(&vdev->vi_th[E_VI_TH_ERR_HANDLER].wq);
}

u32 isp_err_chk(
	struct cvi_vi_dev *vdev,
	struct isp_ctx *ctx,
	union REG_ISP_CSI_BDG_INTERRUPT_STATUS_0 *cbdg_0_sts,
	union REG_ISP_CSI_BDG_INTERRUPT_STATUS_1 *cbdg_1_sts)
{
	u32 ret = 0;
	enum cvi_isp_raw raw_num = ISP_PRERAW0;
	enum cvi_isp_fe_chn_num fe_chn = ISP_FE_CH0;

	if (cbdg_1_sts[raw_num].bits.FIFO_OVERFLOW_INT) {
		vi_pr(VI_ERR, "CSIBDG_A fifo overflow\n");
		_vi_record_debug_info(ctx);
		ctx->isp_pipe_cfg[raw_num].dg_info.bdg_fifo_of_cnt++;
		vi_err_wake_up_th(vdev, raw_num);
		ret = -1;
	}

	if (cbdg_1_sts[raw_num].bits.FRAME_RESOLUTION_OVER_MAX_INT) {
		vi_pr(VI_ERR, "CSIBDG_A frm size over max\n");
		ret = -1;
	}

	if (cbdg_1_sts[raw_num].bits.DMA_ERROR_INT) {
		u32 wdma_0_err = ctx->isp_pipe_cfg[raw_num].dg_info.dma_sts.wdma_0_err_sts;
		u32 wdma_1_err = ctx->isp_pipe_cfg[raw_num].dg_info.dma_sts.wdma_1_err_sts;
		u32 wdma_2_err = ctx->isp_pipe_cfg[raw_num].dg_info.dma_sts.wdma_2_err_sts;
		u32 wdma_3_err = ctx->isp_pipe_cfg[raw_num].dg_info.dma_sts.wdma_3_err_sts;
		u32 rdma_0_err = ctx->isp_pipe_cfg[raw_num].dg_info.dma_sts.rdma_0_err_sts;
		u32 rdma_1_err = ctx->isp_pipe_cfg[raw_num].dg_info.dma_sts.rdma_1_err_sts;
		u32 wdma_0_idle = ctx->isp_pipe_cfg[raw_num].dg_info.dma_sts.wdma_0_idle;
		u32 wdma_1_idle = ctx->isp_pipe_cfg[raw_num].dg_info.dma_sts.wdma_1_idle;
		u32 wdma_2_idle = ctx->isp_pipe_cfg[raw_num].dg_info.dma_sts.wdma_2_idle;
		u32 wdma_3_idle = ctx->isp_pipe_cfg[raw_num].dg_info.dma_sts.wdma_3_idle;
		u32 rdma_0_idle = ctx->isp_pipe_cfg[raw_num].dg_info.dma_sts.rdma_0_idle;
		u32 rdma_1_idle = ctx->isp_pipe_cfg[raw_num].dg_info.dma_sts.rdma_1_idle;

		if ((wdma_0_err & 0x10) || (wdma_1_err & 0x10) ||
		    (wdma_2_err & 0x10) || (wdma_3_err & 0x10) ||
		    (rdma_0_err & 0x10) || (rdma_1_err & 0x10)) {
			vi_pr(VI_ERR, "DMA axi error\n");
			vi_pr(VI_ERR, "Err status wdma[0(0x%x) 1(0x%x) 2(0x%x) 3(0x%x)] rdma[0(0x%x) 1(0x%x)]\n",
					wdma_0_err, wdma_1_err, wdma_2_err, wdma_3_err, rdma_0_err, rdma_1_err);
			ret = -1;
		} else if ((wdma_0_err & 0x20) || (wdma_1_err & 0x20) ||
			   (wdma_2_err & 0x20) || (wdma_3_err & 0x20) ||
			   (rdma_0_err & 0x20) || (rdma_1_err & 0x20)) {
			vi_pr(VI_ERR, "DMA axi mismatch\n");
			vi_pr(VI_ERR, "Err status wdma[0(0x%x) 1(0x%x) 2(0x%x) 3(0x%x)] rdma[0(0x%x) 1(0x%x)]\n",
					wdma_0_err, wdma_1_err, wdma_2_err, wdma_3_err, rdma_0_err, rdma_1_err);
			vi_pr(VI_ERR, "Idle status wdma[0(0x%x) 1(0x%x) 2(0x%x) 3(0x%x)] rdma[0(0x%x) 1(0x%x)]\n",
					wdma_0_idle, wdma_1_idle, wdma_2_idle, wdma_3_idle, rdma_0_idle, rdma_1_idle);
			ret = -1;
		} else if ((wdma_0_err & 0x40) || (wdma_1_err & 0x40) ||
			   (wdma_2_err & 0x40) || (wdma_3_err & 0x40)) {
			vi_pr(VI_WARN, "WDMA buffer full\n");
			vi_pr(VI_ERR, "Err status wdma[0(0x%x) 1(0x%x) 2(0x%x) 3(0x%x)]\n",
					wdma_0_err, wdma_1_err, wdma_2_err, wdma_3_err);
		}
	}

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!vdev->ctx.isp_pipe_enable[raw_num])
			continue;

		fe_chn = ISP_FE_CH0;

		if (cbdg_0_sts[raw_num].bits.CH0_FRAME_WIDTH_GT_INT) {
			vi_pr(VI_ERR, "CSIBDG_%d CH%d frm width greater than setting(%d)\n",
					raw_num, fe_chn, ctx->isp_pipe_cfg[raw_num].csibdg_width);
			ctx->isp_pipe_cfg[raw_num].dg_info.bdg_w_gt_cnt[fe_chn]++;
			vi_err_wake_up_th(vdev, raw_num);
			ret = -1;
		}

		if (cbdg_0_sts[raw_num].bits.CH0_FRAME_WIDTH_LS_INT) {
			vi_pr(VI_ERR, "CSIBDG_%d CH%d frm width less than setting(%d)\n",
					raw_num, fe_chn, ctx->isp_pipe_cfg[raw_num].csibdg_width);
			ctx->isp_pipe_cfg[raw_num].dg_info.bdg_w_ls_cnt[fe_chn]++;
			vi_err_wake_up_th(vdev, raw_num);
			ret = -1;
		}

		if (cbdg_0_sts[raw_num].bits.CH0_FRAME_HEIGHT_GT_INT) {
			vi_pr(VI_ERR, "CSIBDG_%d CH%d frm height greater than setting(%d)\n",
					raw_num, fe_chn, ctx->isp_pipe_cfg[raw_num].csibdg_height);
			ctx->isp_pipe_cfg[raw_num].dg_info.bdg_h_gt_cnt[fe_chn]++;
			vi_err_wake_up_th(vdev, raw_num);
			ret = -1;
		}

		if (cbdg_0_sts[raw_num].bits.CH0_FRAME_HEIGHT_LS_INT) {
			vi_pr(VI_ERR, "CSIBDG_%d CH%d frm height less than setting(%d)\n",
					raw_num, fe_chn, ctx->isp_pipe_cfg[raw_num].csibdg_height);
			ctx->isp_pipe_cfg[raw_num].dg_info.bdg_h_ls_cnt[fe_chn]++;
			vi_err_wake_up_th(vdev, raw_num);
			ret = -1;
		}

		if (ctx->isp_pipe_cfg[raw_num].is_hdr_on ||
		    ctx->isp_pipe_cfg[raw_num].muxMode > VI_WORK_MODE_1Multiplex) {
			fe_chn = ISP_FE_CH1;

			if (cbdg_0_sts[raw_num].bits.CH1_FRAME_WIDTH_GT_INT) {
				vi_pr(VI_ERR, "CSIBDG_%d CH%d frm width greater than setting(%d)\n",
						raw_num, fe_chn, ctx->isp_pipe_cfg[raw_num].csibdg_width);
				ctx->isp_pipe_cfg[raw_num].dg_info.bdg_w_gt_cnt[fe_chn]++;
				vi_err_wake_up_th(vdev, raw_num);
				ret = -1;
			}

			if (cbdg_0_sts[raw_num].bits.CH1_FRAME_WIDTH_LS_INT) {
				vi_pr(VI_ERR, "CSIBDG_%d CH%d frm width less than setting(%d)\n",
						raw_num, fe_chn, ctx->isp_pipe_cfg[raw_num].csibdg_width);
				ctx->isp_pipe_cfg[raw_num].dg_info.bdg_w_ls_cnt[fe_chn]++;
				vi_err_wake_up_th(vdev, raw_num);
				ret = -1;
			}

			if (cbdg_0_sts[raw_num].bits.CH1_FRAME_HEIGHT_GT_INT) {
				vi_pr(VI_ERR, "CSIBDG_%d CH%d frm height greater than setting(%d)\n",
						raw_num, fe_chn, ctx->isp_pipe_cfg[raw_num].csibdg_height);
				ctx->isp_pipe_cfg[raw_num].dg_info.bdg_h_gt_cnt[fe_chn]++;
				vi_err_wake_up_th(vdev, raw_num);
				ret = -1;
			}

			if (cbdg_0_sts[raw_num].bits.CH1_FRAME_HEIGHT_LS_INT) {
				vi_pr(VI_ERR, "CSIBDG_%d CH%d frm height less than setting(%d)\n",
						raw_num, fe_chn, ctx->isp_pipe_cfg[raw_num].csibdg_height);
				ctx->isp_pipe_cfg[raw_num].dg_info.bdg_h_ls_cnt[fe_chn]++;
				vi_err_wake_up_th(vdev, raw_num);
				ret = -1;
			}
		}

		if (ctx->isp_pipe_cfg[raw_num].muxMode > VI_WORK_MODE_2Multiplex) {
			fe_chn = ISP_FE_CH2;

			if (cbdg_0_sts[raw_num].bits.CH2_FRAME_WIDTH_GT_INT) {
				vi_pr(VI_ERR, "CSIBDG_%d CH%d frm width greater than setting(%d)\n",
						raw_num, fe_chn, ctx->isp_pipe_cfg[raw_num].csibdg_width);
				ctx->isp_pipe_cfg[raw_num].dg_info.bdg_w_gt_cnt[fe_chn]++;
				vi_err_wake_up_th(vdev, raw_num);
				ret = -1;
			}

			if (cbdg_0_sts[raw_num].bits.CH2_FRAME_WIDTH_LS_INT) {
				vi_pr(VI_ERR, "CSIBDG_%d CH%d frm width less than setting(%d)\n",
						raw_num, fe_chn, ctx->isp_pipe_cfg[raw_num].csibdg_width);
				ctx->isp_pipe_cfg[raw_num].dg_info.bdg_w_ls_cnt[fe_chn]++;
				vi_err_wake_up_th(vdev, raw_num);
				ret = -1;
			}

			if (cbdg_0_sts[raw_num].bits.CH2_FRAME_HEIGHT_GT_INT) {
				vi_pr(VI_ERR, "CSIBDG_%d CH%d frm height greater than setting(%d)\n",
						raw_num, fe_chn, ctx->isp_pipe_cfg[raw_num].csibdg_height);
				ctx->isp_pipe_cfg[raw_num].dg_info.bdg_h_gt_cnt[fe_chn]++;
				vi_err_wake_up_th(vdev, raw_num);
				ret = -1;
			}

			if (cbdg_0_sts[raw_num].bits.CH2_FRAME_HEIGHT_LS_INT) {
				vi_pr(VI_ERR, "CSIBDG_%d CH%d frm height less than setting(%d)\n",
						raw_num, fe_chn, ctx->isp_pipe_cfg[raw_num].csibdg_height);
				ctx->isp_pipe_cfg[raw_num].dg_info.bdg_h_ls_cnt[fe_chn]++;
				vi_err_wake_up_th(vdev, raw_num);
				ret = -1;
			}
		}

		if (ctx->isp_pipe_cfg[raw_num].muxMode > VI_WORK_MODE_3Multiplex) {
			fe_chn = ISP_FE_CH3;

			if (cbdg_0_sts[raw_num].bits.CH3_FRAME_WIDTH_GT_INT) {
				vi_pr(VI_ERR, "CSIBDG_%d CH%d frm width greater than setting(%d)\n",
						raw_num, fe_chn, ctx->isp_pipe_cfg[raw_num].csibdg_width);
				ctx->isp_pipe_cfg[raw_num].dg_info.bdg_w_gt_cnt[fe_chn]++;
				vi_err_wake_up_th(vdev, raw_num);
				ret = -1;
			}

			if (cbdg_0_sts[raw_num].bits.CH3_FRAME_WIDTH_LS_INT) {
				vi_pr(VI_ERR, "CSIBDG_%d CH%d frm width less than setting(%d)\n",
						raw_num, fe_chn, ctx->isp_pipe_cfg[raw_num].csibdg_width);
				ctx->isp_pipe_cfg[raw_num].dg_info.bdg_w_ls_cnt[fe_chn]++;
				vi_err_wake_up_th(vdev, raw_num);
				ret = -1;
			}

			if (cbdg_0_sts[raw_num].bits.CH3_FRAME_HEIGHT_GT_INT) {
				vi_pr(VI_ERR, "CSIBDG_%d CH%d frm height greater than setting(%d)\n",
						raw_num, fe_chn, ctx->isp_pipe_cfg[raw_num].csibdg_height);
				ctx->isp_pipe_cfg[raw_num].dg_info.bdg_h_gt_cnt[fe_chn]++;
				vi_err_wake_up_th(vdev, raw_num);
				ret = -1;
			}

			if (cbdg_0_sts[raw_num].bits.CH3_FRAME_HEIGHT_LS_INT) {
				vi_pr(VI_ERR, "CSIBDG_%d CH%d frm height less than setting(%d)\n",
						raw_num, fe_chn, ctx->isp_pipe_cfg[raw_num].csibdg_height);
				ctx->isp_pipe_cfg[raw_num].dg_info.bdg_h_ls_cnt[fe_chn]++;
				vi_err_wake_up_th(vdev, raw_num);
				ret = -1;
			}
		}
	}

	return ret;
}

void isp_post_tasklet(unsigned long data)
{
	struct cvi_vi_dev *vdev = (struct cvi_vi_dev *)data;

	_post_hw_enque(vdev);
}

static int _vi_preraw_thread(void *arg)
{
	struct cvi_vi_dev *vdev = (struct cvi_vi_dev *)arg;
	enum cvi_isp_raw raw_num = ISP_PRERAW0;
	struct isp_ctx *ctx = &vdev->ctx;

	struct list_head *pos, *temp;
	struct _isp_raw_num_n  *n[VI_MAX_LIST_NUM];
	unsigned long flags;
	u32 enq_num = 0, i = 0;
	u8 dev_num = 0;
	enum E_VI_TH th_id = E_VI_TH_PRERAW;

	while (1) {
		wait_event(vdev->vi_th[th_id].wq, vdev->vi_th[th_id].flag != 0 || kthread_should_stop());
		vdev->vi_th[th_id].flag = 0;

		if (kthread_should_stop()) {
			pr_info("%s exit\n", vdev->vi_th[th_id].th_name);
			atomic_set(&vdev->vi_th[th_id].thread_exit, 1);
			do_exit(1);
		}

		spin_lock_irqsave(&raw_num_lock, flags);
		list_for_each_safe(pos, temp, &pre_raw_num_q.list) {
			n[enq_num] = list_entry(pos, struct _isp_raw_num_n, list);
			enq_num++;
		}
		spin_unlock_irqrestore(&raw_num_lock, flags);

		for (i = 0; i < enq_num; i++) {
			raw_num = n[i]->raw_num;
			spin_lock_irqsave(&raw_num_lock, flags);
			list_del_init(&n[i]->list);
			kfree(n[i]);
			spin_unlock_irqrestore(&raw_num_lock, flags);

			if (ctx->isp_pipe_cfg[raw_num].is_raw_replay_be) {
				pre_be_tuning_update(&vdev->ctx, raw_num);
				//Update pre be sts size/addr
				_swap_pre_be_sts_buf(vdev, raw_num, ISP_BE_CH0);

				postraw_tuning_update(&vdev->ctx, raw_num);
				//Update postraw sts awb/dci/hist_edge_v dma size/addr
				_swap_post_sts_buf(ctx, raw_num);
			} else {
				if (!ctx->isp_pipe_cfg[raw_num].is_raw_replay_fe)
					_isp_snr_cfg_deq_and_fire(vdev, raw_num, 0);

				pre_fe_tuning_update(&vdev->ctx, raw_num);

				//fe->be->dram->post or on the fly
				if (_is_fe_be_online(ctx) || _is_all_online(ctx)) {
					pre_be_tuning_update(&vdev->ctx, raw_num);

					//on the fly or slice buffer mode on
					if (_is_all_online(ctx) || ctx->is_slice_buf_on) {
						postraw_tuning_update(&vdev->ctx, raw_num);
					}
				}
			}

			if ((ctx->is_multi_sensor) && (!ctx->isp_pipe_cfg[raw_num].is_yuv_sensor)) {
				dev_num = vi_get_dev_num_by_raw(ctx, raw_num);
				if ((tuning_dis[0] > 0) && ((tuning_dis[0] - 1) != dev_num)) {
					vi_pr(VI_DBG, "dev_%d start drop\n", dev_num);
					ctx->isp_pipe_cfg[raw_num].is_drop_next_frame = true;
				}
			}

			if (ctx->isp_pipe_cfg[raw_num].is_drop_next_frame) {
				//if !is_drop_next_frame, set is_drop_next_frame flags false;
				if (_is_drop_next_frame(vdev, raw_num, ISP_FE_CH0))
					++vdev->drop_frame_number[raw_num];
				else {
					vi_pr(VI_DBG, "raw_%d stop drop\n", raw_num);
					ctx->isp_pipe_cfg[raw_num].isp_reset_frm =
						vdev->pre_fe_frm_num[raw_num][ISP_FE_CH0] + 1;
					_clear_drop_frm_info(vdev, raw_num);
				}

				//vi onthefly and vpss online will trigger preraw in post_hw_enque
				if (_is_all_online(ctx) && !ctx->isp_pipe_cfg[raw_num].is_offline_scaler)
					continue;

				if (!ctx->isp_pipe_cfg[raw_num].is_raw_replay_fe &&
					!ctx->isp_pipe_cfg[raw_num].is_raw_replay_be) {
					_pre_hw_enque(vdev, raw_num, ISP_FE_CH0);
					if (ctx->isp_pipe_cfg[raw_num].is_hdr_on)
						_pre_hw_enque(vdev, raw_num, ISP_FE_CH1);
				}
			}
		}

		enq_num = 0;
	}

	return 0;
}

static int _vi_vblank_handler_thread(void *arg)
{
	struct cvi_vi_dev *vdev = (struct cvi_vi_dev *)arg;
	enum cvi_isp_raw raw_num = ISP_PRERAW0;

	enum E_VI_TH th_id = E_VI_TH_VBLANK_HANDLER;

	while (1) {
		wait_event(vdev->vi_th[th_id].wq, vdev->vi_th[th_id].flag != 0 || kthread_should_stop());
		raw_num = vdev->vi_th[th_id].flag - 1;
		vdev->vi_th[th_id].flag = 0;

		if (kthread_should_stop()) {
			pr_info("%s exit\n", vdev->vi_th[th_id].th_name);
			atomic_set(&vdev->vi_th[th_id].thread_exit, 1);
			do_exit(1);
		}

		_isp_snr_cfg_deq_and_fire(vdev, raw_num, 1);
	}

	return 0;
}

static void _isp_yuv_online_handler(struct cvi_vi_dev *vdev, const enum cvi_isp_raw raw_num, const u8 hw_chn_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	struct isp_buffer *b = NULL;

	atomic_set(&vdev->pre_fe_state[raw_num][hw_chn_num], ISP_STATE_IDLE);

	b = isp_buf_remove(&pre_fe_out_q[raw_num][hw_chn_num]);
	if (b == NULL) {
		vi_pr(VI_INFO, "fe_%d chn_num_%d done outbuf is empty\n", raw_num, hw_chn_num);
		return;
	}

	b->crop_le.x = 0;
	b->crop_le.y = 0;
	b->crop_le.w = ctx->isp_pipe_cfg[raw_num].post_img_w;
	b->crop_le.h = ctx->isp_pipe_cfg[raw_num].post_img_h;
	b->raw_num = raw_num;
	b->chn_num = hw_chn_num;

	if (_is_be_post_online(ctx))
		isp_buf_queue(&pre_be_in_q, b);
	else if (_is_fe_be_online(ctx))
		isp_buf_queue(&postraw_in_q[ISP_RAW_PATH_LE], b);

	// if preraw offline, let usr_pic_timer_handler do it.
	if (!ctx->isp_pipe_cfg[raw_num].is_raw_replay_fe &&
		!ctx->isp_pipe_cfg[raw_num].is_raw_replay_be)
		_pre_hw_enque(vdev, raw_num, hw_chn_num);
}

static void _isp_yuv_bypass_handler(struct cvi_vi_dev *vdev, const enum cvi_isp_raw raw_num, const u8 hw_chn_num)
{
	u8 buf_chn = vdev->ctx.raw_chnstr_num[raw_num] + hw_chn_num;

	atomic_set(&vdev->pre_fe_state[raw_num][hw_chn_num], ISP_STATE_IDLE);

	cvi_isp_dqbuf_list(vdev, vdev->pre_fe_frm_num[raw_num][hw_chn_num], buf_chn);

	++vdev->postraw_frame_number[raw_num];

	vdev->vi_th[E_VI_TH_EVENT_HANDLER].flag = raw_num + 1;
	wake_up(&vdev->vi_th[E_VI_TH_EVENT_HANDLER].wq);

	if (cvi_isp_rdy_buf_empty(vdev, buf_chn))
		vi_pr(VI_INFO, "fe_%d chn_num_%d yuv bypass outbuf is empty\n", raw_num, buf_chn);
	else
		_isp_yuv_bypass_trigger(vdev, raw_num, hw_chn_num);
}

static inline void _vi_wake_up_vblank_th(struct cvi_vi_dev *vdev, const enum cvi_isp_raw raw_num)
{
	vdev->vi_th[E_VI_TH_VBLANK_HANDLER].flag = raw_num + 1;
	wake_up(&vdev->vi_th[E_VI_TH_VBLANK_HANDLER].wq);
}

static void _isp_sof_handler(struct cvi_vi_dev *vdev, const enum cvi_isp_raw raw_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	// struct _isp_dqbuf_n *n = NULL;
	// unsigned long flags;

	if (atomic_read(&vdev->isp_streamoff) == 1)
		return;

	if (!(_is_fe_be_online(ctx) && ctx->is_slice_buf_on) || ctx->isp_pipe_cfg[raw_num].is_drop_next_frame)
		_vi_wake_up_preraw_th(vdev, raw_num);

	if (atomic_read(&vdev->isp_raw_dump_en[raw_num]) == 2) //raw_dump flow
		atomic_set(&vdev->isp_raw_dump_en[raw_num], 3);

	tasklet_hi_schedule(&vdev->job_work);

	// if (ctx->isp_pipe_cfg[raw_num].is_offline_scaler) {
	// 	spin_lock_irqsave(&dq_lock, flags);
	// 	if (!list_empty(&dqbuf_q[raw_num].list)) {
	// 		n = list_first_entry(&dqbuf_q[raw_num].list, struct _isp_dqbuf_n, list);
	// 		vdev->vi_th[E_VI_TH_EVENT_HANDLER].flag = n->chn_id + 1;
	// 		wake_up(&vdev->vi_th[E_VI_TH_EVENT_HANDLER].wq);
	// 	}
	// 	spin_unlock_irqrestore(&dq_lock, flags);
	// }

	isp_sync_task_process(raw_num);
}

static inline void _isp_splt_wdma_done_handler(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw hw_raw_num,
	const enum cvi_isp_fe_chn_num chn_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	struct isp_queue *splt_w_q = NULL;
	struct isp_queue *splt_r_q = NULL;
	struct isp_buffer *b = NULL;
	enum cvi_isp_raw raw_num = hw_raw_num;
	bool trigger = false;

	vi_pr(VI_DBG, "splt_wdma_%d frm_done chn_num=%d frm_num=%d\n",
			raw_num, chn_num, vdev->splt_wdma_frm_num[raw_num][chn_num]);

	//raw ai isp move to FE, abandon splt flow
	return;

	atomic_set(&vdev->splt_state[raw_num][chn_num], ISP_STATE_IDLE);

	if (ctx->isp_pipe_cfg[raw_num].is_tile) {
		raw_num = ISP_PRERAW0;
		trigger = (vdev->splt_wdma_frm_num[raw_num][chn_num] ==
				vdev->splt_wdma_frm_num[raw_num + 1][chn_num]);
	} else {
		trigger = true;
	}

	if (trigger) {
		splt_w_q = &splt_out_q[raw_num][chn_num];
		splt_r_q = &pre_fe_in_q[raw_num][chn_num];

		if (!ctx->isp_pipe_cfg[raw_num].is_fake_splt_wdma) {
			b = isp_buf_remove(splt_w_q);
			if (b == NULL) {
				vi_pr(VI_ERR, "splt_wdma_%d chn_num_%d outbuf is empty\n",
						raw_num, chn_num);
				return;
			}

			// TODO transfer frame to TPU
			// This is for test.
			isp_buf_queue(splt_r_q, b);
		}

		if (ctx->isp_pipe_cfg[raw_num].is_hdr_on) {
			trigger = vdev->splt_wdma_frm_num[raw_num][ISP_FE_CH0] ==
					vdev->splt_wdma_frm_num[raw_num][ISP_FE_CH1];
		}

		if (trigger) {
			_pre_hw_enque(vdev, raw_num, ISP_FE_CH0);
			if (ctx->isp_pipe_cfg[raw_num].is_hdr_on)
				_pre_hw_enque(vdev, raw_num, ISP_FE_CH1);

			if (ctx->isp_pipe_cfg[raw_num].is_tile) {
				_pre_hw_enque(vdev, raw_num + 1, ISP_FE_CH0);
				if (ctx->isp_pipe_cfg[raw_num].is_hdr_on)
					_pre_hw_enque(vdev, raw_num + 1, ISP_FE_CH1);
			}
		}

		_splt_hw_enque(vdev, raw_num);
	}
}

static inline void _isp_splt_rdma_done_handler(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw hw_raw_num,
	const enum cvi_isp_fe_chn_num chn_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	struct isp_queue *splt_w_q = NULL;
	struct isp_queue *splt_r_q = NULL;
	struct isp_buffer *b = NULL;
	enum ISP_BLK_ID_T splt_rdma_id = 0;
	enum cvi_isp_raw raw_num = hw_raw_num;
	bool trigger = false;

	vi_pr(VI_DBG, "splt_rdma_%d frm_done chn_num=%d frm_num=%d\n",
			raw_num, chn_num, vdev->splt_rdma_frm_num[raw_num][chn_num]);

	//raw ai isp move to FE, abandon splt flow
	return;

	if (ctx->isp_pipe_cfg[raw_num].is_tile) {
		raw_num = ISP_PRERAW0;
		trigger = (vdev->splt_rdma_frm_num[raw_num][chn_num] ==
				vdev->splt_rdma_frm_num[raw_num + 1][chn_num]);
	} else {
		trigger = true;
	}

	if (trigger) {
		splt_w_q = &splt_out_q[raw_num][chn_num];
		splt_r_q = &pre_fe_in_q[raw_num][chn_num];

		b = isp_buf_remove(splt_r_q);
		if (b == NULL) {
			vi_pr(VI_ERR, "splt_rdma_%d chn_num_%d outbuf is empty\n",
					raw_num, chn_num);
			return;
		}

		isp_buf_queue(splt_w_q, b);

		splt_rdma_id = (chn_num == ISP_FE_CH0) ?
				ISP_BLK_ID_SPLT_FE0_RDMA_LE : ISP_BLK_ID_SPLT_FE0_RDMA_SE;
		ispblk_splt_rdma_ctrl_config(ctx, splt_rdma_id, false);
		if (ctx->isp_pipe_cfg[raw_num].is_tile) {
			splt_rdma_id = (chn_num == ISP_FE_CH0) ?
					ISP_BLK_ID_SPLT_FE1_RDMA_LE : ISP_BLK_ID_SPLT_FE1_RDMA_SE;
			ispblk_splt_rdma_ctrl_config(ctx, splt_rdma_id, false);
		}
	}
}

static inline void _isp_pre_fe_done_handler(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw hw_raw_num,
	const enum cvi_isp_fe_chn_num chn_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	enum cvi_isp_raw raw_num = hw_raw_num;
	bool trigger = false;

	if (ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) {
		if (ctx->isp_pipe_cfg[raw_num].yuv_scene_mode == ISP_YUV_SCENE_BYPASS) {
			vi_pr(VI_DBG, "pre_fe_%d yuv bypass done chn_num=%d frm_num=%d\n",
					raw_num, chn_num, vdev->pre_fe_frm_num[raw_num][chn_num]);
			_isp_yuv_bypass_handler(vdev, raw_num, chn_num);
		} else if (ctx->isp_pipe_cfg[raw_num].yuv_scene_mode == ISP_YUV_SCENE_ONLINE) {
			vi_pr(VI_DBG, "pre_fe_%d yuv online done chn_num=%d frm_num=%d\n",
					raw_num, chn_num, vdev->pre_fe_frm_num[raw_num][chn_num]);
			_isp_yuv_online_handler(vdev, raw_num, chn_num);
		} else if (ctx->isp_pipe_cfg[raw_num].yuv_scene_mode == ISP_YUV_SCENE_ISP) {
			vi_pr(VI_DBG, "pre_fe_%d yuv isp done chn_num=%d frm_num=%d\n",
					raw_num, chn_num, vdev->pre_fe_frm_num[raw_num][chn_num]);
			_isp_yuv_online_handler(vdev, raw_num, chn_num);
		} else {
			vi_pr(VI_ERR, "pre_fe_%d isp_domain error, yuv_scene_mode=%d\n",
					raw_num, ctx->isp_pipe_cfg[raw_num].yuv_scene_mode);
		}
		return;
	}

	vi_pr(VI_DBG, "pre_fe_%d frm_done chn_num=%d frm_num=%d\n",
			raw_num, chn_num, vdev->pre_fe_frm_num[raw_num][chn_num]);

	if (ctx->isp_pipe_cfg[raw_num].is_tile) {
		//it's means fe0/fe1 chn frame done
		trigger = (vdev->pre_fe_frm_num[ISP_PRERAW0][chn_num] ==
					vdev->pre_fe_frm_num[ISP_PRERAW1][chn_num]);

		if (!trigger) {
			atomic_set(&vdev->pre_fe_state[hw_raw_num][chn_num], ISP_STATE_IDLE);
			vi_pr(VI_DBG, "tile mode wait fe0/fe1 frame done\n");
			return;
		}

		//for tile mode, it's only can opreate fe0
		raw_num = ISP_PRERAW0;
	}

	// No changed in onthefly mode or slice buffer on
	if (!_is_all_online(ctx) && !(_is_fe_be_online(ctx) && ctx->is_rgbmap_sbm_on)) {
		ispblk_tnr_rgbmap_chg(ctx, raw_num, chn_num);
		_pre_fe_rgbmap_update(vdev, raw_num, chn_num);
	}

	if (_is_fe_be_online(ctx) || _is_all_online(ctx)) { //fe->be->dram->post or on the fly mode
		if (atomic_read(&vdev->isp_raw_dump_en[raw_num]) == 3) { //raw_dump flow
			struct isp_buffer *b = NULL;
			struct isp_queue *fe_out_q = (chn_num == ISP_FE_CH0) ?
							&raw_dump_b_q[raw_num] : &raw_dump_b_se_q[raw_num];
			struct isp_queue *raw_d_q = (chn_num == ISP_FE_CH0) ?
							&raw_dump_b_dq[raw_num] : &raw_dump_b_se_dq[raw_num];
			u32 x, y, w, h, dmaid;

			if (ctx->isp_pipe_cfg[raw_num].rawdump_crop.w &&
				ctx->isp_pipe_cfg[raw_num].rawdump_crop.h) {
				x = (chn_num == ISP_FE_CH0) ?
					ctx->isp_pipe_cfg[raw_num].rawdump_crop.x :
					ctx->isp_pipe_cfg[raw_num].rawdump_crop_se.x;
				y = (chn_num == ISP_FE_CH0) ?
					ctx->isp_pipe_cfg[raw_num].rawdump_crop.y :
					ctx->isp_pipe_cfg[raw_num].rawdump_crop_se.y;
				w = (chn_num == ISP_FE_CH0) ?
					ctx->isp_pipe_cfg[raw_num].rawdump_crop.w :
					ctx->isp_pipe_cfg[raw_num].rawdump_crop_se.w;
				h = (chn_num == ISP_FE_CH0) ?
					ctx->isp_pipe_cfg[raw_num].rawdump_crop.h :
					ctx->isp_pipe_cfg[raw_num].rawdump_crop_se.h;
			} else {
				x = 0;
				y = 0;
				w = (chn_num == ISP_FE_CH0) ?
					ctx->isp_pipe_cfg[raw_num].crop.w :
					ctx->isp_pipe_cfg[raw_num].crop_se.w;
				h = (chn_num == ISP_FE_CH0) ?
					ctx->isp_pipe_cfg[raw_num].crop.h :
					ctx->isp_pipe_cfg[raw_num].crop_se.h;
			}

			dmaid = csibdg_dma_find_hwid(raw_num, chn_num);

			if (chn_num == ISP_FE_CH0)
				++vdev->dump_frame_number[raw_num];

			ispblk_csidbg_dma_wr_en(ctx, raw_num, chn_num, 0);

			b = isp_buf_remove(fe_out_q);
			if (b == NULL) {
				vi_pr(VI_ERR, "Pre_fe_%d chn_num_%d outbuf is empty\n", raw_num, chn_num);
				return;
			}

			b->crop_le.x = b->crop_se.x = x;
			b->crop_le.y = b->crop_se.y = y;
			b->crop_le.w = b->crop_se.w = w;
			b->crop_le.h = b->crop_se.h = h;
			b->byr_size = ispblk_dma_get_size(ctx, dmaid, w, h);
			b->frm_num = vdev->pre_fe_frm_num[raw_num][chn_num];

			isp_buf_queue(raw_d_q, b);

			if (ctx->isp_pipe_cfg[raw_num].is_hdr_on) {
				trigger = (vdev->pre_fe_frm_num[raw_num][ISP_FE_CH0] ==
						vdev->pre_fe_frm_num[raw_num][ISP_FE_CH1]);
			} else
				trigger = true;

			if (trigger) {
				_isp_raw_dump_chk(vdev, raw_num, b->frm_num);
			}
		}

		atomic_set(&vdev->pre_fe_state[raw_num][chn_num], ISP_STATE_IDLE);

		if (_is_all_online(ctx)) {
			struct isp_grid_s_info m_info;

			m_info = ispblk_rgbmap_info(ctx, raw_num);
			ctx->isp_pipe_cfg[raw_num].rgbmap_i.w_bit = m_info.w_bit;
			ctx->isp_pipe_cfg[raw_num].rgbmap_i.h_bit = m_info.h_bit;

			m_info = ispblk_lmap_info(ctx, raw_num);
			ctx->isp_pipe_cfg[raw_num].lmap_i.w_bit = m_info.w_bit;
			ctx->isp_pipe_cfg[raw_num].lmap_i.h_bit = m_info.h_bit;
		}
	} else if (_is_be_post_online(ctx)) { //fe->dram->be->post
		struct isp_buffer *b = NULL;
		struct isp_grid_s_info m_info;
		struct isp_queue *fe_out_q, *be_in_q, *raw_d_q;

		if (_postraw_outbuf_empty(vdev, raw_num) ||
			!atomic_read(&vdev->is_streaming[raw_num])) {
			vi_pr(VI_DBG, "postraw%d_outbuf is empty, block it on FE\n", raw_num);
			atomic_set(&vdev->pre_fe_state[hw_raw_num][chn_num], ISP_STATE_IDLE);
			_pre_hw_enque(vdev, raw_num, chn_num);
			return;
		}

		fe_out_q = &pre_fe_out_q[raw_num][chn_num];
		if (ctx->isp_pipe_cfg[raw_num].is_raw_ai_isp) {
			be_in_q = &bnr_ai_isp_q[raw_num][chn_num];
		} else {
			be_in_q = (chn_num == ISP_FE_CH0) ? &pre_be_in_q : &pre_be_in_se_q[raw_num];
		}
		raw_d_q = (chn_num == ISP_FE_CH0) ?
				&raw_dump_b_dq[raw_num] : &raw_dump_b_se_dq[raw_num];

		if (atomic_read(&vdev->isp_raw_dump_en[raw_num]) == 3) //raw dump enable
			fe_out_q = (chn_num == ISP_FE_CH0) ? &raw_dump_b_q[raw_num] : &raw_dump_b_se_q[raw_num];

		b = isp_buf_remove(fe_out_q);
		if (b == NULL) {
			vi_pr(VI_ERR, "Pre_fe_%d chn_num_%d outbuf is empty\n", raw_num, chn_num);
			return;
		}

		if (atomic_read(&vdev->isp_raw_dump_en[raw_num]) == 3) { //raw dump enable
			u32 w = (chn_num == ISP_FE_CH0) ?
				ctx->isp_pipe_cfg[raw_num].crop.w :
				ctx->isp_pipe_cfg[raw_num].crop_se.w;
			u32 h = (chn_num == ISP_FE_CH0) ?
				ctx->isp_pipe_cfg[raw_num].crop.h :
				ctx->isp_pipe_cfg[raw_num].crop_se.h;
			u32 dmaid = csibdg_dma_find_hwid(raw_num, chn_num);

			if (chn_num == ISP_FE_CH0)
				++vdev->dump_frame_number[raw_num];

			b->crop_le.x = b->crop_se.x = 0;
			b->crop_le.y = b->crop_se.y = 0;
			b->crop_le.w = b->crop_se.w = ctx->isp_pipe_cfg[raw_num].crop.w;
			b->crop_le.h = b->crop_se.h = ctx->isp_pipe_cfg[raw_num].crop.h;
			b->byr_size = ispblk_dma_get_size(ctx, dmaid, w, h);
			b->frm_num = vdev->pre_fe_frm_num[raw_num][chn_num];

			isp_buf_queue(raw_d_q, b);
		} else {
			m_info = ispblk_rgbmap_info(ctx, raw_num);
			b->rgbmap_i.w_bit = m_info.w_bit;
			b->rgbmap_i.h_bit = m_info.h_bit;

			m_info = ispblk_lmap_info(ctx, raw_num);
			b->lmap_i.w_bit = m_info.w_bit;
			b->lmap_i.h_bit = m_info.h_bit;

			b->chn_num	= 0;

			isp_buf_queue(be_in_q, b);
		}


		atomic_set(&vdev->pre_fe_state[hw_raw_num][chn_num], ISP_STATE_IDLE);

		if (ctx->isp_pipe_cfg[raw_num].is_hdr_on) {
			trigger = (vdev->pre_fe_frm_num[raw_num][ISP_FE_CH0] ==
					vdev->pre_fe_frm_num[raw_num][ISP_FE_CH1]);
		} else
			trigger = true;

		if (trigger) {
			vi_pr(VI_DBG, "fe->dram->be->post trigger raw_num=%d\n", raw_num);

			if (atomic_read(&vdev->isp_raw_dump_en[raw_num]) == 3) { //raw dump flow
				_isp_raw_dump_chk(vdev, raw_num, b->frm_num);
				if (ctx->isp_pipe_cfg[raw_num].is_raw_ai_isp) {
					_pre_hw_enque(vdev, raw_num, chn_num);
				}
			} else {
				if (ctx->isp_pipe_cfg[raw_num].is_raw_ai_isp) {
					_vi_wake_up_tpu_th(vdev, raw_num, AI_ISP_TYPE_BNR);
				} else {
					tasklet_hi_schedule(&vdev->job_work);
				}
				_vi_wake_up_vblank_th(vdev, raw_num);
			}
		}

		if (!ctx->isp_pipe_cfg[raw_num].is_raw_replay_fe &&
			!ctx->isp_pipe_cfg[raw_num].is_raw_replay_be &&
			!ctx->isp_pipe_cfg[raw_num].is_raw_ai_isp) {
			_pre_hw_enque(vdev, raw_num, chn_num);
			if (ctx->isp_pipe_cfg[raw_num].is_tile)
				_pre_hw_enque(vdev, raw_num + 1, chn_num);

			if (ctx->isp_pipe_cfg[raw_num].is_tile || line_spliter_en)
				_splt_hw_enque(vdev, raw_num);
		}
	}
}

static inline void _isp_pre_be_done_handler(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_be_chn_num chn_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	enum cvi_isp_raw raw_num = vi_get_first_raw_num(ctx);
	bool trigger = false;

	if (_is_fe_be_online(ctx) && !ctx->is_slice_buf_on) { // fe->be->dram->post
		struct isp_buffer *b = NULL;
		struct isp_grid_s_info m_info;
		struct isp_queue *be_out_q = &pre_be_out_q[chn_num];
		struct isp_queue *post_in_q = &postraw_in_q[chn_num];

		++vdev->pre_be_frm_num[raw_num][chn_num];

		vi_pr(VI_DBG, "pre_be_%d frm_done chn_num=%d frm_num=%d\n",
				raw_num, chn_num, vdev->pre_be_frm_num[raw_num][chn_num]);

		b = isp_buf_remove(be_out_q);
		if (b == NULL) {
			vi_pr(VI_ERR, "Pre_be chn_num_%d outbuf is empty\n", chn_num);
			return;
		}

		m_info = ispblk_rgbmap_info(ctx, raw_num);
		b->rgbmap_i.w_bit = m_info.w_bit;
		b->rgbmap_i.h_bit = m_info.h_bit;

		m_info = ispblk_lmap_info(ctx, raw_num);
		b->lmap_i.w_bit = m_info.w_bit;
		b->lmap_i.h_bit = m_info.h_bit;

		isp_buf_queue(post_in_q, b);

		//Pre_be done for tuning to get stt.
		_swap_pre_be_sts_buf(vdev, raw_num, chn_num);

		atomic_set(&vdev->pre_be_state[chn_num], ISP_STATE_IDLE);

		if (!ctx->isp_pipe_cfg[raw_num].is_raw_replay_fe &&
			!ctx->isp_pipe_cfg[raw_num].is_raw_replay_be)
			_pre_hw_enque(vdev, raw_num, chn_num);

		if (ctx->isp_pipe_cfg[raw_num].is_hdr_on) {
			trigger = (vdev->pre_be_frm_num[raw_num][ISP_BE_CH0] ==
					vdev->pre_be_frm_num[raw_num][ISP_BE_CH1]);
		} else
			trigger = true;

		if (trigger) {
			vi_event_queue(vdev, VI_EVENT_PRE0_EOF + vi_get_dev_num_by_raw(ctx, raw_num),
					vdev->pre_be_frm_num[raw_num][ISP_BE_CH0]);

			tasklet_hi_schedule(&vdev->job_work);
			_vi_wake_up_vblank_th(vdev, raw_num);
		}
	} else if (_is_be_post_online(ctx)) { // fe->dram->be->post
		struct isp_buffer *b = NULL;
		struct isp_queue *be_in_q = (chn_num == ISP_BE_CH0) ?
						&pre_be_in_q : &pre_be_in_se_q[ctx->cam_id];
		struct isp_queue *pre_out_q = NULL;

		if (ctx->isp_pipe_cfg[ctx->cam_id].is_tile) {
			if (vdev->postraw_proc_num % 2 != 0) {
				vi_pr(VI_DBG, "pre_be_%d chn_num=%d left tile frm_done\n",
					ctx->cam_id, chn_num);
				atomic_set(&vdev->pre_be_state[chn_num], ISP_STATE_IDLE);
				return;
			}
		}

		if (!ctx->isp_pipe_cfg[raw_num].is_raw_replay_be) {
			b = isp_buf_remove(be_in_q);
			if (b == NULL) {
				vi_pr(VI_ERR, "Pre_be chn_num_%d input buf is empty\n", chn_num);
				return;
			}
			if (b->raw_num >= ISP_PRERAW_MAX) {
				vi_pr(VI_ERR, "buf raw_num_%d is wrong\n", b->raw_num);
				return;
			}
			raw_num = b->raw_num;
		}

		++vdev->pre_be_frm_num[raw_num][chn_num];

		vi_pr(VI_DBG, "pre_be_%d frm_done chn_num=%d frm_num=%d\n",
				raw_num, chn_num, vdev->pre_be_frm_num[raw_num][chn_num]);

		if (!ctx->isp_pipe_cfg[raw_num].is_raw_replay_be) {
			pre_out_q = &pre_fe_out_q[raw_num][chn_num];
			isp_buf_queue(pre_out_q, b);
		}

		atomic_set(&vdev->pre_be_state[chn_num], ISP_STATE_IDLE);

		if (ctx->isp_pipe_cfg[raw_num].is_hdr_on) {
			trigger = (vdev->pre_be_frm_num[raw_num][ISP_BE_CH0] ==
					vdev->pre_be_frm_num[raw_num][ISP_BE_CH1]);
		} else
			trigger = true;

		if (trigger)
			vi_event_queue(vdev, VI_EVENT_PRE0_EOF + vi_get_dev_num_by_raw(ctx, raw_num),
				vdev->pre_be_frm_num[raw_num][ISP_BE_CH0]);
	}
}

static void _isp_postraw_shaw_done_handler(struct cvi_vi_dev *vdev)
{
	if (_is_fe_be_online(&vdev->ctx) && vdev->ctx.is_slice_buf_on) {
		vi_pr(VI_INFO, "postraw shaw done\n");
		_vi_wake_up_preraw_th(vdev, vi_get_first_raw_num(&vdev->ctx));
	}
}

static void _isp_postraw_done_handler(struct cvi_vi_dev *vdev)
{
	struct isp_ctx *ctx = &vdev->ctx;
	enum cvi_isp_raw raw_num = ISP_PRERAW0;
	enum cvi_isp_fe_chn_num chn_num = ISP_FE_CH0;

	if (vdev->postraw_proc_num % 2 == 0) {
		vdev->postraw_proc_num = 0;
		ctx->is_work_on_r_tile = false;
	} else {
		atomic_set(&vdev->postraw_state, ISP_STATE_IDLE);
		if (_is_be_post_online(ctx) && ctx->isp_pipe_cfg[raw_num].is_yuv_sensor)
			atomic_set(&vdev->pre_be_state[ISP_BE_CH0], ISP_STATE_IDLE);

		vi_pr(VI_DBG, "Postraw_%d left tile frm_done frm_num=%d\n",
			raw_num, vdev->postraw_frame_number[raw_num]);
		tasklet_hi_schedule(&vdev->job_work);
		return;
	}

	if (_is_fe_be_online(ctx))
		raw_num = ctx->cam_id;

	++ctx->isp_pipe_cfg[raw_num].first_frm_cnt;

	if (_is_fe_be_online(ctx) && !ctx->is_slice_buf_on) { //fe->be->dram->post
		struct isp_buffer *b;

		b = isp_buf_remove(&postraw_in_q[ISP_RAW_PATH_LE]);
		if (b == NULL) {
			vi_pr(VI_ERR, "post_in_q is empty\n");
			return;
		}
		if (b->raw_num >= ISP_PRERAW_MAX) {
			vi_pr(VI_ERR, "buf raw_num_%d is wrong\n", b->raw_num);
			return;
		}
		raw_num = b->raw_num;
		chn_num = b->chn_num;

		if (ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) {
			isp_buf_queue(&pre_fe_out_q[raw_num][chn_num], b);
		} else {
			isp_buf_queue(&pre_be_out_q[ISP_RAW_PATH_LE], b);

			if (ctx->isp_pipe_cfg[raw_num].is_hdr_on) {
				b = isp_buf_remove(&postraw_in_q[ISP_RAW_PATH_SE]);
				if (b == NULL) {
					vi_pr(VI_ERR, "post_in_se_q is empty\n");
					return;
				}
				isp_buf_queue(&pre_be_out_q[ISP_RAW_PATH_SE], b);
			}
		}
	} else if (_is_be_post_online(ctx)) { // fe->dram->be->post
		raw_num = ctx->cam_id;

		if (ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) {
			struct isp_buffer *b = NULL;
			struct isp_queue *be_in_q = &pre_be_in_q;
			struct isp_queue *pre_out_q = NULL;

			b = isp_buf_remove(be_in_q);
			if (b == NULL) {
				vi_pr(VI_ERR, "pre_be_in_q is empty\n");
				return;
			}
			if (b->chn_num >= ISP_FE_CHN_MAX) {
				vi_pr(VI_ERR, "buf chn_num_%d is wrong\n", b->chn_num);
				return;
			}
			chn_num = b->chn_num;

			pre_out_q = &pre_fe_out_q[raw_num][chn_num];
			isp_buf_queue(pre_out_q, b);
		}
	}

	atomic_set(&vdev->postraw_state, ISP_STATE_IDLE);
	if (_is_be_post_online(ctx) && ctx->isp_pipe_cfg[raw_num].is_yuv_sensor)
		atomic_set(&vdev->pre_be_state[ISP_BE_CH0], ISP_STATE_IDLE);

	++vdev->postraw_frame_number[raw_num];

	vi_pr(VI_DBG, "Postraw_%d frm_done frm_num=%d\n", raw_num, vdev->postraw_frame_number[raw_num]);

	if (!ctx->isp_pipe_cfg[raw_num].is_yuv_sensor) { //ISP team no need yuv post done
		if (isp_bufpool[raw_num].fswdr_rpt)
			ispblk_fswdr_update_rpt(ctx, isp_bufpool[raw_num].fswdr_rpt);

		ctx->mmap_grid_size[raw_num] = ctx->isp_pipe_cfg[raw_num].rgbmap_i.w_bit;

		vi_event_queue(vdev, VI_EVENT_POST0_EOF + vi_get_dev_num_by_raw(ctx, raw_num),
				vdev->postraw_frame_number[raw_num]);
	}

	cvi_isp_dqbuf_list(vdev, vdev->postraw_frame_number[raw_num], raw_num);
	vdev->vi_th[E_VI_TH_EVENT_HANDLER].flag = raw_num + 1;
	wake_up(&vdev->vi_th[E_VI_TH_EVENT_HANDLER].wq);

	if (!ctx->isp_pipe_cfg[raw_num].is_raw_replay_fe &&
		!ctx->isp_pipe_cfg[raw_num].is_raw_replay_be) {
		tasklet_hi_schedule(&vdev->job_work);

		if (!_is_all_online(ctx) && !ctx->is_slice_buf_on) {
			_pre_hw_enque(vdev, raw_num, ISP_FE_CH0);
			if (ctx->isp_pipe_cfg[raw_num].is_hdr_on)
				_pre_hw_enque(vdev, raw_num, ISP_FE_CH1);
		}
	}
}

static void _isp_cmdq_done_chk(struct cvi_vi_dev *vdev, const u32 cmdq_intr)
{
	uintptr_t cmdq = vdev->ctx.phys_regs[ISP_BLK_ID_CMDQ];

	if (cmdq_intr & BIT(0)) {
		u8 cmdq_intr_status = cmdQ_intr_status(cmdq);

		vi_pr(VI_DBG, "cmdq_intr 0x%08x\n", cmdq_intr_status);
		cmdQ_intr_clr(cmdq, cmdq_intr_status);
	}
}

static void _isp_csibdg_lite_frame_start_chk(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	const u32 frame_start)
{
	if (frame_start & BIT(1)) {
		vi_record_sof_perf(vdev, raw_num, ISP_FE_CH0);

		++vdev->pre_fe_sof_cnt[raw_num][ISP_FE_CH0];

		vi_pr(VI_INFO, "csibgd_lite_%d sof chn_num=%d frm_num=%d\n",
			raw_num - ISP_PRERAW_LITE0, ISP_FE_CH0, vdev->pre_fe_sof_cnt[raw_num][ISP_FE_CH0]);
	}

	if (frame_start & BIT(9)) {
		++vdev->pre_fe_sof_cnt[raw_num][ISP_FE_CH1];
	}

	if (frame_start & BIT(17)) {
		++vdev->pre_fe_sof_cnt[raw_num][ISP_FE_CH2];
	}

	if (frame_start & BIT(25)) {
		++vdev->pre_fe_sof_cnt[raw_num][ISP_FE_CH3];
	}
}

static void _isp_csibdg_lite_frame_done_chk(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	const u32 frame_start)
{
	if (frame_start & BIT(3)) {
		vi_record_fe_perf(vdev, raw_num, ISP_FE_CH0);

		++vdev->pre_fe_frm_num[raw_num][ISP_FE_CH0];
		_isp_pre_fe_done_handler(vdev, raw_num, ISP_FE_CH0);
	}

	if (frame_start & BIT(11)) {
		++vdev->pre_fe_frm_num[raw_num][ISP_FE_CH1];
		_isp_pre_fe_done_handler(vdev, raw_num, ISP_FE_CH1);
	}

	if (frame_start & BIT(19)) {
		++vdev->pre_fe_frm_num[raw_num][ISP_FE_CH2];
		_isp_pre_fe_done_handler(vdev, raw_num, ISP_FE_CH2);
	}

	if (frame_start & BIT(27)) {
		++vdev->pre_fe_frm_num[raw_num][ISP_FE_CH3];
		_isp_pre_fe_done_handler(vdev, raw_num, ISP_FE_CH3);
	}
}

static void _isp_splt_frame_done_chk(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	const u8 frame_done)
{
	// wdma done
	if (frame_done & BIT(0)) {
		++vdev->splt_wdma_frm_num[raw_num][ISP_FE_CH0];
		_isp_splt_wdma_done_handler(vdev, raw_num, ISP_FE_CH0);
	}

	if (frame_done & BIT(1)) {
		++vdev->splt_wdma_frm_num[raw_num][ISP_FE_CH1];
		_isp_splt_wdma_done_handler(vdev, raw_num, ISP_FE_CH1);
	}

	// rdma done
	if (frame_done & BIT(2)) {
		++vdev->splt_rdma_frm_num[raw_num][ISP_FE_CH0];
		_isp_splt_rdma_done_handler(vdev, raw_num, ISP_FE_CH0);
	}

	if (frame_done & BIT(3)) {
		++vdev->splt_rdma_frm_num[raw_num][ISP_FE_CH1];
		_isp_splt_rdma_done_handler(vdev, raw_num, ISP_FE_CH1);
	}

}

static void _isp_pre_fe_frame_start_chk(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	const u8 frame_start)
{
	if (frame_start & BIT(0)) {
		vi_record_sof_perf(vdev, raw_num, ISP_FE_CH0);

		if (!vdev->ctx.isp_pipe_cfg[raw_num].is_raw_replay_be)
			++vdev->pre_fe_sof_cnt[raw_num][ISP_FE_CH0];

		vi_pr(VI_DBG, "pre_fe_%d sof chn_num=%d frm_num=%d\n",
			raw_num, ISP_FE_CH0, vdev->pre_fe_sof_cnt[raw_num][ISP_FE_CH0]);

		if (vdev->ctx.isp_pipe_cfg[raw_num].is_yuv_sensor) { //YUV sensor
			if (!vdev->ctx.isp_pipe_cfg[raw_num].is_offline_scaler)
				tasklet_hi_schedule(&vdev->job_work);
		} else { //RGB sensor
			if (!vdev->ctx.isp_pipe_cfg[raw_num].is_raw_replay_be) {
				_isp_sof_handler(vdev, raw_num);
				vi_event_queue(vdev, VI_EVENT_PRE0_SOF + vi_get_dev_num_by_raw(&vdev->ctx, raw_num),
						vdev->pre_fe_sof_cnt[raw_num][ISP_FE_CH0]);
			}
		}
	}

	if (frame_start & BIT(1)) {
		++vdev->pre_fe_sof_cnt[raw_num][ISP_FE_CH1];
	}

	if (frame_start & BIT(2)) {
		++vdev->pre_fe_sof_cnt[raw_num][ISP_FE_CH2];
	}

	if (frame_start & BIT(3)) {
		++vdev->pre_fe_sof_cnt[raw_num][ISP_FE_CH3];
	}
}

static void _isp_pre_fe_frame_done_chk(
	struct cvi_vi_dev *vdev,
	const enum cvi_isp_raw raw_num,
	const u8 frame_done)
{
	if (frame_done & BIT(0)) {
		vi_record_fe_perf(vdev, raw_num, ISP_FE_CH0);

		++vdev->pre_fe_frm_num[raw_num][ISP_FE_CH0];
		_isp_pre_fe_done_handler(vdev, raw_num, ISP_FE_CH0);
	}

	if (frame_done & BIT(1)) {
		++vdev->pre_fe_frm_num[raw_num][ISP_FE_CH1];
		_isp_pre_fe_done_handler(vdev, raw_num, ISP_FE_CH1);
	}

	if (frame_done & BIT(2)) {
		++vdev->pre_fe_frm_num[raw_num][ISP_FE_CH2];
		_isp_pre_fe_done_handler(vdev, raw_num, ISP_FE_CH2);
	}

	if (frame_done & BIT(3)) {
		++vdev->pre_fe_frm_num[raw_num][ISP_FE_CH3];
		_isp_pre_fe_done_handler(vdev, raw_num, ISP_FE_CH3);
	}
}

static void _isp_pre_be_frame_done_chk(
	struct cvi_vi_dev *vdev,
	const u8 frame_done)
{
	if (frame_done & BIT(0)) {
		vi_record_be_perf(vdev, vi_get_first_raw_num(&vdev->ctx), ISP_BE_CH0);

		_isp_pre_be_done_handler(vdev, ISP_BE_CH0);
	}

	if (frame_done & BIT(1)) {
		_isp_pre_be_done_handler(vdev, ISP_BE_CH1);
	}
}

static void _isp_postraw_shadow_done_chk(
	struct cvi_vi_dev *vdev,
	const u8 shadow_done)
{
	if (shadow_done) {
		_isp_postraw_shaw_done_handler(vdev);
	}
}

static void _isp_postraw_frame_done_chk(
	struct cvi_vi_dev *vdev,
	const u8 frame_done)
{
	if (frame_done) {
		vi_record_post_end(vdev, vi_get_first_raw_num(&vdev->ctx));

		_isp_postraw_done_handler(vdev);
	}
}

void vi_irq_handler(struct cvi_vi_dev *vdev)
{
	struct isp_ctx *ctx = &vdev->ctx;
	union REG_ISP_CSI_BDG_INTERRUPT_STATUS_0 cbdg_0_sts[ISP_PRERAW_MAX] = { 0 };
	union REG_ISP_CSI_BDG_INTERRUPT_STATUS_1 cbdg_1_sts[ISP_PRERAW_MAX] = { 0 };
	union REG_ISP_TOP_INT_EVENT0 top_sts_0;
	union REG_ISP_TOP_INT_EVENT1 top_sts_1;
	union REG_ISP_TOP_INT_EVENT2 top_sts_2;
	union REG_ISP_TOP_INT_EVENT0_FE345 top_sts_0_fe345;
	union REG_ISP_TOP_INT_EVENT1_FE345 top_sts_1_fe345;
	union REG_ISP_TOP_INT_EVENT2_FE345 top_sts_2_fe345;
	u8 i = 0, raw_num = ISP_PRERAW0;

	isp_intr_status(ctx, &top_sts_0, &top_sts_1, &top_sts_2,
			&top_sts_0_fe345, &top_sts_1_fe345, &top_sts_2_fe345);

	if (!atomic_read(&vdev->isp_streamon))
		return;

	vi_perf_record_dump();

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!vdev->ctx.isp_pipe_enable[raw_num])
			continue;

		isp_csi_intr_status(ctx, raw_num, &cbdg_0_sts[raw_num], &cbdg_1_sts[raw_num]);
	}

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!vdev->ctx.isp_pipe_enable[raw_num])
			continue;

		ctx->isp_pipe_cfg[raw_num].dg_info.bdg_int_sts_0 = cbdg_0_sts[raw_num].raw;
		ctx->isp_pipe_cfg[raw_num].dg_info.bdg_int_sts_1 = cbdg_1_sts[raw_num].raw;

		if (!ctx->isp_pipe_cfg[raw_num].is_bt_demux)
			ctx->isp_pipe_cfg[raw_num].dg_info.fe_sts = ispblk_fe_dbg_info(ctx, raw_num);

		if (raw_num == vi_get_first_raw_num(ctx)) {
			ctx->isp_pipe_cfg[raw_num].dg_info.be_sts = ispblk_be_dbg_info(ctx);
			ctx->isp_pipe_cfg[raw_num].dg_info.post_sts = ispblk_post_dbg_info(ctx);
			ctx->isp_pipe_cfg[raw_num].dg_info.dma_sts = ispblk_dma_dbg_info(ctx);
		}

		for (i = 0; i < ISP_FE_CHN_MAX; i++)
			ctx->isp_pipe_cfg[raw_num].dg_info.bdg_chn_debug[i] = ispblk_csibdg_chn_dbg(ctx, raw_num, i);
	}

	if (isp_err_chk(vdev, ctx, cbdg_0_sts, cbdg_1_sts) == -1)
		return;

	//if (top_sts_0.bits.INT_DMA_ERR)
	//	vi_pr(VI_ERR, "DMA error\n");

	for (raw_num = ISP_PRERAW0; raw_num < ISP_PRERAW_MAX; raw_num++) {
		if (!vdev->ctx.isp_pipe_enable[raw_num])
			continue;

		if (ctx->isp_pipe_cfg[raw_num].is_bt_demux) {
			_isp_csibdg_lite_frame_start_chk(vdev, raw_num, cbdg_0_sts[raw_num].raw);
			_isp_csibdg_lite_frame_done_chk(vdev, raw_num, cbdg_0_sts[raw_num].raw);
		}
	}

	_isp_cmdq_done_chk(vdev, top_sts_2.bits.CMDQ_INT);
	_isp_splt_frame_done_chk(vdev, ISP_PRERAW0, top_sts_0_fe345.bits.LINE_SPLITER_DMA_DONE_FE0);
	_isp_splt_frame_done_chk(vdev, ISP_PRERAW1, top_sts_0_fe345.bits.LINE_SPLITER_DMA_DONE_FE1);

	_isp_pre_fe_frame_start_chk(vdev, ISP_PRERAW0, top_sts_2.bits.FRAME_START_FE0);
	_isp_pre_fe_frame_start_chk(vdev, ISP_PRERAW1, top_sts_2.bits.FRAME_START_FE1);
	_isp_pre_fe_frame_start_chk(vdev, ISP_PRERAW2, top_sts_2.bits.FRAME_START_FE2);
	_isp_pre_fe_frame_start_chk(vdev, ISP_PRERAW3, top_sts_2_fe345.bits.FRAME_START_FE3);
	_isp_pre_fe_frame_start_chk(vdev, ISP_PRERAW4, top_sts_2_fe345.bits.FRAME_START_FE4);
	_isp_pre_fe_frame_start_chk(vdev, ISP_PRERAW5, top_sts_2_fe345.bits.FRAME_START_FE5);

	_isp_pre_fe_frame_done_chk(vdev, ISP_PRERAW0, top_sts_0.bits.FRAME_DONE_FE0);
	_isp_pre_fe_frame_done_chk(vdev, ISP_PRERAW1, top_sts_0.bits.FRAME_DONE_FE1);
	_isp_pre_fe_frame_done_chk(vdev, ISP_PRERAW2, top_sts_0.bits.FRAME_DONE_FE2);
	_isp_pre_fe_frame_done_chk(vdev, ISP_PRERAW3, top_sts_0_fe345.bits.FRAME_DONE_FE3);
	_isp_pre_fe_frame_done_chk(vdev, ISP_PRERAW4, top_sts_0_fe345.bits.FRAME_DONE_FE4);
	_isp_pre_fe_frame_done_chk(vdev, ISP_PRERAW5, top_sts_0_fe345.bits.FRAME_DONE_FE5);

	_isp_pre_be_frame_done_chk(vdev, top_sts_0.bits.FRAME_DONE_BE);
	_isp_postraw_shadow_done_chk(vdev, top_sts_0.bits.SHAW_DONE_POST);
	_isp_postraw_frame_done_chk(vdev, top_sts_0.bits.FRAME_DONE_POST);
}

static int get_dma_buf_size(struct cvi_vi_dev *vdev)
{
	struct isp_ctx *ctx = &vdev->ctx;
	enum cvi_isp_raw raw_max = ISP_PRERAW_MAX - 1;
	u32 tmp_size = 0;
	u32 buf_size = 0;
	u8 raw_num = 0;

	tmp_size = isp_mempool.size;
	isp_mempool.base = 0xabde2000; //tmp addr only for check alignment
	isp_mempool.size = 0x40000000; //1024M
	isp_mempool.byteused = 0;

	_vi_scene_ctrl(vdev);

	for (raw_num = ISP_PRERAW0; raw_num < raw_max; raw_num++) {
		if (!ctx->isp_pipe_enable[raw_num])
			continue;

		ctx->isp_pipe_cfg[raw_num].is_patgen_en = csi_patgen_en[raw_num];
		if (ctx->isp_pipe_cfg[raw_num].is_patgen_en) {
#ifndef PORTING_TEST
			vdev->usr_fmt.width = vdev->snr_info[raw_num].snr_fmt.img_size[0].active_w;
			vdev->usr_fmt.height = vdev->snr_info[raw_num].snr_fmt.img_size[0].active_h;
			vdev->usr_crop.width = vdev->snr_info[raw_num].snr_fmt.img_size[0].active_w;
			vdev->usr_crop.height = vdev->snr_info[raw_num].snr_fmt.img_size[0].active_h;
#else
			vdev->usr_fmt.width = 1920;
			vdev->usr_fmt.height = 1080;
			vdev->usr_crop.width = 1920;
			vdev->usr_crop.height = 1080;
#endif
			vdev->usr_fmt.code = ISP_BAYER_TYPE_BG;
			vdev->usr_crop.left = 0;
			vdev->usr_crop.top = 0;
		}
		vi_pr(VI_INFO, "pipe_%d patgen(%d), [usr]w_h(%d:%d), color mode(%d), [snr]w/h(%d:%d)\n",
						raw_num, vdev->ctx.isp_pipe_cfg[raw_num].is_patgen_en,
						vdev->usr_fmt.width, vdev->usr_fmt.height, vdev->usr_fmt.code,
						vdev->snr_info[raw_num].snr_fmt.img_size[0].active_w,
						vdev->snr_info[raw_num].snr_fmt.img_size[0].active_h);

		_vi_ctrl_init(raw_num, vdev);
	}

	if (ctx->isp_pipe_cfg[ISP_PRERAW0].is_tile) {
		ctx->isp_pipe_enable[ISP_PRERAW1] = true;
		ctx->isp_bind_info[ISP_PRERAW1].is_bind = true;
		ctx->isp_bind_info[ISP_PRERAW1].bind_dev_num = ctx->isp_bind_info[ISP_PRERAW0].bind_dev_num;
		_isp_tile_calc_size(ctx);
	}

	_vi_get_dma_buf_size(&vdev->ctx);

	buf_size = isp_mempool.byteused;

	isp_mempool.base	= 0;
	isp_mempool.size	= tmp_size;
	isp_mempool.byteused	= 0;

	return buf_size;
}

/*************************************************************************
 *	ISP V4L2 definition
 *************************************************************************/
static void subcall_open(struct cvi_vi_dev *vdev, bool on)
{
	struct cvi_isp_device *dev =
		container_of(vdev, struct cvi_isp_device, vi_dev);
	struct v4l2_subdev *isp_sd = &dev->isp_sdev.sd;

	v4l2_subdev_call(isp_sd, core, s_power, on);
}

static void subcall_s_stream(struct cvi_vi_dev *vdev, bool on)
{
	struct cvi_isp_device *dev =
		container_of(vdev, struct cvi_isp_device, vi_dev);
	struct v4l2_subdev *isp_sd = &dev->isp_sdev.sd;

	v4l2_subdev_call(isp_sd, video, s_stream, on);
}

static void subcall_s_sensor_hdr(struct cvi_vi_dev *vdev, u8 chn_id, int on)
{
	struct cvi_isp_device *dev =
		container_of(vdev, struct cvi_isp_device, vi_dev);
	struct v4l2_subdev *sensor_sd = dev->sensors[chn_id].sd;
	int hdr_on = on;

	if (!sensor_sd) {
		vi_pr(VI_INFO, "sensor subdev is NULL!\n");
		return;
	}

	v4l2_subdev_call(sensor_sd, core, ioctl, SNS_V4L2_SET_HDR_ON, &hdr_on);
}

static int subcall_get_sensor_mode(struct cvi_vi_dev *vdev, u8 raw_num)
{
	struct cvi_isp_device *dev =
		container_of(vdev, struct cvi_isp_device, vi_dev);
	struct isp_ctx *ctx = &vdev->ctx;
	u8 chn_id = vi_get_dev_num_by_raw(ctx, raw_num);
	struct v4l2_subdev *sensor_sd = dev->sensors[chn_id].sd;
	struct v4l2_ctrl *link_freq;
	struct v4l2_querymenu qm = { .id = V4L2_CID_LINK_FREQ};
	int ret = 0;
	int mode_index = SNS_CFG_TYPE_WDR_MODE;

	if (!sensor_sd) {
		vi_pr(VI_INFO, "sensor subdev is NULL!\n");
		return -1;
	}

	link_freq = v4l2_ctrl_find(sensor_sd->ctrl_handler, V4L2_CID_LINK_FREQ);
	if (!link_freq) {
		vi_pr(VI_INFO, "No pixel rate control in %s\n", sensor_sd->name);
		return -EPIPE;
	}

	qm.index = v4l2_ctrl_g_ctrl(link_freq);
	ret = v4l2_querymenu(sensor_sd->ctrl_handler, &qm);
	if (ret < 0) {
		vi_pr(VI_INFO, "Failed to get sns clk menu\n");
		return ret;
	}

	//get wdr mode
	qm.index = mode_index;
	ret = v4l2_querymenu(sensor_sd->ctrl_handler, &qm);
	ctx->isp_pipe_cfg[raw_num].is_stagger_vsync = 0;
	if (qm.value == CVI_MIPI_WDR_MODE_NONE) { //hdr off
		ctx->is_hdr_on = 0;
		ctx->isp_pipe_cfg[raw_num].is_hdr_on = 0;
		vdev->snr_info[raw_num].snr_fmt.frm_num = 1;
	} else {//hdr on
		ctx->is_hdr_on = 1;
		ctx->isp_pipe_cfg[raw_num].is_hdr_on = 1;
		if (qm.value == CVI_MIPI_WDR_MODE_VC) {
			ctx->isp_pipe_cfg[raw_num].is_stagger_vsync = 1;
		}
		vdev->snr_info[raw_num].snr_fmt.frm_num = 2;
	}

	return ret;
}

static int subcall_get_sensor_info(struct cvi_vi_dev *vdev, u8 chn_id, u8 *raw_num)
{
	struct cvi_isp_device *dev =
		container_of(vdev, struct cvi_isp_device, vi_dev);
	struct v4l2_subdev *sensor_sd = dev->sensors[chn_id].sd;
	struct v4l2_ctrl *link_freq;
	struct v4l2_querymenu qm = { .id = V4L2_CID_LINK_FREQ};
	int ret = 0;
	int devno_index = SNS_CFG_TYPE_MIPI_DEV;

	if (!sensor_sd) {
		vi_pr(VI_INFO, "sensor subdev is NULL!\n");
		return -1;
	}

	link_freq = v4l2_ctrl_find(sensor_sd->ctrl_handler, V4L2_CID_LINK_FREQ);
	if (!link_freq) {
		vi_pr(VI_INFO, "No pixel rate control in %s\n", sensor_sd->name);
		return -EPIPE;
	}

	qm.index = v4l2_ctrl_g_ctrl(link_freq);
	ret = v4l2_querymenu(sensor_sd->ctrl_handler, &qm);
	if (ret < 0) {
		vi_pr(VI_INFO, "Failed to get sns clk menu\n");
		return ret;
	}

	//get mac devno
	qm.index = devno_index;
	ret = v4l2_querymenu(sensor_sd->ctrl_handler, &qm);
	if (qm.value >= ISP_PRERAW_MAX || qm.value < 0) {
		vi_pr(VI_INFO, "invalid devno(%lld)\n", qm.value);
		return ret;
	}
	*raw_num = qm.value;
	vi_pr(VI_INFO, "%s links to mac%lld\n", sensor_sd->name, qm.value);

	return ret;
}

static int subcall_get_size(struct cvi_vi_dev *vdev, u8 chn_id,
							struct v4l2_subdev_frame_size_enum *fse)
{
	struct cvi_isp_device *dev =
		container_of(vdev, struct cvi_isp_device, vi_dev);
	struct v4l2_subdev *sd = dev->sensors[chn_id].sd;

	if (!sd) {
		vi_pr(VI_INFO, "sensor[%d] sd is NULL!\n", chn_id);
		return -1;
	}

	fse->index = 0;
	fse->pad = CVI_ISP_PAD_SINK;
	fse->which = V4L2_SUBDEV_FORMAT_ACTIVE;

	v4l2_subdev_call(sd, pad, enum_frame_size, NULL, fse);

	return 0;
}

static void _subdev_init(struct cvi_vi_dev *vdev, u8 chn_id, u8 *raw_num)
{
	struct v4l2_subdev_frame_size_enum fse;
	struct isp_ctx *ctx = &vdev->ctx;
	int ret;
	u32 max_w, max_h;
	u32 crop_w, crop_h;

	ret = subcall_get_sensor_info(vdev, chn_id, raw_num);
	if (ret < 0) {
		vi_pr(VI_INFO, "use chn_id(%d) as raw_num, may effect orignal bind\n", chn_id);
		*raw_num = chn_id;
	}

#if defined(__CV186X__)
	if (*raw_num == ISP_PRERAW1)
		*raw_num = ISP_PRERAW3;
	else if (*raw_num == ISP_PRERAW3)
		*raw_num = ISP_PRERAW1;
#endif

	ret = subcall_get_size(vdev, chn_id, &fse);

	if (ret < 0) {
		max_w = ISP_DEFAULT_WIDTH;
		max_h = ISP_DEFAULT_HEIGHT;
		crop_w = ISP_DEFAULT_WIDTH;
		crop_h = ISP_DEFAULT_HEIGHT;
		vi_pr(VI_INFO, "pipe_%d use default size: max(%d*%d) crop(%d*%d)\n",
			chn_id, max_w, max_h, crop_w, crop_h);
	} else {
		max_w = fse.max_width;
		max_h = fse.max_height;
		crop_w = fse.min_width;
		crop_h = fse.min_height;
		vi_pr(VI_INFO, "get size from sensor%d: max(%d*%d) crop(%d*%d)\n",
			chn_id, max_w, max_h, crop_w, crop_h);
	}

	ctx->isp_pipe_cfg[*raw_num].csibdg_width = max_w;
	ctx->isp_pipe_cfg[*raw_num].csibdg_height = max_h;
	ctx->isp_pipe_cfg[*raw_num].post_img_w = crop_w;
	ctx->isp_pipe_cfg[*raw_num].post_img_h = crop_h;

	// use default fmt
	vdev->fmt[*raw_num] = &cvi_vip_formats[ISP_OUT_FMT_NV21];
}

static int get_video_index(struct video_device *dev)
{
	char devname[16];
	int videoId = 0;
	int strLen = sizeof("videoX");

	memcpy(devname, video_device_node_name(dev), strLen);
	videoId = devname[strLen - 2] - '0';
	if (videoId < 0 || videoId >= VI_MAX_CHN_NUM) {
		vi_pr(VI_ERR, "invalid dev name:/dev/%s\n", devname);
		return 0;
	}

	return videoId;
}

static int calculate_sizeimage(struct cvi_vi_dev *vdev, u8 raw_num)
{
	struct isp_ctx *ctx = &vdev->ctx;
	u32 crop_w = ctx->isp_pipe_cfg[raw_num].post_img_w;
	u32 crop_h = ctx->isp_pipe_cfg[raw_num].post_img_h;
	u32 uv_ratio = 1;

	uv_ratio = vdev->fmt[raw_num]->bit_depth[1] == vdev->fmt[raw_num]->bit_depth[2] ? 1 : 2;

	vdev->pipe[raw_num].bytesperline[0] = vdev->pipe[raw_num].bytesperline[1] = crop_w;
	vdev->pipe[raw_num].sizeimage[0] = vdev->pipe[raw_num].bytesperline[0] * crop_h;
	vdev->pipe[raw_num].sizeimage[1] = vdev->pipe[raw_num].bytesperline[1] * crop_h / uv_ratio;

	return 0;
}

static void _v4l2_init_config_info(struct cvi_vi_dev *vdev, u8 ViDev, u8 raw_num)
{
	struct cvi_isp_snr_info snr_info;
	VI_PIPE_ATTR_S stPipeAttr;
	struct isp_ctx *ctx = &vdev->ctx;
	u32 *src_w = &ctx->isp_pipe_cfg[raw_num].csibdg_width;
	u32 *src_h = &ctx->isp_pipe_cfg[raw_num].csibdg_height;
	u32 *crop_w = &ctx->isp_pipe_cfg[raw_num].post_img_w;
	u32 *crop_h = &ctx->isp_pipe_cfg[raw_num].post_img_h;

	vi_pr(VI_INFO, "config videv_%d, raw_pipe_%d\n", ViDev, raw_num);

	check_size(src_w, src_h);
	check_size(crop_w, crop_h);

	gViCtx->devAttr[ViDev].enIntfMode = VI_MODE_MIPI;
	gViCtx->devAttr[ViDev].stSize.u32Width = *src_w;
	gViCtx->devAttr[ViDev].stSize.u32Height = *src_h;
	gViCtx->chnStatus[ViDev].bEnable = CVI_TRUE;
	gViCtx->chnStatus[ViDev].stSize.u32Width = *crop_w;
	gViCtx->chnStatus[ViDev].stSize.u32Height = *crop_h;
	gViCtx->chnStatus[ViDev].u32IntCnt = 0;
	gViCtx->chnStatus[ViDev].u32FrameNum = 0;
	gViCtx->chnStatus[ViDev].u64PrevTime = 0;
	gViCtx->chnStatus[ViDev].u32FrameRate = 0;

	stPipeAttr.bYuvSkip = CVI_FALSE;
	stPipeAttr.bYuvBypassPath = CVI_FALSE;
	stPipeAttr.u32MaxW = gViCtx->devAttr[ViDev].stSize.u32Width;
	stPipeAttr.u32MaxH = gViCtx->devAttr[ViDev].stSize.u32Height;
	stPipeAttr.enPixFmt = PIXEL_FORMAT_RGB_BAYER_12BPP;
	stPipeAttr.enBitWidth = DATA_BITWIDTH_12;
	stPipeAttr.stFrameRate.s32SrcFrameRate = -1;
	stPipeAttr.stFrameRate.s32DstFrameRate = -1;
	stPipeAttr.bNrEn = CVI_TRUE;
	stPipeAttr.bYuvBypassPath = CVI_FALSE;
	stPipeAttr.enCompressMode = COMPRESS_MODE_NONE;
	memset(&gViCtx->pipeAttr[ViDev], 0, sizeof(gViCtx->pipeAttr[ViDev]));
	gViCtx->isPipeCreated[ViDev] = true;
	if (!(gViCtx->enSource[ViDev] == VI_PIPE_FRAME_SOURCE_USER_FE ||
		gViCtx->enSource[ViDev] == VI_PIPE_FRAME_SOURCE_USER_BE))
		gViCtx->enSource[ViDev] = VI_PIPE_FRAME_SOURCE_DEV;
	gViCtx->pipeAttr[ViDev] = stPipeAttr;

	snr_info.raw_num = raw_num;
	vdev->usr_fmt.width = *crop_w;
	vdev->usr_fmt.height = *crop_h;
	vdev->ctx.isp_pipe_enable[snr_info.raw_num] = true; //A2 config
	vdev->snr_info[raw_num].snr_fmt.img_size[0].max_width = *src_w;
	vdev->snr_info[raw_num].snr_fmt.img_size[0].max_height = *src_h;
	vdev->snr_info[raw_num].snr_fmt.img_size[0].start_x = 0;
	vdev->snr_info[raw_num].snr_fmt.img_size[0].start_y = 0;
	vdev->snr_info[raw_num].snr_fmt.img_size[0].width = *src_w;
	vdev->snr_info[raw_num].snr_fmt.img_size[0].height = *src_h;
	vdev->snr_info[raw_num].snr_fmt.img_size[0].active_w = *crop_w;
	vdev->snr_info[raw_num].snr_fmt.img_size[0].active_h = *crop_h;
	if (vdev->snr_info[raw_num].snr_fmt.frm_num == 2) {
		vdev->snr_info[raw_num].snr_fmt.img_size[1].max_width = *src_w;
		vdev->snr_info[raw_num].snr_fmt.img_size[1].max_height = *src_h;
		vdev->snr_info[raw_num].snr_fmt.img_size[1].start_x = 0;
		vdev->snr_info[raw_num].snr_fmt.img_size[1].start_y = 0;
		vdev->snr_info[raw_num].snr_fmt.img_size[1].width = *src_w;
		vdev->snr_info[raw_num].snr_fmt.img_size[1].height = *src_h;
		vdev->snr_info[raw_num].snr_fmt.img_size[1].active_w = *crop_w;
		vdev->snr_info[raw_num].snr_fmt.img_size[1].active_h = *crop_h;
	}

	calculate_sizeimage(vdev, raw_num);
}

static int cvi_isp_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct cvi_vi_dev *vdev = video_drvdata(file);
	vi_pr(VI_INFO, "+\n");
	cap->capabilities = vdev->vid_caps | V4L2_CAP_DEVICE_CAPS |
						V4L2_CAP_VIDEO_CAPTURE;

	vi_pr(VI_INFO, "caps:0x%x\n", cap->capabilities);
	return 0;
}

static int cvi_isp_g_ctrl(
	struct file *file,
	void *priv,
	struct v4l2_control *vc)
{
	int rc = -EINVAL;
	return rc;
}

static int cvi_isp_s_ctrl(
	struct file *file,
	void *priv,
	struct v4l2_control *vc)
{
	int rc = -EINVAL;
	return rc;
}

static int cvi_isp_g_ext_ctrls(
	struct file *file,
	void *priv,
	struct v4l2_ext_controls *vc)
{
	struct cvi_vi_dev *vdev = video_drvdata(file);
	struct video_device *video = video_devdata(file);
	struct isp_ctx *ctx = &vdev->ctx;
	struct v4l2_ext_control *p = vc->controls;
	u8 chn_id = get_video_index(video);
	u8 raw_num = vi_get_raw_num_by_dev(ctx, chn_id);
	int rc = -EINVAL, i = 0;

	WARN_ON(!vdev);

	if (raw_num >= ISP_PRERAW_MAX) {
		vi_pr(VI_INFO, "wrong raw num(%d)!\n", raw_num);
		return rc;
	}

	for (i = 0; i < vc->count; ++i) {
		switch (p[i].id) {
		case VI_IOCTL_STS_GET:
		{
			unsigned long flags;

			spin_lock_irqsave(&isp_bufpool[raw_num].pre_be_sts_lock, flags);
			isp_bufpool[raw_num].pre_be_sts_in_use = 1;
			p->value = isp_bufpool[raw_num].pre_be_sts_busy_idx ^ 1;
			spin_unlock_irqrestore(&isp_bufpool[raw_num].pre_be_sts_lock, flags);

			rc = 0;
			break;
		}

		case VI_IOCTL_POST_STS_GET:
		{
			unsigned long flags;

			spin_lock_irqsave(&isp_bufpool[raw_num].post_sts_lock, flags);
			isp_bufpool[raw_num].post_sts_in_use = 1;
			p->value = isp_bufpool[raw_num].post_sts_busy_idx ^ 1;
			spin_unlock_irqrestore(&isp_bufpool[raw_num].post_sts_lock, flags);

			rc = 0;
			break;
		}

		case VI_IOCTL_STS_MEM:
		{
			struct cvi_isp_sts_mem sts_mem;
			int rval = 0;

			if (copy_from_user(&sts_mem, p->ptr, sizeof(struct cvi_isp_sts_mem)) != 0)
				break;

			raw_num = vi_get_raw_num_by_dev(ctx, sts_mem.raw_num);
			if (raw_num >= ISP_PRERAW_MAX) {
				vi_pr(VI_ERR, "sts_mem wrong raw_num(%d)\n", raw_num);
				break;
			}

			rval = copy_to_user(p->ptr,
						isp_bufpool[raw_num].sts_mem,
						sizeof(struct cvi_isp_sts_mem) * 2);

			if (rval)
				vi_pr(VI_ERR, "fail copying %d bytes of ISP_STS_MEM info\n", rval);
			else
				rc = 0;
			break;
		}

		case VI_IOCTL_GET_LSC_PHY_BUF:
		{
			struct cvi_vip_memblock *isp_mem;

			isp_mem = vmalloc(sizeof(struct cvi_vip_memblock));
			if (copy_from_user(isp_mem, p->ptr, sizeof(struct cvi_vip_memblock)) != 0) {
				vfree(isp_mem);
				break;
			}

			raw_num = vi_get_raw_num_by_dev(ctx, isp_mem->raw_num);

			isp_mem->phy_addr = isp_bufpool[raw_num].lsc;
			isp_mem->size = ispblk_dma_buf_get_size(ctx, raw_num, ISP_BLK_ID_DMA_CTL_LSC_LE);

			if (copy_to_user(p->ptr, isp_mem, sizeof(struct cvi_vip_memblock)) != 0) {
				vfree(isp_mem);
				break;
			}

			vfree(isp_mem);

			rc = 0;
			break;
		}

		case VI_IOCTL_GET_PIPE_DUMP:
		{
			struct cvi_vip_isp_raw_blk dump[2];

			if (copy_from_user(&dump[0], p->ptr, sizeof(struct cvi_vip_isp_raw_blk) * 2) != 0)
				break;

	#if 0//PORTING_TEST //test only
			dump[0].raw_dump.phy_addr = 0x11223344;
			if (copy_to_user(p->ptr, &dump[0], sizeof(struct cvi_vip_isp_raw_blk) * 2) != 0)
				break;
			rc = 0;
	#else
			dump[0].raw_dump.raw_num = raw_num;
			rc = isp_raw_dump(vdev, &dump[0]);
			if (copy_to_user(p->ptr, &dump[0], sizeof(struct cvi_vip_isp_raw_blk) * 2) != 0)
				break;
	#endif
			break;
		}

		case VI_IOCTL_GET_SCENE_INFO:
		{
			enum ISP_SCENE_INFO info = FE_ON_BE_OFF_POST_ON_SC;

			if (ctx->isp_pipe_cfg[ISP_PRERAW0].is_offline_scaler) {
				if (_is_fe_be_online(ctx))
					info = FE_ON_BE_OFF_POST_OFF_SC;
				else if (_is_be_post_online(ctx))
					info = FE_OFF_BE_ON_POST_OFF_SC;
				else if (_is_all_online(ctx))
					info = FE_ON_BE_ON_POST_OFF_SC;
			} else {
				if (_is_fe_be_online(ctx))
					info = FE_ON_BE_OFF_POST_ON_SC;
				else if (_is_be_post_online(ctx))
					info = FE_OFF_BE_ON_POST_ON_SC;
				else if (_is_all_online(ctx))
					info = FE_ON_BE_ON_POST_ON_SC;
			}

			p->value = info;

			rc = 0;
			break;
		}

		case VI_IOCTL_GET_BUF_SIZE:
		{
			p->value = get_dma_buf_size(vdev);

			rc = 0;
			break;
		}

		case VI_IOCTL_GET_DEV_NUM:
		{
			p->value = gViCtx->total_dev_num;
			rc = 0;
			break;
		}

		case VI_IOCTL_GET_TUN_ADDR:
		{
			void *tun_addr = NULL;
			u32 size;

			vi_tuning_buf_setup(&vdev->ctx);

			tun_addr = vi_get_tuning_buf_addr(&size);

			if (copy_to_user(p->ptr, tun_addr, size) != 0) {
				vi_pr(VI_ERR, "Failed to copy tun_addr\n");
				break;
			}

			rc = 0;
			break;
		}

		case VI_IOCTL_DQEVENT:
		{
			struct vi_event ev_u = {.type = VI_EVENT_MAX};
			struct vi_event_k *ev_k;
			unsigned long flags;

			spin_lock_irqsave(&event_lock, flags);
			if (!list_empty(&event_q.list)) {
				ev_k = list_first_entry(&event_q.list, struct vi_event_k, list);
				ev_u.dev_id		= ev_k->ev.dev_id;
				ev_u.type		= ev_k->ev.type;
				ev_u.frame_sequence	= ev_k->ev.frame_sequence;
				ev_u.timestamp		= ev_k->ev.timestamp;
				list_del_init(&ev_k->list);
				kfree(ev_k);
			} else {
				spin_unlock_irqrestore(&event_lock, flags);
				rc = 0;
				break;
			}
			spin_unlock_irqrestore(&event_lock, flags);

			if (copy_to_user(p->ptr, &ev_u, sizeof(struct vi_event))) {
				vi_pr(VI_ERR, "Failed to dqevent\n");
				break;
			}

			rc = 0;
			break;
		}

		case VI_IOCTL_GET_CLUT_TBL_IDX:
		{
			int tun_idx = p->value;

			p->value = vi_tuning_get_clut_tbl_idx(raw_num, tun_idx);

			if (p->value < 0) {
				break;
			}

			rc = 0;
			break;
		}

		case VI_IOCTL_GET_FSWDR_PHY_BUF:
		{
			struct cvi_vip_memblock *isp_mem;

			isp_mem = vmalloc(sizeof(struct cvi_vip_memblock));
			if (copy_from_user(isp_mem, p->ptr, sizeof(struct cvi_vip_memblock)) != 0) {
				vfree(isp_mem);
				break;
			}

			isp_mem->size = sizeof(struct cvi_vip_isp_fswdr_report);
			if (isp_bufpool[raw_num].fswdr_rpt == NULL) {
				isp_bufpool[raw_num].fswdr_rpt = kmalloc(
					isp_mem->size, GFP_DMA | GFP_KERNEL);
				if (isp_bufpool[raw_num].fswdr_rpt == NULL) {
					vi_pr(VI_ERR, "dev_%d isp_bufpool[%d].fswdr_rpt alloc size(%d) fail\n",
						isp_mem->raw_num, raw_num, isp_mem->size);
					vfree(isp_mem);
					break;
				}
			}
			isp_mem->vir_addr = isp_bufpool[raw_num].fswdr_rpt;
			isp_mem->phy_addr = virt_to_phys(isp_bufpool[raw_num].fswdr_rpt);

			if (copy_to_user(p->ptr, isp_mem, sizeof(struct cvi_vip_memblock)) != 0) {
				vfree(isp_mem);
				break;
			}

			vfree(isp_mem);

			rc = 0;
			break;
		}

		case VI_IOCTL_GET_IP_INFO:
		{
			if (copy_to_user(p->ptr, &ip_info_list,
					sizeof(struct ip_info) * IP_INFO_ID_MAX) != 0) {
				vi_pr(VI_ERR, "Failed to copy ip_info_list\n");
				break;
			}

			rc = 0;
			break;
		}

		case VI_IOCTL_GET_RGBMAP_LE_PHY_BUF:
		{
			struct cvi_vip_memblock *isp_mem;
			u8 raw_num;

			isp_mem = vmalloc(sizeof(struct cvi_vip_memblock));
			if (copy_from_user(isp_mem, p->ptr, sizeof(struct cvi_vip_memblock)) != 0) {
				vfree(isp_mem);
				break;
			}

			raw_num = vi_get_raw_num_by_dev(ctx, isp_mem->raw_num);
			isp_mem->phy_addr = isp_bufpool[raw_num].rgbmap_le[0];
			isp_mem->size = ispblk_dma_buf_get_size(ctx, raw_num, ISP_BLK_ID_DMA_CTL_FE0_RGBMAP_LE);

			if (copy_to_user(p->ptr, isp_mem, sizeof(struct cvi_vip_memblock)) != 0) {
				vfree(isp_mem);
				break;
			}

			vfree(isp_mem);

			rc = 0;
			break;
		}

		case VI_IOCTL_GET_RGBMAP_SE_PHY_BUF:
		{
			struct cvi_vip_memblock *isp_mem;
			u8 raw_num;

			isp_mem = vmalloc(sizeof(struct cvi_vip_memblock));
			if (copy_from_user(isp_mem, p->ptr, sizeof(struct cvi_vip_memblock)) != 0) {
				vfree(isp_mem);
				break;
			}

			raw_num = vi_get_raw_num_by_dev(ctx, isp_mem->raw_num);
			isp_mem->phy_addr = isp_bufpool[raw_num].rgbmap_se[0];
			isp_mem->size = ispblk_dma_buf_get_size(ctx, raw_num, ISP_BLK_ID_DMA_CTL_FE0_RGBMAP_SE);

			if (copy_to_user(p->ptr, isp_mem, sizeof(struct cvi_vip_memblock)) != 0) {
				vfree(isp_mem);
				break;
			}

			vfree(isp_mem);

			rc = 0;
			break;
		}

		case VI_IOCTL_GET_V4L2_BUF_PHY_ADDR:
		{
			struct cvi_isp_buf *cvi_vb, *tmp;
			struct vb2_buffer *vb2_buf;
			unsigned long flags;
			uint64_t *dma_addr;
			int size;
			int i = 0;

			if (!p->ptr) {
				vi_pr(VI_ERR, "error ptr!\n");
				break;
			}

			size = vdev->qbuf_num[raw_num] * sizeof(uint64_t);
			if (size <= 0) {
				vi_pr(VI_ERR, "raw_%d qbuf_num is error!\n", raw_num);
				break;
			}
			dma_addr = vmalloc(size);

			spin_lock_irqsave(&vdev->qbuf_lock, flags);
			list_for_each_entry_safe(cvi_vb, tmp, &(vdev->qbuf_list[raw_num]), list) {
				vb2_buf = &(cvi_vb->buf.vb2_buf);
				dma_addr[i] = vb2_dma_contig_plane_dma_addr(vb2_buf, 0);
				i++;
			}
			spin_unlock_irqrestore(&vdev->qbuf_lock, flags);

			if (copy_to_user(p->ptr, dma_addr, size) != 0) {
				vfree(dma_addr);
				break;
			}

			vfree(dma_addr);

			rc = 0;
			break;
		}

		case VI_IOCTL_GET_PIPE_ATTR:
		{
			if (gViCtx->pipeAttr[chn_id].u32MaxW == 0 &&
				gViCtx->pipeAttr[chn_id].u32MaxH == 0) {
				vi_pr(VI_ERR, "SetPipeAttr first\n");
				return -1;
			}

			if (copy_to_user(p->ptr, &gViCtx->pipeAttr[chn_id],
					sizeof(VI_PIPE_ATTR_S)) != 0) {
				vi_pr(VI_ERR, "VI_PIPE_ATTR_S copy to user fail.\n");
				rc = -1;
			}

			rc = 0;
			break;
		}

		case VI_IOCTL_GET_CHN_FRAME:
		{
			VIDEO_FRAME_INFO_S v_frm_info;

			vi_pr(VI_INFO, "GET_CHN_FRAME\n");

			if (copy_from_user(&v_frm_info, p->ptr, sizeof(VIDEO_FRAME_INFO_S)) != 0) {
				vi_pr(VI_ERR, "VIDEO_FRAME_INFO_S copy from user fail.\n");
				break;
			}

			rc = vi_get_chn_frame(vdev, chn_id, &v_frm_info, 1000);

			if (copy_to_user(p->ptr, &v_frm_info, sizeof(VIDEO_FRAME_INFO_S)) != 0) {
				vi_pr(VI_ERR, "VIDEO_FRAME_INFO_S copy to user fail.\n");
				rc = -1;
				break;
			}
			break;
		}

		case VI_IOCTL_GET_PIPE_FRAME:
		{
			VIDEO_FRAME_INFO_S v_frm_info[2];

			if (copy_from_user(v_frm_info, p->ptr, sizeof(VIDEO_FRAME_INFO_S) * 2) != 0) {
				vi_pr(VI_ERR, "VIDEO_FRAME_INFO_S copy from user fail.\n");
				break;
			}

			rc = vi_get_pipe_frame(vdev, chn_id, v_frm_info, 1000);

			if (copy_to_user(p->ptr, v_frm_info, sizeof(VIDEO_FRAME_INFO_S) * 2) != 0) {
				vi_pr(VI_ERR, "VIDEO_FRAME_INFO_S copy to user fail.\n");
				rc = -1;
				break;
			}
			break;
		}

		case VI_IOCTL_GET_PIPE_DUMP_ATTR:
		{
			if (copy_to_user(p->ptr, &gViCtx->dumpAttr[chn_id],
					sizeof(VI_DUMP_ATTR_S)) != 0) {
				vi_pr(VI_ERR, "VI_PIPE_ATTR_S copy to user fail.\n");
				rc = -1;
				break;
			}

			break;
		}

		case VI_IOCTL_GET_DEV_ATTR:
		{
			if (copy_to_user(p->ptr, &gViCtx->devAttr[chn_id],
					sizeof(VI_DEV_ATTR_S)) != 0) {
				vi_pr(VI_ERR, "VI_DEV_ATTR_S copy to user fail.\n");
				break;
			}

			rc = 0;
			break;
		}

		case VI_IOCTL_GET_AI_ISP_RAW:
		{
			int timeout = 100;
			int ret;

			ret = wait_event_timeout(
			vdev->ai_isp_wait_q[raw_num],
			atomic_read(&vdev->ai_isp_int_flag[raw_num]) == 1,
			msecs_to_jiffies(timeout));

			if (!ret) {
				vi_pr(VI_WARN, "dev_%d wait raw timeout(%d ms)\n",
					chn_id, timeout);
				break;
			}

			if (copy_to_user(p->ptr, ai_isp_cfg_info[raw_num].ai_bnr_addr_pool,
					sizeof(ai_isp_cfg_info[raw_num].ai_bnr_addr_pool)) != 0) {
				break;
			}

			rc = 0;
			break;
		}

		default:
			vi_pr(VI_INFO, "unsupport ext ctrl cmd!\n");
			break;
		}
	}
	return rc;
}

static int cvi_isp_s_ext_ctrls(
	struct file *file,
	void *priv,
	struct v4l2_ext_controls *vc)
{
	struct cvi_vi_dev *vdev = video_drvdata(file);
	struct video_device *video = video_devdata(file);
	struct isp_ctx *ctx = &vdev->ctx;
	struct v4l2_ext_control *p = vc->controls;
	u8 chn_id = get_video_index(video);
	u8 raw_num = vi_get_raw_num_by_dev(ctx, chn_id);
	int rc = -EINVAL, i = 0;

	WARN_ON(!vdev);

	if (raw_num >= ISP_PRERAW_MAX) {
		vi_pr(VI_INFO, "wrong raw num(%d)!\n", raw_num);
		return rc;
	}

	for (i = 0; i < vc->count; ++i) {
		switch (p[i].id) {
		case VI_IOCTL_HDR:
		{
			vi_pr(VI_INFO, "set raw_%d HDR(%d)\n", raw_num, p->value);
			ctx->is_hdr_on = p->value;
			ctx->isp_pipe_cfg[raw_num].is_hdr_on = p->value;
			subcall_s_sensor_hdr(vdev, chn_id, p->value);
			rc = 0;
			break;
		}

		case VI_IOCTL_3DNR:
		{
			ctx->is_3dnr_on = p->value;
			vi_pr(VI_INFO, "is_3dnr_on=%d\n", ctx->is_3dnr_on);
			rc = 0;
			break;
		}

		case VI_IOCTL_NEW_3DNR:
		{
			ctx->isp_pipe_cfg[raw_num].tnr_mode =
				p->value ? ISP_TNR_TYPE_NEW_MODE : ISP_TNR_TYPE_OLD_MODE;
			vi_pr(VI_INFO, "set raw_%d is_new_3dnr_on=%d\n", raw_num,
				(ctx->isp_pipe_cfg[raw_num].tnr_mode == ISP_TNR_TYPE_NEW_MODE) ? 1 : 0);
			rc = 0;
			break;
		}

		case VI_IOCTL_COMPRESS_EN:
		{
			ctx->is_dpcm_on = p->value;
			vi_pr(VI_INFO, "ISP_COMPRESS_ON(%d)\n", ctx->is_dpcm_on);
			rc = 0;
			break;
		}

		case VI_IOCTL_STS_PUT:
		{
			unsigned long flags;

			spin_lock_irqsave(&isp_bufpool[raw_num].pre_be_sts_lock, flags);
			isp_bufpool[raw_num].pre_be_sts_in_use = 0;
			spin_unlock_irqrestore(&isp_bufpool[raw_num].pre_be_sts_lock, flags);

			rc = 0;
			break;
		}

		case VI_IOCTL_POST_STS_PUT:
		{
			unsigned long flags;

			spin_lock_irqsave(&isp_bufpool[raw_num].post_sts_lock, flags);
			isp_bufpool[raw_num].post_sts_in_use = 0;
			spin_unlock_irqrestore(&isp_bufpool[raw_num].post_sts_lock, flags);

			rc = 0;
			break;
		}

		case VI_IOCTL_ONLINE:
			ctx->is_offline_postraw = !p->value;
			vi_pr(VI_INFO, "is_offline_postraw=%d\n", ctx->is_offline_postraw);
			rc = 0;
			break;

		case VI_IOCTL_BE_ONLINE:
			ctx->is_offline_be = !p->value;
			vi_pr(VI_INFO, "is_offline_be=%d\n", ctx->is_offline_be);
			rc = 0;
			break;

		case VI_IOCTL_SET_SNR_CFG_NODE:
		{
			struct cvi_isp_snr_update *snr_update;

			if (vdev->ctx.isp_pipe_cfg[ISP_PRERAW0].is_raw_replay_be)
				break;

			snr_update = vmalloc(sizeof(struct cvi_isp_snr_update));
			if (copy_from_user(snr_update, p->ptr, sizeof(struct cvi_isp_snr_update)) != 0) {
				vi_pr(VI_ERR, "SNR_CFG_NODE copy from user fail.\n");
				vfree(snr_update);
				break;
			}
			raw_num = vi_get_raw_num_by_dev(ctx, snr_update->raw_num);

			if (raw_num >= ISP_PRERAW_MAX) {
				vfree(snr_update);
				break;
			}

			if (vdev->ctx.isp_pipe_cfg[raw_num].is_raw_replay_be ||
				vdev->ctx.isp_pipe_cfg[raw_num].is_patgen_en) {
				rc = 0;
				vfree(snr_update);
				break;
			}

			vi_pr(VI_DBG, "raw_num=%d, magic_num=%d, regs_num=%d, i2c_update=%d, isp_update=%d\n",
				raw_num,
				snr_update->snr_cfg_node.snsr.magic_num,
				snr_update->snr_cfg_node.snsr.regs_num,
				snr_update->snr_cfg_node.snsr.need_update,
				snr_update->snr_cfg_node.isp.need_update);

			_isp_snr_cfg_enq(snr_update, raw_num);

			vfree(snr_update);

			rc = 0;
			break;
		}

		case VI_IOCTL_SET_SNR_INFO:
		{
			struct cvi_isp_snr_info snr_info;

			if (copy_from_user(&snr_info, p->ptr, sizeof(struct cvi_isp_snr_info)) != 0)
				break;

			vdev->snr_info[raw_num].color_mode = snr_info.color_mode;
			vi_pr(VI_INFO, "set raw_%d color_mode=%d\n", raw_num, snr_info.color_mode);

			rc = 0;
			break;
		}

		case VI_IOCTL_MMAP_GRID_SIZE:
		{
			struct cvi_isp_mmap_grid_size m_gd_sz;

			if (copy_from_user(&m_gd_sz, p->ptr, sizeof(struct cvi_isp_mmap_grid_size)) != 0)
				break;

			raw_num = vi_get_raw_num_by_dev(ctx, m_gd_sz.raw_num);
			m_gd_sz.grid_size = ctx->mmap_grid_size[raw_num];

			if (copy_to_user(p->ptr, &m_gd_sz, sizeof(struct cvi_isp_mmap_grid_size)) != 0)
				break;

			rc = 0;
			break;
		}

		case VI_IOCTL_SET_PROC_CONTENT:
		{
			struct isp_proc_cfg proc_cfg;
			int rval = 0;

			rval = copy_from_user(&proc_cfg, p->ptr, sizeof(struct isp_proc_cfg));
			if ((rval != 0) || (proc_cfg.buffer_size == 0))
				break;
			isp_proc_setProcContent(proc_cfg.buffer, proc_cfg.buffer_size);

			rc = 0;
			break;
		}

		case VI_IOCTL_SET_DMA_BUF_INFO:
		{
			struct cvi_vi_dma_buf_info info;
			int rval = 0;

			rval = copy_from_user(&info, p->ptr, sizeof(struct cvi_vi_dma_buf_info));
			if ((rval != 0) || (info.size == 0) || (info.paddr == 0))
				break;

			isp_mempool.base = info.paddr;
			isp_mempool.size = info.size;

			vi_pr(VI_INFO, "ISP dma buf paddr(0x%llx) size=0x%x\n",
					isp_mempool.base, isp_mempool.size);

			rc = 0;
			break;
		}

		case VI_IOCTL_SET_PIPE_ATTR:
		{
			if (copy_from_user(&gViCtx->pipeAttr[chn_id], p->ptr,
					sizeof(VI_PIPE_ATTR_S)) != 0) {
				vi_pr(VI_ERR, "VI_PIPE_ATTR_S copy from user fail.\n");
				rc = -1;
				break;
			}

			rc = 0;
			break;
		}

		case VI_IOCTL_SET_BYPASS_FRM:
		{
			gViCtx->bypass_frm[raw_num] =  p->value;
			vi_pr(VI_INFO, "dev_%d set pipe_%d bypass_frm(%d)\n", chn_id, raw_num, p->value);
			rc = 0;
			break;
		}

		case VI_IOCTL_RELEASE_CHN_FRAME:
		{
			vi_pr(VI_INFO, "RELEASE_CHN_FRAME\n");

			rc = vi_release_chn_frame(vdev);
			break;
		}

		case VI_IOCTL_RELEASE_PIPE_FRAME:
		{
			VIDEO_FRAME_INFO_S v_frm_info[2];
			vi_pr(VI_INFO, "RELEASE_PIPE_FRAME\n");
			if (copy_from_user(v_frm_info, p->ptr, sizeof(VIDEO_FRAME_INFO_S) * 2) != 0) {
				vi_pr(VI_ERR, "VIDEO_FRAME_INFO_S copy from user fail.\n");
				break;
			}

			rc = vi_release_pipe_frame(vdev, raw_num);
			break;
		}

		case VI_IOCTL_SET_PIPE_DUMP_ATTR:
		{
			VI_DUMP_ATTR_S dump_attr;
			vi_pr(VI_INFO, "SET_PIPE_DUMP_ATTR\n");
			if (copy_from_user(&dump_attr, p->ptr, sizeof(VI_DUMP_ATTR_S)) != 0) {
				vi_pr(VI_ERR, "VI_PIPE_ATTR_S copy from user fail.\n");
				break;
			}
			gViCtx->dumpAttr[chn_id] = dump_attr;

			rc = 0;
			break;
		}

		case VI_IOCTL_SET_DEV_ATTR:
		{
			VI_DEV_ATTR_S dev_attr;
			u8 chn_num;

			vi_pr(VI_INFO, "raw_%d set dev_attr\n", raw_num);

			if (copy_from_user(&dev_attr, p->ptr, sizeof(VI_DEV_ATTR_S)) != 0) {
				vi_pr(VI_ERR, "VI_DEV_ATTR_S copy from user fail.\n");
				break;
			}

			if (dev_attr.enInputDataType == VI_DATA_TYPE_YUV) {
				switch (dev_attr.enWorkMode) {
				case VI_WORK_MODE_1Multiplex:
					chn_num = 1;
					break;
				case VI_WORK_MODE_2Multiplex:
					chn_num = 2;
					break;
				case VI_WORK_MODE_3Multiplex:
					chn_num = 3;
					break;
				case VI_WORK_MODE_4Multiplex:
					chn_num = 4;
					break;
				default:
					vi_pr(VI_ERR, "SNR work mode(%d) is wrong\n",
							dev_attr.enWorkMode);
					return rc;
				}
			}
			gViCtx->devAttr[chn_id].chn_num = chn_num;

			if (dev_attr.enInputDataType == VI_DATA_TYPE_YUV ||
				dev_attr.enInputDataType == VI_DATA_TYPE_YUV_EARLY) {
				ctx->isp_pipe_cfg[raw_num].is_yuv_sensor = true;
				ctx->isp_pipe_cfg[raw_num].infMode = dev_attr.enIntfMode;
				ctx->isp_pipe_cfg[raw_num].muxMode = dev_attr.enWorkMode;
				ctx->isp_pipe_cfg[raw_num].enDataSeq = dev_attr.enDataSeq;
				ctx->isp_pipe_cfg[raw_num].yuv_scene_mode = dev_attr.enYuvSceneMode;
				vdev->fmt[raw_num] = &cvi_vip_formats[ISP_OUT_FMT_YUYV];
				calculate_sizeimage(vdev, raw_num);
			}

			if (dev_attr.enIntfMode == VI_MODE_LVDS) {
				ctx->is_sublvds_path = true;
				vi_pr(VI_WARN, "SUBLVDS_PATH_ON(%d)\n", ctx->is_sublvds_path);
			}

			gViCtx->devAttr[chn_id] = dev_attr;

			rc = 0;
			break;
		}

		case VI_IOCTL_AI_ISP_CFG:
		{
			ai_isp_cfg_t cfg;

			if (copy_from_user(&cfg, p->ptr, sizeof(ai_isp_cfg_t)))
				break;

			vi_pr(VI_INFO, "[ai_isp_cfg] vipipe(%d) algo_type=%d, cfg_type=%d\n",
					cfg.ViPipe, cfg.ai_isp_type, cfg.ai_isp_cfg_type);

			rc = ai_isp_resolve_cfg(vdev, cfg);
			break;
		}

		case VI_IOCTL_AI_ISP_SET_BUF:
		{
			ai_isp_api_t usr_api;
			enum ai_isp_type ai_isp_type;

			if (copy_from_user(&usr_api, p->ptr, sizeof(ai_isp_api_t)))
				break;

			ai_isp_type = usr_api.ai_isp_type;
			raw_num = vi_get_raw_num_by_dev(ctx, usr_api.ViPipe);
			vi_pr(VI_DBG, "[ai_isp_set_buf] raw_num(%d) algo_type=%d\n",
				raw_num, ai_isp_type);

			rc = 0;
			break;
		}

		case VI_IOCTL_PUT_AI_ISP_RAW:
		{
			atomic_set(&vdev->ai_isp_int_flag[raw_num], 2);
			wake_up(&vdev->ai_isp_wait_q[raw_num]);

			rc = 0;
			break;
		}

		default:
			vi_pr(VI_INFO, "unsupport ext ctrl cmd!\n");
			break;
		}
	}

	return rc;
}

int cvi_isp_g_input(struct file *file, void *priv, unsigned int *i)
{
	struct video_device *vdev = video_devdata(file);
	WARN_ON(!vdev);

	*i = get_video_index(vdev);

	vi_pr(VI_INFO, "current input:%u\n", *i);

	return 0;
}

int cvi_isp_s_input(struct file *file, void *priv, unsigned int i)
{
	struct cvi_vi_dev *vdev = video_drvdata(file);
	WARN_ON(!vdev);

	vi_pr(VI_INFO, "no support set input:%u\n", i);

	return 0;
}

int cvi_isp_enum_input(struct file *file, void *priv, struct v4l2_input *inp)
{
	int rc = 0;

	inp->index = 1;
	inp->type = V4L2_INPUT_TYPE_CAMERA;

	return rc;
}

int cvi_isp_g_param(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	int rc = 0;
	u32 numerator = 40;
	u32 denominator = 1000;

	if (!a) {
		vi_pr(VI_ERR, "streamparm is NULL !!\n");
		return -1;
	}

	if (a->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		a->parm.capture.capturemode = 0;
		a->parm.capture.timeperframe.numerator = numerator;
		a->parm.capture.timeperframe.denominator = denominator;
		a->parm.capture.extendedmode = 0;
		a->parm.capture.readbuffers = 0;
	} else {
		a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
		a->parm.output.outputmode = 0;
		a->parm.output.timeperframe.numerator = numerator;
		a->parm.output.timeperframe.denominator = denominator;
	}
	return rc;
}

int cvi_isp_s_param(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	int rc = 0;

	return rc;
}

static int cvi_isp_s_selection(struct file *file, void *fh, struct v4l2_selection *sel)
{
	// struct cvi_vi_dev *vdev = video_drvdata(file);
	int rc = -EINVAL;

	switch (sel->target) {

	default:
		return rc;
	}

	vi_pr(VI_INFO, "target(%d) rect(%d %d %d %d)\n", sel->target,
			sel->r.left, sel->r.top, sel->r.width, sel->r.height);
	return rc;
}

static int cvi_isp_enum_fmt_vid_cap(
	struct file *file,
	void  *priv,
	struct v4l2_fmtdesc *f)
{
	vi_pr(VI_INFO, "+\n");
	f->pixelformat = V4L2_PIX_FMT_YUV420;
	// return cvi_vip_enum_fmt_vid(file, priv, f);
	return 0;
}

static char *v4l2_fourcc2s(u32 fourcc, char *buf)
{
	buf[0] = fourcc & 0x7f;
	buf[1] = (fourcc >> 8) & 0x7f;
	buf[2] = (fourcc >> 16) & 0x7f;
	buf[3] = (fourcc >> 24) & 0x7f;
	buf[4] = '\0';
	return buf;
}

static int cvi_isp_g_fmt_vid_cap(
	struct file *file,
	void *priv,
	struct v4l2_format *f)
{
	struct cvi_vi_dev *videv = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	char buf[5];
	u8 chn_id = get_video_index(vdev);
	u8 raw_num = vi_get_raw_num_by_dev(&videv->ctx, chn_id);

	WARN_ON(!vdev);

	pix->width        = videv->ctx.isp_pipe_cfg[raw_num].post_img_w;
	pix->height       = videv->ctx.isp_pipe_cfg[raw_num].post_img_h;
	pix->field        = V4L2_FIELD_NONE;
	pix->pixelformat  = videv->fmt[raw_num]->fourcc;
	pix->colorspace   = videv->pipe[raw_num].colorspace;
	pix->xfer_func    = V4L2_XFER_FUNC_DEFAULT;
	pix->ycbcr_enc    = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix->bytesperline   = videv->pipe[raw_num].bytesperline[0];

	v4l2_fourcc2s(f->fmt.pix.pixelformat, buf);
	vi_pr(VI_INFO, "w_h(%d,%d),fmt:%s\n",
		f->fmt.pix.width, f->fmt.pix.height, buf);

	return 0;
}

static int cvi_isp_g_fmt_vid_cap_mplane(
	struct file *file,
	void *priv,
	struct v4l2_format *f)
{
	struct cvi_vi_dev *videv = video_drvdata(file);
	struct v4l2_pix_format_mplane *mp = &f->fmt.pix_mp;
	struct video_device *vdev = video_devdata(file);
	unsigned int p;
	u8 chn_id = get_video_index(vdev);
	u8 raw_num = vi_get_raw_num_by_dev(&videv->ctx, chn_id);
	vi_pr(VI_INFO, "+\n");
	WARN_ON(!vdev);

	mp->width        = videv->ctx.isp_pipe_cfg[raw_num].post_img_w;
	mp->height       = videv->ctx.isp_pipe_cfg[raw_num].post_img_h;
	mp->field        = V4L2_FIELD_NONE;
	mp->pixelformat  = videv->fmt[raw_num]->fourcc;
	mp->colorspace   = videv->pipe[raw_num].colorspace;
	mp->xfer_func    = V4L2_XFER_FUNC_DEFAULT;
	mp->ycbcr_enc    = V4L2_YCBCR_ENC_DEFAULT;
	mp->quantization = V4L2_QUANTIZATION_DEFAULT;
	mp->num_planes   = videv->fmt[raw_num]->buffers;
	for (p = 0; p < mp->num_planes; p++) {
		mp->plane_fmt[p].bytesperline = videv->pipe[raw_num].bytesperline[p];
		mp->plane_fmt[p].sizeimage = videv->pipe[raw_num].sizeimage[p];
	}

	return 0;
}

static struct cvi_vip_fmt *cvi_vip_get_format(u32 pixelformat)
{
	struct cvi_vip_fmt *fmt;
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(cvi_vip_formats); k++) {
		fmt = &cvi_vip_formats[k];
		if (fmt->fourcc == pixelformat)
			return fmt;
	}

	return NULL;
}

static int cvi_vip_try_fmt_vid_mplane(struct v4l2_format *f, u8 align)
{
	struct v4l2_pix_format_mplane *mp = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *pfmt = mp->plane_fmt;
	struct cvi_vip_fmt *fmt;
	unsigned int bytesperline;
	u8 p;
	char buf[5];

	fmt = cvi_vip_get_format(mp->pixelformat);
	if (!fmt) {
		vi_pr(VI_ERR, "fourcc(%s) unsupport.\n",
			v4l2_fourcc2s(mp->pixelformat, buf));
		return -EINVAL;
	}

	vi_pr(VI_INFO, "size(%d-%d) fourcc(%s)\n", mp->width, mp->height,
					v4l2_fourcc2s(mp->pixelformat, buf));

	mp->field = V4L2_FIELD_NONE;    // progressive only
	mp->width = clamp_val(mp->width, 32, ISP_MAX_WIDTH);
	if (mp->pixelformat == V4L2_PIX_FMT_YUV420M ||
	    mp->pixelformat == V4L2_PIX_FMT_NV21M ||
	    mp->pixelformat == V4L2_PIX_FMT_NV12M)
		mp->height = ALIGN(mp->height, 2);

	// YUV422/420
	mp->width &= ~(fmt->plane_sub_h - 1);
	mp->height &= ~(fmt->plane_sub_v - 1);

	mp->num_planes = fmt->buffers;
	for (p = 0; p < mp->num_planes; p++) {
		u8 plane_sub_v = (p == 0) ? 1 : fmt->plane_sub_v;
		/* Calculate the minimum supported bytesperline value */
		bytesperline = ALIGN((mp->width * fmt->bit_depth[p]) >> 3, align);
		pfmt[p].bytesperline = bytesperline;
		pfmt[p].sizeimage = pfmt[p].bytesperline * mp->height / plane_sub_v;

		vi_pr(VI_INFO, "plane-%d: bytesperline(%d) (%d*%d %d) sizeimage(%d)\n", p,
			pfmt[p].bytesperline, mp->width, mp->height, align, pfmt[p].sizeimage);
		memset(pfmt[p].reserved, 0, sizeof(pfmt[p].reserved));
	}

	mp->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	mp->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	mp->quantization = V4L2_QUANTIZATION_DEFAULT;
	if (mp->pixelformat == V4L2_PIX_FMT_RGB24 ||
	    mp->pixelformat == V4L2_PIX_FMT_BGR24 ||
	    mp->pixelformat == V4L2_PIX_FMT_GREY) {
		mp->colorspace = V4L2_COLORSPACE_SRGB;
	} else if ((mp->pixelformat == V4L2_PIX_FMT_HSV24)) {
		mp->colorspace = V4L2_COLORSPACE_SRGB;
	} else {
		mp->colorspace = V4L2_COLORSPACE_SMPTE170M;
	}
	memset(mp->reserved, 0, sizeof(mp->reserved));

	return 0;
}

static int cvi_isp_try_fmt_vid_cap(
	struct file *file,
	void *priv,
	struct v4l2_format *f)
{
	struct cvi_vi_dev *videv = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	u8 chn_id = get_video_index(vdev);
	u8 raw_num = vi_get_raw_num_by_dev(&videv->ctx, chn_id);
	u32 w, h;
	u32 *crop_w, *crop_h;
	WARN_ON(!videv);

	w = pix->width;
	h = pix->height;
	check_size(&w, &h);

	vi_pr(VI_INFO, "try set fmt:0x%x.\n", pix->pixelformat);

	crop_w = &videv->ctx.isp_pipe_cfg[raw_num].post_img_w;
	crop_h = &videv->ctx.isp_pipe_cfg[raw_num].post_img_h;

	if ((w != *crop_w) || (h != *crop_h)) {
		vi_pr(VI_INFO, "set new crop size(%u*%u), original size(%u*%u).\n",
			w, h, *crop_w, *crop_h);

		*crop_w = w;
		*crop_h = h;

		// if size change ,need to reconfig
		_v4l2_init_config_info(videv, chn_id, raw_num);
	}

	return 0;
}

static int cvi_isp_try_fmt_vid_cap_mplane(
	struct file *file,
	void *priv,
	struct v4l2_format *f)
{
	struct cvi_vi_dev *videv = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);
	struct v4l2_pix_format_mplane *mp = &f->fmt.pix_mp;
	u8 chn_id = get_video_index(vdev);
	u8 raw_num = vi_get_raw_num_by_dev(&videv->ctx, chn_id);
	u32 w, h;
	u32 *crop_w, *crop_h;
	int rc;
	WARN_ON(!videv);

	w = mp->width;
	h = mp->height;
	check_size(&w, &h);

	rc = cvi_vip_try_fmt_vid_mplane(f, VIP_ALIGNMENT);
	if (rc < 0)
		return rc;

	crop_w = &videv->ctx.isp_pipe_cfg[raw_num].post_img_w;
	crop_h = &videv->ctx.isp_pipe_cfg[raw_num].post_img_h;

	if ((w != *crop_w) || (h != *crop_h)) {
		vi_pr(VI_INFO, "set new crop size(%u*%u), original size(%u*%u).\n",
			w, h, *crop_w, *crop_h);

		*crop_w = w;
		*crop_h = h;

		// if size change ,need to reconfig
		_v4l2_init_config_info(videv, chn_id, raw_num);
	}

	return rc;
}

static int cvi_isp_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct cvi_vi_dev *videv = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct cvi_vip_fmt *fmt;
	u8 chn_id = get_video_index(vdev);
	u8 raw_num = vi_get_raw_num_by_dev(&videv->ctx, chn_id);
	int rc;

	WARN_ON(!vdev);

	rc = cvi_isp_try_fmt_vid_cap_mplane(file, priv, f);
	if (rc < 0) {
		vi_pr(VI_ERR, "try set fmt fail!\n");
		return rc;
	}

	fmt = cvi_vip_get_format(pix->pixelformat);
	videv->fmt[raw_num] = fmt;
	videv->pipe[raw_num].colorspace = pix->colorspace;

	calculate_sizeimage(videv, raw_num);

	vi_pr(VI_WARN, "bytesperline:%d, size:%d\n", videv->pipe[raw_num].bytesperline[0],
			videv->pipe[raw_num].sizeimage[0] + videv->pipe[raw_num].sizeimage[1]);

	return rc;
}

static int cvi_isp_s_fmt_vid_cap_mplane(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct cvi_vi_dev *videv = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);
	struct v4l2_pix_format_mplane *mp = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *pfmt = mp->plane_fmt;
	struct cvi_vip_fmt *fmt;
	unsigned int p;
	int rc = 0;
	u8 chn_id = get_video_index(vdev);
	u8 raw_num = vi_get_raw_num_by_dev(&videv->ctx, chn_id);

	WARN_ON(!vdev);
	vi_pr(VI_INFO, "+\n");

	rc = cvi_isp_try_fmt_vid_cap_mplane(file, priv, f);

	if (rc < 0)
		return rc;

	fmt = cvi_vip_get_format(mp->pixelformat);
	videv->fmt[raw_num] = fmt;
	videv->pipe[raw_num].colorspace = mp->colorspace;
	for (p = 0; p < mp->num_planes; p++) {
		videv->pipe[raw_num].bytesperline[p] = pfmt[p].bytesperline;
		videv->pipe[raw_num].sizeimage[p] = pfmt[p].sizeimage;
		vi_pr(VI_WARN, "bytesperline:%d, size:%d\n",
				pfmt[p].bytesperline, pfmt[p].sizeimage);
	}

	return rc;
}

static void _fill_v4l2_buffer(struct vb2_buffer *vb, void *pb)
{
	struct v4l2_buffer *b = pb;
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vb2_queue *q = vb->vb2_queue;
	unsigned int plane;

	/* Copy back data such as timestamp, flags, etc. */
	b->index = vb->index;
	b->type = vb->type;
	b->memory = vb->memory;
	b->bytesused = 0;

	b->flags = vbuf->flags;
	b->field = vbuf->field;
	v4l2_buffer_set_timestamp(b, vb->timestamp);
	b->timecode = vbuf->timecode;
	b->reserved2 = 0;
	b->request_fd = 0;

	if (q->is_multiplanar) {
		/*
		 * Fill in plane-related data if userspace provided an array
		 * for it. The caller has already verified memory and size.
		 */
		b->length = vb->num_planes;
		for (plane = 0; plane < vb->num_planes; ++plane) {
			struct v4l2_plane *pdst = &b->m.planes[plane];
			struct vb2_plane *psrc = &vb->planes[plane];

			pdst->bytesused = psrc->bytesused;
			pdst->length = psrc->length;
			if (q->memory == VB2_MEMORY_MMAP)
				pdst->m.mem_offset = psrc->m.offset;
			else if (q->memory == VB2_MEMORY_USERPTR)
				pdst->m.userptr = psrc->m.userptr;
			else if (q->memory == VB2_MEMORY_DMABUF)
				pdst->m.fd = psrc->m.fd;
			pdst->data_offset = psrc->data_offset;
			memset(pdst->reserved, 0, sizeof(pdst->reserved));
		}
	} else {
		/*
		 * We use length and offset in v4l2_planes array even for
		 * single-planar buffers, but userspace does not.
		 */
		b->length = vb->planes[0].length;
		b->bytesused = vb->planes[0].length;
		if (q->memory == VB2_MEMORY_MMAP)
			b->m.offset = vb->planes[0].m.offset;
		else if (q->memory == VB2_MEMORY_USERPTR)
			b->m.userptr = vb->planes[0].m.userptr;
		else if (q->memory == VB2_MEMORY_DMABUF)
			b->m.fd = vb->planes[0].m.fd;
	}

	switch (vb->state) {
	case VB2_BUF_STATE_QUEUED:
	case VB2_BUF_STATE_ACTIVE:
		b->flags |= V4L2_BUF_FLAG_QUEUED;
		break;
	case VB2_BUF_STATE_IN_REQUEST:
		b->flags |= V4L2_BUF_FLAG_IN_REQUEST;
		break;
	case VB2_BUF_STATE_ERROR:
		b->flags |= V4L2_BUF_FLAG_ERROR;
		fallthrough;
	case VB2_BUF_STATE_DONE:
		b->flags |= V4L2_BUF_FLAG_DONE;
		break;
	case VB2_BUF_STATE_PREPARING:
	case VB2_BUF_STATE_DEQUEUED:
		/* nothing */
		break;
	}

	if ((vb->state == VB2_BUF_STATE_DEQUEUED ||
	     vb->state == VB2_BUF_STATE_IN_REQUEST) &&
	    vb->synced && vb->prepared)
		b->flags |= V4L2_BUF_FLAG_PREPARED;

	if (vb2_buffer_in_use(q, vb))
		b->flags |= V4L2_BUF_FLAG_MAPPED;
	if (vbuf->request_fd >= 0) {
		b->flags |= V4L2_BUF_FLAG_REQUEST_FD;
		b->request_fd = vbuf->request_fd;
	}
}

int cvi_isp_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct cvi_vi_dev *videv = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);
	struct cvi_isp_buf *b = NULL;
	struct vb2_buffer *vb2_buf = NULL;
	struct vb2_queue *vb2_q = NULL;
	struct _isp_dqbuf_n *n = NULL;
	struct timespec64 ts;
	unsigned long flags;
	int ret = 0;
	int index = 0;
	int timeout = 2000; // base timeout ms
	u8 chn_id = get_video_index(vdev);
	u8 raw_num = vi_get_raw_num_by_dev(&videv->ctx, chn_id);

	WARN_ON(!videv);

	vi_pr(VI_DBG, "dev_%d dq pipe_%d start\n", chn_id, raw_num);

	if (gViCtx->bypass_frm[raw_num] > videv->postraw_frame_number[raw_num])
		timeout += gViCtx->bypass_frm[raw_num] * 100;

	ret = wait_event_timeout(
		videv->isp_dq_wait_q[raw_num],
		atomic_read(&videv->post_dq_flag[raw_num]) == 0,
		msecs_to_jiffies(timeout));

	if (!ret) {
		vi_pr(VI_ERR, "dev_%d dqbuf[%d] timeout(%d ms)\n", chn_id,
			videv->postraw_frame_number[raw_num] + 1, timeout);
		return -ETIME;
	}

	atomic_set(&videv->post_dq_flag[raw_num], 1);

	ktime_get_ts64(&ts);
	spin_lock_irqsave(&dq_lock, flags);
	if (!list_empty(&dqbuf_q[raw_num].list)) {
		n = list_first_entry(&dqbuf_q[raw_num].list, struct _isp_dqbuf_n, list);
		p->flags = n->chn_id;
		p->sequence = videv->postraw_frame_number[raw_num];
		p->timestamp.tv_sec	= ts.tv_sec;
		p->timestamp.tv_usec = ts.tv_nsec / 1000;
		list_del_init(&n->list);
		kfree(n);
	} else {
		vi_pr(VI_ERR, "dev_%d dqbuf_q is empty!\n", chn_id);
		spin_unlock_irqrestore(&dq_lock, flags);
		return -1;
	}
	spin_unlock_irqrestore(&dq_lock, flags);

	spin_lock_irqsave(&videv->qbuf_lock, flags);
	if (!list_empty(&videv->dqbuf_list[raw_num])) {
		b = list_first_entry(&videv->dqbuf_list[raw_num], struct cvi_isp_buf, list);
		vb2_buf = &(b->buf.vb2_buf);
		vb2_buf->state = VB2_BUF_STATE_DEQUEUED;
		index = vb2_buf->index;
		vb2_q = vb2_buf->vb2_queue;
		atomic_dec(&vb2_q->owned_by_drv_count);
		list_del_init(&b->list);
	} else {
		vi_pr(VI_ERR, "dev_%d dqbuf is empty!\n", chn_id);
		spin_unlock_irqrestore(&videv->qbuf_lock, flags);
		return -1;
	}
	spin_unlock_irqrestore(&videv->qbuf_lock, flags);

	_fill_v4l2_buffer(vb2_buf, p);

	vi_pr(VI_DBG, "dev_%d dqbuf at postframe[%d] vb_index[%d] done\n",
			chn_id, videv->postraw_frame_number[raw_num], index);

	return 0;
}

int cvi_isp_expbuf(struct file *file, void *priv, struct v4l2_exportbuffer *p)
{
	struct cvi_vi_dev *videv = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);
	struct cvi_isp_buf *cvi_vb, *tmp;
	struct vb2_buffer *vb2_buf = NULL;
	struct mem_mapping *mem_info;
	unsigned long flags;
	uint32_t match_index = 0;
	uint64_t match_addr = 0;
	u8 chn_id = get_video_index(vdev);
	u8 raw_num = vi_get_raw_num_by_dev(&videv->ctx, chn_id);
	int i;

	vi_pr(VI_INFO, "dev_%d export vb_index[%d], qbuf_num(%d)\n",
			chn_id, p->index, videv->qbuf_num[raw_num]);

	spin_lock_irqsave(&videv->qbuf_lock, flags);
	// expbuf only support when target vb buffer queue in qbuf_list
	list_for_each_entry_safe(cvi_vb, tmp, &(videv->qbuf_list[raw_num]), list) {
		vb2_buf = &(cvi_vb->buf.vb2_buf);
		match_index = vb2_buf->index;
		if (match_index == p->index) {
			match_addr = vb2_dma_contig_plane_dma_addr(vb2_buf, 0);
			goto find_fd;
		}
	}
	spin_unlock_irqrestore(&videv->qbuf_lock, flags);

	vi_pr(VI_INFO, "no vb index match\n");

	return -1;

find_fd:
	spin_unlock_irqrestore(&videv->qbuf_lock, flags);

	for (i = 0; i < MAX_VB2_BUF_NUM; i++) {
		mem_info = &vb2_buf_meminfo[i];
		if (mem_info->phy_addr == match_addr) {
			p->fd = mem_info->dmabuf_fd;
			break;
		}
	}

	if (i == MAX_VB2_BUF_NUM) {
		vi_pr(VI_ERR, "no dma addr match buf(%d)! (0x%llx)\n", p->index, match_addr);
		return -1;
	}

	return 0;
}

static int isp_subscribe_event(struct v4l2_fh *fh,
	const struct v4l2_event_subscription *sub)
{
	return v4l2_event_subscribe(fh, sub, CVI_ISP_NEVENTS, NULL);
}

static const struct v4l2_ioctl_ops cvi_isp_ioctl_ops = {
	.vidioc_querycap               = cvi_isp_querycap,
	.vidioc_g_ctrl                 = cvi_isp_g_ctrl,
	.vidioc_s_ctrl                 = cvi_isp_s_ctrl,
	.vidioc_g_input                = cvi_isp_g_input,
	.vidioc_s_input                = cvi_isp_s_input,
	.vidioc_enum_input             = cvi_isp_enum_input,
	.vidioc_g_parm                 = cvi_isp_g_param,
	.vidioc_s_parm                 = cvi_isp_s_param,
	.vidioc_g_ext_ctrls            = cvi_isp_g_ext_ctrls,
	.vidioc_s_ext_ctrls            = cvi_isp_s_ext_ctrls,
	.vidioc_s_selection            = cvi_isp_s_selection,
	.vidioc_enum_fmt_vid_cap       = cvi_isp_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap          = cvi_isp_g_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap_mplane   = cvi_isp_g_fmt_vid_cap_mplane,
	.vidioc_try_fmt_vid_cap        = cvi_isp_try_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap_mplane = cvi_isp_try_fmt_vid_cap_mplane,
	.vidioc_s_fmt_vid_cap          = cvi_isp_s_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap_mplane   = cvi_isp_s_fmt_vid_cap_mplane,
	.vidioc_reqbufs                = vb2_ioctl_reqbufs,
	.vidioc_create_bufs            = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf            = vb2_ioctl_prepare_buf,
	.vidioc_querybuf               = vb2_ioctl_querybuf,
	.vidioc_qbuf                   = vb2_ioctl_qbuf,
	.vidioc_dqbuf                  = cvi_isp_dqbuf,
	.vidioc_expbuf                 = cvi_isp_expbuf,
	.vidioc_streamon               = vb2_ioctl_streamon,
	.vidioc_streamoff              = vb2_ioctl_streamoff,
	.vidioc_subscribe_event        = isp_subscribe_event,
	.vidioc_unsubscribe_event      = v4l2_event_unsubscribe,
};

static int cvi_isp_queue_setup(struct vb2_queue *vq,
			      unsigned int *nbuffers, unsigned int *nplanes,
			      unsigned int sizes[], struct device *alloc_devs[])
{
	struct cvi_vi_dev *videv = vb2_get_drv_priv(vq);
	struct cvi_vdev_node *vnode = container_of(vq, struct cvi_vdev_node, vb_q);
	unsigned int p;
	u8 chn_id = get_video_index(&vnode->vdev);
	u8 raw_num = vi_get_raw_num_by_dev(&videv->ctx, chn_id);
	unsigned int planes = videv->fmt[raw_num]->buffers;

	switch (planes) {
	case 1:
		sizes[0] = videv->pipe[raw_num].sizeimage[0] + videv->pipe[raw_num].sizeimage[1];
		break;
	case 2:
		sizes[0] = videv->pipe[raw_num].sizeimage[0];
		sizes[1] = videv->pipe[raw_num].sizeimage[1];
		break;
	default:
		vi_pr(VI_ERR, "unsupport planes:%d\n", p);
		return -1;
	}

	*nplanes = planes;

	for (p = 0; p < *nplanes; p++)
		vi_pr(VI_INFO, "plane[%u] size=%u\n", p, sizes[p]);

	return 0;
}


static void cvi_isp_buf_queue_vb2(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vb2_queue *vq = vb->vb2_queue;
	struct cvi_vi_dev *videv = vb2_get_drv_priv(vq);
	struct cvi_isp_buf *qbuf = container_of(vbuf, struct cvi_isp_buf, buf);
	struct cvi_vdev_node *vnode = container_of(vq, struct cvi_vdev_node, vb_q);
	u8 chn_id = get_video_index(&vnode->vdev);
	u8 raw_num = vi_get_raw_num_by_dev(&videv->ctx, chn_id);
	WARN_ON(!videv);

	cvi_isp_rdy_buf_queue(videv, qbuf, raw_num);

	vi_pr(VI_DBG, "dev_%d queue vb[%d] at pipe_%d, total_buf_num[%d]\n",
			chn_id, vb->index, raw_num, videv->qbuf_num[raw_num]);
}

static int cvi_isp_buf_prepare(struct vb2_buffer *vb)
{
	// struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	if (vb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT &&
		vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0)) {
		vi_pr(VI_ERR, "Bytes used out of bounds.\n");
		return -EINVAL;
	}

	return 0;
}

static int cvi_isp_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct cvi_vi_dev *videv = vb2_get_drv_priv(vq);
	struct cvi_vdev_node *vnode = container_of(vq, struct cvi_vdev_node, vb_q);
	u8 chn_id = get_video_index(&vnode->vdev);
	u8 raw_num = vi_get_raw_num_by_dev(&videv->ctx, chn_id);
	int rc = 0;
	int i;
	WARN_ON(!videv);

	atomic_set(&videv->is_streaming[raw_num], 1);

	mutex_lock(&videv->stream_lock);

	atomic_inc(&videv->isp_streamon);

	if (atomic_read(&videv->isp_streamon) > 1) {
		vi_pr(VI_INFO, "dev_%d already streaming\n", chn_id);
		mutex_unlock(&videv->stream_lock);
		return rc;
	}

	vi_pr(VI_INFO, "first_usr [dev_%d] begin to start stream\n", chn_id);

	rc = vi_create_thread(videv, E_VI_TH_EVENT_HANDLER);
	if (rc) {
		vi_pr(VI_ERR, "Failed to create VI_EVENT_HANDLER thread\n");
	}

	for (i = ISP_PRERAW0; i < ISP_PRERAW_MAX; i++) {
		if (!videv->ctx.isp_pipe_enable[i])
			continue;
		subcall_get_sensor_mode(videv, i);
	}

	vi_mempool_alloc_ion(videv);

	if (!videv->ctx.isp_pipe_cfg[raw_num].is_patgen_en)
		subcall_s_stream(videv, true);

	vi_start_streaming(videv);

	mutex_unlock(&videv->stream_lock);

	return rc;
}

static void cvi_isp_stop_streaming(struct vb2_queue *vq)
{
	struct cvi_vi_dev *videv = vb2_get_drv_priv(vq);
	struct cvi_vdev_node *vnode = container_of(vq, struct cvi_vdev_node, vb_q);
	u8 chn_id = get_video_index(&vnode->vdev);
	u8 raw_num = vi_get_raw_num_by_dev(&videv->ctx, chn_id);
	u8 stream_usr;
	WARN_ON(!videv);

	atomic_set(&videv->is_streaming[raw_num], 0);

	vi_dq_vb_buffer(videv, raw_num);

	mutex_lock(&videv->stream_lock);

	stream_usr = atomic_read(&videv->isp_streamon);
	if (stream_usr > 1) {
		atomic_dec(&videv->isp_streamon);
		vi_pr(VI_INFO, "wait(dev_%d)! there's %d stream_usr\n", chn_id, stream_usr);
		mutex_unlock(&videv->stream_lock);
		return;
	}

	vi_pr(VI_INFO, "last_usr [dev_%d] begin to stop stream\n", chn_id);

	vi_destory_thread(videv, E_VI_TH_EVENT_HANDLER);

	vi_stop_streaming(videv);

	if (!videv->ctx.isp_pipe_cfg[raw_num].is_patgen_en)
		subcall_s_stream(videv, false);

	vi_mempool_free_ion(videv);

	atomic_set(&videv->isp_streamon, 0);

	mutex_unlock(&videv->stream_lock);
}

const struct vb2_ops cvi_isp_qops = {
	.queue_setup        = cvi_isp_queue_setup,
	.buf_queue          = cvi_isp_buf_queue_vb2,
	.buf_prepare        = cvi_isp_buf_prepare,
	.wait_prepare       = vb2_ops_wait_prepare,
	.wait_finish        = vb2_ops_wait_finish,
	.start_streaming    = cvi_isp_start_streaming,
	.stop_streaming     = cvi_isp_stop_streaming,
};

/*************************************************************************
 *	FOPS definition
 *************************************************************************/
static int cvi_isp_open(struct file *file)
{
	int rc = 0;
	struct cvi_vi_dev *videv = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);
	struct cvi_isp_device *dev =
		container_of(videv, struct cvi_isp_device, vi_dev);
	u8 chn_id = get_video_index(vdev);
	u8 raw_num = ISP_PRERAW_MAX;
	u8 open_cnt;
	u8 file_open_cnt;
	int i;

	WARN_ON(!videv);

	mutex_lock(&videv->dev_lock);

	atomic_inc(&videv->file_open_cnt[chn_id]);
	file_open_cnt = atomic_read(&videv->file_open_cnt[chn_id]);
	file->private_data = vdev;

	if (file_open_cnt > 1) {
		rc = v4l2_fh_open(file);
		vi_pr(VI_INFO, "video%d file open cnt(%d)\n", chn_id, file_open_cnt);
		mutex_unlock(&videv->dev_lock);
		return rc;
	}

	atomic_inc(&videv->open_dev_cnt);
	open_cnt = atomic_read(&videv->open_dev_cnt);

	vi_pr(VI_INFO, "open video%d, dev_cnt(%d)\n", chn_id, open_cnt);

	if (open_cnt == 1) {
		subcall_open(videv, true);

		_vi_sw_init(videv);

#ifndef FPGA_PORTING
		_vi_clk_ctrl(videv, true);
#endif

		for (i = 0; i < MAX_SENSOR_NUM; i++) {
			if (!dev->sensors[i].sd)
				continue;

			gViCtx->isDevEnable[i] = true;
			gViCtx->total_dev_num++;

			_subdev_init(videv, i, &raw_num);

			_vi_set_dev_bind_info(videv, i, raw_num);

			_v4l2_init_config_info(videv, i, raw_num);

			vi_mac_clk_ctrl(videv, raw_num, true);
		}
	}

	rc = v4l2_fh_open(file);

	mutex_unlock(&videv->dev_lock);

	vi_pr(VI_INFO, "-\n");

	return rc;
}

static int cvi_isp_release(struct file *file)
{
	struct cvi_vi_dev *videv = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);
	struct cvi_isp_device *dev =
		container_of(videv, struct cvi_isp_device, vi_dev);
	struct vb2_queue *vb_q;
	u8 chn_id = get_video_index(vdev);
	u8 raw_num;
	u8 file_open_cnt;
	int i;

	mutex_lock(&videv->dev_lock);

	atomic_dec(&videv->file_open_cnt[chn_id]);
	file_open_cnt = atomic_read(&videv->file_open_cnt[chn_id]);

	if (file_open_cnt > 0) {
		vb2_fop_release(file);
		vi_pr(VI_INFO, "video%d wait for cnt(%d) to close\n", chn_id, file_open_cnt);
		mutex_unlock(&videv->dev_lock);
		return 0;
	}

	atomic_dec(&videv->open_dev_cnt);

	vi_pr(VI_INFO, "release video%d\n", chn_id);

	if (atomic_read(&videv->open_dev_cnt) == 0) {
#ifndef FPGA_PORTING
		_vi_clk_ctrl(videv, false);
#endif
		_vi_sdk_release(videv);

		gViCtx->total_dev_num = 0;

		for (i = 0; i < MAX_SENSOR_NUM; i++) {
			if (!dev->sensors[i].sd)
				continue;
			vb_q = &videv->vnode[i].vb_q;
			if (vb2_is_streaming(vb_q))
				vb2_streamoff(vb_q, vb_q->type);
			gViCtx->isDevEnable[i] = false;
			raw_num = vi_get_raw_num_by_dev(&videv->ctx, i);
			vi_mac_clk_ctrl(videv, raw_num, false);
			vi_pr(VI_INFO, "close vi_%d, pipe_%d\n", i, raw_num);
		}

		subcall_open(videv, false);
	}

	vb2_fop_release(file);

	mutex_unlock(&videv->dev_lock);

	vi_pr(VI_INFO, "-\n");

	return 0;
}

static __poll_t cvi_isp_poll(struct file *file, poll_table *wait)
{
	struct cvi_vi_dev *videv = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);
	struct vb2_queue *q = vdev->queue;
	struct mutex *lock = q->lock ? q->lock : vdev->lock;
	unsigned long req_events = poll_requested_events(wait);
	unsigned long flags;
	unsigned int res = 0;
	void *fileio;

	/*
	 * If this helper doesn't know how to lock, then you shouldn't be using
	 * it but you should write your own.
	 */
	WARN_ON(!lock);

	fileio = q->fileio;

	if (req_events & POLLPRI) {
		/*
		 * If event buf is not empty, then notify MW to DQ event.
		 * Otherwise poll_wait.
		 */
		spin_lock_irqsave(&event_lock, flags);
		if (!list_empty(&event_q.list))
			res = POLLPRI;
		else
			poll_wait(file, &videv->isp_event_wait_q, wait);
		spin_unlock_irqrestore(&event_lock, flags);
	}

	if (req_events & POLLIN) {
		if (atomic_read(&videv->isp_dbg_flag)) {
			res = POLLIN | POLLRDNORM;
			atomic_set(&videv->isp_dbg_flag, 0);
		} else {
			poll_wait(file, &videv->isp_dbg_wait_q, wait);
		}
	}

	/* If fileio was started, then we have a new queue owner. */
	if (!fileio && q->fileio)
		q->owner = file->private_data;

	return res;
}

static struct v4l2_file_operations cvi_isp_fops = {
	.owner          = THIS_MODULE,
	.open           = cvi_isp_open,
	.release        = cvi_isp_release,
	.poll           = cvi_isp_poll,
	.mmap           = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = video_ioctl2,
#endif
};

static irqreturn_t vi_core_isr(int irq, void *priv)
{
	struct cvi_vi_dev *vdev = priv;

	vi_irq_handler(vdev);

	return IRQ_HANDLED;
}

#ifndef FPGA_PORTING
static int vi_core_clk_init(struct cvi_vi_dev *videv)
{
	u8 i = 0;

	for (i = 0; i < ARRAY_SIZE(clk_sys_name); ++i) {
		videv->clk_sys[i] = devm_clk_get(videv->dev, clk_sys_name[i]);
		if (IS_ERR(videv->clk_sys[i])) {
			dev_err(videv->dev, "Cannot get clk for %s\n", clk_sys_name[i]);
			return PTR_ERR(videv->clk_sys[i]);
		}
	}

	for (i = 0; i < ARRAY_SIZE(clk_isp_name); ++i) {
		videv->clk_isp[i] = devm_clk_get(videv->dev, clk_isp_name[i]);
		if (IS_ERR(videv->clk_isp[i])) {
			dev_err(videv->dev, "Cannot get clk for %s\n", clk_isp_name[i]);
			return PTR_ERR(videv->clk_isp[i]);
		}
	}

	for (i = 0; i < ARRAY_SIZE(clk_mac_name); ++i) {
		videv->clk_mac[i] = devm_clk_get(videv->dev, clk_mac_name[i]);
		if (IS_ERR(videv->clk_mac[i])) {
			dev_err(videv->dev, "Cannot get clk for %s\n", clk_mac_name[i]);
			return PTR_ERR(videv->clk_mac[i]);
		}
	}

	return 0;
}
#endif

#ifdef VI_MEM_BM_ION
static struct dma_map_ops vb2_dma_ops = {
	.alloc = v4l2_vb_dma_alloc,
	.free  = v4l2_vb_dma_free,
	.mmap  = v4l2_vb_dma_mmap,
};
#endif

int vi_core_init(struct platform_device *pdev)
{
	struct cvi_isp_device *isp_dev;
	struct cvi_vi_dev *videv;
	struct resource *res;
	int ret = 0;

	isp_dev = dev_get_drvdata(&pdev->dev);
	if (!isp_dev) {
		vi_pr(VI_ERR, "invalid data\n");
		return -EINVAL;
	}

	videv = &isp_dev->vi_dev;
	videv->dev = &pdev->dev;
	gvidev = videv;
#ifdef VI_MEM_BM_ION
	videv->dev->dma_ops = &vb2_dma_ops;
#endif

	mutex_init(&videv->dev_lock);
	mutex_init(&videv->stream_lock);
	mutex_init(&videv->v4l2_vb_lock);

	/* ISP IP register base address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	videv->reg_base = devm_ioremap_resource(&pdev->dev, res);
	vi_pr(VI_INFO, "res-reg[0]: start: 0x%llx, end: 0x%llx, virt-addr(%px).\n",
			res->start, res->end, videv->reg_base);
	if (IS_ERR(videv->reg_base)) {
		ret = PTR_ERR(videv->reg_base);
		return ret;
	}

	/* SYS IP register base address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);

	sys_base_address = (uintptr_t)devm_ioremap(&pdev->dev, res->start, resource_size(res));
	vi_pr(VI_INFO, "res-reg[1]: start: 0x%llx, end: 0x%llx, virt-addr(%lx).\n",
			res->start, res->end, sys_base_address);

	/* Interrupt */
	videv->irq_num = platform_get_irq_byname(pdev, CVI_ISP_IRQ_NAME);
	if (videv->irq_num < 0) {
		dev_err(&pdev->dev, "No IRQ resource for [%s]\n", CVI_ISP_IRQ_NAME);
		return -ENODEV;
	}
	vi_pr(VI_INFO, "irq(%d) for %s get from platform driver(%s).\n",
			videv->irq_num, CVI_ISP_IRQ_NAME, pdev->name);

#ifndef FPGA_PORTING
	ret = vi_core_clk_init(videv);
	if (ret) {
		dev_err(&pdev->dev, "Failed to init clk, err %d\n", ret);
		return -EINVAL;
	}
#endif

	vi_set_base_addr(videv->reg_base);

	_vi_mempool_reset();

	ret = _vi_create_proc(videv);
	if (ret) {
		vi_pr(VI_ERR, "Failed to create proc\n");
		goto err;
	}

	gViCtx = (struct cvi_vi_ctx *)videv->shared_mem;

	_vi_init_param(videv);

	ret = vi_create_thread(videv, E_VI_TH_PRERAW);
	if (ret) {
		vi_pr(VI_ERR, "Failed to create preraw thread\n");
		goto err;
	}

	ret = vi_create_thread(videv, E_VI_TH_ERR_HANDLER);
	if (ret) {
		vi_pr(VI_ERR, "Failed to create err_handler thread\n");
		goto err;
	}

	ret = vi_create_thread(videv, E_VI_TH_VBLANK_HANDLER);
	if (ret) {
		vi_pr(VI_ERR, "Failed to create vblank_update thread\n");
		goto err;
	}

	ret = devm_request_irq(&pdev->dev, videv->irq_num, vi_core_isr, 0,
				pdev->name, videv);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq_num(%d) ret(%d)\n",
				videv->irq_num, ret);
		ret = -EINVAL;
		goto err;
	}

	// init device dma mask
	dma_set_mask_and_coherent(videv->dev, DMA_BIT_MASK(64));

	vi_pr(VI_INFO, "done\n");

err:

	return ret;
}

int vi_core_deinit(struct cvi_vi_dev *videv)
{
	int ret = 0;
	int i;

	mutex_destroy(&videv->dev_lock);

	_vi_destroy_proc(videv);

	vi_destory_thread(videv, E_VI_TH_PRERAW);
	vi_destory_thread(videv, E_VI_TH_ERR_HANDLER);
	vi_destory_thread(videv, E_VI_TH_VBLANK_HANDLER);

	for (i = 0; i < videv->num_dev; i++) {
		sync_task_exit(i);
		kfree(isp_bufpool[i].fswdr_rpt);
		isp_bufpool[i].fswdr_rpt = 0;
	}

	tasklet_kill(&videv->job_work);

	vi_pr(VI_INFO, "-\n");

	return ret;
}

/*******************************************************
 *  Common interface for core
 ******************************************************/
int vi_register_video_device(struct cvi_isp_device *isp_dev)
{
	int rc = 0;
	struct cvi_vi_dev *videv;
	struct video_device *vfd;
	struct vb2_queue *q;
	struct mutex *vd_lock;
	struct mutex *vq_lock;

	videv = &isp_dev->vi_dev;
	videv->vid_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	q = &videv->vnode[videv->num_dev].vb_q;
	vd_lock = &videv->vnode[videv->num_dev].vlock;
	vq_lock = &videv->vq_lock[videv->num_dev];

	mutex_init(vd_lock);
	mutex_init(vq_lock);

	vfd = &videv->vnode[videv->num_dev].vdev;
	snprintf(vfd->name, sizeof(vfd->name), "cvi-isp%d", videv->num_dev);
	vfd->fops = &cvi_isp_fops;
	vfd->ioctl_ops = &cvi_isp_ioctl_ops;
	vfd->vfl_dir = VFL_DIR_RX;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	vfd->vfl_type = VFL_TYPE_VIDEO;
#else
	vfd->vfl_type = VFL_TYPE_GRABBER;
#endif
	vfd->minor = -1;
	vfd->device_caps = videv->vid_caps;
	vfd->release = video_device_release_empty;
	vfd->v4l2_dev = &isp_dev->v4l2_dev;
	vfd->lock = vd_lock;
	vfd->queue = q;

	// vb2_queue init
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF | VB2_READ;
	q->buf_struct_size = sizeof(struct cvi_isp_buf);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 0;
	q->drv_priv = videv;
	q->dev = isp_dev->v4l2_dev.dev;
	q->ops = &cvi_isp_qops;
	q->lock = vq_lock;
	q->mem_ops = &vb2_dma_contig_memops;
	rc = vb2_queue_init(q);
	if (rc) {
		vi_pr(VI_ERR, "vb2_queue_init failed, ret=%d\n", rc);
		return rc;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	rc = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
#else
	rc = video_register_device(vfd, VFL_TYPE_GRABBER, -1);
#endif
	if (rc) {
		vi_pr(VI_ERR, "Failed to register vi device\n");
		goto err_register;
	}

	video_set_drvdata(vfd, videv);

	vi_pr(VI_INFO, "pipe_%d registered as %s\n",
		videv->num_dev, video_device_node_name(vfd));

	videv->num_dev++;

	return rc;

err_register:
	video_unregister_device(vfd);

	return rc;
}

int vi_destroy_instance(struct cvi_isp_device *isp_dev)
{
	struct cvi_vi_dev *videv = &isp_dev->vi_dev;
	struct video_device *vfd;
	int i;

	vi_core_deinit(videv);

	for (i = 0; i < videv->num_dev; i++) {
		vfd = &videv->vnode[i].vdev;
		mutex_destroy(&videv->vnode[i].vlock);
		mutex_destroy(&videv->vq_lock[i]);
		media_entity_cleanup(&vfd->entity);
		video_unregister_device(vfd);
		video_set_drvdata(vfd, NULL);
	}

	return 0;
}