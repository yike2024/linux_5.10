// SPDX-License-Identifier: GPL-2.0+
//
// Secure OTP implementation
//

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>
#include <linux/arm-smccc.h>

#include <linux/dma-map-ops.h>
#include <linux/dma-direction.h>

#include "secure_otp.h"

#define ERROR(fmt, ...) pr_err(fmt, ##__VA_ARGS__)
#define VERBOSE(fmt, ...) pr_debug(fmt, ##__VA_ARGS__)

#define SYSTEM_OTP_BASE			0x27100000
#define SYSTEM_OTP2_BS          0x0

#define PIF_BS                  0x1100
#define PIF_OTP2_LOCK           (PIF_BS + 0x10)
#define PIF_OTP2_SECRP_PROT_EN  (PIF_BS + 0x20)
#define PIF_OTP2_CDE_SECRP      (PIF_BS + 0x24)
#define PIF_OTP2_LVL2LCK        (PIF_BS + 0x5C)

#define CFG_BS                  0x1280
#define CFG_INTERRUPT           (CFG_BS + 0x10)
#define CFG_BUSY                (CFG_BS + 0x20)
#define CFG_PDSTB               (CFG_BS + 0x24)
#define CFG_WAKE_UP_TIME        (CFG_BS + 0x28)
#define CFG_CDE_PSMSK_0         (CFG_BS + 0x60)
#define CFG_CDE_PSMSK_1         (CFG_BS + 0x64)
#define CFG_SEC_RANGE           (CFG_BS + 0x78)
#define CFG_LOCK_WRITE          (CFG_BS + 0x7c)

#define OPTEE_SMC_CALL_CV_OTP3_READ		0x03000006
#define OPTEE_SMC_CALL_CV_OTP3_WRITE	0x03000007

#define DEVICE_NAME  "otp"

static void __iomem *otp_base;
static int otp_major;
static struct class *otp_class;

static inline void mmio_write_32(void __iomem *addr, uint32_t value)
{
	iowrite32(value, addr);
}

static inline uint32_t mmio_read_32(void __iomem *addr)
{
	return ioread32(addr);
}

static inline void mmio_setbits_32(void __iomem *addr, uint32_t set)
{
	mmio_write_32(addr, mmio_read_32(addr) | set);
}

static inline void cvi_otp_enter_sleep_mode(void)
{
	uint32_t value;

	value = mmio_read_32(otp_base + CFG_PDSTB);
	if ((value & 0x1) == 1) {
		mmio_write_32(otp_base + CFG_PDSTB, 0);
		while ((value = mmio_read_32(otp_base + CFG_BUSY)) & 0xC);
	} else {
		pr_info("otp is already sleep mode\n");
	}
}

static inline void cvi_otp_enter_active_mode(void)
{
	uint32_t value;

	value = mmio_read_32(otp_base + CFG_PDSTB);
	if ((value & 0x1) == 0) {
		mmio_write_32(otp_base + CFG_PDSTB, 1);
		while (((value = mmio_read_32(otp_base + CFG_BUSY)) & 0x1));
	} else {
		pr_info("otp is already active mode\n");
	}
}

static inline void cvi_otp_power_switch(int on)
{
	if (on) {
		cvi_otp_enter_active_mode();
	} else {
		cvi_otp_enter_sleep_mode();
	}
}

static inline uint32_t otp2_segment_read(uint32_t segment, uint32_t addr)
{
	pr_info("otp2 read 0x%lx \n", (SYSTEM_OTP_BASE + SYSTEM_OTP2_BS + (((segment << 5) + addr) << 2)));
	return mmio_read_32(otp_base + SYSTEM_OTP2_BS + (((segment << 5) + addr) << 2));
}

static void otp2_segment_dump(uint32_t segment)
{
	const uint32_t size = 32; // one segment size
	uint32_t buf[size];
	uint32_t i = 0;

	for (; i < size; i++) {
		buf[i] = otp2_segment_read(segment, i);
	}
}

static inline void otp2_segment_addr_program(uint32_t segment, uint32_t addr, uint32_t value)
{
	pr_info("otp2 program 0x%x : 0x%x\n", (SYSTEM_OTP_BASE + SYSTEM_OTP2_BS + (((segment << 5) + addr) << 2)), value);
	mmio_write_32(otp_base + SYSTEM_OTP2_BS + (((segment << 5) + addr) << 2), value);
}

static void otp2_segment_program(uint32_t segment, uint32_t *value, uint32_t size)
{
	uint32_t i = 0;

	if (size > 32) 
		size = 32;

	for (;i < size; i++) {
		otp2_segment_addr_program(segment, i, value[i]);
	}
}

#if defined(CONFIG_ARM) || defined(__arm__) || defined(__aarch64__)
static int otp3_read(uint32_t segment, uint32_t addr, uint32_t size, uint32_t *buffer)
{
	void *value = NULL;
	struct arm_smccc_res res = {0};
	phys_addr_t value_phys;

	pr_info("segment[%d] addr[%d] size[%d]\n", segment, addr, size);

	value = kzalloc((size << 2), GFP_KERNEL);
	if (!value) {
		return -ENOMEM;
	}

	value_phys = virt_to_phys(value);

	arm_smccc_smc(OPTEE_SMC_CALL_CV_OTP3_READ, segment, addr, size, (unsigned long)value_phys, 0, 0, 0, &res);
	if (res.a0 < 0) {
		ERROR("smc OPTEE_SMC_CALL_CV_OTP3_READ failed\n");
		kfree(value);
		return -1;
	}

	arch_sync_dma_for_cpu(value_phys, (size << 2), DMA_FROM_DEVICE);
	memcpy(buffer, value, (size << 2));
	kfree(value);

	return 0;
}

static int otp3_write(uint32_t segment, uint32_t addr, uint32_t size, uint32_t *buffer)
{
	struct arm_smccc_res res = {0};
	phys_addr_t value_phys;
	void *value = NULL;

	value = kzalloc((size << 2), GFP_KERNEL);
	if (!value) {
		return -ENOMEM;
	}

	memcpy(value, (void *)buffer, (size << 2));
	__flush_dcache_area(value, (size << 2));
	value_phys = virt_to_phys(value);

	arm_smccc_smc(OPTEE_SMC_CALL_CV_OTP3_WRITE, segment, addr, size, (unsigned long)value_phys, 0, 0, 0, &res);
	if (res.a0 < 0) {
		ERROR("smc OPTEE_SMC_CALL_CV_OTP3_WRITE failed\n");
		kfree(value);
		return -1;
	}

	kfree(value);

	return 0;
}
#else
static int otp3_read(uint32_t segment, uint32_t addr, uint32_t size, uint32_t *buffer)
{
    return 0;
}

static int otp3_write(uint32_t segment, uint32_t addr, uint32_t size, uint32_t *buffer)
{
    return 0;
}
#endif

static inline int otp3_segment_read(uint32_t segment, uint32_t *buffer)
{
	return otp3_read(segment, 0, 32, buffer);
}

static ssize_t otp3_reserve_segment2_show(struct class *class,
                                      struct class_attribute *attr, char *buf)
{
	cvi_otp_power_switch(1);
	otp3_segment_read(2, (uint32_t *)buf);
	cvi_otp_power_switch(0);

	return 128;
}

static ssize_t otp3_reserve_segment2_store(struct class *class,
                                       struct class_attribute *attr,
                                       const char *buf, size_t count)
{
	uint32_t addr = 0, value = 0;

	if (sscanf(buf, "0x%x=0x%x", &addr, &value) != 2)
		return -ENOMEM;

	pr_info("addr=%x value=%x\n", addr, value);
	if (addr > 31)
		return -1;

	cvi_otp_power_switch(1);
	otp3_write(2, addr, 1, &value);
	cvi_otp_power_switch(0);

	return count;
}

static ssize_t otp3_reserve_segment3_show(struct class *class,
                                      struct class_attribute *attr, char *buf)
{
	cvi_otp_power_switch(1);
	otp3_segment_read(3, (uint32_t *)buf);
	cvi_otp_power_switch(0);

	return 128;
}

static ssize_t otp3_reserve_segment3_store(struct class *class,
                                       struct class_attribute *attr,
                                       const char *buf, size_t count)
{
	uint32_t addr = 0, value = 0;

	if (sscanf(buf, "0x%x=0x%x", &addr, &value) != 2)
		return -ENOMEM;

	pr_info("addr=%x value=%x\n", addr, value);
	if (addr > 31)
		return -1;

	cvi_otp_power_switch(1);
	otp3_write(3, addr, 1, &value);
	cvi_otp_power_switch(0);

	return count;
}

static ssize_t otp2_reserve_segment0_show(struct class *class,
                                      struct class_attribute *attr, char *buf)
{
	uint32_t value[32] = {0};
	int i;
	//UNUSED(class);
	//UNUSED(attr);

	cvi_otp_power_switch(1);
	for (i = 0; i < 32; i++) {
		value[i] = otp2_segment_read(0, i);
	}
	cvi_otp_power_switch(0);

	memcpy(buf, (void *)value, sizeof(value));

	return sizeof(value);
}

static ssize_t otp2_reserve_segment0_store(struct class *class,
                                       struct class_attribute *attr,
                                       const char *buf, size_t count)
{
	uint32_t addr = 0, value = 0;

	if (sscanf(buf, "0x%x=0x%x", &addr, &value) != 2)
		return -ENOMEM;

	pr_info("addr=%x value=%x\n", addr, value);
	if (addr > 31)
		return -1;

	cvi_otp_power_switch(1);
	otp2_segment_addr_program(0, addr, value);
	cvi_otp_power_switch(0);

	return count;
}

CLASS_ATTR_RW(otp2_reserve_segment0);
CLASS_ATTR_RW(otp3_reserve_segment2);
CLASS_ATTR_RW(otp3_reserve_segment3);

static int otp_open(struct inode *inode, struct file *file)
{
	cvi_otp_power_switch(1);

	return 0;
}

static ssize_t otp_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

static ssize_t otp_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

static int otp_release(struct inode *inode, struct file *file)
{
	cvi_otp_power_switch(0);

	return 0;
}

static long otp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct otp_config config;
	int ret;

	switch (cmd) {
	case IOCTL_OTP_VERSION: {
		uint32_t version = mmio_read_32(otp_base + CFG_BS);

		ret = copy_to_user((unsigned char *)arg, (unsigned char *)&version, sizeof(version));
		if (ret != 0)
			return -1;

		break;
	}
	case IOCTL_OTP2_READ_CONFIG: {
		int i;

		ret = copy_from_user((unsigned char *)&config, (unsigned char *)arg, sizeof(config));
		if (ret != 0)
			return -1;

		if (config.segment > 27)
			return -1;

		if (config.addr > 31)
			return -1;

		if ((config.addr + config.size) > 32) {
			config.size = 32 - config.addr;
		}

		for (i = 0; i < config.size; i++) {
			config.value[i] = otp2_segment_read(config.segment, (config.addr + i));
		}

		ret = copy_to_user((unsigned char *)arg, (unsigned char *)&config, sizeof(config));
		if (ret != 0)
			return -1;

		break;
	}
	case IOCTL_OTP2_WRITE_CONFIG: {
		int i;

		ret = copy_from_user((unsigned char *)&config, (unsigned char *)arg, sizeof(config));
		if (ret != 0)
			return -1;

		if (config.segment > 27)
			return -1;

		if (config.addr > 31)
			return -1;

		if ((config.addr + config.size) > 32) {
			config.size = 32 - config.addr;
		}

		for (i = 0; i < config.size; i++) {
			otp2_segment_addr_program(config.segment, (config.addr + i), config.value[i]);
		}

		break;
	}
	case IOCTL_OTP3_READ_CONFIG: {
		ret = copy_from_user((unsigned char *)&config, (unsigned char *)arg, sizeof(config));
		if (ret != 0)
			return -1;

		if (config.segment > 27)
			return -1;

		if (config.addr > 31)
			return -1;
		
		if ((config.addr + config.size) > 32) {
			config.size = 32 - config.addr;
		}

		otp3_read(config.segment, config.addr, config.size, config.value);

		ret = copy_to_user((unsigned char *)arg, (unsigned char *)&config, sizeof(config));
		if (ret != 0)
			return -1;

		break;
	}
	case IOCTL_OTP3_WRITE_CONFIG: {
		ret = copy_from_user((unsigned char *)&config, (unsigned char *)arg, sizeof(config));
		if (ret != 0)
			return -1;

		if (config.segment > 27)
			return -1;

		if (config.addr > 31)
			return -1;

		if ((config.addr + config.size) > 32) {
			config.size = 32 - config.addr;
		}

		otp3_write(config.segment, config.addr, config.size, config.value);

		break;
	}
	}

	return 0;
}

const struct file_operations otp_fops = {
	.owner  =   THIS_MODULE,
	.open   =   otp_open,
	.read   =   otp_read,
	.write  =   otp_write,
	.release =  otp_release,
	.unlocked_ioctl = otp_ioctl,
};

int __init cvi_otp_init(void)
{
	int ret;

	otp_base = ioremap(SYSTEM_OTP_BASE, 0x2000);
	if (otp_base == NULL)
		return -ENOMEM;

	// No need op
	//mmio_setbits_32(otp_base + CFG_WAKE_UP_TIME, 0x3);

	otp_major = register_chrdev(0, DEVICE_NAME, &otp_fops);
	if (otp_major < 0) {
		pr_err("can't register major number\n");
		goto failed;
	}

	otp_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(otp_class)) {
		pr_err("Err: failed in My Driver class.\n");
		goto failed;
	}

	ret = class_create_file(otp_class, &class_attr_otp3_reserve_segment2);
	if (ret) {
		pr_err("base: can't create sysfs otp3_reserve_segment0 file\n");
		goto failed;
	}

	ret = class_create_file(otp_class, &class_attr_otp3_reserve_segment3);
	if (ret) {
		pr_err("base: can't create sysfs otp3_reserve_segment0 file\n");
		goto failed;
	}

	ret = class_create_file(otp_class, &class_attr_otp2_reserve_segment0);
	if (ret) {
		pr_err("base: can't create sysfs otp3_reserve_segment0 file\n");
		goto failed;
	}

	device_create(otp_class, NULL, MKDEV(otp_major, 0), NULL, DEVICE_NAME);

	return 0;
failed:
	if (otp_major) {
		unregister_chrdev(otp_major, DEVICE_NAME);
	}

	class_remove_file(otp_class, &class_attr_otp2_reserve_segment0);
	class_remove_file(otp_class, &class_attr_otp3_reserve_segment2);
	class_remove_file(otp_class, &class_attr_otp3_reserve_segment3);
	class_destroy(otp_class);

	return -1;
}

void __exit cvi_otp_exit(void)
{
	iounmap(otp_base);

	class_remove_file(otp_class, &class_attr_otp2_reserve_segment0);
	class_remove_file(otp_class, &class_attr_otp3_reserve_segment2);
	class_remove_file(otp_class, &class_attr_otp3_reserve_segment3);

	device_destroy(otp_class, MKDEV(otp_major, 0));
	class_destroy(otp_class);
	unregister_chrdev(otp_major, DEVICE_NAME);
}

module_init(cvi_otp_init);
module_exit(cvi_otp_exit);

MODULE_AUTHOR("leon.liao@sophgo.com");
MODULE_DESCRIPTION("secure otp driver");
MODULE_LICENSE("GPL");
