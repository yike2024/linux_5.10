/*
 * PCIe host controller driver for athena2_pcie SoCs
 *
 * athena2_pcie PCIe RC Source Code
 *
 * Copyright (C) 2019-2025 Bitmain
 * Jibin Xu <jibin.xu@bitmain.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pci.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include "pcie-designware.h"

#define dw_pcie_readl_rc(pci, reg) dw_pcie_read_dbi((pci), (reg), 4)
#define dw_pcie_writel_rc(pci, reg, val) dw_pcie_write_dbi((pci), (reg), 4, (val))

#define PCIE_TOP_APB_BASE 0x20be0000
#define PCIE_MODE_BASE 0x28100004

#define PCIE_MSI_CTRL_REG     0x010
#define PCIE_INT_STATUS_REG   0x1fc
#define PCIE_RC_CFG_WRITE_REG 0x8bc
#define PCIE_PRST_REG         0x04C
#define PCIE_MSI_EN           (1 << 24)
#define PCIE_MSI_INT          (1 << 8)
#define DBI_RESET_BIT         2
#define RC_PRST_X_IN_BIT      23
#define to_athena2_pcie(x)     dev_get_drvdata((x)->dev)

/* pcie ctrl intf */
#define PCIE_LTSSM_REG        0x0058
#define PCIE_LTSSM_EN_BIT     BIT(0)
#define PCIE_CORE_REG         0x005c
#define CORE_RESET_BIT        8
#define PCIE_CORE_RST_BIT     BIT(8)
#define PCIE_CORE_TYPE_MASK   (0x3 << 4)
//#define PCIE_RDLH_LINKUP_REG  0x0374
//#define PCIE_SMLH_LINKUP_REG  0x0380
#define PCIE_LINKUP_REG  0x00b4
#define PCIE_RDLH_LINKUP_BIT  BIT(7)
#define PCIE_SMLH_LINKUP_BIT  BIT(6)

#define PCIE_SEL_0 BIT(0)
#define PCIE_SEL_1 BIT(1)

struct athena2_pcie {
	struct dw_pcie *pci; /* DT dbi is pci.dbi_base */
	void __iomem *apb_base; /* snps apb base */
	void __iomem *sii_base; /* sii base */
	void __iomem *top_apb_base; /* athena2 top apb base */
	struct clk *clk;
	int reset_gpio;
};

static int athena2_pcie_reset(struct athena2_pcie *athena2_pcie, unsigned int ms)
{
	gpio_set_value(athena2_pcie->reset_gpio, 0);
	msleep(ms);
	gpio_set_value(athena2_pcie->reset_gpio, 1);
	return 0;
}

static int athena2_pcie_wait_core_rstn(struct athena2_pcie *athena2_pcie, int timeout_ms)
{
	unsigned int val;
	int ms = 0;

	do {
		val = readl(athena2_pcie->sii_base + PCIE_CORE_REG);
		val &= (1 << CORE_RESET_BIT);
		if (val)
			return 0;
		ms++;
		mdelay(1);
	} while (!val && ms < timeout_ms);

	return -1;
}

static int athena2_pcie_wait_core_clk(struct athena2_pcie *athena2_pcie, int timeout_ms)
{
	struct dw_pcie *pci = athena2_pcie->pci;
	unsigned int val;
	int ms = 0;

	do {
		val = readl(pci->dbi_base + 0x258);
		val &= (0xffff << 16);
		if (!val)
			return 0;
		ms++;
		mdelay(1);
	} while (val && ms < timeout_ms);

	return -1;
}

static int athena2_pcie_establish_link(struct athena2_pcie *athena2_pcie)
{
	unsigned int val;
	struct dw_pcie *pci = athena2_pcie->pci;
	struct pcie_port *pp = &pci->pp;

	/* enable rc configuration write */
	val = dw_pcie_readl_rc(pci, PCIE_RC_CFG_WRITE_REG);
	val |= 0x1;
	dw_pcie_writel_rc(pci, PCIE_RC_CFG_WRITE_REG, val);

	dw_pcie_setup_rc(pp);

	/* disable RC dl scale feature */
	val = dw_pcie_readl_rc(pci, 0x2d4);
	val &= ~(1 << 31);
	dw_pcie_writel_rc(pci, 0x2d4, val);

	/* fast link for dual chip simulation*/
	//val = dw_pcie_readl_rc(pci, 0x710);
	//val |= (0x1 << 7);
	//dw_pcie_writel_rc(pci, 0x710, val);
	//printk("fast link mode: %x\n", dw_pcie_readl_rc(pci, 0x710));
	//val = dw_pcie_readl_rc(pci, 0x718);
	//val &= ~(0x3 << 29);
	//dw_pcie_writel_rc(pci, 0x718, val);
	//printk("fast link mode: %x\n", dw_pcie_readl_rc(pci, 0x718));

	/* pcie Gen3 */
	val = dw_pcie_readl_rc(pci, 0xa0);
	val = (val & 0xfffffff0) | 0x3;
	dw_pcie_writel_rc(pci, 0xa0, val);

	/* disable rc configuration write */
	val = dw_pcie_readl_rc(pci, PCIE_RC_CFG_WRITE_REG);
	val &= ~0x1;
	dw_pcie_writel_rc(pci, PCIE_RC_CFG_WRITE_REG, val);

	/* enable ltssm */
	val = readl(athena2_pcie->sii_base + PCIE_LTSSM_REG);
	val |= PCIE_LTSSM_EN_BIT;
	writel(val, athena2_pcie->sii_base + PCIE_LTSSM_REG);

	return dw_pcie_wait_for_link(pci);
}

#if 0
static irqreturn_t athena2_pcie_irq_handler(int irq, void *arg)
{
	struct athena2_pcie *athena2_pcie = arg;
	struct dw_pcie *pci = athena2_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	unsigned int status;

	status = readl(athena2_pcie->top_apb_base + PCIE_INT_STATUS_REG);
	if (status & PCIE_MSI_INT) {
		WARN_ON(!IS_ENABLED(CONFIG_PCI_MSI));
		dw_handle_msi_irq(pp);
	}

	return IRQ_HANDLED;
}
#endif

/* static int dw_pcie_wr_own_conf(struct pcie_port *pp, int where, int size,
			       u32 val)
{
	struct dw_pcie *pci;

	if (pp->ops->wr_own_conf)
		return pp->ops->wr_own_conf(pp, where, size, val);

	pci = to_dw_pcie_from_pp(pp);
	return dw_pcie_write(pci->dbi_base + where, size, val);
} */

static void athena2_pcie_msi_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	u64 msi_target;

	/*
	 * for compatibility with devices which do not support
	 * 64bit MSI address. the original dw_pcie_msi_init
	 * function would alloc a page and use its address as MSI
	 * address. on BM168x, this will always produce an address
	 * above 32bit, as DDRs are all there.
	 */
	pp->msi_data = 0x10000000;
	msi_target = (u64)pp->msi_data;

	/* Program the msi_data */
	dw_pcie_writel_dbi(pci, PCIE_MSI_ADDR_LO, lower_32_bits(msi_target));
	dw_pcie_writel_dbi(pci, PCIE_MSI_ADDR_HI, upper_32_bits(msi_target));
}

static void athena2_pcie_enable_interrupts(struct athena2_pcie *athena2_pcie)
{
	struct dw_pcie *pci = athena2_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	unsigned int val;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		athena2_pcie_msi_init(pp);
		val = readl(athena2_pcie->apb_base + PCIE_MSI_CTRL_REG);
		val |= PCIE_MSI_EN;
		writel(val, athena2_pcie->apb_base + PCIE_MSI_CTRL_REG);
	}
}

static void athena2_pcie_disable_interrupts(struct athena2_pcie *athena2_pcie)
{
	unsigned int val;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		val = readl(athena2_pcie->apb_base + PCIE_MSI_CTRL_REG);
		val &= ~PCIE_MSI_EN;
		writel(val, athena2_pcie->apb_base + PCIE_MSI_CTRL_REG);
	}
}

static int athena2_pcie_link_up(struct dw_pcie *pci)
{
	unsigned int status;
	struct athena2_pcie *athena2_pcie = to_athena2_pcie(pci);

	status = readl(athena2_pcie->sii_base + PCIE_LINKUP_REG);
	if ((status&PCIE_SMLH_LINKUP_BIT) == 0) {
		dev_err(pci->dev, "smlh linkup state(0x%x)\n", status);
		return 0;
	}
	//status = readl(athena2_pcie->apb_base + PCIE_RDLH_LINKUP_REG);
	if ((status&PCIE_RDLH_LINKUP_BIT) == 0) {
		dev_err(pci->dev, "rdlh linkup state(0x%x)\n", status);
		return 0;
	}
	return 1;
}

static int athena2_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct athena2_pcie *athena2_pcie = to_athena2_pcie(pci);

	athena2_pcie_establish_link(athena2_pcie);
	athena2_pcie_enable_interrupts(athena2_pcie);

	return 0;
}

static struct dw_pcie_host_ops athena2_pcie_host_ops = {
	.host_init = athena2_pcie_host_init,
};

static int athena2_pcie_add_pcie_port(struct athena2_pcie *athena2_pcie,
		struct platform_device *pdev)
{
	struct dw_pcie *pci = athena2_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = pci->dev;
	int ret;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq_byname(pdev, "msi");
		if (pp->msi_irq <= 0) {
			dev_err(dev, "failed to get MSI irq\n");
			return -ENODEV;
		}
		/*
		 * now we have irq_set_chained_handler_and_data in dw_pcie_host_init,
		 * no need to request irq here.
		 */
		#if 0
		ret = devm_request_irq(dev, pp->msi_irq,
				athena2_pcie_irq_handler,
				IRQF_SHARED | IRQF_NO_THREAD,
				"athena2-pcie-irq", athena2_pcie);
		if (ret) {
			dev_err(dev, "failed to request MSI irq\n");
			return ret;
		}
		#endif
	}

	//pp->root_bus_nr = -1;
	pp->ops = &athena2_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static void athena2_pcie_sram_init(struct athena2_pcie *athena2_pcie,
                unsigned int index)
{
	int timeout = 100;
	uint32_t flag = (index == 0) ? 0x800 : 0x20;
	uint32_t value;
	while (!(readl(athena2_pcie->top_apb_base + 0x4b8) & flag)) {
		if (timeout-- > 0) {
			mdelay(1);
		} else {
			pr_err("PCIE: sram init timeout\n");
			break;
		}
	}
	if (index == 0x1) {
		value = readl(athena2_pcie->top_apb_base + 0x4b0);
		value |= (0x1 << 27);
		writel(value, athena2_pcie->top_apb_base + 0x4b0);
	} else {
		value = readl(athena2_pcie->top_apb_base + 0x4b4);
		value |= (0x1 << 5);
		writel(value, athena2_pcie->top_apb_base + 0x4b4);
	}

}

static bool athena2_pcie_controller_is_ep_enalbled(unsigned int index)
{
	void __iomem * addr = ioremap(PCIE_MODE_BASE, 0x1);
	uint32_t val = readl(addr) >> 25;

	if (index == 0)
		val &= PCIE_SEL_1;
	else
		val &= PCIE_SEL_0;

	return val ? true : false;
}

static int athena2_pcie_controller_config(struct athena2_pcie *athena2_pcie,
		struct device *dev)
{
	int ret = 0;
	unsigned int val, reset_timeout = 100;
	unsigned int addr, idx = 0;
	//struct dw_pcie *pci = athena2_pcie->pci;
	ret = of_property_read_u32(dev->of_node, "ctrl-index", &idx);
	if (ret) {
		idx = 0x0;
	}

	if (athena2_pcie_controller_is_ep_enalbled(idx)) {
		dev_warn(dev, "PCIe EP Mode\n");
		return -1;
	}

	athena2_pcie->reset_gpio = of_get_named_gpio(dev->of_node, "reset-gpio", 0);
	if (athena2_pcie->reset_gpio < 0) {
		dev_err(dev, "could not get pcie reset gpio\n");
		return -1;
	}

	ret = gpio_request_one(athena2_pcie->reset_gpio, GPIOF_OUT_INIT_LOW, "pcie-reset");
	if (ret) {
		dev_err(dev, "could not request gpio %d failed!\n", athena2_pcie->reset_gpio);
		return ret;
	}

	if (athena2_pcie_reset(athena2_pcie, 300)) {
		dev_err(dev, "unable to reset gpio\n");
		return -1;
	}

	ret = of_property_read_u32(dev->of_node, "pipe-mode", &val);
	if (ret) {
		val = 0x0;
		dev_warn(dev, "could not get pcie pipe mode, use default mode\n");
	}

	/* set pipe mode */
	//b'00 pcie4 x2 && pcie2 x2
	//b'01 pcie4 x4
	//b'10 sata && pcie2 x2
	val |= (readl(athena2_pcie->top_apb_base + 0x44) & 0xfffffffC);
	writel(val, athena2_pcie->top_apb_base + 0x44);

	//phy pwr stable cgf
	if (idx == 0) {
		// pcie2
		writel(readl(athena2_pcie->top_apb_base + 0x4a4) | 0x800, athena2_pcie->top_apb_base + 0x4a4);
		writel(readl(athena2_pcie->top_apb_base + 0x4b8) | 0xc00000, athena2_pcie->top_apb_base + 0x4b8);
	} else {
		// pcie4
		writel(readl(athena2_pcie->top_apb_base + 0x4a4) | 0x400, athena2_pcie->top_apb_base + 0x4a4);
		writel(readl(athena2_pcie->top_apb_base + 0x4b8) | 0x18000, athena2_pcie->top_apb_base + 0x4b8);
	}

	//soft_phy_reset_assert_cfg
	writel((readl(athena2_pcie->apb_base) & 0xfffffffb), athena2_pcie->apb_base);

	//soft_cold_reset_cfg
	writel((readl(athena2_pcie->apb_base) & 0xfffffffe), athena2_pcie->apb_base);
	writel((readl(athena2_pcie->apb_base) | 0x1), athena2_pcie->apb_base);

	// config device_type
	writel((readl(athena2_pcie->sii_base + 0x50) & 0xffffe1ff) | (0x4 << 9), athena2_pcie->sii_base + 0x50);

	//TODO remote config
	ret = of_property_read_u32_index(dev->of_node, "remote-upper", 0, &addr);
	ret |= of_property_read_u32_index(dev->of_node, "remote-upper", 1, &val);
	if (!ret) {
		writel(val, athena2_pcie->top_apb_base + addr);
		dev_info(dev, "pcie remote[0x%x] = 0x%08x\n", addr, val);
	}

	ret = of_property_read_u32_index(dev->of_node, "remote-lower", 0, &addr);
	ret |= of_property_read_u32_index(dev->of_node, "remote-lower", 1, &val);
	if (!ret) {
		writel(val, athena2_pcie->top_apb_base + addr);
		dev_info(dev, "pcie remote[0x%x] = 0x%08x\n", addr, val);
	}

	//app_clk_req_cfg
	writel(readl(athena2_pcie->sii_base + 0x60) | (0x1 << 20), athena2_pcie->sii_base + 0x60);
	//DEBUG
	//writel((readl(pci->dbi_base + 0xa0) & 0xfffffff0) | (0x3), pci->dbi_base + 0xa0);

#ifdef SRAM_BYPASS
	//phy_bypass_cfg
	if (idx == 0) {
		writel(readl(athena2_pcie->top_apb_base + 0x4b4) | (0x1 << 4), athena2_pcie->sii_base + 0x4b4);
	} else {
		writel(readl(athena2_pcie->top_apb_base + 0x4b0) | (0x1 << 26), athena2_pcie->sii_base + 0x4b0);
	}
	//soft_phy_reset_deassert_cfg
	writel((readl(athena2_pcie->apb_base) | 0x4), athena2_pcie->apb_base);
#else
	//soft_phy_reset_deassert_cfg
	writel((readl(athena2_pcie->apb_base) | 0x4), athena2_pcie->apb_base);
	athena2_pcie_sram_init(athena2_pcie, idx);
#endif

	//wait_core_rstn
	ret = athena2_pcie_wait_core_rstn(athena2_pcie, reset_timeout);
	if (ret) {
		dev_err(dev, "polling core reset failed\n");
		return ret;
	}

	ret = athena2_pcie_wait_core_clk(athena2_pcie, reset_timeout);
	if (ret) {
		dev_err(dev, "core clock not ready\n");
		return ret;
	}
	return 0;
}

static const struct dw_pcie_ops dw_pcie_ops = {
	.link_up = athena2_pcie_link_up,
};

static u64 pci_dma_mask = DMA_BIT_MASK(45); //TBD

void __athena2_pcie_remove(struct athena2_pcie * athena2_pcie) {
	athena2_pcie_disable_interrupts(athena2_pcie);
	if (athena2_pcie->reset_gpio >= 0)
		gpio_set_value(athena2_pcie->reset_gpio, 0);
	if (athena2_pcie->clk)
		clk_disable_unprepare(athena2_pcie->clk);
}

static int athena2_pcie_remove(struct platform_device *pdev)
{
	struct athena2_pcie *athena2_pcie = platform_get_drvdata(pdev);
	struct dw_pcie *pci = athena2_pcie->pci;
	struct pcie_port *pp = &pci->pp;

	dw_pcie_host_deinit(pp);
	__athena2_pcie_remove(athena2_pcie);

	return 0;
}

static int athena2_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct athena2_pcie *athena2_pcie;
	struct dw_pcie *pci;
	int ret;//, bin_type = 0;

	dev_info(dev, "A2 PCIe\n");
	athena2_pcie = devm_kzalloc(dev, sizeof(*athena2_pcie), GFP_KERNEL);
	if (!athena2_pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	athena2_pcie->pci = pci;
	dma_set_mask(dev, pci_dma_mask);
	pci->dev = dev;
	pci->ops = &dw_pcie_ops;

	athena2_pcie->clk = devm_clk_get(&pdev->dev, "pcie_clk");
	if (IS_ERR(athena2_pcie->clk)) {
		dev_err(dev, "get pcie clk failed\n");
		return -1;
	} else {
		clk_prepare_enable(athena2_pcie->clk);
	}

	pci->dbi_base = devm_platform_ioremap_resource_byname(pdev, "dbi");
	if (IS_ERR(pci->dbi_base)) {
		dev_err(dev, "couldn't remap dbi base\n");
		ret = PTR_ERR(pci->dbi_base);
		return ret;
	}

	athena2_pcie->apb_base = devm_platform_ioremap_resource_byname(pdev, "apb");
	if (IS_ERR(athena2_pcie->apb_base)) {
		dev_err(dev, "couldn't remap apb base\n");
		ret = PTR_ERR(athena2_pcie->apb_base);
		return ret;
	}

	athena2_pcie->sii_base = devm_platform_ioremap_resource_byname(pdev, "sii");
	if (IS_ERR(athena2_pcie->sii_base)) {
		dev_err(dev, "couldn't remap sii base\n");
		ret = PTR_ERR(athena2_pcie->sii_base);
		return ret;
	}

	athena2_pcie->top_apb_base = ioremap(PCIE_TOP_APB_BASE, 0x2000);

	ret = athena2_pcie_controller_config(athena2_pcie, dev);
	if (ret) {
		return ret;
	}

	iounmap(athena2_pcie->top_apb_base);

	platform_set_drvdata(pdev, athena2_pcie);

	ret = athena2_pcie_add_pcie_port(athena2_pcie, pdev);
	if (ret < 0)
		return ret;
	dev_info(dev, "Athena2 PCIe success\n");

	return 0;
}

static const struct of_device_id athena2_pcie_of_match[] = {
	{ .compatible = "cvitek,athena2-pcie", },
	{},
};

static struct platform_driver athena2_pcie_driver = {
	.probe = athena2_pcie_probe,
	.remove = athena2_pcie_remove,
	.driver = {
		.name = "athena2_pcie",
		.of_match_table = of_match_ptr(athena2_pcie_of_match),
	},
};

static int __init athena2_pcie_init(void)
{
	return platform_driver_register(&athena2_pcie_driver);
}

device_initcall(athena2_pcie_init);
