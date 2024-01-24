// SPDX-License-Identifier: GPL-2.0
/*
 * dwc3-of-simple.c - OF glue layer for simple integrations
 *
 * Copyright (c) 2015 Texas Instruments Incorporated - https://www.ti.com
 *
 * Author: Felipe Balbi <balbi@ti.com>
 *
 * This is a combination of the old dwc3-qcom.c by Ivan T. Ivanov
 * <iivanov@mm-sol.com> and the original patch adding support for Xilinx' SoC
 * by Subbaraya Sundeep Bhatta <subbaraya.sundeep.bhatta@xilinx.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/io.h>

#if IS_ENABLED(CONFIG_ARCH_CVITEK)
#include <linux/of_gpio.h>
#endif

struct dwc3_of_simple {
	struct device		*dev;
	struct clk_bulk_data	*clks;
	int			num_clocks;
	struct reset_control	*resets;
	bool			need_reset;
	int vbus_gpio;
};

#define REG_USB_SYS_REG_0100	0x0
#define reg_usb0_en				(1 << 0)

#define REG_USB_SYS_REG_010C	0x0c
#define reg_phy0_ref_ssp_en		(1 << 24)

#define REG_USB_SYS_REG_0114	0x14
#define reg_phy0_phy_reset		(1 << 5)

static int dwc3_of_simple_probe(struct platform_device *pdev)
{
	struct dwc3_of_simple	*simple;
	struct device		*dev = &pdev->dev;
	struct device_node	*np = dev->of_node;

#if IS_ENABLED(CONFIG_ARCH_CVITEK)
	struct resource *res;
	void __iomem *reg_usbsys;
	uint32_t value;
#endif

	int			ret;

	simple = devm_kzalloc(dev, sizeof(*simple), GFP_KERNEL);
	if (!simple)
		return -ENOMEM;

	platform_set_drvdata(pdev, simple);
	simple->dev = dev;

#if IS_ENABLED(CONFIG_ARCH_CVITEK)
	if (of_device_is_compatible(np, "sophgo,athena2-dwc3"))
	{
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		reg_usbsys = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(reg_usbsys))
			return PTR_ERR(reg_usbsys);

		value = readl(reg_usbsys + REG_USB_SYS_REG_0114) & (~reg_phy0_phy_reset);
		writel(value, reg_usbsys + REG_USB_SYS_REG_0114);

		value = readl(reg_usbsys + REG_USB_SYS_REG_010C) | (reg_phy0_ref_ssp_en);
		writel(value, reg_usbsys + REG_USB_SYS_REG_010C);

		uint32_t val;
		uint32_t reg_addr = 0x28100044;
		void __iomem *reg = ioremap(reg_addr, sizeof(uint32_t));
		val = readl(reg);
		val &= ~(0x3ff << 13);
		val |= (0x78L << 13);
		writel(val, reg);
		iounmap(reg);

		value = readl(reg_usbsys + REG_USB_SYS_REG_0100) | (1<<0) | (1<<29);
		writel(value, reg_usbsys + REG_USB_SYS_REG_0100);

		simple->vbus_gpio = of_get_named_gpio(simple->dev->of_node, "vbus-gpio", 0);
		dev_dbg(simple->dev, "get vbus_gpio number:%d\n", simple->vbus_gpio);
		if (gpio_is_valid(simple->vbus_gpio)) {
			if (devm_gpio_request_one(simple->dev, simple->vbus_gpio, GPIOF_OUT_INIT_HIGH, "vbus-gpio"))
			{
				simple->vbus_gpio = -EINVAL;
				dev_err(simple->dev, "request gpio fail!\n");
			}
		}
	}
#endif

	/*
	 * Some controllers need to toggle the usb3-otg reset before trying to
	 * initialize the PHY, otherwise the PHY times out.
	 */
	if (of_device_is_compatible(np, "rockchip,rk3399-dwc3"))
		simple->need_reset = true;

	simple->resets = of_reset_control_array_get(np, false, true,
						    true);
	if (IS_ERR(simple->resets)) {
		ret = PTR_ERR(simple->resets);
		dev_err(dev, "failed to get device resets, err=%d\n", ret);
		return ret;
	}

	ret = reset_control_deassert(simple->resets);
	if (ret)
		goto err_resetc_put;

	ret = clk_bulk_get_all(simple->dev, &simple->clks);
	if (ret < 0)
		goto err_resetc_assert;

	simple->num_clocks = ret;
	ret = clk_bulk_prepare_enable(simple->num_clocks, simple->clks);
	if (ret)
		goto err_resetc_assert;

	ret = of_platform_populate(np, NULL, NULL, dev);
	if (ret)
		goto err_clk_put;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	return 0;

err_clk_put:
	clk_bulk_disable_unprepare(simple->num_clocks, simple->clks);
	clk_bulk_put_all(simple->num_clocks, simple->clks);

err_resetc_assert:
	reset_control_assert(simple->resets);

err_resetc_put:
	reset_control_put(simple->resets);
	return ret;
}

static void __dwc3_of_simple_teardown(struct dwc3_of_simple *simple)
{
	of_platform_depopulate(simple->dev);

	clk_bulk_disable_unprepare(simple->num_clocks, simple->clks);
	clk_bulk_put_all(simple->num_clocks, simple->clks);
	simple->num_clocks = 0;

	reset_control_assert(simple->resets);

	reset_control_put(simple->resets);

	pm_runtime_disable(simple->dev);
	pm_runtime_put_noidle(simple->dev);
	pm_runtime_set_suspended(simple->dev);
}

static int dwc3_of_simple_remove(struct platform_device *pdev)
{
	struct dwc3_of_simple	*simple = platform_get_drvdata(pdev);

	__dwc3_of_simple_teardown(simple);

	return 0;
}

static void dwc3_of_simple_shutdown(struct platform_device *pdev)
{
	struct dwc3_of_simple	*simple = platform_get_drvdata(pdev);

	__dwc3_of_simple_teardown(simple);
}

static int __maybe_unused dwc3_of_simple_runtime_suspend(struct device *dev)
{
	struct dwc3_of_simple	*simple = dev_get_drvdata(dev);

	clk_bulk_disable(simple->num_clocks, simple->clks);

	return 0;
}

static int __maybe_unused dwc3_of_simple_runtime_resume(struct device *dev)
{
	struct dwc3_of_simple	*simple = dev_get_drvdata(dev);

	return clk_bulk_enable(simple->num_clocks, simple->clks);
}

static int __maybe_unused dwc3_of_simple_suspend(struct device *dev)
{
	struct dwc3_of_simple *simple = dev_get_drvdata(dev);

	if (simple->need_reset)
		reset_control_assert(simple->resets);

	return 0;
}

static int __maybe_unused dwc3_of_simple_resume(struct device *dev)
{
	struct dwc3_of_simple *simple = dev_get_drvdata(dev);

	if (simple->need_reset)
		reset_control_deassert(simple->resets);

	return 0;
}

static const struct dev_pm_ops dwc3_of_simple_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_of_simple_suspend, dwc3_of_simple_resume)
	SET_RUNTIME_PM_OPS(dwc3_of_simple_runtime_suspend,
			dwc3_of_simple_runtime_resume, NULL)
};

static const struct of_device_id of_dwc3_simple_match[] = {
	{ .compatible = "rockchip,rk3399-dwc3" },
	{ .compatible = "xlnx,zynqmp-dwc3" },
	{ .compatible = "cavium,octeon-7130-usb-uctl" },
	{ .compatible = "sprd,sc9860-dwc3" },
	{ .compatible = "allwinner,sun50i-h6-dwc3" },
	{ .compatible = "hisilicon,hi3670-dwc3" },
	{ .compatible = "intel,keembay-dwc3" },
	{ .compatible = "sophgo,athena2-dwc3" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_dwc3_simple_match);

static struct platform_driver dwc3_of_simple_driver = {
	.probe		= dwc3_of_simple_probe,
	.remove		= dwc3_of_simple_remove,
	.shutdown	= dwc3_of_simple_shutdown,
	.driver		= {
		.name	= "dwc3-of-simple",
		.of_match_table = of_dwc3_simple_match,
		.pm	= &dwc3_of_simple_dev_pm_ops,
	},
};

module_platform_driver(dwc3_of_simple_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 OF Simple Glue Layer");
MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
