/* SPDX-License-Identifier: GPL-2.0 */
/* CAN bus driver for Bosch M_CAN controller
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 */

#ifndef _CAN_SDVT_CAN_H_
#define _CAN_SDVT_CAN_H_

#include <linux/can/core.h>
#include <linux/can/led.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/freezer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/iopoll.h>
#include <linux/can/dev.h>
#include <linux/pinctrl/consumer.h>

struct sdvt_can_classdev;

struct sdvt_can_ops {
	u32 (*read_reg)(struct sdvt_can_classdev *cdev, int reg);
	int (*write_reg)(struct sdvt_can_classdev *cdev, int reg, int val);
};

struct sdvt_can_classdev {
	struct can_priv can;
	struct napi_struct napi;
	struct net_device *net;
	struct device *dev;
	struct clk *cclk;

	struct workqueue_struct *tx_wq;
	struct work_struct tx_work;
	struct sk_buff *tx_skb;

	struct can_bittiming_const *bit_timing;
	struct can_bittiming_const *data_timing;

	struct sdvt_can_ops *ops;

	void *device_data;

	int version;
	int freq;
	u32 irqstatus;

	int pm_clock_support;
	int is_peripheral;
};

struct sdvt_can_classdev *sdvt_can_class_allocate_dev(struct device *dev);
void sdvt_can_class_free_dev(struct net_device *net);
int sdvt_can_class_register(struct sdvt_can_classdev *cdev);
void sdvt_can_class_unregister(struct sdvt_can_classdev *cdev);
int sdvt_can_class_get_clocks(struct sdvt_can_classdev *cdev);
void sdvt_can_init_ram(struct sdvt_can_classdev *priv);
void sdvt_can_config_endisable(struct sdvt_can_classdev *priv, bool enable);

int sdvt_can_class_suspend(struct device *dev);
int sdvt_can_class_resume(struct device *dev);
#endif	/* _CAN_M_H_ */
