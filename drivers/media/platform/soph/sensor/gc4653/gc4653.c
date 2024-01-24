// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2017 Intel Corporation.
#include <linux/clk.h>
#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio/consumer.h>

#include <linux/cif_uapi.h>
#include <linux/sns_v4l2_uapi.h>

#include "gc4653.h"

/* I2C per write of bits */
#define REG_VALUE_08BIT		1
#define REG_VALUE_16BIT		2
#define REG_VALUE_24BIT		3

/* Chip ID */
#define GC4653_CHIP_ID_ADDR_H	0x03f0
#define GC4653_CHIP_ID_ADDR_L	0x03f1
#define GC4653_CHIP_ID			0x4653

#define USE_MODE_INDEX	0

static V4L2_SNS_TYPE_E gc4653_sns_type = V4L2_GCORE_GC4653_MIPI_4M_30FPS_10BIT;

static int gc4653_count;
static int force_bus[MAX_SENSOR_DEVICE] = {[0 ... (MAX_SENSOR_DEVICE - 1)] = -1};
module_param_array(force_bus, int, &gc4653_count, 0644);

static int gc4653_probe_index;
static const unsigned short gc4653_i2c_list[] = {0x10, 0x29};
static int gc4653_bus_map[MAX_SENSOR_DEVICE] = {1, 2, -1, -1, -1, -1};

struct gc4653_reg_list {
	u32 num_of_regs;
	const struct gc4653_reg *regs;
};

/* Mode : resolution and related config&values */
struct gc4653_mode {
	u32 max_width;
	u32 max_height;
	u32 width;
	u32 height;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	struct v4l2_fract max_fps;
	struct gc4653_reg_list reg_list;
};

/* Mode configs */
static const struct gc4653_mode supported_modes[] = {
	{
		.max_width = 2560,
		.max_height = 1440,
		.width = 2560,
		.height = 1440,
		.exp_def = 0x2000,
		.hts_def = 1484,
		.vts_def = 2432,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2560x1440_12bit_regs),
			.regs = mode_2560x1440_12bit_regs,
		},
	},
};

struct gc4653 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct i2c_client *client;
	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	/* Current mode */
	const struct gc4653_mode *cur_mode;
	/* Mutex for serialized access */
	struct mutex mutex;
	/* Streaming on/off */
	bool streaming;
	/*dtsi config*/
	struct clk       *xvclk;
	struct gpio_desc *power_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;
	struct pinctrl   *pinctrl;

	unsigned int lane_num;
	unsigned int module_index;
};

#define to_gc4653(_sd)	container_of(_sd, struct gc4653, sd)

/* Read registers up to 4 at a time */
static int gc4653_read_reg(struct gc4653 *gc4653, u16 reg, u32 len,
			    u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gc4653->sd);
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	int ret;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);

	if (len > 4)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

/* Write registers up to 4 at a time */
static int gc4653_write_reg(struct gc4653 *gc4653, u16 reg, u32 len,
			     u32 __val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gc4653->sd);
	int buf_i, val_i;
	u8 buf[6], *val_p;
	__be32 val;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val = cpu_to_be32(__val);
	val_p = (u8 *)&val;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/* Write a list of registers */
static int gc4653_write_regs(struct gc4653 *gc4653,
			      const struct gc4653_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gc4653->sd);
	int ret;
	u32 i;

	for (i = 0; i < len; i++) {
		ret = gc4653_write_reg(gc4653, regs[i].address, 1,
					regs[i].val);
		if (ret) {
			dev_err_ratelimited(
				&client->dev,
				"Failed to write reg 0x%4.4x. error = %d\n",
				regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

/* Open sub-device */
static int gc4653_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gc4653 *gc4653 = to_gc4653(sd);
	struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(sd, fh->pad, 0);

	mutex_lock(&gc4653->mutex);

	/* Initialize try_fmt */
	try_fmt->width = gc4653->cur_mode->width;
	try_fmt->height = gc4653->cur_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	/* No crop or compose */
	mutex_unlock(&gc4653->mutex);

	return 0;
}

static int gc4653_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc4653 *gc4653 = container_of(ctrl->handler,
					       struct gc4653, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&gc4653->sd);

	pm_runtime_put(&client->dev);

	return 0;
}


static const struct v4l2_ctrl_ops gc4653_ctrl_ops = {
	.s_ctrl = gc4653_set_ctrl,
};

static int g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	config->type = V4L2_MBUS_CSI2_DPHY;
	return 0;
}


static int enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	/* Only one bayer order(GRBG) is supported */
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;

	return 0;
}

static int enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes)) {
		printk("[gc4653] error fse index!\n");
		return -EINVAL;
	}

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = supported_modes[fse->index].max_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = supported_modes[fse->index].max_height;

	return 0;
}

static void update_pad_format(const struct gc4653_mode *mode,
				      struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int get_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_format *fmt)
{
	struct gc4653 *gc4653 = to_gc4653(sd);
	struct v4l2_mbus_framefmt *framefmt;
	int ret;

	mutex_lock(&gc4653->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		fmt->format = *framefmt;
		mutex_unlock(&gc4653->mutex);
		return -ENOTTY;
	} else {
		update_pad_format(gc4653->cur_mode, fmt);
	}
	mutex_unlock(&gc4653->mutex);

	return ret;
}

static int set_pad_format(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *fmt)
{
	struct gc4653 *gc4653 = to_gc4653(sd);
	const struct gc4653_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;

	mutex_lock(&gc4653->mutex);

	/* Only one raw bayer(GRBG) order is supported */
	if (fmt->format.code != MEDIA_BUS_FMT_SGRBG10_1X10)
		fmt->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);
	update_pad_format(mode, fmt);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		*framefmt = fmt->format;
	} else {
		gc4653->cur_mode = mode;
	}

	mutex_unlock(&gc4653->mutex);

	return 0;
}

static void gc4653_standby(struct gc4653 *gc4653)
{
	gc4653_write_reg(gc4653, 0x0100, REG_VALUE_08BIT, 0x00);
	printk("[gc4653] standby\n");
}

static void gc4653_restart(struct gc4653 *gc4653)
{
	gc4653_write_reg(gc4653, 0x0100, REG_VALUE_08BIT, 0x01);
	printk("[gc4653] restart\n");
}

/* Start streaming */
static int start_streaming(struct gc4653 *gc4653)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gc4653->sd);
	const struct gc4653_reg_list *reg_list;
	int ret;

	/* Apply default values of current mode */
	reg_list = &gc4653->cur_mode->reg_list;
	ret = gc4653_write_regs(gc4653, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	usleep_range(100 * 1000, 120 * 2000);
	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(gc4653->sd.ctrl_handler);
	if (ret)
		return ret;

	printk("[gc4653] init reg done\n");

	return ret;
}

/* Stop streaming */
static int stop_streaming(struct gc4653 *gc4653)
{
	gc4653_standby(gc4653);
	return 0;
}

static int set_stream(struct v4l2_subdev *sd, int enable)
{
	struct gc4653 *gc4653 = to_gc4653(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	printk("[gc4653] set stream(%d)\n", enable);

	mutex_lock(&gc4653->mutex);
	if (gc4653->streaming == enable) {
		mutex_unlock(&gc4653->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = start_streaming(gc4653);
		if (ret)
			goto err_rpm_put;
	} else {
		stop_streaming(gc4653);
		pm_runtime_put(&client->dev);
	}

	gc4653->streaming = enable;
	mutex_unlock(&gc4653->mutex);

	printk("[gc4653] set stream success\n");

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&gc4653->mutex);

	return ret;
}

static int __maybe_unused suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc4653 *gc4653 = to_gc4653(sd);

	if (gc4653->streaming)
		stop_streaming(gc4653);

	return 0;
}

static int __maybe_unused resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc4653 *gc4653 = to_gc4653(sd);
	int ret;

	if (gc4653->streaming) {
		ret = start_streaming(gc4653);
		if (ret)
			goto error;
	}

	return 0;

error:
	stop_streaming(gc4653);
	gc4653->streaming = false;
	return ret;
}

/* Verify chip ID */
static int gc4653_identify_module(struct gc4653 *gc4653)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gc4653->sd);
	int ret;
	int nVal, nVal2, nVal3;
	int read_data = 0;

	ret = gc4653_read_reg(gc4653, GC4653_CHIP_ID_ADDR_H,
			       REG_VALUE_08BIT, &nVal);
	printk("[gc4653] read id:0x%x, ret:%d", nVal, ret);
	if (ret)
		return ret;

	ret = gc4653_read_reg(gc4653, GC4653_CHIP_ID_ADDR_L,
			       REG_VALUE_08BIT, &nVal3);

	read_data = ((nVal & 0xFF) << 8) | (nVal3 & 0xFF);

	if (read_data != GC4653_CHIP_ID) {
		dev_err(&client->dev, "chip id(%x) mismatch, read(%x)\n",
			GC4653_CHIP_ID, read_data);
		return -EIO;
	}

	return 0;
}

static void gc4653_mirror_flip(struct gc4653 *gc4653, int orient)
{
	int val = 0;
	int ori_addr = 0x3820;
	printk("[gc4653] set mirror_flip:%d", orient);

	gc4653_read_reg(gc4653, ori_addr, REG_VALUE_08BIT, &val);

	val &= ~(0x3 << 1);

	switch (orient) {
	case 0:
		break;
	case 1:
		val |= 0x1 << 1;
		break;
	case 2:
		val |= 0x1 << 2;
		break;
	case 3:
		val |= 0x1 << 1;
		val |= 0x1 << 2;
		break;
	default:
		return;
	}

	gc4653_standby(gc4653);
	gc4653_write_reg(gc4653, ori_addr, REG_VALUE_08BIT, val);
	gc4653_restart(gc4653);
}

static long gc4653_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct gc4653 *gc4653 = to_gc4653(sd);
	long ret = 0;

	switch (cmd) {
	case SNS_V4L2_GET_TYPE:
	{
		int type = gc4653_sns_type;
		memcpy(arg, &type, sizeof(int));
		break;
	}

	case SNS_V4L2_SET_MIRROR_FLIP:
	{
		int orient = 0;
		memcpy(&orient, arg, sizeof(int));
		gc4653_mirror_flip(gc4653, orient);
		break;
	}

	case SNS_V4L2_GET_I2C_INFO:
	{
		struct i2c_client *client = v4l2_get_subdevdata(&gc4653->sd);
		sns_i2c_info_t i2c_info;
		i2c_info.i2c_addr =  client->addr;
		i2c_info.i2c_idx  =  client->adapter->i2c_idx;
		memcpy(arg, &i2c_info, sizeof(sns_i2c_info_t));
		break;
	}

	default:
		ret = -ENOIOCTLCMD;
		printk("[gc4653] unknown ioctl cmd:%d", cmd);
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long gc4653_compat_ioctl32(struct v4l2_subdev *sd,
				   unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	long ret;
	printk("[gc4653] compat ioctl cmd:%d", cmd);

	switch (cmd) {
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif


static const struct v4l2_subdev_core_ops gc4653_core_ops = {
	.ioctl = gc4653_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = gc4653_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops gc4653_video_ops = {
	.s_stream = set_stream,
};

static const struct v4l2_subdev_pad_ops gc4653_pad_ops = {
	.enum_mbus_code = enum_mbus_code,
	.get_fmt = get_pad_format,
	.set_fmt = set_pad_format,
	.enum_frame_size = enum_frame_size,
	.get_mbus_config = g_mbus_config,
};

static const struct v4l2_subdev_ops gc4653_subdev_ops = {
	.core	= &gc4653_core_ops,
	.video  = &gc4653_video_ops,
	.pad    = &gc4653_pad_ops,
};

static const struct media_entity_operations gc4653_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops gc4653_internal_ops = {
	.open = gc4653_open,
};

/* Initialize control handlers */
static int gc4653_init_controls(struct gc4653 *gc4653, int index_id)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gc4653->sd);
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_ctrl *ctrl;
	int ret;
	int i;

	ctrl_hdlr = &gc4653->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 10);
	if (ret) {
		dev_err(&client->dev, "%s ctrl handler init failed (%d)\n",
			__func__, ret);
		return ret;
	}

	mutex_init(&gc4653->mutex);
	ctrl_hdlr->lock = &gc4653->mutex;
	for (i = 0; i < SNS_CFG_TYPE_MAX; i++) {
		ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr,
					&gc4653_ctrl_ops,
					V4L2_CID_LINK_FREQ,
					gc4653_link_cif_menu[index_id][i], 0,
					gc4653_link_cif_menu[index_id]);
		if (ctrl)
			ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	}

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s new std menu failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &gc4653_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	gc4653->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&gc4653->mutex);

	return ret;
}

static void gc4653_free_controls(struct gc4653 *gc4653)
{
	v4l2_ctrl_handler_free(gc4653->sd.ctrl_handler);
	mutex_destroy(&gc4653->mutex);
}

static int gc4653_probe(struct i2c_client *client,
			 const struct i2c_device_id *devid)
{
	struct gc4653 *gc4653;
	struct v4l2_subdev *sd;
	struct device *dev = &client->dev;
	int index_id = gc4653_probe_index;
	int addr_num = sizeof(gc4653_i2c_list) / sizeof(unsigned short);
	int bus_id;
	int ret = -1;
	int i;

	printk("[gc4653] probe id[%d] start\n", gc4653_probe_index);

	gc4653_probe_index++;

	if (index_id >= MAX_SENSOR_DEVICE || index_id < 0) {
		printk("[gc4653] invalid devid(%d)\n", index_id);
		return ret;
	}

	gc4653 = devm_kzalloc(&client->dev, sizeof(*gc4653), GFP_KERNEL);
	if (!gc4653) {
		dev_err(dev, "Failed to alloc devmem!\n");
		return -ENOMEM;
	}

	sd = &gc4653->sd;

	for (i = 0; i < addr_num; i++) {
		if (force_bus[index_id] < 0) {
			bus_id = gc4653_bus_map[index_id];
		} else {
			bus_id = force_bus[index_id];
		}

		if (bus_id < 0 || bus_id > MAX_I2C_BUS_NUM) {
			return ret;
		}

		client->addr = gc4653_i2c_list[i];
		client->adapter = i2c_get_adapter(bus_id);
		gc4653->client = client;
		v4l2_i2c_subdev_init(sd, client, &gc4653_subdev_ops);

		/* Check module identity */
		ret = gc4653_identify_module(gc4653);
		if (ret) {
			printk("id[%d] bus[%d] i2c_addr[%d][0x%x] no sensor found\n",
				index_id, bus_id, i, client->addr);

			if (i == addr_num - 1) {
				return ret;
			}
			continue;;
		} else {
			printk("id[%d] bus[%d] i2c_addr[0x%x] sensor found\n",
				index_id, bus_id, client->addr);
			break;
		}
	}

	gc4653->module_index = index_id;

	/* Set default mode to max resolution */
	gc4653->cur_mode = &supported_modes[USE_MODE_INDEX];

	mutex_init(&gc4653->mutex);

	ret = gc4653_init_controls(gc4653, index_id);
	if (ret) {
		return ret;
	}

	/* Initialize subdev */
	sd->internal_ops = &gc4653_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.ops = &gc4653_subdev_entity_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	gc4653->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &gc4653->pad);
	if (ret) {
		dev_err(&client->dev, "failed to init pads:%d\n", ret);
		goto error_handler_free;
	}

	snprintf(sd->name, sizeof(sd->name), "cam%d_%s %s",
		 gc4653->module_index, "gc4653", dev_name(sd->dev));

	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret < 0) {
		dev_err(&client->dev, "failed to async subdev:%d\n", ret);
		goto error_media_entity;
	}

	/*
	 * Device is already turned on by i2c-core with ACPI domain PM.
	 * Enable runtime PM and turn off the device.
	 */
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	printk("[gc4653] sensor_%d probe success\n", index_id);

	return 0;

error_media_entity:
	media_entity_cleanup(&gc4653->sd.entity);

error_handler_free:
	gc4653_free_controls(gc4653);
	dev_err(&client->dev, "%s failed:%d\n", __func__, ret);

	return ret;
}

static int gc4653_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc4653 *gc4653 = to_gc4653(sd);

	gc4653_probe_index = 0;

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	gc4653_free_controls(gc4653);

	pm_runtime_disable(&client->dev);

	return 0;
}

static const struct of_device_id gc4653_of_match[] = {
	{ .compatible = "v4l2,sensor0" },
	{ .compatible = "v4l2,sensor1" },
	{ .compatible = "v4l2,sensor2" },
	{ .compatible = "v4l2,sensor3" },
	{ .compatible = "v4l2,sensor4" },
	{ .compatible = "v4l2,sensor5" },
	{},
};
MODULE_DEVICE_TABLE(of, gc4653_of_match);

static const struct dev_pm_ops gc4653_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(suspend, resume)
};

static struct i2c_driver gc4653_i2c_driver = {
	.driver = {
		.name = "gc4653",
		.pm = &gc4653_pm_ops,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(gc4653_of_match),
	},
	.probe    = gc4653_probe,
	.remove   = gc4653_remove,
};

static int __init sensor_mod_init(void)
{
	printk("== gc4653 mod add ==\n");
	return i2c_add_driver(&gc4653_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&gc4653_i2c_driver);
}

module_init(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("gc4653 sensor driver");
MODULE_LICENSE("GPL v2");
