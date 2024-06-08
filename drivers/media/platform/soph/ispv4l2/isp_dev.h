#ifndef _ISP_DEV_H
#define _ISP_DEV_H

#include <linux/compat.h>
#include <linux/iopoll.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <media/v4l2-event.h>
#include <media/v4l2-subdev.h>
#include <linux/platform_device.h>
#include <media/media-entity.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-device.h>
#include <linux/platform_device.h>
#include <media/v4l2-fwnode.h>
#include <linux/of_graph.h>
#include <media/media-device.h>
#include <media/v4l2-mc.h>
#include <linux/media.h>

#include "vi_defines.h"

#define ISP_V4L2_EVENT_ELEMS 4
#define ISP_SUBDEV_NAME   "ispv4l2_subdev"
#define V4L2_ISP_IRQ_NAME "ispv4l2_irq"

#define GRP_ID_SENSOR		BIT(0)
#define GRP_ID_CIF			BIT(1)
#define GRP_ID_ISP			BIT(2)

#define MAX_SENSOR_NUM 6

enum cvi_isp_pad {
	CVI_ISP_PAD_SINK,
	CVI_ISP_PAD_SOURCE,
	CVI_ISP_PAD_MAX
};

struct cvi_sensor_info {
	struct v4l2_subdev *sd;
	struct device *dev;
	struct v4l2_mbus_config mbus;
	struct v4l2_subdev_frame_interval fi;
	struct v4l2_subdev_format fmt[2];
	struct v4l2_subdev_pad_config cfg;
};

struct cvi_isp_subdev {
	struct v4l2_subdev sd;
	struct media_pad pads[CVI_ISP_PAD_MAX];
	atomic_t frm_sync_seq;
};

struct cvi_isp_async_subdev {
	struct v4l2_async_subdev asd;
	struct v4l2_mbus_config mbus;
};

struct cvi_isp_device {
	char name[128];
	struct mutex  mutex;
	struct device *dev;
	struct video_device vdev;
	struct v4l2_device v4l2_dev;
	struct cvi_vi_dev vi_dev;
	struct cvi_isp_subdev isp_sdev;
	struct media_device media_dev;
	struct v4l2_async_notifier notifier;
	struct cvi_sensor_info *active_sensor;
	struct cvi_sensor_info sensors[MAX_SENSOR_NUM];
	int num_sensors;
	int is_powerOn;
};
#endif /* _ISP_DEV_H */
