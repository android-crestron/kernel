/*
 *ov5645 - This code emulates a real video device with v4l2 api
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the BSD Licence, GNU General Public License
 * as published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version
 */
#include <linux/sizes.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/pci.h>
#include <linux/random.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/highmem.h>
#include <linux/freezer.h>
#include <media/videobuf-res.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <linux/wakelock.h>

#include <linux/i2c.h>
#include <media/v4l2-chip-ident.h>
#include <linux/amlogic/camera/aml_cam_info.h>
#include <linux/amlogic/vmapi.h>

#include <mach/am_regs.h>
//#include <mach/am_eth_pinmux.h>
#include <mach/pinmux.h>
#include <mach/gpio.h>

#include "common/plat_ctrl.h"
#include "common/vm.h"
#include "ov5645_firmware.h"

#if MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON6
#include <mach/mod_gate.h>
#endif
#define ov5645_CAMERA_MODULE_NAME "ov5645"
#define MAGIC_RE_MEM 0x123039dc
#define ov5645_RES0_CANVAS_INDEX CAMERA_USER_CANVAS_INDEX

/* Wake up at about 30 fps */
#define WAKE_NUMERATOR 30
#define WAKE_DENOMINATOR 1001
#define BUFFER_TIMEOUT     msecs_to_jiffies(500)  /* 0.5 seconds */

#define ov5645_CAMERA_MAJOR_VERSION 0
#define ov5645_CAMERA_MINOR_VERSION 7
#define ov5645_CAMERA_RELEASE 0
#define ov5645_CAMERA_VERSION \
	KERNEL_VERSION(ov5645_CAMERA_MAJOR_VERSION, ov5645_CAMERA_MINOR_VERSION, ov5645_CAMERA_RELEASE)

MODULE_DESCRIPTION("ov5645 On Board");
MODULE_AUTHOR("amlogic-sh");
MODULE_LICENSE("GPL v2");

#define ov5645_DRIVER_VERSION "ov5645-COMMON-01-150701"

static unsigned video_nr = -1;  /* videoX start number, -1 is autodetect. */

static unsigned debug;
//module_param(debug, uint, 0644);
//MODULE_PARM_DESC(debug, "activates debug info");

static unsigned int vid_limit = 32;
//module_param(vid_limit, uint, 0644);
//MODULE_PARM_DESC(vid_limit, "capture memory limit in megabytes");
struct i2c_client *g_client = NULL ; 
#define OV5645MIPI_write_cmos_sensor(a,b) i2c_put_byte(g_client, a, b)
#define OV5645MIPIYUV_read_cmos_sensor(v) i2c_get_byte(g_client, v) 

//extern int disable_ov5645;
static int ov5645_have_opened = 0;

static void OV5645_set_AWB_mode(bool AWB_enable);
static void do_download(struct work_struct *work);
static DECLARE_DELAYED_WORK(dl_work, do_download);

static struct vdin_v4l2_ops_s *vops;

static bool bDoingAutoFocusMode=false;
static int temp_frame=-1;
static struct v4l2_fract ov5645_frmintervals_active = {
	.numerator = 1,
	.denominator = 15,
};

static struct v4l2_frmivalenum ov5645_frmivalenum[]={
	{
		.index = 0,
		.pixel_format = V4L2_PIX_FMT_NV21,
		.width = 640,
		.height = 480,
		.type = V4L2_FRMIVAL_TYPE_DISCRETE,
		{
			.discrete ={
				.numerator = 1,
				.denominator = 30,
			}
		}
	},{
		.index = 0,
		.pixel_format = V4L2_PIX_FMT_NV21,
		.width = 1024,
		.height = 768,
		.type = V4L2_FRMIVAL_TYPE_DISCRETE,
		{
			.discrete ={
				.numerator = 1,
				.denominator = 30,
			}
		}
	},{
		.index = 0,
		.pixel_format = V4L2_PIX_FMT_NV21,
		.width = 1280,
		.height = 720,
		.type = V4L2_FRMIVAL_TYPE_DISCRETE,
		{
			.discrete ={
				.numerator = 1,
				.denominator = 30,
			}
		}
	},{
		.index = 0,
		.pixel_format = V4L2_PIX_FMT_NV21,
		.width = 1920,
		.height = 1080,
		.type = V4L2_FRMIVAL_TYPE_DISCRETE,
		{
			.discrete ={
				.numerator = 1,
				.denominator = 30,
			}
		}
	},{
		.index = 1,
		.pixel_format = V4L2_PIX_FMT_NV21,
		.width = 1600,
		.height = 1200,
		.type = V4L2_FRMIVAL_TYPE_DISCRETE,
		{
			.discrete ={
				.numerator = 1,
				.denominator = 5,
			}
		}
	},{
		.index = 1,
		.pixel_format = V4L2_PIX_FMT_NV21,
		.width = 2048,
		.height = 1536,
		.type = V4L2_FRMIVAL_TYPE_DISCRETE,
		{
			.discrete ={
				.numerator = 1,
				.denominator = 5,
			}
		}
	},{
		.index = 1,
		.pixel_format = V4L2_PIX_FMT_NV21,
		.width = 2592,
		.height = 1944,
		.type = V4L2_FRMIVAL_TYPE_DISCRETE,
		{
			.discrete ={
				.numerator = 1,
				.denominator = 5,
			}
		}
	},
};

/* supported controls */
static struct v4l2_queryctrl ov5645_qctrl[] = {
	{
		.id            = V4L2_CID_BRIGHTNESS,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Brightness",
		.minimum       = 0,
		.maximum       = 255,
		.step          = 1,
		.default_value = 127,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id            = V4L2_CID_CONTRAST,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Contrast",
		.minimum       = 0x10,
		.maximum       = 0x60,
		.step          = 0xa,
		.default_value = 0x30,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	},{
		.id            = V4L2_CID_HFLIP,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "flip on horizontal",
		.minimum       = 0,
		.maximum       = 1,
		.step          = 0x1,
		.default_value = 0,
		.flags         = V4L2_CTRL_FLAG_DISABLED,
	} ,{
		.id            = V4L2_CID_VFLIP,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "flip on vertical",
		.minimum       = 0,
		.maximum       = 1,
		.step          = 0x1,
		.default_value = 0,
		.flags         = V4L2_CTRL_FLAG_DISABLED,
	},{
		.id            = V4L2_CID_DO_WHITE_BALANCE,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "white balance",
		.minimum       = 0,
		.maximum       = 6,
		.step          = 0x1,
		.default_value = 0,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	},{
		.id            = V4L2_CID_EXPOSURE,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "exposure",
		.minimum       = 0,
		.maximum       = 8,
		.step          = 0x1,
		.default_value = 4,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	},{
		.id            = V4L2_CID_COLORFX,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "effect",
		.minimum       = 0,
		.maximum       = 6,
		.step          = 0x1,
		.default_value = 0,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	},{
		.id            = V4L2_CID_WHITENESS,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "banding",
		.minimum       = 0,
		.maximum       = 1,
		.step          = 0x1,
		.default_value = 0,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	},
	/*
	{
		.id            = V4L2_CID_FOCUS_AUTO,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "auto focus",
		.minimum       = CAM_FOCUS_MODE_RELEASE,
		.maximum       = CAM_FOCUS_MODE_CONTI_PIC,
		.step          = 0x1,
		.default_value = CAM_FOCUS_MODE_CONTI_PIC,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	},
	*/
	{
		.id            = V4L2_CID_BACKLIGHT_COMPENSATION,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "flash",
		.minimum       = FLASHLIGHT_ON,
		.maximum       = FLASHLIGHT_TORCH,
		.step          = 0x1,
		.default_value = FLASHLIGHT_OFF,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	},{
		.id            = V4L2_CID_ZOOM_ABSOLUTE,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Zoom, Absolute",
		.minimum       = 100,
		.maximum       = 300,
		.step          = 20,
		.default_value = 100,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	},{
		.id		= V4L2_CID_ROTATE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Rotate",
		.minimum	= 0,
		.maximum	= 270,
		.step		= 90,
		.default_value	= 0,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
	},{
		.id            = V4L2_CID_AUTO_FOCUS_STATUS,
		.type          = 8,//V4L2_CTRL_TYPE_BITMASK,
		.name          = "focus status",
		.minimum       = 0,
		.maximum       = ~3,
		.step          = 0x1,
		.default_value = V4L2_AUTO_FOCUS_STATUS_IDLE,
		.flags         = V4L2_CTRL_FLAG_READ_ONLY,
	},{
		.id		= V4L2_CID_FOCUS_ABSOLUTE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "focus center",
		.minimum	= 0,
		.maximum	= ((2000) << 16) | 2000,
		.step		= 1,
		.default_value	= (1000 << 16) | 1000,
		.flags         = V4L2_CTRL_FLAG_SLIDER,
 	}
};

struct v4l2_querymenu ov5645_qmenu_autofocus[] = {
	{
		.id         = V4L2_CID_FOCUS_AUTO,
		.index      = CAM_FOCUS_MODE_INFINITY,
		.name       = "infinity",
		.reserved   = 0,
	},{
		.id         = V4L2_CID_FOCUS_AUTO,
		.index      = CAM_FOCUS_MODE_AUTO,
		.name       = "auto",
		.reserved   = 0,
	},{
		.id         = V4L2_CID_FOCUS_AUTO,
		.index      = CAM_FOCUS_MODE_CONTI_VID,
		.name       = "continuous-video",
		.reserved   = 0,
	},{
		.id         = V4L2_CID_FOCUS_AUTO,
		.index      = CAM_FOCUS_MODE_CONTI_PIC,
		.name       = "continuous-picture",
		.reserved   = 0,
	}
};

struct v4l2_querymenu ov5645_qmenu_flashmode[] = {
	{
		.id         = V4L2_CID_BACKLIGHT_COMPENSATION,
		.index      = FLASHLIGHT_ON,
		.name       = "on",
		.reserved   = 0,
	},{
		.id         = V4L2_CID_BACKLIGHT_COMPENSATION,
		.index      = FLASHLIGHT_OFF,
		.name       = "off",
		.reserved   = 0,
	},{
		.id         = V4L2_CID_BACKLIGHT_COMPENSATION,
		.index      = FLASHLIGHT_TORCH,
		.name       = "torch",
		.reserved   = 0,
	}
};

struct v4l2_querymenu ov5645_qmenu_wbmode[] = {
    {
        .id         = V4L2_CID_DO_WHITE_BALANCE,
        .index      = CAM_WB_AUTO,
        .name       = "auto",
        .reserved   = 0,
    },{
        .id         = V4L2_CID_DO_WHITE_BALANCE,
        .index      = CAM_WB_CLOUD,
        .name       = "cloudy-daylight",
        .reserved   = 0,
    },{
        .id         = V4L2_CID_DO_WHITE_BALANCE,
        .index      = CAM_WB_INCANDESCENCE,
        .name       = "incandescent",
        .reserved   = 0,
    },{
        .id         = V4L2_CID_DO_WHITE_BALANCE,
        .index      = CAM_WB_DAYLIGHT,
        .name       = "daylight",
        .reserved   = 0,
    },{
        .id         = V4L2_CID_DO_WHITE_BALANCE,
        .index      = CAM_WB_FLUORESCENT,
        .name       = "fluorescent", 
        .reserved   = 0,
    },{
        .id         = V4L2_CID_DO_WHITE_BALANCE,
        .index      = CAM_WB_FLUORESCENT,
        .name       = "warm-fluorescent", 
        .reserved   = 0,
    },
};

typedef struct {
	__u32   id;
	int     num;
	struct v4l2_querymenu* ov5645_qmenu;
}ov5645_qmenu_set_t;

ov5645_qmenu_set_t ov5645_qmenu_set[] = {
	/*
	{
		.id             = V4L2_CID_FOCUS_AUTO,
		.num            = ARRAY_SIZE(ov5645_qmenu_autofocus),
		.ov5645_qmenu   = ov5645_qmenu_autofocus,
	},*/
   	{
		.id             = V4L2_CID_BACKLIGHT_COMPENSATION,
		.num            = ARRAY_SIZE(ov5645_qmenu_flashmode),
		.ov5645_qmenu   = ov5645_qmenu_flashmode,
	},{
        .id         	= V4L2_CID_DO_WHITE_BALANCE,
        .num            = ARRAY_SIZE(ov5645_qmenu_wbmode),
        .ov5645_qmenu   = ov5645_qmenu_wbmode,
    }
};

#define dprintk(dev, level, fmt, arg...) \
	v4l2_dbg(level, debug, &dev->v4l2_dev, fmt, ## arg)

/* ------------------------------------------------------------------
	Basic structures
   ------------------------------------------------------------------*/

struct ov5645_fmt {
	char  *name;
	u32   fourcc;          /* v4l2 format id */
	int   depth;
};

static struct ov5645_fmt formats[] = {
	{
		.name     = "RGB565 (BE)",
		.fourcc   = V4L2_PIX_FMT_RGB565X, /* rrrrrggg gggbbbbb */
		.depth    = 16,
	}, {
		.name     = "RGB888 (24)",
		.fourcc   = V4L2_PIX_FMT_RGB24, /* 24  RGB-8-8-8 */
		.depth    = 24,
	}, {
		.name     = "BGR888 (24)",
		.fourcc   = V4L2_PIX_FMT_BGR24, /* 24  BGR-8-8-8 */
		.depth    = 24,
	}, {
		.name     = "12  Y/CbCr 4:2:0SP",
		.fourcc   = V4L2_PIX_FMT_NV12,
		.depth    = 12,    
	}, {
		.name     = "12  Y/CbCr 4:2:0SP",
		.fourcc   = V4L2_PIX_FMT_NV21,
		.depth    = 12,    
	}, {
		.name     = "YUV420P",
		.fourcc   = V4L2_PIX_FMT_YUV420,
		.depth    = 12,
	},{
		.name     = "YVU420P",
		.fourcc   = V4L2_PIX_FMT_YVU420,
		.depth    = 12,
	}
};

static struct ov5645_fmt *get_format(struct v4l2_format *f)
{
	struct ov5645_fmt *fmt;
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(formats); k++) {
		fmt = &formats[k];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			break;
	}

	if (k == ARRAY_SIZE(formats))
    	return NULL;

	return &formats[k];
}

struct sg_to_addr {
	int pos;
	struct scatterlist *sg;
};

/* buffer for one video frame */
struct ov5645_buffer {
    /* common v4l buffer stuff -- must be first */
	struct videobuf_buffer vb;

	struct ov5645_fmt        *fmt;

	unsigned int canvas_id;
};

struct ov5645_dmaqueue {
	struct list_head       active;

    /* thread for generating video stream*/
	struct task_struct         *kthread;
	wait_queue_head_t          wq;
    /* Counters to control fps rate */
	int                        frame;
	int                        ini_jiffies;
};

typedef struct resolution_param {
	struct v4l2_frmsize_discrete frmsize;
	struct v4l2_frmsize_discrete active_frmsize;
	int active_fps;
	resolution_size_t size_type;
	struct aml_camera_i2c_fig_s* reg_script;
} resolution_param_t;

static LIST_HEAD(ov5645_devicelist);

struct ov5645_device {
	struct list_head	    	ov5645_devicelist;
	struct v4l2_subdev	    	sd;
	struct v4l2_device	    	v4l2_dev;

	spinlock_t                 slock;
	struct mutex	        	mutex;

	int                        users;

	/* various device info */
	struct video_device        *vdev;

	struct ov5645_dmaqueue       vidq;

	/* Several counters */
	unsigned long              jiffies;

	/* Input Number */
	int	           input;

	/* platform device data from board initting. */
	aml_cam_info_t  cam_info;
    
	/* Control 'registers' */
	int                qctl_regs[ARRAY_SIZE(ov5645_qctrl)];
	
	/* current resolution param for preview and capture */
	resolution_param_t* cur_resolution_param;
	
	/* wake lock */
	struct wake_lock	wake_lock;
	
	/* for down load firmware */
	struct work_struct dl_work;
	
	int firmware_ready;
};

static DEFINE_MUTEX(firmware_mutex);

static inline struct ov5645_device *to_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov5645_device, sd);
}

struct ov5645_fh {
	struct ov5645_device            *dev;

	/* video capture */
	struct ov5645_fmt            *fmt;
	unsigned int               width, height;
	struct videobuf_queue      vb_vidq;

	struct videobuf_res_privdata res;

	enum v4l2_buf_type         type;
	int	           input;     /* Input Number on bars */
	int  stream_on;
	unsigned int		f_flags;
};

/*static inline struct ov5645_fh *to_fh(struct ov5645_device *dev)
{
	return container_of(dev, struct ov5645_fh, dev);
}*/

#if 0
//MTK used
/* ------------------------------------------------------------------
	reg spec of ov5645
   ------------------------------------------------------------------*/
static struct aml_camera_i2c_fig_s ov5645_script[] = {
//;OV5645MIPI 1280x960,30fps
//56Mhz, 224Mbps/Lane, 2 Lane 
	{0x3008, 0x42},//	, software power down
	{0x3103, 0x03},//	, clock from PLL
	{0x3503, 0x07},//	, AGC manual, AEC manual
	{0x3406, 0x01},//	, awb manual, 	
	{0x3000, 0x30},
	{0x3004, 0xef},
	{0x3002, 0x1c},//	, system reset
	{0x3006, 0xc3},//	, clock enable
	{0x300e, 0x45},//	, MIPI 2 lane
	{0x3017, 0x40},//	, Frex, CSK input, Vsync output
	{0x3018, 0x00},//	, GPIO input
	{0x302c, 0x02},//	, GPIO input
	{0x302e, 0x0b},//
	{0x3031, 0x08},//	zhouliao:from 0x08	//0x00
	{0x3611, 0x06},//   laimingji:from 0x06, low light noise 0xc6
	{0x3612, 0xab},//   laimingji:from 0xab, low light noise 0x2b
	{0x3614, 0x50},//
	{0x3618, 0x00},//
	{0x4800, 0x24},//	{0x4800, 0x04}chage mipi data free/gate
	{0x3034, 0x18},//	, PLL, MIPI 8-bit mode
	{0x3035, 0x21},//	, PLL
	{0x3036, 0x70},//	, PLL
	{0x3037, 0x13}, // , PLL
	{0x3108, 0x01}, // , PLL
	{0x3824, 0x01}, // , PLL
	{0x460c, 0x20}, // , PLL
	{0x3400, 0x05},// r
	{0x3401, 0x42}, 
	{0x3402, 0x04},// g
	{0x3403, 0x00}, 
	{0x3404, 0x07},// b
	{0x3405, 0x60}, 
	{0x3600, 0x09},//
	{0x3601, 0x43},//
	{0x3620, 0x33},//
	{0x3621, 0xe0},//
	{0x3622, 0x01},//
	{0x3630, 0x2d},//
	{0x3631, 0x00},//
	{0x3632, 0x32},//
	{0x3633, 0x52},//
	{0x3634, 0x70},//
	{0x3635, 0x13},//
	{0x3636, 0x03},//
	{0x3702, 0x6e},//
	{0x3703, 0x52},//
	{0x3704, 0xa0},//
	{0x3705, 0x33},//
	{0x3708, 0x66},//
	{0x3709, 0x12},//
	{0x370b, 0x61},//
	{0x370c, 0xc3},//
	{0x370f, 0x10},//
	{0x3715, 0x08},//
	{0x3717, 0x01},//
	{0x371b, 0x20},//
	{0x3731, 0x22},//
	{0x3739, 0x70},//
	{0x3901, 0x0a},//
	{0x3905, 0x02},//
	{0x3906, 0x10},//
	{0x3719, 0x86},//
	{0x3800, 0x00},//	, HS = 0
	{0x3801, 0x00},//	, HS
	{0x3802, 0x00},//	, VS = 250
	{0x3803, 0x06},//	, VS
	{0x3804, 0x0a},//	, HW = 2623
	{0x3805, 0x3f},//	, HW
	{0x3806, 0x07},//	, VH = 1705
	{0x3807, 0x9d},//	, VH
	{0x3808, 0x05},//	, DVPHO = 1280
	{0x3809, 0x00},//	, DVPHO
	{0x380a, 0x03},//	, DVPHO
	{0x380b, 0xc0},//	, DVPVO
	{0x380c, 0x07},//	, HTS = 2160
	{0x380d, 0x68},//	, HTS
	{0x380e, 0x03},//	, VTS = 740
	{0x380f, 0xd8},//	, VTS
	{0x3810, 0x00},//	, H OFF = 16
	{0x3811, 0x10},//	, H OFF
	{0x3812, 0x00},//	, V OFF = 4
	{0x3813, 0x06},//	, V OFF
	{0x3814, 0x31},//	, X INC
	{0x3815, 0x31},//	, Y INC
	{0x3820, 0x47},//	, flip off, V bin on
	{0x3821, 0x01},//	, mirror on, H bin on
	{0x3826, 0x03}, // 
	{0x3828, 0x08},//
	{0x3a02, 0x03},//	, max exp 60 = 740
	{0x3a03, 0xd8},//	, max exp 60
	{0x3a08, 0x01},//	, B50 = 222
	{0x3a09, 0x27}, // , B50
	{0x3a0a, 0x00}, // , B60 = 185
	{0x3a0b, 0xf6}, // , B60
	{0x3a0e, 0x03}, // , max 50
	{0x3a0d, 0x04}, // , max 60
	{0x3a14, 0x03}, // , max exp 50 = 740
	{0x3a15, 0xd8},//	, max exp 50
	{0x3a18, 0x00},//	, gain ceiling = 15.5x
	{0x3a19, 0x60},//	, gain ceiling
	{0x3a05, 0x30},//	, enable band insert, ken,  
	{0x3c01, 0xb4}, // ,manual banding mode
	{0x3c00, 0x04}, // ,50 Banding mode 
	{0x3c04, 0x28},//
	{0x3c05, 0x98},//
	{0x3c07, 0x07},//
	{0x3c08, 0x01},//
	{0x3c09, 0xc2},//
	{0x3c0a, 0x9c},//
	{0x3c0b, 0x40},//
	{0x4001, 0x02},//	, BLC start line
	{0x4004, 0x02},//	, BLC line number
	{0x4005, 0x18},//	, BLC update triggered by gain change
	{0x4050, 0x6e},//	, BLC line number
	{0x4051, 0x8f},//	, BLC update triggered by gain change
	{0x4300, 0x32},//	, YUV 422, YUYV
	{0x4514, 0x00},//
	{0x4520, 0xb0},//
	{0x460b, 0x37},//
	{0x4818, 0x01},//
	{0x481d, 0xf0},//
	{0x481f, 0x50},//
	{0x4823, 0x70},//
	{0x4831, 0x14},//
	{0x4837, 0x11},//
	{0x5000, 0xa7},//	, Lenc on, raw gamma on, BPC on, WPC on, color interpolation on
	{0x5001, 0xa3},//	, SDE on, scale off, UV adjust off, color matrix on, AWB on
	{0x5002, 0x80},//   
	{0x501f, 0x00},//	, select ISP YUV 422
	{0x503d, 0x00},//
	{0x505c, 0x30},//
	{0x5181, 0x59},//
	{0x5183, 0x00},//
	{0x5191, 0xf0},//
	{0x5192, 0x03},//    
	{0x5684, 0x10},//
	{0x5685, 0xa0},//
	{0x5686, 0x0c},//
	{0x5687, 0x78},//
	{0x5a00, 0x08},//
	{0x5a21, 0x00},//
	{0x5a24, 0x00},//
#if 1
	//MTK darling_ff
	{0x5180, 0xff},//awb
	{0x5181, 0xf2},//
	{0x5182, 0x00},//
	{0x5183, 0x14},//
	{0x5184, 0x25},//
	{0x5185, 0x24},//
	{0x5186, 0x0e},//
	{0x5187, 0x16},//
	{0x5188, 0x0d},//
	{0x5189, 0x74},//0x75
	{0x518a, 0x54},//
	{0x518b, 0xd8},//0xcc
	{0x518c, 0x84},//
	{0x518d, 0x3b},//
	{0x518e, 0x31},//
	{0x518f, 0x52},//
	{0x5190, 0x3d},//
	{0x5191, 0xf8},//
	{0x5192, 0x04},//
	{0x5193, 0x70},//
	{0x5194, 0xf0},//
	{0x5195, 0xf0},//
	{0x5196, 0x03},//
	{0x5197, 0x01},//
	{0x5198, 0x05},//
	{0x5199, 0x18},//
	{0x519a, 0x04},//
	{0x519b, 0x00},//
	{0x519c, 0x07},//
	{0x519d, 0x33},//
	{0x519e, 0x38},//

    //CCM
	{0x5381, 0x21},//ccm
	{0x5382, 0x54},
	{0x5383, 0x15},
	{0x5384, 0x08},
	{0x5385, 0x75},
	{0x5386, 0x7D},
	{0x5387, 0x81},
	{0x5388, 0x74},
	{0x5389, 0x0D},
	{0x538a, 0x01},//
	{0x538b, 0x98},//

	//From OV Kobe sharpness
	{0x5308, 0x35},
	{0x5300, 0x08},//	; sharpen MT th1
	{0x5301, 0x48},//	; sharpen MT th2
	{0x5302, 0x20},//	; sharpen MT off1
	{0x5303, 0x00},//	; sharpen MT off2
	{0x5304, 0x08},//	; DNS th1
	{0x5305, 0x30},//	; DNS th2
	{0x5306, 0x10},//	; DNS off1 //0x08 0x10
	{0x5307, 0x16},//	; DNS off2 //0x16 0x20
	{0x5309, 0x08},//	; sharpen TH th1
	{0x530a, 0x30},//	; sharpen TH th2
	{0x530b, 0x04},//	; sharpen TH th2
	{0x530c, 0x06},//	; sharpen TH off2

	{0x5480, 0x01},//	; bias on
	{0x5481, 0x08},//	; Y yst 00
	{0x5482, 0x14},//
	{0x5483, 0x28},//
	{0x5484, 0x51},//
	{0x5485, 0x65},//
	{0x5486, 0x71},//
	{0x5487, 0x7d},//
	{0x5488, 0x87},//
	{0x5489, 0x91},//
	{0x548a, 0x9a},//
	{0x548b, 0xaa},//
	{0x548c, 0xb8},//
	{0x548d, 0xcd},//
	{0x548e, 0xdd},//
	{0x548f, 0xea},//
	{0x5490, 0x1d},//

    //DPC laimingji 20130815
	{0x5780, 0xfc},//
	{0x5781, 0x13},//
	{0x5782, 0x03},//
	{0x5786, 0x20},//
	{0x5787, 0x40},//
	{0x5788, 0x08},//
	{0x5789, 0x08},//
	{0x578a, 0x02},//
	{0x578b, 0x01},//
	{0x578c, 0x01},//
	{0x578d, 0x0c},//
	{0x578e, 0x02},//
	{0x578f, 0x01},//
	{0x5790, 0x01},//
	{0x5580, 0x06},//uv
	{0x5588, 0x01},//
	{0x5583, 0x40},//
	{0x5584, 0x20},//
	{0x5589, 0x18},//
	{0x558a, 0x00},//
	{0x558b, 0x3c},//

	//From OV Kobe DNP.
	{0x5800, 0x1C},//lsc
	{0x5801, 0x17},//
	{0x5802, 0x12},//
	{0x5803, 0x12},//
	{0x5804, 0x16},//
	{0x5805, 0x18},//
	{0x5806, 0x11},//
	{0x5807, 0x0A},//
	{0x5808, 0x06},//
	{0x5809, 0x06},//
	{0x580a, 0x08},//
	{0x580b, 0x0F},//
	{0x580c, 0x0A},//
	{0x580d, 0x05},//
	{0x580e, 0x00},//
	{0x580f, 0x00},//
	{0x5810, 0x03},//
	{0x5811, 0x0A},//
	{0x5812, 0x0E},//
	{0x5813, 0x05},//
	{0x5814, 0x01},//
	{0x5815, 0x00},//
	{0x5816, 0x04},//
	{0x5817, 0x0A},//
	{0x5818, 0x0C},//
	{0x5819, 0x0D},//
	{0x581a, 0x08},//
	{0x581b, 0x08},//
	{0x581c, 0x0B},//
	{0x581d, 0x12},//
	{0x581e, 0x3E},//
	{0x581f, 0x11},//
	{0x5820, 0x15},//
	{0x5821, 0x12},//
	{0x5822, 0x16},//
	{0x5823, 0x19},//
	{0x5824, 0x46},//
	{0x5825, 0x28},//
	{0x5826, 0x0A},//
	{0x5827, 0x26},//
	{0x5828, 0x2A},//
	{0x5829, 0x2A},//
	{0x582a, 0x26},//
	{0x582b, 0x24},//
	{0x582c, 0x26},//
	{0x582d, 0x26},//
	{0x582e, 0x0C},//
	{0x582f, 0x22},//
	{0x5830, 0x40},//
	{0x5831, 0x22},//
	{0x5832, 0x08},//
	{0x5833, 0x2A},//
	{0x5834, 0x28},//
	{0x5835, 0x06},//
	{0x5836, 0x26},//
	{0x5837, 0x26},//
	{0x5838, 0x26},//
	{0x5839, 0x28},//
	{0x583a, 0x2A},//
	{0x583b, 0x28},//
	{0x583c, 0x44},//
	{0x583d, 0xCE},//

	{0x583e, 0x10},//
	{0x583f, 0x08},//
	{0x5840, 0x00},//
	{0x5025, 0x00},//
	{0x3a00, 0x38},//	; ae mode	
	{0x3a0f, 0x29},//	; AEC in H
	{0x3a10, 0x23},//	; AEC in L
	{0x3a1b, 0x29},//	; AEC out H
	{0x3a1e, 0x23},//	; AEC out L
	{0x3a11, 0x53},//	; control zone H
	{0x3a1f, 0x12},//	; control zone L	
	{0x501d, 0x00},//	; control zone L

#endif
	{0x3008, 0x02},//	, wake up from software standby

	{0xffff, 0xff}
};
#else
#if 0
//OV provided , 20150717
static struct aml_camera_i2c_fig_s ov5645_script[] = {
{ 0x3008, 0x42},
{ 0x3103, 0x03},
{ 0x3503, 0x07},
{ 0x3002, 0x1c},
{ 0x3006, 0xc3},
{ 0x300e, 0x45},
{ 0x3017, 0x40},
{ 0x3018, 0x00},
{ 0x302e, 0x0b},
{ 0x3037, 0x13},
{ 0x3108, 0x01},
{ 0x3611, 0xc6},
{ 0x3612, 0x0b},
{ 0x3614, 0x50},
{ 0x3618, 0x00},
{ 0x3034, 0x18},
{ 0x3035, 0x12},
{ 0x3036, 0x50},
{ 0x3500, 0x00},
{ 0x3501, 0x01},
{ 0x3502, 0x00},
{ 0x350a, 0x00},
{ 0x350b, 0x3f},
{ 0x3600, 0x09},
{ 0x3601, 0x43},
{ 0x3620, 0x33},
{ 0x3621, 0xe0},
{ 0x3622, 0x01},
{ 0x3630, 0x2d},
{ 0x3631, 0x00},
{ 0x3632, 0x32},
{ 0x3633, 0x52},
{ 0x3634, 0x70},
{ 0x3635, 0x13},
{ 0x3636, 0x03},
{ 0x3702, 0x6e},
{ 0x3703, 0x52},
{ 0x3704, 0xa0},
{ 0x3705, 0x33},
{ 0x3708, 0x66},
{ 0x3709, 0x12},
{ 0x370b, 0x61},
{ 0x370c, 0xc3},
{ 0x370f, 0x10},
{ 0x3715, 0x08},
{ 0x3717, 0x01},
{ 0x371b, 0x20},
{ 0x3731, 0x22},
{ 0x3739, 0x70},
{ 0x3901, 0x0a},
{ 0x3905, 0x02},
{ 0x3906, 0x10},
{ 0x3719, 0x86},
{ 0x3800, 0x00},
{ 0x3801, 0x00},
{ 0x3802, 0x00},
{ 0x3803, 0x04},
{ 0x3804, 0x0a},
{ 0x3805, 0x3f},
{ 0x3806, 0x07},
{ 0x3807, 0x9b},
{ 0x3808, 0x02},
{ 0x3809, 0x80},
{ 0x380a, 0x01},
{ 0x380b, 0xe0},
{ 0x380c, 0x08},
{ 0x380d, 0x00},
{ 0x380e, 0x04},
{ 0x380f, 0x10},
{ 0x3810, 0x00},
{ 0x3811, 0x10},
{ 0x3812, 0x00},
{ 0x3813, 0x06},
{ 0x3814, 0x31},
{ 0x3815, 0x31},
{ 0x3820, 0x41},
{ 0x3821, 0x07},
{ 0x3824, 0x01},
{ 0x3826, 0x03},
{ 0x3828, 0x08},
{ 0x3a02, 0x03},
{ 0x3a03, 0xab},
{ 0x3a08, 0x01},
{ 0x3a09, 0x39},
{ 0x3a0a, 0x01},
{ 0x3a0b, 0x04},
{ 0x3a0e, 0x03},
{ 0x3a0d, 0x04},
{ 0x3a14, 0x03},
{ 0x3a15, 0xab},
{ 0x3a18, 0x00},
{ 0x3a19, 0xf8},
{ 0x3c01, 0x34},
{ 0x3c04, 0x28},
{ 0x3c05, 0x98},
{ 0x3c07, 0x07},
{ 0x3c09, 0xc2},
{ 0x3c0a, 0x9c},
{ 0x3c0b, 0x40},
{ 0x3c01, 0x84},
{ 0x3c00, 0x04},
{ 0x4001, 0x02},
{ 0x4004, 0x02},
{ 0x4005, 0x18},
{ 0x4050, 0x6e},
{ 0x4051, 0x8f},
{ 0x4300, 0x32},
{ 0x4514, 0x00},
{ 0x4520, 0xb0},
{ 0x460b, 0x37},
{ 0x460c, 0x20},
{ 0x4818, 0x01},
{ 0x481d, 0xf0},
{ 0x481f, 0x50},
{ 0x4823, 0x70},
{ 0x4831, 0x14},
{ 0x4837, 0x20},
{ 0x4800, 0x24},
{ 0x5000, 0xa7},
{ 0x5001, 0xa3},
{ 0x501d, 0x00},
{ 0x501f, 0x00},
{ 0x503d, 0x00},
{ 0x505c, 0x30},
{ 0x5181, 0x59},
{ 0x5183, 0x00},
{ 0x5191, 0xf0},
{ 0x5192, 0x03},
{ 0x5684, 0x10},
{ 0x5685, 0xa0},
{ 0x5686, 0x0c},
{ 0x5687, 0x78},
{ 0x5a00, 0x08},
{ 0x5a21, 0x00},
{ 0x5a24, 0x00},
{ 0x3008, 0x02},
{ 0x3503, 0x00},
{ 0x5180, 0xff},
{ 0x5181, 0xf2},
{ 0x5182, 0x00},
{ 0x5183, 0x14},
{ 0x5184, 0x25},
{ 0x5185, 0x24},
{ 0x5186, 0x16},
{ 0x5187, 0x16},
{ 0x5188, 0x16},
{ 0x5189, 0x64},
{ 0x518a, 0x5c},
{ 0x518b, 0xe0},
{ 0x518c, 0xb2},
{ 0x518d, 0x42},
{ 0x518e, 0x30},
{ 0x518f, 0x4c},
{ 0x5190, 0x56},
{ 0x5191, 0xf8},
{ 0x5192, 0x04},
{ 0x5193, 0x70},
{ 0x5194, 0xf0},
{ 0x5195, 0xf0},
{ 0x5196, 0x03},
{ 0x5197, 0x01},
{ 0x5198, 0x04},
{ 0x5199, 0x12},
{ 0x519a, 0x04},
{ 0x519b, 0x00},
{ 0x519c, 0x06},
{ 0x519d, 0x82},
{ 0x519e, 0x38},
{ 0x5381, 0x1e},
{ 0x5382, 0x5b},
{ 0x5383, 0x12},
{ 0x5384, 0x05},
{ 0x5385, 0x6f},
{ 0x5386, 0x74},
{ 0x5387, 0x78},
{ 0x5388, 0x6e},
{ 0x5389, 0x0a},
{ 0x538a, 0x01},
{ 0x538b, 0x98},
{ 0x5300, 0x08},
{ 0x5301, 0x30},
{ 0x5302, 0x28},
{ 0x5303, 0x10},
{ 0x5304, 0x08},
{ 0x5305, 0x30},
{ 0x5306, 0x0a},
{ 0x5307, 0x1a},
{ 0x5309, 0x08},
{ 0x530a, 0x30},
{ 0x530b, 0x04},
{ 0x530c, 0x06},
{ 0x5480, 0x01},
{ 0x5481, 0x06},
{ 0x5482, 0x0c},
{ 0x5483, 0x24},
{ 0x5484, 0x4a},
{ 0x5485, 0x58},
{ 0x5486, 0x65},
{ 0x5487, 0x72},
{ 0x5488, 0x7d},
{ 0x5489, 0x88},
{ 0x548a, 0x92},
{ 0x548b, 0xa3},
{ 0x548c, 0xb2},
{ 0x548d, 0xc8},
{ 0x548e, 0xdd},
{ 0x548f, 0xf0},
{ 0x5490, 0x15},
{ 0x5580, 0x06},
{ 0x5583, 0x40},
{ 0x5584, 0x18},
{ 0x5589, 0x10},
{ 0x558a, 0x00},
{ 0x558b, 0xf8},
{ 0x5800, 0x28},
{ 0x5801, 0x20},
{ 0x5802, 0x18},
{ 0x5803, 0x18},
{ 0x5804, 0x20},
{ 0x5805, 0x28},
{ 0x5806, 0x17},
{ 0x5807, 0x0C},
{ 0x5808, 0x08},
{ 0x5809, 0x08},
{ 0x580A, 0x0C},
{ 0x580B, 0x17},
{ 0x580C, 0x0C},
{ 0x580D, 0x05},
{ 0x580E, 0x01},
{ 0x580F, 0x01},
{ 0x5810, 0x05},
{ 0x5811, 0x0C},
{ 0x5812, 0x0C},
{ 0x5813, 0x05},
{ 0x5814, 0x01},
{ 0x5815, 0x01},
{ 0x5816, 0x05},
{ 0x5817, 0x0C},
{ 0x5818, 0x16},
{ 0x5819, 0x0B},
{ 0x581A, 0x07},
{ 0x581B, 0x08},
{ 0x581C, 0x0C},
{ 0x581D, 0x17},
{ 0x581E, 0x27},
{ 0x581F, 0x1F},
{ 0x5820, 0x18},
{ 0x5821, 0x18},
{ 0x5822, 0x20},
{ 0x5823, 0x28},//;2a;29
{ 0x5824, 0x16},//;18;17
{ 0x5825, 0x1a},//;1c;1B
{ 0x5826, 0x1C},//
{ 0x5827, 0x1a},//;1c;1B
{ 0x5828, 0x26},//
{ 0x5829, 0x1A},//
{ 0x582A, 0x26},//;28;27
{ 0x582B, 0x34},//;36;35
{ 0x582C, 0x26},//;28;27
{ 0x582D, 0x1A},//
{ 0x582E, 0x18},//;1a;19
{ 0x582F, 0x42},//
{ 0x5830, 0x40},//
{ 0x5831, 0x42},//
{ 0x5832, 0x18},//;1a;19
{ 0x5833, 0x2A},//
{ 0x5834, 0x26},//;28;27
{ 0x5835, 0x26},//
{ 0x5836, 0x26},//;28;27
{ 0x5837, 0x28},//;2a;29
{ 0x5838, 0x16},//;18;17
{ 0x5839, 0x1A},//
{ 0x583A, 0x1A},//
{ 0x583B, 0x1A},//
{ 0x583C, 0x26},//;28;27
{ 0x583D, 0xCE},
{ 0x5025, 0x00},
{ 0x3a0f, 0x30},
{ 0x3a10, 0x28},
{ 0x3a11, 0x60},
{ 0x3a1b, 0x30},
{ 0x3a1e, 0x28},
{ 0x3a1f, 0x10},
{ 0x3008, 0x02},
{ 0xffff, 0xff},
};
#else 
//below is OV provided. 20150713
static struct aml_camera_i2c_fig_s ov5645_script[] = {
	//@@ MIPI_2lane_(YUV) 960p  30fps
	{0x3031, 0x00},
	{0x302c, 0x42},
	{0x3008, 0x42},
	{0x3103, 0x03},
	{0x4050, 0x6e},
	{0x4051, 0x7f},
	{0x3017, 0x40},
	{0x3018, 0x00},
	{0x3503, 0x00}, //0x07
	{0x3500, 0x00}, 
	{0x3501, 0x3d},
	{0x3502, 0x80},
	{0x350a, 0x00},
	{0x350b, 0x3c},
	{0x3611, 0x06},
	{0x3614, 0x50},
	{0x3702, 0x6e},
	{0x370f, 0x10},
	{0x3739, 0x70},
	{0x3719, 0x86},
	{0x3826, 0x03},
	{0x3828, 0x08},
	{0x4520, 0xb0},
	{0x4800, 0x24},//	{0x4800, 0x04}chage mipi data free/gate
	{0x4818, 0x01},
	{0x481d, 0xf0},
	{0x481f, 0x50}, //0x50
	{0x4823, 0x70},
	{0x4831, 0x14},
	{0x505c, 0x30},
	{0x5a00, 0x08},
	{0x5a21, 0x00},
	{0x5a24, 0x00},
	{0x3108, 0x01},
	{0x3630, 0x2d},
	{0x3631, 0x00},
	{0x3632, 0x32},
	{0x3633, 0x52},
	{0x3621, 0xe0},
	{0x3704, 0xa0},
	{0x3703, 0x52},
	{0x3715, 0x08},
	{0x3717, 0x01},
	{0x370b, 0x61},
	{0x3705, 0x33},
	{0x3905, 0x02},
	{0x3906, 0x10},
	{0x3901, 0x0a},
	{0x3731, 0x22},
	{0x3600, 0x09},
	{0x3601, 0x43},
	{0x3620, 0x33},
	{0x371b, 0x20},
	{0x3a18, 0x00},
	{0x3a19, 0xc0},//f8
	{0x3635, 0x13},
	{0x3636, 0x03},
	{0x3634, 0x70},
	{0x3622, 0x01},
	{0x3c04, 0x28},
	{0x3c05, 0x98},
	{0x3c07, 0x07},
	{0x3c09, 0xc2},
	{0x3c0a, 0x9c},
	{0x3c0b, 0x40},
	{0x3004, 0xef},
	{0x3820, 0x41},
	{0x3821, 0x07},
	{0x4514, 0x00},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x06},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0x9d},
	{0x3808, 0x05},
	{0x3809, 0x00},
	{0x380a, 0x03},
	{0x380b, 0xc0},
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x06},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3034, 0x18},
	{0x3035, 0x11},
	{0x3036, 0x38},
	{0x3037, 0x13},
	{0x380c, 0x07},
	{0x380d, 0x68},
	{0x380e, 0x03},
	{0x380f, 0xd8},
	{0x3c01, 0xb4},
	{0x3c00, 0x04},
	{0x3a08, 0x01},
	{0x3a09, 0x27},
	{0x3a0e, 0x03},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0d, 0x04},
	{0x3a00, 0x3c},
	{0x3a02, 0x0b},
	{0x3a03, 0x88},
	{0x3a14, 0x0b},
	{0x3a15, 0x88},
	{0x3a17, 0x02},
	{0x3618, 0x00},
	{0x3612, 0xab},
	{0x3708, 0x66},
	{0x3709, 0x52},
	{0x370c, 0xc3},
	{0x4001, 0x02},
	{0x4004, 0x02},
	{0x3002, 0x1c},
	{0x3006, 0xc3},
	{0x300e, 0x45},
	{0x302e, 0x0b},
	{0x4300, 0x32}, //YUV order
	{0x501f, 0x00},
	{0x460b, 0x37},
	{0x460c, 0x20},
	{0x4837, 0x20},
	{0x3824, 0x01},
	{0x5001, 0xa3},
	{0x5180, 0xff},//0xff   
	{0x5181, 0xf2},
	{0x5182, 0x00},
	{0x5183, 0x14},
	{0x5184, 0x25},
	{0x5185, 0x24},
	{0x5186, 0x15},
	{0x5187, 0x20},
	{0x5188, 0x16},
	{0x5189, 0x71},//64//5e
	{0x518a, 0x5c},//5c
	{0x518b, 0xf9},
	{0x518c, 0xb2},
	{0x518d, 0x3f},
	{0x518e, 0x33},
	{0x518f, 0x5c},
	{0x5190, 0x48},
	{0x5191, 0xf8},
	{0x5192, 0x04},
	{0x5193, 0x70},
	{0x5194, 0xf0},
	{0x5195, 0xf0},
	{0x5196, 0x03},
	{0x5197, 0x01},
	{0x5198, 0x04},
	{0x5199, 0xd6},
	{0x519a, 0x04},
	{0x519b, 0x00},
	{0x519c, 0x07},
	{0x519d, 0x03},
	{0x519e, 0x38},
	{0x5180, 0xff},//0xff   
	{0x5181, 0xf2},
	{0x5182, 0x00},
	{0x5183, 0x14},
	{0x5184, 0x25},
	{0x5185, 0x24},
	{0x5186, 0x15},//1
	{0x5187, 0x20},// 16
	{0x5188, 0x22},
	{0x5189, 0x75},//5e
	{0x518a, 0x5e},//5c
	{0x518b, 0xf9},//e4
	{0x518c, 0xb2},//dd
	{0x518d, 0x3f},
	{0x518e, 0x31}, // 40 2e
	{0x518f, 0x61},
	{0x5190, 0x4a}, //43
	{0x5191, 0xf8},
	{0x5192, 0x04},
	{0x5193, 0x70},
	{0x5194, 0xf0},
	{0x5195, 0xf0},
	{0x5196, 0x03},
	{0x5197, 0x01},
	{0x5198, 0x06},
	{0x5199, 0xc2},
	{0x519a, 0x04},
	{0x519b, 0x00},
	{0x519c, 0x05},
	{0x519d, 0x8c},
	{0x519e, 0x38},
	{0x5381, 0x1a},              
	{0x5382, 0x68},
	{0x5383, 0x01},
	{0x5384, 0x1f},
	{0x5385, 0x5a},
	{0x5386, 0x79},
	{0x5387, 0x8d},
	{0x5388, 0x74},
	{0x5389, 0x18},
	{0x538a, 0x01},
	{0x538b, 0x9C},
	{0x5480, 0x01},
	{0x5481, 0x08},
	{0x5482, 0x14},
	{0x5483, 0x28},
	{0x5484, 0x51},
	{0x5485, 0x65},
	{0x5486, 0x71},
	{0x5487, 0x7d},
	{0x5488, 0x87},
	{0x5489, 0x91},
	{0x548a, 0x9a},
	{0x548b, 0xaa},
	{0x548c, 0xb8},
	{0x548d, 0xcd},
	{0x548e, 0xdd},
	{0x548f, 0xea},
	{0x5490, 0x1d},
	{0x5300, 0x08},
	{0x5301, 0x30},
	{0x5302, 0x3f},
	{0x5303, 0x10},
	{0x5304, 0x08},
	{0x5305, 0x30},
	{0x5306, 0x04},
	{0x5307, 0x14},
	{0x5309, 0x08},
	{0x530a, 0x30},
	{0x530b, 0x04},
	{0x530c, 0x06},
	{0x5580, 0x06},
	{0x5583, 0x40},
	{0x5584, 0x10},//20//10
	{0x5589, 0x10},
	{0x558a, 0x00},
	{0x558b, 0xf8},
	{0x5000, 0xa7},
	{0x5800, 0x28},
	{0x5801, 0x20},
	{0x5802, 0x18},
	{0x5803, 0x18},
	{0x5804, 0x20},
	{0x5805, 0x28},
	{0x5806, 0x17},
	{0x5807, 0x0C},
	{0x5808, 0x08},
	{0x5809, 0x08},
	{0x580A, 0x0C},
	{0x580B, 0x17},
	{0x580C, 0x0C},
	{0x580D, 0x05},
	{0x580E, 0x01},
	{0x580F, 0x01},
	{0x5810, 0x05},
	{0x5811, 0x0C},
	{0x5812, 0x0C},
	{0x5813, 0x05},
	{0x5814, 0x01},
	{0x5815, 0x01},
	{0x5816, 0x05},
	{0x5817, 0x0C},
	{0x5818, 0x16},
	{0x5819, 0x0B},
	{0x581A, 0x07},
	{0x581B, 0x08},
	{0x581C, 0x0C},
	{0x581D, 0x17},
	{0x581E, 0x27},
	{0x581F, 0x1F},
	{0x5820, 0x18},
	{0x5821, 0x18},
	{0x5822, 0x20},
	{0x5823, 0x28},//;2a;29
	{0x5824, 0x16},//;18;17
	{0x5825, 0x1a},//;1c;1B
	{0x5826, 0x1C},//
	{0x5827, 0x1a},//;1c;1B
	{0x5828, 0x26},//
	{0x5829, 0x1A},//
	{0x582A, 0x26},//;28;27
	{0x582B, 0x34},//;36;35
	{0x582C, 0x26},//;28;27
	{0x582D, 0x1A},//
	{0x582E, 0x18},//;1a;19
	{0x582F, 0x42},//
	{0x5830, 0x40},//
	{0x5831, 0x42},//
	{0x5832, 0x18},//;1a;19
	{0x5833, 0x2A},//
	{0x5834, 0x26},//;28;27
	{0x5835, 0x26},//
	{0x5836, 0x26},//;28;27
	{0x5837, 0x28},//;2a;29
	{0x5838, 0x16},//;18;17
	{0x5839, 0x1A},//
	{0x583A, 0x1A},//
	{0x583B, 0x1A},//
	{0x583C, 0x26},//;28;27
	{0x583D, 0xCE},//
	{0x5688, 0x11},
	{0x5689, 0x11},
	{0x568a, 0x11},
	{0x568b, 0x11},
	{0x568c, 0x11},
	{0x568d, 0x11},
	{0x568e, 0x11},
	{0x568f, 0x11},
	{0x5025, 0x00},
	{0x3a0f, 0x28}, //30  //28
	{0x3a10, 0x23}, //28  //20
	{0x3a11, 0x51}, //61  //51
	{0x3a1b, 0x28}, //30  //28
	{0x3a1e, 0x23}, //28  //20
	{0x3a1f, 0x10},   
	{0x3a17, 0x02},
	{0x4005, 0x18},
	{0x3503, 0x00},
	{0x3008, 0x02},	
	{0x501d, 0x00},
	{0x5780, 0xfc},
	{0x5781, 0x13}, 
	{0x5782, 0x03},
	{0x5786, 0x20},  
	{0x5787, 0x40},
	{0x5788, 0x08}, 
	{0x5789, 0x08},
	{0x578a, 0x02}, 
	{0x578b, 0x01},
	{0x578c, 0x01}, 
	{0x578d, 0x0c},
	{0x578e, 0x02}, 
	{0x578f, 0x01},
	{0x5790, 0x01},

	{ 0xffff, 0xff}
};
#endif
#endif

#if 0
//below is MTK used.
static struct aml_camera_i2c_fig_s ov5645_preview_VGA_script[] = {
	//;OV5645MIPI 1280x960,30fps
	//56Mhz, 224Mbps/Lane, 2Lane.
	{0x3008, 0x42}, //	; software power down

	//{0x4202, 0x0f}, //	; stop mipi stream
	{0x300e, 0x45}, //	; MIPI 2 lane
	{0x3034, 0x18}, // PLL, MIPI 8-bit mode
	{0x3035, 0x21}, // PLL
	{0x3036, 0x70}, // PLL
	{0x3037, 0x13}, // PLL
	{0x3108, 0x01}, // PLL
	{0x3824, 0x01}, // PLL
	{0x460c, 0x20}, // PLL
	{0x3618, 0x00}, //
	{0x3600, 0x09}, //
	{0x3601, 0x43}, //
	{0x3708, 0x66}, //
	{0x3709, 0x12}, //
	{0x370c, 0xc3}, //
	{0x3800, 0x00},  // HS = 0
	{0x3801, 0x00},  // HS
	{0x3802, 0x00},  // VS = 250
	{0x3803, 0x06},  // VS
	{0x3804, 0x0a},  // HW = 2623
	{0x3805, 0x3f}, //	; HW
	{0x3806, 0x07}, //	; VH = 
	{0x3807, 0x9d}, //	; VH
	{0x3808, 0x02}, //	; DVPHO =  640
	{0x3809, 0x80}, //	; DVPHO
	{0x380a, 0x01}, //	; DVPVO = 480
	{0x380b, 0xe0}, //	; DVPVO
	{0x380c, 0x07}, //	; HTS = 2160
	{0x380d, 0x68}, //	; HTS
	{0x380e, 0x03}, //	; VTS = 740
	{0x380f, 0xd8}, //	; VTS
	{0x3810, 0x00},  // H OFF = 16
	{0x3811, 0x10},  // H OFF
	{0x3812, 0x00},  // V OFF = 4
	{0x3813, 0x06}, //	; V OFF
	{0x3814, 0x31}, //	; X INC
	{0x3815, 0x31}, //	; Y INC
	{0x3820, 0x47}, //	; flip off, V bin on
	{0x3821, 0x01}, //	; mirror on, H bin on
	{0x4514, 0x00}, 
 
	{0x3a02, 0x0b}, //	; max exp 60 = 740
	{0x3a03, 0x88}, //	; max exp 60
	{0x3a14, 0x0b}, //	; max exp 50 = 740
	{0x3a15, 0x88}, //	; max exp 50

	{0x3a08, 0x01}, //	; B50 = 222
	{0x3a09, 0x27}, //	; B50
	{0x3a0a, 0x00}, //	; B60 = 185
	{0x3a0b, 0xf6}, //	; B60
	{0x3a0e, 0x03}, //	; max 50
	{0x3a0d, 0x04}, //	; max 60
	{0x3c07, 0x07}, //	; 50/60 auto detect
	{0x3c08, 0x01}, //	; 50/60 auto detect
	{0x3c09, 0xc2}, //	; 50/60 auto detect
	{0x4004, 0x02}, //	; BLC line number
	{0x4005, 0x18}, //	; BLC triggered by gain change
	{0x4837, 0x11},  // MIPI global timing 16           
	{0x503d, 0x00}, //
	{0x5000, 0xa7}, //
	{0x5001, 0xa3}, //
	{0x5002, 0x80}, //
	{0x5003, 0x08}, //
	{0x3032, 0x00}, //
	{0x4000, 0x89}, //
	{0x3a00, 0x3c}, //	; ae mode	
 
	//dgt added
	{0x3400, 0x05 },
	{0x3401, 0x9c },
	{0x3402, 0x04 },
	{0x3403, 0x00 },
	{0x3404, 0x06 },
	{0x3405, 0x37 },

	{0x3008, 0x02}, //	; software power up

	{0xffff, 0xff}
};
#else
//below is OV provided, 20150713
static struct aml_camera_i2c_fig_s ov5645_preview_VGA_script[] = {
	//640x480

	{0x3008, 0x42},	                                       
	{0x4050, 0x6e},       
	{0x4051, 0x8f},                                                             
	{0x5302, 0x1d},//     
	{0x5303, 0x00},//     
	{0x5306, 0x08},// dns 
	{0x5307, 0x1f},// dns                                                       
	{0x3600, 0x09},       
	{0x3601, 0x43},       
	{0x3820, 0x47},//0x41  
	{0x3821, 0x01},//0x07  	                                                  
	{0x4514, 0x00},                                                            
	{0x3800, 0x00},       
	{0x3801, 0x00},       
	{0x3802, 0x00},       
	{0x3803, 0x06},       
	{0x3804, 0x0a},       
	{0x3805, 0x3f},       
	{0x3806, 0x07},       
	{0x3807, 0x9d},       
	{0x3808, 0x02},       
	{0x3809, 0x80},       
	{0x380a, 0x01},       
	{0x380b, 0xe0},	      
	{0x3810, 0x00},       
	{0x3811, 0x10},       
	{0x3812, 0x00},       
	{0x3813, 0x06},       
	{0x3814, 0x31},       
	{0x3815, 0x31},                                                              
	{0x3034, 0x18},       
	{0x3035, 0x11},       
	{0x3036, 0x38},       
	{0x3037, 0x13},                                                             
	{0x380c, 0x07},       
	{0x380d, 0x68},       
	{0x380e, 0x03},       
	{0x380f, 0xd8},                                                            
	{0x3a08, 0x01},       
	{0x3a09, 0x27},       
	{0x3a0e, 0x03},       
	{0x3a0a, 0x00},       
	{0x3a0b, 0xf6},	      
	{0x3a0d, 0x04},                                                             
	{0x3618, 0x00},	      
	{0x3612, 0xab},       
	{0x3708, 0x66},       
	{0x3709, 0x52},       
	{0x370c, 0xc3},                                                             
	{0x4004, 0x02},		     
	{0x460b, 0x37},       
	{0x460c, 0x20},       
	{0x4837, 0x11},	      
	{0x3824, 0x01},       
	{0x5001, 0x83},       
	{0x5002, 0x80},       
	{0x5003, 0x08},       
	{0x3032, 0x00},                                                             
	{0x3008, 0x02}, 

	{ 0xffff, 0xff },
};
#endif
//#define FPS_30_HZ_960P
//#define FPS_20_HZ_960P
#define FPS_15_HZ_960P
//#define FPS_10_HZ_960P

#if 1
#if 1
//below is OV provided. 20150713
static struct aml_camera_i2c_fig_s ov5645_preview_960P_30HZ_script[] = {
//1280x960 30fps
	{0x3008, 0x42},	                                       

	{0x3a00, 0x38},
	{0x4050, 0x6e},       
	{0x4051, 0x8f},                                                             
	{0x5302, 0x1d},//
	{0x5303, 0x00},//
	{0x5306, 0x08},// dns
	{0x5307, 0x1f},// dns
	{0x3600, 0x09},       
	{0x3601, 0x43},       
	{0x3820, 0x47},//0x41  
	{0x3821, 0x01},//0x07  	                                                  
	{0x4514, 0x00},                                                            
	{0x3800, 0x00},       
	{0x3801, 0x00},       
	{0x3802, 0x00},       
	{0x3803, 0x06},       
	{0x3804, 0x0a},       
	{0x3805, 0x3f},       
	{0x3806, 0x07},       
	{0x3807, 0x9d},       
	{0x3808, 0x05},       
	{0x3809, 0x00}, //0x00      
	{0x380a, 0x03},       
	{0x380b, 0xc0}, //0xc0	      
	{0x3810, 0x00},       
	{0x3811, 0x10},       
	{0x3812, 0x00},       
	{0x3813, 0x06},       
	{0x3814, 0x31},       
	{0x3815, 0x31},                                                              
	{0x3034, 0x18},       
	{0x3035, 0x11},       
	{0x3036, 0x38},       
	{0x3037, 0x13},                                                             
	{0x380c, 0x07},       
	{0x380d, 0x68},       
	{0x380e, 0x03},       
	{0x380f, 0xd8},                                                            
	{0x3a08, 0x01},       
	{0x3a09, 0x27},       
	{0x3a0e, 0x03},       
	{0x3a0a, 0x00},       
	{0x3a0b, 0xf6},	      
	{0x3a0d, 0x04},                                                             
	{0x3618, 0x00},	      
	{0x3612, 0xab},       
	{0x3708, 0x66},       
	{0x3709, 0x52},       
	{0x370c, 0xc3},                                                             
	{0x4004, 0x02},		     
	{0x460b, 0x37},       
	{0x460c, 0x20},       
	{0x4837, 0x11},	      
	{0x3824, 0x01},       
	{0x5001, 0x83},       
	{0x5002, 0x80},       
	{0x5003, 0x08},       
	{0x3032, 0x00},                                                             
	{0x3008, 0x02}, 

	{0x3503, 0x00}, // auto AE
	{0x3406, 0x00},// auto AWB

	{0xffff, 0xff}
};

//SL-20160421-Fix1080PFlicker-01*{
static struct aml_camera_i2c_fig_s ov5645_preview_1080P_15HZ_script[] = {
//1920x1080 15fps
	{0x4202, 0x08},//stop mipi stream
	
	//{0x3008, 0x42},// software power down

	{0x3a00, 0x38},
	{0x4050, 0x6e},
	{0x4051, 0x8f},
	{0x5302, 0x1c},
	{0x5303, 0x06},//08
	{0x5306, 0x06},
	{0x5307, 0x5f},//16
	{0x3600, 0x08},
	{0x3601, 0x33},
	{0x3820, 0x46},//46
	{0x3821, 0x00},//00	
	{0x4514, 0x00},//88
	{0x3800, 0x00}, 
	{0x3801, 0x00}, 
	{0x3802, 0x00}, 
	{0x3803, 0x00}, 
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0x9f},
	{0x3808, 0x0a}, //0x0a HO= 2592
	{0x3809, 0x20}, //0x20
	{0x380a, 0x05}, //VO=1464
	{0x380b, 0xb8}, //	
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0xf4}, //0x04 //
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3034, 0x18},
	{0x3035, 0x11},//0x11=15fps,0x21=7.5fps
	{0x3036, 0x5a}, //0x54
	{0x3037, 0x13},
	{0x380c, 0x0b},
	{0x380d, 0xec}, //0xec 0x1c //here is debuged
	{0x380e, 0x07},
	{0x380f, 0xb0},
	{0x3a08, 0x01},
	{0x3a09, 0x27},
	{0x3a0e, 0x06},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},	
	{0x3a0d, 0x08},
	{0x3618, 0x04},
	{0x3612, 0xab}, 
	{0x3708, 0x63},
	{0x3709, 0x12},
	{0x370c, 0xc0},
	{0x4004, 0x06},
	{0x460c, 0x20}, 
	{0x4837, 0x16}, //0x16 here is debuged 20150720	
	{0x3824, 0x01}, 
	{0x5001, 0xa3}, //0x83
	{0x5002, 0x81},
	{0x5003, 0x08},
	{0x3032, 0x00},

	{0x4202, 0x00}, //open mipi steam
	
	//{0x3008, 0x02}, //	; software power up

	{0xffff, 0xff},

};
//SL-20160421-Fix1080PFlicker-01*}

#else
//OV provided 20170717
static struct aml_camera_i2c_fig_s ov5645_preview_960P_30HZ_script[] = {
{ 0x3618, 0x00},
{ 0x3034, 0x18},
{ 0x3035, 0x14},
{ 0x3036, 0x68},
{ 0x3037, 0x13},
{ 0x3108, 0x01},
{ 0x3824, 0x04},
{ 0x460C, 0x20},
{ 0x300E, 0x45},
{ 0x3600, 0x09},
{ 0x3601, 0x43},
{ 0x3708, 0x66},
{ 0x370c, 0xc3},
{ 0x3800, 0x00},
{ 0x3801, 0x00},
{ 0x3802, 0x00},
{ 0x3803, 0x04},
{ 0x3804, 0x0a},
{ 0x3805, 0x3f},
{ 0x3806, 0x07},
{ 0x3807, 0x9b},
{ 0x3808, 0x05},
{ 0x3809, 0x00},
{ 0x380a, 0x03},
{ 0x380b, 0xc0},
{ 0x380c, 0x09},
{ 0x380d, 0x00},
{ 0x380e, 0x05},
{ 0x380f, 0x10},
{ 0x3813, 0x06},
{ 0x3814, 0x31},
{ 0x3815, 0x31},
{ 0x3820, 0x41},
{ 0x3821, 0x07},
{ 0x3a08, 0x01},
{ 0x3a09, 0x6c},
{ 0x3a0a, 0x01},
{ 0x3a0b, 0x58},
{ 0x3a0e, 0x03},
{ 0x3a0d, 0x04},
{ 0x3a00, 0x3c},
{ 0x3a02, 0x0e},
{ 0x3a03, 0x1b},
{ 0x3a14, 0x0e},
{ 0x3a15, 0x1b},
{ 0x3a18, 0x00},
{ 0x3a19, 0xf8},
{ 0x4004, 0x02},
{ 0x4005, 0x18},
{ 0x4837, 0x20},
{ 0x3503, 0x00},
};
#endif

#else

//below is MTK used.
static struct aml_camera_i2c_fig_s ov5645_preview_960P_30HZ_script[] = {
	//;OV5645MIPI 1280x960,30fps
	//56Mhz, 224Mbps/Lane, 2Lane.
	{0x3008, 0x42}, //	; software power down

	{0x4202, 0x0f}, //	; stop mipi stream
	{0x300e, 0x45}, //	; MIPI 2 lane
	{0x3034, 0x18}, // PLL, MIPI 8-bit mode
	{0x3035, 0x21}, // PLL
	{0x3036, 0x70}, // PLL
	{0x3037, 0x13}, // PLL
	{0x3108, 0x01}, // PLL
	{0x3824, 0x01}, // PLL
	{0x460c, 0x20}, // PLL
	{0x3618, 0x00}, //
	{0x3600, 0x09}, //
	{0x3601, 0x43}, //
	{0x3708, 0x66}, //
	{0x3709, 0x52}, // 0x12
	{0x370c, 0xc3}, //
	{0x3800, 0x00},  // HS = 0
	{0x3801, 0x00},  // HS
	{0x3802, 0x00},  // VS = 250
	{0x3803, 0x06},  // VS
	{0x3804, 0x0a},  // HW = 2623
	{0x3805, 0x3f}, //	; HW
	{0x3806, 0x07}, //	; VH = 
	{0x3807, 0x9d}, //	; VH
	{0x3808, 0x05}, //	; DVPHO = 1280
	{0x3809, 0x00}, //	; DVPHO
	{0x380a, 0x03}, //	; DVPVO = 960
	{0x380b, 0xc0}, //	; DVPVO
	{0x380c, 0x07}, //	; HTS = 2160
	{0x380d, 0x68}, //	; HTS
	{0x380e, 0x03}, //	; VTS = 740
	{0x380f, 0xd8}, //	; VTS
	{0x3810, 0x00},  // H OFF = 16
	{0x3811, 0x10},  // H OFF
	{0x3812, 0x00},  // V OFF = 4
	{0x3813, 0x06}, //	; V OFF
	{0x3814, 0x31}, //	; X INC
	{0x3815, 0x31}, //	; Y INC
	{0x3820, 0x47}, //	; flip off, V bin on
	{0x3821, 0x01}, //	; mirror on, H bin on
	{0x4514, 0x00}, 
 
	{0x3a02, 0x0b}, //	; max exp 60 = 740
	{0x3a03, 0x88}, //	; max exp 60
	{0x3a14, 0x0b}, //	; max exp 50 = 740
	{0x3a15, 0x88}, //	; max exp 50

	{0x3a08, 0x01}, //	; B50 = 222
	{0x3a09, 0x27}, //	; B50
	{0x3a0a, 0x00}, //	; B60 = 185
	{0x3a0b, 0xf6}, //	; B60
	{0x3a0e, 0x03}, //	; max 50
	{0x3a0d, 0x04}, //	; max 60
	{0x3c07, 0x07}, //	; 50/60 auto detect
	{0x3c08, 0x01}, //	; 50/60 auto detect
	{0x3c09, 0xc2}, //	; 50/60 auto detect
	{0x4004, 0x02}, //	; BLC line number
	{0x4005, 0x18}, //	; BLC triggered by gain change
	{0x4837, 0x11},  // MIPI global timing 16           
	{0x503d, 0x00}, //
	{0x5000, 0xa7}, //
	{0x5001, 0xa3}, // 0x83
	{0x5002, 0x80}, //
	{0x5003, 0x08}, //
	{0x3032, 0x00}, //
	{0x4000, 0x89}, //
	{0x3a00, 0x38}, //	; ae mode	
	{0x3503, 0x00}, // auto AE
	{0x3406, 0x00},// auto AWB
 
	{0x3008, 0x02}, //	; software power up

	{0xffff, 0xff}
};

#endif

#if 1

//below OV provide. 20150713
static struct aml_camera_i2c_fig_s ov5645_capture_5M_script[]  = {
	{0x4202, 0x08},//stop mipi stream
	
	//{0x3008, 0x42},// software power down

	{0x3a00, 0x38},
	{0x4050, 0x6e},
	{0x4051, 0x8f},
	{0x5302, 0x1c},
	{0x5303, 0x06},//08
	{0x5306, 0x06},
	{0x5307, 0x5f},//16
	{0x3600, 0x08},
	{0x3601, 0x33},
	{0x3820, 0x46},//46
	{0x3821, 0x00},//00	
	{0x4514, 0x00},//88
	{0x3800, 0x00}, 
	{0x3801, 0x00}, 
	{0x3802, 0x00}, 
	{0x3803, 0x00}, 
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0x9f},
	{0x3808, 0x0a}, //0x0a
	{0x3809, 0x20}, //0x20
	{0x380a, 0x07}, //0x07
	{0x380b, 0x98}, //0x98	
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x04}, //0x04
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3034, 0x18},
	{0x3035, 0x11},//0x11=15fps,0x21=7.5fps
	{0x3036, 0x5a}, //0x54
	{0x3037, 0x13},
	{0x380c, 0x0b},
	{0x380d, 0xec}, //0xec 0x1c //here is debuged
	{0x380e, 0x07},
	{0x380f, 0xb0},
	{0x3a08, 0x01},
	{0x3a09, 0x27},
	{0x3a0e, 0x06},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},	
	{0x3a0d, 0x08},
	{0x3618, 0x04},
	{0x3612, 0xab}, 
	{0x3708, 0x63},
	{0x3709, 0x12},
	{0x370c, 0xc0},
	{0x4004, 0x06},
	{0x460c, 0x20}, 
	{0x4837, 0x16}, //0x16 here is debuged 20150720	
	{0x3824, 0x01}, 
	{0x5001, 0xa3}, //0x83
	{0x5002, 0x81},
	{0x5003, 0x08},
	{0x3032, 0x00},

	{0x4202, 0x00}, //open mipi steam
	
	//{0x3008, 0x02}, //	; software power up

	{0xffff, 0xff},
};
#else

// OV provided,reset sensor and set it to 500M, 20150721, can photo
static struct aml_camera_i2c_fig_s ov5645_capture_5M_script[] = {
	//OV5645MIPI 2592x1944,10fps
	//90Mhz, 360Mbps/Lane, 2Lane.15
	{0x3103, 0x11}, //reset sensor
	{0x3008, 0x82}, //reset sensor
	
	{0x3008, 0x42},
	{0x3103, 0x03},
	{0x3503, 0x07},
	{0x3002, 0x1c},
	{0x3006, 0xc3},
	{0x300e, 0x45},
	{0x3017, 0x40},
	{0x3018, 0x00},
	{0x302e, 0x0b},
	{0x4800, 0x24},
	{0x3037, 0x13},
	{0x3108, 0x01},
	{0x3611, 0x06},
	{0x3612, 0xab},
	{0x3614, 0x50},
	{0x3618, 0x04},
	{0x3034, 0x18},
	{0x3035, 0x11}, // 0x11 frame rate 15pfs 0x21: 7.5fps
	{0x3036, 0x54},
	{0x3500, 0x00},
	{0x3501, 0x01},
	{0x3502, 0x00},
	{0x350a, 0x00},
	{0x350b, 0x3f},
	{0x3600, 0x08},
	{0x3601, 0x33},
	{0x3620, 0x33},
	{0x3621, 0xe0},
	{0x3622, 0x01},
	{0x3630, 0x2d},
	{0x3631, 0x00},
	{0x3632, 0x32},
	{0x3633, 0x52},
	{0x3634, 0x70},
	{0x3635, 0x13},
	{0x3636, 0x03},
	{0x3702, 0x6e},
	{0x3703, 0x52},
	{0x3704, 0xa0},
	{0x3705, 0x33},
	{0x3708, 0x63},
	{0x3709, 0x12},
	{0x370b, 0x61},
	{0x370c, 0xc0},
	{0x370f, 0x10},
	{0x3715, 0x08},
	{0x3717, 0x01},
	{0x371b, 0x20},
	{0x3731, 0x22},
	{0x3739, 0x70},
	{0x3901, 0x0a},
	{0x3905, 0x02},
	{0x3906, 0x10},
	{0x3719, 0x86},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0x9f},
	{0x3808, 0x0a},
	{0x3809, 0x20},
	{0x380a, 0x07},
	{0x380b, 0x98},
	{0x380c, 0x0b},
	{0x380d, 0x20}, //0x1c
	{0x380e, 0x07},
	{0x380f, 0xb4}, // 0xb0
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x06},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3820, 0x40},
	{0x3821, 0x06},
	{0x3824, 0x01},
	{0x3826, 0x03},
	{0x3828, 0x08},
	{0x3a02, 0x07},
	{0x3a03, 0xb0},
	{0x3a08, 0x01},
	{0x3a09, 0x27},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0e, 0x06},
	{0x3a0d, 0x08},
	{0x3a14, 0x07},
	{0x3a15, 0xb0},
	{0x3a18, 0x00},
	{0x3a19, 0xf8},
	{0x3c01, 0x34},
	{0x3c04, 0x28},
	{0x3c05, 0x98},
	{0x3c07, 0x07},
	{0x3c09, 0xc2},
	{0x3c0a, 0x9c},
	{0x3c0b, 0x40},
	{0x3c01, 0x34},
	{0x4001, 0x02},
	{0x4004, 0x06},
	{0x4005, 0x18},
	{0x4050, 0x6e},
	{0x4051, 0x8f},
	{0x4300, 0x32},
	{0x4514, 0x00},
	{0x4520, 0xb0},
	{0x460b, 0x37},
	{0x460c, 0x20},
	{0x4818, 0x01},
	{0x481d, 0xf0},
	{0x481f, 0x50},
	{0x4823, 0x70},
	{0x4831, 0x14},
	{0x4837, 0x0b},
	{0x5000, 0xa7},
	{0x5001, 0x83},
	{0x501d, 0x00},
	{0x501f, 0x00},
	{0x503d, 0x00},
	{0x505c, 0x30},
	{0x5181, 0x59},
	{0x5183, 0x00},
	{0x5191, 0xf0},
	{0x5192, 0x03},
	{0x5684, 0x10},
	{0x5685, 0xa0},
	{0x5686, 0x0c},
	{0x5687, 0x78},
	{0x5a00, 0x08},
	{0x5a21, 0x00},
	{0x5a24, 0x00},
	{0x3008, 0x02},
	{0x3503, 0x07}, //0x07 disable auto AE
	{0x5180, 0xff},
	{0x5181, 0xf2},
	{0x5182, 0x00},
	{0x5183, 0x14},
	{0x5184, 0x25},
	{0x5185, 0x24},
	{0x5186, 0x09},
	{0x5187, 0x09},
	{0x5188, 0x0a},
	{0x5189, 0x75},
	{0x518a, 0x52},
	{0x518b, 0xea},
	{0x518c, 0xa8},
	{0x518d, 0x42},
	{0x518e, 0x38},
	{0x518f, 0x56},
	{0x5190, 0x42},
	{0x5191, 0xf8},
	{0x5192, 0x04},
	{0x5193, 0x70},
	{0x5194, 0xf0},
	{0x5195, 0xf0},
	{0x5196, 0x03},
	{0x5197, 0x01},
	{0x5198, 0x04},
	{0x5199, 0x12},
	{0x519a, 0x04},
	{0x519b, 0x00},
	{0x519c, 0x06},
	{0x519d, 0x82},
	{0x519e, 0x38},
	{0x5381, 0x1e},
	{0x5382, 0x5b},
	{0x5383, 0x08},
	{0x5384, 0x0b},
	{0x5385, 0x84},
	{0x5386, 0x8f},
	{0x5387, 0x82},
	{0x5388, 0x71},
	{0x5389, 0x11},
	{0x538a, 0x01},
	{0x538b, 0x98},
	{0x5300, 0x08},
	{0x5301, 0x30},
	{0x5302, 0x10},
	{0x5303, 0x00},
	{0x5304, 0x08},
	{0x5305, 0x30},
	{0x5306, 0x08},
	{0x5307, 0x16},
	{0x5309, 0x08},
	{0x530a, 0x30},
	{0x530b, 0x04},
	{0x530c, 0x06},
	{0x5480, 0x01},
	{0x5481, 0x0e},
	{0x5482, 0x18},
	{0x5483, 0x2b},
	{0x5484, 0x52},
	{0x5485, 0x65},
	{0x5486, 0x71},
	{0x5487, 0x7d},
	{0x5488, 0x87},
	{0x5489, 0x91},
	{0x548a, 0x9a},
	{0x548b, 0xaa},
	{0x548c, 0xb8},
	{0x548d, 0xcd},
	{0x548e, 0xdd},
	{0x548f, 0xea},
	{0x5490, 0x1d},
	{0x5580, 0x02},
	{0x5583, 0x40},
	{0x5584, 0x30},
	{0x5589, 0x10},
	{0x558a, 0x00},
	{0x558b, 0xf8},
	{0x5800, 0x3f},
	{0x5801, 0x16},
	{0x5802, 0x0e},
	{0x5803, 0x0d},
	{0x5804, 0x17},
	{0x5805, 0x3f},
	{0x5806, 0x0b},
	{0x5807, 0x06},
	{0x5808, 0x04},
	{0x5809, 0x04},
	{0x580a, 0x06},
	{0x580b, 0x0b},
	{0x580c, 0x09},
	{0x580d, 0x03},
	{0x580e, 0x00},
	{0x580f, 0x00},
	{0x5810, 0x03},
	{0x5811, 0x08},
	{0x5812, 0x0a},
	{0x5813, 0x03},
	{0x5814, 0x00},
	{0x5815, 0x00},
	{0x5816, 0x04},
	{0x5817, 0x09},
	{0x5818, 0x0f},
	{0x5819, 0x08},
	{0x581a, 0x06},
	{0x581b, 0x06},
	{0x581c, 0x08},
	{0x581d, 0x0c},
	{0x581e, 0x3f},
	{0x581f, 0x1e},
	{0x5820, 0x12},
	{0x5821, 0x13},
	{0x5822, 0x21},
	{0x5823, 0x3f},
	{0x5824, 0x68},
	{0x5825, 0x28},
	{0x5826, 0x2c},
	{0x5827, 0x28},
	{0x5828, 0x08},
	{0x5829, 0x48},
	{0x582a, 0x64},
	{0x582b, 0x62},
	{0x582c, 0x64},
	{0x582d, 0x28},
	{0x582e, 0x46},
	{0x582f, 0x62},
	{0x5830, 0x60},
	{0x5831, 0x62},
	{0x5832, 0x26},
	{0x5833, 0x48},
	{0x5834, 0x66},
	{0x5835, 0x44},
	{0x5836, 0x64},
	{0x5837, 0x28},
	{0x5838, 0x66},
	{0x5839, 0x48},
	{0x583a, 0x2c},
	{0x583b, 0x28},
	{0x583c, 0x26},
	{0x583d, 0xae},
	{0x5025, 0x00},
	{0x3a0f, 0x38},
	{0x3a10, 0x30},
	{0x3a11, 0x70},
	{0x3a1b, 0x38},
	{0x3a1e, 0x30},
	{0x3a1f, 0x18},

	{0x3008, 0x02},

	{0xffff, 0xff},
};
#endif


static resolution_param_t  prev_resolution_array[] = {
	{
		.frmsize			= {1280, 960},
		.active_frmsize		= {2592, 1944},
		.active_fps			= 15,
		.size_type			= SIZE_1280X960,
		.reg_script			= ov5645_capture_5M_script,
	},{
		.frmsize			= {1280, 720},
		.active_frmsize		= {1280, 960},
		.active_fps			= 30,
		.size_type			= SIZE_1280X720,
		.reg_script			= ov5645_preview_960P_30HZ_script,
	},{
		//SL-20160421-Fix1080PFlicker-01*{
		/*Base on 5M setting to replace 1080P's setting, but it could only support to 15 fps,
		    if we use truly 1080P's setting , it would serious FOV problem*/
		.frmsize			= {1920, 1080},
		.active_frmsize		= {2592, 1464}, //{2048, 1536},
		.active_fps			= 30,
		.size_type			= SIZE_1920X1080,
		.reg_script			= ov5645_preview_1080P_15HZ_script,
		//SL-20160421-Fix1080PFlicker-01*}
	},
	/*
	{
		.frmsize			= {1024, 768},
		.active_frmsize		= {1280, 960},
		.active_fps			= 15,
		.size_type			= SIZE_1024X768,
		.reg_script			= ov5645_preview_960P_30HZ_script,
	},
	{
		.frmsize			= {640, 480},
		.active_frmsize		= {640, 480},
		.active_fps			= 30,
		.size_type			= SIZE_640X480,
		.reg_script			= ov5645_preview_VGA_script,
	},{
		.frmsize			= {320, 240},
		.active_frmsize		= {640, 480},
		.active_fps			= 30,
		.size_type			= SIZE_320X240,
		.reg_script			= ov5645_preview_VGA_script,
	},{
		.frmsize			= {352, 288},
		.active_frmsize		= {640, 480},
		.active_fps			= 30,
		.size_type			= SIZE_352X288,
		.reg_script			= ov5645_preview_VGA_script,
	},{
		.frmsize			= {176, 144},
		.active_frmsize		= {640, 480},
		.active_fps			= 30,
		.size_type			= SIZE_176X144,
		.reg_script			= ov5645_preview_VGA_script,
	}, */ 
};

static resolution_param_t  capture_resolution_array[] = {
	{
		.frmsize			= {1280, 960},
		.active_frmsize		= {2592, 1944},
		.active_fps			= 15,
		.size_type			= SIZE_1280X960,
		.reg_script			= ov5645_capture_5M_script,
	},
	{
		.frmsize			= {2592, 1944},
		.active_frmsize		= {2592, 1944},
		.active_fps			= 15,
		.size_type			= SIZE_2592X1944,
		.reg_script			= ov5645_capture_5M_script,
	},
/*	
	{
		.frmsize			= {1600, 1200},
		.active_frmsize		= {2592, 1942},
		.active_fps			= 15,
		.size_type			= SIZE_1600X1200,
		.reg_script			= ov5645_capture_5M_script,
	},{
		.frmsize			= {2048, 1536},
		.active_frmsize		= {2592, 1942},
		.active_fps			= 15,
		.size_type			= SIZE_2048X1536,
		.reg_script			= ov5645_capture_5M_script,
	},
	*/
};

static void ov5645_open_mipi_stream(void)
{
	pr_info("%s \n", __func__);
	i2c_put_byte(g_client, 0x4202, 0x00); //ov5645 open mipi stream
}

// download firmware by single i2c write
static int ov5645_download_firmware(struct ov5645_device *dev)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	int i=0, d=0;

	pr_info("%s start ....\n", __func__);

	i2c_put_byte(client, 0x3000,0x20);
	if (ov5645_have_opened) {
		//OV5645_FW_data_pair_size
		for(d=0; d<sizeof(OV5645_AF_firmware)/sizeof(OV5645_AF_firmware[0]); d++) {
			u8 *pdata = OV5645_AF_firmware[d];
			for(i=0; i<OV5645_FW_data_pair_size[d]; i++) {
				if((i2c_put_byte(client, OV5645_FW_ADDR, pdata[i])) < 0) {
					printk("fail in download firmware ov5645. i=%d\n",i);
					break;
				}
			}
			i++;
		}
	}
	OV5645MIPI_write_cmos_sensor(0x3022,0x00);
	OV5645MIPI_write_cmos_sensor(0x3023,0x00);
	OV5645MIPI_write_cmos_sensor(0x3024,0x00);
	OV5645MIPI_write_cmos_sensor(0x3025,0x00);
	OV5645MIPI_write_cmos_sensor(0x3026,0x00);
	OV5645MIPI_write_cmos_sensor(0x3027,0x00);
	OV5645MIPI_write_cmos_sensor(0x3028,0x00);
	OV5645MIPI_write_cmos_sensor(0x3029,0x7F);
	OV5645MIPI_write_cmos_sensor(0x3000, 0x00);        // Enable MCU
	/*
	if(false == AF_Power) {
		OV5645MIPI_write_cmos_sensor(0x3602,0x00);
		OV5645MIPI_write_cmos_sensor(0x3603,0x00);
	}*/

	return 0;
}

static camera_focus_mode_t start_focus_mode = CAM_FOCUS_MODE_RELEASE;
static int ov5645_AutoFocus(struct ov5645_device *dev, int focus_mode);

static void do_download(struct work_struct *work)
{
	struct ov5645_device *dev = container_of(work, struct ov5645_device, dl_work);
	//struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	mutex_lock(&firmware_mutex);
	if (ov5645_have_opened) {
		if(ov5645_download_firmware(dev) >= 0) {
			msleep(10);
		}
	}
	dev->firmware_ready = 1;
	mutex_unlock(&firmware_mutex);
	if (start_focus_mode) {
		ov5645_AutoFocus(dev,(int)start_focus_mode);
		start_focus_mode = CAM_FOCUS_MODE_RELEASE;
	}
}

void ov5645_reset(struct i2c_client *client)
{
	pr_info("%s \n", __func__);
	i2c_put_byte(client, 0x3103, 0x11);//	; PLL clock selection
	msleep(5);
	i2c_put_byte(client,0x3008, 0x82);//	; software reset
	//msleep(10); //delay(2);		//; delay 2ms
}

void ov5645_init_regs(struct ov5645_device *dev)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	int i=0;
	ov5645_reset(client);
	while (1) {
		if (ov5645_script[i].val==0xff&&ov5645_script[i].addr==0xffff) {
			printk("success in initial ov5645.\n");
			break;
		}
		if ((i2c_put_byte(client,ov5645_script[i].addr, ov5645_script[i].val)) < 0) {
			printk("fail in initial ov5645. \n");
			return;
		}
		i++;
	}
	return;
}

#if 0
static int get_exposure_param(struct ov5645_device *dev,
	unsigned char *gain,unsigned char *exposurelow,unsigned char *exposuremid,unsigned char *exposurehigh)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	int ret;
	unsigned char ogain,oexposurelow,oexposuremid,oexposurehigh;
	int capture_maxLines;
	int preview_maxlines;
	int preview_maxLine_H;
	int preview_maxLine_L;
	long capture_Exposure;
	long capture_exposure_gain;
	long previewExposure;
	long capture_gain;
	int lines_10ms;
	#define CAPTURE_FRAMERATE 375
	#define PREVIEW_FRAMERATE 1500

	//i2c_put_byte(client, 0x3503, 0x07);
	
	ogain = i2c_get_byte(client, 0x350b);
	printk("0x350b = 0x%x\n",ogain);

	oexposurelow = i2c_get_byte(client, 0x3502);
	printk("0x3502 = 0x%x\n",oexposurelow);

	oexposuremid = i2c_get_byte(client, 0x3501);
	printk("0x3501 = 0x%x\n",oexposuremid);

	oexposurehigh = i2c_get_byte(client, 0x3500) & 0xf;
	printk("0x3500 = 0x%x\n",oexposurehigh);
	
	preview_maxLine_H = i2c_get_byte(client, 0x380e);
	printk("preview_maxLine_H = %d\n", preview_maxLine_H);
	preview_maxLine_L = i2c_get_byte(client, 0x380f);
	printk("preview_maxLine_L = %d\n", preview_maxLine_L);
	preview_maxlines = preview_maxLine_H << 8 + preview_maxLine_L;
	printk("preview_maxlines = %d\n", preview_maxlines);
	preview_maxlines = 984;
	capture_maxLines = 1968;
	if(ov5645_qctrl[7].default_value == CAM_BANDING_60HZ) { //60Hz
		lines_10ms = CAPTURE_FRAMERATE * capture_maxLines/12000;
	} else {
		lines_10ms = CAPTURE_FRAMERATE * capture_maxLines/10000;
	}
	
	printk("lines_10ms is %d\n", lines_10ms);

	previewExposure = ((unsigned int)(oexposurehigh))<<12 ;
	previewExposure += ((unsigned int)oexposuremid)<<4 ;
	previewExposure += ((unsigned int)oexposurelow)>>4;

	capture_Exposure = (previewExposure*CAPTURE_FRAMERATE*capture_maxLines)/
							(preview_maxlines*PREVIEW_FRAMERATE); 
	printk("capture_Exposure = %d\n", capture_Exposure);

	capture_gain = (ogain&0xf) + 16;
	
	if (ogain & 0x10) {
		capture_gain = capture_gain << 1;
	}
	if (ogain & 0x20) {
		capture_gain = capture_gain << 1;
	}
	if (ogain & 0x40) {
		capture_gain = capture_gain << 1;
	}
	if (ogain & 0x80) {
		capture_gain = capture_gain << 1;
	}
	printk("capture_gain = %d\n", capture_gain);
	
	capture_exposure_gain = capture_Exposure * capture_gain;
	
	printk("capture_exposure_gain = %d\n", capture_exposure_gain);

	if(capture_exposure_gain <
			((signed int)(capture_maxLines)*16)) {
		capture_Exposure = capture_exposure_gain/16;
		if (capture_Exposure > lines_10ms) {
			capture_Exposure /= lines_10ms;
			capture_Exposure *= lines_10ms;
			printk("capture_Exposure is %d\n", capture_Exposure);
		}
	} else {
		capture_Exposure = capture_maxLines;
	} 
	printk("capture_Exposure is %d\n", capture_Exposure);
	if(capture_Exposure == 0) {
		capture_Exposure = 1;
	} 
	capture_gain =(capture_exposure_gain*2/capture_Exposure + 1)/2;
	*exposurelow = (unsigned char)(capture_Exposure<<4) & 0xff;
	*exposuremid = (unsigned char)(capture_Exposure >> 4) & 0xff;
	*exposurehigh = (unsigned char)(capture_Exposure >> 12) & 0xf;
	*gain =(unsigned char) capture_gain;

	return 0;
}
static int set_exposure_param_500m(struct ov5645_device *dev,
	unsigned char gain ,unsigned char exposurelow, unsigned char exposuremid, unsigned char exposurehigh)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	int ret;
	ret = i2c_put_byte(client, 0x350b, gain);
	if (ret < 0) {
		printk("sensor_write err !\n");
		return ret;
	}
	
	ret = i2c_put_byte(client, 0x3502, exposurelow);
	if (ret < 0) {
		printk("sensor_write err !\n");
		return ret;
	}

	ret = i2c_put_byte(client, 0x3501, exposuremid);
	if (ret < 0) {
		printk("sensor_write err !\n");
		return ret;
	}

	ret = i2c_put_byte(client, 0x3500, exposurehigh);
	if (ret < 0) {
		printk("sensor_write err !\n");
		return ret;
	}
	
	return 0;
}
#else
#define PRE_EXP (0x5c4)
#define PRE_EXTRA_LINES (0)
#define PRE_GAIN (0x38)

/* Sensor Exposure Line Limitation */
#define OV5645MIPI_PV_EXPOSURE_LIMITATION      	(984-4)
#define OV5645MIPI_FULL_EXPOSURE_LIMITATION    	(1968-4)

#if 0
static unsigned long ov5645_preview_maxlines;
static int Get_preview_exposure_gain(struct ov5645_device *dev)
{
	int rc = 0;
	unsigned int ret_l,ret_m,ret_h;
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	ret_h = ret_m = ret_l = 0;
	ret_h = i2c_get_byte(client, 0x350c);
	ret_l = i2c_get_byte(client,0x350d);
	ov5645_preview_extra_lines = ((ret_h << 8) + ret_l);
	i2c_put_byte(client,0x3503, 0x07);//stop aec/agc
	//get preview exp & gain
	ret_h = ret_m = ret_l = 0;
	ov5645_preview_exposure = 0;
	ret_h = i2c_get_byte(client, 0x3500);
	ret_m = i2c_get_byte(client,0x3501);
	ret_l = i2c_get_byte(client,0x3502);
	pr_info("%s, exp h:0x%x04x m:0x%04x l:0x%04x \n", __func__,ret_h, ret_m, ret_l);
	ov5645_preview_exposure = (ret_h << 12) + (ret_m << 4) + (ret_l >> 4);
	ret_h = ret_m = ret_l = 0;
	ov5645_preview_exposure = ov5645_preview_exposure + (ov5645_preview_extra_lines)/16;
	printk("preview_exposure=%lx \n", ov5645_preview_exposure);
	ret_h = ret_m = ret_l = 0;
	ov5645_preview_maxlines = 0;
	ret_h = i2c_get_byte(client, 0x380e);
	ret_l = i2c_get_byte(client, 0x380f);
	ov5645_preview_maxlines = (ret_h << 8) + ret_l;
	printk("%s, Preview_Maxlines=%lx \n", __func__, ov5645_preview_maxlines);
	//Read back AGC Gain for preview
	ov5645_gain = 0;
	ov5645_gain = i2c_get_byte(client, 0x350b);
	printk("Gain,0x350b=0x%lx\n", ov5645_gain);

	return rc;
}

static int cal_exposure(struct ov5645_device *dev, int pre_fps, int cap_fps)
{
	int rc = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	//calculate capture exp & gain
	unsigned char ExposureLow,ExposureMid,ExposureHigh,Capture_MaxLines_High,Capture_MaxLines_Low;
	unsigned int ret_l,ret_m,ret_h,Lines_10ms;
	unsigned short ulCapture_Exposure,iCapture_Gain;
	unsigned int ulCapture_Exposure_Gain,Capture_MaxLines;
	ret_h = ret_m = ret_l = 0;
	ret_h = i2c_get_byte(client, 0x380e);
	ret_l = i2c_get_byte(client, 0x380f);
	Capture_MaxLines = (ret_h << 8) + ret_l;
	Capture_MaxLines = Capture_MaxLines + (ov5645_preview_extra_lines)/16;
	printk("Capture_MaxLines=%d\n", Capture_MaxLines);
	if(ov5645_qctrl[7].default_value == CAM_BANDING_60HZ) { //60Hz
		Lines_10ms = cap_fps * Capture_MaxLines/12000;
	} else {
		Lines_10ms = cap_fps * Capture_MaxLines/10000;
	}
	if(ov5645_preview_maxlines == 0) {
		ov5645_preview_maxlines = 1;
	}
	ulCapture_Exposure = (ov5645_preview_exposure*(cap_fps)*(Capture_MaxLines))/
	(((ov5645_preview_maxlines)*(pre_fps)));
	iCapture_Gain = ov5645_gain;
	ulCapture_Exposure_Gain = ulCapture_Exposure * iCapture_Gain;
	if(ulCapture_Exposure_Gain < Capture_MaxLines*16) {
		ulCapture_Exposure = ulCapture_Exposure_Gain/16;
		if (ulCapture_Exposure > Lines_10ms) {
			ulCapture_Exposure /= Lines_10ms;
			ulCapture_Exposure *= Lines_10ms;
		}
	} else {
		ulCapture_Exposure = Capture_MaxLines;
	}
	if(ulCapture_Exposure == 0) {
		ulCapture_Exposure = 1;
	}
	iCapture_Gain = (ulCapture_Exposure_Gain*2/ulCapture_Exposure + 1)/2;
	ExposureLow = ((unsigned char)ulCapture_Exposure)<<4;
	ExposureMid = (unsigned char)(ulCapture_Exposure >> 4) & 0xff;
	ExposureHigh = (unsigned char)(ulCapture_Exposure >> 12);
	Capture_MaxLines_Low = (unsigned char)(Capture_MaxLines & 0xff);
	Capture_MaxLines_High = (unsigned char)(Capture_MaxLines >> 8);
	i2c_put_byte(client, 0x380e, Capture_MaxLines_High);
	i2c_put_byte(client, 0x380f, Capture_MaxLines_Low);
	i2c_put_byte(client, 0x350b, iCapture_Gain);
	i2c_put_byte(client, 0x3502, ExposureLow);
	i2c_put_byte(client, 0x3501, ExposureMid);
	i2c_put_byte(client, 0x3500, ExposureHigh);
	printk("iCapture_Gain=%d\n", iCapture_Gain);
	printk("ExposureLow=%d\n", ExposureLow);
	printk("ExposureMid=%d\n", ExposureMid);
	printk("ExposureHigh=%d\n", ExposureHigh);
	//msleep(250);
	return rc;
}
#else

static void OV5645_WB_calibattion(u32 color_r_gain,u32 color_g_gain,u32 color_b_gain, int ignore)
{
	u32 color_r_gain_w = 0x400;
	u32 color_g_gain_w = 0x400;
	u32 color_b_gain_w = 0x400;
	u8 temp = OV5645MIPIYUV_read_cmos_sensor(0x350b); 
	pr_info("[OV5645MIPIOFILM]enter OV5645WBcalibattion function:\n ");

	if(ignore){
		return;
	}
		if(color_r_gain>0x600) {
			if (temp>=0xb0) {
				color_r_gain_w=color_r_gain *94/100;	
				color_g_gain_w=color_g_gain; 																																													
				color_b_gain_w=color_b_gain*96/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x600 temp>=0x70:\n ");
			} else if (temp>=0x70) {
				color_r_gain_w=color_r_gain *95/100;	
				color_g_gain_w=color_g_gain; 																																													
				color_b_gain_w=color_b_gain*97/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x600 temp>=0x70:\n ");
			} else if (temp>=0x58) {
				color_r_gain_w=color_r_gain*97/100;	
				color_g_gain_w=color_g_gain;   																																														
				color_b_gain_w=color_b_gain*98/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x600 temp>=0x58:\n ");
			}
			else if (temp>=0x48)
			{
				color_r_gain_w=color_r_gain*97/100;	
				color_g_gain_w=color_g_gain;

				color_b_gain_w=color_b_gain*98/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x600 temp>=0x48:\n ");
			}
			else if (temp>=0x30)
			{
				color_r_gain_w=color_r_gain*97/100;	
				color_g_gain_w=color_g_gain;

				color_b_gain_w=color_b_gain*98/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x600 temp>=0x30:\n ");
			}
			else
			{
				color_r_gain_w=color_r_gain*97/100;	
				color_g_gain_w=color_g_gain;

				color_b_gain_w=color_b_gain*98/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x600 temp else:\n ");
			}	
		}else if(color_r_gain>0x540)//cwf  //?¡ã?¡ì¨ªa
		{
			if (temp>=0xb0)
			{
				color_r_gain_w=color_r_gain *94/100;	
				color_g_gain_w=color_g_gain; 																																													
				color_b_gain_w=color_b_gain*96/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x600 temp>=0x70:\n ");
			}
			else if (temp>=0x70)

			{
				color_r_gain_w=color_r_gain *97/100;
				color_g_gain_w=color_g_gain;

				color_b_gain_w=color_b_gain*98/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x540 temp>=0x70:\n ");
			}
			else if (temp>=0x58)
			{
				color_r_gain_w=color_r_gain*98/100;	
				color_g_gain_w=color_g_gain;	

				color_b_gain_w=color_b_gain*99/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x540 temp>=0x58:\n ");
			}
			else if (temp>=0x48)
			{
				color_r_gain_w=color_r_gain*98/100;
				color_g_gain_w=color_g_gain;

				color_b_gain_w=color_b_gain*99/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x540 temp>=0x48:\n ");
			}
			else if (temp>=0x30)
			{
				color_r_gain_w=color_r_gain*99/100;	
				color_g_gain_w=color_g_gain;

				color_b_gain_w=color_b_gain*99/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x540 temp>=0x30:\n ");
			}
			else
			{
				color_r_gain_w=color_r_gain*99/100;	
				color_g_gain_w=color_g_gain;

				color_b_gain_w=color_b_gain*99/100; 
				pr_info("OV5645WBcalibattion function color_r_gain>0x540 temp else:\n ");
			}				
		}else if(color_r_gain>0x480)//tl84
		{
			if (temp>=0xb0)
			{
				color_r_gain_w=color_r_gain *94/100;	
				color_g_gain_w=color_g_gain; 																																													
				color_b_gain_w=color_b_gain*96/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x600 temp>=0x70:\n ");
			}

			else if (temp>=0x70)
			{
				color_r_gain_w=color_r_gain *97/100;	
				color_g_gain_w=color_g_gain;	

				color_b_gain_w=color_b_gain*97/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x480 temp>=0x70:\n ");
			}
			else if (temp>=0x58)
			{
				color_r_gain_w=color_r_gain*97/100;
				color_g_gain_w=color_g_gain;

				color_b_gain_w=color_b_gain*97/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x480 temp>=0x58:\n ");
			}
			else if (temp>=0x48)
			{
				color_r_gain_w=color_r_gain*97/100;	
				color_g_gain_w=color_g_gain;	

				color_b_gain_w=color_b_gain*97/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x480 temp>=0x48:\n ");
			}
			else if (temp>=0x30)
			{
				color_r_gain_w=color_r_gain*97/100;
				color_g_gain_w=color_g_gain;	

				color_b_gain_w=color_b_gain*97/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x480 temp>=0x30:\n ");
			}
			else
			{
				color_r_gain_w=color_r_gain*97/100;	
				color_g_gain_w=color_g_gain;

				color_b_gain_w=color_b_gain*97/100; 
				pr_info("OV5645WBcalibattion function color_r_gain>0x480 temp>=else:\n ");
			}	
		}else//h/a
		{
			if (temp>=0xb0)
			{
				color_r_gain_w=color_r_gain *94/100;	
				color_g_gain_w=color_g_gain; 																																													
				color_b_gain_w=color_b_gain*96/100;
				pr_info("OV5645WBcalibattion function color_r_gain>0x600 temp>=0x70:\n ");
			}
			else if (temp>=0x70)
			{
				color_r_gain_w=color_r_gain *98/100;	
				color_g_gain_w=color_g_gain;

				color_b_gain_w=color_b_gain*97/100;
				pr_info("OV5645WBcalibattion function color_r_gain>else temp>=0x70:\n ");
			}
			else if (temp>=0x58)
			{
				color_r_gain_w=color_r_gain*98/100;	
				color_g_gain_w=color_g_gain;	

				color_b_gain_w=color_b_gain*97/100;
				pr_info("OV5645WBcalibattion function color_r_gain>else temp>=0x58:\n ");
			}
			else if (temp>=0x48)
			{
				color_r_gain_w=color_r_gain*98/100;	
				color_g_gain_w=color_g_gain;	

				color_b_gain_w=color_b_gain*97/100;
				pr_info("OV5645WBcalibattion function color_r_gain>else temp>=0x48:\n ");
			}
			else if (temp>=0x30)
			{
				color_r_gain_w=color_r_gain*98/100;	
				color_g_gain_w=color_g_gain;	

				color_b_gain_w=color_b_gain*97/100;
				pr_info("OV5645WBcalibattion function color_r_gain>else temp>=0x30:\n ");
			}
			else
			{
				color_r_gain_w=color_r_gain*98/100;	
				color_g_gain_w=color_g_gain;

				color_b_gain_w=color_b_gain*97/100;
				pr_info("OV5645WBcalibattion function color_r_gain>else temp>=else:\n ");
			}	
		}		
	//OV5645MIPI_write_cmos_sensor(0x3212,0x02);		  																																																																							
	OV5645MIPI_write_cmos_sensor(0x3400,(color_r_gain_w & 0xff00)>>8);																																														
	OV5645MIPI_write_cmos_sensor(0x3401,color_r_gain_w & 0xff); 


	OV5645MIPI_write_cmos_sensor(0x3402,(color_g_gain_w & 0xff00)>>8);	

	OV5645MIPI_write_cmos_sensor(0x3403,color_g_gain_w & 0xff); 	
	OV5645MIPI_write_cmos_sensor(0x3404,(color_b_gain_w & 0xff00)>>8);	

	OV5645MIPI_write_cmos_sensor(0x3405,color_b_gain_w & 0xff); 
	//OV5645MIPI_write_cmos_sensor(0x3212,0x12);
	//OV5645MIPI_write_cmos_sensor(0x3212,0xa2);
	pr_info("OV5645WBcalibattion function color_r_gain_w=%x,color_g_gain_w=%x,color_b_gain_w=%x \n",color_r_gain_w,color_g_gain_w,color_b_gain_w);
}	
static void OV5645MIPI_Write_Shutter(u32 shutter)
{
	u32 extra_exposure_vts = 0;
	pr_info("[OV5645MIPI]enter OV5645MIPIWriteShutter function:shutter=0x%04x\n ",shutter);
	if (shutter < 1) {
		shutter = 1;
	}
	if (shutter > OV5645MIPI_FULL_EXPOSURE_LIMITATION) {
		extra_exposure_vts =shutter+4;
		OV5645MIPI_write_cmos_sensor(0x380f, extra_exposure_vts & 0xFF);          // EXVTS[b7~b0]
		OV5645MIPI_write_cmos_sensor(0x380e, (extra_exposure_vts & 0xFF00) >> 8); // EXVTS[b15~b8]
		OV5645MIPI_write_cmos_sensor(0x350D,0x00); 
		OV5645MIPI_write_cmos_sensor(0x350C,0x00);
	}
	shutter *= 16;
	OV5645MIPI_write_cmos_sensor(0x3502, (shutter & 0x00FF));           //AEC[7:0]
	OV5645MIPI_write_cmos_sensor(0x3501, ((shutter & 0x0FF00) >>8));  //AEC[15:8]
	OV5645MIPI_write_cmos_sensor(0x3500, ((shutter & 0xFF0000) >> 16));	
	pr_info("[OV5645MIPI]exit OV5645MIPIWriteShutter function: shutter:0x%04x \n", shutter);
}  //OV5645MIPI_write_shutter

static u32 OV5645MIPI_Read_Shutter(void)
{
	u16 temp_reg1, temp_reg2 ,temp_reg3;
	u32 PreviewShutter = 0;
	pr_info("[OV5645MIPI]enter OV5645MIPIReadShutter function:\n ");
	temp_reg1 = OV5645MIPIYUV_read_cmos_sensor(0x3500);    // AEC[b19~b16]
	temp_reg2 = OV5645MIPIYUV_read_cmos_sensor(0x3501);    // AEC[b15~b8]
	temp_reg3 = OV5645MIPIYUV_read_cmos_sensor(0x3502);    // AEC[b7~b0]

	PreviewShutter = (temp_reg1 <<12)| (temp_reg2<<4)|(temp_reg3>>4);
	pr_info("[OV5645MIPI]exit OV5645MIPIReadShutter function:\n ");	
	return PreviewShutter;
}
#endif
#if 0
static unsigned long ov5645_preview_exposure = 0x5c4;
static unsigned long ov5645_preview_extra_lines = 0;
static unsigned long ov5645_gain = 0x38;
static void OV5645MIPIWriteExpShutter(u32 shutter)
{
	shutter*=16;
	pr_info("[OV5645MIPI]enter OV5645MIPIWriteExpShutter function:shutter=%d\n ",shutter);
	OV5645MIPI_write_cmos_sensor(0x3502, (shutter & 0x00FF));           //AEC[7:0]
	OV5645MIPI_write_cmos_sensor(0x3501, ((shutter & 0x0FF00) >>8));  //AEC[15:8]
	OV5645MIPI_write_cmos_sensor(0x3500, ((shutter & 0xFF0000) >> 16));	
	pr_info("[OV5645MIPI]exit OV5645MIPIWriteExpShutter function:\n ");
}
static void OV5645MIPIWriteExtraShutter(u32 shutter)
{
	pr_info("[OV5645MIPI]enter OV5645MIPIWriteExtraShutter function:shutter=%d\n ",shutter);
	OV5645MIPI_write_cmos_sensor(0x350D, shutter & 0xFF);          // EXVTS[b7~b0]
	OV5645MIPI_write_cmos_sensor(0x350C, (shutter & 0xFF00) >> 8); // EXVTS[b15~b8]
	pr_info("[OV5645MIPI]exit OV5645MIPIWriteExtraShutter function:\n ");
}
static void OV5645MIPIWriteSensorGain(u32 gain)
{
	u16 temp_reg ;
	pr_info("[OV5645MIPI]enter OV5645MIPIWriteSensorGain function:gain=%d\n",gain);
	if(gain > 1024) {
		pr_err("gain is bigger then 1024\n");
		return;
	}
	temp_reg = 0;
	temp_reg=gain&0x0FF;	
	OV5645MIPI_write_cmos_sensor(0x350B,temp_reg);
	pr_info("[OV5645MIPI]exit OV5645MIPIWriteSensorGain function:\n ");
}

static void pre_default_exposure(void)
{
	pr_info("%s ...\n", __func__);
	ov5645_preview_extra_lines = PRE_EXTRA_LINES;
	ov5645_preview_exposure = PRE_EXP;
	ov5645_gain = PRE_GAIN;
	pr_info("preview set extra_lines:0x%04x, exposure:0x%04x, gain:0x%04x\n", 
			(int)ov5645_preview_extra_lines, (int)ov5645_preview_exposure, (int)ov5645_gain);
	OV5645MIPIWriteExtraShutter(ov5645_preview_extra_lines);
	OV5645MIPIWriteExpShutter(ov5645_preview_exposure);   
	OV5645MIPIWriteSensorGain( ov5645_gain );
} 
#endif

#endif
void OV5645MIPI_set_contrast(u16 para)
{   
    pr_info("[OV5645MIPI]enter OV5645MIPI_set_contrast function:\n ");
    switch (para) {
		case 1://ISP_CONTRAST_LOW:
             OV5645MIPI_write_cmos_sensor(0x5586,0x14);
             OV5645MIPI_write_cmos_sensor(0x5585,0x14);
             break;
		case 9:// ISP_CONTRAST_HIGH:
             OV5645MIPI_write_cmos_sensor(0x5586,0x2c);
             OV5645MIPI_write_cmos_sensor(0x5585,0x1c);
             break;
		case 5:// ISP_CONTRAST_MIDDLE:
        default:
             OV5645MIPI_write_cmos_sensor(0x5586,0x20);
             OV5645MIPI_write_cmos_sensor(0x5585,0x00);
             break;
    }
    pr_info("[OV5645MIPI]exit OV5645MIPI_set_contrast function:\n ");
    return;
}

void OV5645MIPI_set_brightness(u16 para)
{
    pr_info("[OV5645MIPI]enter OV5645MIPI_set_brightness function:\n ");
    switch (para) {
		case 1://ISP_BRIGHT_LOW:
                   OV5645MIPI_write_cmos_sensor(0x5587,0x40);
                   OV5645MIPI_write_cmos_sensor(0x5588,0x09);
                   break;
		case 9: //ISP_BRIGHT_HIGH:
                   OV5645MIPI_write_cmos_sensor(0x5587,0x40);
                   OV5645MIPI_write_cmos_sensor(0x5588,0x01);
                   break;
		case 5:// ISP_BRIGHT_MIDDLE:
        default:
                   OV5645MIPI_write_cmos_sensor(0x5587,0x00);
                   OV5645MIPI_write_cmos_sensor(0x5588,0x01);
                   break;
    }
    pr_info("[OV5645MIPI]exit OV5645MIPI_set_brightness function:\n ");
    return;
}

void OV5645MIPI_set_saturation(u16 para)
{
	pr_info("[OV5645MIPI]enter OV5645MIPI_set_saturation function:\n ");
    switch (para) {
		case 9://ISP_SAT_HIGH:
		OV5645MIPI_write_cmos_sensor(0x3212,0x03);
		OV5645MIPI_write_cmos_sensor(0x5381,0x22);//ccm
		OV5645MIPI_write_cmos_sensor(0x5382,0x55);//
		OV5645MIPI_write_cmos_sensor(0x5383,0x12);//
		OV5645MIPI_write_cmos_sensor(0x5384,0x04 );//
		OV5645MIPI_write_cmos_sensor(0x5385,0x94);//
		OV5645MIPI_write_cmos_sensor(0x5386,0x98);//
		OV5645MIPI_write_cmos_sensor(0x5387,0xa9);//
		OV5645MIPI_write_cmos_sensor(0x5388,0x9a);//
		OV5645MIPI_write_cmos_sensor(0x5389,0x0e);//    
		OV5645MIPI_write_cmos_sensor(0x538a,0x01);//      
		OV5645MIPI_write_cmos_sensor(0x538b,0x98);//      			                                           
		OV5645MIPI_write_cmos_sensor(0x3212,0x13);
		OV5645MIPI_write_cmos_sensor(0x3212,0xa3);
             break;
		case 1://ISP_SAT_LOW:
		OV5645MIPI_write_cmos_sensor(0x3212,0x03);
		OV5645MIPI_write_cmos_sensor(0x5381,0x22);//ccm
		OV5645MIPI_write_cmos_sensor(0x5382,0x55);//
		OV5645MIPI_write_cmos_sensor(0x5383,0x12);//
		OV5645MIPI_write_cmos_sensor(0x5384,0x02);//
		OV5645MIPI_write_cmos_sensor(0x5385,0x62);//
		OV5645MIPI_write_cmos_sensor(0x5386,0x66);//
		OV5645MIPI_write_cmos_sensor(0x5387,0x71);//
		OV5645MIPI_write_cmos_sensor(0x5388,0x66);//
		OV5645MIPI_write_cmos_sensor(0x5389,0x0a);//    
		OV5645MIPI_write_cmos_sensor(0x538a,0x01);//      
		OV5645MIPI_write_cmos_sensor(0x538b,0x98);//      			                                           
		OV5645MIPI_write_cmos_sensor(0x3212,0x13);
		OV5645MIPI_write_cmos_sensor(0x3212,0xa3);
             break;
		case 5://ISP_SAT_MIDDLE:
        default:
		OV5645MIPI_write_cmos_sensor(0x3212,0x03);
		OV5645MIPI_write_cmos_sensor(0x5381,0x24);//ccm //0x22 28
		OV5645MIPI_write_cmos_sensor(0x5382,0x5b);// //0x55 66
		OV5645MIPI_write_cmos_sensor(0x5383,0x13);// //0x12 15
		OV5645MIPI_write_cmos_sensor(0x5384,0x03);//
		OV5645MIPI_write_cmos_sensor(0x5385,0x7f);//
		OV5645MIPI_write_cmos_sensor(0x5386,0x83);//
		OV5645MIPI_write_cmos_sensor(0x5387,0x91);//
		OV5645MIPI_write_cmos_sensor(0x5388,0x84);//
		OV5645MIPI_write_cmos_sensor(0x5389,0x0d);//
		OV5645MIPI_write_cmos_sensor(0x538a,0x01);//      
		OV5645MIPI_write_cmos_sensor(0x538b,0x98);//      			                                           
		OV5645MIPI_write_cmos_sensor(0x3212,0x13);
		OV5645MIPI_write_cmos_sensor(0x3212,0xa3);
		break;
    }
    msleep(50);
	pr_info("[OV5645MIPI]exit OV5645MIPI_set_saturation function:\n ");
     return;
}

#if 0
void OV5645MIPI_scene_mode_PORTRAIT()
{
	pr_info("[OV5645MIPI]enter OV5645MIPI_scene_mode_PORTRAIT function:\n ");
	OV5645MIPI_write_cmos_sensor(0x3212,0x03);
		/*FRAME rate*/
	OV5645MIPI_write_cmos_sensor(0x3A00,0x3c); //10-30
	OV5645MIPI_write_cmos_sensor(0x3a02,0x0b); 
	OV5645MIPI_write_cmos_sensor(0x3a03,0x88);						   
	OV5645MIPI_write_cmos_sensor(0x3a14,0x0b); 
	OV5645MIPI_write_cmos_sensor(0x3a15,0x88);
	OV5645MIPI_write_cmos_sensor(0x350c,0x00);
	OV5645MIPI_write_cmos_sensor(0x350d,0x00);
	/*AE Weight - CenterAverage*/ 
	OV5645MIPI_write_cmos_sensor(0x501d,0x00);
	OV5645MIPI_write_cmos_sensor(0x5688,0x11);
	OV5645MIPI_write_cmos_sensor(0x5689,0x11);
	OV5645MIPI_write_cmos_sensor(0x568a,0x21);
	OV5645MIPI_write_cmos_sensor(0x568b,0x12);
	OV5645MIPI_write_cmos_sensor(0x568c,0x12);
	OV5645MIPI_write_cmos_sensor(0x568d,0x21);
	OV5645MIPI_write_cmos_sensor(0x568e,0x11);
	OV5645MIPI_write_cmos_sensor(0x568f,0x11);
	OV5645MIPI_write_cmos_sensor(0x3212,0x13); 
	OV5645MIPI_write_cmos_sensor(0x3212,0xa3);
	pr_info("[OV5645MIPI]exit OV5645MIPI_scene_mode_PORTRAIT function:\n ");
}

void OV5645MIPI_scene_mode_LANDSCAPE()
{
	pr_info("[OV5645MIPI]enter OV5645MIPI_scene_mode_LANDSCAPE function:\n ");
	OV5645MIPI_write_cmos_sensor(0x3212,0x03);
	/*FRAME rate*/
	OV5645MIPI_write_cmos_sensor(0x3A00,0x3c); //10-30
	OV5645MIPI_write_cmos_sensor(0x3a02,0x0b); 
	OV5645MIPI_write_cmos_sensor(0x3a03,0x88);                         
	OV5645MIPI_write_cmos_sensor(0x3a14,0x0b); 
	OV5645MIPI_write_cmos_sensor(0x3a15,0x88);
	OV5645MIPI_write_cmos_sensor(0x350c,0x00);
	OV5645MIPI_write_cmos_sensor(0x350d,0x00);
	/*AE Weight - CenterAverage*/ 
	OV5645MIPI_write_cmos_sensor(0x501d,0x00);
	OV5645MIPI_write_cmos_sensor(0x5688,0x11); 
	OV5645MIPI_write_cmos_sensor(0x5689,0x11); 
	OV5645MIPI_write_cmos_sensor(0x568a,0x11); 
	OV5645MIPI_write_cmos_sensor(0x568b,0x11); 
	OV5645MIPI_write_cmos_sensor(0x568c,0x11); 
	OV5645MIPI_write_cmos_sensor(0x568d,0x11); 
	OV5645MIPI_write_cmos_sensor(0x568e,0x11); 
	OV5645MIPI_write_cmos_sensor(0x568f,0x11);
    OV5645MIPI_write_cmos_sensor(0x3212,0x13); 
	OV5645MIPI_write_cmos_sensor(0x3212,0xa3);
	pr_info("[OV5645MIPI]exit OV5645MIPI_scene_mode_LANDSCAPE function:\n ");
}
void OV5645MIPI_scene_mode_SUNSET()
{
	pr_info("[OV5645MIPI]enter OV5645MIPI_scene_mode_SUNSET function:\n ");
	OV5645MIPI_write_cmos_sensor(0x3212,0x03);
	/*FRAME rate*/
	OV5645MIPI_write_cmos_sensor(0x3A00,0x3c); //10-30
	OV5645MIPI_write_cmos_sensor(0x3a02,0x0b); 
	OV5645MIPI_write_cmos_sensor(0x3a03,0x88);                         
	OV5645MIPI_write_cmos_sensor(0x3a14,0x0b); 
	OV5645MIPI_write_cmos_sensor(0x3a15,0x88);
	OV5645MIPI_write_cmos_sensor(0x350c,0x00);
	OV5645MIPI_write_cmos_sensor(0x350d,0x00);
	/*AE Weight - CenterAverage*/
	OV5645MIPI_write_cmos_sensor(0x501d,0x00);
	OV5645MIPI_write_cmos_sensor(0x5688,0x11);
	OV5645MIPI_write_cmos_sensor(0x5689,0x11);
	OV5645MIPI_write_cmos_sensor(0x568a,0x11);
	OV5645MIPI_write_cmos_sensor(0x568b,0x11);
	OV5645MIPI_write_cmos_sensor(0x568c,0x11);
	OV5645MIPI_write_cmos_sensor(0x568d,0x11);
	OV5645MIPI_write_cmos_sensor(0x568e,0x11);
	OV5645MIPI_write_cmos_sensor(0x568f,0x11);
	OV5645MIPI_write_cmos_sensor(0x3212,0x13);
	OV5645MIPI_write_cmos_sensor(0x3212,0xa3);
	pr_info("[OV5645MIPI]exit OV5645MIPI_scene_mode_SUNSET function:\n ");
}
void OV5645MIPI_scene_mode_SPORTS()
{
	pr_info("[OV5645MIPI]enter OV5645MIPI_scene_mode_SPORTS function:\n ");
	OV5645MIPI_write_cmos_sensor(0x3212,0x03);
	/*FRAME rate*/
#if 0   //Fix SBELI-302
	OV5645MIPI_write_cmos_sensor(0x3A00,0x38); //10-30
	OV5645MIPI_write_cmos_sensor(0x3a02,0x03); 
	OV5645MIPI_write_cmos_sensor(0x3a03,0xd8);                         
	OV5645MIPI_write_cmos_sensor(0x3a14,0x03); 
	OV5645MIPI_write_cmos_sensor(0x3a15,0xd8);
	OV5645MIPI_write_cmos_sensor(0x350c,0x00);
	OV5645MIPI_write_cmos_sensor(0x350d,0x00);
#else
	OV5645MIPI_write_cmos_sensor(0x3A00,0x3c); //10-30
	OV5645MIPI_write_cmos_sensor(0x3a02,0x0b);
	OV5645MIPI_write_cmos_sensor(0x3a03,0x88);
	OV5645MIPI_write_cmos_sensor(0x3a14,0x0b);
	OV5645MIPI_write_cmos_sensor(0x3a15,0x88);
	OV5645MIPI_write_cmos_sensor(0x350c,0x00);
	OV5645MIPI_write_cmos_sensor(0x350d,0x00);
#endif
	/*AE Weight - CenterAverage*/ 
	OV5645MIPI_write_cmos_sensor(0x501d,0x00);
	OV5645MIPI_write_cmos_sensor(0x5688,0x11);
	OV5645MIPI_write_cmos_sensor(0x5689,0x11);
	OV5645MIPI_write_cmos_sensor(0x568a,0x11);
	OV5645MIPI_write_cmos_sensor(0x568b,0x11);
	OV5645MIPI_write_cmos_sensor(0x568c,0x11);
	OV5645MIPI_write_cmos_sensor(0x568d,0x11);
	OV5645MIPI_write_cmos_sensor(0x568e,0x11);
	OV5645MIPI_write_cmos_sensor(0x568f,0x11);
    OV5645MIPI_write_cmos_sensor(0x3212,0x13); 
	OV5645MIPI_write_cmos_sensor(0x3212,0xa3);
	pr_info("[OV5645MIPI]exit OV5645MIPI_scene_mode_SPORTS function:\n ");
}
void OV5645MIPI_scene_mode_OFF()
{
	pr_info("[OV5645MIPI]enter OV5645MIPI_scene_mode_OFF function:\n ");
	OV5645MIPI_write_cmos_sensor(0x3212,0x03);
	/*FRAME rate*/
	OV5645MIPI_write_cmos_sensor(0x3A00,0x3c); //0x38--night mode off, 0x3c--night mode on--Jieve Liu
	OV5645MIPI_write_cmos_sensor(0x3a02,0x0b); 
	OV5645MIPI_write_cmos_sensor(0x3a03,0x88);						   
	OV5645MIPI_write_cmos_sensor(0x3a14,0x0b); 
	OV5645MIPI_write_cmos_sensor(0x3a15,0x88);
	OV5645MIPI_write_cmos_sensor(0x350c,0x00);
	OV5645MIPI_write_cmos_sensor(0x350d,0x00);
	/*AE Weight - CenterAverage*/
	OV5645MIPI_write_cmos_sensor(0x501d,0x00);
	OV5645MIPI_write_cmos_sensor(0x5688,0x11);
	OV5645MIPI_write_cmos_sensor(0x5689,0x11);
	OV5645MIPI_write_cmos_sensor(0x568a,0x11);
	OV5645MIPI_write_cmos_sensor(0x568b,0x11);
	OV5645MIPI_write_cmos_sensor(0x568c,0x11);
	OV5645MIPI_write_cmos_sensor(0x568d,0x11);
	OV5645MIPI_write_cmos_sensor(0x568e,0x11);
	OV5645MIPI_write_cmos_sensor(0x568f,0x11);	
	OV5645MIPI_write_cmos_sensor(0x3212,0x13); 
	OV5645MIPI_write_cmos_sensor(0x3212,0xa3);
	mDELAY(100);
	pr_info("[OV5645MIPI]exit OV5645MIPI_scene_mode_OFF function:\n ");
}
void OV5645MIPI_scene_mode_NIGHT()
{
	pr_info("[OV5645MIPI]enter OV5645MIPI_scene_mode_NIGHT function:\n ");
	OV5645MIPI_write_cmos_sensor(0x3212,0x03);
		/*FRAME rate*/
	OV5645MIPI_write_cmos_sensor(0x3A00,0x3c); //10-30
	OV5645MIPI_write_cmos_sensor(0x3a02,0x17); 
	OV5645MIPI_write_cmos_sensor(0x3a03,0x10);                         
	OV5645MIPI_write_cmos_sensor(0x3a14,0x17); 
	OV5645MIPI_write_cmos_sensor(0x3a15,0x10);
	OV5645MIPI_write_cmos_sensor(0x350c,0x00);
	OV5645MIPI_write_cmos_sensor(0x350d,0x00);
	/*AE Weight - Average*/
	OV5645MIPI_write_cmos_sensor(0x501d,0x00);
	OV5645MIPI_write_cmos_sensor(0x5688,0x11);
	OV5645MIPI_write_cmos_sensor(0x5689,0x11);
	OV5645MIPI_write_cmos_sensor(0x568a,0x11);
	OV5645MIPI_write_cmos_sensor(0x568b,0x11);
	OV5645MIPI_write_cmos_sensor(0x568c,0x11);
	OV5645MIPI_write_cmos_sensor(0x568d,0x11);
	OV5645MIPI_write_cmos_sensor(0x568e,0x11);
	OV5645MIPI_write_cmos_sensor(0x568f,0x11);
	OV5645MIPI_write_cmos_sensor(0x3212,0x13);
	OV5645MIPI_write_cmos_sensor(0x3212,0xa3);

	mdelay(10);
	pr_info("[OV5645MIPI]exit OV5645MIPI_scene_mode_NIGHT function:\n ");
}

void OV5645MIPI_set_scene_mode(u16 para)
{
	pr_info("[OV5645MIPI]enter OV5645MIPI_set_scene_mode function:\n ");	
    pr_info("[OV5645MIPI] OV5645MIPI_set_scene_mode=%d\n",para);
	
    switch (para) { 
		case SCENE_MODE_NIGHTSCENE:
			 OV5645MIPI_scene_mode_NIGHT(); 
			break;
        case SCENE_MODE_PORTRAIT:
			OV5645MIPI_scene_mode_PORTRAIT();		 
             break;
        case SCENE_MODE_LANDSCAPE:
			OV5645MIPI_scene_mode_LANDSCAPE();		 
             break;
        case SCENE_MODE_SUNSET:
			OV5645MIPI_scene_mode_SUNSET();		 
            break;
        case SCENE_MODE_SPORTS:
            OV5645MIPI_scene_mode_SPORTS();		 
            break;
        case SCENE_MODE_HDR:
            if (1 == OV5645MIPISensor.manualAEStart)
            {
                OV5645MIPI_set_AE_mode(KAL_TRUE);//Manual AE disable
                spin_lock(&ov5645mipi_drv_lock);
            	OV5645MIPISensor.manualAEStart = 0;
                OV5645MIPISensor.currentExposureTime = 0;
                OV5645MIPISensor.currentAxDGain = 0;
				spin_unlock(&ov5645mipi_drv_lock);
            }
            break;
        case SCENE_MODE_OFF:
            pr_info("[OV5645MIPI]set SCENE_MODE_OFF :\n ");
            OV5645MIPI_scene_mode_OFF();
            break;
        default:
            pr_info("[OV5645MIPI]set default mode :\n ");
            OV5645MIPI_scene_mode_OFF();
            break;
    }
	pr_info("[OV5645MIPI]exit OV5645MIPI_set_scene_mode function:\n ");
	return;
}
#endif
enum ISO_ID{
	AE_ISO_AUTO,
	AE_ISO_100,
	AE_ISO_200,
	AE_ISO_400,
};
void OV5645MIPI_set_iso(u16 para)
{
    pr_info("[OV5645MIPI]enter OV5645MIPI_set_iso function:para == %d\n",para);
    switch (para) {
        case AE_ISO_100:
		case AE_ISO_AUTO:
             OV5645MIPI_write_cmos_sensor(0x3a18, 0x00);
             OV5645MIPI_write_cmos_sensor(0x3a19, 0x60);
             break;
        case AE_ISO_200:
             OV5645MIPI_write_cmos_sensor(0x3a18, 0x00);
             OV5645MIPI_write_cmos_sensor(0x3a19, 0x90);
             break;
        case AE_ISO_400:
             OV5645MIPI_write_cmos_sensor(0x3a18, 0x00);
             OV5645MIPI_write_cmos_sensor(0x3a19, 0xc0);
             break;
        default:
             break;
    }
    pr_info("[OV5645MIPI]exit OV5645MIPI_set_iso function:\n ");
    return;
}
/*
static void ov5645_set_contrast_brightness_satuation_iso_def(void)
{
	OV5645MIPI_set_contrast(5);
	OV5645MIPI_set_brightness(5);
	OV5645MIPI_set_saturation(5);
	OV5645MIPI_set_iso(AE_ISO_AUTO);
}
*/
static void OV5645_set_AE_mode(bool AE_enable)
{
	u8 AeTemp;
	pr_info("[OV5645MIPI]enter OV5645MIPI_set_AE_mode function:\n ");
	AeTemp = i2c_get_byte(g_client, 0x3503);
	if (AE_enable == true) {
		// turn on AEC/AGC
		OV5645MIPI_write_cmos_sensor(0x3503,(AeTemp&(~0x07)));
	} else {
		// turn off AEC/AGC
		OV5645MIPI_write_cmos_sensor(0x3503,(AeTemp|0x07));
	}
	pr_info("[OV5645MIPI]exit OV5645MIPI_set_AE_mode function:\n ");
}
static void OV5645_set_AWB_mode(bool AWB_enable)
{
	u8 AwbTemp;
	pr_info("[OV5645MIPI]enter OV5645MIPI_set_AWB_mode function:\n ");
	AwbTemp = i2c_get_byte(g_client, 0x3406);   

	if (AWB_enable == true) {
		OV5645MIPI_write_cmos_sensor(0x3406,AwbTemp&0xFE); 
	} else {             
		OV5645MIPI_write_cmos_sensor(0x3406,AwbTemp|0x01); 
	}
	pr_info("[OV5645MIPI]exit OV5645MIPI_set_AWB_mode function:\n ");
}

/*************************************************************************
* FUNCTION
*    ov5645_set_param_wb
*
* DESCRIPTION
*    wb setting.
*
* PARAMETERS
*    none
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void ov5645_set_param_wb(struct ov5645_device *dev,enum  camera_wb_flip_e para)//??ƽ??
{
	//struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);

	switch (para) {      
	case CAM_WB_AUTO://?Զ?
		OV5645_set_AWB_mode(true);
		break;
	
	case CAM_WB_CLOUD: //????
		OV5645MIPI_write_cmos_sensor(0x3212,0x03); 
		OV5645_set_AWB_mode(false);         	                
		OV5645MIPI_write_cmos_sensor(0x3400,0x06); 
		OV5645MIPI_write_cmos_sensor(0x3401,0x30); 
		OV5645MIPI_write_cmos_sensor(0x3402,0x04); 
		OV5645MIPI_write_cmos_sensor(0x3403,0x00); 
		OV5645MIPI_write_cmos_sensor(0x3404,0x04); 
		OV5645MIPI_write_cmos_sensor(0x3405,0x30);                          
		OV5645MIPI_write_cmos_sensor(0x3212,0x13); 
		OV5645MIPI_write_cmos_sensor(0x3212,0xa3);

		break;
	
	case CAM_WB_DAYLIGHT: //
		OV5645MIPI_write_cmos_sensor(0x3212,0x03);
		OV5645_set_AWB_mode(false);                           
		OV5645MIPI_write_cmos_sensor(0x3400,0x06); 
		OV5645MIPI_write_cmos_sensor(0x3401,0x10); 
		OV5645MIPI_write_cmos_sensor(0x3402,0x04); 
		OV5645MIPI_write_cmos_sensor(0x3403,0x00); 
		OV5645MIPI_write_cmos_sensor(0x3404,0x04); 
		OV5645MIPI_write_cmos_sensor(0x3405,0x48);                           
		OV5645MIPI_write_cmos_sensor(0x3212,0x13); 
		OV5645MIPI_write_cmos_sensor(0x3212,0xa3);

		break;
	
	case CAM_WB_INCANDESCENCE: 
		OV5645MIPI_write_cmos_sensor(0x3212,0x03);
		OV5645_set_AWB_mode(false);                           
		OV5645MIPI_write_cmos_sensor(0x3400,0x04); 
		OV5645MIPI_write_cmos_sensor(0x3401,0xe0); 
		OV5645MIPI_write_cmos_sensor(0x3402,0x04); 
		OV5645MIPI_write_cmos_sensor(0x3403,0x00); 
		OV5645MIPI_write_cmos_sensor(0x3404,0x05); 
		OV5645MIPI_write_cmos_sensor(0x3405,0xa0);                           
		OV5645MIPI_write_cmos_sensor(0x3212,0x13); 
		OV5645MIPI_write_cmos_sensor(0x3212,0xa3);

		break;
	    
	case CAM_WB_TUNGSTEN: 
		OV5645MIPI_write_cmos_sensor(0x3212,0x03);
		OV5645_set_AWB_mode(false);
		OV5645MIPI_write_cmos_sensor(0x3400,0x05); 
		OV5645MIPI_write_cmos_sensor(0x3401,0x48); 
		OV5645MIPI_write_cmos_sensor(0x3402,0x04); 
		OV5645MIPI_write_cmos_sensor(0x3403,0x00); 
		OV5645MIPI_write_cmos_sensor(0x3404,0x05); 
		OV5645MIPI_write_cmos_sensor(0x3405,0xe0); 
		OV5645MIPI_write_cmos_sensor(0x3212,0x13); 
		OV5645MIPI_write_cmos_sensor(0x3212,0xa3);
	
		break;
	
	case CAM_WB_FLUORESCENT:
		OV5645MIPI_write_cmos_sensor(0x3212,0x03);
		OV5645_set_AWB_mode(false);                           
		OV5645MIPI_write_cmos_sensor(0x3400,0x04); 
		OV5645MIPI_write_cmos_sensor(0x3401,0x00); 
		OV5645MIPI_write_cmos_sensor(0x3402,0x04); 
		OV5645MIPI_write_cmos_sensor(0x3403,0x00); 
		OV5645MIPI_write_cmos_sensor(0x3404,0x06); 
		OV5645MIPI_write_cmos_sensor(0x3405,0x50);                           
		OV5645MIPI_write_cmos_sensor(0x3212,0x13); 
		OV5645MIPI_write_cmos_sensor(0x3212,0xa3);
		break;
	
	case CAM_WB_MANUAL:
	        // TODO
		break;
        default:
        	break;
	}
} /* ov5645_set_param_wb */
/*************************************************************************
* FUNCTION
*    ov5645_set_param_exposure
*
* DESCRIPTION
*    exposure setting.
*
* PARAMETERS
*    none
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void ov5645_set_param_exposure(struct ov5645_device *dev,enum camera_exposure_e para)//?ع?????
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);


	switch (para) {
		/*
	case EXPOSURE_N4_STEP:  //??4??  
		i2c_put_byte(client,0x3a0f , 0x10);
		i2c_put_byte(client,0x3a10 , 0x08);
		i2c_put_byte(client,0x3a1b , 0x10);
		i2c_put_byte(client,0x3a1e , 0x08);
		i2c_put_byte(client,0x3a11 , 0x20);
		i2c_put_byte(client,0x3a1f , 0x10);
		break;
	    
	case EXPOSURE_N3_STEP:
		i2c_put_byte(client,0x3a0f , 0x18);
		i2c_put_byte(client,0x3a10 , 0x10);
		i2c_put_byte(client,0x3a1b , 0x18);
		i2c_put_byte(client,0x3a1e , 0x10);
		i2c_put_byte(client,0x3a11 , 0x30);
		i2c_put_byte(client,0x3a1f , 0x10);
		break;
	    */
	case EXPOSURE_N2_STEP:
		i2c_put_byte(client, 0x3a0f, 0x16);//	; AEC in H
		i2c_put_byte(client, 0x3a10, 0x10);//	; AEC in L
		i2c_put_byte(client, 0x3a11, 0x3c);//	; AEC out H
		i2c_put_byte(client, 0x3a1b, 0x16);//	; AEC out L
		i2c_put_byte(client, 0x3a1e, 0x10);//	; control zone H
		i2c_put_byte(client, 0x3a1f, 0x0c);//	; control zone L
		break;
	    
	case EXPOSURE_N1_STEP:
		i2c_put_byte(client, 0x3a0f, 0x1e);//	; AEC in H
		i2c_put_byte(client, 0x3a10, 0x18);//	; AEC in L
		i2c_put_byte(client, 0x3a11, 0x3c);//	; AEC out H
		i2c_put_byte(client, 0x3a1b, 0x1e);//	; AEC out L
		i2c_put_byte(client, 0x3a1e, 0x18);//	; control zone H
		i2c_put_byte(client, 0x3a1f, 0x0c);//	; control zone L
		break;
	    
	case EXPOSURE_0_STEP://Ĭ???㵵
		i2c_put_byte(client, 0x3a0f, 0x22);//	; AEC in H
		i2c_put_byte(client, 0x3a10, 0x1e);//	; AEC in L
		i2c_put_byte(client, 0x3a11, 0x45);//	; AEC out H
		i2c_put_byte(client, 0x3a1b, 0x22);//	; AEC out L
		i2c_put_byte(client, 0x3a1e, 0x1e);//	; control zone H
		i2c_put_byte(client, 0x3a1f, 0x10);
		break;
	    
	case EXPOSURE_P1_STEP://??һ??
		i2c_put_byte(client, 0x3a0f, 0x40);//	; AEC in H
		i2c_put_byte(client, 0x3a10, 0x38);//	; AEC in L
		i2c_put_byte(client, 0x3a11, 0x80);//	; AEC out H
		i2c_put_byte(client, 0x3a1b, 0x40);//	; AEC out L
		i2c_put_byte(client, 0x3a1e, 0x38);//	; control zone H
		i2c_put_byte(client, 0x3a1f, 0x1c);//	; control zone L
		break;
	    
	case EXPOSURE_P2_STEP:
		i2c_put_byte(client, 0x3a0f, 0x50);//	; AEC in H
		i2c_put_byte(client, 0x3a10, 0x48);//	; AEC in L
		i2c_put_byte(client, 0x3a11, 0x90);//	; AEC out H
		i2c_put_byte(client, 0x3a1b, 0x50);//	; AEC out L
		i2c_put_byte(client, 0x3a1e, 0x48);//	; control zone H
		i2c_put_byte(client, 0x3a1f, 0x24);//	; control zone L
		break;
	/*
	case EXPOSURE_P3_STEP:
		i2c_put_byte(client,0x3a0f , 0x58);
		i2c_put_byte(client,0x3a10 , 0x50);
		i2c_put_byte(client,0x3a11 , 0x91);
		i2c_put_byte(client,0x3a1b , 0x58);
		i2c_put_byte(client,0x3a1e , 0x50);
		i2c_put_byte(client,0x3a1f , 0x20);
		break;
	    
	case EXPOSURE_P4_STEP:    
		i2c_put_byte(client,0x3a0f , 0x60);
		i2c_put_byte(client,0x3a10 , 0x58);
		i2c_put_byte(client,0x3a11 , 0xa0);
		i2c_put_byte(client,0x3a1b , 0x60);
		i2c_put_byte(client,0x3a1e , 0x58);
		i2c_put_byte(client,0x3a1f , 0x20);
		break;
	*/    
	default:
		i2c_put_byte(client,0x3a0f, 0x22);
		i2c_put_byte(client,0x3a10, 0x1e);
		i2c_put_byte(client,0x3a11, 0x45);
		i2c_put_byte(client,0x3a1b, 0x22);
		i2c_put_byte(client,0x3a1e, 0x1e);
		i2c_put_byte(client,0x3a1f, 0x10);
		break;
	}
} /* ov5645_set_param_exposure */
/*************************************************************************
* FUNCTION
*    ov5645_set_param_effect
*
* DESCRIPTION
*    effect setting.
*
* PARAMETERS
*    none
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void ov5645_set_param_effect(struct ov5645_device *dev,enum camera_effect_flip_e para)//??Ч????
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
  
	switch (para) {
	case CAM_EFFECT_ENC_NORMAL:
		i2c_put_byte(client, 0x5580, 0x06); 
        i2c_put_byte(client, 0x5583, 0x38); //0x40
        i2c_put_byte(client, 0x5584, 0x20);
		break;        
	
	case CAM_EFFECT_ENC_GRAYSCALE:
		break;
	
	case CAM_EFFECT_ENC_SEPIA:
		i2c_put_byte(client, 0x5580, 0x1e);
        i2c_put_byte(client, 0x5583, 0x40); 
        i2c_put_byte(client, 0x5584, 0xa0);
		break;        
	        
	case CAM_EFFECT_ENC_SEPIAGREEN:
		i2c_put_byte(client, 0x5580,0x1e);			 
        i2c_put_byte(client, 0x5583,0x60); 
        i2c_put_byte(client, 0x5584,0x60);
		break;                    
	
	case CAM_EFFECT_ENC_SEPIABLUE:
		i2c_put_byte(client, 0x5580,0x1e);             
        i2c_put_byte(client, 0x5583,0xa0); 
        i2c_put_byte(client, 0x5584,0x40);
		break;                                
	
	case CAM_EFFECT_ENC_COLORINV:
		i2c_put_byte(client,0x5001,0x83);
		i2c_put_byte(client,0x5580,0x40);
		break;        
	
	default:
		break;
	}
} /* ov5645_set_param_effect */

/*************************************************************************
* FUNCTION
*	ov5645_night_mode
*
* DESCRIPTION
*    This function night mode of ov5645.
*
* PARAMETERS
*    none
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void ov5645_set_param_banding(struct ov5645_device *dev,enum  camera_banding_flip_e banding)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	//unsigned char buf[4];
	switch(banding){
	case CAM_BANDING_60HZ:
		printk("set banding 60Hz\n");
		i2c_put_byte(client, 0x3c00, 0x00);
		i2c_put_byte(client, 0x3c01, 0x80);
		break;
	case CAM_BANDING_50HZ:
		printk("set banding 50Hz\n");
		i2c_put_byte(client, 0x3c00, 0x04);
		i2c_put_byte(client, 0x3c01, 0x80);
		break;
	default:
		break;
	}
}

static int ov5645_AutoFocus(struct ov5645_device *dev, int focus_mode)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	int ret = 0;
	//int i = 0;
    
	switch (focus_mode) {
	case CAM_FOCUS_MODE_AUTO:
		i2c_put_byte(client, 0x3023 , 0x1);
		i2c_put_byte(client, 0x3022 , 0x3); //start to auto focus
		bDoingAutoFocusMode = true;
		#if 0
		while(i2c_get_byte(client, 0x3023) && i < 40){//200) {//wait for the auto focus to be done
			//printk("waiting for focus ready\n");
			i++;
			msleep(10);
		}
		
		if (i2c_get_byte(client, 0x3028) == 0)
			ret = -1;
		else {
			msleep(10);
			i2c_put_byte(client, 0x3022 , 0x6); //pause the auto focus
			i2c_put_byte(client, 0x3023 , 0x1);
		}
		#endif
		printk("auto mode start\n");
		break;
		
	case CAM_FOCUS_MODE_CONTI_VID:
	case CAM_FOCUS_MODE_CONTI_PIC:
		i2c_put_byte(client, 0x3023 , 0x1);
		i2c_put_byte(client, 0x3022 , 0x4); //start to auto focus
		/*while(i2c_get_byte(client, 0x3023) == 0x1) {
			msleep(10);
		}*/
		printk("start continous focus\n");
		break;
		
	case CAM_FOCUS_MODE_RELEASE:
	case CAM_FOCUS_MODE_FIXED:
	default:
		i2c_put_byte(client, 0x3023 , 0x1);
		i2c_put_byte(client, 0x3022 , 0x6);
		printk("release focus to infinit\n");
		break;
	}
	return ret;

}    /* ov5645_AutoFocus */

static int ov5645_FlashCtrl(struct ov5645_device *dev, int flash_mode)
{
	//struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	int ret = 0;
    
	switch (flash_mode) {
	case FLASHLIGHT_ON:
	case FLASHLIGHT_AUTO:
		if (dev->cam_info.torch_support)
			aml_cam_torch(&dev->cam_info, 1);
		aml_cam_flash(&dev->cam_info, 1);
		break;
	case FLASHLIGHT_TORCH:
		if (dev->cam_info.torch_support) {
			aml_cam_torch(&dev->cam_info, 1);
			aml_cam_flash(&dev->cam_info, 0);
		} else 
			aml_cam_torch(&dev->cam_info, 1);
		break;
	case FLASHLIGHT_OFF:
		aml_cam_flash(&dev->cam_info, 0);
		if (dev->cam_info.torch_support)
			aml_cam_torch(&dev->cam_info, 0);
		break;
	default:
		printk("this flash mode not support yet\n");
		break;
	}
	return ret;

}    /* ov5645_FlashCtrl */

static resolution_size_t get_size_type(int width, int height)
{
	resolution_size_t rv = SIZE_NULL;
	if (width * height >= 2500 * 1900)
		rv = SIZE_2592X1944;
	else if (width * height >= 2000 * 1500)
		rv = SIZE_2048X1536;
	else if (width * height >= 1920 * 1080)
		rv = SIZE_1920X1080;
	else if (width * height >= 1600 * 1200)
		rv = SIZE_1600X1200;
	else if (width * height >= 1280 * 960)
		rv = SIZE_1280X960;
	else if (width * height >= 1280 * 720)
		rv = SIZE_1280X720;
	else if (width * height >= 1024 * 768)
		rv = SIZE_1024X768;
	else if (width * height >= 800 * 600)
		rv = SIZE_800X600;
	else if (width * height >= 600 * 400)
		rv = SIZE_640X480;
	else if (width * height >= 352 * 288)
		rv = SIZE_352X288;
	else if (width * height >= 300 * 200)
		rv = SIZE_320X240;
	else if (width * height >= 170 * 140)
		rv = SIZE_176X144;
	return rv;
}

static int set_flip(struct ov5645_device *dev)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	unsigned char temp;
	
	// Crestron  - forcing to 1 here makes stream video NOT flipped horizontally.
	dev->cam_info.m_flip = 1;	
	// Crestron end
	
	temp = i2c_get_byte(client, 0x3821);
	temp &= 0xf9;
	temp |= dev->cam_info.m_flip << 1 | dev->cam_info.m_flip << 2;
	if((i2c_put_byte(client, 0x3821, temp)) < 0) {
		printk("fail in setting sensor orientation\n");
		return -1;
	}
	temp = i2c_get_byte(client, 0x3820);
	temp &= 0xf9;
	temp |= dev->cam_info.v_flip << 1 | dev->cam_info.v_flip << 2;
	if((i2c_put_byte(client, 0x3820, temp)) < 0) {
		printk("fail in setting sensor orientation\n");
		return -1;
	}

	return 0;
}

static resolution_param_t* get_resolution_param(struct ov5645_device *dev, int is_capture, int width, int height)
{
	int i = 0;
	int arry_size = 0;
	resolution_param_t* tmp_resolution_param = NULL;
	resolution_size_t res_type = SIZE_NULL;
	res_type = get_size_type(width, height);
	if (res_type == SIZE_NULL)
		return NULL;
	if (is_capture) {
		tmp_resolution_param = capture_resolution_array;
		arry_size = sizeof(capture_resolution_array);
	} else {
		tmp_resolution_param = prev_resolution_array;
		arry_size = sizeof(prev_resolution_array);
		ov5645_frmintervals_active.denominator = 23;
		ov5645_frmintervals_active.numerator = 1;
	}
	
	for (i = 0; i < arry_size; i++) {
		if (tmp_resolution_param[i].size_type == res_type) {
			ov5645_frmintervals_active.denominator = tmp_resolution_param[i].active_fps;
			ov5645_frmintervals_active.numerator = 1;
			return &tmp_resolution_param[i];
		}
	}
	return NULL;
}

static int set_resolution_param(struct ov5645_device *dev, resolution_param_t* res_param)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	int i=0;
	//int rc = -1;
	if (!res_param->reg_script) {
		printk("error, resolution reg script is NULL\n");
		return -1;
	}
	while(1) {
		if (res_param->reg_script[i].val==0xff&&res_param->reg_script[i].addr==0xffff) {
			printk("setting resolutin param complete\n");
			break;
		}
		if((i2c_put_byte(client,res_param->reg_script[i].addr, res_param->reg_script[i].val)) < 0) {
			printk("fail in setting resolution param. i=%d\n",i);
			break;
		}else {
            //pr_info("set %04x %02x \n", res_param->reg_script[i].addr, res_param->reg_script[i].val );
        }
		i++;
	}
	dev->cur_resolution_param = res_param;
	set_flip(dev);
	
	return 0;
}

static int set_focus_zone(struct ov5645_device *dev, int value)
{
	int xc, yc;
	//struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	int retry_count = 10;
	int ret = -1;
	
	xc = ((value >> 16) & 0xffff) * 80 / 2000;
	yc = (value & 0xffff) * 60 / 2000;
	printk("xc = %d, yc = %d\n", xc, yc);
	/*
	i2c_put_byte(client, CMD_PARA0, xc);
	i2c_put_byte(client, CMD_PARA1, yc);
	i2c_put_byte(client, CMD_ACK, 0x01);
	i2c_put_byte(client, CMD_MAIN, 0x81);
	do {
		msleep(5);
		pr_info("waiting for focus zone to be set\n");
	} while (i2c_get_byte(client, CMD_ACK) && retry_count--);
	*/
	if (retry_count)
		ret = 0;
	return ret;
}

unsigned char v4l_2_ov5645(int val)
{
	int ret=val/0x20;
	if(ret<4) return ret*0x20+0x80;
	else if(ret<8) return ret*0x20+0x20;
	else return 0;
}

static int convert_canvas_index(unsigned int v4l2_format, unsigned int start_canvas)
{
	int canvas = start_canvas;

	switch(v4l2_format){
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_VYUY:
		canvas = start_canvas;
		break;
	case V4L2_PIX_FMT_YUV444:
	case V4L2_PIX_FMT_BGR24:
	case V4L2_PIX_FMT_RGB24:
		canvas = start_canvas;
		break; 
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21: 
		canvas = start_canvas | ((start_canvas+1)<<8);
		break;
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_YUV420:
		if(V4L2_PIX_FMT_YUV420 == v4l2_format){
			canvas = start_canvas|((start_canvas+1)<<8)|((start_canvas+2)<<16);
		}else{
			canvas = start_canvas|((start_canvas+2)<<8)|((start_canvas+1)<<16);
		}
		break;
	default:
		break;
	}
	return canvas;
}

static int ov5645_setting(struct ov5645_device *dev,int PROP_ID,int value ) 
{
	int ret=0;
	//unsigned char cur_val;
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	switch(PROP_ID)  {
	case V4L2_CID_BRIGHTNESS:
		mutex_lock(&firmware_mutex);
		pr_info("setting brightned:%d\n",v4l_2_ov5645(value));
		//ret=i2c_put_byte(client,0x0201,v4l_2_ov5645(value));
		//OV5645MIPI_set_brightness();
		mutex_unlock(&firmware_mutex);
		break;
	case V4L2_CID_CONTRAST:
		mutex_lock(&firmware_mutex);
		ret=i2c_put_byte(client,0x0200, value);
		mutex_unlock(&firmware_mutex);
		break;    
	case V4L2_CID_SATURATION:
		mutex_lock(&firmware_mutex);
		ret=i2c_put_byte(client,0x0202, value);
		mutex_unlock(&firmware_mutex);
		break;
	case V4L2_CID_HFLIP:    /* set flip on H. */
		value = value & 0x3;
		if(ov5645_qctrl[2].default_value!=value){
			ov5645_qctrl[2].default_value=value;
			printk(" set camera  h filp =%d. \n ",value);
		}
		break;
	case V4L2_CID_VFLIP:    /* set flip on V. */
		break;    
	case V4L2_CID_DO_WHITE_BALANCE:
		mutex_lock(&firmware_mutex);
		if(ov5645_qctrl[4].default_value!=value){
			ov5645_qctrl[4].default_value=value;
			ov5645_set_param_wb(dev,value);
			printk(KERN_INFO " set camera  white_balance=%d. \n ",value);
		}
		mutex_unlock(&firmware_mutex);
		break;
	case V4L2_CID_EXPOSURE:
		mutex_lock(&firmware_mutex);
		if(ov5645_qctrl[5].default_value!=value){
			ov5645_qctrl[5].default_value=value;
			ov5645_set_param_exposure(dev,value);
			printk(KERN_INFO " set camera  exposure=%d. \n ",value);
		}
		mutex_unlock(&firmware_mutex);
		break;
	case V4L2_CID_COLORFX:
		mutex_lock(&firmware_mutex);
		if(ov5645_qctrl[6].default_value!=value){
			ov5645_qctrl[6].default_value=value;
			ov5645_set_param_effect(dev,value);
			printk(KERN_INFO " set camera  effect=%d. \n ",value);
		}
		mutex_unlock(&firmware_mutex);
		break;
	case V4L2_CID_WHITENESS:
		mutex_lock(&firmware_mutex);
		if(ov5645_qctrl[7].default_value!=value){
			ov5645_qctrl[7].default_value=value;
			ov5645_set_param_banding(dev,value);
			printk(KERN_INFO " set camera  banding=%d. \n ",value);
		}
		mutex_unlock(&firmware_mutex);
		break;
	case V4L2_CID_FOCUS_AUTO:
		pr_info("ov5645_setting, V4L2_CID_FOCUS_AUTO \n");
		mutex_lock(&firmware_mutex);
		if (ov5645_have_opened) {
			if (dev->firmware_ready) 
				ret = ov5645_AutoFocus(dev,value);
			else if (value == CAM_FOCUS_MODE_CONTI_VID ||
        				value == CAM_FOCUS_MODE_CONTI_PIC)
				start_focus_mode = value;
			else
				ret = -1;
		}
		mutex_unlock(&firmware_mutex);
		break;
	case V4L2_CID_BACKLIGHT_COMPENSATION:
		if (dev->cam_info.flash_support) 
			ret = ov5645_FlashCtrl(dev,value);
		else
			ret = -1;
		break;
	case V4L2_CID_ZOOM_ABSOLUTE:
		if(ov5645_qctrl[10].default_value!=value){
			ov5645_qctrl[10].default_value=value;
			//printk(KERN_INFO " set camera  zoom mode=%d. \n ",value);
		}
		break;
	case V4L2_CID_ROTATE:
		if(ov5645_qctrl[11].default_value!=value){
			ov5645_qctrl[11].default_value=value;
			printk(" set camera  rotate =%d. \n ",value);
		}
		break;
	case V4L2_CID_FOCUS_ABSOLUTE:
		if(ov5645_qctrl[12].default_value!=value){
			ov5645_qctrl[12].default_value=value;
			printk(" set camera  focus zone =%d. \n ",value);
			set_focus_zone(dev, value);
		}
		break;
	default:
		ret=-1;
		break;
	}
	return ret;
    
}

static void power_down_ov5645(struct ov5645_device *dev)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	i2c_put_byte(client,0x3022, 0x8); //release focus
	i2c_put_byte(client,0x3008, 0x42);//in soft power down mode
}

/* ------------------------------------------------------------------
	DMA and thread functions
   ------------------------------------------------------------------*/

#define TSTAMP_MIN_Y	24
#define TSTAMP_MAX_Y	(TSTAMP_MIN_Y + 15)
#define TSTAMP_INPUT_X	10
#define TSTAMP_MIN_X	(54 + TSTAMP_INPUT_X)

static void ov5645_fillbuff(struct ov5645_fh *fh, struct ov5645_buffer *buf)
{
	struct ov5645_device *dev = fh->dev;
	//void *vbuf = videobuf_to_vmalloc(&buf->vb);
	void *vbuf = (void *)videobuf_to_res(&buf->vb);
	vm_output_para_t para = {0};
	dprintk(dev,1,"%s\n", __func__);    
	if (!vbuf)
    	return;
	/*  0x18221223 indicate the memory type is MAGIC_VMAL_MEM*/
	if(buf->canvas_id == 0)
           buf->canvas_id = convert_canvas_index(fh->fmt->fourcc, ov5645_RES0_CANVAS_INDEX+buf->vb.i*3);
	para.mirror = ov5645_qctrl[2].default_value&3;
	para.v4l2_format = fh->fmt->fourcc;
	para.v4l2_memory = MAGIC_RE_MEM;//0x18221223;
	para.zoom = ov5645_qctrl[10].default_value;
	para.angle = ov5645_qctrl[11].default_value;
	para.vaddr = (unsigned)vbuf;
	para.ext_canvas = buf->canvas_id;
	para.width = buf->vb.width;
	para.height = buf->vb.height;//(buf->vb.height==1080)?1088:buf->vb.height; //Kevin
	vm_fill_buffer(&buf->vb,&para);
	buf->vb.state = VIDEOBUF_DONE;
}

static void ov5645_thread_tick(struct ov5645_fh *fh)
{
	struct ov5645_buffer *buf;
	struct ov5645_device *dev = fh->dev;
	struct ov5645_dmaqueue *dma_q = &dev->vidq;

	unsigned long flags = 0;

	dprintk(dev, 1, "Thread tick\n");
	if(!fh->stream_on){
		dprintk(dev, 1, "sensor doesn't stream on\n");
		return ;
	}

	spin_lock_irqsave(&dev->slock, flags);
	if (list_empty(&dma_q->active)) {
		dprintk(dev, 1, "No active queue to serve\n");
		goto unlock;
	}

	buf = list_entry(dma_q->active.next,
 			struct ov5645_buffer, vb.queue);
	dprintk(dev, 1, "%s\n", __func__);
	dprintk(dev, 1, "list entry get buf is %x\n", (unsigned)buf);


	if(!(fh->f_flags & O_NONBLOCK)){
		/* Nobody is waiting on this buffer, return */
		if (!waitqueue_active(&buf->vb.done))
			goto unlock;
	}
	buf->vb.state = VIDEOBUF_ACTIVE;

	list_del(&buf->vb.queue);

	do_gettimeofday(&buf->vb.ts);

	/* Fill buffer */
	spin_unlock_irqrestore(&dev->slock, flags);
	ov5645_fillbuff(fh, buf);
	dprintk(dev, 1, "filled buffer %p\n", buf);

	wake_up(&buf->vb.done);
	dprintk(dev, 2, "[%p/%d] wakeup\n", buf, buf->vb. i);
	return;
unlock:
	spin_unlock_irqrestore(&dev->slock, flags);
	return;
}

static void ov5645_sleep(struct ov5645_fh *fh)
{
	struct ov5645_device *dev = fh->dev;
	struct ov5645_dmaqueue *dma_q = &dev->vidq;

	//int timeout;
	DECLARE_WAITQUEUE(wait, current);

	dprintk(dev, 1, "%s dma_q=0x%08lx\n", __func__,
        (unsigned long)dma_q);

	add_wait_queue(&dma_q->wq, &wait);
	if (kthread_should_stop())
    	goto stop_task;

	/* Calculate time to wake up */
	//timeout = msecs_to_jiffies(1);

	ov5645_thread_tick(fh);

	schedule_timeout_interruptible(1);

stop_task:
	remove_wait_queue(&dma_q->wq, &wait);
	try_to_freeze();
}

static int ov5645_thread(void *data)
{
	struct ov5645_fh  *fh = data;
	struct ov5645_device *dev = fh->dev;

	dprintk(dev, 1, "thread started\n");

	set_freezable();

	for (;;) {
		ov5645_sleep(fh);
		
		if (kthread_should_stop())
			break;
	}
	dprintk(dev, 1, "thread: exit\n");
	return 0;
}

static int ov5645_start_thread(struct ov5645_fh *fh)
{
	struct ov5645_device *dev = fh->dev;
	struct ov5645_dmaqueue *dma_q = &dev->vidq;

	dma_q->frame = 0;
	dma_q->ini_jiffies = jiffies;

	dprintk(dev, 1, "%s\n", __func__);

	dma_q->kthread = kthread_run(ov5645_thread, fh, "ov5645");

	if (IS_ERR(dma_q->kthread)) {
		v4l2_err(&dev->v4l2_dev, "kernel_thread() failed\n");
		return PTR_ERR(dma_q->kthread);
	}
	/* Wakes thread */
	wake_up_interruptible(&dma_q->wq);

	dprintk(dev, 1, "returning from %s\n", __func__);
	return 0;
}

static void ov5645_stop_thread(struct ov5645_dmaqueue  *dma_q)
{
	struct ov5645_device *dev = container_of(dma_q, struct ov5645_device, vidq);

	dprintk(dev, 1, "%s\n", __func__);
	/* shutdown control thread */
	if (dma_q->kthread) {
		kthread_stop(dma_q->kthread);
		dma_q->kthread = NULL;
	}
}

/* ------------------------------------------------------------------
	Videobuf operations
   ------------------------------------------------------------------*/
static int
buffer_setup(struct videobuf_queue *vq, unsigned int *count, unsigned int *size)
{
	struct videobuf_res_privdata *res = vq->priv_data;
	struct ov5645_fh *fh  = container_of(res, struct ov5645_fh, res);
	struct ov5645_device *dev  = fh->dev;
	//int bytes = fh->fmt->depth >> 3 ;
	int height = fh->height;
	if(height==1080)
           height = 1088;
	*size = (fh->width*height*fh->fmt->depth)>>3;    
	if (0 == *count)
        *count = 32;

	while (*size * *count > vid_limit * 1024 * 1024)
        (*count)--;

	dprintk(dev, 1, "%s, count=%d, size=%d\n", __func__,
        *count, *size);

	return 0;
}

static void free_buffer(struct videobuf_queue *vq, struct ov5645_buffer *buf)
{
	struct videobuf_res_privdata *res = vq->priv_data;
	struct ov5645_fh *fh  = container_of(res, struct ov5645_fh, res);
	struct ov5645_device *dev  = fh->dev;

	dprintk(dev, 1, "%s, state: %i\n", __func__, buf->vb.state);
	videobuf_waiton(vq, &buf->vb, 0, 0);

	if (in_interrupt())
    		BUG();

	videobuf_res_free(vq, &buf->vb);
	dprintk(dev, 1, "free_buffer: freed\n");
	buf->vb.state = VIDEOBUF_NEEDS_INIT;
}

#define norm_maxw() 3000
#define norm_maxh() 3000
static int
buffer_prepare(struct videobuf_queue *vq, struct videobuf_buffer *vb,
                    	enum v4l2_field field)
{
	struct videobuf_res_privdata *res = vq->priv_data;
	struct ov5645_fh *fh  = container_of(res, struct ov5645_fh, res);
	struct ov5645_device    *dev = fh->dev;
	struct ov5645_buffer *buf = container_of(vb, struct ov5645_buffer, vb);
	int rc;
	//int bytes = fh->fmt->depth >> 3 ;
	dprintk(dev, 1, "%s, field=%d\n", __func__, field);

	BUG_ON(NULL == fh->fmt);

	if (fh->width  < 48 || fh->width  > norm_maxw() ||
			fh->height < 32 || fh->height > norm_maxh())
		return -EINVAL;

	buf->vb.size = (fh->width*fh->height*fh->fmt->depth)>>3;
	if (0 != buf->vb.baddr  &&  buf->vb.bsize < buf->vb.size)
    	return -EINVAL;

	/* These properties only change when queue is idle, see s_fmt */
	buf->fmt       = fh->fmt;
	buf->vb.width  = fh->width;
	buf->vb.height = fh->height;
	buf->vb.field  = field;

	//precalculate_bars(fh);

	if (VIDEOBUF_NEEDS_INIT == buf->vb.state) {
    	rc = videobuf_iolock(vq, &buf->vb, NULL);
		if (rc < 0)
			goto fail;
	}

	buf->vb.state = VIDEOBUF_PREPARED;

	return 0;

fail:
	free_buffer(vq, buf);
	return rc;
}

static void
buffer_queue(struct videobuf_queue *vq, struct videobuf_buffer *vb)
{
	struct ov5645_buffer    *buf  = container_of(vb, struct ov5645_buffer, vb);
	struct videobuf_res_privdata *res = vq->priv_data;
	struct ov5645_fh *fh  = container_of(res, struct ov5645_fh, res);
	struct ov5645_device       *dev  = fh->dev;
	struct ov5645_dmaqueue *vidq = &dev->vidq;

	dprintk(dev, 1, "%s\n", __func__);
	buf->vb.state = VIDEOBUF_QUEUED;
	list_add_tail(&buf->vb.queue, &vidq->active);
}

static void buffer_release(struct videobuf_queue *vq,
               struct videobuf_buffer *vb)
{
	struct ov5645_buffer   *buf  = container_of(vb, struct ov5645_buffer, vb);
	struct videobuf_res_privdata *res = vq->priv_data;
	struct ov5645_fh *fh  = container_of(res, struct ov5645_fh, res);
	struct ov5645_device *dev = (struct ov5645_device *)fh->dev;

	dprintk(dev, 1, "%s\n", __func__);

	free_buffer(vq, buf);
}

static struct videobuf_queue_ops ov5645_video_qops = {
	.buf_setup      = buffer_setup,
	.buf_prepare    = buffer_prepare,
	.buf_queue      = buffer_queue,
	.buf_release    = buffer_release,
};

/* ------------------------------------------------------------------
	IOCTL vidioc handling
   ------------------------------------------------------------------*/
static int vidioc_querycap(struct file *file, void  *priv,
                	struct v4l2_capability *cap)
{
	struct ov5645_fh  *fh  = priv;
	struct ov5645_device *dev = fh->dev;

	strcpy(cap->driver, "ov5645");
	strcpy(cap->card, "ov5645.canvas");
	strlcpy(cap->bus_info, dev->v4l2_dev.name, sizeof(cap->bus_info));
	cap->version = ov5645_CAMERA_VERSION;
	cap->capabilities =	V4L2_CAP_VIDEO_CAPTURE |
        			V4L2_CAP_STREAMING     |
        			V4L2_CAP_READWRITE;
	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void  *priv,
                	struct v4l2_fmtdesc *f)
{
	struct ov5645_fmt *fmt;

	if (f->index >= ARRAY_SIZE(formats))
    	return -EINVAL;

	fmt = &formats[f->index];

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
                	struct v4l2_format *f)
{
	struct ov5645_fh *fh = priv;

	printk("vidioc_g_fmt_vid_cap...fh->width =%d,fh->height=%d\n",fh->width,fh->height);
	f->fmt.pix.width        = fh->width;
	f->fmt.pix.height       = fh->height;
	f->fmt.pix.field        = fh->vb_vidq.field;
	f->fmt.pix.pixelformat  = fh->fmt->fourcc;
	f->fmt.pix.bytesperline =
		(f->fmt.pix.width * fh->fmt->depth) >> 3;
	f->fmt.pix.sizeimage =
    	f->fmt.pix.height * f->fmt.pix.bytesperline;

	return (0);
}

static int vidioc_enum_frameintervals(struct file *file, void *priv,
					struct v4l2_frmivalenum *fival)
{
	unsigned int k;
	
	if(fival->index > ARRAY_SIZE(ov5645_frmivalenum))
		return -EINVAL;
	
	for(k =0; k< ARRAY_SIZE(ov5645_frmivalenum); k++) {
		if( (fival->index==ov5645_frmivalenum[k].index)&&
				(fival->pixel_format ==ov5645_frmivalenum[k].pixel_format )&&
				(fival->width==ov5645_frmivalenum[k].width)&&
				(fival->height==ov5645_frmivalenum[k].height)){
			memcpy( fival, &ov5645_frmivalenum[k], sizeof(struct v4l2_frmivalenum));
			return 0;
		}
	}
	
	return -EINVAL;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
        	struct v4l2_format *f)
{
	struct ov5645_fh  *fh  = priv;
	struct ov5645_device *dev = fh->dev;
	struct ov5645_fmt *fmt;
	enum v4l2_field field;
	unsigned int maxw, maxh;
	
	fmt = get_format(f);
	if (!fmt) {
		dprintk(dev, 1, "Fourcc format (0x%08x) invalid.\n",
				f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	field = f->fmt.pix.field;
	
	if (field == V4L2_FIELD_ANY) {
		field = V4L2_FIELD_INTERLACED;
	} else if (V4L2_FIELD_INTERLACED != field) {
		dprintk(dev, 1, "Field type invalid.\n");
		return -EINVAL;
	}

	maxw  = norm_maxw();
	maxh  = norm_maxh();

	f->fmt.pix.field = field;
	v4l_bound_align_image(&f->fmt.pix.width, 48, maxw, 2,
                  &f->fmt.pix.height, 32, maxh, 0, 0);
	f->fmt.pix.bytesperline =
        	(f->fmt.pix.width * fmt->depth) >> 3;
	f->fmt.pix.sizeimage =
    		f->fmt.pix.height * f->fmt.pix.bytesperline;

	return 0;
}

static resolution_param_t* prev_res = NULL;

/*FIXME: This seems to be generic enough to be at videodev2 */
static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
                	struct v4l2_format *f)
{
	struct ov5645_fh *fh = priv;
	struct videobuf_queue *q = &fh->vb_vidq;
	struct ov5645_device *dev = fh->dev;
	resolution_param_t* res_param = NULL;
	int ret = 0;
	//struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	//unsigned char gain = 0, exposurelow = 0, exposuremid = 0, exposurehigh = 0;
	int cap_fps, pre_fps;
	u32 shutter = 0;	
    //u32 extshutter = 0;
    u32 color_r_gain = 0;
    u32 color_b_gain = 0;
    u32 color_g_gain = 0;
    //u32 readval = 0;

    f->fmt.pix.width = (f->fmt.pix.width + (CANVAS_WIDTH_ALIGN-1) ) & (~(CANVAS_WIDTH_ALIGN-1));
	if ((f->fmt.pix.pixelformat==V4L2_PIX_FMT_YVU420) ||
            (f->fmt.pix.pixelformat==V4L2_PIX_FMT_YUV420)){
                f->fmt.pix.width = (f->fmt.pix.width + (CANVAS_WIDTH_ALIGN*2-1) ) & (~(CANVAS_WIDTH_ALIGN*2-1));
    }
	ret = vidioc_try_fmt_vid_cap(file, fh, f);
	if (ret < 0)
    	return ret;

	mutex_lock(&q->vb_lock);

	if (videobuf_queue_is_busy(&fh->vb_vidq)) {
		dprintk(fh->dev, 1, "%s queue busy\n", __func__);
		ret = -EBUSY;
		goto out;
	}
	ret = -EINVAL;

	fh->fmt           = get_format(f);
	fh->width         = f->fmt.pix.width;
	fh->height        = f->fmt.pix.height;
	fh->vb_vidq.field = f->fmt.pix.field;
	fh->type          = f->type;
	printk("system aquire ...fh->height=%d, fh->width= %d\n",fh->height,fh->width);//potti
	if(f->fmt.pix.pixelformat==V4L2_PIX_FMT_RGB24){
		res_param = get_resolution_param(dev, 1, fh->width,fh->height);
		if (!res_param) {
			printk("error,capture resolution param not get\n");
			goto out;
		}

		pr_info("cur w:%d h:%d, willReslution w:%d, h:%d\n", dev->cur_resolution_param->active_frmsize.width, dev->cur_resolution_param->active_frmsize.height, 
				res_param->active_frmsize.width, res_param->active_frmsize.height);
		if( dev->cur_resolution_param->active_frmsize.width == res_param->active_frmsize.width &&
		 	dev->cur_resolution_param->active_frmsize.height == res_param->active_frmsize.height ) {
			pr_info("cur_resolution_param is same as willSetting\n");
			ret = 0;
			goto out;
		}

		/*get_exposure_param(dev, &gain, &exposurelow, &exposuremid, &exposurehigh);
		printk("gain=0x%x, exposurelow=0x%x, exposuremid=0x%x, exposurehigh=0x%x\n",
				 gain, exposurelow, exposuremid, exposurehigh);
		*/
#if 0
		Get_preview_exposure_gain(dev);
#else
		//OV provided
		shutter = OV5645MIPI_Read_Shutter();
		//OV5645_set_AE_mode(false);
		//OV5645_set_AWB_mode(false);
#endif
		color_r_gain = ((i2c_get_byte(g_client, 0x3401)&0xFF)+((i2c_get_byte(g_client, 0x3400)&0xFF)*256));  
		color_b_gain = ((i2c_get_byte(g_client, 0x3405)&0xFF)+((i2c_get_byte(g_client, 0x3404)&0xFF)*256));
		color_g_gain = ((i2c_get_byte(g_client, 0x3403)&0xFF)+((i2c_get_byte(g_client, 0x3402)&0xFF)*256));
		pr_info("color_r_gain:0x%04x, color_b_gain:0x%04x, color_g_gain:0x%04x,shutter:0x%04x \n", color_r_gain, color_b_gain, color_g_gain,shutter);

		set_resolution_param(dev, res_param);
		OV5645_WB_calibattion(color_r_gain, color_g_gain, color_b_gain, 1);
		shutter = shutter * 4;
		OV5645MIPI_Write_Shutter(shutter);
#if 0
		pr_info("msleep 10\n");
		//msleep(10);
		color_r_gain = ((i2c_get_byte(g_client, 0x3401)&0xFF)+((i2c_get_byte(g_client, 0x3400)&0xFF)*256));  
		color_b_gain = ((i2c_get_byte(g_client, 0x3405)&0xFF)+((i2c_get_byte(g_client, 0x3404)&0xFF)*256));
		color_g_gain = ((i2c_get_byte(g_client, 0x3403)&0xFF)+((i2c_get_byte(g_client, 0x3402)&0xFF)*256));
		pr_info("22 color_r_gain:0x%04x, color_b_gain:0x%04x, color_g_gain:0x%04x,shutter:0x%04x \n", color_r_gain, color_b_gain, color_g_gain,shutter);
		readval = i2c_get_byte(g_client, 0x3501);
		pr_info("[0x3501]:0x%04x \n", readval);
		readval = i2c_get_byte(g_client, 0x3502);
		pr_info("[0x3502]:0x%04x \n", readval);
		readval = i2c_get_byte(g_client, 0x3503);
		pr_info("[0x3503]:0x%04x \n", readval);
		readval = i2c_get_byte(g_client, 0x350b);
		pr_info("[0x350b]:0x%04x \n", readval);
#endif
		if (prev_res && (prev_res->size_type == SIZE_1280X960 
				|| prev_res->size_type == SIZE_1024X768)) {
			pre_fps = 1500;
		} else if (prev_res && prev_res->size_type == SIZE_1280X720 ) {
			pre_fps = 3000;
		} else {
			pre_fps = 1500;
		} 

		cap_fps = 500;
		//cal_exposure(dev, pre_fps, cap_fps);
		
		printk("pre_fps=%d,cap_fps=%d\n", pre_fps, cap_fps);
	} else {
		pr_info("preview resolution need to set\n");
		res_param = get_resolution_param(dev, 0, fh->width,fh->height);
		if (!res_param) {
			printk("error,preview resolution param not get\n");
			goto out;
		}
		set_resolution_param(dev, res_param);
#if 1
		//pre_default_exposure();
		OV5645_set_AE_mode(true);
		OV5645_set_AWB_mode(true);
#endif
		prev_res = res_param;
	}

	ov5645_open_mipi_stream();
    
	ret = 0;
out:
	mutex_unlock(&q->vb_lock);
	
	return ret;
}

static int vidioc_g_parm(struct file *file, void *priv,
 				struct v4l2_streamparm *parms)
{
	struct ov5645_fh *fh = priv;
	struct ov5645_device *dev = fh->dev;
	struct v4l2_captureparm *cp = &parms->parm.capture;
	
	dprintk(dev,3,"vidioc_g_parm\n");
	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	
	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	
	cp->timeperframe = ov5645_frmintervals_active;
	printk("g_parm,deno=%d, numerator=%d\n", cp->timeperframe.denominator,
	cp->timeperframe.numerator );
	return 0;
}

static int vidioc_reqbufs(struct file *file, void *priv,
              struct v4l2_requestbuffers *p)
{
	struct ov5645_fh  *fh = priv;

	return (videobuf_reqbufs(&fh->vb_vidq, p));
}

static int vidioc_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct ov5645_fh  *fh = priv;
	int ret = videobuf_querybuf(&fh->vb_vidq, p);
#if MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8
	if(ret == 0){
	    p->reserved  = convert_canvas_index(fh->fmt->fourcc, ov5645_RES0_CANVAS_INDEX+p->index*3);
	}else{
	    p->reserved = 0;
	}
#endif
	return ret;
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct ov5645_fh *fh = priv;

	return (videobuf_qbuf(&fh->vb_vidq, p));
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct ov5645_fh  *fh = priv;

	return (videobuf_dqbuf(&fh->vb_vidq, p,
            	file->f_flags & O_NONBLOCK));
}

#ifdef CONFIG_VIDEO_V4L1_COMPAT
static int vidiocgmbuf(struct file *file, void *priv, struct video_mbuf *mbuf)
{
	struct ov5645_fh  *fh = priv;

	return videobuf_cgmbuf(&fh->vb_vidq, mbuf, 8);
}
#endif

static int vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
        struct ov5645_fh  *fh = priv;
        struct ov5645_device *dev = fh->dev;
        vdin_parm_t para;
        int ret = 0 ;
        if (fh->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
                return -EINVAL;
        if (i != fh->type)
                return -EINVAL;

		memset( &para, 0, sizeof( para ));
		para.port  = TVIN_PORT_MIPI;
		para.fmt = TVIN_SIG_FMT_MAX;//TVIN_SIG_FMT_MAX+1;;TVIN_SIG_FMT_CAMERA_1280x720P_30Hz
		para.scan_mode = TVIN_SCAN_MODE_PROGRESSIVE;	
		para.frame_rate = ov5645_frmintervals_active.denominator;
		para.h_active = fh->dev->cur_resolution_param->active_frmsize.width;
		para.v_active = fh->dev->cur_resolution_param->active_frmsize.height;
		para.hs_bp = 0;
		para.vs_bp = 2;

        if(temp_frame<0 || para.h_active <= 1290){
            temp_frame=para.frame_rate;
            para.skip_count =  2;
        }else {
            temp_frame=para.frame_rate;
            para.skip_count =  3;
        }
        printk("ov5645: h_active = %d; v_active = %d, frame_rate=%d, skip_cnt:%d\n",
                        para.h_active, para.v_active, para.frame_rate, para.skip_count);
        para.cfmt = TVIN_YUV422;
        //para.dfmt = TVIN_NV21;
        para.hsync_phase = 1;
        para.vsync_phase  = 1;    
        para.bt_path = dev->cam_info.bt_path;
		if (CAM_MIPI == dev->cam_info.interface) {
			printk("mipi param init\n");
			para.csi_hw_info.lanes = 2;
			para.csi_hw_info.channel = 1;
			para.csi_hw_info.mode = 1;
			para.csi_hw_info.clock_lane_mode = 1; // 0 clock gate 1: always on
			para.csi_hw_info.active_pixel = para.h_active;
			para.csi_hw_info.active_line = para.v_active;
			para.csi_hw_info.frame_size=0;
			para.csi_hw_info.settle = 24; // 24 need to be changed ?
			para.csi_hw_info.ui_val = 2; //ns
			para.csi_hw_info.urgent = 1;

			para.csi_hw_info.hs_freq = 410; //MHz need to be chenged ?
			para.csi_hw_info.clk_channel = dev->cam_info.clk_channel; //clock channel a or b
		}
        ret =  videobuf_streamon(&fh->vb_vidq);
        if(ret == 0){
                vops->start_tvin_service(0,&para);
                fh->stream_on = 1;
        }
		//ov5645_set_param_wb(dev,ov5645_qctrl[4].default_value);
        return ret;
}

static int vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct ov5645_fh  *fh = priv;

        int ret = 0 ;
	if (fh->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    	return -EINVAL;
	if (i != fh->type)
    	return -EINVAL;
	ret = videobuf_streamoff(&fh->vb_vidq);
	if(ret == 0 ){
		vops->stop_tvin_service(0);
		fh->stream_on        = 0;
	}
	return ret;
}

static int vidioc_enum_framesizes(struct file *file, void *fh,struct v4l2_frmsizeenum *fsize)
{
	int ret = 0,i=0;
	struct ov5645_fmt *fmt = NULL;
	struct v4l2_frmsize_discrete *frmsize = NULL;
	pr_info("%s ....\n", __func__);
	for (i = 0; i < ARRAY_SIZE(formats); i++) {
		if (formats[i].fourcc == fsize->pixel_format){
			fmt = &formats[i];
			break;
		}
	}
	if (fmt == NULL)
		return -EINVAL;
	if ((fmt->fourcc == V4L2_PIX_FMT_NV21)
			||(fmt->fourcc == V4L2_PIX_FMT_NV12)
			||(fmt->fourcc == V4L2_PIX_FMT_YUV420)
			||(fmt->fourcc == V4L2_PIX_FMT_YVU420)){
		printk("ov5645_prev_resolution[fsize->index]"
				"   before fsize->index== %d\n",fsize->index);//potti
		if (fsize->index >= ARRAY_SIZE(prev_resolution_array))
			return -EINVAL;
		frmsize = &prev_resolution_array[fsize->index].frmsize;
		printk("ov5645_prev_resolution[fsize->index]"
				"   after fsize->index== %d\n",fsize->index);
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = frmsize->width;
		fsize->discrete.height = frmsize->height;
	} else if (fmt->fourcc == V4L2_PIX_FMT_RGB24){
		printk("ov5645_pic_resolution[fsize->index]"
				"   before fsize->index== %d\n",fsize->index);
		if (fsize->index >= ARRAY_SIZE(capture_resolution_array))
			return -EINVAL;
		frmsize = &capture_resolution_array[fsize->index].frmsize;
		printk("ov5645_pic_resolution[fsize->index]"
				"   after fsize->index== %d\n",fsize->index);
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = frmsize->width;
		fsize->discrete.height = frmsize->height;
	}
	return ret;
}

static int vidioc_s_std(struct file *file, void *priv, v4l2_std_id i)
{
	return 0;
}

/* only one input in this sample driver */
static int vidioc_enum_input(struct file *file, void *priv,
            	struct v4l2_input *inp)
{
    //if (inp->index >= NUM_INPUTS)
        //return -EINVAL;

	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->std = V4L2_STD_525_60;
	sprintf(inp->name, "Camera %u", inp->index);

	return (0);
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	struct ov5645_fh *fh = priv;
	struct ov5645_device *dev = fh->dev;

	*i = dev->input;

	return (0);
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	struct ov5645_fh *fh = priv;
	struct ov5645_device *dev = fh->dev;

    //if (i >= NUM_INPUTS)
        //return -EINVAL;

	dev->input = i;
    //precalculate_bars(fh);

	return (0);
}

    /* --- controls ---------------------------------------------- */
static int vidioc_queryctrl(struct file *file, void *priv,
                struct v4l2_queryctrl *qc)
{
	int i;
	struct ov5645_fh *fh = priv;
	struct ov5645_device *dev = fh->dev;
	
	if (!dev->cam_info.flash_support 
			&& qc->id == V4L2_CID_BACKLIGHT_COMPENSATION)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ov5645_qctrl); i++)
		if (qc->id && qc->id == ov5645_qctrl[i].id) {
			memcpy(qc, &(ov5645_qctrl[i]),
				sizeof(*qc));
			if (ov5645_qctrl[i].type == V4L2_CTRL_TYPE_MENU)
				return ov5645_qctrl[i].maximum+1;
			else
				return (0);
		}

	return -EINVAL;
}

static int vidioc_querymenu(struct file *file, void *priv,
                struct v4l2_querymenu *a)
{
	int i, j;

	for (i = 0; i < ARRAY_SIZE(ov5645_qmenu_set); i++)
 		if (a->id && a->id == ov5645_qmenu_set[i].id) {
 			for(j = 0; j < ov5645_qmenu_set[i].num; j++)
 				if (a->index == ov5645_qmenu_set[i].ov5645_qmenu[j].index) {
 					memcpy(a, &( ov5645_qmenu_set[i].ov5645_qmenu[j]),
 		    			sizeof(*a));
 					return (0);
 				}
 		}

	return -EINVAL;
}

static int vidioc_g_ctrl(struct file *file, void *priv,
             struct v4l2_control *ctrl)
{
	struct ov5645_fh *fh = priv;
	struct ov5645_device *dev = fh->dev;
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	int i;
	int i2cret = -1;

	for (i = 0; i < ARRAY_SIZE(ov5645_qctrl); i++)
		if (ctrl->id == ov5645_qctrl[i].id) {
			if( (V4L2_CID_FOCUS_AUTO == ctrl->id)
					&& bDoingAutoFocusMode){
				if(i2c_get_byte(client, 0x3023)){
					return -EBUSY;
				}else{
					bDoingAutoFocusMode = false;
					if(i2c_get_byte(client, 0x3028) == 0){
						printk("auto mode failed!\n");
						return -EAGAIN;
					}else {
						i2c_put_byte(client, 0x3022 , 0x6); //pause the auto focus
						i2c_put_byte(client, 0x3023 , 0x1);
						printk("pause auto focus\n");
					}
				}
			}else if( V4L2_CID_AUTO_FOCUS_STATUS == ctrl->id){
				i2cret = i2c_get_byte(client, 0x3029);
				if( 0x00 == i2cret){
					ctrl->value = V4L2_AUTO_FOCUS_STATUS_BUSY;
				}else if( 0x10 == i2cret){
					ctrl->value = V4L2_AUTO_FOCUS_STATUS_REACHED;
				}else if( 0x20 == i2cret){
					ctrl->value = V4L2_AUTO_FOCUS_STATUS_IDLE;
				}else{
					printk("should resart focus\n");
					ctrl->value = V4L2_AUTO_FOCUS_STATUS_FAILED;
				}
		        	
				return 0;
			}
			ctrl->value = dev->qctl_regs[i];
			return 0;
		}

	return -EINVAL;
}

static int vidioc_s_ctrl(struct file *file, void *priv,
            	struct v4l2_control *ctrl)
{
	struct ov5645_fh *fh = priv;
	struct ov5645_device *dev = fh->dev;
	int i;

	for (i = 0; i < ARRAY_SIZE(ov5645_qctrl); i++)
    	if (ctrl->id == ov5645_qctrl[i].id) {
        	if (ctrl->value < ov5645_qctrl[i].minimum ||
			ctrl->value > ov5645_qctrl[i].maximum ||
			ov5645_setting(dev,ctrl->id,ctrl->value)<0) {
			return -ERANGE;
		}
        	dev->qctl_regs[i] = ctrl->value;
        	return 0;
        }
	return -EINVAL;
}

/* ------------------------------------------------------------------
	File operations for the device
   ------------------------------------------------------------------*/

static int ov5645_open(struct file *file)
{
	struct ov5645_device *dev = video_drvdata(file);
	struct ov5645_fh *fh = NULL;
	//struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	resource_size_t mem_start = 0;
	unsigned int mem_size = 0;
	int retval = 0;
	//int reg_val;
	//int i = 0;
#ifdef CONFIG_CMA
    retval = vm_init_buf(24*SZ_1M);
    if(retval <0) {
    	printk("error: no cma memory\n");
        return -1;
    }
#endif
	mutex_lock(&firmware_mutex);
	ov5645_have_opened=1;
	mutex_unlock(&firmware_mutex);
#if MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON6
	switch_mod_gate_by_name("ge2d", 1);
#endif	
	aml_cam_init(&dev->cam_info);
	
	ov5645_init_regs(dev);
	
	msleep(50);
	
	//schedule_work(&(dev->dl_work));

	mutex_lock(&dev->mutex);
	dev->users++;
	if (dev->users > 1) {
		dev->users--;
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	dprintk(dev, 1, "open %s type=%s users=%d\n",
    	video_device_node_name(dev->vdev),
    	v4l2_type_names[V4L2_BUF_TYPE_VIDEO_CAPTURE], dev->users);

        /* init video dma queues */
	INIT_LIST_HEAD(&dev->vidq.active);
	init_waitqueue_head(&dev->vidq.wq);
	spin_lock_init(&dev->slock);
	/* allocate + initialize per filehandle data */
	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (NULL == fh) {
		dev->users--;
		retval = -ENOMEM;
	}
	mutex_unlock(&dev->mutex);

	if (retval)
    	return retval;

	wake_lock(&(dev->wake_lock));
	file->private_data = fh;
	fh->dev      = dev;

	fh->type     = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fh->fmt      = &formats[0];
	fh->width    = 640;
	fh->height   = 480;
	fh->stream_on = 0 ;
	fh->f_flags  = file->f_flags;
	/* Resets frame counters */
	dev->jiffies = jiffies;

	get_vm_buf_info(&mem_start, &mem_size, NULL);
	fh->res.start = mem_start;
	fh->res.end = mem_start+mem_size-1;
	fh->res.magic = MAGIC_RE_MEM;
	fh->res.priv = NULL;		
	videobuf_queue_res_init(&fh->vb_vidq, &ov5645_video_qops,
					NULL, &dev->slock, fh->type, V4L2_FIELD_INTERLACED,
					sizeof(struct ov5645_buffer), (void*)&fh->res, NULL);

	bDoingAutoFocusMode=false;
	ov5645_start_thread(fh);
	pr_info("%s done \n", __func__);
	return 0;
}

static ssize_t
ov5645_read(struct file *file, char __user *data, size_t count, loff_t *ppos)
{
	struct ov5645_fh *fh = file->private_data;

	if (fh->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		return videobuf_read_stream(&fh->vb_vidq, data, count, ppos, 0,
                	file->f_flags & O_NONBLOCK);
	}
	return 0;
}

static unsigned int
ov5645_poll(struct file *file, struct poll_table_struct *wait)
{
	struct ov5645_fh        *fh = file->private_data;
	struct ov5645_device       *dev = fh->dev;
	struct videobuf_queue *q = &fh->vb_vidq;

	dprintk(dev, 1, "%s\n", __func__);

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != fh->type)
    	return POLLERR;

	return videobuf_poll_stream(file, q, wait);
}

static int ov5645_close(struct file *file)
{
	struct ov5645_fh         *fh = file->private_data;
	struct ov5645_device *dev       = fh->dev;
	struct ov5645_dmaqueue *vidq = &dev->vidq;
	struct video_device  *vdev = video_devdata(file);
	pr_info("%s 000\n", __func__);
	mutex_lock(&firmware_mutex);
	ov5645_have_opened=0;
	dev->firmware_ready = 0;
	mutex_unlock(&firmware_mutex);
	
	pr_info("%s 001\n", __func__);
	
	ov5645_stop_thread(vidq);
	videobuf_stop(&fh->vb_vidq);
	if(fh->stream_on){
		vops->stop_tvin_service(0);     
	}
	videobuf_mmap_free(&fh->vb_vidq);

	kfree(fh);
	pr_info("%s 111\n", __func__);

	mutex_lock(&dev->mutex);
	dev->users--;
	mutex_unlock(&dev->mutex);

	dprintk(dev, 1, "close called (dev=%s, users=%d)\n",
    	video_device_node_name(vdev), dev->users);
#if 1	    
	ov5645_qctrl[4].default_value=0;
	ov5645_qctrl[5].default_value=4;
	ov5645_qctrl[6].default_value=0;
	
	ov5645_qctrl[2].default_value=0;
	ov5645_qctrl[10].default_value=100;
	ov5645_qctrl[11].default_value=0;
	temp_frame=-1;
	power_down_ov5645(dev);
#endif
	ov5645_frmintervals_active.numerator = 1;
	ov5645_frmintervals_active.denominator = 25;
	
	aml_cam_uninit(&dev->cam_info);
#if MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON6
	switch_mod_gate_by_name("ge2d", 0);
#endif	
	wake_unlock(&(dev->wake_lock));
#ifdef CONFIG_CMA
    vm_deinit_buf();
#endif
	return 0;
}

static int ov5645_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct ov5645_fh  *fh = file->private_data;
	struct ov5645_device *dev = fh->dev;
	int ret;

	dprintk(dev, 1, "mmap called, vma=0x%08lx\n", (unsigned long)vma);

	ret = videobuf_mmap_mapper(&fh->vb_vidq, vma);

	dprintk(dev, 1, "vma start=0x%08lx, size=%ld, ret=%d\n",
		(unsigned long)vma->vm_start,
		(unsigned long)vma->vm_end-(unsigned long)vma->vm_start,
		ret);

	return ret;
}

static const struct v4l2_file_operations ov5645_fops = {
	.owner	    = THIS_MODULE,
	.open       = ov5645_open,
	.release    = ov5645_close,
	.read       = ov5645_read,
	.poll	    = ov5645_poll,
	.ioctl      = video_ioctl2, /* V4L2 ioctl handler */
	.mmap       = ov5645_mmap,
};

static const struct v4l2_ioctl_ops ov5645_ioctl_ops = {
	.vidioc_querycap      = vidioc_querycap,
	.vidioc_enum_fmt_vid_cap  = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap     = vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap   = vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap     = vidioc_s_fmt_vid_cap,
	.vidioc_reqbufs       = vidioc_reqbufs,
	.vidioc_querybuf      = vidioc_querybuf,
	.vidioc_qbuf          = vidioc_qbuf,
	.vidioc_dqbuf         = vidioc_dqbuf,
	.vidioc_s_std         = vidioc_s_std,
	.vidioc_enum_input    = vidioc_enum_input,
	.vidioc_g_input       = vidioc_g_input,
	.vidioc_s_input       = vidioc_s_input,
	.vidioc_queryctrl     = vidioc_queryctrl,
	.vidioc_querymenu     = vidioc_querymenu,
	.vidioc_g_ctrl        = vidioc_g_ctrl,
	.vidioc_s_ctrl        = vidioc_s_ctrl,
	.vidioc_streamon      = vidioc_streamon,
	.vidioc_streamoff     = vidioc_streamoff,
	.vidioc_enum_framesizes = vidioc_enum_framesizes,
	.vidioc_g_parm = vidioc_g_parm,
	.vidioc_enum_frameintervals = vidioc_enum_frameintervals,
#ifdef CONFIG_VIDEO_V4L1_COMPAT
	.vidiocgmbuf          = vidiocgmbuf,
#endif
};

static struct video_device ov5645_template = {
	.name	        = "ov5645_v4l",
	.fops           = &ov5645_fops,
	.ioctl_ops      = &ov5645_ioctl_ops,
	.release	    = video_device_release,
	
	.tvnorms        = V4L2_STD_525_60,
	.current_norm   = V4L2_STD_NTSC_M,
};

static int ov5645_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_OV5640, 0);
}

static const struct v4l2_subdev_core_ops ov5645_core_ops = {
	.g_chip_ident = ov5645_g_chip_ident,
};

static const struct v4l2_subdev_ops ov5645_ops = {
	.core = &ov5645_core_ops,
};
static ssize_t cam_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i=0;
	pr_info("init regs:\r");
	for(i=0; ;i++) {
		if(ov5645_script[i].addr==0xffff && ov5645_script[i].val == 0xff)
			break;
		pr_info("0x%04x:0x%02x \n", ov5645_script[i].addr,i2c_get_byte(g_client, ov5645_script[i].addr));
	}
	pr_info("preview regs:\r");
	for(i=0;;i++) {
		if(ov5645_preview_VGA_script[i].addr==0xffff && ov5645_preview_VGA_script[i].val==0xff)
			break;
		pr_info("0x%04x:0x%02x \n", ov5645_preview_VGA_script[i].addr,i2c_get_byte(g_client, ov5645_preview_VGA_script[i].addr));
	}
	pr_info("capture regs:\r");
	for(i=0; ;i++) {
		if(ov5645_capture_5M_script[i].addr==0xffff && ov5645_capture_5M_script[i].val==0xff)
			break;
		pr_info("0x%04x:0x%02x \n", ov5645_capture_5M_script[i].addr, i2c_get_byte(g_client, ov5645_capture_5M_script[i].addr));
	}
	return 1;
}

static DEVICE_ATTR(cam_info, 0664, cam_info_show, NULL);
static int ov5645_probe(struct i2c_client *client,
        	const struct i2c_device_id *id)
{
	aml_cam_info_t* plat_dat;
	int err;
	struct ov5645_device *t;
	struct v4l2_subdev *sd;
	vops = get_vdin_v4l2_ops();
	v4l_info(client, "chip found @ 0x%x (%s)\n",
        	client->addr << 1, client->adapter->name);
	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (t == NULL)
    		return -ENOMEM;
	sd = &t->sd;
	v4l2_i2c_subdev_init(sd, client, &ov5645_ops);
	mutex_init(&t->mutex);
	
	g_client = client;

	/* Now create a video4linux device */
	t->vdev = video_device_alloc();
	if (t->vdev == NULL) {
		kfree(t);
		kfree(client);
		return -ENOMEM;
	}
	memcpy(t->vdev, &ov5645_template, sizeof(*t->vdev));
	
	video_set_drvdata(t->vdev, t);
	
	//INIT_WORK(&(t->dl_work), do_download);
	
	wake_lock_init(&(t->wake_lock),WAKE_LOCK_SUSPEND, "ov5645");
	/* Register it */
	plat_dat= (aml_cam_info_t*)client->dev.platform_data;
	if (plat_dat) {
		memcpy(&t->cam_info, plat_dat, sizeof(aml_cam_info_t));
		if (plat_dat->front_back >=0)  
			video_nr = plat_dat->front_back;
	} else {
		printk("camera ov5645: have no platform data\n");
		kfree(t);
		kfree(client);
		return -1;
	}
	
	t->cam_info.version = ov5645_DRIVER_VERSION;
	if (aml_cam_info_reg(&t->cam_info) < 0)
		printk("reg caminfo error\n");
	
	err = video_register_device(t->vdev, VFL_TYPE_GRABBER, video_nr);
	if (err < 0) {
		video_device_release(t->vdev);
		kfree(t);
		return err;
	}
	device_create_file( &t->vdev->dev, &dev_attr_cam_info);
	return 0;
}

static int ov5645_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5645_device *t = to_dev(sd);

	video_unregister_device(t->vdev);
	v4l2_device_unregister_subdev(sd);
	wake_lock_destroy(&(t->wake_lock));
	aml_cam_info_unreg(&t->cam_info);
	kfree(t);
	return 0;
}

static const struct i2c_device_id ov5645_id[] = {
	{ "ov5645", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov5645_id);

static struct i2c_driver ov5645_i2c_driver = {
	.driver = {
		.name = "ov5645",
	},
	.probe = ov5645_probe,
	.remove = ov5645_remove,
	.id_table = ov5645_id,
};

module_i2c_driver(ov5645_i2c_driver);

