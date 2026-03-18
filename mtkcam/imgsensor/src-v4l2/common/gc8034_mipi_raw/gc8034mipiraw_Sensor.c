// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *     GC8034mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 * Setting version:
 * ------------
 *   update full pd setting for GC8034EB_03B
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#define PFX "GC8034_camera_sensor"

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "gc8034mipiraw_Sensor.h"
#include "gc8034_Sensor_setting.h"
#include "gc8034_ana_gain_table.h"

#define MULTI_WRITE 0

// #define SEAMLESS_ 1
// #define SEAMLESS_NO_USE 0
static bool _is_seamless = 0;
#define GC8034_DEBUG_LOG 0


//#define _I2C_BUF_SIZE 4096

//drv add by lipengpeng 20240823 start 
//#define GC8034_MIRROR_FLIP_ENABLE    0
//#if GC8034_MIRROR_FLIP_ENABLE
//#define GC8034_MIRROR  0x57
//#define GC8034_STARTY  0x02
//#define GC8034_STARTX  0x00
//#else
//#define GC8034_MIRROR  0x54
//#define GC8034_STARTY  0x01
//#define GC8034_STARTX  0x01
//#endif
//drv add by lipengpeng 20240823 end 


//static kal_uint16 _i2c_data[_I2C_BUF_SIZE];
static unsigned int _size_to_write;

//drv add by lipengpeng 20250408 start 
static struct agc_params_struct GC8034_AGC_Param[9] = {
	{
		64, /* 1.00x */
		{
			{ 0, 0xfe, {0x00, 0x00} },
			{ 0, 0x20, {0x55, 0x55} },
			{ 0, 0x33, {0x83, 0x83} },
			{ 0, 0xfe, {0x01, 0x01} },
			{ 0, 0xdf, {0x06, 0x06} },
			{ 0, 0xe7, {0x18, 0x18} },
			{ 0, 0xe8, {0x20, 0x20} },
			{ 0, 0xe9, {0x16, 0x16} },
			{ 0, 0xea, {0x17, 0x17} },
			{ 0, 0xeb, {0x50, 0x50} },
			{ 0, 0xec, {0x6c, 0x6c} },
			{ 0, 0xed, {0x9b, 0x9b} },
			{ 0, 0xee, {0xd8, 0xd8} },
			{ 0, 0xfe, {0x00, 0x00} }
		}
	},
	{
		90, /*1.40x*/
		{
			{ 0, 0xfe, {0x00, 0x00} },
			{ 0, 0x20, {0x55, 0x55} },
			{ 0, 0x33, {0x83, 0x83} },
			{ 0, 0xfe, {0x01, 0x01} },
			{ 0, 0xdf, {0x06, 0x06} },
			{ 0, 0xe7, {0x18, 0x18} },
			{ 0, 0xe8, {0x20, 0x20} },
			{ 0, 0xe9, {0x16, 0x16} },
			{ 0, 0xea, {0x17, 0x17} },
			{ 0, 0xeb, {0x50, 0x50} },
			{ 0, 0xec, {0x6c, 0x6c} },
			{ 0, 0xed, {0x9b, 0x9b} },
			{ 0, 0xee, {0xd8, 0xd8} },
			{ 0, 0xfe, {0x00, 0x00} }
		}
	},
	{
		126, /*1.96x*/
		{
			{ 0, 0xfe, {0x00, 0x00} },
			{ 0, 0x20, {0x4e, 0x55} },
			{ 0, 0x33, {0x84, 0x83} },
			{ 0, 0xfe, {0x01, 0x01} },
			{ 0, 0xdf, {0x0c, 0x06} },
			{ 0, 0xe7, {0x2e, 0x18} },
			{ 0, 0xe8, {0x2d, 0x20} },
			{ 0, 0xe9, {0x15, 0x16} },
			{ 0, 0xea, {0x19, 0x17} },
			{ 0, 0xeb, {0x47, 0x50} },
			{ 0, 0xec, {0x70, 0x6c} },
			{ 0, 0xed, {0x9f, 0x9b} },
			{ 0, 0xee, {0xd8, 0xd8} },
			{ 0, 0xfe, {0x00, 0x00} }
		}
	},
	{
		176, /*2.75x*/
		{
			{ 0, 0xfe, {0x00, 0x00} },
			{ 0, 0x20, {0x51, 0x53} },
			{ 0, 0x33, {0x80, 0x82} },
			{ 0, 0xfe, {0x01, 0x01} },
			{ 0, 0xdf, {0x07, 0x03} },
			{ 0, 0xe7, {0x28, 0x28} },
			{ 0, 0xe8, {0x32, 0x3f} },
			{ 0, 0xe9, {0x22, 0x1d} },
			{ 0, 0xea, {0x20, 0x1c} },
			{ 0, 0xeb, {0x49, 0x4d} },
			{ 0, 0xec, {0x70, 0x71} },
			{ 0, 0xed, {0x91, 0x92} },
			{ 0, 0xee, {0xd9, 0xd1} },
			{ 0, 0xfe, {0x00, 0x00} }
		}
	},
	{
		245, /*3,828x */
		{
			{ 0, 0xfe, {0x00, 0x00} },
			{ 0, 0x20, {0x4d, 0x53} },
			{ 0, 0x33, {0x83, 0x82} },
			{ 0, 0xfe, {0x01, 0x01} },
			{ 0, 0xdf, {0x0f, 0x03} },
			{ 0, 0xe7, {0x3b, 0x28} },
			{ 0, 0xe8, {0x3b, 0x3f} },
			{ 0, 0xe9, {0x1c, 0x1d} },
			{ 0, 0xea, {0x1f, 0x1c} },
			{ 0, 0xeb, {0x47, 0x4d} },
			{ 0, 0xec, {0x6f, 0x71} },
			{ 0, 0xed, {0x9b, 0x92} },
			{ 0, 0xee, {0xd3, 0xd1} },
			{ 0, 0xfe, {0x00, 0x00} }
		}
	},
	{
		355, /*5.5468x*/
		{
			{ 0, 0xfe, {0x00, 0x00} },
			{ 0, 0x20, {0x50, 0x54} },
			{ 0, 0x33, {0x83, 0x82} },
			{ 0, 0xfe, {0x01, 0x01} },
			{ 0, 0xdf, {0x08, 0x03} },
			{ 0, 0xe7, {0x35, 0x28} },
			{ 0, 0xe8, {0x46, 0x3f} },
			{ 0, 0xe9, {0x1e, 0x1d} },
			{ 0, 0xea, {0x22, 0x1c} },
			{ 0, 0xeb, {0x4c, 0x4d} },
			{ 0, 0xec, {0x70, 0x71} },
			{ 0, 0xed, {0x9a, 0x92} },
			{ 0, 0xee, {0xd2, 0xd1} },
			{ 0, 0xfe, {0x00, 0x00} }
		}
	},
	{
		474, /*7.406x */
		{
			{ 0, 0xfe, {0x00, 0x00} },
			{ 0, 0x20, {0x52, 0x55} },
			{ 0, 0x33, {0x80, 0x80} },
			{ 0, 0xfe, {0x01, 0x01} },
			{ 0, 0xdf, {0x0c, 0x01} },
			{ 0, 0xe7, {0x35, 0x23} },
			{ 0, 0xe8, {0x3a, 0x38} },
			{ 0, 0xe9, {0x2b, 0x2c} },
			{ 0, 0xea, {0x2d, 0x37} },
			{ 0, 0xeb, {0x4c, 0x4c} },
			{ 0, 0xec, {0x67, 0x67} },
			{ 0, 0xed, {0x8d, 0x8c} },
			{ 0, 0xee, {0xc0, 0xd4} },
			{ 0, 0xfe, {0x00, 0x00} }
		}
	},
	{
		666, /*10.406x */
		{
			{ 0, 0xfe, {0x00, 0x00} },
			{ 0, 0x20, {0x52, 0x55} },
			{ 0, 0x33, {0x80, 0x80} },
			{ 0, 0xfe, {0x01, 0x01} },
			{ 0, 0xdf, {0x0c, 0x01} },
			{ 0, 0xe7, {0x35, 0x23} },
			{ 0, 0xe8, {0x3a, 0x38} },
			{ 0, 0xe9, {0x2b, 0x2c} },
			{ 0, 0xea, {0x2d, 0x37} },
			{ 0, 0xeb, {0x4c, 0x4c} },
			{ 0, 0xec, {0x67, 0x67} },
			{ 0, 0xed, {0x8d, 0x8c} },
			{ 0, 0xee, {0xc0, 0xd4} },
			{ 0, 0xfe, {0x00, 0x00} }
		}
	},
	{
		962, /* 15.03x */
		{
			{ 0, 0xfe, {0x00, 0x00} },
			{ 0, 0x20, {0x52, 0x56} },
			{ 0, 0x33, {0x80, 0x80} },
			{ 0, 0xfe, {0x01, 0x01} },
			{ 0, 0xdf, {0x0c, 0x01} },
			{ 0, 0xe7, {0x35, 0x23} },
			{ 0, 0xe8, {0x3a, 0x36} },
			{ 0, 0xe9, {0x2b, 0x2b} },
			{ 0, 0xea, {0x2d, 0x36} },
			{ 0, 0xeb, {0x4c, 0x4a} },
			{ 0, 0xec, {0x67, 0x67} },
			{ 0, 0xed, {0x8d, 0x8c} },
			{ 0, 0xee, {0xc0, 0xd4} },
			{ 0, 0xfe, {0x00, 0x00} }
		}
	}
};
//drv add by lipengpeng 20250408 end 
static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = GC8034_SENSOR_ID,
//drv add by lipengpeng 20250409 start 
	.checksum_value = 0xb6826aca,//test_Pattern_mode
//drv add by lipengpeng 20250409 end  
//.pclk = 80000000,                /*record different mode's pclk*/
//.linelength = 2136,                /*record different mode's linelength*/
//.framelength = 1250,            /*record different mode's framelength*/
//.startx = 0,                    /*record different mode's startx of grabwindow*/
//.starty = 0,                    /*record different mode's starty of grabwindow*/
//.grabwindow_width = 1632,        /*record different mode's width of grabwindow */
//.grabwindow_height = 1224,        /*record different mode's height of grabwindow */
///* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
//.mipi_data_lp2hs_settle_dc = 85,/*unit , ns*/
///* following for GetDefaultFramerateByScenario() */
//.max_framerate = 300,
		
    .pre = {
		.pclk = 80000000,	//record different mode's pclk
		.linelength = 2136,	//record different mode's linelength
		.framelength = 1250,	//record different mode's framelength
		.startx = 0,	//record different mode's startx of grabwindow
		.starty = 0,	//record different mode's starty of grabwindow
		.grabwindow_width = 1632,	//record different mode's width of grabwindow //1296
		.grabwindow_height = 1224,	//record different mode's height of grabwindow //972
		.mipi_data_lp2hs_settle_dc = 85,	//unit , ns
		.mipi_pixel_rate = 136800000,
		.max_framerate = 300,
    },
    .cap = {
		.pclk = 80000000,	
		.linelength = 2136,	
		.framelength = 1250,	
		.startx = 0,	
		.starty = 0,	
//drv add by lipengpeng 20250408 start 
		//.grabwindow_width = 3264,	
		//.grabwindow_height = 2448,
		.grabwindow_width = 1632,	
		.grabwindow_height = 1224,	
//drv add by lipengpeng 20250408 end
		.mipi_data_lp2hs_settle_dc = 85,	
		.mipi_pixel_rate = 136800000,
		.max_framerate = 300,
    },
    .normal_video = { 
		.pclk = 80000000,	//record different mode's pclk
		.linelength = 2136,	//record different mode's linelength
		.framelength = 1250,	//record different mode's framelength
		.startx = 0,	//record different mode's startx of grabwindow
		.starty = 0,	//record different mode's starty of grabwindow
//drv add by lipengpeng 20250408 start 
		//.grabwindow_width = 3264,	
		//.grabwindow_height = 2448,
		.grabwindow_width = 1632,	
		.grabwindow_height = 1224,	
//drv add by lipengpeng 20250408 end
		.mipi_data_lp2hs_settle_dc = 85,	//unit , ns
		.mipi_pixel_rate = 136800000,
		.max_framerate = 300,
    },
    .hs_video = {
		.pclk = 80000000,	//record different mode's pclk
		.linelength = 2136,	//record different mode's linelength
		.framelength = 1250,	//record different mode's framelength
		.startx = 0,	//record different mode's startx of grabwindow
		.starty = 0,	//record different mode's starty of grabwindow
//drv add by lipengpeng 20250408 start 
		//.grabwindow_width = 3264,	
		//.grabwindow_height = 2448,
		.grabwindow_width = 1632,	
		.grabwindow_height = 1224,	
//drv add by lipengpeng 20250408 end
		.mipi_data_lp2hs_settle_dc = 85,	//unit , ns
		.mipi_pixel_rate = 136800000,
		.max_framerate = 300,
    },
    .slim_video = {
		.pclk = 80000000,	//record different mode's pclk
		.linelength = 2136,	//record different mode's linelength
		.framelength = 2500,	//record different mode's framelength
		.startx = 0,	//record different mode's startx of grabwindow
		.starty = 0,	//record different mode's starty of grabwindow
		.grabwindow_width = 3264,	//record different mode's width of grabwindow //1296
		.grabwindow_height = 2448,	//record different mode's height of grabwindow //972
		.mipi_data_lp2hs_settle_dc = 85,	//unit , ns
		.mipi_pixel_rate = 136800000,
		.max_framerate = 300,
    },

	.margin = 4,					/* sensor framelength & shutter margin */ //0904
	.min_shutter = 2,				/* min shutter */
	.min_gain = BASEGAIN, /*1x gain*/
	.max_gain = 4 * BASEGAIN, /*15.5x * 1024  gain*/ //0904
	.min_gain_iso = 100,
	.exp_step = 1,
	.gain_step = 1, /*minimum step = 4 in 1x~2x gain*/
	.gain_type = 1,/*to be modify,no gain table for sony*/
	.max_frame_length = 0x1fff,     /* max framelength by sensor register's limitation */
	.ae_shut_delay_frame = 0,		//check
	.ae_sensor_gain_delay_frame = 0,//check
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 5,			//support sensor mode num

	.cap_delay_frame = 3,			//enter capture delay frame num
	.pre_delay_frame = 2,			//enter preview delay frame num
	.video_delay_frame = 2,			//enter video delay frame num
	.hs_video_delay_frame = 2,		//enter high speed video  delay frame num
	.slim_video_delay_frame = 2,	//enter slim video delay frame num
	.frame_time_delay_frame = 3,
	
    .isp_driving_current = ISP_DRIVING_8MA, /*mclk driving current*/
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2,
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANUAL, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,//SENSOR_OUTPUT_FORMAT_RAW_B,  SENSOR_OUTPUT_FORMAT_UYVY
//drv add by lipengpeng 20250408 start 
	.mclk = 24,	/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_4_LANE,	/* mipi lane num */
	.i2c_addr_table = { 0x6e, 0xff },
    .i2c_speed = 400,
//drv add by lipengpeng 20250408 end 
	.xtalk_flag = KAL_FALSE,
	// .max_shutter= 1523810,
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{ 3264, 2448, 0, 0, 3264, 2448, 1632, 1224, 0, 0, 1632, 1224, 0, 0, 1632, 1224}, /* Preview */
	{ 3264, 2448, 0, 0, 3264, 2448, 1632, 1224, 0, 0, 1632, 1224, 0, 0, 1632, 1224}, /* capture */
	{ 3264, 2448, 0, 0, 3264, 2448, 1632, 1224, 0, 0, 1632, 1224, 0, 0, 1632, 1224}, /* video */
	{ 3264, 2448, 0, 0, 3264, 2448, 1632, 1224, 0, 0, 1632, 1224, 0, 0, 1632, 1224}, /* hight speed video */
	{ 3264, 2448, 0, 0, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448}  /* slim video */
};


#if MULTI_WRITE
#define I2C_BUFFER_LEN 765	/*trans# max is 255, each 3 bytes*/
#else
#define I2C_BUFFER_LEN 3
#endif

static void set_dummy(struct subdrv_ctx *ctx)
{
//drv add by lipengpeng 20250408 start 
	kal_uint32 /*hb = 0,*/ vb = 16;
	printk("dummyline = %d, dummypixels = %d\n", ctx->dummy_line, ctx->dummy_pixel);

	/*hb = imgsensor.dummy_pixel + GC8034_DEFAULT_DUMMY_PIXEL_NUMS;*/
	//vb = imgsensor.frame_length * 2 - 2484;
	vb = ctx->frame_length * 2 - 2484;
	vb = vb < 16 ? 16 : vb;
	vb = vb > 8191 ? 8191 : vb;
	/*write_cmos_sensor(0x05, (hb >> 8) & 0xFF);
	write_cmos_sensor(0x06, hb & 0xFF);*/

	write_cmos_sensor_8(ctx, 0x07, (vb >> 8) & 0x1F);
	write_cmos_sensor_8(ctx, 0x08, vb & 0xFF);
//drv add by lipengpeng 20250408 end 
}

static void set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate, kal_bool min_framelength_en)
{
//drv add by lipengpeng start 20250408 start	
	kal_uint32 frame_length = ctx->frame_length;

	frame_length = ctx->pclk / framerate * 10 / ctx->line_length;

	ctx->frame_length = (frame_length > ctx->min_frame_length) ?
			frame_length : ctx->min_frame_length;
	ctx->dummy_line = ctx->frame_length -
		ctx->min_frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length) {
		ctx->frame_length = imgsensor_info.max_frame_length;
		ctx->dummy_line = ctx->frame_length - ctx->min_frame_length;
	}
	if (min_framelength_en)
		ctx->min_frame_length = ctx->frame_length;
//drv add by lipengpeng start 20240829 start 	
	set_dummy(ctx);
//drv add by lipengpeng start 20250408 end  
	
}

static void set_max_framerate_video(struct subdrv_ctx *ctx, UINT16 framerate,
					kal_bool min_framelength_en)
{
	set_max_framerate(ctx, framerate, min_framelength_en);
//drv add by lipengpeng start 20240829 start 
	//set_dummy(ctx);
//drv add by lipengpeng start 20240829 end 
}



static kal_uint32 streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
	printk("streaming_control  enable:%d\n", enable);
//drv add by lipengpeng 20240823 start 
	if (enable) {
		//write_cmos_sensor_8(ctx, 0xfe,0x03);
		//write_cmos_sensor_8(ctx, 0x10,0x90);
		//write_cmos_sensor_8(ctx, 0xfe,0x00); 
		ctx->is_streaming = KAL_TRUE;
	} else {
		//write_cmos_sensor_8(ctx, 0xfe,0x03);
		//write_cmos_sensor_8(ctx, 0x10,0x00);
		//write_cmos_sensor_8(ctx, 0xfe,0x00); 
		ctx->is_streaming = KAL_FALSE;
	}
//drv add by lipengpeng 20240823 end
	mdelay(10);
	return ERROR_NONE;
}

static void write_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;
	printk("shutter = %d frame_length %d\n", shutter, ctx->frame_length);
//drv add by lipengpeng 20240823 start 	
	if (shutter > ctx->min_frame_length - imgsensor_info.margin)
		ctx->frame_length = shutter + imgsensor_info.margin;
	else
		ctx->frame_length = ctx->min_frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;

	shutter = (shutter < imgsensor_info.min_shutter) ?
				imgsensor_info.min_shutter : shutter;
	shutter = (shutter >
				(imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
				(imgsensor_info.max_frame_length - imgsensor_info.margin) :
				shutter;
//drv add by lipengpeng 20250408 start 
	//frame_length and shutter should be an even number.
	//shutter = (shutter >> 1) << 1;
	//ctx->frame_length = (ctx->frame_length >> 2) << 2;
		realtime_fps = ctx->pclk / ctx->line_length * 10 /ctx->frame_length;
	if (ctx->autoflicker_en == KAL_TRUE) {

		if (realtime_fps >= 297 && realtime_fps <= 305) {
			realtime_fps = 296;
			set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			realtime_fps = 146;
			set_max_framerate(ctx, 146, 0);
		}else{
			set_max_framerate(ctx, realtime_fps, 0);
		}
	} else{
		   set_max_framerate(ctx, realtime_fps, 0);
	}
	printk("my_realtime_fps = %d\n", realtime_fps);

    shutter = shutter << 1;//ctx->current_ae_effective_frame = 2;

	//write_cmos_sensor_8(ctx,0x04, shutter & 0xff);
	//write_cmos_sensor_8(ctx,0x03, (shutter >> 8) & 0x0f);
	
	write_cmos_sensor_8(ctx,0xfe, 0x00);
	write_cmos_sensor_8(ctx,0x03, (shutter >> 8) & 0x7F);
	write_cmos_sensor_8(ctx,0x04, shutter & 0xFF);
	
	printk("0x03 = 0x%x 0x04 = 0x%x\n", read_cmos_sensor_8(ctx, 0x03),read_cmos_sensor_8(ctx, 0x04));
	printk("shutter =%d, framelength =%d, realtime_fps =%d\n",
			shutter, ctx->frame_length, realtime_fps);

//drv add by lipengpeng 20240823 end 	

	//shutter = shutter << 1;
	/* Update Shutter */
	//write_cmos_sensor(0xfe, 0x00);
	//write_cmos_sensor(0x03, (shutter >> 8) & 0x7F);
	//write_cmos_sensor(0x04, shutter & 0xFF);	
//drv add by lipengpeng 20250408 end 
}
//should not be kal_uint16 -- can't reach long exp
static void set_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{
	ctx->shutter = shutter;
	write_shutter(ctx, shutter);
}

//drv add by lipengpeng 20250408 start  
//#define ANALOG_GAIN_1 64   /* 1.00x */
//#define ANALOG_GAIN_2 91   /* 1.42x */
//#define ANALOG_GAIN_3 126  /* 1.96x */
//#define ANALOG_GAIN_4 178  /* 2.78x */
//#define ANALOG_GAIN_5 242  /* 3.78x */

static kal_uint32 set_gain(struct subdrv_ctx *ctx, kal_uint32 gain)
{
/*	kal_uint16 iReg, temp;
	if( gain/1024 < 2)
		iReg = (64 * gain) / 1024;
	else if( gain/1024 < 3)
		iReg = (64 * gain) / 1024;
	else if( gain/1024 < 4)
		iReg = (64 * gain) / 1024;
	else if( gain/1024 < 5)
		iReg = (64 * gain) / 1024;
	else
		iReg = (64 * gain) / 1024;

	if (iReg < 0x40)
		iReg = 0x40;


	if ((iReg >= ANALOG_GAIN_1) && (iReg < ANALOG_GAIN_2)) {

		write_cmos_sensor_8(ctx,0xb6,  0x00);
		temp = iReg;
		write_cmos_sensor_8(ctx,0xb1, temp >> 6);
		write_cmos_sensor_8(ctx,0xb2, (temp << 2) & 0xfc);
		printk("analogic gain 1x, add pregain = %d\n", temp);
	} else if ((iReg >= ANALOG_GAIN_2) && (iReg < ANALOG_GAIN_3)) {

		write_cmos_sensor_8(ctx,0xb6, 0x01);
		temp = 64 * iReg / ANALOG_GAIN_2;
		write_cmos_sensor_8(ctx,0xb1, temp >> 6);
		write_cmos_sensor_8(ctx,0xb2, (temp << 2) & 0xfc);
		printk("analogic gain 1.42x, add pregain = %d\n", temp);
	} else if ((iReg >= ANALOG_GAIN_3) && (iReg < ANALOG_GAIN_4)) {

		write_cmos_sensor_8(ctx,0xb6, 0x02);
		temp = 64 * iReg / ANALOG_GAIN_3;
		write_cmos_sensor_8(ctx,0xb1, temp >> 6);
		write_cmos_sensor_8(ctx,0xb2, (temp << 2) & 0xfc);
		printk("analogic gain 1.97x, add pregain = %d\n", temp);
	} else if ((iReg >= ANALOG_GAIN_4) && (iReg < ANALOG_GAIN_5)) {

		write_cmos_sensor_8(ctx,0xb6, 0x03);
		temp = 64 * iReg / ANALOG_GAIN_4;
		write_cmos_sensor_8(ctx,0xb1, temp >> 6);
		write_cmos_sensor_8(ctx,0xb2, (temp << 2) & 0xfc);
		printk("analogic gain 2.78x, add pregain = %d\n", temp);
	} else {

		write_cmos_sensor_8(ctx,0xb6, 0x04);
		temp = 64 * iReg / ANALOG_GAIN_5;
		write_cmos_sensor_8(ctx,0xb1, temp >> 6);
		write_cmos_sensor_8(ctx,0xb2, (temp << 2) & 0xfc);
		printk("analogic gain 3.78x, add pregain = %d\n", temp);
	}

	return gain;
	*/
	kal_uint32 real_gain = 0, temp_gain = 0;
	kal_int16 gain_index = 0;
	kal_uint8 i = 0;

		//if( gain/1024 < 2)
		//iReg = (64 * gain) / 1024;
	real_gain = (64*gain)/1024;
	
   	printk("real_gain=%d\n",real_gain);
	
	if (real_gain < SENSOR_BASE_GAIN)
		real_gain = SENSOR_BASE_GAIN;
	else if (real_gain > SENSOR_MAX_GAIN)
		real_gain = SENSOR_MAX_GAIN;
//drv add by lipengpeng 20250408 start 
	for (gain_index = 8; gain_index >= 0; gain_index--)
		if (real_gain >= GC8034_AGC_Param[gain_index].gain_level) {
			write_cmos_sensor_8(ctx,0xb6, gain_index);
			temp_gain = 256 * real_gain / GC8034_AGC_Param[gain_index].gain_level;
			write_cmos_sensor_8(ctx,0xb1, temp_gain >> 8);
			write_cmos_sensor_8(ctx,0xb2, (temp_gain & 0xff));
			for (i = 0; i < AGC_REG_NUM; i++) {
				write_cmos_sensor_8(ctx,GC8034_AGC_Param[gain_index].agc_register[i].addr,
					GC8034_AGC_Param[gain_index].agc_register[i].value[gamma_index]);
			}
			break;
		}
//drv add by lipengpeng 20250408 start 	
	return gain;
	//return real_gain;
}
//drv add by lipengpeng 20250408 end 

static void set_frame_length(struct subdrv_ctx *ctx, kal_uint16 frame_length)
{
//drv add by lipengpeng 20250408 start 
	kal_uint32 vb = 16;
	
	if (frame_length > 1)
		ctx->frame_length = frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	if (ctx->min_frame_length > ctx->frame_length)
		ctx->frame_length = ctx->min_frame_length;

	/* Extend frame length */
   // write_cmos_sensor_8(ctx, 0x05, (((ctx->frame_length - ctx->vblank_convert) * 2) & 0x7F00) >> 8);
   // write_cmos_sensor_8(ctx, 0x06, ((ctx->frame_length - ctx->vblank_convert )* 2) & 0xFF);
   // write_cmos_sensor_8(ctx, 0x01, 0x01);

	vb = ctx->frame_length * 2 - 2484;
	vb = vb < 16 ? 16 : vb;
	vb = vb > 8191 ? 8191 : vb;
	
	write_cmos_sensor_8(ctx, 0x07, (vb >> 8) & 0x1F);
	write_cmos_sensor_8(ctx, 0x08, vb & 0xFF);
	printk("Framelength: set=%d/input=%d/min=%d\n",
		ctx->frame_length, frame_length, ctx->min_frame_length);

//drv add by lipengpeng 20250408 end 
}

/* ITD: Modify Dualcam By Jesse 190924 Start */
static void set_shutter_frame_length(struct subdrv_ctx *ctx, kal_uint16 shutter,
					kal_uint16 target_frame_length)
{

	if (target_frame_length > 1)
		ctx->dummy_line = target_frame_length - ctx->frame_length;
	ctx->frame_length = ctx->frame_length + ctx->dummy_line;
	ctx->min_frame_length = ctx->frame_length;
	set_shutter(ctx, shutter);
}
/* ITD: Modify Dualcam By Jesse 190924 End */

static void ihdr_write_shutter_gain(struct subdrv_ctx *ctx, kal_uint16 le,
				kal_uint16 se, kal_uint16 gain)
{
}

static void night_mode(struct subdrv_ctx *ctx, kal_bool enable)
{
}

static void sensor_init(struct subdrv_ctx *ctx)
{
   printk("gc8034 sensor init start\n");
 
	write_cmos_sensor_8(ctx, 0xfc, 0x01);
	write_cmos_sensor_8(ctx, 0xf2, 0x00);
	write_cmos_sensor_8(ctx, 0xf4, 0x80);
	write_cmos_sensor_8(ctx, 0xf5, 0x19);
	write_cmos_sensor_8(ctx, 0xf6, 0x44);
	write_cmos_sensor_8(ctx, 0xf8, 0x63);
	write_cmos_sensor_8(ctx, 0xfa, 0x45);
	write_cmos_sensor_8(ctx, 0xf9, 0x00);
	write_cmos_sensor_8(ctx, 0xf7, 0x9d);
	write_cmos_sensor_8(ctx, 0xfc, 0x03);
	write_cmos_sensor_8(ctx, 0xfc, 0xea);
	write_cmos_sensor_8(ctx, 0xfe, 0x03);
	write_cmos_sensor_8(ctx, 0x03, 0x9a);
	write_cmos_sensor_8(ctx, 0x18, 0x07);
	write_cmos_sensor_8(ctx, 0x01, 0x07);
	write_cmos_sensor_8(ctx, 0xfc, 0xee);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0x03, 0x08);
	write_cmos_sensor_8(ctx, 0x04, 0xc6);
	write_cmos_sensor_8(ctx, 0x05, 0x02);
	write_cmos_sensor_8(ctx, 0x06, 0x16);
	write_cmos_sensor_8(ctx, 0x07, 0x00);
	write_cmos_sensor_8(ctx, 0x08, 0x10);
	write_cmos_sensor_8(ctx, 0x0a, 0x3a);
	write_cmos_sensor_8(ctx, 0x0b, 0x00);
	write_cmos_sensor_8(ctx, 0x0c, 0x04);
	write_cmos_sensor_8(ctx, 0x0d, 0x09);
	write_cmos_sensor_8(ctx, 0x0e, 0xa0);
	write_cmos_sensor_8(ctx, 0x0f, 0x0c);
	write_cmos_sensor_8(ctx, 0x10, 0xd4);
	write_cmos_sensor_8(ctx, 0x17, MIRROR);
	write_cmos_sensor_8(ctx, 0x18, 0x02);
	write_cmos_sensor_8(ctx, 0x19, 0x17);
	write_cmos_sensor_8(ctx, 0x1e, 0x50);
	write_cmos_sensor_8(ctx, 0x1f, 0x80);
	write_cmos_sensor_8(ctx, 0x21, 0x3a);
	write_cmos_sensor_8(ctx, 0x25, 0x00);
	write_cmos_sensor_8(ctx, 0x28, 0x56);
	write_cmos_sensor_8(ctx, 0x2d, 0x89);
	write_cmos_sensor_8(ctx, 0xca, 0x02);
	write_cmos_sensor_8(ctx, 0xcb, 0x00);
	write_cmos_sensor_8(ctx, 0xcc, 0x39);
	write_cmos_sensor_8(ctx, 0xce, 0x50);
	write_cmos_sensor_8(ctx, 0xcf, 0x93);
	write_cmos_sensor_8(ctx, 0xd0, 0x19);
	write_cmos_sensor_8(ctx, 0xd1, 0xaa);
	write_cmos_sensor_8(ctx, 0xd2, 0xc3);
	write_cmos_sensor_8(ctx, 0xd8, 0x60);
	write_cmos_sensor_8(ctx, 0xd9, 0xff);
	write_cmos_sensor_8(ctx, 0xda, 0x0e);
	write_cmos_sensor_8(ctx, 0xdb, 0xb0);
	write_cmos_sensor_8(ctx, 0xdc, 0x0e);
	write_cmos_sensor_8(ctx, 0xde, 0x08);
	write_cmos_sensor_8(ctx, 0xe4, 0xc6);
	write_cmos_sensor_8(ctx, 0xe5, 0x08);
	write_cmos_sensor_8(ctx, 0xe6, 0x10);
	write_cmos_sensor_8(ctx, 0xed, 0x2a);
	write_cmos_sensor_8(ctx, 0xfe, 0x02);
	write_cmos_sensor_8(ctx, 0x59, 0x02);
	write_cmos_sensor_8(ctx, 0x5a, 0x04);
	write_cmos_sensor_8(ctx, 0x5b, 0x08);
	write_cmos_sensor_8(ctx, 0x5c, 0x20);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0x1a, 0x09);
	write_cmos_sensor_8(ctx, 0x1d, 0x13);
	write_cmos_sensor_8(ctx, 0xfe, 0x10);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0xfe, 0x10);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0x20, 0x55);
	write_cmos_sensor_8(ctx, 0x33, 0x83);
	write_cmos_sensor_8(ctx, 0xfe, 0x01);
	write_cmos_sensor_8(ctx, 0xdf, 0x06);
	write_cmos_sensor_8(ctx, 0xe7, 0x18);
	write_cmos_sensor_8(ctx, 0xe8, 0x20);
	write_cmos_sensor_8(ctx, 0xe9, 0x16);
	write_cmos_sensor_8(ctx, 0xea, 0x17);
	write_cmos_sensor_8(ctx, 0xeb, 0x50);
	write_cmos_sensor_8(ctx, 0xec, 0x6c);
	write_cmos_sensor_8(ctx, 0xed, 0x9b);
	write_cmos_sensor_8(ctx, 0xee, 0xd8);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0x80, 0x10);
	write_cmos_sensor_8(ctx, 0x84, 0x01);
	write_cmos_sensor_8(ctx, 0x88, 0x03);
	write_cmos_sensor_8(ctx, 0x89, 0x03);
	write_cmos_sensor_8(ctx, 0x8d, 0x03);
	write_cmos_sensor_8(ctx, 0x8f, 0x14);
	write_cmos_sensor_8(ctx, 0xad, 0x30);
	write_cmos_sensor_8(ctx, 0x66, 0x2c);
	write_cmos_sensor_8(ctx, 0xbc, 0x49);
	write_cmos_sensor_8(ctx, 0xc2, 0x7f);
	write_cmos_sensor_8(ctx, 0xc3, 0xff);
	write_cmos_sensor_8(ctx, 0x90, 0x01);
	write_cmos_sensor_8(ctx, 0x92, BinStartY);
	write_cmos_sensor_8(ctx, 0x94, BinStartX);
	write_cmos_sensor_8(ctx, 0x95, 0x04);
	write_cmos_sensor_8(ctx, 0x96, 0xc8);
	write_cmos_sensor_8(ctx, 0x97, 0x06);
	write_cmos_sensor_8(ctx, 0x98, 0x60);
	write_cmos_sensor_8(ctx, 0xb0, 0x90);
	write_cmos_sensor_8(ctx, 0xb1, 0x01);
	write_cmos_sensor_8(ctx, 0xb2, 0x00);
	write_cmos_sensor_8(ctx, 0xb6, 0x00);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0x40, 0x22);
	write_cmos_sensor_8(ctx, 0x41, 0x20);
	write_cmos_sensor_8(ctx, 0x42, 0x02);
	write_cmos_sensor_8(ctx, 0x43, 0x08);
	write_cmos_sensor_8(ctx, 0x4e, 0x0f);
	write_cmos_sensor_8(ctx, 0x4f, 0xf0);
	write_cmos_sensor_8(ctx, 0x58, 0x80);
	write_cmos_sensor_8(ctx, 0x59, 0x80);
	write_cmos_sensor_8(ctx, 0x5a, 0x80);
	write_cmos_sensor_8(ctx, 0x5b, 0x80);
	write_cmos_sensor_8(ctx, 0x5c, 0x00);
	write_cmos_sensor_8(ctx, 0x5d, 0x00);
	write_cmos_sensor_8(ctx, 0x5e, 0x00);
	write_cmos_sensor_8(ctx, 0x5f, 0x00);
	write_cmos_sensor_8(ctx, 0x6b, 0x01);
	write_cmos_sensor_8(ctx, 0x6c, 0x00);
	write_cmos_sensor_8(ctx, 0x6d, 0x0c);
	write_cmos_sensor_8(ctx, 0xfe, 0x01);
	write_cmos_sensor_8(ctx, 0xbf, 0x40);
	write_cmos_sensor_8(ctx, 0xfe, 0x01);
	write_cmos_sensor_8(ctx, 0x68, 0x77);
	write_cmos_sensor_8(ctx, 0xfe, 0x01);
	write_cmos_sensor_8(ctx, 0x60, 0x00);
	write_cmos_sensor_8(ctx, 0x61, 0x10);
	write_cmos_sensor_8(ctx, 0x62, 0x28);
	write_cmos_sensor_8(ctx, 0x63, 0x10);
	write_cmos_sensor_8(ctx, 0x64, 0x02);
	write_cmos_sensor_8(ctx, 0xfe, 0x01);
	write_cmos_sensor_8(ctx, 0xa8, 0x60);
	write_cmos_sensor_8(ctx, 0xa2, 0xd1);
	write_cmos_sensor_8(ctx, 0xc8, 0x57);
	write_cmos_sensor_8(ctx, 0xa1, 0xb8);
	write_cmos_sensor_8(ctx, 0xa3, 0x91);
	write_cmos_sensor_8(ctx, 0xc0, 0x50);
	write_cmos_sensor_8(ctx, 0xd0, 0x05);
	write_cmos_sensor_8(ctx, 0xd1, 0xb2);
	write_cmos_sensor_8(ctx, 0xd2, 0x1f);
	write_cmos_sensor_8(ctx, 0xd3, 0x00);
	write_cmos_sensor_8(ctx, 0xd4, 0x00);
	write_cmos_sensor_8(ctx, 0xd5, 0x00);
	write_cmos_sensor_8(ctx, 0xd6, 0x00);
	write_cmos_sensor_8(ctx, 0xd7, 0x00);
	write_cmos_sensor_8(ctx, 0xd8, 0x00);
	write_cmos_sensor_8(ctx, 0xd9, 0x00);
	write_cmos_sensor_8(ctx, 0xa4, 0x10);
	write_cmos_sensor_8(ctx, 0xa5, 0x20);
	write_cmos_sensor_8(ctx, 0xa6, 0x60);
	write_cmos_sensor_8(ctx, 0xa7, 0x80);
	write_cmos_sensor_8(ctx, 0xab, 0x18);
	write_cmos_sensor_8(ctx, 0xc7, 0xc0);
	write_cmos_sensor_8(ctx, 0xfe, 0x01);
	write_cmos_sensor_8(ctx, 0x20, 0x02);
	write_cmos_sensor_8(ctx, 0x21, 0x02);
	write_cmos_sensor_8(ctx, 0x23, 0x42);
	write_cmos_sensor_8(ctx, 0xfe, 0x03);
	write_cmos_sensor_8(ctx, 0x02, 0x03);
	write_cmos_sensor_8(ctx, 0x04, 0x80);
	write_cmos_sensor_8(ctx, 0x11, 0x2b);
	write_cmos_sensor_8(ctx, 0x12, 0xf8);
	write_cmos_sensor_8(ctx, 0x13, 0x07);
	write_cmos_sensor_8(ctx, 0x15, 0x10);
	write_cmos_sensor_8(ctx, 0x16, 0x29);
	write_cmos_sensor_8(ctx, 0x17, 0xff);
	write_cmos_sensor_8(ctx, 0x19, 0xaa);
	write_cmos_sensor_8(ctx, 0x1a, 0x02);
	write_cmos_sensor_8(ctx, 0x21, 0x02);
	write_cmos_sensor_8(ctx, 0x22, 0x03);
	write_cmos_sensor_8(ctx, 0x23, 0x0a);
	write_cmos_sensor_8(ctx, 0x24, 0x00);
	write_cmos_sensor_8(ctx, 0x25, 0x12);
	write_cmos_sensor_8(ctx, 0x26, 0x04);
	write_cmos_sensor_8(ctx, 0x29, 0x04);
	write_cmos_sensor_8(ctx, 0x2a, 0x02);
	write_cmos_sensor_8(ctx, 0x2b, 0x04);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0x3f, 0x00);
	
     printk("gc8034 sensor init end \n");

}

static kal_uint32 return_sensor_id(struct subdrv_ctx *ctx)
{
	//write_cmos_sensor_8(ctx, 0xfe, 0x00);
	return ((read_cmos_sensor_8(ctx, 0xf0) << 8) | read_cmos_sensor_8(ctx, 0xf1));
}

static int get_imgsensor_id(struct subdrv_ctx *ctx, UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
	do {
		*sensor_id = return_sensor_id(ctx);
				printk("gc8034 i2c write id: 0x%x, sensor id: 0x%x\n",ctx->i2c_write_id, *sensor_id);
	if (*sensor_id == imgsensor_info.sensor_id) {
		      printk("gc8034  i2c write id: 0x%x, sensor id: 0x%x\n",ctx->i2c_write_id, *sensor_id);
		return ERROR_NONE;
	}
		retry--;
	} while (retry > 0);
	i++;
	retry = 1;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		printk("%s: 0x%x fail\n", __func__, *sensor_id);
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	return ERROR_NONE;
}

static int open(struct subdrv_ctx *ctx)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 1;
	kal_uint32 sensor_id = 0;

    printk("gc8034 open start \n");
 
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
	do {
		sensor_id = return_sensor_id(ctx);
	if (sensor_id == imgsensor_info.sensor_id) {
		printk("gc8034 i2c write id: 0x%x, sensor id: 0x%x\n",
			ctx->i2c_write_id, sensor_id);
		break;
	}
		retry--;
	} while (retry > 0);
	i++;
	if (sensor_id == imgsensor_info.sensor_id)
		break;
	retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id) {
		printk("Open sensor id: 0x%x fail\n", sensor_id);
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	sensor_init(ctx);
	ctx->autoflicker_en = KAL_FALSE;
	ctx->sensor_mode = IMGSENSOR_MODE_INIT;
	ctx->shutter = 0x3D0;
	ctx->gain = 0x100;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->dummy_pixel = 0;
	ctx->dummy_line = 0;
	ctx->test_pattern = KAL_FALSE;
	ctx->current_fps = imgsensor_info.pre.max_framerate;

	imgsensor_info.xtalk_flag = KAL_FALSE;

    printk("gc8034 open end \n");
	return ERROR_NONE;
}

static int close(struct subdrv_ctx *ctx)
{
	_is_seamless = KAL_FALSE;
	_size_to_write = 0;
	return ERROR_NONE;
}   /*  close  */
//drv add by lipengpeng 20250408 start 
//LOG_INF("E\n");
//spin_lock(&imgsensor_drv_lock);
//imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
//imgsensor.pclk = imgsensor_info.pre.pclk;
///* imgsensor.video_mode = KAL_FALSE; */
//imgsensor.line_length = imgsensor_info.pre.linelength;
//imgsensor.frame_length = imgsensor_info.pre.framelength;
//imgsensor.min_frame_length = imgsensor_info.pre.framelength;
//imgsensor.autoflicker_en = KAL_TRUE;
//spin_unlock(&imgsensor_drv_lock);
//preview_setting();
//return ERROR_NONE;
//drv add by lipengpeng 20250408 end 
static kal_uint32 preview(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("gc8034 %s  start E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_PREVIEW;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	//ctx->vblank_convert = 1252;//drv by lipengpeng
	addr_data_pair_preview_gc8034(ctx);
	printk("gc8034 %s end X\n", __func__);
	return ERROR_NONE;
}

static kal_uint32 capture(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("gc8034 %s  start E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_CAPTURE;
	ctx->pclk = imgsensor_info.cap.pclk;
	ctx->line_length = imgsensor_info.cap.linelength;
	ctx->frame_length = imgsensor_info.cap.framelength;
	ctx->min_frame_length = imgsensor_info.cap.framelength;
	//ctx->vblank_convert = 2504;//drv by lipengpeng
	addr_data_pair_capture_gc8034(ctx);
	printk("gc8034 %s end X\n", __func__);
	return ERROR_NONE;
} /* capture(ctx) */

static kal_uint32 normal_video(struct subdrv_ctx *ctx,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("%s E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_VIDEO;
	ctx->pclk = imgsensor_info.normal_video.pclk;
	ctx->line_length = imgsensor_info.normal_video.linelength;
	ctx->frame_length = imgsensor_info.normal_video.framelength;
	ctx->min_frame_length = imgsensor_info.normal_video.framelength;
	//ctx->vblank_convert = 2504;//drv by lipengpeng
	addr_data_pair_video_gc8034(ctx);
	printk("%s X\n", __func__);
	return ERROR_NONE;
}

static kal_uint32 hs_video(struct subdrv_ctx *ctx,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("%s E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	ctx->pclk = imgsensor_info.hs_video.pclk;
	ctx->line_length = imgsensor_info.hs_video.linelength;
	ctx->frame_length = imgsensor_info.hs_video.framelength;
	ctx->min_frame_length = imgsensor_info.hs_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;
	//ctx->vblank_convert = 776;//drv by lipengpeng
	addr_data_pair_hs_video_gc8034(ctx);
	printk("%s X\n", __func__);
	return ERROR_NONE;
}

static kal_uint32 slim_video(struct subdrv_ctx *ctx,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("%s E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	ctx->pclk = imgsensor_info.slim_video.pclk;
	ctx->line_length = imgsensor_info.slim_video.linelength;
	ctx->frame_length = imgsensor_info.slim_video.framelength;
	ctx->min_frame_length = imgsensor_info.slim_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;
	//ctx->vblank_convert = 768;//528;//drv by lipengpeng
	addr_data_pair_slim_video_gc8034(ctx);
	printk("%s X\n", __func__);
	return ERROR_NONE;
}


static int get_resolution(struct subdrv_ctx *ctx,
		MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	int i = 0;

	for (i = SENSOR_SCENARIO_ID_MIN; i < SENSOR_SCENARIO_ID_MAX; i++) {
		if (i < imgsensor_info.sensor_mode_num) {
			sensor_resolution->SensorWidth[i] = imgsensor_winsize_info[i].w2_tg_size;
			sensor_resolution->SensorHeight[i] = imgsensor_winsize_info[i].h2_tg_size;
		} else {
			sensor_resolution->SensorWidth[i] = 0;
			sensor_resolution->SensorHeight[i] = 0;
		}
	}

	return ERROR_NONE;
}   /*  get_resolution  */

static int get_info(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
		      MSDK_SENSOR_INFO_STRUCT *sensor_info,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
//drv add by lipengpeng 20240902 start 
		int i = 0;
	if (scenario_id >= imgsensor_info.sensor_mode_num)
		scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
//drv add by lipengpeng 20240902 end 	
	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = KAL_FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	//sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;
//drv add by lipengpeng 20240902 start 
		for (i = 0; i < imgsensor_info.sensor_mode_num; i++) {
			sensor_info->gain_ratio[i] = 1000;
			sensor_info->OB_pedestals[i] = ctx->s_ctx.ob_pedestal;
			sensor_info->saturation_level[i] = 1023;
		}
//drv add by lipengpeng 20240902 end 		
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_PREVIEW] =
		imgsensor_info.pre_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_CAPTURE] =
		imgsensor_info.cap_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_VIDEO] =
		imgsensor_info.video_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO] =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_SLIM_VIDEO] =
		imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	/* The frame of setting sensor gain */
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = 0;


	//sensor_info->HDR_Support = 0; /*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR*/
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;   // 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;

	return ERROR_NONE;
}   /*  get_info  */


static int control(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->current_scenario_id = scenario_id;

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		preview(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		capture(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		normal_video(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		hs_video(ctx, image_window, sensor_config_data);
	break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		slim_video(ctx, image_window, sensor_config_data);
	break;

/* ITD: Modify Dualcam By Jesse 190924 End */
	default:
		printk("Error ScenarioId setting");
		preview(ctx, image_window, sensor_config_data);
	return ERROR_INVALID_SCENARIO_ID;
	}

	return ERROR_NONE;
}   /* control(ctx) */

static kal_uint32 set_video_mode(struct subdrv_ctx *ctx, UINT16 framerate)
{
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;

	if ((framerate == 300) && (ctx->autoflicker_en == KAL_TRUE))
		ctx->current_fps = 296;
	else if ((framerate == 150) && (ctx->autoflicker_en == KAL_TRUE))
		ctx->current_fps = 146;
	else
		ctx->current_fps = framerate;

	set_max_framerate_video(ctx, ctx->current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(struct subdrv_ctx *ctx, kal_bool enable,
			UINT16 framerate)
{
	printk("enable = %d, framerate = %d\n",
		enable, framerate);

	if (enable) //enable auto flicker
		ctx->autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		ctx->autoflicker_en = KAL_FALSE;

	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(struct subdrv_ctx *ctx,
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frameHeight;

	printk("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	if (framerate == 0)
		return ERROR_NONE;

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
	    frameHeight = imgsensor_info.pre.pclk / framerate * 10 /
			imgsensor_info.pre.linelength;
		ctx->dummy_line =
			(frameHeight > imgsensor_info.pre.framelength) ?
			(frameHeight - imgsensor_info.pre.framelength):0;
	    ctx->frame_length = imgsensor_info.pre.framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
		printk("scenario_id = %d, vblank_convert = %d\n", scenario_id, ctx->vblank_convert);
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
	    frameHeight = imgsensor_info.normal_video.pclk / framerate * 10 /
				imgsensor_info.normal_video.linelength;
		ctx->dummy_line = (frameHeight >
			imgsensor_info.normal_video.framelength) ?
		(frameHeight - imgsensor_info.normal_video.framelength):0;
	    ctx->frame_length = imgsensor_info.normal_video.framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
		printk("scenario_id = %d\n", scenario_id);
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
	    frameHeight = imgsensor_info.cap.pclk / framerate * 10 /
			imgsensor_info.cap.linelength;

		ctx->dummy_line =
			(frameHeight > imgsensor_info.cap.framelength) ?
			(frameHeight - imgsensor_info.cap.framelength):0;
	    ctx->frame_length = imgsensor_info.cap.framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
		printk("scenario_id = %d\n", scenario_id);
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
	    frameHeight = imgsensor_info.hs_video.pclk / framerate * 10 /
			imgsensor_info.hs_video.linelength;
		ctx->dummy_line =
			(frameHeight > imgsensor_info.hs_video.framelength) ?
			(frameHeight - imgsensor_info.hs_video.framelength):0;
		ctx->frame_length = imgsensor_info.hs_video.framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
		printk("scenario_id = %d\n", scenario_id);
			set_dummy(ctx);
	break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
	    frameHeight = imgsensor_info.slim_video.pclk / framerate * 10 /
			imgsensor_info.slim_video.linelength;
		ctx->dummy_line = (frameHeight >
			imgsensor_info.slim_video.framelength) ?
			(frameHeight - imgsensor_info.slim_video.framelength):0;
	    ctx->frame_length = imgsensor_info.slim_video.framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
		printk("scenario_id = %d\n", scenario_id);
			set_dummy(ctx);
	break;
	default:  //coding with  preview scenario by default
	    frameHeight = imgsensor_info.pre.pclk / framerate * 10 /
			imgsensor_info.pre.linelength;
		ctx->dummy_line = (frameHeight >
			imgsensor_info.pre.framelength) ?
			(frameHeight - imgsensor_info.pre.framelength):0;
	    ctx->frame_length = imgsensor_info.pre.framelength +
			ctx->dummy_line;
	    ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			printk("scenario_id = %d\n", scenario_id);
			set_dummy(ctx);
	break;
	}

	printk("scenario_id = %d, framerate = %d done\n", scenario_id, framerate);

	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(struct subdrv_ctx *ctx,
			enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MUINT32 *framerate)
{
#if GC8034_DEBUG_LOG
	printk("[3058]scenario_id = %d\n", scenario_id);
#endif

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
	    *framerate = imgsensor_info.pre.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
	    *framerate = imgsensor_info.normal_video.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
	    *framerate = imgsensor_info.cap.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
	break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
	break;

	default:
	break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(struct subdrv_ctx *ctx, kal_bool enable)
{
	printk("Test_Pattern modes: %d\n", enable);
//drv add by lipengpeng 20250409 start 
	printk("Test_Pattern modes: %d -> %d\n", ctx->test_pattern, enable);
   if (enable) {
		write_cmos_sensor_8(ctx, 0xfc, 0x00);
		write_cmos_sensor_8(ctx, 0xf7, 0x9d);
		write_cmos_sensor_8(ctx, 0xfc, 0x00);
		write_cmos_sensor_8(ctx, 0xfc, 0xfe);
		write_cmos_sensor_8(ctx, 0xfe, 0x03);
		write_cmos_sensor_8(ctx, 0x21, 0x02);
		write_cmos_sensor_8(ctx, 0x22, 0x03);
		write_cmos_sensor_8(ctx, 0x23, 0x0a);
		write_cmos_sensor_8(ctx, 0x26, 0x04);
		write_cmos_sensor_8(ctx, 0x29, 0x04);
		write_cmos_sensor_8(ctx, 0x2a, 0x02);
		write_cmos_sensor_8(ctx, 0x2b, 0x04);
		write_cmos_sensor_8(ctx, 0xfe, 0x00);
		write_cmos_sensor_8(ctx, 0x8c, 0x01);
	} else {
		write_cmos_sensor_8(ctx, 0xfc, 0x00);
		write_cmos_sensor_8(ctx, 0xf7, 0x95);
		write_cmos_sensor_8(ctx, 0xfc, 0x00);
		write_cmos_sensor_8(ctx, 0xfc, 0xee);
		write_cmos_sensor_8(ctx, 0xfe, 0x03);
		write_cmos_sensor_8(ctx, 0x21, 0x05);
		write_cmos_sensor_8(ctx, 0x22, 0x06);
		write_cmos_sensor_8(ctx, 0x23, 0x2b);
		write_cmos_sensor_8(ctx, 0x26, 0x07);
		write_cmos_sensor_8(ctx, 0x29, 0x07);
		write_cmos_sensor_8(ctx, 0x2a, 0x12);
		write_cmos_sensor_8(ctx, 0x2b, 0x07);
		write_cmos_sensor_8(ctx, 0xfe, 0x00);
		write_cmos_sensor_8(ctx, 0x8c, 0x00);
	}
//drv add by lipengpeng 20250409 end 
	ctx->test_pattern = enable;
	return ERROR_NONE;
}

static int feature_control(struct subdrv_ctx *ctx, MSDK_SENSOR_FEATURE_ENUM feature_id,
			UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	//INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	// UINT32 *pAeCtrls = NULL;
	// UINT32 *pScenarios = NULL;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;


	//printk("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_OUTPUT_FORMAT_BY_SCENARIO:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			*(feature_data + 1)
			= (enum ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM)
				imgsensor_info.sensor_output_dataformat;
			break;
		}
	break;
	case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
	if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
		*(feature_data + 0) =
			sizeof(ov08d_ana_gain_table);
	} else {
		memcpy((void *)(uintptr_t) (*(feature_data + 1)),
		(void *)ov08d_ana_gain_table,
		sizeof(ov08d_ana_gain_table));
	}
		break;

	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MAX_EXP_LINE:
		*(feature_data + 2) =
			imgsensor_info.max_frame_length - imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			// *(feature_data + 2) = 2;
			// break;

		default:
			*(feature_data + 2) = 1;
			break;
		}
		*(feature_data + 2) = imgsensor_info.exp_step;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;

		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;

		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD:
	    *feature_return_para_16++ = ctx->line_length;
	    *feature_return_para_16 = ctx->frame_length;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
	    *feature_return_para_32 = ctx->pclk;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_ESHUTTER:
	    set_shutter(ctx, *feature_data);
	break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
	    night_mode(ctx, (BOOL) * feature_data);
	break;
	case SENSOR_FEATURE_SET_GAIN:
	    set_gain(ctx, (UINT32) * feature_data);
	break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
	break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
	break;
	case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor_8(ctx, sensor_reg_data->RegAddr,
							sensor_reg_data->RegData);
	break;
	case SENSOR_FEATURE_GET_REGISTER:
	    sensor_reg_data->RegData =
			read_cmos_sensor_8(ctx, sensor_reg_data->RegAddr);
	break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
	    *feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
	    set_video_mode(ctx, *feature_data);
	break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
	    get_imgsensor_id(ctx, feature_return_para_32);
	break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
	    set_auto_flicker_mode(ctx, (BOOL)*feature_data_16,
			*(feature_data_16+1));
	break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(ctx,
			(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
			*(feature_data+1));
	break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
	    get_default_framerate_by_scenario(ctx,
			(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data+1)));
	break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode(ctx, (BOOL)*feature_data);
	break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
	    *feature_return_para_32 = imgsensor_info.checksum_value;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_FRAMERATE:
	    ctx->current_fps = *feature_data_32;
		printk("current fps :%d\n", ctx->current_fps);
	break;
	case SENSOR_FEATURE_GET_CROP_INFO:
	    //printk("GET_CROP_INFO scenarioId:%d\n",
		//	*feature_data_32);

	    wininfo = (struct  SENSOR_WINSIZE_INFO_STRUCT *)
			(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[3],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[4],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;

		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[0],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		}
	break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
	    printk("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
	    ihdr_write_shutter_gain(ctx, (UINT16)*feature_data,
			(UINT16)*(feature_data+1),
				(UINT16)*(feature_data+2));
	break;
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
		memcpy(feature_return_para_32,
		&ctx->ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
		break;
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
		*feature_return_para_32 = ctx->current_ae_effective_frame;
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
			switch (*feature_data) {
			case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.cap.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.normal_video.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.hs_video.mipi_pixel_rate;
				break;
			case SENSOR_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.slim_video.mipi_pixel_rate;
				break;

			case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.pre.mipi_pixel_rate;
				break;
			}
	break;
/* ITD: Modify Dualcam By Jesse 190924 Start */
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		//printk("SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME\n");
		set_shutter_frame_length(ctx, (UINT16)*feature_data, (UINT16)*(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;
/* ITD: Modify Dualcam By Jesse 190924 End */
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		streaming_control(ctx, KAL_FALSE);
		break;

	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		if (*feature_data != 0)
			set_shutter(ctx, *feature_data);
		streaming_control(ctx, KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {

		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			*feature_return_para_32 = 2; /*BINNING_SUMMED*/
#if GC8034_DEBUG_LOG
			printk("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
#endif
			break;
		default:
			*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
			break;
		}
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_PRELOAD_EEPROM_DATA:
		/*get eeprom preloader data*/
		*feature_return_para_32 = ctx->is_read_preload_eeprom;
		*feature_para_len = 4;
		//if (ctx->is_read_preload_eeprom != 1)
			//read_sensor_Cali(ctx);
		break;
	case SENSOR_FEATURE_SET_FRAMELENGTH:
		set_frame_length(ctx, (UINT16) (*feature_data));
		break;
	default:
	break;
	}

	return ERROR_NONE;
}   /*  feature_control(ctx)  */

#ifdef IMGSENSOR_VC_ROUTING
static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0cc0,///3264
			.vsize = 0x0990,//2448
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0cc0,///3264
			.vsize = 0x0990,//2448
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0cc0,///3264
			.vsize = 0x0990,//2448
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0cc0,///3264
			.vsize = 0x0990,//2448
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0cc0,///3264
			.vsize = 0x0990,//2448
		},
	},
};

static int get_frame_desc(struct subdrv_ctx *ctx,
		int scenario_id, struct mtk_mbus_frame_desc *fd)
{
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_prev);
		memcpy(fd->entry, frame_desc_prev, sizeof(frame_desc_prev));
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cap);
		memcpy(fd->entry, frame_desc_cap, sizeof(frame_desc_cap));
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_vid);
		memcpy(fd->entry, frame_desc_vid, sizeof(frame_desc_vid));
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_hs_vid);
		memcpy(fd->entry, frame_desc_hs_vid, sizeof(frame_desc_hs_vid));
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_slim_vid);
		memcpy(fd->entry, frame_desc_slim_vid, sizeof(frame_desc_slim_vid));
		break;

	default:
		return -1;
	}

	return 0;
}
#endif

static const struct subdrv_ctx defctx = {

	.ana_gain_def = 4 * BASEGAIN,
	.ana_gain_max = 4 * BASEGAIN, //0904
	.ana_gain_min = BASEGAIN,
	.ana_gain_step = 1,
	.exposure_def = 0x3D0,
	.exposure_max = 0x7FFFEA - 20,
	.exposure_min = 8,
	.exposure_step = 1,
	.frame_time_delay_frame = 3,
	.margin = 16, //0904
	.max_frame_length = 0x7FFFEA,
	.is_streaming = KAL_FALSE,
	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,
	.gain = 4 * BASEGAIN,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.fast_mode_on = 1, // pri add for sat switch ae flicker  by lipengpeng 20240902 
	.current_scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW,
	//.ihdr_en = 0,
	.i2c_write_id = 0x42,
	.ae_ctrl_gph_en = 0,
	//.vblank_convert=2504,  //drv by lipengpeng
};

static int init_ctx(struct subdrv_ctx *ctx,
		struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(ctx, &defctx, sizeof(*ctx));
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}


static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = get_info,
	.get_resolution = get_resolution,
	.control = control,
	.feature_control = feature_control,
	.close = close,
#ifdef IMGSENSOR_VC_ROUTING
	.get_frame_desc = get_frame_desc,
#endif
	//.get_csi_param = get_csi_param,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
//drv add by lipengpeng 20250408 start 			
	{HW_ID_RST, 0, 5},
	{HW_ID_PDN, 0, 2},
	{HW_ID_DOVDD, 1800000, 0},
	{HW_ID_AVDD, 2800000, 0},
	{HW_ID_DVDD, 1200000, 5},
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 1, 1},
	{HW_ID_PDN, 1, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 0},
//drv add by lipengpeng 20250408 end 
};

const struct subdrv_entry gc8034_mipi_raw_entry = {
	.name = "gc8034_mipi_raw",
	.id = GC8034_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

