// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/********************************************************************
 *
 * Filename:
 * ---------
 *	 s5khm6sxmipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *-------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *===================================================================
 *******************************************************************/
#include "s5khm6sxmipiraw_Sensor.h"
#define S5KHM6SX_LOG_INF(format, args...) pr_info(LOG_TAG "[%s] " format, __func__, ##args)
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int s5khm6sx_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5khm6sx_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int s5khm6sx_sensor_init(struct subdrv_ctx *ctx);
//static int s5khm6sx_set_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
//static int s5khm6sx_set_streaming_suspned(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int open(struct subdrv_ctx *ctx);

//drv add by lipengpeng 20240603 start 
//#include "../../../../../../../../kernel/kernel_device_modules-6.1/drivers/misc/mediatek/prize/gms_adc/adc_detect_gms.h"
//#include "../../../../../../kernel/kernel_device_modules-6.1/drivers/misc/mediatek/prize/gms_adc/adc_detect_gms.h"
//drv add by lipengpeng 20240603 end 

//drv add by lipengpeng 20240603 start 
//#include "../../../../../../../../kernel/kernel_device_modules-6.1/drivers/misc/mediatek/prize/gms_adc/adc_detect_gms.h"
#if IS_ENABLED(CONFIG_DRV_ADC_GMS_DETECT)
#include "../../../../../../kernel/kernel_device_modules-6.1/drivers/misc/mediatek/pri/gms_adc/adc_detect_gms.h"
#endif
//drv add by lipengpeng 20240603 end 

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, s5khm6sx_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, s5khm6sx_set_test_pattern_data},
	//{SENSOR_FEATURE_SET_STREAMING_RESUME, s5khm6sx_set_streaming_resume},
	//{SENSOR_FEATURE_SET_STREAMING_SUSPEND, s5khm6sx_set_streaming_suspned},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x010B00FF,
		.addr_header_id = 0x00000001,
		.i2c_write_id = 0xA2,

		.xtalk_support = false,
		.xtalk_size = 2048,
		.addr_xtalk = 0x150F,
	},
};

//drv add by lipengpeng 20240531 start for PDAF
/*static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
	},
	.iMirrorFlip = 0,
//drv add by lipengpeng 20240710 start 
	.i4FullRawW = 4000,  
//drv add by lipengpeng 20240710 end 
	.i4FullRawH = 3000,
	.i4ModeIndex = 3,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
        .i4PDOrder = {1},  // R = 1, L = 0
	},
}; */

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX = 4,
    .i4PitchY = 4,
    .i4PairNum = 2,
    .i4SubBlkW = 2,
    .i4SubBlkH = 4,
    .i4BlockNumX = 1000,
    .i4BlockNumY = 750,
    .iMirrorFlip = 0,  
    .i4PosL = {{1, 0}, {3, 0} },
    .i4PosR = {{0, 0}, {2, 0} },
//drv add by lipengpeng 20240809 start 
//drv add by lipengpeng 20240710 start 
	.i4FullRawW = 4000,  //can be omitted
	.i4FullRawH = 3000,  //can be omitted
	.i4VCPackNum = 1, //2--->1
	 .sPDMapInfo[0] = {
		.i4PDPattern = 3,  //non-interleaved
		.i4PDRepetition = 2,    //LLLLLLLLLLL RRRRRRRRRRRR LLLLLLLLLLLLLLLLL RRRRRRRRRR
		.i4PDOrder = {1,0}, // R = 1, L = 0    // LLLLLLLLLLLLLLLLL
		                                       // RRRRRRRRRRRRRRRRR
	},
//drv add by lipengpeng 20240809 end 

	
};
//drv add by lipengpeng 20240531 end for PDAF  

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0FA0, //4000
			.vsize = 0x0BB8,//3000
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	//drv add by lipengpeng 20240718 start 	
	{
		.bus.csi2 = { //	 0x01, 0x2B, 0x09C4, 0x05DC, 0x03, 0x00, 0x0000, 0x0000},
			.channel = 1,
			.data_type = 0x2b,  //If it's 0x30 here, then hsize should be filled in as 2000 (pixel).
			.hsize = 0x07D0,   //0x09C4:2500(2000*10/8)  0x09C4   2496--->2000
			.vsize = 0x05DC,   //0x05DC:1492(750+750)
			.user_data_desc = VC_PDAF_STATS,
		},
	},
//drv add by lipengpeng 20240718 end 

//drv add by lipengpeng 20240531 start 
	/*{
		.bus.csi2 = {  //{VC_PDAF_STATS_PIX_1, 0x01, 0x30, 0x13ec, 0x300,0x13ec},
			.channel = 1,
			.data_type = 0x30,
//drv add by lipengpeng 20240710 start 
			.hsize = 0x0FF0,   //0x13ec   0x30  pixel
//drv add by lipengpeng 20240710 end 
			.vsize = 0x300,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
//drv add by lipengpeng 20240710 start 
			.user_data_desc = VC_PDAF_STATS_PIX_1,//VC_PDAF_STATS_NE_PIX_1,
//drv add by lipengpeng 20240710 end 
			//.is_active_line = TRUE,
		},
	},*/
//drv add by lipengpeng 20240531 end 	
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0FA0, //4000
			.vsize = 0x0BB8,//3000

			.user_data_desc = VC_STAGGER_NE,
		},
	},
	//drv add by lipengpeng 20240718 start 	
	{
		.bus.csi2 = { //	 0x01, 0x2B, 0x09C4, 0x05DC, 0x03, 0x00, 0x0000, 0x0000},
			.channel = 1,
			.data_type = 0x2b,  //If it's 0x30 here, then hsize should be filled in as 2000 (pixel).
			.hsize = 0x07D0,   //0x09C4:2500(2000*10/8)  0x09C4   2496--->2000
			.vsize = 0x05DC,   //0x05DC:1492(750+750)
			.user_data_desc = VC_PDAF_STATS,
		},
	},
//drv add by lipengpeng 20240718 end 
//drv add by lipengpeng 20240531 start 
	/*{
		.bus.csi2 = {//{VC_PDAF_STATS_PIX_1, 0x01, 0x30, 0x13ec, 0x300,0x13ec},
			.channel = 1,
			.data_type = 0x30,
//drv add by lipengpeng 20240710 start 
			.hsize = 0x0FF0,   //0x13ec   0x30  pixel
//drv add by lipengpeng 20240710 end 
			.vsize = 0x300,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
//drv add by lipengpeng 20240710 start 
			.user_data_desc = VC_PDAF_STATS_PIX_1,//VC_PDAF_STATS_NE_PIX_1,
//drv add by lipengpeng 20240710 end 
			//.is_active_line = TRUE,
		},
	},*/
//drv add by lipengpeng 20240531 end 
	
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0FA0, //4000
			.vsize = 0x0BB8,//3000

			.user_data_desc = VC_STAGGER_NE,
		},
	},
	//drv add by lipengpeng 20240718 start 	
	{
		.bus.csi2 = { //	 0x01, 0x2B, 0x09C4, 0x05DC, 0x03, 0x00, 0x0000, 0x0000},
			.channel = 1,
			.data_type = 0x2b,  //If it's 0x30 here, then hsize should be filled in as 2000 (pixel).
			.hsize = 0x07D0,   //0x09C4:2500(2000*10/8)  0x09C4   2496--->2000
			.vsize = 0x05DC,   //0x05DC:1492(750+750)
			.user_data_desc = VC_PDAF_STATS,
		},
	},
//drv add by lipengpeng 20240718 end 
//drv add by lipengpeng 20240531 start 
	/*{
		.bus.csi2 = { //{VC_PDAF_STATS_PIX_1, 0x01, 0x30, 0x13ec, 0x300,0x13ec},
			.channel = 1,
			.data_type = 0x30,
//drv add by lipengpeng 20240710 start 
			.hsize = 0x0FF0,   //0x13ec   0x30  pixel
//drv add by lipengpeng 20240710 end 
			.vsize = 0x300,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
//drv add by lipengpeng 20240710 start 
			.user_data_desc = VC_PDAF_STATS_PIX_1,//VC_PDAF_STATS_NE_PIX_1,
//drv add by lipengpeng 20240710 end 
			//.is_active_line = TRUE,
		},
	}, */
//drv add by lipengpeng 20240531 end 	
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
//drv add by lipengpeng 20240606 start 
		/*.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0780,  //1920
			.vsize = 0x0438, //1080

			.user_data_desc = VC_STAGGER_NE,
		},*/
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0780, //1920
			.vsize = 0x0438, //1080

			.user_data_desc = VC_STAGGER_NE,
		},
//drv add by lipengpeng 20240606 end
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
//drv add by lipengpeng 20240606 start 
		/*.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0780,  //1920
			.vsize = 0x0438, //1080

			.user_data_desc = VC_STAGGER_NE,
		},*/
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0780, //1920
			.vsize = 0x0438, //1080

			.user_data_desc = VC_STAGGER_NE,
		},
//drv add by lipengpeng 20240606 end
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x2EE0, //12000
			.vsize = 0x2328,  //9000
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};


//1000 base for dcg gain ratio
//static u32 s5khm6sx_dcg_ratio_table_cus5[] = {8000};

//static u32 s5khm6sx_dcg_ratio_table_cus6[] = {8000};

static struct mtk_sensor_saturation_info imgsensor_saturation_info = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};


		//.pclk = 1640000000,
		//.linelength = 11232,
		//.framelength = 4864,
		//.startx = 0,
		//.starty = 0,
		//.grabwindow_width = 4000,
		//.grabwindow_height = 3000,
		//.mipi_data_lp2hs_settle_dc = 85,
		//.max_framerate = 300,
		//.mipi_pixel_rate = 1432300000,
		
static struct subdrv_mode_struct mode_struct[] = {
	{//preview
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = addr_data_pair_preview,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_preview),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1640000000,
		.linelength = 11232,
		.framelength = 4864,
		.max_framerate = 300,
		.mipi_pixel_rate = 1432300000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { //{12000, 9000, 0,   0,    12000, 9000, 4000,  3000, 0, 0, 4000,  3000, 0, 0, 4000,  3000}, /* Preview */
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 12000,
			.h0_size = 9000,

			.scale_w = 4000,
			.scale_h = 3000,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 3000,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 3000,
		},
//prize add by lipengpeng 20240531 start for PDAF
		//.pdaf_cap = FALSE,
		//.imgsensor_pd_info = PARAM_UNDEFINED,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
//prize add by lipengpeng 20240531 end for PDAF
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 98,
		},

	},
	{//capture
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = addr_data_pair_capture,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_capture),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1640000000,
		.linelength = 11232,
		.framelength = 4864,
		.max_framerate = 300,
		.mipi_pixel_rate = 1432300000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { //{12000, 9000, 0,   0,    12000, 9000, 4000,  3000, 0, 0, 4000,  3000, 0, 0, 4000,  3000}, /* Preview */
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 12000,
			.h0_size = 9000,

			.scale_w = 4000,
			.scale_h = 3000,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 3000,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 3000,
		},

//prize add by lipengpeng 20240531 start for PDAF
		//.pdaf_cap = FALSE,
		//.imgsensor_pd_info = PARAM_UNDEFINED,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
//prize add by lipengpeng 20240531 end for PDAF
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 98,
		},

	},
	{//normal video
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = addr_data_pair_normal_video,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_normal_video),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1640000000,
		.linelength = 11232,
		.framelength = 4864,
		.max_framerate = 300,
		.mipi_pixel_rate = 1432300000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { //{12000, 9000, 0,   0,    12000, 9000, 4000,  3000, 0, 0, 4000,  3000, 0, 0, 4000,  3000}, /* Preview */
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 12000,
			.h0_size = 9000,

			.scale_w = 4000,
			.scale_h = 3000,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 3000,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 3000,
		},

//prize add by lipengpeng 20240531 start for PDAF
		//.pdaf_cap = FALSE,
		//.imgsensor_pd_info = PARAM_UNDEFINED,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
//prize add by lipengpeng 20240531 end for PDAF
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 98,
		},

	},
	
		//.pclk = 1640000000,
		//.linelength = 8752,
		//.framelength = 1561,
		//.startx = 0,
		//.starty = 0,
		//.grabwindow_width = 1920,
		//.grabwindow_height = 1080,
		//.mipi_data_lp2hs_settle_dc = 85,
		//.max_framerate = 1200,
		//.mipi_pixel_rate =1432300000,
	{//hs video
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = addr_data_pair_hs_video,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_hs_video),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
//drv add by lipengpeng 20240606 start 
		.pclk = 1640000000,
		.linelength = 8752,
		.framelength = 1561,
		.max_framerate = 1200,
		.mipi_pixel_rate = 1432300000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { //{12000, 9000, 240, 1260, 11520, 6480, 1920,  1080, 0, 0, 1920,  1080, 0, 0, 1920,  1080},
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 240,
			.y0_offset = 1260,
			.w0_size = 11520,
			.h0_size = 6480,

			.scale_w = 1920,
			.scale_h = 1080,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1920,
			.h1_size = 1080,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1920,
			.h2_tg_size = 1080,
		},
//drv add by lipengpeng 20240606 end
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 98,
		},
	},
	{//slim video
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = addr_data_pair_slim_video,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_slim_video),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
//drv add by lipengpeng 20240606 start 
		.pclk = 1640000000,
		.linelength = 8752,
		.framelength = 1561,
		.max_framerate = 1200,
		.mipi_pixel_rate = 1432300000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { //{12000, 9000, 240, 1260, 11520, 6480, 1920,  1080, 0, 0, 1920,  1080, 0, 0, 1920,  1080},
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 240,
			.y0_offset = 1260,
			.w0_size = 11520,
			.h0_size = 6480,

			.scale_w = 1920,
			.scale_h = 1080,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1920,
			.h1_size = 1080,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1920,
			.h2_tg_size = 1080,
		},
//drv add by lipengpeng 20240606 end 
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 98,
		},

	},
	
		//.pclk = 1640000000,
		//.linelength = 20472,
		//.framelength = 9984,
		//.startx = 0,
		//.starty = 0,
		//.grabwindow_width = 12000,
		//.grabwindow_height = 9000,
		//.mipi_data_lp2hs_settle_dc = 85,
		//.max_framerate = 80,
		//.mipi_pixel_rate = 1432300000,
		
	{//custom1
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = addr_data_pair_custom1,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom1),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1640000000,
		.linelength = 20472,
		.framelength = 9984,
		.max_framerate = 300,
		.mipi_pixel_rate = 1432300000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 12000,
			.h0_size = 9000,

			.scale_w = 12000,
			.scale_h = 9000,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 12000,
			.h1_size = 9000,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 12000,
			.h2_tg_size = 9000,
		},

		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = S5KHM6SX_SENSOR_ID,
	.reg_addr_sensor_id = {0x0000, 0x0001},  //((read_cmos_sensor_byte(0x0000) << 8) | read_cmos_sensor_byte(0x0001));
	.i2c_addr_table = {0x20,0x5a,0x7a,0xac, 0xff},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {12000, 9000},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,  //chpy
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,//SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 128,
	.ana_gain_type = 2,
	.ana_gain_step = 1,
	.ana_gain_table = s5khm6sx_ana_gain_table,
	.ana_gain_table_size = sizeof(s5khm6sx_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 3,
	.exposure_max = 0xFFFF - 3,
	.exposure_step = 1,
	.exposure_margin = 3,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = BASE_DGAIN * 16,
	.dig_gain_step = 4,
	.saturation_info = &imgsensor_saturation_info,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 3000000,

//drv add by lipengpeng 20240531 start for PDAF
	.pdaf_type = PDAF_SUPPORT_CAMSV,// PDAF_SUPPORT_CAMSV,  PDAF_SUPPORT_NA
	.hdr_type = HDR_SUPPORT_STAGGER_FDOL,
//drv add by lipengpeng 20240531 end  for PDAF 
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,
	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,  //write_cmos_sensor_byte(0x0100, 0x01);
	.reg_addr_mirror_flip = 0x0101,  //write_cmos_sensor_byte(0x0101, 0x00); //GR
	.reg_addr_exposure = {{0x0202, 0x0203},},  //write_cmos_sensor16(0x0202, shutter & 0xFFFF);
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = 0x0702,
	.reg_addr_ana_gain = {{0x0204, 0x0205},}, 
	.reg_addr_dig_gain = {{0x020e, 0x020f},},
	.reg_addr_frame_length = {0x0340, 0x0341}, //write_cmos_sensor16(0x0340, shutter & 0xFFFF);
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = 0x0005,

	.init_setting_table = PARAM_UNDEFINED,
	.init_setting_len = PARAM_UNDEFINED,
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,

	.checksum_value = 0x47a75476,
};

//drv add by lipengpeng 20240601 start
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = 2;
	u32 addr_h = ctx->s_ctx.reg_addr_sensor_id.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_sensor_id.addr[1];
	u32 addr_ll = ctx->s_ctx.reg_addr_sensor_id.addr[2];

#if IS_ENABLED(CONFIG_DRV_ADC_GMS_DETECT)
    printk("s5khm6sx ---- gms_detection_getadc_v()=%d\n",gms_detection_getadc_v());
  if(check_board_type()==0)
    {
	 printk(" s5khm6sx is volume production\n");
    }else{
	  *sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
    }
#endif	
	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) |
				subdrv_i2c_rd_u8(ctx, addr_l);
			if (addr_ll)
				*sensor_id = ((*sensor_id) << 8) | subdrv_i2c_rd_u8(ctx, addr_ll);
			//*sensor_id +=1;
			printk("s5khm6sx i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == S5KHM6SX_SENSOR_ID) {
				*sensor_id = ctx->s_ctx.sensor_id;
				return ERROR_NONE;
			}
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != ctx->s_ctx.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}
//drv add by lipengpeng 20240601 end

static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_RST, 0, 1},
	{HW_ID_PDN, 0, 1},
	{HW_ID_AVDD, 2800000, 1},
	{HW_ID_DVDD, 1200000, 1},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_RST, 1, 10},
	{HW_ID_PDN, 1, 1},
	{HW_ID_MCLK, 24, 1},
	{HW_ID_MCLK1, 24, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 6, 20},
	{HW_ID_MCLK1_DRIVING_CURRENT, 6, 20},
};

const struct subdrv_entry s5khm6sx_mipi_raw_entry = {
	.name = "s5khm6sx_mipi_raw",
	.id = S5KHM6SX_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	if (en)
		set_i2c_buffer(ctx, 0x0104, 0x01);
	else
		set_i2c_buffer(ctx, 0x0104, 0x00);
}

static u16 get_gain2reg(u32 gain)
{
	return gain * 32 / BASEGAIN;
}

static int s5khm6sx_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	if (mode)
		subdrv_i2c_wr_u16(ctx, 0x0600, 0x0002); /*100% Color bar*/
	else if (ctx->test_pattern)
		subdrv_i2c_wr_u16(ctx, 0x0600, 0x0000); /*No pattern*/

	ctx->test_pattern = mode;

	return 0;
}

static int s5khm6sx_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct mtk_test_pattern_data *data = (struct mtk_test_pattern_data *)para;
	u16 R = (data->Channel_R >> 22) & 0x3ff;
	u16 Gr = (data->Channel_Gr >> 22) & 0x3ff;
	u16 Gb = (data->Channel_Gb >> 22) & 0x3ff;
	u16 B = (data->Channel_B >> 22) & 0x3ff;

	subdrv_i2c_wr_u16(ctx, 0x0602, Gr);
	subdrv_i2c_wr_u16(ctx, 0x0604, R);
	subdrv_i2c_wr_u16(ctx, 0x0606, B);
	subdrv_i2c_wr_u16(ctx, 0x0608, Gb);

	DRV_LOG(ctx, "mode(%u) R/Gr/Gb/B = 0x%04x/0x%04x/0x%04x/0x%04x\n",
		ctx->test_pattern, R, Gr, Gb, B);

	return 0;
}

static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;

	return 0;
}

static int s5khm6sx_sensor_init(struct subdrv_ctx *ctx)
{
	printk("s5khm6sx_sensor_init start\n");
	
	subdrv_i2c_wr_u16(ctx, 0xFCFC,0x4000);
	subdrv_i2c_wr_u16(ctx, 0x0000,0x01D0);
	subdrv_i2c_wr_u16(ctx, 0x0000,0x1AD6);
	subdrv_i2c_wr_u16(ctx, 0xFCFC,0x4000);    //Init
	subdrv_i2c_wr_u16(ctx, 0x6010,0x0001);
    mdelay(30);      //p30	       
	subdrv_i2c_wr_u16(ctx, 0x6218,0xE9C0);
	subdrv_i2c_wr_u16(ctx, 0xF468,0x0000);
	subdrv_i2c_wr_u16(ctx, 0x0136,0x1800);

	i2c_table_write(ctx, sensor_init_addr_data, sizeof(sensor_init_addr_data)/sizeof(u16));
	printk("s5khm6sx_sensor_init end \n");

	return 0;
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	s5khm6sx_sensor_init(ctx);

	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	memset(ctx->ana_gain, 0, sizeof(ctx->gain));
	ctx->exposure[0] = ctx->s_ctx.exposure_def;
	ctx->ana_gain[0] = ctx->s_ctx.ana_gain_def;
	ctx->current_scenario_id = scenario_id;
	ctx->pclk = ctx->s_ctx.mode[scenario_id].pclk;
	ctx->line_length = ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length = ctx->s_ctx.mode[scenario_id].framelength;
	ctx->current_fps = 10 * ctx->pclk / ctx->line_length / ctx->frame_length;
	ctx->readout_length = ctx->s_ctx.mode[scenario_id].readout_length;
	ctx->read_margin = ctx->s_ctx.mode[scenario_id].read_margin;
	ctx->min_frame_length = ctx->frame_length;
	ctx->autoflicker_en = FALSE;
	ctx->test_pattern = 0;
	ctx->ihdr_mode = 0;
	ctx->pdaf_mode = 0;
	ctx->hdr_mode = 0;
	ctx->extend_frame_length_en = 0;
	ctx->is_seamless = 0;
	ctx->fast_mode_on = 0;
	ctx->sof_cnt = 0;
	ctx->ref_sof_cnt = 0;
	ctx->is_streaming = 0;
	
    printk("s5khm6sx open end\n");
	
	return ERROR_NONE;
} /* open */

/*static void s5khm6sx_set_streaming_control(struct subdrv_ctx *ctx, bool enable)
{
	u64 stream_ctrl_delay_timing = 0;

	DRV_LOG(ctx, "E! enable:%u\n", enable);
	check_current_scenario_id_bound(ctx);
	if (ctx->s_ctx.aov_sensor_support && ctx->s_ctx.streaming_ctrl_imp) {
		if (ctx->s_ctx.s_streaming_control != NULL)
			ctx->s_ctx.s_streaming_control((void *) ctx, enable);
		else
			DRV_LOG_MUST(ctx,
				"please implement drive own streaming control!(sid:%u)\n",
				ctx->current_scenario_id);
		ctx->is_streaming = enable;
		DRV_LOG_MUST(ctx, "enable:%u\n", enable);
		return;
	}
	if (ctx->s_ctx.aov_sensor_support && ctx->s_ctx.mode[ctx->current_scenario_id].aov_mode) {
		DRV_LOG_MUST(ctx,
			"stream ctrl implement on scp side!(sid:%u)\n",
			ctx->current_scenario_id);
		ctx->is_streaming = enable;
		DRV_LOG_MUST(ctx, "enable:%u\n", enable);
		return;
	}
	
    subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);	 //// Page pointer HW
	
	if (enable) {
		set_dummy(ctx);
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x01);
		ctx->stream_ctrl_start_time = ktime_get_boottime_ns();
	} else {
		ctx->stream_ctrl_end_time = ktime_get_boottime_ns();
		if (ctx->s_ctx.custom_stream_ctrl_delay &&
			ctx->stream_ctrl_start_time && ctx->stream_ctrl_end_time) {
			stream_ctrl_delay_timing =
				(ctx->stream_ctrl_end_time - ctx->stream_ctrl_start_time) / 1000000;
			DRV_LOG_MUST(ctx,
				"custom_stream_ctrl_delay/stream_ctrl_delay_timing:%llu/%llu\n",
				ctx->s_ctx.custom_stream_ctrl_delay,
				stream_ctrl_delay_timing);
			if (stream_ctrl_delay_timing < ctx->s_ctx.custom_stream_ctrl_delay)
				mdelay(
					ctx->s_ctx.custom_stream_ctrl_delay - stream_ctrl_delay_timing);
		}
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x00);
		
		if (ctx->s_ctx.reg_addr_fast_mode && ctx->fast_mode_on) {
			ctx->fast_mode_on = FALSE;
			ctx->ref_sof_cnt = 0;
			DRV_LOG(ctx, "seamless_switch disabled.");
			set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
			commit_i2c_buffer(ctx);
		}
		memset(ctx->exposure, 0, sizeof(ctx->exposure));
		memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
		ctx->autoflicker_en = FALSE;
		ctx->extend_frame_length_en = 0;
		ctx->is_seamless = 0;
		if (ctx->s_ctx.chk_s_off_end)
			check_stream_off(ctx);
		ctx->stream_ctrl_start_time = 0;
		ctx->stream_ctrl_end_time = 0;
	}
	ctx->sof_no = 0;
	ctx->is_streaming = enable;
	DRV_LOG(ctx, "X! enable:%u\n", enable);
}

static int s5khm6sx_set_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	s5khm6sx_set_streaming_control(ctx, TRUE);
	return 0;
}
static int s5khm6sx_set_streaming_suspned(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	s5khm6sx_set_streaming_control(ctx, FALSE);
	return 0;
}
	if (enable)
	{
		// while (1)
		for (i = 0; i < 1000; i++)
		{
			write_cmos_sensor_byte(0x0100, 0x01);
			isStreamOn = read_cmos_sensor_byte(0x0100); 
			LOG_INF("isStreamOn %d ", isStreamOn);

			if ((isStreamOn & 0x1) == 0x01)
			{
				return ERROR_NONE;
			}
			else
			{
				mdelay(1);
			}
		}
	}
	else
	{

		//while(1) {
		for (i = 0; i < 1000; i++)
		{
			write_cmos_sensor_byte(0x0100, 0x00);
			framecnt = read_cmos_sensor_byte(0x0005);
			if ((framecnt & 0xff) == 0xFF)
			{
				LOG_INF("StreamOff OK at framecnt=%d.\n", framecnt);
				break;
			}
			else
			{
				LOG_INF("StreamOFF is not on, %d, i=%d", framecnt, i);
				mdelay(1);
			}
		}
	}
*/
