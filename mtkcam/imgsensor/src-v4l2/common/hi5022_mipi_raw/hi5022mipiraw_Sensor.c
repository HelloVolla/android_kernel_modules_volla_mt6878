// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 hi5022mipiraw_Sensor.c
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
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include "hi5022mipiraw_Sensor.h"

#define HI5022_SET_SHUTTER 1

static u16 get_gain2reg(u32 gain);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static int hi5022_set_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int hi5022_set_streaming_suspned(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void hi5022_set_streaming_control(struct subdrv_ctx *ctx, bool enable);
//drv add by lipengpeng 20240509 start 
#ifdef HI5022_SET_SHUTTER
static int hi5022_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int hi5022_set_shutter_frame_length(struct subdrv_ctx *ctx,u8 *para, u32 *len);
#endif
static int hi5022_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
//drv add by lipengpeng 20240509 end 

//drv add by lipengpeng 20240829 start 
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
//drv add by lipengpeng 20240829 end 

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, hi5022_set_test_pattern},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, hi5022_set_streaming_resume},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, hi5022_set_streaming_suspned},
#ifdef HI5022_SET_SHUTTER
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, hi5022_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_ESHUTTER, hi5022_set_shutter},
#endif
	
};


static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {//4096x3072
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x1000, //Must be multiple of 8
			.vsize = 0x0c00, //Must be multiple of 4
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x1000, //Must be multiple of 8
			.vsize = 0x0c00, //Must be multiple of 4
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x1000, //Must be multiple of 8
			.vsize = 0x0c00, //Must be multiple of 4
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs[] = { //1280x720
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0500, //Must be multiple of 8
			.vsize = 0x02D0, //Must be multiple of 4
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim[] = {//1920x1080
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0780, //Must be multiple of 8
			.vsize = 0x0438, //Must be multiple of 4
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {//8192x6144
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x1FFE, //Must be multiple of 8
			.vsize = 0x1800, //Must be multiple of 4
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};


static struct subdrv_mode_struct mode_struct[] = {
	// frame_desc_prev
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = hi5022_preview_setting,
		.mode_setting_len = ARRAY_SIZE(hi5022_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 124000000,
		.linelength = 1158,
		.framelength = 3568,
		.max_framerate = 300,
		.mipi_pixel_rate = 662400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = { //{ 8224, 6176,  16,   16, 8192, 6144,  4096, 3072,   0,  0, 4096, 3072, 0, 0, 4096, 3072},
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,  //???不同模式的亮度比
		.fine_integ_line = PARAM_UNDEFINED,
		.delay_frame = 3,
	//DPHY:
		.csi_param = {  
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value  
		},
    //CPHY:
	   //.csi_param = {
	  //	     0
	  //	   },
		.dpc_enabled = true,
	},
	// frame_desc_cap
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = hi5022_capture_setting,
		.mode_setting_len = ARRAY_SIZE(hi5022_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 124000000,
		.linelength = 1158,
		.framelength = 3568,
		.max_framerate = 300,
		.mipi_pixel_rate = 662400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = { //{ 8224, 6176,  16,   16, 8192, 6144,  4096, 3072,   0,  0, 4096, 3072, 0, 0, 4096, 3072},
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,//???不同模式的亮度比
		.fine_integ_line = PARAM_UNDEFINED,
		.delay_frame = 3,
	//DPHY:
		.csi_param = {  
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value  
		},
    //CPHY:
	   //.csi_param = {
	  //	     0
	  //	   },
		.dpc_enabled = true,
	},
	// frame_desc_vid
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = hi5022_video_setting,
		.mode_setting_len = ARRAY_SIZE(hi5022_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 124000000,
		.linelength = 1158,
		.framelength = 3568,
		.max_framerate = 300,
		.mipi_pixel_rate = 662400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = { //{ 8224, 6176,  16,   16, 8192, 6144,  4096, 3072,   0,  0, 4096, 3072, 0, 0, 4096, 3072},
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,//???不同模式的亮度比
		.fine_integ_line = PARAM_UNDEFINED,
		.delay_frame = 3,
	//DPHY:
		.csi_param = {  
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value  
		},
    //CPHY:
	   //.csi_param = {
	  //	     0
	  //	   },
		.dpc_enabled = true,
	},
	// frame_desc_hs
	{
		.frame_desc = frame_desc_hs,
		.num_entries = ARRAY_SIZE(frame_desc_hs),
		.mode_setting_table = hi5022_hs_setting,
		.mode_setting_len = ARRAY_SIZE(hi5022_hs_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 124000000,
		.linelength = 402,
		.framelength = 2507,
		.max_framerate = 1200,
		.mipi_pixel_rate = 662400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = { //{ 8224, 6176,  16, 1648, 8192, 2880,  2048,  720, 384,  0, 1280,  720, 0, 0, 1280,  720},
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 1632,
			.w0_size = 8192,
			.h0_size = 2880,
			.scale_w = 2048,
			.scale_h = 720,
			.x1_offset = 720,
			.y1_offset = 384,
			.w1_size = 1280,
			.h1_size = 720,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1280,
			.h2_tg_size = 720,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,  //???不同模式的亮度比
		.fine_integ_line = PARAM_UNDEFINED,
		.delay_frame = 3,
		.csi_param = {  //mipi配置
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	// frame_desc_slim
//.pclk = 124000000,
//.linelength = 432,
//.framelength = 4794, 	
//.startx = 0,			
//.starty = 0,				
//.grabwindow_width = 1920,
//.grabwindow_height = 1080,
//.mipi_data_lp2hs_settle_dc = 85,
//.max_framerate = 600,
//.mipi_pixel_rate = 662400000,	
	{//{ 8224, 6176,  16,  928, 8192, 4320,  2048, 1080,  64,  0, 1920, 1080, 0, 0, 1920, 1080},
		.frame_desc = frame_desc_slim,
		.num_entries = ARRAY_SIZE(frame_desc_slim),
		.mode_setting_table = hi5022_slim_setting,
		.mode_setting_len = ARRAY_SIZE(hi5022_slim_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 124000000,
		.linelength = 432,
		.framelength = 4794,
		.max_framerate = 600,
		.mipi_pixel_rate = 662400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = { //{ 8224, 6176,  16,  928, 8192, 4320,  2048, 1080,  64,  0, 1920, 1080, 0, 0, 1920, 1080},
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 912,
			.w0_size = 8192,
			.h0_size = 4320,
			.scale_w = 2048,
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
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = PARAM_UNDEFINED,
		.delay_frame = 3,
		.csi_param = {
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	// frame_desc_cus1
//.pclk = 124000000,
//.linelength =  1616,
//.framelength = 7665,
//.startx = 0,
//.starty = 0,
//.grabwindow_width = 8192,
//.grabwindow_height = 6144,
///*     following for MIPIDataLowPwr2HighSpeedSettleDelayCountby different scenario    */
//.mipi_data_lp2hs_settle_dc = 85,
///*     following for GetDefaultFramerateByScenario()    */
//.max_framerate = 100,
//.mipi_pixel_rate =662400000,//1656Mbps*4/10	
	{
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = hi5022_cus1_setting,
		.mode_setting_len = ARRAY_SIZE(hi5022_cus1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 124000000,
		.linelength = 1616,
		.framelength = 7665,
		.max_framerate = 100,
		.mipi_pixel_rate = 662400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = { //{ 8224, 6176, 16,   16, 8192, 6144, 8192, 6144,    0, 0,  8192, 6144, 0,0, 8192, 6144},
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 8192,
			.scale_h = 6144,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 8192,
			.h1_size = 6144,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 8192,
			.h2_tg_size = 6144,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = PARAM_UNDEFINED,
		.delay_frame = 3,
		.csi_param = {
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = HI5022_SENSOR_ID,
//drv add by lipengpeng 20240828 start 
	.reg_addr_sensor_id = {0x0716,0x0717},
	//.reg_addr_sensor_id = {0x0716},
//drv add by lipengpeng 20240828 end 
	.i2c_addr_table = {0x40,0x42, 0x44,0xff},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info = PARAM_UNDEFINED,
	.eeprom_num = 0,
	.resolution = {8192, 6144}, //8224x6176 by lipengpeng
	.mirror = IMAGE_NORMAL,
	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_CSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_type = 3,
	.ana_gain_step = 1,
	.ana_gain_table = hi5022_ana_gain_table,
	.ana_gain_table_size = sizeof(hi5022_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = (0xFFFF - 4),
	.exposure_step = 1,
	.exposure_margin = 4,

	.frame_length_max = 0xffffff,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 5500000,

	.pdaf_type = PDAF_SUPPORT_NA,
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,

	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = get_gain2reg,
	.s_gph = PARAM_UNDEFINED,
	.s_cali = PARAM_UNDEFINED,


	.reg_addr_stream = 0x0b00,  // stream on/off reg:0x0b00   hi5022_set_streaming_resume  add16--->data16
	.reg_addr_mirror_flip = 0x0202,
	//drv add by lipengpeng 20240430 start 
	//.reg_addr_exposure = {{0x0512, 0x0510},}, // Update Shutter  coarse_integration_time_h/coarse_integration_time_l/coarse_integration_time_hw OR  coarse_integration_time_hw/coarse_integration_time
	.reg_addr_exposure = {{0x0512, 0x0513},{0x0510, 0x0511},},  ////0x0512=high  0x0513=low ?   0x0510=high  0x0511=low ? 
    //drv add by lipengpeng 20240430 end  
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x050A, 0x050B},}, //analog_gain_code_global  0x050A=high  0x050B=low ?
	//drv add by lipengpeng 20240430 start 
	.reg_addr_frame_length = {0x050c,0x050d},  //0x050c=high  0x050d=low ?
	//.reg_addr_frame_length = {0x050c,0x050e},  //Extend frame length   frame_length_lines_h/frame_length_lines_l/frame_length_lines_hw  OR  frame_length_lines_hw/frame_length_lines
	//drv add by lipengpeng 20240430 end  
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = PARAM_UNDEFINED,
	.reg_addr_fast_mode = PARAM_UNDEFINED,

	.init_setting_table = hi5022_init_setting,
	.init_setting_len = ARRAY_SIZE(hi5022_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.checksum_value = 0xffffffff,
};

static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,//drv add by lipengpeng 20240828 
	.init_ctx = init_ctx,
	.open = open,//drv add by lipengpeng 20240828 
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.vsync_notify = vsync_notify,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = { //----------> need 4 power supper avdd iovdd dvdd && avdd1V8
//drv add by lipengpeng 20240430 start 
	{HW_ID_RST, 0, 1},
	{HW_ID_PDN, 0, 1},
	{HW_ID_DOVDD, 1800000, 3}, 
	{HW_ID_AVDD, 2800000, 1}, 
	{HW_ID_DVDD, 1100000, 1}, 
	{HW_ID_MCLK, 24, 5},
	{HW_ID_MCLK_DRIVING_CURRENT, 2, 0},
	{HW_ID_RST, 1, 2},
	{HW_ID_PDN, 1, 2},
//drv add by lipengpeng 20240430 end 
};

const struct subdrv_entry hi5022_mipi_raw_entry = {
	.name = "hi5022_mipi_raw",
	.id = HI5022_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static u16 get_gain2reg(u32 gain)//isp7sp BASEGAIN=1024 而非以前的64
{
	//return gain * 16 / BASEGAIN -  1 * 16;
	return  gain*16/BASEGAIN - 16;
}

//static kal_uint16 gain2reg(const kal_uint16 gain)
//{
//    kal_uint16 reg_gain = 0x0000;
//    reg_gain = gain / 4 - 16;
//
//    return (kal_uint16)reg_gain;
//}

//drv add by lipengpeng 20240509 start 
static kal_uint16 test_pattern_enb = 0;
static kal_uint16 test_pattern_isp = 0;
static kal_uint16 test_pattern_solid = 0;

static int hi5022_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);


	
	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
  if (test_pattern_solid == 0) {
		test_pattern_enb = subdrv_i2c_rd_u16(ctx, 0x0B04);
		test_pattern_solid = subdrv_i2c_rd_u16(ctx, 0x0C0A);
		test_pattern_isp = subdrv_i2c_rd_u16(ctx, 0x0B06);
	}
	
	if (mode) {
		subdrv_i2c_wr_u16(ctx, 0x0B04, (test_pattern_enb & ~0x15FE) | 0x0001); //disable B[12], B[10], B[8:1], // Test Pattern Enb B[0]
		subdrv_i2c_wr_u16(ctx, 0x0C0A, test_pattern_solid | 0x0101); //Solid color
		subdrv_i2c_wr_u16(ctx, 0x0B06, test_pattern_isp & ~0x0500);
	} else {
		subdrv_i2c_wr_u16(ctx, 0x0B04, test_pattern_enb);
		subdrv_i2c_wr_u16(ctx, 0x0C0A, test_pattern_solid);
		subdrv_i2c_wr_u16(ctx, 0x0B06, test_pattern_isp);
	}

	ctx->test_pattern = mode;
	return ERROR_NONE;
}
#ifdef HI5022_SET_SHUTTER
static int hi5022_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	return hi5022_set_shutter_frame_length(ctx, para, len);
}

static int hi5022_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u32 shutter = *feature_data;
	u32 frame_length = *(feature_data + 1);
	u32 fine_integ_line = 0;
	static int longexposue = 0;
	u32 fll = 0;
	u32 fll_step = 0;
	u32 dol_cnt = 1;
//drv add by lipengpeng 20240509 start 
	kal_uint16 new_framelength;
	kal_uint16 long_shutter=0;
//drv add by lipengpeng 20240509 end
 
	printk("s5kgn1sp_shutter = 0x%x \n", shutter);
	printk("s5kgn1sp_frame_length = 0x%x \n", frame_length);
	ctx->frame_length = frame_length ? frame_length : ctx->frame_length;
	check_current_scenario_id_bound(ctx);
	/* check boundary of framelength */
	ctx->frame_length =	max(shutter + ctx->s_ctx.exposure_margin, ctx->min_frame_length);
	ctx->frame_length =	min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	/* check boundary of shutter */
	fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	shutter = FINE_INTEG_CONVERT(shutter, fine_integ_line);
	shutter = max(shutter, ctx->s_ctx.exposure_min);
	/* restore shutter */
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	ctx->exposure[0] = shutter;
	/* set_long_exposure */
	if (ctx->s_ctx.long_exposure_support == TRUE) {
		if (shutter > 126310) {
//drv add by lipengpeng 20240509 start 
			printk("s5kgn1sp enter long exposure!");
			longexposue = 1;
			//shutter = (shutter - 0xd10)- 1;
			//subdrv_i2c_wr_u8(ctx, 0x0202, 0x0d);
			//subdrv_i2c_wr_u8(ctx, 0x0203, 0x10);
			//subdrv_i2c_wr_u8(ctx, 0x0340, 0x0d);
			//subdrv_i2c_wr_u8(ctx, 0x0341, 0x20);
			//subdrv_i2c_wr_u8(ctx, 0x022c, 0x8c);
			///subdrv_i2c_wr_u8(ctx, 0x022d, (shutter >> 16) & 0xFF);
			//subdrv_i2c_wr_u8(ctx, 0x022e, (shutter >> 8) & 0xFF);
			//subdrv_i2c_wr_u8(ctx, 0x022f, (shutter) & 0xFF);
			//subdrv_i2c_wr_u8(ctx, 0x0230, 0x0c);
			long_shutter = shutter / 128 ;
			printk("enter long exposure mode long_shutter = %d\n", long_shutter);
			new_framelength = long_shutter + 5;

			subdrv_i2c_wr_u16(ctx, 0x6028, 0x4000);
			subdrv_i2c_wr_u16(ctx, 0x0340, new_framelength & 0xFFFF);
			subdrv_i2c_wr_u16(ctx, 0x0202, long_shutter & 0xFFFF);
			subdrv_i2c_wr_u16(ctx, 0x0702, 0x0700);
			subdrv_i2c_wr_u16(ctx, 0x0704, 0x0700);
//drv add by lipengpeng 20240509 end  		
		} else {
			if (longexposue == 1) {
				printk("s5kgn1sp exit long exposure!");
//drv add by lipengpeng 20240509 start
				//subdrv_i2c_wr_u8(ctx, 0x022f, 0x00);
				//subdrv_i2c_wr_u8(ctx, 0x022e, 0x00);
				//subdrv_i2c_wr_u8(ctx, 0x022d, 0x00);
				//subdrv_i2c_wr_u8(ctx, 0x0232, 0x00);
				//subdrv_i2c_wr_u8(ctx, 0x0230, 0x08);
				subdrv_i2c_wr_u16(ctx, 0x6028, 0x4000);
				//subdrv_i2c_wr_u16(ctx, 0x0340, imgsensor.frame_length & 0xFFFF);
				//subdrv_i2c_wr_u16(ctx, 0x0202, shutter & 0xFFFF);
				subdrv_i2c_wr_u16(ctx, 0x0702, 0x0000);
				subdrv_i2c_wr_u16(ctx, 0x0704, 0x0000);
//drv add by lipengpeng 20240509 end 
				longexposue = 0;
			}
		}
	}
	shutter = min(shutter, ctx->s_ctx.exposure_max);
	/* write framelength */		
	if (set_auto_flicker(ctx, 0) || frame_length ||
		!ctx->s_ctx.reg_addr_auto_extend) {
		fll = ctx->frame_length;
		fll_step = ctx->s_ctx.mode[ctx->current_scenario_id].framelength_step;
		if (fll_step)
			fll = roundup(fll, fll_step);
		ctx->frame_length = fll;
		if (ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_STAGGER)
			dol_cnt = ctx->s_ctx.mode[ctx->current_scenario_id].exp_cnt;
		fll = fll / dol_cnt;
		if (ctx->extend_frame_length_en == FALSE) {
//drv add by lipengpeng 20240509 start 
			//subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0],
			//	(fll >> 8) & 0xFF);
			//subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[1],
			//	fll & 0xFF);
			subdrv_i2c_wr_u16(ctx, 0x050c, fll & 0xFFFF);
//drv add by lipengpeng 20240509 end 
		}
	}
	/* write shutter */
//drv add by lipengpeng 20240509 start
	//subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[0],
	//	(ctx->exposure[0] >> 8) & 0xFF);
	//subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[1],
	//	ctx->exposure[0] & 0xFF);
	subdrv_i2c_wr_u16(ctx, 0x0512, (ctx->exposure[0] & 0xFFFF0000) >> 16 );
    subdrv_i2c_wr_u16(ctx, 0x0510, ctx->exposure[0]);
//drv add by lipengpeng 20240509 end

	printk("exp[0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		ctx->exposure[0], frame_length, ctx->frame_length, ctx->autoflicker_en);
	return ERROR_NONE;
}
#endif
//drv add by lipengpeng 20240509 end  

static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	return 0;
}

static void hi5022_set_streaming_control(struct subdrv_ctx *ctx, bool enable)
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

	if (enable) {
		set_dummy(ctx);
		subdrv_i2c_wr_u16(ctx, ctx->s_ctx.reg_addr_stream, 0x0100);  //stream on
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
		subdrv_i2c_wr_u16(ctx, ctx->s_ctx.reg_addr_stream, 0x0000);// stream off
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

static int hi5022_set_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	hi5022_set_streaming_control(ctx, TRUE);
	return 0;
}
static int hi5022_set_streaming_suspned(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	hi5022_set_streaming_control(ctx, FALSE);
	return 0;
}

//drv add by lipengpeng 20240829 start 
static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	if (ctx->s_ctx.aov_sensor_support && !ctx->s_ctx.init_in_open)
		DRV_LOG_MUST(ctx, "sensor init not in open stage!\n");
	else
		sensor_init(ctx);

	if (ctx->s_ctx.s_cali != NULL)
		ctx->s_ctx.s_cali((void *) ctx);
	else
		write_sensor_Cali(ctx);

	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	memset(ctx->ana_gain, 0, sizeof(ctx->gain));
	ctx->exposure[0] = ctx->s_ctx.exposure_def;
	ctx->ana_gain[0] = ctx->s_ctx.ana_gain_def;
	ctx->current_scenario_id = scenario_id;
	ctx->pclk = ctx->s_ctx.mode[scenario_id].pclk;
	ctx->line_length = ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length = ctx->s_ctx.mode[scenario_id].framelength;
	ctx->frame_length_rg = ctx->frame_length;
	ctx->current_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
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
	if (ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_LBMF) {
		memset(ctx->frame_length_in_lut, 0,
			sizeof(ctx->frame_length_in_lut));

		switch (ctx->s_ctx.mode[ctx->current_scenario_id].exp_cnt) {
		case 2:
			ctx->frame_length_in_lut[0] = ctx->readout_length + ctx->read_margin;
			ctx->frame_length_in_lut[1] = ctx->frame_length -
				ctx->frame_length_in_lut[0];
			break;
		case 3:
			ctx->frame_length_in_lut[0] = ctx->readout_length + ctx->read_margin;
			ctx->frame_length_in_lut[1] = ctx->readout_length + ctx->read_margin;
			ctx->frame_length_in_lut[2] = ctx->frame_length -
				ctx->frame_length_in_lut[1] - ctx->frame_length_in_lut[0];
			break;
		default:
			break;
		}

		memcpy(ctx->frame_length_in_lut_rg, ctx->frame_length_in_lut,
			sizeof(ctx->frame_length_in_lut_rg));
	}

	return ERROR_NONE;
}

static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = 2;
	u32 addr_h = ctx->s_ctx.reg_addr_sensor_id.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_sensor_id.addr[1];
	u32 addr_ll = ctx->s_ctx.reg_addr_sensor_id.addr[2];

	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) |
				subdrv_i2c_rd_u8(ctx, addr_l);
			if (addr_ll)
				*sensor_id = ((*sensor_id) << 8) | subdrv_i2c_rd_u8(ctx, addr_ll);
			//*sensor_id +=2;
			printk("HI5022 get_imgsensor_id(0x%x) sensor_id(0x%x/0x%x)\n",ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == HI5022_SENSOR_ID) {
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
//drv add by lipengpeng 20240829 end
