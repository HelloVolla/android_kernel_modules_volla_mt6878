// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 ov50e40mipiraw_Sensor.c
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
#include "ov50e40mipiraw_Sensor.h"
#define CUSTOM_SET_GAIN_SHUTTER 1
static void set_sensor_cali(void *arg);
static int get_sensor_temperature(void *arg);
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int ov50e40_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int ov50e40_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int ov50e40_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static int ov50e40_set_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int ov50e40_set_streaming_suspned(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void ov50e40_set_streaming_control(struct subdrv_ctx *ctx, bool enable);
#ifdef CUSTOM_SET_GAIN_SHUTTER
static int  ov50e40_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int  ov50e40_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
#endif
/* STRUCT */
#define USE_GAIN_TABLE 1
/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, ov50e40_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, ov50e40_set_test_pattern_data},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, ov50e40_seamless_switch},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, ov50e40_set_streaming_resume},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, ov50e40_set_streaming_suspned},
#ifdef CUSTOM_SET_GAIN_SHUTTER
	{SENSOR_FEATURE_SET_GAIN, ov50e40_set_gain},
	{SENSOR_FEATURE_SET_ESHUTTER, ov50e40_set_shutter},
#endif
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x00000000,
		.addr_header_id = 0x00000005,
		.i2c_write_id = 0xA6,
		// PDC Calibration
		.pdc_support = TRUE,
		.pdc_size = 0x0E00,
		.addr_pdc = 0x0D6F,
		.sensor_reg_addr_pdc = 0x5A20,
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0}},
	.i4PosR = {{0, 0}},
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = {
		// <pre> <cap> <normal_video> <hs_video> <<slim_video>>
		{0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 0},
		// <<cust1>> <<cust2>> <<cust3>> <cust4> <cust5>
		{0, 0}, {0, 0}, {0, 0}, {0, 384}, {0, 384},
		// <cust6> <cust7> <cust8> cust9 cust10
		{0, 384}, {0, 384}, {0, 384}, {2048, 1920}, {2048, 1536},
		// cust11 cust12 cust13 <cust14> <cust15>
		{2048, 1536}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		// <cust16> <cust17> cust18 <cust19> cust20
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {256, 912},
		// <cust21> <cust22> <cust23> <cust24> <cust25>
		{0, 0}, {0, 384}, {0, 384}, {0, 384}, {0, 0},
		// cust26 <cust27> cust28
		{1088, 996}, {0, 384}, {2048, 1536},
	},
	.iMirrorFlip = 0,
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.i4ModeIndex = 3,
	.i4VCPackNum = 2,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {1},
	},
};

//1000 base for dcg gain ratio
static u32 ov50e40_dcg_ratio_table_12bit[] = {4000};

static u32 ov50e40_dcg_ratio_table_14bit[] = {16000};

//static u32 ov50e40_dcg_ratio_table_ratio4[] = {4000};


static struct mtk_sensor_saturation_info imgsensor_saturation_info_10bit = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};

static struct mtk_sensor_saturation_info imgsensor_saturation_info_12bit = {
	.gain_ratio = 4000,
	.OB_pedestal = 16,
	.saturation_level = 3900,
};

static struct mtk_sensor_saturation_info imgsensor_saturation_info_14bit = {
	.gain_ratio = 16000,
	.OB_pedestal = 64,
	.saturation_level = 15408,
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0800,
			.vsize = 0x600,
			//.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0800,
			.vsize = 0x600,
			//.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_PIX_1,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0800,
			.vsize = 0x0480,
			//.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x780,
			.vsize = 0x438,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0780,
			.vsize = 0x0438,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2c,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x31,
			.hsize = 0x0800,
			.vsize = 0x600,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW12,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.valid_bit = 10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2d,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x32,
			.hsize = 0x0800,
			.vsize = 0x600,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW14,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.valid_bit = 10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus8[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus9[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus10[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus11[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus12[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x2000,
			.vsize = 0x1800,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus13[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x2000,
			.vsize = 0x1800,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = ov50e40_preview_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_preview_setting),
		.seamless_switch_group = 0,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 400,
		.framelength = 6250,
		.max_framerate = 300,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		//.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = ov50e40_capture_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_capture_setting),
		.seamless_switch_group = 0,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 400,
		.framelength = 6250,
		.max_framerate = 300,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		//.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = ov50e40_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_normal_video_setting),
		.seamless_switch_group = 0,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 400,
		.framelength = 6250,
		.max_framerate = 300,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		//.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = ov50e40_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 237,
		.framelength = 2636,
		.max_framerate = 1200,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		//.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 192,
			.y0_offset = 896,
			.w0_size = 7808,
			.h0_size = 4352,
			.scale_w = 1952,
			.scale_h = 1088,
			.x1_offset = 16,
			.y1_offset = 4,
			.w1_size = 1920,
			.h1_size = 1080,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1920,
			.h2_tg_size = 1080,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = ov50e40_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 237,
		.framelength = 1318,
		.max_framerate = 2400,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		//.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 192,
			.y0_offset = 896,
			.w0_size = 7808,
			.h0_size = 4352,
			.scale_w = 1952,
			.scale_h = 1088,
			.x1_offset = 16,
			.y1_offset = 4,
			.w1_size = 1920,
			.h1_size = 1080,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1920,
			.h2_tg_size = 1080,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = ov50e40_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 237,
		.framelength = 5274,
		.max_framerate = 600,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		//.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 192,
			.y0_offset = 896,
			.w0_size = 7808,
			.h0_size = 4352,
			.scale_w = 1952,
			.scale_h = 1088,
			.x1_offset = 16,
			.y1_offset = 4,
			.w1_size = 1920,
			.h1_size = 1080,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1920,
			.h2_tg_size = 1080,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_cus2, //4k@60fps
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = ov50e40_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_custom2_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 400,
		.framelength = 3128,
		.max_framerate = 600,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		//.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
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
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = ov50e40_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_custom3_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 78000000,
		.linelength = 214,
		.framelength = 758,
		.max_framerate = 4800,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		//.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 192,
			.y0_offset = 1632,
			.w0_size = 7808,
			.h0_size = 2880,
			.scale_w = 1952,
			.scale_h = 720,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1280,
			.h1_size = 720,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1280,
			.h2_tg_size = 720,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = ov50e40_custom4_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_custom4_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_RAW_DCG_COMPOSE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 775,
		.framelength = 1792,
		.max_framerate = 300,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.imgsensor_winsize_info = {
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW12_B,
		.saturation_info = &imgsensor_saturation_info_12bit,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_COMPOSE,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_HCG_BASE,
			.dcg_gain_ratio_min = 4000,
			.dcg_gain_ratio_max = 4000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = ov50e40_dcg_ratio_table_12bit,
			.dcg_gain_table_size = sizeof(ov50e40_dcg_ratio_table_12bit),
		},
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 4,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 64,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 16,
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = ov50e40_custom5_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_custom5_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_RAW_DCG_COMPOSE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 612,
		.framelength = 4084,
		.max_framerate = 300,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.imgsensor_winsize_info = {
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW14_B,
		.saturation_info = &imgsensor_saturation_info_14bit,
		.dcg_info = {
			.dcg_mode = IMGSENSOR_DCG_COMPOSE,
			.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
			.dcg_gain_base = IMGSENSOR_DCG_GAIN_HCG_BASE,
			.dcg_gain_ratio_min = 16000,
			.dcg_gain_ratio_max = 16000,
			.dcg_gain_ratio_step = 0,
			.dcg_gain_table = ov50e40_dcg_ratio_table_14bit,
			.dcg_gain_table_size = sizeof(ov50e40_dcg_ratio_table_14bit),
		},
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 16,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 256,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 16,
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = ov50e40_custom6_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_custom6_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 400,
		.framelength = 3128,
		.max_framerate = 600,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		//.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
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
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_cus7,
		.num_entries = ARRAY_SIZE(frame_desc_cus7),
		.mode_setting_table = ov50e40_custom7_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_custom7_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 400,
		.framelength = 6250,
		.max_framerate = 300,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		//.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
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
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_cus8,
		.num_entries = ARRAY_SIZE(frame_desc_cus8),
		.mode_setting_table = ov50e40_custom8_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_custom8_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 400,
		.framelength = 6250,
		.max_framerate = 300,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		//.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
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
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_cus9,
		.num_entries = ARRAY_SIZE(frame_desc_cus9),
		.mode_setting_table = ov50e40_custom9_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_custom9_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 400,
		.framelength = 6250,
		.max_framerate = 300,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		//.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
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
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_cus10,
		.num_entries = ARRAY_SIZE(frame_desc_cus10),
		.mode_setting_table = ov50e40_custom10_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_custom10_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 400,
		.framelength = 6250,
		.max_framerate = 300,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		//.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
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
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_cus11,
		.num_entries = ARRAY_SIZE(frame_desc_cus11),
		.mode_setting_table = ov50e40_custom11_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_custom11_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 400,
		.framelength = 6250,
		.max_framerate = 300,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		//.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
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
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_cus12,
		.num_entries = ARRAY_SIZE(frame_desc_cus12),
		.mode_setting_table = ov50e40_custom12_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_custom12_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 600,
		.framelength = 8332,
		.max_framerate = 300,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		//.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
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
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_cus13,
		.num_entries = ARRAY_SIZE(frame_desc_cus13),
		.mode_setting_table = ov50e40_custom13_setting,
		.mode_setting_len = ARRAY_SIZE(ov50e40_custom13_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 75000000,
		.linelength = 600,
		.framelength = 8332,
		.max_framerate = 150,
		.mipi_pixel_rate = 1225728000,
		.readout_length = 0,
		.read_margin = 0,
		//.framelength_step = 8,
		//.coarse_integ_step = 4,
		//.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.imgsensor_winsize_info = {
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
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.cphy_settle = 98,
		},
		.dpc_enabled = true,
	}
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = OV50E40_SENSOR_ID,
	.reg_addr_sensor_id = {0x300A, 0x300B, 0x300C},
	.i2c_addr_table = {0x6c, 0xFF}, // TBD
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {8192, 6144},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 1,
	.ana_gain_step = 4,
	.ana_gain_table = ov50e40_ana_gain_table,
	.ana_gain_table_size = sizeof(ov50e40_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 8,
	.exposure_max = 0xFFFFFF - 32,
	.exposure_step = 2,
	.exposure_margin = 32,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = BASE_DGAIN * 256,
	.dig_gain_step = 4,
	.saturation_info = &imgsensor_saturation_info_10bit,

	.frame_length_max = 0xFFF8,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 500000,

	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
	.hdr_type = HDR_SUPPORT_DCG,
	.seamless_switch_support = TRUE,
	.temperature_support = TRUE,

	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,
	.s_cali = set_sensor_cali,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = PARAM_UNDEFINED,
	.reg_addr_exposure = {
			{0x3500, 0x3501, 0x3502},
	},
	.reg_addr_exposure_in_lut = {

	},
	.long_exposure_support = FALSE,
	//.reg_addr_exposure_lshift = 0x3150,
	.reg_addr_ana_gain = {
			{0x3508, 0x3509},
	},
	.reg_addr_ana_gain_in_lut = {
	},
	.reg_addr_dig_gain = {
	},
	.reg_addr_dig_gain_in_lut = {
	},
	//.reg_addr_dcg_ratio = 0x3172,
	.reg_addr_frame_length = {0x3840, 0x380E, 0x380F},
	.reg_addr_frame_length_in_lut = {
	},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = 0x387F,

	.init_setting_table = ov50e40_init_setting,
	.init_setting_len = ARRAY_SIZE(ov50e40_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,

	.checksum_value = 0x487fbc31,
};

static struct subdrv_ops ops = {
	.get_id = common_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = common_open,
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

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_AFVDD, 2800000, 5},
	{HW_ID_RST, 0, 0},
	{HW_ID_PDN, 1, 10},////DCDC
	{HW_ID_AVDD, 1, 1}, // pmic_ldo for avdd Vin0
	{HW_ID_DVDD, 1100000, 1}, // pmic_ldo/gpio(1.1V ldo) for dvdd
	{HW_ID_DOVDD, 1800000, 1}, // pmic_ldo/gpio(1.8V ldo) for dovdd
	{HW_ID_MCLK, 24, 5},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 1},
	{HW_ID_RST, 1, 5}
};

const struct subdrv_entry ov50e40_mipi_raw_entry = {
	.name = "ov50e40_mipi_raw",
	.id = OV50E40_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static void set_sensor_cali(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	u16 idx = 0;
	u8 support = FALSE;
	u8 *pbuf = NULL;
	u16 size = 0;
	u16 addr = 0;
	u8 value = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;
	if (!probe_eeprom(ctx))
		return;

	idx = ctx->eeprom_index;

	/* PDC data */
	support = info[idx].pdc_support;
	pbuf = info[idx].preload_pdc_table;
	size = info[idx].pdc_size;
	addr = info[idx].sensor_reg_addr_pdc;
	value = subdrv_i2c_rd_u8(ctx, 0x5000);
	if (support) {
		if (pbuf != NULL && addr > 0 && size > 0) {
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
			value = value | 0x40;
			subdrv_i2c_wr_u8(ctx, 0x5000, value);  //BIT[6] SET 1
			DRV_LOG(ctx, "set PDC calibration data done.");
		} else {
			value = value & 0xBF; //BIT[6] SET 0
			subdrv_i2c_wr_u8(ctx, 0x5000, value);
		}
	}
}

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u8 temperature = 0;
	int temperature_convert = 0;

	temperature = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);

	if (temperature < 0x55)
		temperature_convert = temperature;
	else if (temperature < 0x80)
		temperature_convert = 85;
	else if (temperature < 0xED)
		temperature_convert = -20;
	else
		temperature_convert = (char)temperature;

	DRV_LOG(ctx, "temperature: %d degrees\n", temperature_convert);
	return temperature_convert;
}

static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	if (en) {
		set_i2c_buffer(ctx, 0x3208, 0x00);
	} else {
		set_i2c_buffer(ctx, 0x3208, 0x10);
		set_i2c_buffer(ctx, 0x3208, 0xA0);
	}
}

static u16 get_gain2reg(u32 gain)
{
	// ov50e40 max gain: 64x
	kal_uint16 reg_gain = 0x100;
	kal_uint16 gain_value = gain;
#ifdef USE_GAIN_TABLE
	int i = 0;
#endif

	if (gain_value < 0x100 || gain_value > 0xFF00) {
		printk("Error: gain value out of range %d", gain);

		if (gain_value < 0x100)
			gain_value = 0x100;
		else if (gain_value > 0xFF00)
			gain_value = 0xFF00;
	}

#ifdef USE_GAIN_TABLE
	reg_gain = ov50e40_gain_reg[OV50E40_GAIN_TABL_SIZE - 1];
	for (i = 0; i < OV50E40_GAIN_TABL_SIZE; i++) {
		if (gain_value <= ov50e40_gain_ratio[i]) {
			reg_gain = ov50e40_gain_reg[i];
			break;
		}
	}
#else
	reg_gain =gain * 256 / BASEGAIN;
#endif

	return (kal_uint16) reg_gain;
}

static int ov50e40_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	u64 *feature_data = (u64 *)para;
	u32 frame_length_in_lut[IMGSENSOR_STAGGER_EXPOSURE_CNT] = {0};
	u32 exp_cnt = 0;

	if (feature_data == NULL) {
		DRV_LOGE(ctx, "input scenario is null!");
		return ERROR_NONE;
	}
	scenario_id = *feature_data;
	if ((feature_data + 1) != NULL)
		ae_ctrl = (struct mtk_hdr_ae *)((uintptr_t)(*(feature_data + 1)));
	else
		DRV_LOGE(ctx, "no ae_ctrl input");

	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, "E: set seamless switch %u %u\n", ctx->current_scenario_id, scenario_id);
	if (!ctx->extend_frame_length_en)
		DRV_LOGE(ctx, "please extend_frame_length before seamless_switch!\n");
	ctx->extend_frame_length_en = FALSE;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOGE(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		return ERROR_NONE;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_group == 0 ||
		ctx->s_ctx.mode[scenario_id].seamless_switch_group !=
			ctx->s_ctx.mode[ctx->current_scenario_id].seamless_switch_group) {
		DRV_LOGE(ctx, "seamless_switch not supported\n");
		return ERROR_NONE;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table == NULL) {
		DRV_LOGE(ctx, "Please implement seamless_switch setting\n");
		return ERROR_NONE;
	}

	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	ctx->is_seamless = TRUE;

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x02);
	if (ctx->s_ctx.reg_addr_fast_mode_in_lbmf &&
		(ctx->s_ctx.mode[scenario_id].hdr_mode == HDR_RAW_LBMF ||
		ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_LBMF))
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode_in_lbmf, 0x4);

	update_mode_info(ctx, scenario_id);
	i2c_table_write(ctx,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);

	if (ae_ctrl) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_STAGGER:
			set_multi_shutter_frame_length(ctx, (u64 *)&ae_ctrl->exposure, exp_cnt, 0);
			set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		case HDR_RAW_LBMF:
			set_multi_shutter_frame_length_in_lut(ctx,
				(u64 *)&ae_ctrl->exposure, exp_cnt, 0, frame_length_in_lut);
			set_multi_gain_in_lut(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		case HDR_RAW_DCG_RAW:
			set_shutter(ctx, ae_ctrl->exposure.le_exposure);
			if (ctx->s_ctx.mode[scenario_id].dcg_info.dcg_gain_mode
				== IMGSENSOR_DCG_DIRECT_MODE)
				set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			else
				set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		default:
			set_shutter(ctx, ae_ctrl->exposure.le_exposure);
			set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		}
	}
	subdrv_i2c_wr_u8(ctx, 0x3208, 0x00);

	ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	DRV_LOG(ctx, "X: set seamless switch done\n");
	return ERROR_NONE;
}
static int ov50e40_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	if(mode == 1) {
		subdrv_i2c_wr_u8(ctx, 0x5000, 0x81);
		//subdrv_i2c_wr_u8(ctx, 0x5001, 0x58);
		subdrv_i2c_wr_u8(ctx, 0x50c1, 0x11);
	}
	else {
		subdrv_i2c_wr_u8(ctx, 0x5000, 0x89);
		//subdrv_i2c_wr_u8(ctx, 0x5001, 0x5a);
		subdrv_i2c_wr_u8(ctx, 0x50c1, 0x00);
	}
	ctx->test_pattern = mode;
	return ERROR_NONE;
}

static int ov50e40_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct mtk_test_pattern_data *data = (struct mtk_test_pattern_data *)para;
	u16 R = (data->Channel_R >> 22) & 0x3ff;
	u16 Gr = (data->Channel_Gr >> 22) & 0x3ff;
	u16 Gb = (data->Channel_Gb >> 22) & 0x3ff;
	u16 B = (data->Channel_B >> 22) & 0x3ff;

	DRV_LOG(ctx, "mode(%u) R/Gr/Gb/B = 0x%04x/0x%04x/0x%04x/0x%04x\n",
		ctx->test_pattern, R, Gr, Gb, B);
	return ERROR_NONE;
}

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
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
		commit_i2c_buffer(ctx);
	}
	return 0;
}
static void ov50e40_set_streaming_control(struct subdrv_ctx *ctx, bool enable)
{
	u64 stream_ctrl_delay_timing = 0;

	DRV_LOG(ctx, "E! enable:%u\n", enable);

	if (enable) {
			subdrv_i2c_wr_u8(ctx, 0x3208, 0x00);
			subdrv_i2c_wr_u8(ctx, 0x3a88, 0x44);
			subdrv_i2c_wr_u8(ctx, 0x3bd7, 0xc8);
			subdrv_i2c_wr_u8(ctx, 0x3bdc, 0x42);
			subdrv_i2c_wr_u8(ctx, 0x3be5, 0x01);
			subdrv_i2c_wr_u8(ctx, 0x392e, 0x5a);
			subdrv_i2c_wr_u8(ctx, 0x3208, 0x10);
			subdrv_i2c_wr_u8(ctx, 0x3208, 0x01);
			subdrv_i2c_wr_u8(ctx, 0x3a88, 0x04);
			subdrv_i2c_wr_u8(ctx, 0x3bd7, 0x48);
			subdrv_i2c_wr_u8(ctx, 0x3bdc, 0x02);
			subdrv_i2c_wr_u8(ctx, 0x3be5, 0x00);
			subdrv_i2c_wr_u8(ctx, 0x392e, 0x0a);
			subdrv_i2c_wr_u8(ctx, 0x3208, 0x11);
			subdrv_i2c_wr_u8(ctx, 0x320d, 0x01);
			subdrv_i2c_wr_u8(ctx, 0x3209, 0x01);
			subdrv_i2c_wr_u8(ctx, 0x320a, 0x01);
			subdrv_i2c_wr_u8(ctx, 0x320d, 0x91);
			subdrv_i2c_wr_u8(ctx, 0x320e, 0xa0);
			subdrv_i2c_wr_u8(ctx, 0x0100, 0x01);
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
		subdrv_i2c_wr_u8(ctx, 0x0100, 0x00);
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

static int ov50e40_set_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	ov50e40_set_streaming_control(ctx, TRUE);
	return 0;
}
static int ov50e40_set_streaming_suspned(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	ov50e40_set_streaming_control(ctx, FALSE);
	return 0;
}
#ifdef CUSTOM_SET_GAIN_SHUTTER
void ov50e40_set_long_exposure(struct subdrv_ctx *ctx)
{
	u32 shutter = ctx->exposure[IMGSENSOR_STAGGER_EXPOSURE_LE];
	u32 l_shutter = 0;
	u16 l_shift = 0;
	if (shutter > (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin)) {
		if (ctx->s_ctx.long_exposure_support == FALSE) {
			DRV_LOGE(ctx, "sensor no support of exposure lshift!\n");
			return;
		}
		if (ctx->s_ctx.reg_addr_exposure_lshift == PARAM_UNDEFINED) {
			DRV_LOGE(ctx, "please implement lshift register address\n");
			return;
		}
		for (l_shift = 1; l_shift < 7; l_shift++) {
			l_shutter = ((shutter - 1) >> l_shift) + 1;
			if (l_shutter
				< (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin))
				break;
		}
		if (l_shift > 7) {
			DRV_LOGE(ctx, "unable to set exposure:%u, set to max\n", shutter);
			l_shift = 7;
		}
		shutter = ((shutter - 1) >> l_shift) + 1;
		ctx->frame_length = shutter + ctx->s_ctx.exposure_margin;
		DRV_LOG(ctx, "long exposure mode: lshift %u times", l_shift);
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_exposure_lshift, l_shift);
		ctx->l_shift = l_shift;
		/* Frame exposure mode customization for LE*/
		ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		ctx->current_ae_effective_frame = 2;
	} else {
		if (ctx->s_ctx.reg_addr_exposure_lshift != PARAM_UNDEFINED) {
			set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_exposure_lshift, l_shift);
			ctx->l_shift = l_shift;
		}
		ctx->current_ae_effective_frame = 2;
	}

	ctx->exposure[IMGSENSOR_STAGGER_EXPOSURE_LE] = shutter;
}
void ov50e40_set_shutter_frame_length(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length)
{
	int fine_integ_line = 0;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);

	ctx->frame_length = frame_length ? frame_length : ctx->min_frame_length;
	check_current_scenario_id_bound(ctx);
	/* check boundary of shutter */
	fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	shutter = FINE_INTEG_CONVERT(shutter, fine_integ_line);
	shutter = max_t(u64, shutter,
		(u64)ctx->s_ctx.mode[ctx->current_scenario_id].multi_exposure_shutter_range[0].min);
	shutter = min_t(u64, shutter,
		(u64)ctx->s_ctx.mode[ctx->current_scenario_id].multi_exposure_shutter_range[0].max);
	/* check boundary of framelength */
	ctx->frame_length = max((u32)shutter + ctx->s_ctx.exposure_margin, ctx->min_frame_length);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	/* restore shutter */
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	ctx->exposure[0] = (u32) shutter;
	/* group hold start */
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);
	/* enable auto extend */
	if (ctx->s_ctx.reg_addr_auto_extend)
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_auto_extend, 0x01);
	/* write framelength */
	if (set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend)
		write_frame_length(ctx, ctx->frame_length);
	/* write shutter */
	ov50e40_set_long_exposure(ctx);
	if (ctx->s_ctx.reg_addr_exposure[0].addr[2]) {
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[0],
			(ctx->exposure[0] >> 16) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[1],
			(ctx->exposure[0] >> 8) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[2],
			ctx->exposure[0] & 0xFF);
	} else {
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[0],
			(ctx->exposure[0] >> 8) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[1],
			ctx->exposure[0] & 0xFF);
	}
	if(ctx->current_scenario_id == 8 || ctx->current_scenario_id == 9) {
		subdrv_i2c_wr_u8(ctx,	0x3540,(ctx->exposure[0] >> 16) & 0xFF);
		subdrv_i2c_wr_u8(ctx,	0x3541,(ctx->exposure[0] >> 8) & 0xFF);
		subdrv_i2c_wr_u8(ctx,	0x3542, ctx->exposure[0] & 0xFF);
	}
	DRV_LOG_MUST(ctx, "exp[0x%x], fll(input/output):%u/%u, flick_en:%d\n",
		ctx->exposure[0], frame_length, ctx->frame_length, ctx->autoflicker_en);
	if (!ctx->ae_ctrl_gph_en) {
		if (gph)
			ctx->s_ctx.s_gph((void *)ctx, 0);
		commit_i2c_buffer(ctx);
	}
	/* group hold end */
}
static int ov50e40_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u64 shutter = *feature_data;
	/*pri add by zhuzhengjiang for camera access off, sensor need output black start*/
	if(ctx->test_pattern==5) {
		shutter =0;
	}
	/*pri add by zhuzhengjiang for camera access off, sensor need output black end*/
	ov50e40_set_shutter_frame_length(ctx, shutter, 0);
	return ERROR_NONE;
}
static int  ov50e40_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u16  rg_gain;
	u16  sensitivity_l =0;
	u16  gain_lcg =0;
	u16  gain_hcg =0;
    u16  HDR_Ratio = 40; //bit12: HDR Ratio:4 bit16:HDR Ratio:16
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	u32  gain = *((u32 *)para);
	char expo_gain = 0;
	char hcg_maingain = 0;
	gain = max(gain,
		ctx->s_ctx.mode[ctx->current_scenario_id].multi_exposure_ana_gain_range[0].min);
	gain = min(gain,
		ctx->s_ctx.mode[ctx->current_scenario_id].multi_exposure_ana_gain_range[0].max);
	DRV_LOG_MUST(ctx, "gain %d \n",gain);
	/*pri add by zhuzhengjiang for camera access off, sensor need output black start*/
	if(ctx->test_pattern==5) {
		gain =0;
	}
	/*pri add by zhuzhengjiang for camera access off, sensor need output black end*/
	rg_gain = get_gain2reg(gain);
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	ctx->ana_gain[0] = gain;
	if (gph && !ctx->ae_ctrl_gph_en)
		ctx->s_ctx.s_gph((void *)ctx, 1);
	sensitivity_l = (subdrv_i2c_rd_u8(ctx, 0x5060) << 8) + subdrv_i2c_rd_u8(ctx, 0x5061); //
	//DRV_LOG_MUST(ctx, "dcg gain sensitivity_l:%d (0x%x 0x%x ) \n",sensitivity_l,subdrv_i2c_rd_u8(ctx, 0x5060),subdrv_i2c_rd_u8(ctx, 0x5061));
	if(ctx->current_scenario_id == 9)
		HDR_Ratio = 160;
	if(0) {  // lcg base
	gain_lcg = rg_gain;
	gain_hcg = (gain_lcg * 1024 * HDR_Ratio) / sensitivity_l / 10; //base lcg
	}
	else { //hcg base
		gain_hcg = rg_gain;
		gain_lcg = gain_hcg * sensitivity_l *10/1024/HDR_Ratio ;
	}
    //DRV_LOG_MUST(ctx, "dcg sensitivity_l:%d rg_gain:%d gain_hcg:%d gain_lcg:%d\n", sensitivity_l,rg_gain,gain_hcg,gain_lcg);
	expo_gain = subdrv_i2c_rd_u8(ctx, 0x3506);
	hcg_maingain = subdrv_i2c_rd_u8(ctx, 0x3546);
	//DRV_LOG_MUST(ctx, "dcg expo_gain:%d hcg_maingain :%d rg_gain:0x%x\n",expo_gain,hcg_maingain,rg_gain);
		if(rg_gain >= 0x400) {
			expo_gain = expo_gain | 0x02;
			hcg_maingain = hcg_maingain | 0x02;
			set_i2c_buffer(ctx,	0x3506,expo_gain);
			set_i2c_buffer(ctx,	0x3546,hcg_maingain);
		}
		else {
			expo_gain = expo_gain & 0xFD;
			hcg_maingain = hcg_maingain & 0xFD;
			set_i2c_buffer(ctx,	0x3506,expo_gain);
			set_i2c_buffer(ctx,	0x3546,hcg_maingain);
		}


		set_i2c_buffer(ctx,	0x3508,(gain_hcg >> 8) & 0xFF);
		set_i2c_buffer(ctx,	0x3509,gain_hcg & 0xFF);
		gain_hcg = gain_hcg * 4;
	if(ctx->current_scenario_id == 8 || ctx->current_scenario_id == 9) {
		set_i2c_buffer(ctx,	0x5019,(gain_hcg >> 16) & 0xFF);
		set_i2c_buffer(ctx,	0x501a,(gain_hcg >> 8) & 0xFF);
		set_i2c_buffer(ctx,	0x501b,gain_hcg & 0xFF);
		set_i2c_buffer(ctx,	0x3548,(gain_lcg >> 8) & 0xFF);
		set_i2c_buffer(ctx,	0x3549,gain_lcg & 0xFF);
		gain_lcg = 4 * gain_lcg;
		set_i2c_buffer(ctx,	0x501c,(gain_lcg >> 16) & 0xFF);
		set_i2c_buffer(ctx,	0x501d,(gain_lcg >>8) & 0xFF);
		set_i2c_buffer(ctx,	0x501e,gain_lcg & 0xFF);
	}
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 0);
	commit_i2c_buffer(ctx);
	return ERROR_NONE;
}
#endif