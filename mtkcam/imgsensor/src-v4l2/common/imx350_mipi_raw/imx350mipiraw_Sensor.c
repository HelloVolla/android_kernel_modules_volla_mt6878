// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 imx350mipiraw_Sensor.c
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
#include "imx350mipiraw_Sensor.h"



//static int get_sensor_temperature(void *arg);
//static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int imx350_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx350_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
//static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
//extern int is_yft_cts_board(void);
/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, imx350_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, imx350_set_test_pattern_data},
};

#if 0
static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x010B00FF,
		.addr_header_id = 0x0000000B,
		.i2c_write_id = 0xA9,
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 17,
	.i4OffsetY = 12,
	.i4PitchX = 8,
	.i4PitchY = 16,
	.i4PairNum = 8,
	.i4SubBlkW = 8,
	.i4SubBlkH = 2,
	.i4PosL = {{20, 13}, {18, 15}, {22, 17}, {24, 19},
	           {20, 21}, {18, 23}, {22, 25}, {24, 27}},
	.i4PosR = {{19, 13}, {17, 15}, {21, 17}, {23, 19},
	           {19, 21}, {17, 23}, {21, 25}, {23, 27}},
	.i4BlockNumX = 496,
	.i4BlockNumY = 186,
	.i4LeFirst = 0,
	.i4Crop = {
		// <prev> <cap> <vid> <hs_vid> <slim_vid>
		{0, 0}, {0, 0}, {0, 372}, {0, 372}, {0, 372},
		// <cust1> <<cust2>> <<cust3>>
		{0, 0}, {0, 0}, {0, 0},
	},
	.iMirrorFlip = 0,

	.sPDMapInfo[0] = {
		.i4PDPattern = 2,
		.i4PDRepetition = 4,
		.i4PDOrder = {1}, // R = 1, L = 0
	},
};
#endif
static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0A20,
			.vsize = 0x0790,
			.user_data_desc = VC_STAGGER_NE,
		},
	},


};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1440,
			.vsize = 0x0F28,
			.user_data_desc = VC_STAGGER_NE,
		},
	},


};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1440,
			.vsize = 0x0B64,
			.user_data_desc = VC_STAGGER_NE,
		},
	},

};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0500,
			.vsize = 0x02D0,
			.user_data_desc = VC_STAGGER_NE,
		},
	},

};

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0A20,
			.vsize = 0x0790,
			.user_data_desc = VC_STAGGER_NE,
		},
	},

};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = imx350_preview_setting,
		.mode_setting_len = ARRAY_SIZE(imx350_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 420000000,
		.linelength = 6648,
		.framelength = 2104,
		.max_framerate = 300,
		.mipi_pixel_rate = 319200000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 6,
		.imgsensor_winsize_info = {
			.full_w = 5184,
			.full_h = 3880,
			.x0_offset = 0,
			.y0_offset = 4,
			.w0_size = 5184,
			.h0_size = 3872,
			.scale_w = 2592,
			.scale_h = 1936,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2592,
			.h1_size = 1936,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2592,
			.h2_tg_size = 1936,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = NULL,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 634,
		.delay_frame = 3,
		.csi_param = {
		},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_MONO,
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = imx350_capture_setting,
		.mode_setting_len = ARRAY_SIZE(imx350_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 823200000,
		.linelength = 6720,
		.framelength = 4080,
		.max_framerate = 300,
		.mipi_pixel_rate = 673920000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 6,
		.imgsensor_winsize_info = {
			.full_w = 5184,
			.full_h = 3880,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 5184,
			.h0_size = 3880,
			.scale_w = 5184,
			.scale_h = 3880,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 5184,
			.h1_size = 3880,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 5184,
			.h2_tg_size = 3880,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = NULL,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 634,
		.delay_frame = 3,
		.csi_param = {
		},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_MONO,
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = imx350_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(imx350_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 801600000,
		.linelength = 8704,
		.framelength = 3068,
		.max_framerate = 300,
		.mipi_pixel_rate = 673920000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 5,
		.imgsensor_winsize_info = {
			.full_w = 5184,
			.full_h = 3880,
			.x0_offset = 0,
			.y0_offset = 482,
			.w0_size = 5184,
			.h0_size = 2916,
			.scale_w = 5184,
			.scale_h = 2916,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 5184,
			.h1_size = 2916,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 5184,
			.h2_tg_size = 2916,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = NULL,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 357,
		.delay_frame = 3,
		.csi_param = {
		},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_MONO,
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = imx350_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(imx350_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 595000000,
		.linelength = 6024,
		.framelength = 828,
		.max_framerate = 1200,
		.mipi_pixel_rate = 201600000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 5,
		.imgsensor_winsize_info = {
			.full_w = 5184,
			.full_h = 3880,
			.x0_offset = 672,
			.y0_offset = 860,
			.w0_size = 3840,
			.h0_size = 2160,
			.scale_w = 1280,
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
		.imgsensor_pd_info = NULL,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 357,
		.delay_frame = 3,
		.csi_param = {
		},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_MONO,
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = imx350_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(imx350_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 420000000,
		.linelength = 6648,
		.framelength = 2104,
		.max_framerate = 300,
		.mipi_pixel_rate = 319200000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 5,
		.imgsensor_winsize_info = {
			.full_w = 5184,
			.full_h = 3880,
			.x0_offset = 0,
			.y0_offset = 4,
			.w0_size = 5184,
			.h0_size = 3872,
			.scale_w = 2592,
			.scale_h = 1936,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2592,
			.h1_size = 1936,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2592,
			.h2_tg_size = 1936,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = NULL,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 357,
		.delay_frame = 3,
		.csi_param = {
		},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_MONO,
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = IMX350_SENSOR_ID,
	.reg_addr_sensor_id = {0x0016, 0x0017},
	.i2c_addr_table = {0x20, 0x34, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	//.eeprom_info = eeprom_info,
	//.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {2592, 1940},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_MONO,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = 1150,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 0,//0:sony, 1:ov or samusng....etc no used
	.ana_gain_step = 1,// no used
	.ana_gain_table = imx350_ana_gain_table,
	.ana_gain_table_size = sizeof(imx350_ana_gain_table),
	.min_gain_iso = 100, // no change
	.exposure_def = 0x3D0, //no change
	.exposure_min = 16,
	.exposure_max = 128 * (0xFFFF - 48),
	.exposure_step = 2, // Get Maximum Step
	.exposure_margin = 48,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = BASE_DGAIN * 16,
	.dig_gain_step = 4,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,///SHUTTER AND GAIN N+1 (long expose used)
	.frame_time_delay_frame = 3,
	.start_exposure_offset = 6200000,

	.pdaf_type = PDAF_SUPPORT_NA,
	.hdr_type = HDR_SUPPORT_NA,  //check this sensor is stagger or not.
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,

	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = get_gain2reg,
	//.s_gph = set_group_hold,
	.s_cali = NULL,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {
			{0x0202, 0x0203},
	},
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = 0x3100,
	.reg_addr_ana_gain = {
			{0x0204, 0x0205},
	},
	#if 0
	.reg_addr_dig_gain = {
			{0x020E, 0x020F},
	},
	#endif
	.reg_addr_frame_length = {0x0340, 0x0341},
	//.reg_addr_temp_en = 0x0138,
	//.reg_addr_temp_read = 0x013A,
	.reg_addr_auto_extend = 0x0350,
	.reg_addr_frame_count = 0x0005,

	.init_setting_table = imx350_init_setting,
	.init_setting_len = ARRAY_SIZE(imx350_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 0,
	.chk_s_off_end = 0,

	.checksum_value = 0xecaae2a0,


};

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
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	//.vsync_notify = vsync_notify,
	.update_sof_cnt = common_update_sof_cnt,
	.parse_ebd_line = common_parse_ebd_line,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_PDN, 0, 0},
	{HW_ID_RST, 0, 0},
	{HW_ID_AVDD, 2800000, 0}, // pmic_ldo for avdd
	{HW_ID_DOVDD, 1800000, 0}, // pmic_ldo/gpio(1.8V ldo) for dovdd
	{HW_ID_DVDD, 1100000, 5}, // pmic_ldo for dvdd
	{HW_ID_AFVDD, 2800000,0},
	{HW_ID_MCLK, 24, 5},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 1},
	{HW_ID_PDN, 1, 0},
	{HW_ID_RST, 1, 5},
};

const struct subdrv_entry imx350_mipi_raw_entry = {
	.name = "imx350_mipi_raw",
	.id = IMX350_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */
#if 0
static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u8 temperature = 0;
	int temperature_convert = 0;

	temperature = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);

	if (temperature < 0x50)
		temperature_convert = temperature;
	else if (temperature < 0x80)
		temperature_convert = 80;
	else if (temperature < 0xED)
		temperature_convert = -20;
	else
		temperature_convert = (char)temperature;

	DRV_LOG_MUST(ctx, "temperature: %d degrees\n", temperature_convert);
	return temperature_convert;
}

static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	if (en)
		set_i2c_buffer(ctx, 0x0104, 0x01);
	else
		set_i2c_buffer(ctx, 0x0104, 0x00);
}
#endif
static u16 get_gain2reg(u32 gain)
{
	/*the below formula is unconditional carry */
	return (1024 - (1024 * BASEGAIN + (gain >> 1)) / gain);
}

static int imx350_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG_MUST(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	if (mode)
		subdrv_i2c_wr_u8(ctx, 0x0601, mode); /*100% Color bar*/
	else if (ctx->test_pattern)
		subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /*No pattern*/

	ctx->test_pattern = mode;
	return ERROR_NONE;
}


static int imx350_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct mtk_test_pattern_data *data = (struct mtk_test_pattern_data *)para;
	u16 R = (data->Channel_R >> 22) & 0x3ff;
	u16 Gr = (data->Channel_Gr >> 22) & 0x3ff;
	u16 Gb = (data->Channel_Gb >> 22) & 0x3ff;
	u16 B = (data->Channel_B >> 22) & 0x3ff;

	subdrv_i2c_wr_u8(ctx, 0x0602, (R >> 8));
	subdrv_i2c_wr_u8(ctx, 0x0603, (R & 0xff));
	subdrv_i2c_wr_u8(ctx, 0x0604, (Gr >> 8));
	subdrv_i2c_wr_u8(ctx, 0x0605, (Gr & 0xff));
	subdrv_i2c_wr_u8(ctx, 0x0606, (B >> 8));
	subdrv_i2c_wr_u8(ctx, 0x0607, (B & 0xff));
	subdrv_i2c_wr_u8(ctx, 0x0608, (Gb >> 8));
	subdrv_i2c_wr_u8(ctx, 0x0609, (Gb & 0xff));

	DRV_LOG_MUST(ctx, "mode(%u) R/Gr/Gb/B = 0x%04x/0x%04x/0x%04x/0x%04x\n",
		ctx->test_pattern, R, Gr, Gb, B);
	return ERROR_NONE;
}



static kal_uint32 return_lot_id_from_otp(struct subdrv_ctx *ctx)
{
	kal_uint16 val = 0;
	int i = 0;

	if (subdrv_i2c_wr_u8(ctx, 0x0a02, 0x27) < 0) {
		DRV_LOG_MUST(ctx, "read otp fail Err!\n");
		return 0;
	}
	subdrv_i2c_wr_u8(ctx, 0x0a00, 0x01);

	for (i = 0; i < 3; i++) {
		val = subdrv_i2c_rd_u8(ctx, 0x0A01);
		if ((val & 0x01) == 0x01)
			break;
		mDELAY(3);
	}
	if (i == 3) {
		DRV_LOG_MUST(ctx, "read otp fail Err!\n");	/* print log */
		return 0;
	}
	/* DRV_LOG_MUST(ctx, "0x0A38 0x%x 0x0A39 0x%x\n",
	 * read_cmos_sensor(ctx, 0x0A38)<<4,read_cmos_sensor(ctx, 0x0A39)>>4);
	 */
	return ((subdrv_i2c_rd_u8(ctx, 0x0A22) << 4) |
					subdrv_i2c_rd_u8(ctx, 0x0A23) >> 4);
}


static int get_imgsensor_id(struct subdrv_ctx *ctx, UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 * we should detect the module used i2c address
	 */
	while (static_ctx.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = static_ctx.i2c_addr_table[i];
		do {
			*sensor_id = return_lot_id_from_otp(ctx);
			/* return_sensor_id(); */
			if (*sensor_id == static_ctx.sensor_id) {
				//imx350_read_SPC(ctx, imx350_SPC_data);
				DRV_LOG_MUST(ctx, "i2c write id: 0x%x, sensor id: 0x%x\n",
					ctx->i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			DRV_LOG_MUST(ctx, 
				"Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				ctx->i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != static_ctx.sensor_id) {
	/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

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

	return ERROR_NONE;
}
#if 0
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
		set_i2c_buffer(ctx, 0x3010, 0x00);
		commit_i2c_buffer(ctx);
	}
	return 0;
}
#endif
