// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 gc02m1macromipiraw_Sensor.c
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
#include "gc02m1macromipiraw_Sensor.h"

#define LOG_TAG "[gc02m1macro]"
#define LOG_INF(format, args...) pr_info(LOG_TAG "[%s] " format, __func__, ##args)

static int gc02m1macro_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc02m1macro_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc02m1macro_set_shutter_frame_length(struct subdrv_ctx *ctx,u8 *para, u32 *len);
static int gc02m1macro_set_multi_shutter_frame_length(struct subdrv_ctx *ctx,u8 *para, u32 *len);
static int gc02m1macro_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc02m1macro_set_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc02m1macro_set_streaming_suspned(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc02m1macro_set_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc02m1macro_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int gc02m1macro_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static void gc02m1macro_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, gc02m1macro_set_test_pattern},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, gc02m1macro_set_streaming_resume},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, gc02m1macro_set_streaming_suspned},
	{SENSOR_FEATURE_SET_GAIN, gc02m1macro_set_gain},
	{SENSOR_FEATURE_SET_ESHUTTER, gc02m1macro_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, gc02m1macro_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME, gc02m1macro_set_multi_shutter_frame_length},
	{SENSOR_FEATURE_SET_FRAMELENGTH, gc02m1macro_set_frame_length},
	{SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO, gc02m1macro_set_max_framerate_by_scenario},
};

static struct mtk_sensor_saturation_info imgsensor_saturation_info_10bit = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};


static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0580,  // 1408
			.vsize = 0x0420,  // 1056
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0580,  // 1408
			.vsize = 0x0420,  // 1056
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0580,  // 1408
			.vsize = 0x0420,  // 1056
		},
	},
};
/*
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2c,
			.hsize = 0x0640,  // 1600
			.vsize = 0x04B0,  // 1200
		},
	},
};
*/
static struct subdrv_mode_struct mode_struct[] = {

	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = gc02m1macrowide_fullsize_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc02m1macrowide_fullsize_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 84000000,
		.linelength = 2192,
		.framelength = 1276,
		.max_framerate = 300,
		.mipi_pixel_rate = 67200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 1600,
			.full_h = 1200,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 1600,
			.h0_size = 1200,
			.scale_w = 1600,
			.scale_h = 1200,
			.x1_offset = 96,
			.y1_offset = 72,
			.w1_size = 1408,
			.h1_size = 1056,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1408,
			.h2_tg_size = 1056,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 84,
		},
	},

	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = gc02m1macrowide_fullsize_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc02m1macrowide_fullsize_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 84000000,
		.linelength = 2192,
		.framelength = 1276,
		.max_framerate = 300,
		.mipi_pixel_rate = 67200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 1600,
			.full_h = 1200,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 1600,
			.h0_size = 1200,
			.scale_w = 1600,
			.scale_h = 1200,
			.x1_offset = 96,
			.y1_offset = 72,
			.w1_size = 1408,
			.h1_size = 1056,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1408,
			.h2_tg_size = 1056,

		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 84,
		},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = gc02m1macrowide_fullsize_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc02m1macrowide_fullsize_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 84000000,
		.linelength = 2192,
		.framelength = 1276,
		.max_framerate = 300,
		.mipi_pixel_rate = 67200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 1600,
			.full_h = 1200,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 1600,
			.h0_size = 1200,
			.scale_w = 1600,
			.scale_h = 1200,
			.x1_offset = 96,
			.y1_offset = 72,
			.w1_size = 1408,
			.h1_size = 1056,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1408,
			.h2_tg_size = 1056,

		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail  = 84,
		},
	},

	//{
	//	.frame_desc = frame_desc_hs_vid,
	//	.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
	//	.mode_setting_table = gc02m1macrowide_binning_addr_data,
	//	.mode_setting_len = ARRAY_SIZE(gc02m1macrowide_binning_addr_data),
	//	.seamless_switch_group = PARAM_UNDEFINED,
	//	.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
	//	.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
	//	.hdr_mode = HDR_NONE,
	//	.raw_cnt = 1,
	//	.exp_cnt = 2,
	//	.pclk = 84000000,
	//	.linelength = 2192,
	//	.framelength = 1276,
	//	.max_framerate = 300,
	//	.mipi_pixel_rate = 67200000,
	//	.readout_length = 0,
	//	.read_margin = 0,
	//	.imgsensor_winsize_info = {
	//		.full_w = 1600,
	//		.full_h = 1200,
	//		.x0_offset = 0,
	//		.y0_offset = 0,
	//		.w0_size = 1600,
	//		.h0_size = 1200,
	//		.scale_w = 1600,
	//		.scale_h = 1200,
	//		.x1_offset = 0,
	//		.y1_offset = 0,
	//		.w1_size = 1600,
	//		.h1_size = 1200,
	//		.x2_tg_offset = 0,
	//		.y2_tg_offset = 0,
	//		.w2_tg_size = 1600,
	//		.h2_tg_size = 1200,
	//	},
	//	.pdaf_cap = FALSE,
	//	.imgsensor_pd_info = PARAM_UNDEFINED,
	//	.ae_binning_ratio = 1000,
	//	.fine_integ_line = 0,
	//	.delay_frame = 2,
	//	.csi_param = {
	//		.dphy_trail  = 84,
	//	},

	//	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	//},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = GC02M1MACRO_SENSOR_ID,
	.reg_addr_sensor_id = {0xf0, 0xf1},
	.i2c_addr_table = {0x6e, 0xFF},
	.i2c_burst_write_support = FALSE,
	.i2c_transfer_data_type = I2C_DT_ADDR_8_DATA_8,
	.eeprom_info = 0,
	.eeprom_num = 0,
	.resolution = {1408, 1056},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_1_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_type = 4,
	.ana_gain_step = 1,
	.ana_gain_table = gc02m1macro_ana_gain_table,
	.ana_gain_table_size = sizeof(gc02m1macro_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = 0xFFFF - 16,
	.exposure_step = 1,
	.exposure_margin = 16,
	.saturation_info = &imgsensor_saturation_info_10bit,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 0,

	.pdaf_type = PDAF_SUPPORT_NA,
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,
	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = PARAM_UNDEFINED,
	.s_gph = PARAM_UNDEFINED,
	.s_cali = PARAM_UNDEFINED,

	.reg_addr_stream = 0x3e,
	.reg_addr_mirror_flip = PARAM_UNDEFINED,
	.reg_addr_exposure = {{0x03, 0x04},},
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0xb1,0xb2},},
	.reg_addr_frame_length = {{0x41,0x42},},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = PARAM_UNDEFINED,
	.init_setting_table = gc02m1macrowide_init_addr_data,
	.init_setting_len = ARRAY_SIZE(gc02m1macrowide_init_addr_data),

	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.checksum_value = 0xffffffff,
};

static struct subdrv_ops ops = {
	.get_id = gc02m1macro_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,//common_open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_PDN, 0, 0},
	{HW_ID_DOVDD, 1800000, 0},
	{HW_ID_AVDD, 2800000, 3},
	{HW_ID_PDN, 1, 5},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 5},
};

const struct subdrv_entry gc02m1macro_mipi_raw_entry = {
	.name = "gc02m1macro_mipi_raw",
	.id = GC02M1MACRO_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

void gc02m1macro_set_dummy(struct subdrv_ctx *ctx)  //for
{
	LOG_INF("frame_length = %d\n", ctx->frame_length);
	subdrv_i2c_wr_addr8_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0], ctx->frame_length >> 8);
	subdrv_i2c_wr_addr8_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[1], ctx->frame_length & 0xFF);
}	/*	set_dummy  */


static  int gc02m1macro_get_imgsensor_id(struct subdrv_ctx * ctx,u32 * sensor_id)
{
	u8 i = 0;
	u8 retry = 2;
	u32 addr_h = ctx->s_ctx.reg_addr_sensor_id.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_sensor_id.addr[1];

	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = ((subdrv_i2c_rd_addr8_u8(ctx, addr_h) << 8) | subdrv_i2c_rd_addr8_u8(ctx, addr_l)) + 1;
			printk("gc02m1macro i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == GC02M1MACRO_SENSOR_ID) {
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

static void gc02m1macro_set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate, kal_bool min_framelength_en)
{  // for
	/*kal_int16 dummy_line;*/
	kal_uint32 frame_length = ctx->frame_length;
	LOG_INF("framerate = %d, min framelength should enable %d\n", framerate,
		min_framelength_en);
	frame_length = ctx->pclk / framerate * 10 / ctx->line_length;
	if (frame_length >= ctx->min_frame_length)
		ctx->frame_length = frame_length;
	else
		ctx->frame_length = ctx->min_frame_length;
	ctx->dummy_line =
			ctx->frame_length - ctx->min_frame_length;
	if (ctx->frame_length > ctx->s_ctx.frame_length_max) {
		ctx->frame_length = ctx->s_ctx.frame_length_max;
		ctx->dummy_line =
			ctx->frame_length - ctx->min_frame_length;
	}
	if (min_framelength_en)
		ctx->min_frame_length = ctx->frame_length;
	gc02m1macro_set_dummy(ctx);
}	/*	set_max_framerate  */


/* FUNCTION */
static  int gc02m1macro_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 gain = *((u32 *)para);
	//u32 rg_gain;
	u16 reg_gain;
	u32 temp_gain;
	u16 gain_index;
	u16 GC02M1_AGC_Param[GC02M1_SENSOR_GAIN_MAX_VALID_INDEX][2] = {
		{  1024,  0 },
		{  1536,  1 },
		{  2035,  2 },
		{  2519,  3 },
		{  3165,  4 },
		{  3626,  5 },
		{  4147,  6 },
		{  4593,  7 },
		{  5095,  8 },
		{  5697,  9 },
		{  6270, 10 },
		{  6714, 11 },
		{  7210, 12 },
		{  7686, 13 },
		{  8214, 14 },
		{ 10337, 15 },
	};

	LOG_INF("platform_gain = 0x%x \n", gain);
	/* check boundary of gain */
	gain = max(gain, ctx->s_ctx.ana_gain_min);
	gain = min(gain, ctx->s_ctx.ana_gain_max);
	reg_gain = gain * 1024 / BASEGAIN;
	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	ctx->ana_gain[0] = gain;
	for (gain_index = GC02M1_SENSOR_GAIN_MAX_VALID_INDEX - 1; gain_index >= 0; gain_index--)
		if (reg_gain >= GC02M1_AGC_Param[gain_index][0])
			break;
	/* write gain */
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe,0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xb6,GC02M1_AGC_Param[gain_index][1]);

	temp_gain = reg_gain * 1024 / GC02M1_AGC_Param[gain_index][0];
	subdrv_i2c_wr_addr8_u8(ctx, ctx->s_ctx.reg_addr_ana_gain[0].addr[0],(temp_gain >> 8) & 0x1f);
	subdrv_i2c_wr_addr8_u8(ctx, ctx->s_ctx.reg_addr_ana_gain[0].addr[1],temp_gain & 0xff);

	LOG_INF("02m1_reg_gain = 0x%x \n", reg_gain);
	return ERROR_NONE;
}


static  int gc02m1macro_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	return gc02m1macro_set_shutter_frame_length(ctx, para, len);
}

static  int gc02m1macro_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u32 shutter = *feature_data;
	u32 frame_length = *(feature_data + 1);
	u32 fine_integ_line = 0;

	u32 fll = 0;
	u32 fll_step = 0;
	u32 dol_cnt = 1;

	DRV_LOG(ctx, "13a2_shutter = 0x%x \n", shutter);
	DRV_LOG(ctx, "13a2_frame_length = 0x%x \n", frame_length);
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
			subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
			subdrv_i2c_wr_addr8_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0],(fll >> 8) & 0x3F);
			subdrv_i2c_wr_addr8_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[1],fll & 0xFF);
		}
	}
	/* write shutter */
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[0],(ctx->exposure[0] >> 8) & 0x3F);
	subdrv_i2c_wr_addr8_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[1],ctx->exposure[0] & 0xFF);

	DRV_LOG(ctx, "exp[0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		ctx->exposure[0], frame_length, ctx->frame_length, ctx->autoflicker_en);
	return ERROR_NONE;
}

static int gc02m1macro_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u32 *shutters = (u32 *)(*feature_data);
	u16 shutter_cnt = *(feature_data + 1);
	u16 frame_length = *(feature_data + 2);
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	if (shutter_cnt == 1) {
		ctx->shutter = shutters[0];

		/* Change frame time */
		if (frame_length > 1)
			dummy_line = frame_length - ctx->frame_length;
		ctx->frame_length = ctx->frame_length + dummy_line;

		/*  */
		if (shutters[0] > ctx->frame_length - ctx->s_ctx.exposure_margin)
			ctx->frame_length = shutters[0] + ctx->s_ctx.exposure_margin;

		if (ctx->frame_length > ctx->s_ctx.frame_length_max)
			ctx->frame_length = ctx->s_ctx.frame_length_max;

		shutters[0] = (shutters[0] < ctx->s_ctx.exposure_min) ? ctx->s_ctx.exposure_min : shutters[0];
		shutters[0] = (shutters[0] > (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin))
			? (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin) : shutters[0];

		if (ctx->autoflicker_en) {
			realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
			if (realtime_fps >= 593 && realtime_fps <= 607)
				gc02m1macro_set_max_framerate(ctx, 592, 0);
			else if (realtime_fps >= 297 && realtime_fps <= 305)
				gc02m1macro_set_max_framerate(ctx, 296, 0);
			else if (realtime_fps >= 147 && realtime_fps <= 150)
				gc02m1macro_set_max_framerate(ctx, 146, 0);
		}

		/* Update Shutter */
		subdrv_i2c_wr_addr8_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0], ctx->frame_length >> 8);
		subdrv_i2c_wr_addr8_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[1], ctx->frame_length & 0xFF);
		subdrv_i2c_wr_addr8_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[0], (shutters[0] >> 8) & 0xFF);
		subdrv_i2c_wr_addr8_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[1], shutters[0] & 0xFF);
		ctx->frame_length_rg = ctx->frame_length;
		LOG_INF("shutters[0] =%d, framelength =%d\n",shutters[0], ctx->frame_length);
	}
	return ERROR_NONE;
}


static int gc02m1macro_set_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u32 frame_length = *(feature_data);

	if (frame_length)
		ctx->frame_length = frame_length;

	if (ctx->frame_length > ctx->s_ctx.frame_length_max)
		ctx->frame_length = ctx->s_ctx.frame_length_max;
	if (ctx->min_frame_length > ctx->frame_length)
		ctx->frame_length = ctx->min_frame_length;

	subdrv_i2c_wr_addr8_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0], ctx->frame_length >> 8);
	subdrv_i2c_wr_addr8_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[1], ctx->frame_length & 0xFF);
	LOG_INF("Framelength: set=%d/min=%d\n",ctx->frame_length, ctx->min_frame_length);
	return ERROR_NONE;
}




static int gc02m1macro_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	bool enable = mode;

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	if (enable) {
		subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x01);
		subdrv_i2c_wr_addr8_u8(ctx, 0x8c, 0x11);
		subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
	} else {
		subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x01);
		subdrv_i2c_wr_addr8_u8(ctx, 0x8c, 0x10);
		subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
	}
	ctx->test_pattern = enable;

	return ERROR_NONE;
}


static int gc02m1macro_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	enum SENSOR_SCENARIO_ID_ENUM scenario_id = (enum SENSOR_SCENARIO_ID_ENUM)*feature_data;
	u32 framerate = *(feature_data + 1);
	u32 frame_length, calc_fl, exp_cnt, i;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		LOG_INF("invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
	}

	if (framerate == 0) {
		LOG_INF("framerate should not be 0\n");
		return ERROR_NONE;
	}

	if (ctx->s_ctx.mode[scenario_id].linelength == 0) {
		LOG_INF("linelength should not be 0\n");
		return ERROR_NONE;
	}

	if (ctx->line_length == 0) {
		LOG_INF("ctx->line_length should not be 0\n");
		return ERROR_NONE;
	}

	if (ctx->frame_length == 0) {
		LOG_INF("ctx->frame_length should not be 0\n");
		return ERROR_NONE;
	}
	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	calc_fl = ctx->exposure[0];
	for (i = 1; i < exp_cnt; i++)
		calc_fl += ctx->exposure[i];
	calc_fl += ctx->s_ctx.exposure_margin*exp_cnt*exp_cnt;

	frame_length = ctx->s_ctx.mode[scenario_id].pclk / framerate * 10
		/ ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length =
		max(frame_length, ctx->s_ctx.mode[scenario_id].framelength);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	ctx->current_fps = ctx->pclk / ctx->frame_length * 10 / ctx->line_length;
	ctx->min_frame_length = ctx->frame_length;
	LOG_INF("max_fps(input/output):%u/%u(sid:%u), frame_length:%u, calc_fl:%u, min_fl_en:1\n",
		framerate, ctx->current_fps, scenario_id, ctx->frame_length, calc_fl);
	if (ctx->frame_length > calc_fl)
		gc02m1macro_set_dummy(ctx);
	else
		ctx->frame_length = calc_fl;

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

static __maybe_unused void gc02m1macro_sensor_init(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx,"gc02m1macro_sensor_init\n");
	subdrv_i2c_wr_addr8_u8(ctx, 0xfc, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0xf4, 0x41);
	subdrv_i2c_wr_addr8_u8(ctx, 0xf5, 0xe3);
	subdrv_i2c_wr_addr8_u8(ctx, 0xf6, 0x44);
	subdrv_i2c_wr_addr8_u8(ctx, 0xf8, 0x38);
	subdrv_i2c_wr_addr8_u8(ctx, 0xf9, 0x82);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfa, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfd, 0x80);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfc, 0x81);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x03);
	subdrv_i2c_wr_addr8_u8(ctx, 0x01, 0x0b);
	subdrv_i2c_wr_addr8_u8(ctx, 0xf7, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfc, 0x80);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfc, 0x80);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfc, 0x80);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfc, 0x8e);

	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0x87, 0x09);
	subdrv_i2c_wr_addr8_u8(ctx, 0xee, 0x72);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0x8c, 0x90);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0x90, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0x03, 0x04);
	subdrv_i2c_wr_addr8_u8(ctx, 0x04, 0x7d);
	subdrv_i2c_wr_addr8_u8(ctx, 0x41, 0x04);
	subdrv_i2c_wr_addr8_u8(ctx, 0x42, 0xf4);
	subdrv_i2c_wr_addr8_u8(ctx, 0x05, 0x04);
	subdrv_i2c_wr_addr8_u8(ctx, 0x06, 0x48);
	subdrv_i2c_wr_addr8_u8(ctx, 0x07, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0x08, 0x18);
	subdrv_i2c_wr_addr8_u8(ctx, 0x9d, 0x18);
	subdrv_i2c_wr_addr8_u8(ctx, 0x09, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0x0a, 0x02);
	subdrv_i2c_wr_addr8_u8(ctx, 0x0d, 0x04);
	subdrv_i2c_wr_addr8_u8(ctx, 0x0e, 0xbc);
	subdrv_i2c_wr_addr8_u8(ctx, 0x17, 0x80);
	subdrv_i2c_wr_addr8_u8(ctx, 0x19, 0x04);
	subdrv_i2c_wr_addr8_u8(ctx, 0x24, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0x56, 0x20);
	subdrv_i2c_wr_addr8_u8(ctx, 0x5b, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0x5e, 0x01);

	subdrv_i2c_wr_addr8_u8(ctx, 0x21, 0x3c);
	subdrv_i2c_wr_addr8_u8(ctx, 0x44, 0x20);
	subdrv_i2c_wr_addr8_u8(ctx, 0xcc, 0x01);

	subdrv_i2c_wr_addr8_u8(ctx, 0x1a, 0x04);
	subdrv_i2c_wr_addr8_u8(ctx, 0x1f, 0x11);
	subdrv_i2c_wr_addr8_u8(ctx, 0x27, 0x30);
	subdrv_i2c_wr_addr8_u8(ctx, 0x2b, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0x33, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0x53, 0x90);
	subdrv_i2c_wr_addr8_u8(ctx, 0xe6, 0x50);

	subdrv_i2c_wr_addr8_u8(ctx, 0x39, 0x07);
	subdrv_i2c_wr_addr8_u8(ctx, 0x43, 0x04);
	subdrv_i2c_wr_addr8_u8(ctx, 0x46, 0x4a);
	subdrv_i2c_wr_addr8_u8(ctx, 0x7c, 0xa0);
	subdrv_i2c_wr_addr8_u8(ctx, 0xd0, 0xbe);
	subdrv_i2c_wr_addr8_u8(ctx, 0xd1, 0x60);
	subdrv_i2c_wr_addr8_u8(ctx, 0xd2, 0x40);
	subdrv_i2c_wr_addr8_u8(ctx, 0xd3, 0xf3);
	subdrv_i2c_wr_addr8_u8(ctx, 0xde, 0x1d);

	subdrv_i2c_wr_addr8_u8(ctx, 0xcd, 0x05);
	subdrv_i2c_wr_addr8_u8(ctx, 0xce, 0x6f);

	subdrv_i2c_wr_addr8_u8(ctx, 0xfc, 0x88);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x10);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfc, 0x8e);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfc, 0x88);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x10);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfc, 0x8e);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x04);
	subdrv_i2c_wr_addr8_u8(ctx, 0xe0, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);

	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0x53, 0x54);
	subdrv_i2c_wr_addr8_u8(ctx, 0x87, 0x53);
	subdrv_i2c_wr_addr8_u8(ctx, 0x89, 0x03);

	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xb0, 0x74);
	subdrv_i2c_wr_addr8_u8(ctx, 0xb1, 0x04);
	subdrv_i2c_wr_addr8_u8(ctx, 0xb2, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xb6, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x04);
	subdrv_i2c_wr_addr8_u8(ctx, 0xd8, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x40);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x60);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0xc0);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x2a);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x80);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x40);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0xa0);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x90);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x19);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0xc0);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0xD0);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x2F);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0xe0);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x90);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x39);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x20);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x04);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x20);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0xe0);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x0f);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x40);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0xe0);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x1a);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x60);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x20);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x25);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x80);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0xa0);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x2c);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0xa0);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0xe0);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x32);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0xc0);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x20);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x38);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0xe0);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x60);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x3c);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x02);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0xa0);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x40);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x80);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x02);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x18);
	subdrv_i2c_wr_addr8_u8(ctx, 0xc0, 0x5c);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0x9f, 0x10);

	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0x26, 0x20);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0x40, 0x22);
	subdrv_i2c_wr_addr8_u8(ctx, 0x46, 0x7f);
	subdrv_i2c_wr_addr8_u8(ctx, 0x49, 0x0f);
	subdrv_i2c_wr_addr8_u8(ctx, 0x4a, 0xf0);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x04);
	subdrv_i2c_wr_addr8_u8(ctx, 0x14, 0x80);
	subdrv_i2c_wr_addr8_u8(ctx, 0x15, 0x80);
	subdrv_i2c_wr_addr8_u8(ctx, 0x16, 0x80);
	subdrv_i2c_wr_addr8_u8(ctx, 0x17, 0x80);

	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0x41, 0x20);
	subdrv_i2c_wr_addr8_u8(ctx, 0x4c, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0x4d, 0x0c);
	subdrv_i2c_wr_addr8_u8(ctx, 0x44, 0x08);
	subdrv_i2c_wr_addr8_u8(ctx, 0x48, 0x03);

	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0x90, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0x91, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0x92, 0x66);
	subdrv_i2c_wr_addr8_u8(ctx, 0x93, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0x94, 0x4e);
	subdrv_i2c_wr_addr8_u8(ctx, 0x95, 0x04);
	subdrv_i2c_wr_addr8_u8(ctx, 0x96, 0x20);
	subdrv_i2c_wr_addr8_u8(ctx, 0x97, 0x05);
	subdrv_i2c_wr_addr8_u8(ctx, 0x98, 0x80);

	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x03);
	subdrv_i2c_wr_addr8_u8(ctx, 0x01, 0x23);
	subdrv_i2c_wr_addr8_u8(ctx, 0x03, 0xce);
	subdrv_i2c_wr_addr8_u8(ctx, 0x04, 0x48);
	subdrv_i2c_wr_addr8_u8(ctx, 0x15, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0x21, 0x10);
	subdrv_i2c_wr_addr8_u8(ctx, 0x22, 0x05);
	subdrv_i2c_wr_addr8_u8(ctx, 0x23, 0x20);
	subdrv_i2c_wr_addr8_u8(ctx, 0x25, 0x20);
	subdrv_i2c_wr_addr8_u8(ctx, 0x26, 0x08);
	subdrv_i2c_wr_addr8_u8(ctx, 0x29, 0x06);
	subdrv_i2c_wr_addr8_u8(ctx, 0x2a, 0x0a);
	subdrv_i2c_wr_addr8_u8(ctx, 0x2b, 0x08);

	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x01);
	subdrv_i2c_wr_addr8_u8(ctx, 0x8c, 0x10);
	subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
	subdrv_i2c_wr_addr8_u8(ctx, 0x3e, 0x90);

	LOG_INF("gc02m1macro_sensor_init end\n");
}

static __maybe_unused int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (gc02m1macro_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	gc02m1macro_sensor_init(ctx);

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

	return ERROR_NONE;
} /* open */

static void gc02m1macro_set_streaming_control(struct subdrv_ctx *ctx, bool enable)
{
		struct adaptor_ctx *_adaptor_ctx = NULL;
		struct v4l2_subdev *sd = NULL;

		LOG_INF("E! enable:%u\n", enable);

		if (ctx->i2c_client)
			sd = i2c_get_clientdata(ctx->i2c_client);
		if (sd)
			_adaptor_ctx = to_ctx(sd);
		if (!_adaptor_ctx) {
			LOG_INF("null _adaptor_ctx\n");
			return;
		}

		check_current_scenario_id_bound(ctx);
		if (ctx->s_ctx.aov_sensor_support && ctx->s_ctx.streaming_ctrl_imp) {
			if (ctx->s_ctx.s_streaming_control != NULL)
				ctx->s_ctx.s_streaming_control((void *) ctx, enable);
			else
				LOG_INF("please implement drive own streaming control!(sid:%u)\n",
					ctx->current_scenario_id);
			ctx->is_streaming = enable;
			LOG_INF("enable:%u\n", enable);
			return;
		}
		if (ctx->s_ctx.aov_sensor_support && ctx->s_ctx.mode[ctx->current_scenario_id].aov_mode) {
			LOG_INF("stream ctrl implement on scp side!(sid:%u)\n",
				ctx->current_scenario_id);
			ctx->is_streaming = enable;
			LOG_INF("enable:%u\n", enable);
			return;
		}

		if (enable) {
			subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
			subdrv_i2c_wr_addr8_u8(ctx, 0x3e, 0x90);
		} else {
			subdrv_i2c_wr_addr8_u8(ctx, 0xfe, 0x00);
			subdrv_i2c_wr_addr8_u8(ctx, 0x3e, 0x00);
			if (ctx->s_ctx.reg_addr_fast_mode && ctx->fast_mode_on) {
				ctx->fast_mode_on = FALSE;
				ctx->ref_sof_cnt = 0;
				LOG_INF("seamless_switch disabled.");
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
		mdelay(10);
		LOG_INF("wenhuitest reg_addr_stream enable=%d 0x3e = 0x%x",enable,subdrv_i2c_rd_addr8_u8(ctx, 0x3e));
		ctx->sof_no = 0;
		ctx->is_streaming = enable;
		LOG_INF("X! enable:%u\n", enable);
}

static int gc02m1macro_set_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	gc02m1macro_set_streaming_control(ctx, TRUE);
	return 0;
}
static int gc02m1macro_set_streaming_suspned(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	gc02m1macro_set_streaming_control(ctx, FALSE);
	return 0;
}
