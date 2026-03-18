// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 gc32e1submipiraw_Sensor.c
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
#include "gc32e1submipiraw_Sensor.h"

static int gc32e1sub_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc32e1sub_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc32e1sub_set_shutter_frame_length(struct subdrv_ctx *ctx,u8 *para, u32 *len);
static int gc32e1sub_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static void gc32e1sub_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, gc32e1sub_set_test_pattern},
	{SENSOR_FEATURE_SET_GAIN, gc32e1sub_set_gain},
	{SENSOR_FEATURE_SET_ESHUTTER, gc32e1sub_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, gc32e1sub_set_shutter_frame_length},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,  // 3264
			.vsize = 0x0990,  // 2448
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,  // 3264
			.vsize = 0x0990,  // 2448
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,  // 3264
			.vsize = 0x0990,  // 2448
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2c,
			.hsize = 0x0CC0,  // 3264
			.vsize = 0x0990,  // 2448
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {

	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = gc32e1subwide_binning_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc32e1subwide_binning_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 345600000,
		.linelength = 4276,
		.framelength = 2656,
		.max_framerate = 300,
		.mipi_pixel_rate = 305280000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 85,
		},
	},

	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = gc32e1subwide_binning_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc32e1subwide_binning_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 345600000,
		.linelength = 4276,
		.framelength = 2656,
		.max_framerate = 300,
		.mipi_pixel_rate = 305280000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 85,
		},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = gc32e1subwide_binning_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc32e1subwide_binning_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 345600000,
		.linelength = 4276,
		.framelength = 2656,
		.max_framerate = 300,
		.mipi_pixel_rate = 305280000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail  = 85,
		},
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = gc32e1subwide_binning_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc32e1subwide_binning_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 2,
		.pclk = 345600000,
		.linelength = 4276,
		.framelength = 2656,
		.max_framerate = 300,
		.mipi_pixel_rate = 305280000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail  = 85,
		},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = GC32E1SUB_SENSOR_ID,
	.reg_addr_sensor_id = {0x03F0, 0x03F1},
	.i2c_addr_table = {0x94, 0xff},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = 0,
	.eeprom_num = 0,
	.resolution = {3264, 2448},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 32,
	.ana_gain_type = 4,
	.ana_gain_step = 4,
	.ana_gain_table = gc32e1sub_ana_gain_table,
	.ana_gain_table_size = sizeof(gc32e1sub_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = 0xFFFF - 16,
	.exposure_step = 1,
	.exposure_margin = 16,

	.frame_length_max = 0xFFFE,
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

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = PARAM_UNDEFINED,
	.reg_addr_exposure = {{0x0202},},
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x0204},},
	.reg_addr_frame_length = {0x0340,},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = PARAM_UNDEFINED,
	.init_setting_table = PARAM_UNDEFINED,
	.init_setting_len = PARAM_UNDEFINED,

	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.checksum_value = 0xffffffff,
};

static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
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
	{HW_ID_DVDD1, 1200000, 5}, //GPIO37
	{HW_ID_MCLK, 24, 0},
	{HW_ID_PDN, 0, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 5},
	{HW_ID_DOVDD, 1800000, 0},
	{HW_ID_DVDD, 1200000, 0},
	{HW_ID_AVDD, 2800000, 5},
	{HW_ID_PDN, 1, 0},
	{HW_ID_RST, 1, 5},
};

const struct subdrv_entry gc32e1sub_mipi_raw_entry = {
	.name = "gc32e1sub_mipi_raw",
	.id = GC32E1SUB_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};
static __maybe_unused int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = 2;
	u32 addr_h = ctx->s_ctx.reg_addr_sensor_id.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_sensor_id.addr[1];

	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = ((subdrv_i2c_rd_u8(ctx, addr_h) << 8) | subdrv_i2c_rd_u8(ctx, addr_l)) + 1;
			printk("gc32e1sub i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == GC32E1SUB_SENSOR_ID) {
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

/* FUNCTION */

static __maybe_unused int gc32e1sub_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 gain = *((u32 *)para);
	u32 rg_gain;

	DRV_LOG(ctx, "platform_gain = 0x%x \n", gain);
	/* check boundary of gain */
	gain = max(gain, ctx->s_ctx.ana_gain_min);
	gain = min(gain, ctx->s_ctx.ana_gain_max);
	rg_gain = gain * 1024 / BASEGAIN;
	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	ctx->ana_gain[0] = gain;
	/* write gain */
	subdrv_i2c_wr_u16(ctx, ctx->s_ctx.reg_addr_ana_gain[0].addr[0],rg_gain & 0xffff);

	DRV_LOG(ctx, "gc32e1_rg_gain = 0x%x \n", rg_gain);
	return ERROR_NONE;
}

static __maybe_unused int gc32e1sub_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	return gc32e1sub_set_shutter_frame_length(ctx, para, len);
}

static __maybe_unused int gc32e1sub_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u32 shutter = *feature_data;
	u32 frame_length = *(feature_data + 1);
	u32 fine_integ_line = 0;

	DRV_LOG(ctx, "gc32e1_shutter = 0x%x \n", shutter);
	DRV_LOG(ctx, "gc32e1_frame_length = 0x%x \n", frame_length);
	ctx->frame_length = frame_length ? frame_length : ctx->frame_length;
	check_current_scenario_id_bound(ctx);
	/* check boundary of framelength */
	ctx->frame_length =	max(shutter + ctx->s_ctx.exposure_margin, ctx->min_frame_length);
	ctx->frame_length =	min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	/* check boundary of shutter */
	fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	shutter = FINE_INTEG_CONVERT(shutter, fine_integ_line);
	shutter = max(shutter, ctx->s_ctx.exposure_min);
	shutter = min(shutter, ctx->s_ctx.exposure_max);
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	ctx->exposure[0] = shutter;
	/* write framelength */
	subdrv_i2c_wr_u16(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0],ctx->frame_length & 0xfffe);
	/* write shutter */
	subdrv_i2c_wr_u16(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[0],ctx->exposure[0] & 0xffff);

	DRV_LOG(ctx, "exp[0x%x], fll(input/output):%u/%u\n",ctx->exposure[0], frame_length, ctx->frame_length);

	return ERROR_NONE;
}

static int gc32e1sub_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	bool enable = mode;

	if (enable != ctx->test_pattern) {
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, enable);

		if (enable) {
		    subdrv_i2c_wr_u8(ctx, 0x008c, 0x01);
		    subdrv_i2c_wr_u8(ctx, 0x008d, 0x00);
		} else {
		    subdrv_i2c_wr_u8(ctx, 0x008c, 0x00);
		    subdrv_i2c_wr_u8(ctx, 0x008d, 0x10);
		}
		ctx->test_pattern = enable;
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
static __maybe_unused void gc32e1sub_sensor_init(struct subdrv_ctx *ctx)
{

	DRV_LOG(ctx,"gc32e1sub_sensor_init\n");

	subdrv_i2c_wr_u8(ctx, 0x0315, 0xc0);
	subdrv_i2c_wr_u8(ctx, 0x0c10, 0x1b);
	subdrv_i2c_wr_u8(ctx, 0x01a7, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x01aa, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x01a8, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x0c0d, 0xb4);
	subdrv_i2c_wr_u8(ctx, 0x0185, 0xc0);
	subdrv_i2c_wr_u8(ctx, 0x0314, 0x11);
	subdrv_i2c_wr_u8(ctx, 0x031a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x01a1, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x01e3, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0057, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x00a6, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x00d3, 0x30);
	subdrv_i2c_wr_u8(ctx, 0x0087, 0x51);
	subdrv_i2c_wr_u8(ctx, 0x01e2, 0x24);
	subdrv_i2c_wr_u8(ctx, 0x01ea, 0x24);
	subdrv_i2c_wr_u8(ctx, 0x0219, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x0c08, 0x19);
	subdrv_i2c_wr_u8(ctx, 0x0c05, 0xff);
	subdrv_i2c_wr_u8(ctx, 0x0c07, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x0c41, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x0c45, 0xdf);
	subdrv_i2c_wr_u8(ctx, 0x0e15, 0x58);
	subdrv_i2c_wr_u8(ctx, 0x0e6c, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0e6d, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x03a2, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0313, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0a53, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0a65, 0x17);
	subdrv_i2c_wr_u8(ctx, 0x0a68, 0x33);
	subdrv_i2c_wr_u8(ctx, 0x0a58, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0a4f, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0a66, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0a7f, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x0a84, 0x0c);

	DRV_LOG(ctx,"gc32e1sub_sensor_init end\n");
}
static __maybe_unused int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	gc32e1sub_sensor_init(ctx);

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
