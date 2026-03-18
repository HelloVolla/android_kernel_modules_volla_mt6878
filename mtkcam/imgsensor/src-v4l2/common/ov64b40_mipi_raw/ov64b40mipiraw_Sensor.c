// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 ov64b40mipiraw_Sensor.c
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
#include "ov64b40mipiraw_Sensor.h"

static void set_sensor_cali(void *arg);
static int get_sensor_temperature(void *arg);
static void set_group_hold(void *arg, u8 en);
//static void ov64b40_set_dummy(struct subdrv_ctx *ctx);
static u16 get_gain2reg(u32 gain);
static int  ov64b40_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, ov64b40_set_test_pattern},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x010B00FF,
		.addr_header_id = 0x0000000B,
		.i2c_write_id = 0xAA,

		.pdc_support = false,
		.pdc_size = 720,
		.addr_pdc = 0x1A46,
		.sensor_reg_addr_pdc = 0x5F80,

		.xtalk_support = false,
		.xtalk_size = 288,
		.addr_xtalk = 0x1D31,
		.sensor_reg_addr_xtalk = 0x5A40,
	},
};


// mode 0: 4624*3472@30fps, normal preview + PD
static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1210,
			.vsize = 0x0d90,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	// prize add by chenwenhui for pdaf start
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0470,
			.vsize = 0x035c,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
	// prize add by chenwenhui for pdaf end
};

// mode 1: same as preview mode + PD
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1210,
			.vsize = 0x0d90,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	// prize add by chenwenhui for pdaf start
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0470,
			.vsize = 0x035c,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
	// prize add by chenwenhui for pdaf end
};

// mode 2: 4624*3472@30fps, noraml video + PD
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1210,
			.vsize = 0x0d90,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	// prize add by chenwenhui for pdaf start
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0470,
			.vsize = 0x035c,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
	// prize add by chenwenhui for pdaf end
};

// mode 3: 1920*1080@120fps, non-pd
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1210,
			.vsize = 0x0a2c,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

// mode 4: 1920*1080@60fps, non-pd
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1210,
			.vsize = 0x0a2c,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

// prize add by chenwenhui for pdaf start
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 40,
	.i4OffsetY = 16,
	.i4PitchX = 16,
	.i4PitchY = 16,
	.i4PairNum = 8,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4PosL = {{47, 18}, {55, 18}, {43, 22}, {51, 22},
		{47, 26}, {55, 26}, {43, 30}, {51, 30} },
	.i4PosR = {{46, 18}, {54, 18}, {42, 22}, {50, 22},
		{46, 26}, {54, 26}, {42, 30}, {50, 30} },
	.i4BlockNumX = 284,
	.i4BlockNumY = 215,
	.i4LeFirst = 0,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
	},
	.iMirrorFlip = 3,
	.i4FullRawW = 4624,
	.i4FullRawH = 3472,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0,

	.sPDMapInfo[0] = {
	    .i4VCFeature = VC_PDAF_STATS,
	    .i4PDPattern = 2,
	    .i4PDRepetition = 2,
	    .i4PDOrder = {1}, // R = 1, L = 0
	},
};
// prize add by chenwenhui for pdaf end


static struct subdrv_mode_struct mode_struct[] = {
	// mode 0: 4624*3472@30fps, normal preview + pd
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = ov64b40_preview_setting,
		.mode_setting_len = ARRAY_SIZE(ov64b40_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 1056,
		.framelength = 3636,
		.max_framerate = 300,
		.mipi_pixel_rate = 600800000,
		.readout_length = 0,
		.read_margin = 10,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 9248,
			.h0_size = 6944,
			.scale_w = 4624,
			.scale_h = 3472,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4624,
			.h1_size = 3472,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4624,
			.h2_tg_size = 3472,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1467,
		.fine_integ_line = 0,
		.delay_frame = 1,
		.csi_param = {
			.dphy_trail = 72, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = false,
	},
	// mode 1: same as preview mode
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = ov64b40_capture_setting,
		.mode_setting_len = ARRAY_SIZE(ov64b40_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 1056,
		.framelength = 3636,
		.max_framerate = 300,
		.mipi_pixel_rate = 600800000,
		.readout_length = 0,
		.read_margin = 10,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 9248,
			.h0_size = 6944,
			.scale_w = 4624,
			.scale_h = 3472,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4624,
			.h1_size = 3472,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4624,
			.h2_tg_size = 3472,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1467,
		.fine_integ_line = 0,
		.delay_frame = 1,
		.csi_param = {
			.dphy_trail = 72, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = false,
	},
	// mode 2: 4624*3472@30fps, noraml video + pd
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = ov64b40_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(ov64b40_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 1056,
		.framelength = 3636,
		.max_framerate = 300,
		.mipi_pixel_rate = 600800000,
		.readout_length = 0,
		.read_margin = 10,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 9248,
			.h0_size = 6944,
			.scale_w = 4624,
			.scale_h = 3472,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4624,
			.h1_size = 3472,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4624,
			.h2_tg_size = 3472,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1467,
		.fine_integ_line = 0,
		.delay_frame = 1,
		.csi_param = {
			.dphy_trail = 72, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = false,
	},
	// mode 3: 1920*1080@120fps, m-stream
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = ov64b40_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(ov64b40_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 504,
		.framelength = 1904,
		.max_framerate = 1200,
		.mipi_pixel_rate = 600960000,
		.readout_length = 0,
		.read_margin = 10,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 784,
			.y0_offset = 1312,
			.w0_size = 7680,
			.h0_size = 4320,
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
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 2940,
		.fine_integ_line = 0,
		.delay_frame = 1,
		.csi_param = {
			.dphy_trail = 72, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = false,
	},
	// mode 4: 1920*1080@60fps, non-pd
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = ov64b40_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(ov64b40_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 504,
		.framelength = 3808,
		.max_framerate = 600,
		.mipi_pixel_rate = 600960000,
		.readout_length = 0,
		.read_margin = 10,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 784,
			.y0_offset = 1312,
			.w0_size = 7680,
			.h0_size = 4320,
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
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 2940,
		.fine_integ_line = 0,
		.delay_frame = 1,
		.csi_param = {
			.dphy_trail = 72, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = false,
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = OV64B40_SENSOR_ID,
	.reg_addr_sensor_id = {0x300A, 0x300B, 0x300C},
	.i2c_addr_table = {0x20,0x6c, 0xFF}, // TBD
	// .i2c_addr_table = {0x48, 0xFF}, // disable 64b
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {9248, 6944},
	.mirror = IMAGE_HV_MIRROR, // TBD

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_CSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 15.5,
	.ana_gain_type = 1,
	.ana_gain_step = 4,
	.ana_gain_table = ov64b40_ana_gain_table,
	.ana_gain_table_size = sizeof(ov64b40_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 16,
	.exposure_max = 0xFFFFFF - 36,
	.exposure_step = 2,
	.exposure_margin = 36,

	.frame_length_max = 0xFFFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 0,

	.pdaf_type = PDAF_SUPPORT_CAMSV,
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = FALSE,
	.temperature_support = TRUE,

	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,
	.s_cali = set_sensor_cali,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = PARAM_UNDEFINED, // TBD
	.reg_addr_exposure = {{0x3500, 0x3501, 0x3502},
				{0x3580, 0x3581, 0x3582},
				{0x3540, 0x3541, 0x3542}},
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x3508, 0x3509}, {0x3588, 0x3589}, {0x3548, 0x3549}},
	.reg_addr_frame_length = {0x3840, 0x380E, 0x380F},
	.reg_addr_temp_en = 0x4D12,
	.reg_addr_temp_read = 0x4D13,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = 0x387f, // To be verified

	.init_setting_table = PARAM_UNDEFINED,
	.init_setting_len = PARAM_UNDEFINED,
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 0,
	.chk_s_off_end = 0,

	.checksum_value = 0xffffffff,
};

//drv add by lipengpeng 20240601 start
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
			*sensor_id +=1;
			printk("OV64B40 i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == OV64B40_SENSOR_ID) {
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

static int ov64b40_sensor_init(struct subdrv_ctx *ctx)
{
	printk("ov64b40_sensor_init start\n");
	
	i2c_table_write(ctx, sensor_init_addr_data, sizeof(sensor_init_addr_data)/sizeof(u16));
	
	printk("ov64b40_sensor_init end \n");

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
	ov64b40_sensor_init(ctx);

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

	printk("ov64b40 open end\n");
	return ERROR_NONE;
} /* open */


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
	.update_sof_cnt = common_update_sof_cnt,
	.vsync_notify = vsync_notify,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_DVDD1, 1200000, 5}, //GPIO37
	{HW_ID_RST, 0, 1},
	{HW_ID_PDN, 0, 1},
	{HW_ID_AVDD, 2800000, 1}, // pmic_ldo for avdd
	{HW_ID_DOVDD, 1800000, 3}, // pmic_ldo/gpio(1.8V ldo) for dovdd
	{HW_ID_DVDD, 1100000, 1}, // pmic_ldo for dvdd
	{HW_ID_PDN, 1, 1},
	{HW_ID_MCLK, 24, 0},
	{HW_ID_MCLK_DRIVING_CURRENT, 2, 0},
	{HW_ID_RST, 1, 2}
};

const struct subdrv_entry ov64b40_mipi_raw_entry = {
	.name = "ov64b40_mipi_raw",
	.id = OV64B40_SENSOR_ID,
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
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	if (!probe_eeprom(ctx))
		return;

	idx = ctx->eeprom_index;

	/* PDC data */
	support = info[idx].pdc_support;
	if (support) {
		pbuf = info[idx].preload_pdc_table;
		if (pbuf != NULL) {
			size = 720;
			addr = 0x5F80;
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
			DRV_LOG(ctx, "set PDC calibration data done.");
		}
	}

	/* xtalk data */
	support = info[idx].xtalk_support;
	if (support) {
		pbuf = info[idx].preload_xtalk_table;
		if (pbuf != NULL) {
			size = 288;
			addr = 0x5A40;
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
			DRV_LOG(ctx, "set xtalk calibration data done.");
		}
	}
}

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	int temperature = 0;

	/*TEMP_SEN_CTL */
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_temp_en, 0x01);
	temperature = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);
	temperature = (temperature > 0xC0) ? (temperature - 0x100) : temperature;

	DRV_LOG(ctx, "temperature: %d degrees\n", temperature);
	return temperature;
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

//static void ov64b40_set_dummy(struct subdrv_ctx *ctx)
//{
	// bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);

	// if (gph)
	// ctx->s_ctx.s_gph((void *)ctx, 1);
//	write_frame_length(ctx, ctx->frame_length);
	// if (gph)
	// ctx->s_ctx.s_gph((void *)ctx, 0);

//	commit_i2c_buffer(ctx);
//}

static u16 get_gain2reg(u32 gain)
{
	return gain * 256 / BASEGAIN;
}
static int ov64b40_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	bool enable = mode;

	printk("ov64b40_set_test_pattern %d\n",enable);

	if (enable != ctx->test_pattern) {
		DRV_LOG(ctx, "enable(%u->%u)\n", ctx->test_pattern, enable);

		if (enable) {
		    subdrv_i2c_wr_u8(ctx, 0x3208, 0x03);
		    subdrv_i2c_wr_u8(ctx, 0x430b, 0x00);
		    subdrv_i2c_wr_u8(ctx, 0x430c, 0x00);
		    subdrv_i2c_wr_u8(ctx, 0x4310, 0x00);
		    subdrv_i2c_wr_u8(ctx, 0x4311, 0x00);
		    subdrv_i2c_wr_u8(ctx, 0x3208, 0x13);
		    subdrv_i2c_wr_u8(ctx, 0x3208, 0xa3);
		} else {
		    subdrv_i2c_wr_u8(ctx, 0x3208, 0x03);
		    subdrv_i2c_wr_u8(ctx, 0x430b, 0xff);
		    subdrv_i2c_wr_u8(ctx, 0x430c, 0xff);
		    subdrv_i2c_wr_u8(ctx, 0x4310, 0xff);
		    subdrv_i2c_wr_u8(ctx, 0x4311, 0xff);
		    subdrv_i2c_wr_u8(ctx, 0x3208, 0x13);
		    subdrv_i2c_wr_u8(ctx, 0x3208, 0xa3);
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

static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
	}
	return 0;
}
