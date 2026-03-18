// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 ov32d40mipiraw_Sensor.c
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
#include "ov32d40mipiraw_Sensor.h"

static void set_sensor_cali(void *arg);

static void set_group_hold(void *arg, u8 en);
//static void ov32d40_set_dummy(struct subdrv_ctx *ctx);
//static int ov32d40_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static u16 get_gain2reg(u32 gain);
static int ov32d40_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static void ov32d40_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);


/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, ov32d40_set_test_pattern},

};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x010B00FF,
		.addr_header_id = 0x00000001,
		.i2c_write_id = 0xA0,

		.pdc_support = false,
		.pdc_size = 728,
		.addr_pdc = 0x1638,
		.sensor_reg_addr_pdc = 0x5900,

		.xtalk_support = false,
		.xtalk_table = data_xtalk_ov32d402q,
		.xtalk_size = ARRAY_SIZE(data_xtalk_ov32d402q),
		.addr_xtalk = PARAM_UNDEFINED,
		.sensor_reg_addr_xtalk = 0x53C0,
	},
};



static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0cc0, //3264
			.vsize = 0x0990, //2448
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0cc0, //3264
			.vsize = 0x0990, //2448
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0cc0, //3264
			.vsize = 0x072c, //1836
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0cc0, //3264
			.vsize = 0x0990, //2448
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0cc0, //3264
			.vsize = 0x0990, //2448
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1980, //6528
			.vsize = 0x1320, //4896
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1920, //3264
			.vsize = 1080, //2448
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = addr_data_pair_preview_ov32d402q,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_preview_ov32d402q),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 1182,
		.framelength = 2820,
		.max_framerate = 300,
		.mipi_pixel_rate = 120000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6528,
			.h0_size = 4896,
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
		
		//.s_dummy_support = 1,
		//.ae_ctrl_support = 1,
		//.pdaf_cap = TRUE,
		//.imgsensor_pd_info = &imgsensor_pd_info,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = addr_data_pair_capture_ov32d402q,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_capture_ov32d402q),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = addr_data_pair_capture_ov32d402q,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(addr_data_pair_capture_ov32d402q),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 1182,
		.framelength = 2820,
		.max_framerate = 300,
		.mipi_pixel_rate = 120000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6528,
			.h0_size = 4896,
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
		
		//.pdaf_cap = TRUE,
		//.imgsensor_pd_info = &imgsensor_pd_info,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = addr_data_pair_video_ov32d402q,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_video_ov32d402q),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 1182,
		.framelength = 2820,
		.max_framerate = 300,
		.mipi_pixel_rate = 120000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6528,
			.h0_size = 4896,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 306,
			.w1_size = 3264,
			.h1_size = 1836,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1836,
		},
		
		//.pdaf_cap = TRUE,
		//.imgsensor_pd_info = &imgsensor_pd_info,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = addr_data_pair_hs_video_ov32d402q,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_hs_video_ov32d402q),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 1182,
		.framelength = 2820,
		.max_framerate = 300,
		.mipi_pixel_rate = 120000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6528,
			.h0_size = 4896,
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
		
		//.s_dummy_support = 1,
		//.ae_ctrl_support = 1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
		
	},

	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = addr_data_pair_slim_video_ov32d402q,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_slim_video_ov32d402q),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 115200000,
		.linelength = 720,
		.framelength = 2664,
		.max_framerate = 600,
		.mipi_pixel_rate = 512000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 384,
			.y0_offset = 288,
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
		
		//.s_dummy_support = 1,
		//.ae_ctrl_support = 1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	{//custom1
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = addr_data_pair_custom1_ov32d40,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom1_ov32d40),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 3920,
		.framelength = 2551,
		.max_framerate = 100,
		.mipi_pixel_rate = 120000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6528,
			.h0_size = 4896,
			.scale_w = 6528,
			.scale_h = 4896,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 6528,
			.h1_size = 4896,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 6528,
			.h2_tg_size = 4896,
		},

		//.s_dummy_support = 1,
		//.ae_ctrl_support = 1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
		
	},
	{//custom1
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = addr_data_pair_custom2_ov32d40,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom2_ov32d40),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 1182,
		.framelength = 2820,
		.max_framerate = 300,
		.mipi_pixel_rate = 120000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 6528,
			.full_h = 4896,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6528,
			.h0_size = 4896,
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
		
		//.s_dummy_support = 1,
		//.ae_ctrl_support = 1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
		
	},			
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = OV32D40_SENSOR_ID,
	.reg_addr_sensor_id = {0x300A, 0x300B, 0x300C},
	.i2c_addr_table = {0x6c,0x42,0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {6528, 4896},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_CSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 15.5,
	.ana_gain_type = 1,
	.ana_gain_step = 4,
	.ana_gain_table = ov32d40_ana_gain_table,
	.ana_gain_table_size = sizeof(ov32d40_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 8,
	.exposure_max = 0xFFFFFF - 22,
	.exposure_step = 2,
	.exposure_margin = 22,

	.frame_length_max = 0xFFFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 500000,
	.pdaf_type = PDAF_SUPPORT_NA,
	.hdr_type = HDR_SUPPORT_NA,	
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,
	.s_cali = set_sensor_cali,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = PARAM_UNDEFINED,
	.reg_addr_exposure = {{0x3500, 0x3501, 0x3502},},
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x3508, 0x3509},}, 
	.reg_addr_frame_length = {0x380E, 0x380F},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = 0x387F,

	.init_setting_table = PARAM_UNDEFINED,
	.init_setting_len = PARAM_UNDEFINED,
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,
	.checksum_value = 0xffffffff,
};

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
			//*sensor_id +=1;
			printk("ov32d40 i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == OV32D40_SENSOR_ID) {
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

static struct subdrv_ops ops = {
	.get_id = common_get_imgsensor_id,
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
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_AVDD, 2800000, 1},
	{HW_ID_DOVDD, 1800000, 3},
	{HW_ID_DVDD, 1200000, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 2, 0},
	{HW_ID_RST, 1, 2},

};

const struct subdrv_entry ov32d40_mipi_raw_entry = {
	.name = "ov32d40_mipi_raw",
	.id = OV32D40_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* STRUCT */

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
			size = 8;
			addr = 0x5C0E;
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
			pbuf += size;
			size = 720;
			addr = 0x5900;
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
			DRV_LOG(ctx, "set PDC calibration data done.");
		}
	}
}

/*static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	int temperature = 0;


	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_temp_en, 0x01);
	temperature = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);
	temperature = (temperature > 0xC0) ? (temperature - 0x100) : temperature;

	DRV_LOG(ctx, "temperature: %d degrees\n", temperature);
	return temperature;
}*/

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
	kal_uint16 iReg = 0x0000;

	//platform 1xgain = 64, sensor driver 1*gain = 0x100
	iReg = gain*256/BASEGAIN;

	if(iReg < 0x100)// sensor 1xGain
	{
		iReg = 0X100;
	}
	if(iReg > 0xf80)// sensor 15.5xGain
	{
		iReg = 0Xf80;
	}
	return iReg;
}

static int ov32d40_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	if (mode){
		subdrv_i2c_wr_u8(ctx, 0x3019, 0xf0);
		subdrv_i2c_wr_u8(ctx, 0x4308, 0x01);
		subdrv_i2c_wr_u8(ctx, 0x4300, 0x00);
		subdrv_i2c_wr_u8(ctx, 0x4301, 0x00);
		subdrv_i2c_wr_u8(ctx, 0x4302, 0x00);
		subdrv_i2c_wr_u8(ctx, 0x4303, 0x00);
		subdrv_i2c_wr_u8(ctx, 0x4304, 0x00);
		subdrv_i2c_wr_u8(ctx, 0x4305, 0x00);
		subdrv_i2c_wr_u8(ctx, 0x4306, 0x00);
		subdrv_i2c_wr_u8(ctx, 0x4307, 0x00);
	}
	else if (ctx->test_pattern){
		subdrv_i2c_wr_u8(ctx, 0x3019, 0xd2);
		subdrv_i2c_wr_u8(ctx, 0x4308, 0x00);
	}

	ctx->test_pattern = mode;

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
static void ov32d40_sensor_init(struct subdrv_ctx *ctx)
{

	printk("ov32d40_sensor_init\n");
   i2c_table_write(ctx, sensor_init_addr_data_ov32d40, sizeof(sensor_init_addr_data_ov32d40)/sizeof(u16));
   
	printk("ov32d40_sensor_init end\n");
}
static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	ov32d40_sensor_init(ctx);

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
