// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 imx615mipiraw_Sensor.c
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
#include "imx615mipiraw_Sensor.h"

#define IMX615_EMBEDDED_DATA_EN 0

static void set_sensor_cali(void *arg);
static int get_sensor_temperature(void *arg);
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
//drv add by lipengpeng 20240617 start
//static int imx615_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
//drv add by lipengpeng 20240617 end 
static int imx615_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
//drv add by lipengpeng 20240617 start
//static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
//drv add by  lipengpeng 
static int open(struct subdrv_ctx *ctx);
//drv add by  lipengpeng 
//drv add by lipengpeng 20240603 start 
#if IS_ENABLED(CONFIG_DRV_ADC_GMS_DETECT)
//#include "../../../../../../../../kernel/kernel_device_modules-6.1/drivers/misc/mediatek/prize/gms_adc/adc_detect_gms.h"
#include "../../../../../../kernel/kernel_device_modules-6.1/drivers/misc/mediatek/pri/gms_adc/adc_detect_gms.h"
#endif
//drv add by lipengpeng 20240603 end 
/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, imx615_set_test_pattern},
//drv add by lipengpeng 20240617 start	
	//{SENSOR_FEATURE_SEAMLESS_SWITCH, imx615_seamless_switch},
//drv add by lipengpeng 20240617 end
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x01480005,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA0,

		.qsc_support = 0,
		.qsc_size = 0x0C00,
		.addr_qsc = 0x1E30,
		.sensor_reg_addr_qsc = 0xC800,
	},
	{
		.header_id = 0x0148000E,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA0,

		.qsc_support = 0,
		.qsc_size = 0x0C00,
		.addr_qsc = 0x1E30,
		.sensor_reg_addr_qsc = 0xC800,
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
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
	},
	.iMirrorFlip = 3,
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0CD0,
			.vsize = 0x09A0,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0CD0,
			.vsize = 0x09A0,
			.user_data_desc = VC_STAGGER_NE,
		},
	},

};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0CD0,
			.vsize = 0x09A0,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0CD0,
			.vsize = 0x09A0,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0CD0,
			.vsize = 0x09A0,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = imx615_preview_setting,
		.mode_setting_len = ARRAY_SIZE(imx615_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 285600000,
		.linelength = 3768,
		.framelength = 2518,
		.max_framerate = 300,
		.mipi_pixel_rate = 267600000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6560,
			.h0_size = 4928,
			.scale_w = 3280,
			.scale_h = 2464,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3280,
			.h1_size = 2464,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3280,
			.h2_tg_size = 2464,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,

		.ae_binning_ratio = 1465,
		.fine_integ_line = 826,
		.delay_frame = 3,
		.csi_param = {
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = imx615_capture_setting,
		.mode_setting_len = ARRAY_SIZE(imx615_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 285600000,
		.linelength = 3768,
		.framelength = 2518,
		.max_framerate = 300,
		.mipi_pixel_rate = 267600000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6560,
			.h0_size = 4928,
			.scale_w = 3280,
			.scale_h = 2464,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3280,
			.h1_size = 2464,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3280,
			.h2_tg_size = 2464,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,

		.ae_binning_ratio = 1465,
		.fine_integ_line = 826,
		.delay_frame = 3,
		.csi_param = {
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = imx615_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(imx615_normal_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 285600000,
		.linelength = 3768,
		.framelength = 2518,
		.max_framerate = 300,
		.mipi_pixel_rate = 267600000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6560,
			.h0_size = 4928,
			.scale_w = 3280,
			.scale_h = 2464,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3280,
			.h1_size = 2464,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3280,
			.h2_tg_size = 2464,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,

		.ae_binning_ratio = 1465,
		.fine_integ_line = 826,
		.delay_frame = 3,
		.csi_param = {
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = imx615_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(imx615_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 285600000,
		.linelength = 3768,
		.framelength = 2518,
		.max_framerate = 300,
		.mipi_pixel_rate = 267600000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6560,
			.h0_size = 4928,
			.scale_w = 3280,
			.scale_h = 2464,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3280,
			.h1_size = 2464,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3280,
			.h2_tg_size = 2464,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 826,
		.delay_frame = 3,
		.csi_param = {
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = imx615_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(imx615_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 285600000,
		.linelength = 3768,
		.framelength = 2518,
		.max_framerate = 300,
		.mipi_pixel_rate = 267600000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 6560,
			.full_h = 4928,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 6560,
			.h0_size = 4928,
			.scale_w = 3280,
			.scale_h = 2464,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3280,
			.h1_size = 2464,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3280,
			.h2_tg_size = 2464,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 826,
		.delay_frame = 3,
		.csi_param = {
			.dphy_trail = 98, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = IMX615_SENSOR_ID,
	.reg_addr_sensor_id = {0x0016, 0x0017},
	.i2c_addr_table = {0x20,0x34, 0x52,0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {6560, 4928},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,
	#if MIRROR_FILP
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	#else
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,
	#endif
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 0,
	.ana_gain_step = 1,
	.ana_gain_table = imx615_ana_gain_table,
	.ana_gain_table_size = sizeof(imx615_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 24,
	.exposure_max = 128 * (0xFFFC - 48),
	.exposure_step = 4,
	.exposure_margin = 48,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = BASE_DGAIN * 16,
	.dig_gain_step = 4,

	.frame_length_max = 0xFFFC,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 3,
	.start_exposure_offset = 500000,
//drv add by lipengpeng 20240617 start
	.pdaf_type = PDAF_SUPPORT_NA,
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = FALSE,
	
	//.seamless_switch_type = SEAMLESS_SWITCH_CUT_VB_INIT_SHUT,
	//.seamless_switch_hw_re_init_time_ns = 2750000,
	//.seamless_switch_prsh_hw_fixed_value = 32,
	//.seamless_switch_prsh_length_lc = 0,
	//.reg_addr_prsh_length_lines = {0x3039, 0x303a, 0x303b},
	//.reg_addr_prsh_mode = 0x3036,
//drv add by lipengpeng 20240617 end
	.temperature_support = TRUE,

	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,
	.s_cali = set_sensor_cali,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {
			{0x0202, 0x0203},
			{0x313A, 0x313B},
			{0x0224, 0x0225},
	},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x3128,
	.reg_addr_ana_gain = {
			{0x0204, 0x0205},
			{0x313C, 0x313D},
			{0x0216, 0x0217},
	},
	.reg_addr_dig_gain = {
			{0x020E, 0x020F},
			{0x313E, 0x313F},
			{0x0218, 0x0219},
	},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = 0x0138,
	.reg_addr_temp_read = 0x013A,
	.reg_addr_auto_extend = 0x0350,
	.reg_addr_frame_count = 0x0005,
	.reg_addr_fast_mode = 0x3010,

	.init_setting_table = imx615_init_setting,
	.init_setting_len = ARRAY_SIZE(imx615_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,

	.checksum_value = 0xda247687,

};
//drv add by lipengpeng 20240429 start
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = 2;
	u32 addr_h = ctx->s_ctx.reg_addr_sensor_id.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_sensor_id.addr[1];
	u32 addr_ll = ctx->s_ctx.reg_addr_sensor_id.addr[2];

#if IS_ENABLED(CONFIG_DRV_ADC_GMS_DETECT)
  //  printk("imx615 ---- gms_detection_getadc_v()=%d\n",gms_detection_getadc_v());
  if(check_board_type()==0)
    {
	 printk(" imx615 is volume production\n");
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
			//*sensor_id +=2;
			printk("imx615 i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == IMX615_SENSOR_ID) {
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
//drv add by lipengpeng 20240429 end
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
//drv add by lipengpeng 20240617 start
	//.vsync_notify = vsync_notify,
//drv add by lipengpeng 20240617 end
	.update_sof_cnt = common_update_sof_cnt,
//drv add by lipengpeng 20240617 start
	//.parse_ebd_line = common_parse_ebd_line,
//drv add by lipengpeng 20240617 end
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	
	{HW_ID_PDN, 0, 0},
	{HW_ID_RST, 0, 0},
	{HW_ID_AVDD, 2800000, 0},
	{HW_ID_DVDD, 1100000, 0},
	{HW_ID_DOVDD, 1800000, 5},
	{HW_ID_MCLK, 24, 0},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 1},
	{HW_ID_PDN, 1, 0},
	{HW_ID_RST, 1, 2}
};

const struct subdrv_entry imx615_mipi_raw_entry = {
	.name = "imx615_mipi_raw",
	.id = IMX615_SENSOR_ID,
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

	/* QSC data */
	support = info[idx].qsc_support;
	pbuf = info[idx].preload_qsc_table;
	size = info[idx].qsc_size;
	addr = info[idx].sensor_reg_addr_qsc;
	if (support) {
		if (pbuf != NULL && addr > 0 && size > 0) {
			subdrv_i2c_wr_u8(ctx, 0x86A9, 0x4E);
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
			subdrv_i2c_wr_u8(ctx, 0x32D2, 0x01);
			DRV_LOG(ctx, "set QSC calibration data done.");
		} else {
			subdrv_i2c_wr_u8(ctx, 0x32D2, 0x00);
		}
	}
}

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u8 temperature = 0;
	int temperature_convert = 0;

	temperature = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);

	if (temperature <= 0x60)
		temperature_convert = temperature;
	else if (temperature >= 0x61 && temperature <= 0x7F)
		temperature_convert = 97;
	else if (temperature >= 0x80 && temperature <= 0xE2)
		temperature_convert = -30;
	else
		temperature_convert = (char)temperature | 0xFFFFFF0;

	DRV_LOG(ctx, "temperature: %d degrees\n", temperature_convert);
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

static u16 get_gain2reg(u32 gain)
{
//drv add by lipengpeng 20240617 start
	//return (16384 - (16384 * BASEGAIN) / gain);
	return (1024 - (1024 * BASEGAIN + (gain >> 1)) / gain);
//drv add by lipengpeng 20240617 end
}
//drv add by lipengpeng 20240617 start
/*
static int imx615_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	u64 *feature_data = (u64 *)para;
	u32 exp_cnt = 0;
	enum SENSOR_SCENARIO_ID_ENUM pre_seamless_scenario_id;

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
	pre_seamless_scenario_id = ctx->current_scenario_id;
	update_mode_info(ctx, scenario_id);

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x3010, 0x02);
	i2c_table_write(ctx,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);

	if (ae_ctrl) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_STAGGER:
			set_multi_shutter_frame_length(ctx, (u64 *)&ae_ctrl->exposure, exp_cnt, 0);
			set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		default:
			set_shutter(ctx, ae_ctrl->exposure.le_exposure);
			set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		}
		common_get_prsh_length_lines(ctx, ae_ctrl, pre_seamless_scenario_id, scenario_id);
	}

	if (ctx->s_ctx.seamless_switch_prsh_length_lc > 0) {
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_prsh_mode, 0x01);

		subdrv_i2c_wr_u8(ctx,
				ctx->s_ctx.reg_addr_prsh_length_lines.addr[0],
				(ctx->s_ctx.seamless_switch_prsh_length_lc >> 16) & 0xFF);
		subdrv_i2c_wr_u8(ctx,
				ctx->s_ctx.reg_addr_prsh_length_lines.addr[1],
				(ctx->s_ctx.seamless_switch_prsh_length_lc >> 8)  & 0xFF);
		subdrv_i2c_wr_u8(ctx,
				ctx->s_ctx.reg_addr_prsh_length_lines.addr[2],
				(ctx->s_ctx.seamless_switch_prsh_length_lc) & 0xFF);

		DRV_LOG_MUST(ctx, "seamless switch pre-shutter set(%u)\n", ctx->s_ctx.seamless_switch_prsh_length_lc);
	} else
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_prsh_mode, 0x00);

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	DRV_LOG(ctx, "X: set seamless switch done\n");
	return ERROR_NONE;
}*/
//drv add by lipengpeng 20240617 start

static int imx615_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	kal_uint16 Color_R, Color_Gr, Color_Gb, Color_B;
	struct mtk_test_pattern_data *data = (struct mtk_test_pattern_data *)para;
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	if (mode) {
            subdrv_i2c_wr_u8(ctx, 0x0601, 0x02);
			
			Color_R = (data->Channel_R >> 22) & 0x3FF; //10bits depth color
			Color_Gr = (data->Channel_Gr >> 22) & 0x3FF;
			Color_B = (data->Channel_B >> 22) & 0x3FF;
			Color_Gb = (data->Channel_Gb >> 22) & 0x3FF;
			subdrv_i2c_wr_u8(ctx,0x0602, (Color_R >> 8) & 0x3);
			subdrv_i2c_wr_u8(ctx,0x0603, Color_R & 0xFF);
			subdrv_i2c_wr_u8(ctx,0x0604, (Color_Gr >> 8) & 0x3);
			subdrv_i2c_wr_u8(ctx,0x0605, Color_Gr & 0xFF);
			subdrv_i2c_wr_u8(ctx,0x0606, (Color_B >> 8) & 0x3);
			subdrv_i2c_wr_u8(ctx,0x0607, Color_B & 0xFF);
			subdrv_i2c_wr_u8(ctx,0x0608, (Color_Gb >> 8) & 0x3);
			subdrv_i2c_wr_u8(ctx,0x0609, Color_Gb & 0xFF);
	}else{
		    subdrv_i2c_wr_u8(ctx, 0x0601, 0x00);
	}

	ctx->test_pattern = mode;
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
//drv add by lipengpeng 20240601 start 
int open(struct subdrv_ctx *ctx)
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

/*static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_prsh_mode, 0x00);
		set_i2c_buffer(ctx, 0x3010, 0x00);
		commit_i2c_buffer(ctx);
	}
	return 0;
}*/
//drv add by lipengpeng 20240617 end