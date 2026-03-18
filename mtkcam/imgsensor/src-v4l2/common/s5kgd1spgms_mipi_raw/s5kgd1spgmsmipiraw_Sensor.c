// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5kgd1spgmsmipiraw_Sensor.c
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
#include "s5kgd1spgmsmipiraw_Sensor.h"

#define USING_DPHY_N_LANE 4

static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int s5kgd1spgms_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kgd1spgms_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static void s5kgd1spgms_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
//static int s5kgd1spgms_set_ctrl_locker(struct subdrv_ctx *ctx, u32 cid, bool *is_lock);

//drv add by lipengpeng 20240603 start 
#if IS_ENABLED(CONFIG_DRV_ADC_GMS_DETECT)
//#include "../../../../../../../../kernel/kernel_device_modules-6.1/drivers/misc/mediatek/prize/gms_adc/adc_detect_gms.h"
#include "../../../../../../kernel/kernel_device_modules-6.1/drivers/misc/mediatek/pri/gms_adc/adc_detect_gms.h"
#endif
//drv add by lipengpeng 20240603 end 

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, s5kgd1spgms_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, s5kgd1spgms_set_test_pattern_data},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x010B00FF,
		.addr_header_id = 0x00000001,
		.i2c_write_id = 0xA0,

		.xtalk_support = false,
		.xtalk_size = 2048,
		.addr_xtalk = 0x150F,
	},
};

//1632,  1224}, // Preview 
//3264,  2448}, // capture 
//1632,  1224}, // video 
// 816,   612},// hight video 120
//0, 1280,   720},// slim video 

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,  //0x2b 
			.hsize = 0x0CD0,  // /8  3280
			.vsize = 0x09A0,  // /4 2464
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CD0,  // /8  3280
			.vsize = 0x09A0,  // /4 2464
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CD0,  // /8  3280
			.vsize = 0x09A0,  // /4 2464
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CD0,  // /8  3280
			.vsize = 0x09A0,  // /4 2464
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CD0,  // /8  3280
			.vsize = 0x09A0,  // /4 2464
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,  //0x2b  4080, 3072},
			.hsize = 0x0CD0,  // /8  3280
			.vsize = 0x09A0,  // /4 2464

		},
	},
};

//.pclk = 1600000000,
//.linelength = 17792,
//.framelength = 2992,
//.startx = 0,
//.starty = 0,
//.grabwindow_width = 3280,
//.grabwindow_height = 2464,
//.mipi_data_lp2hs_settle_dc = 85,
//.max_framerate = 300,
//.mipi_pixel_rate = 320000000,
		
static struct subdrv_mode_struct mode_struct[] = {
	{
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
		.pclk = 1144000000,
		.linelength = 14528,
		.framelength = 2624,
		.max_framerate = 300,
		.mipi_pixel_rate = 345600000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { //	6560, 4928,    0,    0, 6560, 4928, 3280, 2464,   0, 0, 3280, 2464,   0, 0, 3280, 2464,	/* Preview */
			.full_w = 6560, //
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
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
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
		.pclk = 1144000000,
		.linelength = 14528,
		.framelength = 2624,
		.max_framerate = 300,
		.mipi_pixel_rate = 345600000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { //	{4608, 3456, 0, 0, 4608, 3456, 2304, 1728, 0000, 0000, 2304, 1728, 0, 0, 2304, 1728},	/* Preview */
			.full_w = 6560, //
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
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
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
		.pclk = 1144000000,
		.linelength = 14528,
		.framelength = 2624,
		.max_framerate = 300,
		.mipi_pixel_rate = 345600000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { //	{4608, 3456, 0, 0, 4608, 3456, 2304, 1728, 0000, 0000, 2304, 1728, 0, 0, 2304, 1728},	/* Preview */
			.full_w = 6560, //
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
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
//.pclk = 1600000000,
//.linelength = 4560,
//.framelength = 2916,
//.startx = 0,
//.starty = 0,
//.grabwindow_width = 3280,
//.grabwindow_height = 2464,
//.mipi_data_lp2hs_settle_dc = 85,
//.max_framerate = 1203,
//.mipi_pixel_rate = 1440000000,	
	{
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
		.pclk = 1144000000,
		.linelength = 14528,
		.framelength = 2624,
		.max_framerate = 1200,
		.mipi_pixel_rate = 345600000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { //	{4608, 3456, 0, 0, 4608, 3456, 2304, 1728, 0000, 0000, 2304, 1728, 0, 0, 2304, 1728},	/* Preview */
			.full_w = 6560, //
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
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
//.pclk = 1600000000,
//.linelength = 10144,
//.framelength = 5182,
//.startx = 0,
//.starty = 0,
//.grabwindow_width = 6560,
//.grabwindow_height = 4936,
//.mipi_data_lp2hs_settle_dc = 85,
//.max_framerate = 304,
//.mipi_pixel_rate = 1296000000,
	{
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
		.pclk = 1144000000,
		.linelength = 14528,
		.framelength = 2624,
		.max_framerate = 300,
		.mipi_pixel_rate = 345600000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { //	{4608, 3456, 0, 0, 4608, 3456, 2304, 1728, 0000, 0000, 2304, 1728, 0, 0, 2304, 1728},	/* Preview */
			.full_w = 6560, //
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
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},

	{
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
		.pclk = 1144000000,
		.linelength = 14528,
		.framelength = 2624,
		.max_framerate = 300,
		.mipi_pixel_rate = 345600000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = { //	{4608, 3456, 0, 0, 4608, 3456, 2304, 1728, 0000, 0000, 2304, 1728, 0, 0, 2304, 1728},	/* Preview */
			.full_w = 6560, //
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
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},

};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = S5KGD1SPGMS_SENSOR_ID,
	.reg_addr_sensor_id = {0x0000, 0x0001},  //((read_cmos_sensor_8(0x0000) << 8) | read_cmos_sensor_8(0x0001));
	.i2c_addr_table = {0x20, 0x5a, 0xff},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16, //drv add by lipengpeng 20240723 I2C_DT_ADDR_16_DATA_8
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {6560, 4928},
	.mirror = IMAGE_HV_MIRROR, //drv add by lipengpeng 20240723 camera mirror filip

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_2MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,
//drv add by lipengpeng 20240723 camera mirror filip
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gb,  //   SENSOR_OUTPUT_FORMAT_RAW_B SENSOR_OUTPUT_FORMAT_RAW_Gb  SENSOR_OUTPUT_FORMAT_RAW_R SENSOR_OUTPUT_FORMAT_RAW_Gr
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_type = 2,
	.ana_gain_step = 32,
	.ana_gain_table = s5kgd1spgms_ana_gain_table,
	.ana_gain_table_size = sizeof(s5kgd1spgms_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 3,
	.exposure_max = 0xFFFF - 3,
	.exposure_step = 1,
	.exposure_margin = 3,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 500000,
#ifdef IMGSENSOR_FUSION_TEST_WORKAROUND
	.start_exposure_offset_custom = 5500000,
#endif
	.pdaf_type = PDAF_SUPPORT_NA,
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,
	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,  //write_cmos_sensor_8(0x0100, 0X01); //drv add by lipengpeng    ./adaptor-subdrv-ctrl.c +2064 subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x01); 
	.reg_addr_mirror_flip = 0x0101,  //write_cmos_sensor_8(0x0101, itemp);   ./adaptor-subdrv-ctrl.c +202 itemp = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_mirror_flip) & ~0x03;
	.reg_addr_exposure = {{0x0202, 0x0203},},  //write_cmos_sensor16(0x0202, shutter);
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x0204, 0x0205},},  //write_cmos_sensor16(0x0204, reg_gain);
	.reg_addr_frame_length = {0x0340, 0x0341}, //write_cmos_sensor16(0x0340, imgsensor.frame_length);
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
	.chk_s_off_end = 1,

	.checksum_value = 0x31E3FBE2,

	/* custom stream control to mipi delay time for hw limitation */
	.custom_stream_ctrl_delay = TRUE,
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
    printk("S5KGD1SPGMS ---- gms_detection_getadc_v()=%d\n",gms_detection_getadc_v());
  if(check_board_type()==1)
    {
	 printk(" s5kgd1spgms is volume production\n");
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
			*sensor_id += 0x1001;
			printk("S5KGD1SPGMS i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == S5KGD1SPGMS_SENSOR_ID) {
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
	//.set_ctrl_locker = s5kgd1spgms_set_ctrl_locker,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_PDN, 0, 1},
	{HW_ID_DVDD, 1100000, 1}, // pmic_ldo for dvdd
	{HW_ID_AVDD, 2800000, 1}, // pmic_ldo for avdd
	{HW_ID_DOVDD, 1800000, 3}, // pmic_ldo/gpio(1.8V ldo) for dovdd
	{HW_ID_MCLK_DRIVING_CURRENT, 2, 0},
	{HW_ID_PDN, 1, 1},
	{HW_ID_RST, 1, 2}
};

const struct subdrv_entry s5kgd1spgms_mipi_raw_entry = {
	.name = "s5kgd1spgms_mipi_raw",
	.id = S5KGD1SPGMS_SENSOR_ID,
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
	//reg_gain = gain/2;  
}

static int s5kgd1spgms_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
	return ERROR_NONE;
}

static int s5kgd1spgms_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct mtk_test_pattern_data *data = (struct mtk_test_pattern_data *)para;
	u16 R = (data->Channel_R >> 22) & 0x3ff;
	u16 Gr = (data->Channel_Gr >> 22) & 0x3ff;
	u16 Gb = (data->Channel_Gb >> 22) & 0x3ff;
	u16 B = (data->Channel_B >> 22) & 0x3ff;

	subdrv_i2c_wr_u16(ctx, 0x0602, R);
	subdrv_i2c_wr_u16(ctx, 0x0604, Gr);
	subdrv_i2c_wr_u16(ctx, 0x0606, B);
	subdrv_i2c_wr_u16(ctx, 0x0608, Gb);

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

static void s5kgd1spgms_sensor_init(struct subdrv_ctx *ctx)
{

	printk("s5kgd1spgms_sensor_init\n");

	i2c_table_write(ctx, sensor_init_addr_data, sizeof(sensor_init_addr_data)/sizeof(u16));
   
	printk("s5kgd1spgms_sensor_init end\n");
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	s5kgd1spgms_sensor_init(ctx);

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
