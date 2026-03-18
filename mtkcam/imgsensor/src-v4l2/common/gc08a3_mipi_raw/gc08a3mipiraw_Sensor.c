// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 gc08a3mipiraw_Sensor.c
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
#include "gc08a3mipiraw_Sensor.h"

#define LOG_TAG "[gc08a3]"
#define LOG_INF(format, args...) pr_info(LOG_TAG "[%s] " format, __func__, ##args)

// prize add by chenwenhui for otp start
#define GC08A3_OTP_FOR_MTK       1
#define GC08A3_OTP_INFO_FLAG_ADDR_OFFSET    0x15A0
#define GC08A3_OTP_AWB_FLAG_ADDR_OFFSET     0x16E0
#define GC08A3_OTP_AWB_DATA_ADDR_OFFSET     0x16E8
#define GC08A3_OTP_AWB_GROUP_OFFSET         0x88
#define GC08A3_OTP_AWB_DATA_SIZE            16

#define GC08A3_OTP_LSC_FLAG_ADDR_OFFSET     0x1880
#define GC08A3_OTP_LSC_DATA_ADDR_OFFSET     0x1888
#define GC08A3_OTP_LSC_GROUP_OFFSET         0x3A68
#define GC08A3_OTP_LSC_DATA_SIZE            1868

#if GC08A3_OTP_FOR_MTK
struct otp_awb_info_struct {
	unsigned char  awb_flag;
	unsigned char  unit_r_h;
	unsigned char  unit_r_l;
	unsigned char  unit_gr_h;
	unsigned char  unit_gr_l;
	unsigned char  unit_gb_h;
	unsigned char  unit_gb_l;
	unsigned char  unit_b_h;
	unsigned char  unit_b_l;
	unsigned char  golden_r_h;
	unsigned char  golden_r_l;
	unsigned char  golden_gr_h;
	unsigned char  golden_gr_l;
	unsigned char  golden_gb_h;
	unsigned char  golden_gb_l;
	unsigned char  golden_b_h;
	unsigned char  golden_b_l;
	unsigned char  checksum_of_awb;
};

struct imgsensor_otp_info_struct {
	unsigned char  info_flag;
	unsigned char  supply_id;
	unsigned char  module_id;
	unsigned char  lends_id;
	unsigned char  vcm_ld;
	unsigned char  driver_id;
	unsigned char  module_version;
	unsigned char  software_version;
	unsigned char  year;
	unsigned char  month;
	unsigned char  day;
	unsigned char  reserved0;
	unsigned char  reserved1;
	unsigned char  checksum_of_info;
	struct otp_awb_info_struct awb;
	unsigned char  lsc_flag;
	unsigned char  lsc[1868];
	unsigned char  checksum_of_lsc;
};

extern struct imgsensor_otp_info_struct gc08a3_otp_info;
#define GC08A3_OTP_DATA_DUMP    0
static bool GC08A3_OTP_Enable = true;
#endif
// prize add by chenwenhui for otp end

static int gc08a3_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc08a3_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc08a3_set_shutter_frame_length(struct subdrv_ctx *ctx,u8 *para, u32 *len);
static int gc08a3_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc08a3_set_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc08a3_set_streaming_suspned(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int gc08a3_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static void gc08a3_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, gc08a3_set_test_pattern},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, gc08a3_set_streaming_resume},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, gc08a3_set_streaming_suspned},
	{SENSOR_FEATURE_SET_GAIN, gc08a3_set_gain},
	{SENSOR_FEATURE_SET_ESHUTTER, gc08a3_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, gc08a3_set_shutter_frame_length},
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

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = gc08a3_3264x2448_30fps_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc08a3_3264x2448_30fps_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 280000000,
		.linelength = 3640,
		.framelength = 2548,
		.max_framerate = 300,
		.mipi_pixel_rate = 268800000,
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
			.dphy_trail = 84,
		},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = gc08a3_3264x2448_30fps_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc08a3_3264x2448_30fps_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 280000000,
		.linelength = 3640,
		.framelength = 2548,
		.max_framerate = 300,
		.mipi_pixel_rate = 268800000,
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
			.dphy_trail = 84,
		},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = gc08a3_3264x2448_30fps_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc08a3_3264x2448_30fps_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 280000000,
		.linelength = 3640,
		.framelength = 2548,
		.max_framerate = 300,
		.mipi_pixel_rate = 268800000,
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
			.dphy_trail  = 84,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = GC08A3_SENSOR_ID,
	.reg_addr_sensor_id = {0x03F0, 0x03F1},
	.i2c_addr_table = {0x62, 0xFF},
	.i2c_burst_write_support = FALSE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = 0,
	.eeprom_num = 0,
	.resolution = {3264, 2448},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_type = 4,
	.ana_gain_step = 1,
	.ana_gain_table = gc08a3_ana_gain_table,
	.ana_gain_table_size = sizeof(gc08a3_ana_gain_table),
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

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = PARAM_UNDEFINED,
	.reg_addr_exposure = {{0x0202},},
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x0204},},
	.reg_addr_frame_length = {0x0340},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = PARAM_UNDEFINED,
	.init_setting_table = gc08a3_init_setting,
	.init_setting_len = ARRAY_SIZE(gc08a3_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.checksum_value = 0xffffffff,
};

static struct subdrv_ops ops = {
	.get_id = gc08a3_get_imgsensor_id,
	//.get_id = common_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	//.open = common_open,
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
	{HW_ID_DVDD, 1200000, 5},
	{HW_ID_AVDD, 2800000, 0},
	{HW_ID_PDN, 1, 0},
	{HW_ID_RST, 1, 5},
};

const struct subdrv_entry gc08a3_mipi_raw_entry = {
	.name = "gc08a3_mipi_raw",
	.id = GC08A3_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

#if GC08A3_OTP_FOR_MTK
static void gc08a3_custom_read_otp_init(struct subdrv_ctx *ctx)
{
	memset(&gc08a3_otp_info, 0, sizeof(struct imgsensor_otp_info_struct));

	subdrv_i2c_wr_u8(ctx, 0x031c, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x0315, 0x80);
	//otp read init setting
	subdrv_i2c_wr_u8(ctx, 0x0324, 0x44);
	subdrv_i2c_wr_u8(ctx, 0x0316, 0x09);//[3] otpclk_en
	subdrv_i2c_wr_u8(ctx, 0x0a67, 0x80);//[7] otp_en
	subdrv_i2c_wr_u8(ctx, 0x0313, 0x00);

	subdrv_i2c_wr_u8(ctx, 0x0a53, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0a65, 0x17);
	subdrv_i2c_wr_u8(ctx, 0x0a68, 0xa1);
	subdrv_i2c_wr_u8(ctx, 0x0a47, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0a58, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0ace, 0x0c);
}

static void gc08a3_custom_read_otp_info(struct subdrv_ctx *ctx)
{
    kal_uint32 addr = 0;
    unsigned char index = 0;

	//info flag
	addr = GC08A3_OTP_INFO_FLAG_ADDR_OFFSET;
	subdrv_i2c_wr_u8(ctx, 0x0a69,(addr >> 8));
	subdrv_i2c_wr_u8(ctx, 0x0a6a,(addr & 0xff));
	subdrv_i2c_wr_u8(ctx, 0x0313,0x20);// otp mode3,[7]:otp auto start [6]:otp write [5]otp read [4:0]:reserved
	gc08a3_otp_info.info_flag = subdrv_i2c_rd_u8(ctx, 0x0a6c);  //[5:4]:grop3 [3:2]:grop2 [1:0]:group1
	if (gc08a3_otp_info.info_flag == 0) {
		LOG_INF("error: gc08a3 otp data is empty \n");
		return;
	} else if(gc08a3_otp_info.info_flag == 0x10) {
		index = 3;
	}
	index  = (gc08a3_otp_info.info_flag / 4) + 1;
	LOG_INF("gc08a3 otp  flag %d group %d info is valid\n",gc08a3_otp_info.info_flag,index);

	// module inf0
	addr = 0x15b0 + (index-1) * 0x68;
	subdrv_i2c_wr_u8(ctx, 0x0a69,(addr >> 8));
	subdrv_i2c_wr_u8(ctx, 0x0a6a,(addr & 0xff));
	subdrv_i2c_wr_u8(ctx, 0x0313,0x20);// otp mode3,[7]:otp auto start [6]:otp write [5]otp read [4:0]:reserved
	gc08a3_otp_info.module_id = subdrv_i2c_rd_u8(ctx, 0x0a6c);

	// year
	addr = 0x15e0 + (index-1) * 0x68;
	subdrv_i2c_wr_u8(ctx, 0x0a69,(addr >> 8));
	subdrv_i2c_wr_u8(ctx, 0x0a6a,(addr & 0xff));
	subdrv_i2c_wr_u8(ctx, 0x0313,0x20);
	gc08a3_otp_info.year = subdrv_i2c_rd_u8(ctx, 0x0a6c);

	// month
	addr = 0x15e8 + (index-1) * 0x68;
	subdrv_i2c_wr_u8(ctx, 0x0a69,(addr >> 8));
	subdrv_i2c_wr_u8(ctx, 0x0a6a,(addr & 0xff));
	subdrv_i2c_wr_u8(ctx, 0x0313,0x20);
	gc08a3_otp_info.month = subdrv_i2c_rd_u8(ctx, 0x0a6c);

	// day
	addr = 0x15f0 + (index-1) * 0x68;
	subdrv_i2c_wr_u8(ctx, 0x0a69,(addr >> 8));
	subdrv_i2c_wr_u8(ctx, 0x0a6a,(addr & 0xff));
	subdrv_i2c_wr_u8(ctx, 0x0313,0x20);
	gc08a3_otp_info.day = subdrv_i2c_rd_u8(ctx, 0x0a6c);
	LOG_INF("gc08a3 otp module_id: 0x%x  date: %d-%d-%d \n",gc08a3_otp_info.module_id,gc08a3_otp_info.year,gc08a3_otp_info.month,gc08a3_otp_info.day);

}

static void gc08a3_custom_read_kgroup(struct subdrv_ctx *ctx,kal_uint32 addr, unsigned char *buff, int size)
{
	kal_uint16 i;
	for (i = 0; i < size; i++) {
		subdrv_i2c_wr_u8(ctx, 0x0a69,(addr >> 8));
		subdrv_i2c_wr_u8(ctx, 0x0a6a,(addr & 0xff));
		subdrv_i2c_wr_u8(ctx, 0x0313,0x20);// otp mode3,[7]:otp auto start [6]:otp write [5]otp read [4:0]:reserved
		*(buff) = subdrv_i2c_rd_u8(ctx, 0x0a6c);
	#if GC08A3_OTP_DATA_DUMP
		LOG_INF("gc08a3 otp  data: 0x%x \n",*(buff));
	#endif
		addr += 8;
		buff++;
	}
}

static void gc08a3_custom_get_otp_data(struct subdrv_ctx *ctx)
{
	// get awb data
	kal_uint32 addr = 0;
	unsigned char index = 0;
	//info awb flag
	addr = GC08A3_OTP_AWB_FLAG_ADDR_OFFSET;
	subdrv_i2c_wr_u8(ctx, 0x0a69,(addr >> 8));
	subdrv_i2c_wr_u8(ctx, 0x0a6a,(addr & 0xff));
	subdrv_i2c_wr_u8(ctx, 0x0313,0x20);//
	gc08a3_otp_info.awb.awb_flag = subdrv_i2c_rd_u8(ctx, 0x0a6c);  //[5:4]:grop3 [3:2]:grop2 [1:0]:group1
	if (gc08a3_otp_info.awb.awb_flag == 0) {
		LOG_INF("error: gc08a3 otp awb data is empty \n");
		return;
	} else if(gc08a3_otp_info.awb.awb_flag == 0x10) {
		index = 3;
	} else
		index  = (gc08a3_otp_info.awb.awb_flag / 4) + 1;

	addr = GC08A3_OTP_AWB_GROUP_OFFSET * (index -1)  + GC08A3_OTP_AWB_DATA_ADDR_OFFSET;
#if GC08A3_OTP_DATA_DUMP
		LOG_INF("gc08a3 otp dump awb data\n");
#endif
	gc08a3_custom_read_kgroup(ctx, addr,&(gc08a3_otp_info.awb.unit_r_h),GC08A3_OTP_AWB_DATA_SIZE);

	LOG_INF("gc08a3 otp awb unit (R/Gr/Gb/B): 0x%x 0x%x 0x%x 0x%x ,golden(R/Gr/Gb/B): 0x%x 0x%x 0x%x 0x%x \n",
		gc08a3_otp_info.awb.unit_r_h << 8 | gc08a3_otp_info.awb.unit_r_l,gc08a3_otp_info.awb.unit_gr_h << 8 | gc08a3_otp_info.awb.unit_gr_l,
		gc08a3_otp_info.awb.unit_gb_h << 8 | gc08a3_otp_info.awb.unit_gb_l,gc08a3_otp_info.awb.unit_b_h << 8 | gc08a3_otp_info.awb.unit_b_l,
		gc08a3_otp_info.awb.golden_r_h << 8 | gc08a3_otp_info.awb.golden_r_l,gc08a3_otp_info.awb.golden_gr_h << 8 | gc08a3_otp_info.awb.golden_gr_l,
		gc08a3_otp_info.awb.golden_gb_h << 8 | gc08a3_otp_info.awb.golden_gb_l,gc08a3_otp_info.awb.golden_b_h << 8 | gc08a3_otp_info.awb.golden_b_l);

	//info lsc flag
	addr = GC08A3_OTP_LSC_FLAG_ADDR_OFFSET;
	subdrv_i2c_wr_u8(ctx, 0x0a69,(addr >> 8));
	subdrv_i2c_wr_u8(ctx, 0x0a6a,(addr & 0xff));
	subdrv_i2c_wr_u8(ctx, 0x0313,0x20);//
	gc08a3_otp_info.lsc_flag = subdrv_i2c_rd_u8(ctx, 0x0a6c);  //[5:4]:grop3 [3:2]:grop2 [1:0]:group1
	if (gc08a3_otp_info.lsc_flag == 0) {
		LOG_INF("error: gc08a3 otp lsc data is empty \n");
		return;
	} else if(gc08a3_otp_info.lsc_flag == 0x10) {
		index = 3;
	} else
		index  = (gc08a3_otp_info.info_flag / 4) + 1;

	LOG_INF("gc08a3 otp lsc flag %d group %d data is valid\n",gc08a3_otp_info.info_flag,index);

#if GC08A3_OTP_DATA_DUMP
	LOG_INF("gc08a3 otp dump lsc data\n");
#endif

	addr = GC08A3_OTP_LSC_GROUP_OFFSET * (index -1)  + GC08A3_OTP_LSC_DATA_ADDR_OFFSET;

	gc08a3_custom_read_kgroup(ctx, addr,&gc08a3_otp_info.lsc[0],GC08A3_OTP_LSC_DATA_SIZE);
}
#endif

/* FUNCTION */
static int gc08a3_get_imgsensor_id(struct subdrv_ctx *ctx,u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = 2;
	u32 addr_h = ctx->s_ctx.reg_addr_sensor_id.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_sensor_id.addr[1];

	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) | subdrv_i2c_rd_u8(ctx, addr_l);

			LOG_INF("gc08a3 i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == GC08A3_SENSOR_ID) {
				*sensor_id = ctx->s_ctx.sensor_id;
				LOG_INF("get 2lane id successful!!! gc08a3 i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);	

				#if GC08A3_OTP_FOR_MTK
				if (GC08A3_OTP_Enable) {
					gc08a3_custom_read_otp_init(ctx);
					mdelay(10);
					gc08a3_custom_read_otp_info(ctx);
					gc08a3_custom_get_otp_data(ctx);
					GC08A3_OTP_Enable = false;
				}
				#endif

				return ERROR_NONE;
			}
			LOG_INF("[gc08a3]: Read sensor id fail! i2c write id: 0x%x, sensor id: 0x%x\n",
				ctx->i2c_write_id, *sensor_id);
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
static int gc08a3_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 gain = *((u32 *)para);
	u32 rg_gain;

	LOG_INF("platform_gain = 0x%x \n", gain);
	/* check boundary of gain */
	gain = max(gain, ctx->s_ctx.ana_gain_min);
	gain = min(gain, ctx->s_ctx.ana_gain_max);
	rg_gain = gain;
	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	ctx->ana_gain[0] = gain;
	/* write gain */
	subdrv_i2c_wr_u16(ctx, ctx->s_ctx.reg_addr_ana_gain[0].addr[0], rg_gain & 0xffff);

	LOG_INF("08A3_rg_gain = 0x%x \n", rg_gain);
	return ERROR_NONE;
}

static int gc08a3_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	return gc08a3_set_shutter_frame_length(ctx, para, len);
}

static int gc08a3_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u32 shutter = *feature_data;
	u32 frame_length = *(feature_data + 1);
	u32 fine_integ_line = 0;

	LOG_INF("gc08a3_shutter = 0x%x \n", shutter);
	LOG_INF("gc08a3_frame_length = 0x%x \n", frame_length);
	ctx->frame_length = frame_length ? frame_length : ctx->frame_length;
	check_current_scenario_id_bound(ctx);
	/* check boundary of framelength */
	ctx->frame_length = max(shutter + ctx->s_ctx.exposure_margin, ctx->min_frame_length);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	/* check boundary of shutter */
	fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	shutter = FINE_INTEG_CONVERT(shutter, fine_integ_line);
	shutter = max(shutter, ctx->s_ctx.exposure_min);
	shutter = min(shutter, ctx->s_ctx.exposure_max);
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	ctx->exposure[0] = shutter;
	/* write framelength */
	subdrv_i2c_wr_u16(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0], ctx->frame_length & 0xfffe);
	/* write shutter */
	subdrv_i2c_wr_u16(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[0], ctx->exposure[0]);

	LOG_INF("gc08a3 exp[0x%x], fll(input/output):%u/%u\n",ctx->exposure[0], frame_length, ctx->frame_length);

	return ERROR_NONE;
}

static int gc08a3_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	bool enable = mode;

	if (enable != ctx->test_pattern) {
		LOG_INF("mode(%u->%u)\n", ctx->test_pattern, enable);
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

static __maybe_unused void gc08a3_sensor_init(struct subdrv_ctx *ctx)
{
	LOG_INF("gc08a3_sensor_init\n");
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x0337, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0335, 0x51);
	subdrv_i2c_wr_u8(ctx, 0x0336, 0x70);
	subdrv_i2c_wr_u8(ctx, 0x0383, 0xbb);
	subdrv_i2c_wr_u8(ctx, 0x031a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0321, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0327, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0325, 0x40);
	subdrv_i2c_wr_u8(ctx, 0x0326, 0x23);
	subdrv_i2c_wr_u8(ctx, 0x0314, 0x11);
	subdrv_i2c_wr_u8(ctx, 0x0315, 0xd6);
	subdrv_i2c_wr_u8(ctx, 0x0316, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0334, 0xc0);
	subdrv_i2c_wr_u8(ctx, 0x0324, 0x42);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x039a, 0x13);
	subdrv_i2c_wr_u8(ctx, 0x0084, 0x30);
	subdrv_i2c_wr_u8(ctx, 0x02b3, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x0057, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x05c3, 0x50);
	subdrv_i2c_wr_u8(ctx, 0x0311, 0x90);
	subdrv_i2c_wr_u8(ctx, 0x05a0, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x0074, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x0059, 0x11);
	subdrv_i2c_wr_u8(ctx, 0x0070, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x0101, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0344, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0345, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x0346, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0347, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0348, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x0349, 0xd0);
	subdrv_i2c_wr_u8(ctx, 0x034a, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x034b, 0x9c);
	subdrv_i2c_wr_u8(ctx, 0x0202, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x0203, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0340, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x0341, 0xf4);
	subdrv_i2c_wr_u8(ctx, 0x0342, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x0343, 0x1c);
	subdrv_i2c_wr_u8(ctx, 0x0219, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x0226, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0227, 0x28);
	subdrv_i2c_wr_u8(ctx, 0x0e0a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0e0b, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0e24, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0e25, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0e26, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0e27, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0e01, 0x74);
	subdrv_i2c_wr_u8(ctx, 0x0e03, 0x47);
	subdrv_i2c_wr_u8(ctx, 0x0e04, 0x33);
	subdrv_i2c_wr_u8(ctx, 0x0e05, 0x44);
	subdrv_i2c_wr_u8(ctx, 0x0e06, 0x44);
	subdrv_i2c_wr_u8(ctx, 0x0e0c, 0x1e);
	subdrv_i2c_wr_u8(ctx, 0x0e17, 0x3a);
	subdrv_i2c_wr_u8(ctx, 0x0e18, 0x3c);
	subdrv_i2c_wr_u8(ctx, 0x0e19, 0x40);
	subdrv_i2c_wr_u8(ctx, 0x0e1a, 0x42);
	subdrv_i2c_wr_u8(ctx, 0x0e28, 0x21);
	subdrv_i2c_wr_u8(ctx, 0x0e2b, 0x68);
	subdrv_i2c_wr_u8(ctx, 0x0e2c, 0x0d);
	subdrv_i2c_wr_u8(ctx, 0x0e2d, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x0e34, 0xf4);
	subdrv_i2c_wr_u8(ctx, 0x0e35, 0x44);
	subdrv_i2c_wr_u8(ctx, 0x0e36, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x0e38, 0x49);
	subdrv_i2c_wr_u8(ctx, 0x0210, 0x13);
	subdrv_i2c_wr_u8(ctx, 0x0218, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0241, 0x88);
	subdrv_i2c_wr_u8(ctx, 0x0e32, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0e33, 0x18);
	subdrv_i2c_wr_u8(ctx, 0x0e42, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0e43, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x0e44, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0e45, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0e4f, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x057a, 0x20);
	subdrv_i2c_wr_u8(ctx, 0x0381, 0x7c);
	subdrv_i2c_wr_u8(ctx, 0x0382, 0x9b);
	subdrv_i2c_wr_u8(ctx, 0x0384, 0xfb);
	subdrv_i2c_wr_u8(ctx, 0x0389, 0x38);
	subdrv_i2c_wr_u8(ctx, 0x038a, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0390, 0x6a);
	subdrv_i2c_wr_u8(ctx, 0x0391, 0x0b);
	subdrv_i2c_wr_u8(ctx, 0x0392, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x0393, 0xc1);
	subdrv_i2c_wr_u8(ctx, 0x0396, 0xff);
	subdrv_i2c_wr_u8(ctx, 0x0398, 0x62);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x0360, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0360, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0316, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x0a67, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x0313, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0a53, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0a65, 0x17);
	subdrv_i2c_wr_u8(ctx, 0x0a68, 0xa1);
	subdrv_i2c_wr_u8(ctx, 0x0a58, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0ace, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x00a4, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00a5, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x00a7, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x00a8, 0x9c);
	subdrv_i2c_wr_u8(ctx, 0x00a9, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x00aa, 0xd0);
	subdrv_i2c_wr_u8(ctx, 0x0a8a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0a8b, 0xe0);
	subdrv_i2c_wr_u8(ctx, 0x0a8c, 0x13);
	subdrv_i2c_wr_u8(ctx, 0x0a8d, 0xe8);
	subdrv_i2c_wr_u8(ctx, 0x0a90, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x0a91, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0a92, 0xf8);
	subdrv_i2c_wr_u8(ctx, 0x0a71, 0xf2);
	subdrv_i2c_wr_u8(ctx, 0x0a72, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x0a73, 0x64);
	subdrv_i2c_wr_u8(ctx, 0x0a75, 0x41);
	subdrv_i2c_wr_u8(ctx, 0x0a70, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x0313, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x00a0, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0080, 0xd2);
	subdrv_i2c_wr_u8(ctx, 0x0081, 0x3f);
	subdrv_i2c_wr_u8(ctx, 0x0087, 0x52);
	subdrv_i2c_wr_u8(ctx, 0x0089, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x009b, 0x40);
	subdrv_i2c_wr_u8(ctx, 0x05a0, 0x82);
	subdrv_i2c_wr_u8(ctx, 0x05ac, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x05ad, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x05ae, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0800, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x0801, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x0802, 0x28);
	subdrv_i2c_wr_u8(ctx, 0x0803, 0x34);
	subdrv_i2c_wr_u8(ctx, 0x0804, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0805, 0x33);
	subdrv_i2c_wr_u8(ctx, 0x0806, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0807, 0x8a);
	subdrv_i2c_wr_u8(ctx, 0x0808, 0x50);
	subdrv_i2c_wr_u8(ctx, 0x0809, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x080a, 0x34);
	subdrv_i2c_wr_u8(ctx, 0x080b, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x080c, 0x26);
	subdrv_i2c_wr_u8(ctx, 0x080d, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x080e, 0x18);
	subdrv_i2c_wr_u8(ctx, 0x080f, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0810, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0811, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0812, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0813, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0814, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0815, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0816, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0817, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0818, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0819, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x081a, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x081b, 0x6c);
	subdrv_i2c_wr_u8(ctx, 0x081c, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x081d, 0x0b);
	subdrv_i2c_wr_u8(ctx, 0x081e, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x081f, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0820, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0821, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x0822, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x0823, 0xd9);
	subdrv_i2c_wr_u8(ctx, 0x0824, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0825, 0x0d);
	subdrv_i2c_wr_u8(ctx, 0x0826, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0827, 0xf0);
	subdrv_i2c_wr_u8(ctx, 0x0828, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0829, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x082a, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x082b, 0x94);
	subdrv_i2c_wr_u8(ctx, 0x082c, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x082d, 0x6e);
	subdrv_i2c_wr_u8(ctx, 0x082e, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x082f, 0xe6);
	subdrv_i2c_wr_u8(ctx, 0x0830, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0831, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0832, 0x0b);
	subdrv_i2c_wr_u8(ctx, 0x0833, 0x2c);
	subdrv_i2c_wr_u8(ctx, 0x0834, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x0835, 0xae);
	subdrv_i2c_wr_u8(ctx, 0x0836, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x0837, 0xc4);
	subdrv_i2c_wr_u8(ctx, 0x0838, 0x18);
	subdrv_i2c_wr_u8(ctx, 0x0839, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x05ac, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x059a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x059b, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x059c, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0598, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0597, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x05ab, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x05a4, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x05a3, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x05a0, 0xc2);
	subdrv_i2c_wr_u8(ctx, 0x0207, 0xc4);
	subdrv_i2c_wr_u8(ctx, 0x0208, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0209, 0x72);
	subdrv_i2c_wr_u8(ctx, 0x0204, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0205, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0040, 0x22);
	subdrv_i2c_wr_u8(ctx, 0x0041, 0x20);
	subdrv_i2c_wr_u8(ctx, 0x0043, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0044, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0046, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x0047, 0xf0);
	subdrv_i2c_wr_u8(ctx, 0x0048, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x004b, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x004c, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0050, 0x5c);
	subdrv_i2c_wr_u8(ctx, 0x0051, 0x44);
	subdrv_i2c_wr_u8(ctx, 0x005b, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x00c0, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00c1, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x00c2, 0x31);
	subdrv_i2c_wr_u8(ctx, 0x00c3, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0460, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0462, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x0464, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0466, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x0468, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x046a, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x046c, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x046e, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x0461, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0463, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0465, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0467, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0469, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x046b, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x046d, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x046f, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0470, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0472, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0474, 0x26);
	subdrv_i2c_wr_u8(ctx, 0x0476, 0x38);
	subdrv_i2c_wr_u8(ctx, 0x0478, 0x20);
	subdrv_i2c_wr_u8(ctx, 0x047a, 0x30);
	subdrv_i2c_wr_u8(ctx, 0x047c, 0x38);
	subdrv_i2c_wr_u8(ctx, 0x047e, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x0471, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x0473, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x0475, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x0477, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x0479, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x047b, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x047d, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x047f, 0x04);

	LOG_INF("gc08a3_sensor_3264x2448 start \n");
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x0337, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0335, 0x51);
	subdrv_i2c_wr_u8(ctx, 0x0336, 0x70);
	subdrv_i2c_wr_u8(ctx, 0x0383, 0xbb);
	subdrv_i2c_wr_u8(ctx, 0x031a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0321, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0327, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0325, 0x40);
	subdrv_i2c_wr_u8(ctx, 0x0326, 0x23);
	subdrv_i2c_wr_u8(ctx, 0x0314, 0x11);
	subdrv_i2c_wr_u8(ctx, 0x0315, 0xd6);
	subdrv_i2c_wr_u8(ctx, 0x0316, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0334, 0xc0);
	subdrv_i2c_wr_u8(ctx, 0x0324, 0x42);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x0344, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0345, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x0346, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0347, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0348, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x0349, 0xd0);
	subdrv_i2c_wr_u8(ctx, 0x034a, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x034b, 0x9c);
	subdrv_i2c_wr_u8(ctx, 0x0202, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x0203, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0340, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x0341, 0xf4);
	subdrv_i2c_wr_u8(ctx, 0x0342, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x0343, 0x1c);
	subdrv_i2c_wr_u8(ctx, 0x0226, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0227, 0x28);
	subdrv_i2c_wr_u8(ctx, 0x0e38, 0x49);
	subdrv_i2c_wr_u8(ctx, 0x0210, 0x13);
	subdrv_i2c_wr_u8(ctx, 0x0218, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0241, 0x88);
	subdrv_i2c_wr_u8(ctx, 0x0392, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x03fe, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x031c, 0x9f);
	subdrv_i2c_wr_u8(ctx, 0x00a2, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00a3, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00ab, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00ac, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x05a0, 0x82);
	subdrv_i2c_wr_u8(ctx, 0x05ac, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x05ad, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x05ae, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0800, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x0801, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x0802, 0x28);
	subdrv_i2c_wr_u8(ctx, 0x0803, 0x34);
	subdrv_i2c_wr_u8(ctx, 0x0804, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0805, 0x33);
	subdrv_i2c_wr_u8(ctx, 0x0806, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0807, 0x8a);
	subdrv_i2c_wr_u8(ctx, 0x0808, 0x50);
	subdrv_i2c_wr_u8(ctx, 0x0809, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x080a, 0x34);
	subdrv_i2c_wr_u8(ctx, 0x080b, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x080c, 0x26);
	subdrv_i2c_wr_u8(ctx, 0x080d, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x080e, 0x18);
	subdrv_i2c_wr_u8(ctx, 0x080f, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0810, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0811, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0812, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0813, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0814, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0815, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0816, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0817, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0818, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0819, 0x0a);
	subdrv_i2c_wr_u8(ctx, 0x081a, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x081b, 0x6c);
	subdrv_i2c_wr_u8(ctx, 0x081c, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x081d, 0x0b);
	subdrv_i2c_wr_u8(ctx, 0x081e, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x081f, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0820, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0821, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x0822, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x0823, 0xd9);
	subdrv_i2c_wr_u8(ctx, 0x0824, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0825, 0x0d);
	subdrv_i2c_wr_u8(ctx, 0x0826, 0x03);
	subdrv_i2c_wr_u8(ctx, 0x0827, 0xf0);
	subdrv_i2c_wr_u8(ctx, 0x0828, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0829, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x082a, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x082b, 0x94);
	subdrv_i2c_wr_u8(ctx, 0x082c, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x082d, 0x6e);
	subdrv_i2c_wr_u8(ctx, 0x082e, 0x07);
	subdrv_i2c_wr_u8(ctx, 0x082f, 0xe6);
	subdrv_i2c_wr_u8(ctx, 0x0830, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0831, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x0832, 0x0b);
	subdrv_i2c_wr_u8(ctx, 0x0833, 0x2c);
	subdrv_i2c_wr_u8(ctx, 0x0834, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x0835, 0xae);
	subdrv_i2c_wr_u8(ctx, 0x0836, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x0837, 0xc4);
	subdrv_i2c_wr_u8(ctx, 0x0838, 0x18);
	subdrv_i2c_wr_u8(ctx, 0x0839, 0x0e);
	subdrv_i2c_wr_u8(ctx, 0x05ac, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x059a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x059b, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x059c, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0598, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0597, 0x14);
	subdrv_i2c_wr_u8(ctx, 0x05ab, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x05a4, 0x02);
	subdrv_i2c_wr_u8(ctx, 0x05a3, 0x05);
	subdrv_i2c_wr_u8(ctx, 0x05a0, 0xc2);
	subdrv_i2c_wr_u8(ctx, 0x0207, 0xc4);
	subdrv_i2c_wr_u8(ctx, 0x0204, 0x04);
	subdrv_i2c_wr_u8(ctx, 0x0205, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0050, 0x5c);
	subdrv_i2c_wr_u8(ctx, 0x0051, 0x44);
	subdrv_i2c_wr_u8(ctx, 0x009a, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0351, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0352, 0x06);
	subdrv_i2c_wr_u8(ctx, 0x0353, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0354, 0x08);
	subdrv_i2c_wr_u8(ctx, 0x034c, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x034d, 0xc0);
	subdrv_i2c_wr_u8(ctx, 0x034e, 0x09);
	subdrv_i2c_wr_u8(ctx, 0x034f, 0x90);
	subdrv_i2c_wr_u8(ctx, 0x0114, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0180, 0x67);
	subdrv_i2c_wr_u8(ctx, 0x0181, 0x30);
	subdrv_i2c_wr_u8(ctx, 0x0185, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0115, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x011b, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x011c, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x0121, 0x0b);
	subdrv_i2c_wr_u8(ctx, 0x0122, 0x0d);
	subdrv_i2c_wr_u8(ctx, 0x0123, 0x2f);
	subdrv_i2c_wr_u8(ctx, 0x0124, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0125, 0x12);
	subdrv_i2c_wr_u8(ctx, 0x0126, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x0129, 0x0c);
	subdrv_i2c_wr_u8(ctx, 0x012a, 0x13);
	subdrv_i2c_wr_u8(ctx, 0x012b, 0x0f);
	subdrv_i2c_wr_u8(ctx, 0x0a73, 0x60);
	subdrv_i2c_wr_u8(ctx, 0x0a70, 0x11);
	subdrv_i2c_wr_u8(ctx, 0x0313, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0aff, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0a70, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x00a4, 0x80);
	subdrv_i2c_wr_u8(ctx, 0x0316, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x0a67, 0x00);
	subdrv_i2c_wr_u8(ctx, 0x0084, 0x10);
	subdrv_i2c_wr_u8(ctx, 0x0102, 0x09);

	LOG_INF("gc08a3_sensor_init end\n");
}

static __maybe_unused int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;
	LOG_INF("open start");

	/* get sensor id */
	if (gc08a3_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	gc08a3_sensor_init(ctx);

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

static void gc08a3_set_streaming_control(struct subdrv_ctx *ctx, bool enable)
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
				LOG_INF("please implement drive own streaming control!(sid:%u)\n",ctx->current_scenario_id);

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
			subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x01);
		} else {
			subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x00);
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
		mdelay(30);
		LOG_INF("reg_addr_stream enable=%d 0x0100 = 0x%x",enable,subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_stream));
		ctx->sof_no = 0;
		ctx->is_streaming = enable;
		LOG_INF("X! enable:%u\n", enable);
}

static int gc08a3_set_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	gc08a3_set_streaming_control(ctx, TRUE);
	return 0;
}
static int gc08a3_set_streaming_suspned(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	gc08a3_set_streaming_control(ctx, FALSE);
	return 0;
}
