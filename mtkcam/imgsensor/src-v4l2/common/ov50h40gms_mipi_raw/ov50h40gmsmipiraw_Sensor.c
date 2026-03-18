// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 ov50h40gmsmipiraw_Sensor.c
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
#include "ov50h40gmsmipiraw_Sensor.h"

static void set_sensor_cali(void *arg);

static void set_group_hold(void *arg, u8 en);
//static void ov50h40gms_set_dummy(struct subdrv_ctx *ctx);
//static int ov50h40gms_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static u16 get_gain2reg(u32 gain);
static int ov50h40gms_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static void ov50h40gms_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
//drv add by lipengpeng 20241018 start 
static int ov50h40gms_set_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int ov50h40gms_set_streaming_suspned(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int close(struct subdrv_ctx *ctx);
//drv add by lipengpeng 20241018 end  

//drv add by lipengpeng 20240603 start 
#if IS_ENABLED(CONFIG_DRV_ADC_GMS_DETECT)
//#include "../../../../../../../../kernel/kernel_device_modules-6.1/drivers/misc/mediatek/prize/gms_adc/adc_detect_gms.h"
#include "../../../../../../kernel/kernel_device_modules-6.1/drivers/misc/mediatek/pri/gms_adc/adc_detect_gms.h"
#endif
//drv add by lipengpeng 20240603 end 

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, ov50h40gms_set_test_pattern},
//drv add by lipengpeng 20241018 start 
	{SENSOR_FEATURE_SET_STREAMING_RESUME, ov50h40gms_set_streaming_resume},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, ov50h40gms_set_streaming_suspned},
//drv add by lipengpeng 20241018 end 

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
		.xtalk_table = data_xtalk_ov50h40gms2q,
		.xtalk_size = ARRAY_SIZE(data_xtalk_ov50h40gms2q),
		.addr_xtalk = PARAM_UNDEFINED,
		.sensor_reg_addr_xtalk = 0x53C0,
	},
};
/*
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	 .i4OffsetX = 16,
	 .i4OffsetY = 4,
	 .i4PitchX = 16,
	 .i4PitchY = 16,
	 .i4PairNum = 8,
	 .i4SubBlkW = 8,
	 .i4SubBlkH = 4,
	 .i4PosL = {{23, 6}, {31, 6}, {19, 10}, {27, 10},
		{23, 14}, {31, 14}, {19, 18}, {27, 18} },
	 .i4PosR = {{22, 6}, {30, 6}, {18, 10}, {26, 10},
		{22, 14}, {30, 14}, {18, 18}, {26, 18} },
	 .iMirrorFlip = 0,
	 .i4BlockNumX = 248,
	 .i4BlockNumY = 187,
	 .i4Crop = { {0, 0}, {0, 0}, {0, 200}, {0, 0}, {0, 0},
			 {0, 0}, {80, 420}, {0, 0}, {0, 0}, {0, 0} },
	 .i4VCPackNum = 2,
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_vid = {
	 .i4OffsetX = 16,
	 .i4OffsetY = 4,
	 .i4PitchX = 16,
	 .i4PitchY = 16,
	 .i4PairNum = 8,
	 .i4SubBlkW = 8,
	 .i4SubBlkH = 4,
	 .i4PosL = {{23, 6}, {31, 6}, {19, 10}, {27, 10},
		{23, 14}, {31, 14}, {19, 18}, {27, 18} },
	 .i4PosR = {{22, 6}, {30, 6}, {18, 10}, {26, 10},
		{22, 14}, {30, 14}, {18, 18}, {26, 18} },
	 .iMirrorFlip = 0,
	 .i4BlockNumX = 248,
	 .i4BlockNumY = 162,
	 .i4Crop = { {0, 0}, {0, 0}, {0, 200}, {0, 0}, {0, 0},
			 {0, 0}, {80, 420}, {0, 0}, {0, 0}, {0, 0} },
	 .i4VCPackNum = 2,
};
*/


static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,  //0x30 no open camera
			.hsize = 0x1000, //4096
			.vsize = 0x0C00, //3072
		},
	},
	//{
	//	.bus.csi2 = {
	//		.channel = 1,
	//		.data_type = 0x2b,
	//		.hsize = 0x01f0,
	//		.vsize = 0x05d8,
	//		.user_data_desc = VC_PDAF_STATS,
	//	},
	//},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,//0x30 no open camera
			.hsize = 0x1000, //4096
			.vsize = 0x0C00, //3072
		},
	},
	//{
	//	.bus.csi2 = {
	//		.channel = 1,
	//		.data_type = 0x2b,
	//		.hsize = 0x01f0,
	//		.vsize = 0x05d8,
	//		.user_data_desc = VC_PDAF_STATS,
	//	},
	//},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b, //0x30 no open camera
			.hsize = 0x1000, //4096
			.vsize = 0x0C00, //3072
		},
	},
	//{
	//	.bus.csi2 = {
	//		.channel = 1,
	//		.data_type = 0x2b,
	//		.hsize = 0x01f0,
	//		.vsize = 0x0510,
	//		.user_data_desc = VC_PDAF_STATS,
	//	},
	//},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b, //0x30 no open camera
			.hsize = 0x0780, //1920
			.vsize = 0x0438, //1080
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b, //0x30 no open camera
			.hsize = 0x0800, //2048
			.vsize = 0x0600, //1536
		},
	},
};

//.pclk = 300000000,
//.linelength = 1800, 
//.framelength = 5552, 
//.startx = 0,
//.starty = 0,
//.grabwindow_width = 4096,
//.grabwindow_height = 3072,
//.mipi_data_lp2hs_settle_dc = 85,	//unit(ns), 16/23/65/85 recommanded
//.mipi_pixel_rate = 1750000000,
//.max_framerate = 300, 8192,  6144

		
static struct subdrv_mode_struct mode_struct[] = {
//drv add by lipengpeng 20240815 start
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = addr_data_pair_preview_ov50h40gms2q,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_preview_ov50h40gms2q),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 300000000,
		.linelength = 1800,
		.framelength = 5552,
		.max_framerate = 300,
		.mipi_pixel_rate = 512000000,
		.readout_length = 0,
		.read_margin = 0,
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
			.cphy_settle = 98,
		},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = addr_data_pair_capture_ov50h40gms2q,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_capture_ov50h40gms2q),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = addr_data_pair_capture_ov50h40gms2q,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(addr_data_pair_capture_ov50h40gms2q),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 300000000,
		.linelength = 1800,
		.framelength = 5552,
		.max_framerate = 300,
		.mipi_pixel_rate = 512000000,
		.readout_length = 0,
		.read_margin = 0,
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
			.cphy_settle = 98,
		},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = addr_data_pair_video_ov50h40gms2q,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_video_ov50h40gms2q),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 300000000,
		.linelength = 1800,
		.framelength = 5552,
		.max_framerate = 300,
		.mipi_pixel_rate = 512000000,
		.readout_length = 0,
		.read_margin = 0,
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
			.cphy_settle = 98,
		},
	},
	{
//.pclk = 100000000,
//.linelength = 726,
//.framelength = 1147,
//.startx = 0,
//.starty = 0,
//.grabwindow_width = 1920,
//.grabwindow_height = 1080,
//.mipi_data_lp2hs_settle_dc = 85,
//.mipi_pixel_rate = 512000000,
//.max_framerate = 1200, 8192,  6144
		
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = addr_data_pair_hs_video_ov50h40gms2q,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_hs_video_ov50h40gms2q),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 300000000,
		.linelength = 1800,
		.framelength = 5552,
		.max_framerate = 1200,
		.mipi_pixel_rate = 512000000,
		.readout_length = 0,
		.read_margin = 0,
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
		
		//.s_dummy_support = 1,
		//.ae_ctrl_support = 1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 98,
		},
	},
	{
//.pclk = 100000000,
//.linelength = 520,
//.framelength = 3204,
//.startx = 0,
//.starty = 0,
//.grabwindow_width = 2048,
//.grabwindow_height = 1536,
//.mipi_data_lp2hs_settle_dc = 85,	//unit(ns), 16/23/65/85 recommanded
//.mipi_pixel_rate = 486400000,
//.max_framerate = 600,
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = addr_data_pair_slim_video_ov50h40gms2q,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_slim_video_ov50h40gms2q),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 520,
		.framelength = 3204,
		.max_framerate = 600,
		.mipi_pixel_rate = 486400000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 2048,
			.scale_h = 1536,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1536,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1536,
		},
		
		//.s_dummy_support = 1,
		//.ae_ctrl_support = 1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 98,
		},
	},
//drv add by lipengpeng 20240815 end 
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = OV50H40GMS_SENSOR_ID,
	.reg_addr_sensor_id = {0x300A, 0x300B, 0x300C}, //drv ad by lipengpeng (((read_cmos_sensor(0x300a) << 16) |(read_cmos_sensor(0x300b) << 8) | read_cmos_sensor(0x300c)));
	.i2c_addr_table = {0x44,0x46, 0xff}, //drv add by lipengpeng 20240815 
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8, //drv add by lipengpeng 20240815  add16 data8
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {8192, 6144}, //drv add by lipengpeng 20240815 
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,//drv lipengpeng cphy
	.mipi_lane_num = SENSOR_MIPI_3_LANE,  //drv lipengpeng cphy
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 15.5,
	.ana_gain_type = 1,
	.ana_gain_step = 4,
	.ana_gain_table = ov50h40gms_ana_gain_table,
	.ana_gain_table_size = sizeof(ov50h40gms_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 8,
	.exposure_max = 0xFFFFFF - 160,
	.exposure_step = 2,
	.exposure_margin = 160,

	.frame_length_max = 0xFFFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 500000,
//drv add by lipengpeng 20240815 start 
	.pdaf_type = PDAF_SUPPORT_NA, //PDAF_SUPPORT_NA PDAF_SUPPORT_CAMSV
	.hdr_type = HDR_SUPPORT_NA,
	//.seamless_switch_support = TRUE,
	//.temperature_support = TRUE,
	//.g_temp = get_sensor_temperature,
//drv add by lipengpeng 20240815 end 		
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,
	.s_cali = set_sensor_cali,

	.reg_addr_stream = 0x0100,  //write_cmos_sensor(0x0100,0x01);
	.reg_addr_mirror_flip = PARAM_UNDEFINED,
	.reg_addr_exposure = {{0x3500, 0x3501, 0x3502},}, //write_cmos_sensor(0x3500, (yft_shutter >> 16) & 0xFF); 
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x3508, 0x3509},},  //	write_cmos_sensor(0x03508, (reg_gain >> 8)); write_cmos_sensor(0x03509, (reg_gain & 0xff));
	.reg_addr_frame_length = {0x3840, 0x380E, 0x380F}, //		write_cmos_sensor(0x3840, yft_frame_length >> 16);write_cmos_sensor(0x380e, yft_frame_length >> 8); write_cmos_sensor(0x380f, yft_frame_length & 0xFF);
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
//drv add by lipengpeng 20240815 start 
	.checksum_value = 0x388C7147,
	//.aov_sensor_support = TRUE,
	//.init_in_open = TRUE,
	//.streaming_ctrl_imp = FALSE,
//drv add by lipengpeng 20240815 end 
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
   printk("ov50h40gms ---- gms_detection_getadc_v()=%d\n",gms_detection_getadc_v());
  if(check_board_type()==1)
    {
	 printk(" ov50h40gms is volume production\n");
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
			*sensor_id +=1;
			printk("ov50h40gms i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == OV50H40GMS_SENSOR_ID) {
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
//drv add by lipengpeng 20241018 start 
	.close = close,
//drv add by lipengpeng 20241018 end 
	.get_frame_desc = common_get_frame_desc,
//drv add by lipengpeng 20240815 start 
	//.get_temp = common_get_temp,
//drv add by lipengpeng 20240815 end 
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	//{HW_ID_RST, 0, 1},
	{HW_ID_PDN, 0, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 2},
	{HW_ID_AVDD, 2800000, 2}, // pmic_ldo for avdd
	{HW_ID_DOVDD, 1800000, 2}, // pmic_ldo/gpio(1.8V ldo) for dovdd
	{HW_ID_DVDD, 1090000, 5}, // pmic_ldo for dvdd
	{HW_ID_PDN, 1, 1},
	//{HW_ID_RST, 1, 5},

};

const struct subdrv_entry ov50h40gms_mipi_raw_entry = {
	.name = "ov50h40gms_mipi_raw",
	.id = OV50H40GMS_SENSOR_ID,
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
	iReg = gain * 256 / BASEGAIN;

	// sensor 1xGain
	if (iReg < 0x100)
		iReg = 0X100;

	// sensor 15.5xGain
	if (iReg > 0xF80)
		iReg = 0XF80;

	return iReg;
	//return gain * 256 / BASEGAIN;
}

static int ov50h40gms_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

//	if (mode != ctx->test_pattern)
//		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
//	if (mode)
//		subdrv_i2c_wr_u8(ctx, 0x50c1, 0x81);
//	else if (ctx->test_pattern)
//		subdrv_i2c_wr_u8(ctx, 0x50c1, 0x00);

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
static void ov50h40gms_sensor_init(struct subdrv_ctx *ctx)
{

	printk("ov50h40gms_sensor_init\n");
//drv add by lipengpeng 20240828 start 
	subdrv_i2c_wr_u8(ctx, 0x0103, 0x01);	//SW Reset, need delay
	mdelay(10);
//drv add by lipengpeng 20240828 end  
   i2c_table_write(ctx, sensor_gms_init_addr_data, sizeof(sensor_gms_init_addr_data)/sizeof(u16));
   
	printk("ov50h40gms_sensor_init end\n");
}
//drv add by lipengpeng 20241018 start 
int close(struct subdrv_ctx *ctx)
{
	subdrv_i2c_wr_u8(ctx, 0x0100, 0x00);
	DRV_LOG(ctx, "subdrv closed\n");
	return ERROR_NONE;
}
//drv add by lipengpeng 20241018 end 
static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	ov50h40gms_sensor_init(ctx);

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
//drv add by lipengpeng 20241018 start 
static void ov50h40gms_set_streaming_control(struct subdrv_ctx *ctx, bool enable)
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
	
   // subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);	 //// Page pointer HW
	
	if (enable) {
		set_dummy(ctx);
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x01);
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
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x00);
		
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
	mdelay(10);
	ctx->sof_no = 0;
	ctx->is_streaming = enable;
	DRV_LOG(ctx, "X! enable:%u\n", enable);
}

static int ov50h40gms_set_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	ov50h40gms_set_streaming_control(ctx, TRUE);
	return 0;
}
static int ov50h40gms_set_streaming_suspned(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	ov50h40gms_set_streaming_control(ctx, FALSE);
	return 0;
}
//drv add by lipengpeng 20241018 end 