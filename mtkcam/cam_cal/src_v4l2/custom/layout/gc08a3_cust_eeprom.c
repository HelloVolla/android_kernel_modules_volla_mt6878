// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "cam_cal_config.h"

// prize add by chenwenhui for otp start
#define GC08A3_OTP_FOR_MTK 1

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
#endif
// prize add by chenwenhui for otp end


static unsigned int do_single_lsc_gc08a3(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_2a_gain_gc08a3(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);

static struct STRUCT_CALIBRATION_LAYOUT_STRUCT cal_layout_table = {
	0x00000768, 0x000008a3, CAM_CAL_SINGLE_EEPROM_DATA,
	{
		{0x00000001, 0x00000000, 0x00000000, do_module_version},
		{0x00000001, 0x00000001, 0x00000002, do_part_number},
		{0x00000001, 0x0000077B, 0x0000074C, do_single_lsc_gc08a3},
		{0x00000001, 0x00000007, 0x0000000E, do_2a_gain_gc08a3},
		{0x00000000, 0x00000820, 0x000005F9, do_pdaf},
		{0x00000000, 0x00000FAE, 0x00000550, do_stereo_data},
		{0x00000001, 0x00000000, 0x00001600, do_dump_all},
		{0x00000001, 0x00000008, 0x00000002, do_lens_id}
	}
};

struct STRUCT_CAM_CAL_CONFIG_STRUCT gc08a3_cust_eeprom = {
	.name = "gc08a3_cust_eeprom",
	.check_layout_function = gc08a3_layout_check,
	.read_function = Common_read_region,
	.layout = &cal_layout_table,
	.sensor_id = GC08A3_SENSOR_ID,
	.i2c_write_id = 0x62,
	.max_size = 0x4000,
	.enable_preload = 1,
	.preload_size = 0x1500,
	.has_stored_data = 1,
};

static int read_data_gc08a3(struct EEPROM_DRV_FD_DATA *pdata, unsigned int sensor_id, unsigned int device_id,
		unsigned int offset, unsigned int length, unsigned char *data)
{
	memcpy(data, gc08a3_otp_info.lsc, length);
	debug_log("read_data_gc08a3 length  %d\n", length);
	return length;
}

static unsigned int do_single_lsc_gc08a3(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;

	int read_data_size;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];
	unsigned short table_size;

	if (pCamCalData->DataVer >= CAM_CAL_TYPE_NUM) {
		err = CAM_CAL_ERR_NO_DEVICE;
		error_log("Read Failed\n");
		show_cmd_error_log(pCamCalData->Command);
		return err;
	}

	if (block_size != CAM_CAL_SINGLE_LSC_SIZE)
		error_log("block_size(%d) is not match (%d)\n",
				block_size, CAM_CAL_SINGLE_LSC_SIZE);

	pCamCalData->SingleLsc.LscTable.MtkLcsData.MtkLscType = 2;//mtk type
	pCamCalData->SingleLsc.LscTable.MtkLcsData.PixId = 8;

	table_size = 1868;

	debug_log("lsc table_size %d\n", table_size);
	pCamCalData->SingleLsc.LscTable.MtkLcsData.TableSize = table_size;
	if (table_size > 0) {
		pCamCalData->SingleLsc.TableRotation = 0;
		debug_log("u4Offset=%d u4Length=%d", start_addr, table_size);
		read_data_size = read_data_gc08a3(pdata,
			pCamCalData->sensorID, pCamCalData->deviceID,
			start_addr, table_size, (unsigned char *)
			&pCamCalData->SingleLsc.LscTable.MtkLcsData.SlimLscType);

		if (table_size == read_data_size)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			error_log("Read Failed\n");
			err = CamCalReturnErr[pCamCalData->Command];
			show_cmd_error_log(pCamCalData->Command);
		}
	}

	debug_log("======================SingleLsc Data==================\n");
	debug_log("[1st] = %x, %x, %x, %x\n",
		pCamCalData->SingleLsc.LscTable.Data[0],
		pCamCalData->SingleLsc.LscTable.Data[1],
		pCamCalData->SingleLsc.LscTable.Data[2],
		pCamCalData->SingleLsc.LscTable.Data[3]);
	debug_log("[1st] = SensorLSC(1)?MTKLSC(2)?  %x\n",
		pCamCalData->SingleLsc.LscTable.MtkLcsData.MtkLscType);
	debug_log("CapIspReg =0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[0],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[1],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[2],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[3],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[4]);
	debug_log("RETURN = 0x%x\n", err);
	debug_log("======================SingleLsc Data==================\n");

	return err;
}

static unsigned int do_2a_gain_gc08a3(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];

	unsigned char AWBAFConfig = 0xf;

	int tempMax = 0;
	int CalR = 1, CalGr = 1, CalGb = 1, CalG = 1, CalB = 1;
	int FacR = 1, FacGr = 1, FacGb = 1, FacG = 1, FacB = 1;

	(void) start_addr;
	(void) block_size;

	debug_log("block_size=%d sensor_id=%x\n", block_size, pCamCalData->sensorID);
	memset((void *)&pCamCalData->Single2A, 0, sizeof(struct STRUCT_CAM_CAL_SINGLE_2A_STRUCT));
	/* Check rule */
	if (pCamCalData->DataVer >= CAM_CAL_TYPE_NUM) {
		err = CAM_CAL_ERR_NO_DEVICE;
		error_log("Read Failed\n");
		show_cmd_error_log(pCamCalData->Command);
		return err;
	}
	/* Check AWB & AF enable bit */
	pCamCalData->Single2A.S2aVer = 0x01;
	pCamCalData->Single2A.S2aBitEn = (0x03 & AWBAFConfig);
	pCamCalData->Single2A.S2aAfBitflagEn = (0x0C & AWBAFConfig);
	debug_log("S2aBitEn=0x%02x", pCamCalData->Single2A.S2aBitEn);

	/* AWB Calibration Data*/
	if (0x1 & AWBAFConfig) {
		pCamCalData->Single2A.S2aAwb.rGainSetNum = 0;

		CalR = (gc08a3_otp_info.awb.unit_r_h << 8) | gc08a3_otp_info.awb.unit_r_l;
		CalGr = (gc08a3_otp_info.awb.unit_gr_h << 8) | gc08a3_otp_info.awb.unit_gr_l;
		CalGb = (gc08a3_otp_info.awb.unit_gb_h << 8) | gc08a3_otp_info.awb.unit_gb_l;
		CalG = ((CalGr + CalGb) + 1) >> 1;
		CalB = (gc08a3_otp_info.awb.unit_b_h << 8) | gc08a3_otp_info.awb.unit_b_l;

		debug_log("gc08a3 otp CalR:%d, CalGr:%d, CalGb:%d, CalB=%d",
					CalR, CalGr, CalGb, CalB);

		if (CalR != 0 && CalGr != 0 && CalGb != 0 && CalB != 0)	{

			if (CalR > CalG) {
				/* R > G */
				if (CalR > CalB)
					tempMax = CalR;
				else
					tempMax = CalB;
			} else {
				/* G > R */
				if (CalG > CalB)
					tempMax = CalG;
				else
					tempMax = CalB;
			}
			debug_log(
					"UnitR:%d, UnitG:%d, UnitB:%d, New Unit Max=%d",
					CalR, CalG, CalB, tempMax);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read CalGain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}

		if (CalR && CalG && CalB) {
			pCamCalData->Single2A.S2aAwb.rGainSetNum++;
			pCamCalData->Single2A.S2aAwb.rUnitGainu4R =
					(unsigned int)((tempMax * 512 + (CalR >> 1)) / CalR);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4G =
					(unsigned int)((tempMax * 512 + (CalG >> 1)) / CalG);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4B =
					(unsigned int)((tempMax * 512 + (CalB >> 1)) / CalB);
		} else
			error_log("Something wrong on EEPROM, plz contact module vendor\n");


		FacR = (gc08a3_otp_info.awb.golden_r_h << 8) | gc08a3_otp_info.awb.golden_r_l;
		FacGr = (gc08a3_otp_info.awb.golden_gr_h << 8) | gc08a3_otp_info.awb.golden_gr_l;
		FacGb = (gc08a3_otp_info.awb.golden_gb_h << 8) | gc08a3_otp_info.awb.golden_gb_l;
		FacG = ((FacGr + FacGb) + 1) >> 1;
		FacB = (gc08a3_otp_info.awb.golden_b_h << 8) | gc08a3_otp_info.awb.golden_b_l;

		debug_log("gc08a3 otp FacR:%d, FacGr:%d, FacGb:%d, FacB=%d",
					FacR, FacGr, FacGb, FacB);

		if (FacR != 0 && FacGr != 0 && FacGb != 0 && FacB != 0)	{
			debug_log("Read FacGain OK\n");

			if (FacR > FacG) {
				if (FacR > FacB)
					tempMax = FacR;
				else
					tempMax = FacB;
			} else {
				if (FacG > FacB)
					tempMax = FacG;
				else
					tempMax = FacB;
			}
			debug_log(
					"GoldenR:%d, GoldenG:%d, GoldenB:%d, New Golden Max=%d",
					FacR, FacG, FacB, tempMax);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read FacGain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}

		if (FacR && FacG && FacB) {
			pCamCalData->Single2A.S2aAwb.rGoldGainu4R =
					(unsigned int)((tempMax * 512 + (FacR >> 1)) / FacR);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4G =
					(unsigned int)((tempMax * 512 + (FacG >> 1)) / FacG);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4B =
					(unsigned int)((tempMax * 512 + (FacB >> 1)) / FacB);
		} else
			error_log("Something wrong on EEPROM, plz contact module vendor\n");

		/* Set AWB to 3A Layer */
		pCamCalData->Single2A.S2aAwb.rValueR   = CalR;
		pCamCalData->Single2A.S2aAwb.rValueGr  = CalGr;
		pCamCalData->Single2A.S2aAwb.rValueGb  = CalGb;
		pCamCalData->Single2A.S2aAwb.rValueB   = CalB;
		pCamCalData->Single2A.S2aAwb.rGoldenR  = FacR;
		pCamCalData->Single2A.S2aAwb.rGoldenGr = FacGr;
		pCamCalData->Single2A.S2aAwb.rGoldenGb = FacGb;
		pCamCalData->Single2A.S2aAwb.rGoldenB  = FacB;

		debug_log("======================AWB CAM_CAL==================\n");

		debug_log("[rCalGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4R);
		debug_log("[rCalGain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4G);
		debug_log("[rCalGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4B);
		debug_log("[rFacGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4R);
		debug_log("[rFacGain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4G);
		debug_log("[rFacGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4B);
	}
	return err;
}
