/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 gc02m1macro_Sensor_setting.h
 *
 * Project:
 * --------
 * Description:
 * ------------
 *	 CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _GC02M1MACRO_SENSOR_SETTING_H
#define _GC02M1MACRO_SENSOR_SETTING_H

#include "gc02m1macromipiraw_Sensor.h"
#include "kd_camera_typedef.h"

/* SENSOR PRIVATE INFO FOR GAIN SETTING */
#define GC02M1_SENSOR_GAIN_BASE             0x400
#define GC02M1_SENSOR_GAIN_MAX              (12 * GC02M1_SENSOR_GAIN_BASE)
#define GC02M1_SENSOR_GAIN_MAX_VALID_INDEX  16
#define GC02M1_SENSOR_GAIN_MAP_SIZE         16
#define GC02M1_SENSOR_DGAIN_BASE            0x400

static kal_uint16 gc02m1macrowide_init_addr_data[] = {
	/*system*/
};
static kal_uint16 gc02m1macrowide_fullsize_addr_data[] = {
	   /*system*/
};
static kal_uint16 gc02m1macrowide_binning_addr_data[] = {
	/*system*/
};

#endif
