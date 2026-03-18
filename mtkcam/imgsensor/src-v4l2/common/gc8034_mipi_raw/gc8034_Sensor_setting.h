/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 gc8034_Sensor_setting.h
 *
 * Project:
 * --------
 * Description:
 * ------------
 *	 CMOS sensor header file
 *
 ****************************************************************************/
#include "gc8034mipiraw_Sensor.h"
#ifndef _GC8034_SENSOR_SETTING_H
#define _GC8034_SENSOR_SETTING_H

#define SEQUENTIAL_WRITE_EN 1
//#define XTALK_OTP_ADDR 0x53C0

#define read_cmos_sensor_8(...) subdrv_i2c_rd_addr8_u8(__VA_ARGS__)
#define write_cmos_sensor_8(...) subdrv_i2c_wr_addr8_u8(__VA_ARGS__)
static kal_uint8 gamma_index = 1;
static void addr_data_pair_preview_gc8034(struct subdrv_ctx *ctx){
	
/*	printk("Enter preview_setting!\n");
	
	write_cmos_sensor_8(ctx, 0xfc, 0x01);
	write_cmos_sensor_8(ctx, 0xf2, 0x00);
	write_cmos_sensor_8(ctx, 0xf4, 0x80);
	write_cmos_sensor_8(ctx, 0xf5, 0x19);
	write_cmos_sensor_8(ctx, 0xf6, 0x44);
	if (ctx->current_fps == 240) {
		write_cmos_sensor_8(ctx, 0xf8, 0x4f);
		write_cmos_sensor_8(ctx, 0xfa, 0x36);
		gamma_index=1;
	} else {
		write_cmos_sensor_8(ctx, 0xf8, 0x63);
		write_cmos_sensor_8(ctx, 0xfa, 0x45);
		gamma_index=0;
	}
	write_cmos_sensor_8(ctx, 0xf9, 0x00);
	write_cmos_sensor_8(ctx, 0xf7, 0x95);
	write_cmos_sensor_8(ctx, 0xfc, 0x03);
	write_cmos_sensor_8(ctx, 0xfc, 0xea);
	write_cmos_sensor_8(ctx, 0xfe, 0x03);
	write_cmos_sensor_8(ctx, 0x03, 0x9a);
	write_cmos_sensor_8(ctx, 0x18, 0x07);
	write_cmos_sensor_8(ctx, 0x01, 0x07);
	write_cmos_sensor_8(ctx, 0xfc, 0xee);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0x80, 0x13);
	write_cmos_sensor_8(ctx, 0xad, 0x00);
	write_cmos_sensor_8(ctx, 0x90, 0x01);
	write_cmos_sensor_8(ctx, 0x92, FullStartY);
	write_cmos_sensor_8(ctx, 0x94, FullStartX);
	write_cmos_sensor_8(ctx, 0x95, 0x09);
	write_cmos_sensor_8(ctx, 0x96, 0x90);
	write_cmos_sensor_8(ctx, 0x97, 0x0c);
	write_cmos_sensor_8(ctx, 0x98, 0xc0);
	write_cmos_sensor_8(ctx, 0xfe, 0x01);
	write_cmos_sensor_8(ctx, 0x62, 0x60);
	write_cmos_sensor_8(ctx, 0x63, 0x48);
	write_cmos_sensor_8(ctx, 0xfe, 0x03);
	write_cmos_sensor_8(ctx, 0x02, 0x03);
	write_cmos_sensor_8(ctx, 0x04, 0x80);
	write_cmos_sensor_8(ctx, 0x11, 0x2b);
	write_cmos_sensor_8(ctx, 0x12, 0xf0);
	write_cmos_sensor_8(ctx, 0x13, 0x0f);
	write_cmos_sensor_8(ctx, 0x15, 0x10);
	write_cmos_sensor_8(ctx, 0x16, 0x29);
	write_cmos_sensor_8(ctx, 0x17, 0xff);
	write_cmos_sensor_8(ctx, 0x19, 0xaa);
	write_cmos_sensor_8(ctx, 0x1a, 0x02);
	write_cmos_sensor_8(ctx, 0x21, 0x05);
	write_cmos_sensor_8(ctx, 0x22, 0x06);
	write_cmos_sensor_8(ctx, 0x23, 0x16);
	write_cmos_sensor_8(ctx, 0x24, 0x00);
	write_cmos_sensor_8(ctx, 0x25, 0x12);
	write_cmos_sensor_8(ctx, 0x26, 0x07);
	write_cmos_sensor_8(ctx, 0x29, 0x07);
	write_cmos_sensor_8(ctx, 0x2a, 0x08);
	write_cmos_sensor_8(ctx, 0x2b, 0x07);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0x3f, 0xd0);
	*/
	printk("Enter preview_setting!\n");
write_cmos_sensor_8(ctx, 0xfc, 0x01);
write_cmos_sensor_8(ctx, 0xf2, 0x00);
write_cmos_sensor_8(ctx, 0xf4, 0x80);
write_cmos_sensor_8(ctx, 0xf5, 0x19);
write_cmos_sensor_8(ctx, 0xf6, 0x44);
write_cmos_sensor_8(ctx, 0xf8, 0x63);
write_cmos_sensor_8(ctx, 0xfa, 0x45);
write_cmos_sensor_8(ctx, 0xf9, 0x00);
write_cmos_sensor_8(ctx, 0xf7, 0x95);
write_cmos_sensor_8(ctx, 0xfc, 0x03);
write_cmos_sensor_8(ctx, 0xfc, 0xea);
write_cmos_sensor_8(ctx, 0xfe, 0x03);
write_cmos_sensor_8(ctx, 0x03, 0x9a);
write_cmos_sensor_8(ctx, 0x18, 0x07);
write_cmos_sensor_8(ctx, 0x01, 0x07);
write_cmos_sensor_8(ctx, 0xfc, 0xee);
write_cmos_sensor_8(ctx, 0xfe, 0x00);
write_cmos_sensor_8(ctx, 0x80, 0x13);
write_cmos_sensor_8(ctx, 0xad, 0x00);
write_cmos_sensor_8(ctx, 0x90, 0x01);
write_cmos_sensor_8(ctx, 0x92, FullStartY);
write_cmos_sensor_8(ctx, 0x94, FullStartX);
write_cmos_sensor_8(ctx, 0x95, 0x09);
write_cmos_sensor_8(ctx, 0x96, 0x90);
write_cmos_sensor_8(ctx, 0x97, 0x0c);
write_cmos_sensor_8(ctx, 0x98, 0xc0);
write_cmos_sensor_8(ctx, 0xfe, 0x01);
write_cmos_sensor_8(ctx, 0x62, 0x60);
write_cmos_sensor_8(ctx, 0x63, 0x48);
write_cmos_sensor_8(ctx, 0xfe, 0x03);
write_cmos_sensor_8(ctx, 0x02, 0x03);
write_cmos_sensor_8(ctx, 0x04, 0x80);
write_cmos_sensor_8(ctx, 0x11, 0x2b);
write_cmos_sensor_8(ctx, 0x12, 0xf0);
write_cmos_sensor_8(ctx, 0x13, 0x0f);
write_cmos_sensor_8(ctx, 0x15, 0x10);
write_cmos_sensor_8(ctx, 0x16, 0x29);
write_cmos_sensor_8(ctx, 0x17, 0xff);
write_cmos_sensor_8(ctx, 0x19, 0xaa);
write_cmos_sensor_8(ctx, 0x1a, 0x02);
write_cmos_sensor_8(ctx, 0x21, 0x05);
write_cmos_sensor_8(ctx, 0x22, 0x06);
write_cmos_sensor_8(ctx, 0x23, 0x16);
write_cmos_sensor_8(ctx, 0x24, 0x00);
write_cmos_sensor_8(ctx, 0x25, 0x12);
write_cmos_sensor_8(ctx, 0x26, 0x07);
write_cmos_sensor_8(ctx, 0x29, 0x07);
write_cmos_sensor_8(ctx, 0x2a, 0x08);
write_cmos_sensor_8(ctx, 0x2b, 0x07);
write_cmos_sensor_8(ctx, 0xfe, 0x00);
write_cmos_sensor_8(ctx, 0x3f, 0xd0);

};

static void addr_data_pair_capture_gc8034(struct subdrv_ctx *ctx){
	
/*	printk("Enter preview_setting!\n");
	
	write_cmos_sensor_8(ctx, 0xfc, 0x01);
	write_cmos_sensor_8(ctx, 0xf2, 0x00);
	write_cmos_sensor_8(ctx, 0xf4, 0x80);
	write_cmos_sensor_8(ctx, 0xf5, 0x19);
	write_cmos_sensor_8(ctx, 0xf6, 0x44);
	if (ctx->current_fps == 240) {
		write_cmos_sensor_8(ctx, 0xf8, 0x4f);
		write_cmos_sensor_8(ctx, 0xfa, 0x36);
		gamma_index=1;
	} else {
		write_cmos_sensor_8(ctx, 0xf8, 0x63);
		write_cmos_sensor_8(ctx, 0xfa, 0x45);
		gamma_index=0;
	}
	write_cmos_sensor_8(ctx, 0xf9, 0x00);
	write_cmos_sensor_8(ctx, 0xf7, 0x95);
	write_cmos_sensor_8(ctx, 0xfc, 0x03);
	write_cmos_sensor_8(ctx, 0xfc, 0xea);
	write_cmos_sensor_8(ctx, 0xfe, 0x03);
	write_cmos_sensor_8(ctx, 0x03, 0x9a);
	write_cmos_sensor_8(ctx, 0x18, 0x07);
	write_cmos_sensor_8(ctx, 0x01, 0x07);
	write_cmos_sensor_8(ctx, 0xfc, 0xee);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0x80, 0x13);
	write_cmos_sensor_8(ctx, 0xad, 0x00);
	write_cmos_sensor_8(ctx, 0x90, 0x01);
	write_cmos_sensor_8(ctx, 0x92, FullStartY);
	write_cmos_sensor_8(ctx, 0x94, FullStartX);
	write_cmos_sensor_8(ctx, 0x95, 0x09);
	write_cmos_sensor_8(ctx, 0x96, 0x90);
	write_cmos_sensor_8(ctx, 0x97, 0x0c);
	write_cmos_sensor_8(ctx, 0x98, 0xc0);
	write_cmos_sensor_8(ctx, 0xfe, 0x01);
	write_cmos_sensor_8(ctx, 0x62, 0x60);
	write_cmos_sensor_8(ctx, 0x63, 0x48);
	write_cmos_sensor_8(ctx, 0xfe, 0x03);
	write_cmos_sensor_8(ctx, 0x02, 0x03);
	write_cmos_sensor_8(ctx, 0x04, 0x80);
	write_cmos_sensor_8(ctx, 0x11, 0x2b);
	write_cmos_sensor_8(ctx, 0x12, 0xf0);
	write_cmos_sensor_8(ctx, 0x13, 0x0f);
	write_cmos_sensor_8(ctx, 0x15, 0x10);
	write_cmos_sensor_8(ctx, 0x16, 0x29);
	write_cmos_sensor_8(ctx, 0x17, 0xff);
	write_cmos_sensor_8(ctx, 0x19, 0xaa);
	write_cmos_sensor_8(ctx, 0x1a, 0x02);
	write_cmos_sensor_8(ctx, 0x21, 0x05);
	write_cmos_sensor_8(ctx, 0x22, 0x06);
	write_cmos_sensor_8(ctx, 0x23, 0x16);
	write_cmos_sensor_8(ctx, 0x24, 0x00);
	write_cmos_sensor_8(ctx, 0x25, 0x12);
	write_cmos_sensor_8(ctx, 0x26, 0x07);
	write_cmos_sensor_8(ctx, 0x29, 0x07);
	write_cmos_sensor_8(ctx, 0x2a, 0x08);
	write_cmos_sensor_8(ctx, 0x2b, 0x07);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0x3f, 0xd0);
	*/
	printk("Enter preview_setting!\n");
write_cmos_sensor_8(ctx, 0xfc, 0x01);
write_cmos_sensor_8(ctx, 0xf2, 0x00);
write_cmos_sensor_8(ctx, 0xf4, 0x80);
write_cmos_sensor_8(ctx, 0xf5, 0x19);
write_cmos_sensor_8(ctx, 0xf6, 0x44);
write_cmos_sensor_8(ctx, 0xf8, 0x63);
write_cmos_sensor_8(ctx, 0xfa, 0x45);
write_cmos_sensor_8(ctx, 0xf9, 0x00);
write_cmos_sensor_8(ctx, 0xf7, 0x95);
write_cmos_sensor_8(ctx, 0xfc, 0x03);
write_cmos_sensor_8(ctx, 0xfc, 0xea);
write_cmos_sensor_8(ctx, 0xfe, 0x03);
write_cmos_sensor_8(ctx, 0x03, 0x9a);
write_cmos_sensor_8(ctx, 0x18, 0x07);
write_cmos_sensor_8(ctx, 0x01, 0x07);
write_cmos_sensor_8(ctx, 0xfc, 0xee);
write_cmos_sensor_8(ctx, 0xfe, 0x00);
write_cmos_sensor_8(ctx, 0x80, 0x13);
write_cmos_sensor_8(ctx, 0xad, 0x00);
write_cmos_sensor_8(ctx, 0x90, 0x01);
write_cmos_sensor_8(ctx, 0x92, FullStartY);
write_cmos_sensor_8(ctx, 0x94, FullStartX);
write_cmos_sensor_8(ctx, 0x95, 0x09);
write_cmos_sensor_8(ctx, 0x96, 0x90);
write_cmos_sensor_8(ctx, 0x97, 0x0c);
write_cmos_sensor_8(ctx, 0x98, 0xc0);
write_cmos_sensor_8(ctx, 0xfe, 0x01);
write_cmos_sensor_8(ctx, 0x62, 0x60);
write_cmos_sensor_8(ctx, 0x63, 0x48);
write_cmos_sensor_8(ctx, 0xfe, 0x03);
write_cmos_sensor_8(ctx, 0x02, 0x03);
write_cmos_sensor_8(ctx, 0x04, 0x80);
write_cmos_sensor_8(ctx, 0x11, 0x2b);
write_cmos_sensor_8(ctx, 0x12, 0xf0);
write_cmos_sensor_8(ctx, 0x13, 0x0f);
write_cmos_sensor_8(ctx, 0x15, 0x10);
write_cmos_sensor_8(ctx, 0x16, 0x29);
write_cmos_sensor_8(ctx, 0x17, 0xff);
write_cmos_sensor_8(ctx, 0x19, 0xaa);
write_cmos_sensor_8(ctx, 0x1a, 0x02);
write_cmos_sensor_8(ctx, 0x21, 0x05);
write_cmos_sensor_8(ctx, 0x22, 0x06);
write_cmos_sensor_8(ctx, 0x23, 0x16);
write_cmos_sensor_8(ctx, 0x24, 0x00);
write_cmos_sensor_8(ctx, 0x25, 0x12);
write_cmos_sensor_8(ctx, 0x26, 0x07);
write_cmos_sensor_8(ctx, 0x29, 0x07);
write_cmos_sensor_8(ctx, 0x2a, 0x08);
write_cmos_sensor_8(ctx, 0x2b, 0x07);
write_cmos_sensor_8(ctx, 0xfe, 0x00);
write_cmos_sensor_8(ctx, 0x3f, 0xd0);
};

static void addr_data_pair_video_gc8034(struct subdrv_ctx *ctx){
/*	printk("Enter preview_setting!\n");
	
	write_cmos_sensor_8(ctx, 0xfc, 0x01);
	write_cmos_sensor_8(ctx, 0xf2, 0x00);
	write_cmos_sensor_8(ctx, 0xf4, 0x80);
	write_cmos_sensor_8(ctx, 0xf5, 0x19);
	write_cmos_sensor_8(ctx, 0xf6, 0x44);
	if (ctx->current_fps == 240) {
		write_cmos_sensor_8(ctx, 0xf8, 0x4f);
		write_cmos_sensor_8(ctx, 0xfa, 0x36);
		gamma_index=1;
	} else {
		write_cmos_sensor_8(ctx, 0xf8, 0x63);
		write_cmos_sensor_8(ctx, 0xfa, 0x45);
		gamma_index=0;
	}
	write_cmos_sensor_8(ctx, 0xf9, 0x00);
	write_cmos_sensor_8(ctx, 0xf7, 0x95);
	write_cmos_sensor_8(ctx, 0xfc, 0x03);
	write_cmos_sensor_8(ctx, 0xfc, 0xea);
	write_cmos_sensor_8(ctx, 0xfe, 0x03);
	write_cmos_sensor_8(ctx, 0x03, 0x9a);
	write_cmos_sensor_8(ctx, 0x18, 0x07);
	write_cmos_sensor_8(ctx, 0x01, 0x07);
	write_cmos_sensor_8(ctx, 0xfc, 0xee);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0x80, 0x13);
	write_cmos_sensor_8(ctx, 0xad, 0x00);
	write_cmos_sensor_8(ctx, 0x90, 0x01);
	write_cmos_sensor_8(ctx, 0x92, FullStartY);
	write_cmos_sensor_8(ctx, 0x94, FullStartX);
	write_cmos_sensor_8(ctx, 0x95, 0x09);
	write_cmos_sensor_8(ctx, 0x96, 0x90);
	write_cmos_sensor_8(ctx, 0x97, 0x0c);
	write_cmos_sensor_8(ctx, 0x98, 0xc0);
	write_cmos_sensor_8(ctx, 0xfe, 0x01);
	write_cmos_sensor_8(ctx, 0x62, 0x60);
	write_cmos_sensor_8(ctx, 0x63, 0x48);
	write_cmos_sensor_8(ctx, 0xfe, 0x03);
	write_cmos_sensor_8(ctx, 0x02, 0x03);
	write_cmos_sensor_8(ctx, 0x04, 0x80);
	write_cmos_sensor_8(ctx, 0x11, 0x2b);
	write_cmos_sensor_8(ctx, 0x12, 0xf0);
	write_cmos_sensor_8(ctx, 0x13, 0x0f);
	write_cmos_sensor_8(ctx, 0x15, 0x10);
	write_cmos_sensor_8(ctx, 0x16, 0x29);
	write_cmos_sensor_8(ctx, 0x17, 0xff);
	write_cmos_sensor_8(ctx, 0x19, 0xaa);
	write_cmos_sensor_8(ctx, 0x1a, 0x02);
	write_cmos_sensor_8(ctx, 0x21, 0x05);
	write_cmos_sensor_8(ctx, 0x22, 0x06);
	write_cmos_sensor_8(ctx, 0x23, 0x16);
	write_cmos_sensor_8(ctx, 0x24, 0x00);
	write_cmos_sensor_8(ctx, 0x25, 0x12);
	write_cmos_sensor_8(ctx, 0x26, 0x07);
	write_cmos_sensor_8(ctx, 0x29, 0x07);
	write_cmos_sensor_8(ctx, 0x2a, 0x08);
	write_cmos_sensor_8(ctx, 0x2b, 0x07);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0x3f, 0xd0);
	*/
	printk("Enter preview_setting!\n");
write_cmos_sensor_8(ctx, 0xfc, 0x01);
write_cmos_sensor_8(ctx, 0xf2, 0x00);
write_cmos_sensor_8(ctx, 0xf4, 0x80);
write_cmos_sensor_8(ctx, 0xf5, 0x19);
write_cmos_sensor_8(ctx, 0xf6, 0x44);
write_cmos_sensor_8(ctx, 0xf8, 0x63);
write_cmos_sensor_8(ctx, 0xfa, 0x45);
write_cmos_sensor_8(ctx, 0xf9, 0x00);
write_cmos_sensor_8(ctx, 0xf7, 0x95);
write_cmos_sensor_8(ctx, 0xfc, 0x03);
write_cmos_sensor_8(ctx, 0xfc, 0xea);
write_cmos_sensor_8(ctx, 0xfe, 0x03);
write_cmos_sensor_8(ctx, 0x03, 0x9a);
write_cmos_sensor_8(ctx, 0x18, 0x07);
write_cmos_sensor_8(ctx, 0x01, 0x07);
write_cmos_sensor_8(ctx, 0xfc, 0xee);
write_cmos_sensor_8(ctx, 0xfe, 0x00);
write_cmos_sensor_8(ctx, 0x80, 0x13);
write_cmos_sensor_8(ctx, 0xad, 0x00);
write_cmos_sensor_8(ctx, 0x90, 0x01);
write_cmos_sensor_8(ctx, 0x92, FullStartY);
write_cmos_sensor_8(ctx, 0x94, FullStartX);
write_cmos_sensor_8(ctx, 0x95, 0x09);
write_cmos_sensor_8(ctx, 0x96, 0x90);
write_cmos_sensor_8(ctx, 0x97, 0x0c);
write_cmos_sensor_8(ctx, 0x98, 0xc0);
write_cmos_sensor_8(ctx, 0xfe, 0x01);
write_cmos_sensor_8(ctx, 0x62, 0x60);
write_cmos_sensor_8(ctx, 0x63, 0x48);
write_cmos_sensor_8(ctx, 0xfe, 0x03);
write_cmos_sensor_8(ctx, 0x02, 0x03);
write_cmos_sensor_8(ctx, 0x04, 0x80);
write_cmos_sensor_8(ctx, 0x11, 0x2b);
write_cmos_sensor_8(ctx, 0x12, 0xf0);
write_cmos_sensor_8(ctx, 0x13, 0x0f);
write_cmos_sensor_8(ctx, 0x15, 0x10);
write_cmos_sensor_8(ctx, 0x16, 0x29);
write_cmos_sensor_8(ctx, 0x17, 0xff);
write_cmos_sensor_8(ctx, 0x19, 0xaa);
write_cmos_sensor_8(ctx, 0x1a, 0x02);
write_cmos_sensor_8(ctx, 0x21, 0x05);
write_cmos_sensor_8(ctx, 0x22, 0x06);
write_cmos_sensor_8(ctx, 0x23, 0x16);
write_cmos_sensor_8(ctx, 0x24, 0x00);
write_cmos_sensor_8(ctx, 0x25, 0x12);
write_cmos_sensor_8(ctx, 0x26, 0x07);
write_cmos_sensor_8(ctx, 0x29, 0x07);
write_cmos_sensor_8(ctx, 0x2a, 0x08);
write_cmos_sensor_8(ctx, 0x2b, 0x07);
write_cmos_sensor_8(ctx, 0xfe, 0x00);
write_cmos_sensor_8(ctx, 0x3f, 0xd0);
};

static void addr_data_pair_hs_video_gc8034(struct subdrv_ctx *ctx){
/*	printk("Enter preview_setting!\n");
	
	write_cmos_sensor_8(ctx, 0xfc, 0x01);
	write_cmos_sensor_8(ctx, 0xf2, 0x00);
	write_cmos_sensor_8(ctx, 0xf4, 0x80);
	write_cmos_sensor_8(ctx, 0xf5, 0x19);
	write_cmos_sensor_8(ctx, 0xf6, 0x44);
	if (ctx->current_fps == 240) {
		write_cmos_sensor_8(ctx, 0xf8, 0x4f);
		write_cmos_sensor_8(ctx, 0xfa, 0x36);
		gamma_index=1;
	} else {
		write_cmos_sensor_8(ctx, 0xf8, 0x63);
		write_cmos_sensor_8(ctx, 0xfa, 0x45);
		gamma_index=0;
	}
	write_cmos_sensor_8(ctx, 0xf9, 0x00);
	write_cmos_sensor_8(ctx, 0xf7, 0x95);
	write_cmos_sensor_8(ctx, 0xfc, 0x03);
	write_cmos_sensor_8(ctx, 0xfc, 0xea);
	write_cmos_sensor_8(ctx, 0xfe, 0x03);
	write_cmos_sensor_8(ctx, 0x03, 0x9a);
	write_cmos_sensor_8(ctx, 0x18, 0x07);
	write_cmos_sensor_8(ctx, 0x01, 0x07);
	write_cmos_sensor_8(ctx, 0xfc, 0xee);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0x80, 0x13);
	write_cmos_sensor_8(ctx, 0xad, 0x00);
	write_cmos_sensor_8(ctx, 0x90, 0x01);
	write_cmos_sensor_8(ctx, 0x92, FullStartY);
	write_cmos_sensor_8(ctx, 0x94, FullStartX);
	write_cmos_sensor_8(ctx, 0x95, 0x09);
	write_cmos_sensor_8(ctx, 0x96, 0x90);
	write_cmos_sensor_8(ctx, 0x97, 0x0c);
	write_cmos_sensor_8(ctx, 0x98, 0xc0);
	write_cmos_sensor_8(ctx, 0xfe, 0x01);
	write_cmos_sensor_8(ctx, 0x62, 0x60);
	write_cmos_sensor_8(ctx, 0x63, 0x48);
	write_cmos_sensor_8(ctx, 0xfe, 0x03);
	write_cmos_sensor_8(ctx, 0x02, 0x03);
	write_cmos_sensor_8(ctx, 0x04, 0x80);
	write_cmos_sensor_8(ctx, 0x11, 0x2b);
	write_cmos_sensor_8(ctx, 0x12, 0xf0);
	write_cmos_sensor_8(ctx, 0x13, 0x0f);
	write_cmos_sensor_8(ctx, 0x15, 0x10);
	write_cmos_sensor_8(ctx, 0x16, 0x29);
	write_cmos_sensor_8(ctx, 0x17, 0xff);
	write_cmos_sensor_8(ctx, 0x19, 0xaa);
	write_cmos_sensor_8(ctx, 0x1a, 0x02);
	write_cmos_sensor_8(ctx, 0x21, 0x05);
	write_cmos_sensor_8(ctx, 0x22, 0x06);
	write_cmos_sensor_8(ctx, 0x23, 0x16);
	write_cmos_sensor_8(ctx, 0x24, 0x00);
	write_cmos_sensor_8(ctx, 0x25, 0x12);
	write_cmos_sensor_8(ctx, 0x26, 0x07);
	write_cmos_sensor_8(ctx, 0x29, 0x07);
	write_cmos_sensor_8(ctx, 0x2a, 0x08);
	write_cmos_sensor_8(ctx, 0x2b, 0x07);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0x3f, 0xd0);
	*/
	printk("Enter preview_setting!\n");
write_cmos_sensor_8(ctx, 0xfc, 0x01);
write_cmos_sensor_8(ctx, 0xf2, 0x00);
write_cmos_sensor_8(ctx, 0xf4, 0x80);
write_cmos_sensor_8(ctx, 0xf5, 0x19);
write_cmos_sensor_8(ctx, 0xf6, 0x44);
write_cmos_sensor_8(ctx, 0xf8, 0x63);
write_cmos_sensor_8(ctx, 0xfa, 0x45);
write_cmos_sensor_8(ctx, 0xf9, 0x00);
write_cmos_sensor_8(ctx, 0xf7, 0x95);
write_cmos_sensor_8(ctx, 0xfc, 0x03);
write_cmos_sensor_8(ctx, 0xfc, 0xea);
write_cmos_sensor_8(ctx, 0xfe, 0x03);
write_cmos_sensor_8(ctx, 0x03, 0x9a);
write_cmos_sensor_8(ctx, 0x18, 0x07);
write_cmos_sensor_8(ctx, 0x01, 0x07);
write_cmos_sensor_8(ctx, 0xfc, 0xee);
write_cmos_sensor_8(ctx, 0xfe, 0x00);
write_cmos_sensor_8(ctx, 0x80, 0x13);
write_cmos_sensor_8(ctx, 0xad, 0x00);
write_cmos_sensor_8(ctx, 0x90, 0x01);
write_cmos_sensor_8(ctx, 0x92, FullStartY);
write_cmos_sensor_8(ctx, 0x94, FullStartX);
write_cmos_sensor_8(ctx, 0x95, 0x09);
write_cmos_sensor_8(ctx, 0x96, 0x90);
write_cmos_sensor_8(ctx, 0x97, 0x0c);
write_cmos_sensor_8(ctx, 0x98, 0xc0);
write_cmos_sensor_8(ctx, 0xfe, 0x01);
write_cmos_sensor_8(ctx, 0x62, 0x60);
write_cmos_sensor_8(ctx, 0x63, 0x48);
write_cmos_sensor_8(ctx, 0xfe, 0x03);
write_cmos_sensor_8(ctx, 0x02, 0x03);
write_cmos_sensor_8(ctx, 0x04, 0x80);
write_cmos_sensor_8(ctx, 0x11, 0x2b);
write_cmos_sensor_8(ctx, 0x12, 0xf0);
write_cmos_sensor_8(ctx, 0x13, 0x0f);
write_cmos_sensor_8(ctx, 0x15, 0x10);
write_cmos_sensor_8(ctx, 0x16, 0x29);
write_cmos_sensor_8(ctx, 0x17, 0xff);
write_cmos_sensor_8(ctx, 0x19, 0xaa);
write_cmos_sensor_8(ctx, 0x1a, 0x02);
write_cmos_sensor_8(ctx, 0x21, 0x05);
write_cmos_sensor_8(ctx, 0x22, 0x06);
write_cmos_sensor_8(ctx, 0x23, 0x16);
write_cmos_sensor_8(ctx, 0x24, 0x00);
write_cmos_sensor_8(ctx, 0x25, 0x12);
write_cmos_sensor_8(ctx, 0x26, 0x07);
write_cmos_sensor_8(ctx, 0x29, 0x07);
write_cmos_sensor_8(ctx, 0x2a, 0x08);
write_cmos_sensor_8(ctx, 0x2b, 0x07);
write_cmos_sensor_8(ctx, 0xfe, 0x00);
write_cmos_sensor_8(ctx, 0x3f, 0xd0);
};

static void addr_data_pair_slim_video_gc8034(struct subdrv_ctx *ctx){
	printk("Enter capture_setting!\n");
	write_cmos_sensor_8(ctx, 0xfc, 0x01);
	write_cmos_sensor_8(ctx, 0xf2, 0x00);
	write_cmos_sensor_8(ctx, 0xf4, 0x80);
	write_cmos_sensor_8(ctx, 0xf5, 0x19);
	write_cmos_sensor_8(ctx, 0xf6, 0x44);
	if (ctx->current_fps == 240) {	/*240, 300 for test*/
		write_cmos_sensor_8(ctx, 0xf8, 0x4f);
		write_cmos_sensor_8(ctx, 0xfa, 0x36);
		gamma_index=1;
	} else {
		write_cmos_sensor_8(ctx, 0xf8, 0x63);
		write_cmos_sensor_8(ctx, 0xfa, 0x45);
		gamma_index=0;
	}
	write_cmos_sensor_8(ctx, 0xf9, 0x00);
	write_cmos_sensor_8(ctx, 0xf7, 0x95);
	write_cmos_sensor_8(ctx, 0xfc, 0x03);
	write_cmos_sensor_8(ctx, 0xfc, 0xea);
	write_cmos_sensor_8(ctx, 0xfe, 0x03);
	write_cmos_sensor_8(ctx, 0x03, 0x9a);
	write_cmos_sensor_8(ctx, 0x18, 0x07);
	write_cmos_sensor_8(ctx, 0x01, 0x07);
	write_cmos_sensor_8(ctx, 0xfc, 0xee);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0x80, 0x13);
	write_cmos_sensor_8(ctx, 0xad, 0x00);
	write_cmos_sensor_8(ctx, 0x90, 0x01);
	write_cmos_sensor_8(ctx, 0x92, FullStartY);
	write_cmos_sensor_8(ctx, 0x94, FullStartX);
	write_cmos_sensor_8(ctx, 0x95, 0x09);
	write_cmos_sensor_8(ctx, 0x96, 0x90);
	write_cmos_sensor_8(ctx, 0x97, 0x0c);
	write_cmos_sensor_8(ctx, 0x98, 0xc0);
	write_cmos_sensor_8(ctx, 0xfe, 0x01);
	write_cmos_sensor_8(ctx, 0x62, 0x60);
	write_cmos_sensor_8(ctx, 0x63, 0x48);
	write_cmos_sensor_8(ctx, 0xfe, 0x03);
	write_cmos_sensor_8(ctx, 0x02, 0x03);
	write_cmos_sensor_8(ctx, 0x04, 0x80);
	write_cmos_sensor_8(ctx, 0x11, 0x2b);
	write_cmos_sensor_8(ctx, 0x12, 0xf0);
	write_cmos_sensor_8(ctx, 0x13, 0x0f);
	write_cmos_sensor_8(ctx, 0x15, 0x10);
	write_cmos_sensor_8(ctx, 0x16, 0x29);
	write_cmos_sensor_8(ctx, 0x17, 0xff);
	write_cmos_sensor_8(ctx, 0x19, 0xaa);
	write_cmos_sensor_8(ctx, 0x1a, 0x02);
	write_cmos_sensor_8(ctx, 0x21, 0x05);
	write_cmos_sensor_8(ctx, 0x22, 0x06);
	write_cmos_sensor_8(ctx, 0x23, 0x16);
	write_cmos_sensor_8(ctx, 0x24, 0x00);
	write_cmos_sensor_8(ctx, 0x25, 0x12);
	write_cmos_sensor_8(ctx, 0x26, 0x07);
	write_cmos_sensor_8(ctx, 0x29, 0x07);
	write_cmos_sensor_8(ctx, 0x2a, 0x08);
	write_cmos_sensor_8(ctx, 0x2b, 0x07);
	write_cmos_sensor_8(ctx, 0xfe, 0x00);
	write_cmos_sensor_8(ctx, 0x3f, 0xd0);
};

#endif