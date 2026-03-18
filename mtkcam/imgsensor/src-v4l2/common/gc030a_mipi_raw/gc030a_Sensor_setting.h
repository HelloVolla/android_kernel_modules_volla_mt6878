/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 gc030a_Sensor_setting.h
 *
 * Project:
 * --------
 * Description:
 * ------------
 *	 CMOS sensor header file
 *
 ****************************************************************************/
#include "gc030amipiraw_Sensor.h"
#ifndef _GC030A_SENSOR_SETTING_H
#define _GC030A_SENSOR_SETTING_H

#define SEQUENTIAL_WRITE_EN 1
//#define XTALK_OTP_ADDR 0x53C0

#define read_cmos_sensor_8(...) subdrv_i2c_rd_addr8_u8(__VA_ARGS__)
#define write_cmos_sensor_8(...) subdrv_i2c_wr_addr8_u8(__VA_ARGS__)

/*static void addr_data_pair_init_gc030a(struct subdrv_ctx *ctx){
	
write_cmos_sensor_8(ctx, 0xfe,0xf0);
write_cmos_sensor_8(ctx, 0xfe,0xf0);
write_cmos_sensor_8(ctx, 0xfe,0x00);
write_cmos_sensor_8(ctx, 0xfc,0x0e);
write_cmos_sensor_8(ctx, 0xfc,0x0e);
write_cmos_sensor_8(ctx, 0xf2,0x80);
write_cmos_sensor_8(ctx, 0xf3,0x00);
write_cmos_sensor_8(ctx, 0xf7,0x1b);
write_cmos_sensor_8(ctx, 0xf8,0x04); 
write_cmos_sensor_8(ctx, 0xf9,0x8e);
write_cmos_sensor_8(ctx, 0xfa,0x11);    
write_cmos_sensor_8(ctx, 0xfe,0x03);
write_cmos_sensor_8(ctx, 0x40,0x08);
write_cmos_sensor_8(ctx, 0x42,0x00);
write_cmos_sensor_8(ctx, 0x43,0x00);
write_cmos_sensor_8(ctx, 0x01,0x03);
write_cmos_sensor_8(ctx, 0x10,0x84);                                       
write_cmos_sensor_8(ctx, 0x01,0x03);             
write_cmos_sensor_8(ctx, 0x02,0x11);  // 00 20150522        
write_cmos_sensor_8(ctx, 0x03,0x94);             
write_cmos_sensor_8(ctx, 0x04,0x01);            
write_cmos_sensor_8(ctx, 0x05,0x00);             
write_cmos_sensor_8(ctx, 0x06,0x80);             
write_cmos_sensor_8(ctx, 0x11,0x1e);             
write_cmos_sensor_8(ctx, 0x12,0x00);      
write_cmos_sensor_8(ctx, 0x13,0x05);             
write_cmos_sensor_8(ctx, 0x15,0x10);                                                                    
write_cmos_sensor_8(ctx, 0x21,0x10);             
write_cmos_sensor_8(ctx, 0x22,0x01);             
write_cmos_sensor_8(ctx, 0x23,0x10);                                             
write_cmos_sensor_8(ctx, 0x24,0x02);                                             
write_cmos_sensor_8(ctx, 0x25,0x10);                                             
write_cmos_sensor_8(ctx, 0x26,0x03);                                             
write_cmos_sensor_8(ctx, 0x29,0x02);                                             
write_cmos_sensor_8(ctx, 0x2a,0x0a);                                             
write_cmos_sensor_8(ctx, 0x2b,0x04);                                             
write_cmos_sensor_8(ctx, 0xfe,0x00);
write_cmos_sensor_8(ctx, 0x00,0x2f);
write_cmos_sensor_8(ctx, 0x01,0x0f);
write_cmos_sensor_8(ctx, 0x02,0x04);
write_cmos_sensor_8(ctx, 0x03,0x03);
write_cmos_sensor_8(ctx, 0x04,0x50);
write_cmos_sensor_8(ctx, 0x09,0x00);
write_cmos_sensor_8(ctx, 0x0a,0x00);
write_cmos_sensor_8(ctx, 0x0b,0x00);
write_cmos_sensor_8(ctx, 0x0c,0x04);
write_cmos_sensor_8(ctx, 0x0d,0x01);
write_cmos_sensor_8(ctx, 0x0e,0xe8);
write_cmos_sensor_8(ctx, 0x0f,0x02);
write_cmos_sensor_8(ctx, 0x10,0x88);
write_cmos_sensor_8(ctx, 0x16,0x00);	
write_cmos_sensor_8(ctx, 0x17,0x14);
write_cmos_sensor_8(ctx, 0x18,0x1a);
write_cmos_sensor_8(ctx, 0x19,0x14);
write_cmos_sensor_8(ctx, 0x1b,0x48);
write_cmos_sensor_8(ctx, 0x1c,0x1c);
write_cmos_sensor_8(ctx, 0x1e,0x6b);
write_cmos_sensor_8(ctx, 0x1f,0x28);
write_cmos_sensor_8(ctx, 0x20,0x8b);//0x89 travis20140801
write_cmos_sensor_8(ctx, 0x21,0x49);
write_cmos_sensor_8(ctx, 0x22,0xb0);
write_cmos_sensor_8(ctx, 0x23,0x04);
write_cmos_sensor_8(ctx, 0x24,0x16);
write_cmos_sensor_8(ctx, 0x34,0x20);
write_cmos_sensor_8(ctx, 0x26,0x23);
write_cmos_sensor_8(ctx, 0x28,0xff);
write_cmos_sensor_8(ctx, 0x29,0x00);
write_cmos_sensor_8(ctx, 0x32,0x00);
write_cmos_sensor_8(ctx, 0x33,0x10); 
write_cmos_sensor_8(ctx, 0x37,0x20);
write_cmos_sensor_8(ctx, 0x38,0x10);
write_cmos_sensor_8(ctx, 0x47,0x80);
write_cmos_sensor_8(ctx, 0x4e,0x66);
write_cmos_sensor_8(ctx, 0xa8,0x02);
write_cmos_sensor_8(ctx, 0xa9,0x80);
write_cmos_sensor_8(ctx, 0x40,0xff);
write_cmos_sensor_8(ctx, 0x41,0x21);
write_cmos_sensor_8(ctx, 0x42,0xcf);
write_cmos_sensor_8(ctx, 0x44,0x02);
write_cmos_sensor_8(ctx, 0x45,0xa8); 
write_cmos_sensor_8(ctx, 0x46,0x02); 
write_cmos_sensor_8(ctx, 0x4a,0x11);
write_cmos_sensor_8(ctx, 0x4b,0x01);
write_cmos_sensor_8(ctx, 0x4c,0x20);
write_cmos_sensor_8(ctx, 0x4d,0x05);
write_cmos_sensor_8(ctx, 0x4f,0x01);
write_cmos_sensor_8(ctx, 0x50,0x01);
write_cmos_sensor_8(ctx, 0x55,0x01);
write_cmos_sensor_8(ctx, 0x56,0xe0);
write_cmos_sensor_8(ctx, 0x57,0x02);
write_cmos_sensor_8(ctx, 0x58,0x80);
write_cmos_sensor_8(ctx, 0x70,0x70);
write_cmos_sensor_8(ctx, 0x5a,0x84);
write_cmos_sensor_8(ctx, 0x5b,0xc9);
write_cmos_sensor_8(ctx, 0x5c,0xed);
write_cmos_sensor_8(ctx, 0x77,0x74);
write_cmos_sensor_8(ctx, 0x78,0x40);
write_cmos_sensor_8(ctx, 0x79,0x5f);
write_cmos_sensor_8(ctx, 0x82,0x14); 
write_cmos_sensor_8(ctx, 0x83,0x0b);
write_cmos_sensor_8(ctx, 0x89,0xf0);
write_cmos_sensor_8(ctx, 0x8f,0xaa); 
write_cmos_sensor_8(ctx, 0x90,0x8c); 
write_cmos_sensor_8(ctx, 0x91,0x90);
write_cmos_sensor_8(ctx, 0x92,0x03); 
write_cmos_sensor_8(ctx, 0x93,0x03); 
write_cmos_sensor_8(ctx, 0x94,0x05); 
write_cmos_sensor_8(ctx, 0x95,0x65); 
write_cmos_sensor_8(ctx, 0x96,0xf0); 
write_cmos_sensor_8(ctx, 0xfe,0x00);
write_cmos_sensor_8(ctx, 0x9a,0x20);
write_cmos_sensor_8(ctx, 0x9b,0x80);
write_cmos_sensor_8(ctx, 0x9c,0x40);
write_cmos_sensor_8(ctx, 0x9d,0x80); 
write_cmos_sensor_8(ctx, 0xa1,0x30);
write_cmos_sensor_8(ctx, 0xa2,0x32);
write_cmos_sensor_8(ctx, 0xa4,0x30);
write_cmos_sensor_8(ctx, 0xa5,0x30);
write_cmos_sensor_8(ctx, 0xaa,0x10); 
write_cmos_sensor_8(ctx, 0xac,0x22);
write_cmos_sensor_8(ctx, 0xfe,0x00);//default
write_cmos_sensor_8(ctx, 0xbf,0x08);
write_cmos_sensor_8(ctx, 0xc0,0x16);
write_cmos_sensor_8(ctx, 0xc1,0x28);
write_cmos_sensor_8(ctx, 0xc2,0x41);
write_cmos_sensor_8(ctx, 0xc3,0x5a);
write_cmos_sensor_8(ctx, 0xc4,0x6c);
write_cmos_sensor_8(ctx, 0xc5,0x7a);
write_cmos_sensor_8(ctx, 0xc6,0x96);
write_cmos_sensor_8(ctx, 0xc7,0xac);
write_cmos_sensor_8(ctx, 0xc8,0xbc);
write_cmos_sensor_8(ctx, 0xc9,0xc9);
write_cmos_sensor_8(ctx, 0xca,0xd3);
write_cmos_sensor_8(ctx, 0xcb,0xdd);
write_cmos_sensor_8(ctx, 0xcc,0xe5);
write_cmos_sensor_8(ctx, 0xcd,0xf1);
write_cmos_sensor_8(ctx, 0xce,0xfa);
write_cmos_sensor_8(ctx, 0xcf,0xff);
write_cmos_sensor_8(ctx, 0xd0,0x40);
write_cmos_sensor_8(ctx, 0xd1,0x34); 
write_cmos_sensor_8(ctx, 0xd2,0x34); 
write_cmos_sensor_8(ctx, 0xd3,0x40); 
write_cmos_sensor_8(ctx, 0xd6,0xf2);
write_cmos_sensor_8(ctx, 0xd7,0x1b);
write_cmos_sensor_8(ctx, 0xd8,0x18);
write_cmos_sensor_8(ctx, 0xdd,0x03); 
write_cmos_sensor_8(ctx, 0xfe,0x01);
write_cmos_sensor_8(ctx, 0x05,0x30); 
write_cmos_sensor_8(ctx, 0x06,0x75); 
write_cmos_sensor_8(ctx, 0x07,0x40); 
write_cmos_sensor_8(ctx, 0x08,0xb0); 
write_cmos_sensor_8(ctx, 0x0a,0xc5); 
write_cmos_sensor_8(ctx, 0x0b,0x11); 
write_cmos_sensor_8(ctx, 0x0c,0x00);
write_cmos_sensor_8(ctx, 0x12,0x52); 
write_cmos_sensor_8(ctx, 0x13,0x38); 
write_cmos_sensor_8(ctx, 0x18,0x95); 
write_cmos_sensor_8(ctx, 0x19,0x96); 
write_cmos_sensor_8(ctx, 0x1f,0x20); 
write_cmos_sensor_8(ctx, 0x20,0xc0); 
write_cmos_sensor_8(ctx, 0x3e,0x40); 
write_cmos_sensor_8(ctx, 0x3f,0x57); 
write_cmos_sensor_8(ctx, 0x40,0x7d); 
write_cmos_sensor_8(ctx, 0x03,0x60);
write_cmos_sensor_8(ctx, 0x44,0x02);
write_cmos_sensor_8(ctx, 0xfe,0x01);	
write_cmos_sensor_8(ctx, 0x1c,0x91); 
write_cmos_sensor_8(ctx, 0x21,0x15); 
write_cmos_sensor_8(ctx, 0x50,0x80); 
write_cmos_sensor_8(ctx, 0x56,0x04); 
write_cmos_sensor_8(ctx, 0x59,0x08); 
write_cmos_sensor_8(ctx, 0x5b,0x02);
write_cmos_sensor_8(ctx, 0x61,0x8d); 
write_cmos_sensor_8(ctx, 0x62,0xa7); 
write_cmos_sensor_8(ctx, 0x63,0xd0); 
write_cmos_sensor_8(ctx, 0x65,0x06);
write_cmos_sensor_8(ctx, 0x66,0x06); 
write_cmos_sensor_8(ctx, 0x67,0x84); 
write_cmos_sensor_8(ctx, 0x69,0x08);
write_cmos_sensor_8(ctx, 0x6a,0x25);
write_cmos_sensor_8(ctx, 0x6b,0x01); 
write_cmos_sensor_8(ctx, 0x6c,0x00); 
write_cmos_sensor_8(ctx, 0x6d,0x02); 
write_cmos_sensor_8(ctx, 0x6e,0xf0); 
write_cmos_sensor_8(ctx, 0x6f,0x80); 
write_cmos_sensor_8(ctx, 0x76,0x80);
write_cmos_sensor_8(ctx, 0x78,0xaf); 
write_cmos_sensor_8(ctx, 0x79,0x75);
write_cmos_sensor_8(ctx, 0x7a,0x40);
write_cmos_sensor_8(ctx, 0x7b,0x50);	
write_cmos_sensor_8(ctx, 0x7c,0x0c); 
write_cmos_sensor_8(ctx, 0x90,0xc9);//stable AWB 
write_cmos_sensor_8(ctx, 0x91,0xbe);
write_cmos_sensor_8(ctx, 0x92,0xe2);
write_cmos_sensor_8(ctx, 0x93,0xc9);
write_cmos_sensor_8(ctx, 0x95,0x1b);
write_cmos_sensor_8(ctx, 0x96,0xe2);
write_cmos_sensor_8(ctx, 0x97,0x49);
write_cmos_sensor_8(ctx, 0x98,0x1b);
write_cmos_sensor_8(ctx, 0x9a,0x49);
write_cmos_sensor_8(ctx, 0x9b,0x1b);
write_cmos_sensor_8(ctx, 0x9c,0xc3);
write_cmos_sensor_8(ctx, 0x9d,0x49);
write_cmos_sensor_8(ctx, 0x9f,0xc7);
write_cmos_sensor_8(ctx, 0xa0,0xc8);
write_cmos_sensor_8(ctx, 0xa1,0x00);
write_cmos_sensor_8(ctx, 0xa2,0x00);
write_cmos_sensor_8(ctx, 0x86,0x00);
write_cmos_sensor_8(ctx, 0x87,0x00);
write_cmos_sensor_8(ctx, 0x88,0x00);
write_cmos_sensor_8(ctx, 0x89,0x00);
write_cmos_sensor_8(ctx, 0xa4,0xb9);
write_cmos_sensor_8(ctx, 0xa5,0xa0);
write_cmos_sensor_8(ctx, 0xa6,0xba);
write_cmos_sensor_8(ctx, 0xa7,0x92);
write_cmos_sensor_8(ctx, 0xa9,0xba);
write_cmos_sensor_8(ctx, 0xaa,0x80);
write_cmos_sensor_8(ctx, 0xab,0x9d);
write_cmos_sensor_8(ctx, 0xac,0x7f);
write_cmos_sensor_8(ctx, 0xae,0xbb);
write_cmos_sensor_8(ctx, 0xaf,0x9d);
write_cmos_sensor_8(ctx, 0xb0,0xc8);
write_cmos_sensor_8(ctx, 0xb1,0x97);
write_cmos_sensor_8(ctx, 0xb3,0xb7);
write_cmos_sensor_8(ctx, 0xb4,0x7f);
write_cmos_sensor_8(ctx, 0xb5,0x00);
write_cmos_sensor_8(ctx, 0xb6,0x00);
write_cmos_sensor_8(ctx, 0x8b,0x00);
write_cmos_sensor_8(ctx, 0x8c,0x00);
write_cmos_sensor_8(ctx, 0x8d,0x00);
write_cmos_sensor_8(ctx, 0x8e,0x00);
write_cmos_sensor_8(ctx, 0x94,0x55);
write_cmos_sensor_8(ctx, 0x99,0xa6);
write_cmos_sensor_8(ctx, 0x9e,0xaa);
write_cmos_sensor_8(ctx, 0xa3,0x0a);
write_cmos_sensor_8(ctx, 0x8a,0x00);
write_cmos_sensor_8(ctx, 0xa8,0x55);
write_cmos_sensor_8(ctx, 0xad,0x55);
write_cmos_sensor_8(ctx, 0xb2,0x55);
write_cmos_sensor_8(ctx, 0xb7,0x05);
write_cmos_sensor_8(ctx, 0x8f,0x00);
write_cmos_sensor_8(ctx, 0xb8,0xcb);
write_cmos_sensor_8(ctx, 0xb9,0x9b);	
write_cmos_sensor_8(ctx, 0xfe,0x01);                              
write_cmos_sensor_8(ctx, 0xd0,0x38);
write_cmos_sensor_8(ctx, 0xd1,0x00);
write_cmos_sensor_8(ctx, 0xd2,0x02);
write_cmos_sensor_8(ctx, 0xd3,0x04);
write_cmos_sensor_8(ctx, 0xd4,0x38);
write_cmos_sensor_8(ctx, 0xd5,0x12);
write_cmos_sensor_8(ctx, 0xd6,0x30);
write_cmos_sensor_8(ctx, 0xd7,0x00);
write_cmos_sensor_8(ctx, 0xd8,0x0a);
write_cmos_sensor_8(ctx, 0xd9,0x16);
write_cmos_sensor_8(ctx, 0xda,0x39);
write_cmos_sensor_8(ctx, 0xdb,0xf8);
write_cmos_sensor_8(ctx, 0xfe,0x01);
write_cmos_sensor_8(ctx, 0xc1,0x3c);
write_cmos_sensor_8(ctx, 0xc2,0x50);
write_cmos_sensor_8(ctx, 0xc3,0x00);
write_cmos_sensor_8(ctx, 0xc4,0x40);
write_cmos_sensor_8(ctx, 0xc5,0x30);
write_cmos_sensor_8(ctx, 0xc6,0x30);
write_cmos_sensor_8(ctx, 0xc7,0x10);
write_cmos_sensor_8(ctx, 0xc8,0x00);
write_cmos_sensor_8(ctx, 0xc9,0x00);
write_cmos_sensor_8(ctx, 0xdc,0x20);
write_cmos_sensor_8(ctx, 0xdd,0x10);
write_cmos_sensor_8(ctx, 0xdf,0x00);
write_cmos_sensor_8(ctx, 0xde,0x00);
write_cmos_sensor_8(ctx, 0x01,0x10);
write_cmos_sensor_8(ctx, 0x0b,0x31);
write_cmos_sensor_8(ctx, 0x0e,0x50);
write_cmos_sensor_8(ctx, 0x0f,0x0f);
write_cmos_sensor_8(ctx, 0x10,0x6e);
write_cmos_sensor_8(ctx, 0x12,0xa0);
write_cmos_sensor_8(ctx, 0x15,0x60);
write_cmos_sensor_8(ctx, 0x16,0x60);
write_cmos_sensor_8(ctx, 0x17,0xe0);
write_cmos_sensor_8(ctx, 0xcc,0x0c); 
write_cmos_sensor_8(ctx, 0xcd,0x10);
write_cmos_sensor_8(ctx, 0xce,0xa0);
write_cmos_sensor_8(ctx, 0xcf,0xe6);
write_cmos_sensor_8(ctx, 0x45,0xf7);
write_cmos_sensor_8(ctx, 0x46,0xff);
write_cmos_sensor_8(ctx, 0x47,0x15);
write_cmos_sensor_8(ctx, 0x48,0x03); 
write_cmos_sensor_8(ctx, 0x4f,0x60);
write_cmos_sensor_8(ctx, 0xfe,0x00);
write_cmos_sensor_8(ctx, 0x05,0x02);
write_cmos_sensor_8(ctx, 0x06,0xd1); //HB
write_cmos_sensor_8(ctx, 0x07,0x00);
write_cmos_sensor_8(ctx, 0x08,0x22); //VB
write_cmos_sensor_8(ctx, 0xfe,0x01);
write_cmos_sensor_8(ctx, 0x25,0x00); //step 
write_cmos_sensor_8(ctx, 0x26,0x6a); 
write_cmos_sensor_8(ctx, 0x27,0x02); //20fps
write_cmos_sensor_8(ctx, 0x28,0x12);  
write_cmos_sensor_8(ctx, 0x29,0x03); //12.5fps
write_cmos_sensor_8(ctx, 0x2a,0x50); 
write_cmos_sensor_8(ctx, 0x2b,0x05); //7.14fps
write_cmos_sensor_8(ctx, 0x2c,0xcc); 
write_cmos_sensor_8(ctx, 0x2d,0x07); //5.55fps
write_cmos_sensor_8(ctx, 0x2e,0x74);
write_cmos_sensor_8(ctx, 0x3c,0x20);
write_cmos_sensor_8(ctx, 0xfe,0x00);
write_cmos_sensor_8(ctx, 0xfe,0x03);
write_cmos_sensor_8(ctx, 0x10,0x94);
write_cmos_sensor_8(ctx, 0xfe,0x00); 

};
*/

static void addr_data_pair_preview_gc030a(struct subdrv_ctx *ctx){
	write_cmos_sensor_8(ctx, 0xfe,0x03);
	write_cmos_sensor_8(ctx, 0x10,0x90);
	write_cmos_sensor_8(ctx, 0xfe,0x00); 
};

static void addr_data_pair_capture_gc030a(struct subdrv_ctx *ctx){
	write_cmos_sensor_8(ctx, 0xfe,0x03);
	write_cmos_sensor_8(ctx, 0x10,0x90);
	write_cmos_sensor_8(ctx, 0xfe,0x00); 
};

static void addr_data_pair_video_gc030a(struct subdrv_ctx *ctx){
	write_cmos_sensor_8(ctx, 0xfe,0x03);
	write_cmos_sensor_8(ctx, 0x10,0x90);
	write_cmos_sensor_8(ctx, 0xfe,0x00); 
};

static void addr_data_pair_hs_video_gc030a(struct subdrv_ctx *ctx){
	write_cmos_sensor_8(ctx, 0xfe,0x03);
	write_cmos_sensor_8(ctx, 0x10,0x90);
	write_cmos_sensor_8(ctx, 0xfe,0x00); 
};

static void addr_data_pair_slim_video_gc030a(struct subdrv_ctx *ctx){
	write_cmos_sensor_8(ctx, 0xfe,0x03);
	write_cmos_sensor_8(ctx, 0x10,0x90);
	write_cmos_sensor_8(ctx, 0xfe,0x00); 
};

#endif