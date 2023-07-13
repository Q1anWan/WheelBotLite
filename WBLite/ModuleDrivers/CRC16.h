/*********************************************************************************
  *FileName:	    CRC16.h
  *Author:				qianwan
  *Details:				查表法计算CRC
  *Mode:				CRC-16/MAXIM
  *Polynomial:    	X16+X15+X2+1(0x8005)
  *Init:					0x0000
  *XOROUT:        	0xFFFF

  *Version:  			1.0
  *Date:  				2023/04/07
  *Describe:    		创建代码
**********************************************************************************/
#ifndef CRC16_H
#define CRC16_H
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
uint16_t cal_crc16_table(uint8_t *ptr, uint32_t len);
#ifdef __cplusplus
}
#endif
#endif