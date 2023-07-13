#ifndef DL_H750_H
#define DL_H750_H

/*
	使用自定分散加载文件时
	不指定:使用DTCM,其中DTCM的前0x400已被用作向量中断表
	下面的函数将指定SRAM域
*/
#define SRAM_SET_DTCM	__attribute__((section(".RAM_DTCM")))
#define SRAM_SET_D1		__attribute__((section(".RAM_D1")))
#define SRAM_SET_D2		__attribute__((section(".RAM_D2")))
#define SRAM_SET_D3		__attribute__((section(".RAM_D3")))

#endif