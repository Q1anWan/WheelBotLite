#ifndef DL_H750_H
#define DL_H750_H

/*
	ʹ���Զ���ɢ�����ļ�ʱ
	��ָ��:ʹ��DTCM,����DTCM��ǰ0x400�ѱ����������жϱ�
	����ĺ�����ָ��SRAM��
*/
#define SRAM_SET_DTCM	__attribute__((section(".RAM_DTCM")))
#define SRAM_SET_D1		__attribute__((section(".RAM_D1")))
#define SRAM_SET_D2		__attribute__((section(".RAM_D2")))
#define SRAM_SET_D3		__attribute__((section(".RAM_D3")))

#endif