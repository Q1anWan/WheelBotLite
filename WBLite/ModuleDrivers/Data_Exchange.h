/**********************************
		qianwan's Code
		  2022.4.10
///////////////////////////////////	
	���ݴ���
	������������ת��
	ת��ǰ������Ϊ�ɸ�λ����λ(���)
	���鳤�������bit���й�
**********************************/
#ifndef DATA_EXCHANGE_H
#define DATA_EXCHANGE_H
#include "main.h"
//�ṹ���װ����
struct Transform_t
{
	void (*Float_To_U8)(float *Input, uint8_t *Output, uint8_t RawNum);
	void (*U8_To_Float)(uint8_t *Input, float *Output, uint8_t RawNum);
	void (*Float_To_U16)(float *Input, uint16_t *Output, uint8_t RawNum);
	void (*U16_To_Float)(uint16_t *Input, float *Output, uint8_t RawNum);
	void (*Float_To_U32)(float *Input, uint32_t *Output, uint8_t RawNum);
	void (*U32_To_Float)(uint32_t *Input, float *Output, uint8_t RawNum);
};

extern struct Transform_t Transform;
#endif