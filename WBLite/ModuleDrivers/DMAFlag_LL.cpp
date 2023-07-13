/*********************************************************************************
  *FileName:	DMAFlag_LL.cpp
  *Author:  	qianwan
  *Details:		LL DMA Flag�жϱ�־����㺯��,
				�����ṩ�����жϱ�־��ͳһ�ӿ�

  *Version:  	1.3.0
  *Date:  		2023/04/03
  *Describ:		�����ж�����,���STREAM���ȼ�
  
  *Version:  	1.2.1
  *Date:  		2023/02/27
  *Describ:		�Ż��ӿ���ʽ,�ḻ��������
  
  *Version:  	1.1
  *Date:  		2022/09/03
  *Describ:		�������ں˶�DMA�Ĳ��컯���÷�ʽ
**********************************************************************************/

#include "DMAFlag_LL.h"
/*�Ƿ�����DMA*/
#if defined(DMA1_BASE) || defined(DMA2_BASE) || defined(DMA3_BASE)

/*��̬����,��������DMA��ͨ��*/
void cDMA::Config(DMA_TypeDef *DMAx)
{
	this->DMAx = DMAx;
}
void cDMA::Config(uint32_t CH)
{
	this->CH = CH;
}
void cDMA::Config(DMA_TypeDef *DMAx, uint32_t CH)
{
	this->DMAx = DMAx;
	this->CH = CH;
}

/*ͨ���������*/
#if defined(LL_DMA_STREAM_1)
void cDMA::ClearFlag_TC(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_STREAM_0)
		case LL_DMA_STREAM_0:
			LL_DMA_ClearFlag_TC0(this->DMAx);
		break;
		#endif
		case LL_DMA_STREAM_1:
			LL_DMA_ClearFlag_TC1(this->DMAx);
		break;
		case LL_DMA_STREAM_2:
			LL_DMA_ClearFlag_TC2(this->DMAx);
		break;
		case LL_DMA_STREAM_3:
			LL_DMA_ClearFlag_TC3(this->DMAx);
		break;
		case LL_DMA_STREAM_4:
			LL_DMA_ClearFlag_TC4(this->DMAx);
		break;
		case LL_DMA_STREAM_5:
			LL_DMA_ClearFlag_TC5(this->DMAx);
		break;
		case LL_DMA_STREAM_6:
			LL_DMA_ClearFlag_TC6(this->DMAx);
		break;
		case LL_DMA_STREAM_7:
			LL_DMA_ClearFlag_TC7(this->DMAx);
		break;
		#if defined(LL_DMA_STREAM_8)
		case LL_DMA_STREAM_8:
			LL_DMA_ClearFlag_TC8(this->DMAx);
		break;
		#endif
	}
}
uint32_t cDMA::GetFlag_TC(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_STREAM_0)
		case LL_DMA_STREAM_0:
			return LL_DMA_IsActiveFlag_TC0(this->DMAx);
		break;
		#endif
		case LL_DMA_STREAM_1:
			return LL_DMA_IsActiveFlag_TC1(this->DMAx);
		break;
		case LL_DMA_STREAM_2:
			return LL_DMA_IsActiveFlag_TC2(this->DMAx);
		break;
		case LL_DMA_STREAM_3:
			return LL_DMA_IsActiveFlag_TC3(this->DMAx);
		break;
		case LL_DMA_STREAM_4:
			return LL_DMA_IsActiveFlag_TC4(this->DMAx);
		break;
		case LL_DMA_STREAM_5:
			return LL_DMA_IsActiveFlag_TC5(this->DMAx);
		break;
		case LL_DMA_STREAM_6:
			return LL_DMA_IsActiveFlag_TC6(this->DMAx);
		break;
		case LL_DMA_STREAM_7:
			return LL_DMA_IsActiveFlag_TC7(this->DMAx);
		break;
		#if defined(LL_DMA_STREAM_8)
		case LL_DMA_STREAM_8:
			return LL_DMA_IsActiveFlag_TC8(this->DMAx);
		break;
		#endif
	}
	return 0;
}
#elif defined(LL_DMA_CHANNEL_1)
void cDMA::ClearFlag_TC(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_CHANNEL_0)
		case LL_DMA_CHANNEL_0:
			LL_DMA_ClearFlag_TC0(this->DMAx);return;
		break;
		#endif
		case LL_DMA_CHANNEL_1:
			LL_DMA_ClearFlag_TC1(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_2:
			LL_DMA_ClearFlag_TC2(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_3:
			LL_DMA_ClearFlag_TC3(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_4:
			LL_DMA_ClearFlag_TC4(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_5:
			LL_DMA_ClearFlag_TC5(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_6:
			LL_DMA_ClearFlag_TC6(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_7:
			LL_DMA_ClearFlag_TC7(this->DMAx);return;
		break;
		#if defined(LL_DMA_CHANNEL_8)
		case LL_DMA_CHANNEL_8:
			LL_DMA_ClearFlag_TC8(this->DMAx);return;
		break;
		#endif
	}
}
uint32_t cDMA::GetFlag_TC(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_CHANNEL_0)
		case LL_DMA_CHANNEL_0:
			return LL_DMA_IsActiveFlag_TC0(this->DMAx);
		break;
		#endif
		case LL_DMA_CHANNEL_1:
			return LL_DMA_IsActiveFlag_TC1(this->DMAx);
		break;
		case LL_DMA_CHANNEL_2:
			return LL_DMA_IsActiveFlag_TC2(this->DMAx);
		break;
		case LL_DMA_CHANNEL_3:
			return LL_DMA_IsActiveFlag_TC3(this->DMAx);
		break;
		case LL_DMA_CHANNEL_4:
			return LL_DMA_IsActiveFlag_TC4(this->DMAx);
		break;
		case LL_DMA_CHANNEL_5:
			return LL_DMA_IsActiveFlag_TC5(this->DMAx);
		break;
		case LL_DMA_CHANNEL_6:
			return LL_DMA_IsActiveFlag_TC6(this->DMAx);
		break;
		case LL_DMA_CHANNEL_7:
			return LL_DMA_IsActiveFlag_TC7(this->DMAx);
		break;
		#if defined(LL_DMA_CHANNEL_8)
		case LL_DMA_CHANNEL_8:
			return LL_DMA_IsActiveFlag_TC8(this->DMAx);
		break;
		#endif
	}
	return 1;
}
#endif


/*ͨ������һ�����*/
#if defined(LL_DMA_STREAM_1)
void cDMA::ClearFlag_HT(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_STREAM_0)
		case LL_DMA_STREAM_0:
			LL_DMA_ClearFlag_HT0(this->DMAx);return;
		break;
		#endif
		case LL_DMA_STREAM_1:
			LL_DMA_ClearFlag_HT1(this->DMAx);return;
		break;
		case LL_DMA_STREAM_2:
			LL_DMA_ClearFlag_HT2(this->DMAx);return;
		break;
		case LL_DMA_STREAM_3:
			LL_DMA_ClearFlag_HT3(this->DMAx);return;
		break;
		case LL_DMA_STREAM_4:
			LL_DMA_ClearFlag_HT4(this->DMAx);return;
		break;
		case LL_DMA_STREAM_5:
			LL_DMA_ClearFlag_HT5(this->DMAx);return;
		break;
		case LL_DMA_STREAM_6:
			LL_DMA_ClearFlag_HT6(this->DMAx);return;
		break;
		case LL_DMA_STREAM_7:
			LL_DMA_ClearFlag_HT7(this->DMAx);return;
		break;
		#if defined(LL_DMA_STREAM_8)
		case LL_DMA_STREAM_8:
			LL_DMA_ClearFlag_HT8(this->DMAx);return;
		break;
		#endif
	}
}
uint32_t cDMA::GetFlag_HT(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_STREAM_0)
		case LL_DMA_STREAM_0:
			return LL_DMA_IsActiveFlag_HT0(this->DMAx);
		break;
		#endif
		case LL_DMA_STREAM_1:
			return LL_DMA_IsActiveFlag_HT1(this->DMAx);
		break;
		case LL_DMA_STREAM_2:
			return LL_DMA_IsActiveFlag_HT2(this->DMAx);
		break;
		case LL_DMA_STREAM_3:
			return LL_DMA_IsActiveFlag_HT3(this->DMAx);
		break;
		case LL_DMA_STREAM_4:
			return LL_DMA_IsActiveFlag_HT4(this->DMAx);
		break;
		case LL_DMA_STREAM_5:
			return LL_DMA_IsActiveFlag_HT5(this->DMAx);
		break;
		case LL_DMA_STREAM_6:
			return LL_DMA_IsActiveFlag_HT6(this->DMAx);
		break;
		case LL_DMA_STREAM_7:
			return LL_DMA_IsActiveFlag_HT7(this->DMAx);
		break;
		#if defined(LL_DMA_STREAM_8)
		case LL_DMA_STREAM_8:
			return LL_DMA_IsActiveFlag_HT8(this->DMAx);
		break;
		#endif
	}
	return 1;
}
#elif defined(LL_DMA_CHANNEL_1)
void cDMA::ClearFlag_HT(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_CHANNEL_0)
		case LL_DMA_CHANNEL_0:
			LL_DMA_ClearFlag_HT0(this->DMAx);return;
		break;
		#endif
		case LL_DMA_CHANNEL_1:
			LL_DMA_ClearFlag_HT1(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_2:
			LL_DMA_ClearFlag_HT2(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_3:
			LL_DMA_ClearFlag_HT3(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_4:
			LL_DMA_ClearFlag_HT4(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_5:
			LL_DMA_ClearFlag_HT5(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_6:
			LL_DMA_ClearFlag_HT6(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_7:
			LL_DMA_ClearFlag_HT7(this->DMAx);return;
		break;
		#if defined(LL_DMA_CHANNEL_8)
		case LL_DMA_CHANNEL_8:
			LL_DMA_ClearFlag_HT8(this->DMAx);return;
		break;
		#endif
	}
}
uint32_t cDMA::GetFlag_HT(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_CHANNEL_0)
		case LL_DMA_CHANNEL_0:
			return LL_DMA_IsActiveFlag_HT0(this->DMAx);
		break;
		#endif
		case LL_DMA_CHANNEL_1:
			return LL_DMA_IsActiveFlag_HT1(this->DMAx);
		break;
		case LL_DMA_CHANNEL_2:
			return LL_DMA_IsActiveFlag_HT2(this->DMAx);
		break;
		case LL_DMA_CHANNEL_3:
			return LL_DMA_IsActiveFlag_HT3(this->DMAx);
		break;
		case LL_DMA_CHANNEL_4:
			return LL_DMA_IsActiveFlag_HT4(this->DMAx);
		break;
		case LL_DMA_CHANNEL_5:
			return LL_DMA_IsActiveFlag_HT5(this->DMAx);
		break;
		case LL_DMA_CHANNEL_6:
			return LL_DMA_IsActiveFlag_HT6(this->DMAx);
		break;
		case LL_DMA_CHANNEL_7:
			return LL_DMA_IsActiveFlag_HT7(this->DMAx);
		break;
		#if defined(LL_DMA_CHANNEL_8)
		case LL_DMA_CHANNEL_8:
			return LL_DMA_IsActiveFlag_HT8(this->DMAx);
		break;
		#endif
	}
	return 1;
}
#endif

/*��������־*/
#if defined(LL_DMA_STREAM_1)
void cDMA::ClearFlag_TE(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_STREAM_0)
		case LL_DMA_STREAM_0:
			LL_DMA_ClearFlag_TE0(this->DMAx);return;
		break;
		#endif
		case LL_DMA_STREAM_1:
			LL_DMA_ClearFlag_TE1(this->DMAx);return;
		break;
		case LL_DMA_STREAM_2:
			LL_DMA_ClearFlag_TE2(this->DMAx);return;
		break;
		case LL_DMA_STREAM_3:
			LL_DMA_ClearFlag_TE3(this->DMAx);return;
		break;
		case LL_DMA_STREAM_4:
			LL_DMA_ClearFlag_TE4(this->DMAx);return;
		break;
		case LL_DMA_STREAM_5:
			LL_DMA_ClearFlag_TE5(this->DMAx);return;
		break;
		case LL_DMA_STREAM_6:
			LL_DMA_ClearFlag_TE6(this->DMAx);return;
		break;
		case LL_DMA_STREAM_7:
			LL_DMA_ClearFlag_TE7(this->DMAx);return;
		break;
		#if defined(LL_DMA_STREAM_8)
		case LL_DMA_STREAM_8:
			LL_DMA_ClearFlag_TE8(this->DMAx);return;
		break;
		#endif
	}
}
uint32_t cDMA::GetFlag_TE(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_STREAM_0)
		case LL_DMA_STREAM_0:
			return LL_DMA_IsActiveFlag_TE0(this->DMAx);
		break;
		#endif
		case LL_DMA_STREAM_1:
			return LL_DMA_IsActiveFlag_TE1(this->DMAx);
		break;
		case LL_DMA_STREAM_2:
			return LL_DMA_IsActiveFlag_TE2(this->DMAx);
		break;
		case LL_DMA_STREAM_3:
			return LL_DMA_IsActiveFlag_TE3(this->DMAx);
		break;
		case LL_DMA_STREAM_4:
			return LL_DMA_IsActiveFlag_TE4(this->DMAx);
		break;
		case LL_DMA_STREAM_5:
			return LL_DMA_IsActiveFlag_TE5(this->DMAx);
		break;
		case LL_DMA_STREAM_6:
			return LL_DMA_IsActiveFlag_TE6(this->DMAx);
		break;
		case LL_DMA_STREAM_7:
			return LL_DMA_IsActiveFlag_TE7(this->DMAx);
		break;
		#if defined(LL_DMA_STREAM_8)
		case LL_DMA_STREAM_8:
			return LL_DMA_IsActiveFlag_TE8(this->DMAx);
		break;
		#endif
	}
	return 1;
}
#elif defined(LL_DMA_CHANNEL_1)
void cDMA::ClearFlag_TE(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_CHANNEL_0)
		case LL_DMA_CHANNEL_0:
			LL_DMA_ClearFlag_TE0(this->DMAx);return;
		break;
		#endif
		case LL_DMA_CHANNEL_1:
			LL_DMA_ClearFlag_TE1(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_2:
			LL_DMA_ClearFlag_TE2(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_3:
			LL_DMA_ClearFlag_TE3(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_4:
			LL_DMA_ClearFlag_TE4(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_5:
			LL_DMA_ClearFlag_TE5(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_6:
			LL_DMA_ClearFlag_TE6(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_7:
			LL_DMA_ClearFlag_TE7(this->DMAx);return;
		break;
		#if defined(LL_DMA_CHANNEL_8)
		case LL_DMA_CHANNEL_8:
			LL_DMA_ClearFlag_TE8(this->DMAx);return;
		break;
		#endif
	}
}
uint32_t cDMA::GetFlag_TE(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_CHANNEL_0)
		case LL_DMA_CHANNEL_0:
			return LL_DMA_IsActiveFlag_TE0(this->DMAx);
		break;
		#endif
		case LL_DMA_CHANNEL_1:
			return LL_DMA_IsActiveFlag_TE1(this->DMAx);
		break;
		case LL_DMA_CHANNEL_2:
			return LL_DMA_IsActiveFlag_TE2(this->DMAx);
		break;
		case LL_DMA_CHANNEL_3:
			return LL_DMA_IsActiveFlag_TE3(this->DMAx);
		break;
		case LL_DMA_CHANNEL_4:
			return LL_DMA_IsActiveFlag_TE4(this->DMAx);
		break;
		case LL_DMA_CHANNEL_5:
			return LL_DMA_IsActiveFlag_TE5(this->DMAx);
		break;
		case LL_DMA_CHANNEL_6:
			return LL_DMA_IsActiveFlag_TE6(this->DMAx);
		break;
		case LL_DMA_CHANNEL_7:
			return LL_DMA_IsActiveFlag_TE7(this->DMAx);
		break;
		#if defined(LL_DMA_CHANNEL_8)
		case LL_DMA_CHANNEL_8:
			return LL_DMA_IsActiveFlag_TE8(this->DMAx);
		break;
		#endif
	}
	return 1;
}
#endif

#if defined(LL_DMA_STREAM_1)
/*ֱ��ģʽ�����ж�*/
void cDMA::ClearFlag_DME(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_STREAM_0)
		case LL_DMA_STREAM_0:
			LL_DMA_ClearFlag_DME0(this->DMAx);return;
		break;
		#endif
		case LL_DMA_STREAM_1:
			LL_DMA_ClearFlag_DME1(this->DMAx);return;
		break;
		case LL_DMA_STREAM_2:
			LL_DMA_ClearFlag_DME2(this->DMAx);return;
		break;
		case LL_DMA_STREAM_3:
			LL_DMA_ClearFlag_DME3(this->DMAx);return;
		break;
		case LL_DMA_STREAM_4:
			LL_DMA_ClearFlag_DME4(this->DMAx);return;
		break;
		case LL_DMA_STREAM_5:
			LL_DMA_ClearFlag_DME5(this->DMAx);return;
		break;
		case LL_DMA_STREAM_6:
			LL_DMA_ClearFlag_DME6(this->DMAx);return;
		break;
		case LL_DMA_STREAM_7:
			LL_DMA_ClearFlag_DME7(this->DMAx);return;
		break;
		#if defined(LL_DMA_STREAM_8)
		case LL_DMA_STREAM_8:
			LL_DMA_ClearFlag_DME8(this->DMAx);return;
		break;
		#endif
	}
}
uint32_t cDMA::GetFlag_DME(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_STREAM_0)
		case LL_DMA_STREAM_0:
			return LL_DMA_IsActiveFlag_DME0(this->DMAx);
		break;
		#endif
		case LL_DMA_STREAM_1:
			return LL_DMA_IsActiveFlag_DME1(this->DMAx);
		break;
		case LL_DMA_STREAM_2:
			return LL_DMA_IsActiveFlag_DME2(this->DMAx);
		break;
		case LL_DMA_STREAM_3:
			return LL_DMA_IsActiveFlag_DME3(this->DMAx);
		break;
		case LL_DMA_STREAM_4:
			return LL_DMA_IsActiveFlag_DME4(this->DMAx);
		break;
		case LL_DMA_STREAM_5:
			return LL_DMA_IsActiveFlag_DME5(this->DMAx);
		break;
		case LL_DMA_STREAM_6:
			return LL_DMA_IsActiveFlag_DME6(this->DMAx);
		break;
		case LL_DMA_STREAM_7:
			return LL_DMA_IsActiveFlag_DME7(this->DMAx);
		break;
		#if defined(LL_DMA_STREAM_8)
		case LL_DMA_STREAM_8:
			return LL_DMA_IsActiveFlag_DME8(this->DMAx);
		break;
		#endif
	}
	return 1;
}
/*FIFO�����ж�*/
void cDMA::ClearFlag_FE(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_STREAM_0)
		case LL_DMA_STREAM_0:
			LL_DMA_ClearFlag_FE0(this->DMAx);return;
		break;
		#endif
		case LL_DMA_STREAM_1:
			LL_DMA_ClearFlag_FE1(this->DMAx);return;
		break;
		case LL_DMA_STREAM_2:
			LL_DMA_ClearFlag_FE2(this->DMAx);return;
		break;
		case LL_DMA_STREAM_3:
			LL_DMA_ClearFlag_FE3(this->DMAx);return;
		break;
		case LL_DMA_STREAM_4:
			LL_DMA_ClearFlag_FE4(this->DMAx);return;
		break;
		case LL_DMA_STREAM_5:
			LL_DMA_ClearFlag_FE5(this->DMAx);return;
		break;
		case LL_DMA_STREAM_6:
			LL_DMA_ClearFlag_FE6(this->DMAx);return;
		break;
		case LL_DMA_STREAM_7:
			LL_DMA_ClearFlag_FE7(this->DMAx);return;
		break;
		#if defined(LL_DMA_STREAM_8)
		case LL_DMA_STREAM_8:
			LL_DMA_ClearFlag_FE8(this->DMAx);return;
		break;
		#endif
	}
}
uint32_t cDMA::GetFlag_FE(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_STREAM_0)
		case LL_DMA_STREAM_0:
			return LL_DMA_IsActiveFlag_FE0(this->DMAx);
		break;
		#endif
		case LL_DMA_STREAM_1:
			return LL_DMA_IsActiveFlag_FE1(this->DMAx);
		break;
		case LL_DMA_STREAM_2:
			return LL_DMA_IsActiveFlag_FE2(this->DMAx);
		break;
		case LL_DMA_STREAM_3:
			return LL_DMA_IsActiveFlag_FE3(this->DMAx);
		break;
		case LL_DMA_STREAM_4:
			return LL_DMA_IsActiveFlag_FE4(this->DMAx);
		break;
		case LL_DMA_STREAM_5:
			return LL_DMA_IsActiveFlag_FE5(this->DMAx);
		break;
		case LL_DMA_STREAM_6:
			return LL_DMA_IsActiveFlag_FE6(this->DMAx);
		break;
		case LL_DMA_STREAM_7:
			return LL_DMA_IsActiveFlag_FE7(this->DMAx);
		break;
		#if defined(LL_DMA_STREAM_8)
		case LL_DMA_STREAM_8:
			return LL_DMA_IsActiveFlag_FE8(this->DMAx);
		break;
		#endif
	}
	return 1;
}
/*ȫ���жϱ�־ ���ɰ�DMA�д˱�־*/
#elif defined(LL_DMA_CHANNEL_1)
void cDMA::ClearFlag_GI(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_CHANNEL_0)
		case LL_DMA_CHANNEL_0:
			LL_DMA_ClearFlag_GI0(this->DMAx);return;
		break;
		#endif
		case LL_DMA_CHANNEL_1:
			LL_DMA_ClearFlag_GI1(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_2:
			LL_DMA_ClearFlag_GI2(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_3:
			LL_DMA_ClearFlag_GI3(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_4:
			LL_DMA_ClearFlag_GI4(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_5:
			LL_DMA_ClearFlag_GI5(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_6:
			LL_DMA_ClearFlag_GI6(this->DMAx);return;
		break;
		case LL_DMA_CHANNEL_7:
			LL_DMA_ClearFlag_GI7(this->DMAx);return;
		break;
		#if defined(LL_DMA_CHANNEL_8)
		case LL_DMA_CHANNEL_8:
			LL_DMA_ClearFlag_GI8(this->DMAx);return;
		break;
		#endif
	}
}
uint32_t cDMA::GetFlag_GI(void)
{
	switch (this->CH)
	{
		#if defined(LL_DMA_CHANNEL_0)
		case LL_DMA_CHANNEL_0:
			return LL_DMA_IsActiveFlag_GI0(this->DMAx);
		break;
		#endif
		case LL_DMA_CHANNEL_1:
			return LL_DMA_IsActiveFlag_GI1(this->DMAx);
		break;
		case LL_DMA_CHANNEL_2:
			return LL_DMA_IsActiveFlag_GI2(this->DMAx);
		break;
		case LL_DMA_CHANNEL_3:
			return LL_DMA_IsActiveFlag_GI3(this->DMAx);
		break;
		case LL_DMA_CHANNEL_4:
			return LL_DMA_IsActiveFlag_GI4(this->DMAx);
		break;
		case LL_DMA_CHANNEL_5:
			return LL_DMA_IsActiveFlag_GI5(this->DMAx);
		break;
		case LL_DMA_CHANNEL_6:
			return LL_DMA_IsActiveFlag_GI6(this->DMAx);
		break;
		case LL_DMA_CHANNEL_7:
			return LL_DMA_IsActiveFlag_GI7(this->DMAx);
		break;
		#if defined(LL_DMA_CHANNEL_8)
		case LL_DMA_CHANNEL_8:
			return LL_DMA_IsActiveFlag_GI8(this->DMAx);
		break;
		#endif
	}
	return 1;
}
#endif
#endif