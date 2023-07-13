/*********************************************************************************
  *FileName:	DMAFlag_LL.cpp
  *Author:  	qianwan
  *Details:		LL DMA Flag中断标志抽象层函数,
				用于提供消除中断标志的统一接口

  *Version:  	1.3.0
  *Date:  		2023/04/03
  *Describ:		更改判断条件,提高STREAM优先级
  
  *Version:  	1.2.1
  *Date:  		2023/02/27
  *Describ:		优化接口形式,丰富函数内容
  
  *Version:  	1.1
  *Date:  		2022/09/03
  *Describ:		消除各内核对DMA的差异化调用方式
**********************************************************************************/

#include "DMAFlag_LL.h"
/*是否启用DMA*/
#if defined(DMA1_BASE) || defined(DMA2_BASE) || defined(DMA3_BASE)

/*多态函数,用于配置DMA和通道*/
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

/*通道传输完成*/
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


/*通道传输一半完成*/
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

/*传输错误标志*/
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
/*直接模式错误中断*/
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
/*FIFO错误中断*/
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
/*全局中断标志 仅旧版DMA有此标志*/
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