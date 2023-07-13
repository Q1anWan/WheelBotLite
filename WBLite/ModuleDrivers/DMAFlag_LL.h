/*********************************************************************************
  *FileName:	DMAFlag_LL.h
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

#ifndef DMAFLAG_LL_H
#define DMAFLAG_LL_H
#include "main.h"

#if defined(DMA1_BASE) || defined(DMA2_BASE) || defined(DMA3_BASE)

#include "dma.h"
#ifdef __cplusplus

class cDMA
{
	public:
		DMA_TypeDef *DMAx;
		uint32_t CH;

		/*多态函数，用于配置DMA和通道*/
		void Config(uint32_t CH);
		void Config(DMA_TypeDef *DMAx);
		void Config(DMA_TypeDef *DMAx, uint32_t CH);
		inline void EnableCH(void);
		inline void DisableCH(void);
		
		/*传输完成标志*/
		void ClearFlag_TC(void);
		uint32_t GetFlag_TC(void);
		/*传输完成一半标志*/
		void ClearFlag_HT(void);
		uint32_t GetFlag_HT(void);	
		/*清除传输错误标志*/
		void ClearFlag_TE(void);
		uint32_t GetFlag_TE(void);
		/*全局中断标志*/
		#if defined(DMA_IFCR_CTCIF1)
		void ClearFlag_GI(void);
		uint32_t GetFlag_GI(void);
		#endif

		#if defined(LL_DMA_STREAM_1)
		/*直接模式错误中断*/
		void ClearFlag_DME(void);
		uint32_t GetFlag_DME(void);
		/*FIFO错误中断*/
		void ClearFlag_FE(void);
		uint32_t GetFlag_FE(void);
		#endif

};
/*通道使能失能*/

#if defined(LL_DMA_STREAM_1)
inline void cDMA::EnableCH(void)
{
	LL_DMA_EnableStream(this->DMAx, this->CH);
}
inline void cDMA::DisableCH(void)
{
	LL_DMA_DisableStream(this->DMAx, this->CH);
}
#elif defined(LL_DMA_CHANNEL_1)
inline void cDMA::EnableCH(void)
{
	LL_DMA_EnableChannel(this->DMAx, this->CH);
}
inline void cDMA::DisableCH(void)
{
	LL_DMA_DisableChannel(this->DMAx, this->CH);
}
#endif
extern "C"
{
}
#endif
#endif
#endif