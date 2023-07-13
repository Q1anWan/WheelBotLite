/*********************************************************************************
  *FileName:	DMAFlag_LL.h
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

		/*��̬��������������DMA��ͨ��*/
		void Config(uint32_t CH);
		void Config(DMA_TypeDef *DMAx);
		void Config(DMA_TypeDef *DMAx, uint32_t CH);
		inline void EnableCH(void);
		inline void DisableCH(void);
		
		/*������ɱ�־*/
		void ClearFlag_TC(void);
		uint32_t GetFlag_TC(void);
		/*�������һ���־*/
		void ClearFlag_HT(void);
		uint32_t GetFlag_HT(void);	
		/*�����������־*/
		void ClearFlag_TE(void);
		uint32_t GetFlag_TE(void);
		/*ȫ���жϱ�־*/
		#if defined(DMA_IFCR_CTCIF1)
		void ClearFlag_GI(void);
		uint32_t GetFlag_GI(void);
		#endif

		#if defined(LL_DMA_STREAM_1)
		/*ֱ��ģʽ�����ж�*/
		void ClearFlag_DME(void);
		uint32_t GetFlag_DME(void);
		/*FIFO�����ж�*/
		void ClearFlag_FE(void);
		uint32_t GetFlag_FE(void);
		#endif

};
/*ͨ��ʹ��ʧ��*/

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