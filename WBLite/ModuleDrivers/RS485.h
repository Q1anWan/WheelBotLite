#ifndef RS485_H
#define RS485_H
#ifdef __cplusplus
#include "UART_LL.h"
// <<< Use Configuration Wizard in Context Menu >>>
// <c1>����Ӳ������
//  <i>DE����������ƽ��ر�
//#define RS485_HARD_FLOW_CONTROL
// </c>
// <<< end of configuration section >>>

/*�˴��漰ָ��ת��, ���н�����ӱ���*/
class cUART_Access : public cUART
{
	friend class cRS485;
};

class cRS485
{
	protected:
	enum eBusStatue
	{
		BUS_IDLE = 0,
		BUS_BSY = 1
	};
	eBusStatue BusStatue = BUS_BSY;
	#ifndef RS485_HARD_FLOW_CONTROL
	GPIO_TypeDef *DEGPIOx = 0;
	uint32_t DEPinMask = 0;
	#endif
	public:
	cUART_Access *pUART = 0;
	uint8_t SetUART(cUART *pUART, GPIO_TypeDef *GPIOx, uint32_t PinMask);
	uint8_t SetUART(cUART *pUART);
	uint8_t CheckBus(void);
	
	inline uint8_t CheckDE(void){return LL_GPIO_IsOutputPinSet(this->DEGPIOx,this->DEPinMask);}
	inline void SetDE(void){LL_GPIO_SetOutputPin(this->DEGPIOx,this->DEPinMask);}
	inline void ResetDE(void){LL_GPIO_ResetOutputPin(this->DEGPIOx,this->DEPinMask);}
	
	uint8_t BusTransmit(uint8_t* buf, uint8_t len); 
};
#endif
#endif