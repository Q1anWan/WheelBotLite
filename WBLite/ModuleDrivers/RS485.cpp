#include "RS485.h"

uint8_t cRS485::SetUART(cUART *pUART, GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
	if(pUART==0){return 1;}
	this->pUART = (cUART_Access*)pUART;
	this->DEGPIOx = GPIOx;
	this->DEPinMask = PinMask;
	return 0;
}

uint8_t cRS485::SetUART(cUART *pUART)
{
	if(pUART==0){return 1;}
	this->pUART = (cUART_Access*)pUART;
	return 0;
}

uint8_t cRS485::CheckBus(void)
{
	uint8_t Flag = BUS_IDLE;
	
	/*�Ƿ����ڷ���, ʹ��cUART���ͱ�ʶ���ж�*/
	if(this->pUART->TX_Statue==this->pUART->STATUE_BSY)
	{
		Flag = BUS_BSY;
	}
	
	/*�Ƿ����ڷ���, ʹ��DMAʣ�ഫ�����������ڴ��������ж�*/	
	if(LL_DMA_GetDataLength(this->pUART->DMAr.DMAx,this->pUART->DMAr.CH) != this->pUART->TargetRecieve_Length)
	{
		Flag = BUS_BSY;
	}
	
	this->BusStatue = (eBusStatue)Flag;
	return Flag;
}

uint8_t cRS485::BusTransmit(uint8_t* buf, uint8_t len)
{
	if(this->CheckBus()==BUS_IDLE)
	{	
		this->SetDE();
		this->pUART->Transmit(buf,len,0xFFFFFFFF);
		this->ResetDE();
	}
	return 1;
}
