#ifndef REMOTERTASK_H
#define REMOTERTASK_H
#include "main.h"
#include "UART_LL.h"


#define SBUS_DATA_LEN 25
#define DBUS_DATA_LEN 18
#define BUS_LEN SBUS_DATA_LEN

#if	(BUS_LEN==SBUS_DATA_LEN)

#define BUS_MIN		200
#define BUS_MAX		1800
#define BUS_DEATHZOON	10

#elif (BUS_LEN==DBUS_DATA_LEN)

#define BUS_MIN		0
#define BUS_MAX		1320
#define BUS_DEATHZOON	0

#endif

/*
	-1.0 - 1.0
	坐标系左手系
	左最大上最大逆时针增大
	SW 上到下 321
*/
typedef __PACKED_STRUCT
{
	__PACKED_STRUCT
	{
		float CH0;
		float CH1;
		float CH2;
		float CH3;
		float WHEEL;
		uint8_t SW1;
		uint8_t SW2;
	}rmt;
	
	__PACKED_STRUCT
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	
	__PACKED_STRUCT
	 {
		  uint16_t W : 1;
		  uint16_t S : 1;
		  uint16_t A : 1;
		  uint16_t D : 1;
		  uint16_t SHIFT : 1;
		  uint16_t CTRL : 1;
		  uint16_t Q : 1;
		  uint16_t E : 1;
		  uint16_t R : 1;
		  uint16_t F : 1;
		  uint16_t G : 1;
		  uint16_t Z : 1;
		  uint16_t X : 1;
		  uint16_t C : 1;
		  uint16_t V : 1;
		  uint16_t B : 1;
	 }key;
	 
}RC_Ctl_t;

class cRemoter : public cUART
{
	public:
	uint8_t	 *pbuf = 0;
	RC_Ctl_t *RC_Data = 0;
	uint8_t  IsRCOffline = 0;
	void ConfigRemoter(RC_Ctl_t* RCData)
	{
		RC_Data = RCData;
	}
};

extern "C" {
	void UART4_IRQHandler(void);
	void DMA1_Stream0_IRQHandler(void);
}
#endif