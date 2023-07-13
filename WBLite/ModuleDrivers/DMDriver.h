#include "main.h"
#include "fdcan.h"
#ifndef DMDRIVER_H
#define DMDRIVER_H
#ifdef __cplusplus

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f


typedef struct{
	uint8_t state;
	float pos;
	float vel;
	float toq;
	float Tmos;
	float Tcoil;
	float Kp;
	float Kd;
}Motor_Inf;

class cDM4310
{
	protected:
		uint16_t CANID;
		FDCAN_HandleTypeDef *hfdcan;
		FDCAN_TxHeaderTypeDef TxHeader;
		Motor_Inf MTR={0},CMD={0};
		uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
		{
			/// Converts a float to an unsigned int, given range and number of bits ///
			float span = x_max - x_min;
			float offset = x_min;
			return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
		}

		float uint_to_float(int x_int, float x_min, float x_max, int bits)
		{
			/// converts unsigned int to float, given range and number of bits ///
			float span = x_max - x_min;
			float offset = x_min;
			return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
		}
	public:
		

		void SetID(FDCAN_HandleTypeDef *hfdcan,uint16_t ID)
		{
			this->CANID	 = ID;
			this->hfdcan = hfdcan;
			
			this->TxHeader.Identifier = ID;
			this->TxHeader.IdType = FDCAN_STANDARD_ID;
			this->TxHeader.TxFrameType = FDCAN_DATA_FRAME;
			this->TxHeader.DataLength = FDCAN_DLC_BYTES_8;
			this->TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
			this->TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
			this->TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
			this->TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
			this->TxHeader.MessageMarker = 0;
		}
		void EnableMotor(void);
		void DisableMotor(void);
		void SetZero(void);
		void MITUpdate(float Position, float Velocity, float KP, float KD, float Torque);
		void MITTransmit(void);
		uint8_t MessageDecode(uint8_t *buf);
		
		inline float GetRadian(void)
		{return this->MTR.pos;}
		inline float GetVelocity(void)
		{return this->MTR.vel;}
};

#endif
#endif