#ifndef MOTORTASK_H
#define MOTORTASK_H
#ifdef __cplusplus
#include "main.h"
#include "fdcan.h"
#include "Filter.h"
#include "arm_math.h"
#include "DMDriver.h"
#include "LinkSolver.h"


#define AngelCalib   0.34472f
#define AngelCalib0  1.915516f // PI/2 + CAL
#define AngelCalib1  1.226076f // PI/2 - CAL

#define WHEELCOEFF	0.0625f
class cMotorUnit
{
	protected:
	float velocity = 0.0f;
	float dlast[2] = {0};
	float displacement = 0.0f;
	
	cFilterBTW2_40Hz FilterV;
	cFilterBTW2_40Hz FilterD;
	
	public:
	cDM4310 Motor[6];
	cLinkSolver LinkSolver[2];/*0左1右*/
	void UpdateLink(void);
	void UpdateOdomentor(void);
	void InitOdomentor(void);
	inline float GetVel(void)
	{return this->velocity;}
	inline float GetDis(void)
	{return this->displacement;}
	
	/*0左关节后 1左关节前 4左轮子*/
	/*3右关节后 2右关节前 5右轮子*/
};


extern "C" {
void CANFilterConfig(void);	
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);
}

#endif
#endif