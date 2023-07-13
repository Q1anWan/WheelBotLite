#include "CloseLoopTask.h"
#include "MotorTask.h"
#include "ControlTask.h"

extern cMotorUnit *MotorUnit;
extern cRobotControl *RobotControl;

TX_THREAD	CloseLoopThread;
uint8_t 	CloseLoopThreadStack[512]={0};
float outbut[2];
void CloseLoopThreadFun(ULONG initial_input)
{
	
	ULONG timer;
	RobotControl->ChasisControl.ObserveVal.X[0]=1.5;
	for(;;)
	{
		timer = tx_time_get();
		if(RobotControl->CheckRobotMode()==ROBOTMODE_IDLE)
		{
			/*Shutdown!*/
			MotorUnit->Motor[0].MITUpdate(0,0,0,0,0);
			MotorUnit->Motor[1].MITUpdate(0,0,0,0,0);
			MotorUnit->Motor[2].MITUpdate(0,0,0,0,0);
			MotorUnit->Motor[3].MITUpdate(0,0,0,0,0);
			MotorUnit->Motor[4].MITUpdate(0,0,0,0,0);
			MotorUnit->Motor[5].MITUpdate(0,0,0,0,0);
		}
		else if(RobotControl->CheckRobotMode()==ROBOTMODE_ESCAPE)
		{
			/*Open-loop control*/
			MotorUnit->Motor[0].MITUpdate(0,0,0,0,0);
			MotorUnit->Motor[1].MITUpdate(0,0,0,0,0);
			MotorUnit->Motor[2].MITUpdate(0,0,0,0,0);
			MotorUnit->Motor[3].MITUpdate(0,0,0,0,0);
			MotorUnit->Motor[4].MITUpdate(0,0,0,0,RobotControl->ChasisControl.OpenLoopWheelI[0]);
			MotorUnit->Motor[5].MITUpdate(0,0,0,0,RobotControl->ChasisControl.OpenLoopWheelI[1]);			
		}
		else
		{
			/*Shutdown!*/
			MotorUnit->Motor[0].MITUpdate(0,0,0,0,0);
			MotorUnit->Motor[1].MITUpdate(0,0,0,0,0);
			MotorUnit->Motor[2].MITUpdate(0,0,0,0,0);
			MotorUnit->Motor[3].MITUpdate(0,0,0,0,0);
			MotorUnit->Motor[4].MITUpdate(0,0,0,0,0);
			MotorUnit->Motor[5].MITUpdate(0,0,0,0,0);
		}
			
		
		
		tx_thread_sleep_until(&timer,2);
	}
}
