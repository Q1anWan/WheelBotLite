#include "CloseLoopTask.h"
#include "MotorTask.h"
#include "ControlTask.h"
#include "IMUTask.h"
#include "Filter.h"
#include "RemoterTask.h"

extern cRemoter *Msg_Remoter;
extern cRobotControl *RobotControl;
extern cINS *INS;


TX_THREAD	CloseLoopThread;
uint8_t 	CloseLoopThreadStack[512]={0};
float outbut[2];
void CloseLoopThreadFun(ULONG initial_input)
{
	
	ULONG timer;

	cFilterBTW2_40Hz VelocityBTW;
	cFilterBTW2_40Hz DisplacementBTW;
	
	for(;;)
	{
		timer = tx_time_get();
		
		RobotControl->ChasisControl.ObserveVal.ChasisPsaiYaw	= QCS.Yaw(INS->Q);
		RobotControl->ChasisControl.ObserveVal.ChasisRoll		= QCS.Roll(INS->Q);
		RobotControl->ChasisControl.ObserveVal.ChasisPitch		= QCS.Pitch(INS->Q);
		
		if(RobotControl->CheckRobotMode()==ROBOTMODE_IDLE)
		{
			/*Shutdown!*/
			RobotControl->ChasisControl.MotorUnits->Motor[0].MITUpdate(0,0,0,0,0);
			RobotControl->ChasisControl.MotorUnits->Motor[1].MITUpdate(0,0,0,0,0);
			RobotControl->ChasisControl.MotorUnits->Motor[2].MITUpdate(0,0,0,0,0);
			RobotControl->ChasisControl.MotorUnits->Motor[3].MITUpdate(0,0,0,0,0);
			RobotControl->ChasisControl.MotorUnits->Motor[4].MITUpdate(0,0,0,0,0);
			RobotControl->ChasisControl.MotorUnits->Motor[5].MITUpdate(0,0,0,0,0);
						
		}
		else if(RobotControl->CheckRobotMode()==ROBOTMODE_ESCAPE)
		{
			/*Open-loop control*/
			RobotControl->ChasisControl.MotorUnits->Motor[0].MITUpdate(0,0,0,0,0);
			RobotControl->ChasisControl.MotorUnits->Motor[1].MITUpdate(0,0,0,0,0);
			RobotControl->ChasisControl.MotorUnits->Motor[2].MITUpdate(0,0,0,0,0);
			RobotControl->ChasisControl.MotorUnits->Motor[3].MITUpdate(0,0,0,0,0);
			RobotControl->ChasisControl.MotorUnits->Motor[4].MITUpdate(0,0,0,0,RobotControl->ChasisControl.OpenLoopWheelI[0]);
			RobotControl->ChasisControl.MotorUnits->Motor[5].MITUpdate(0,0,0,0,RobotControl->ChasisControl.OpenLoopWheelI[1]);			
		}
		else
		{
			/*Chasis follow gimbal*/
			float WheelMotor[2]={0};
			if(RobotControl->ChasisControl.GetCFGENFlag())
			{
				/*Keep follow*/
				if(RobotControl->ChasisControl.GetCFGSFlag())
				{RobotControl->ChasisControl.LoopCFG.PID_Cal(RobotControl->ChasisControl.GetCFGErr());}
				else
				{
					if(fabs(RobotControl->ChasisControl.GetCFGErr()) < CHASIS_FOLLOW_GIMBAL_RANGE)
					{RobotControl->ChasisControl.LoopCFG.PID_Cal(RobotControl->ChasisControl.GetCFGErr());}
					else
					{RobotControl->ChasisControl.LoopCFG.Reset();}
				}
				
				if(fabs(RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw-RobotControl->ChasisControl.ObserveVal.ChasisPsaiYaw)<CHASIS_FOLLOW_GIMBAL_SAFE_RANGE)
				{RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw += RobotControl->ChasisControl.LoopYaw.GetOut();}
			}
			else
			{RobotControl->ChasisControl.LoopCFG.Reset();}
			/*Chasis Yaw close-loop*/
			RobotControl->ChasisControl.LoopYaw.PID_Cal(RobotControl->ChasisControl.ObserveVal.ChasisPsaiYaw - RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw);
			WheelMotor[0] += RobotControl->ChasisControl.LoopYaw.GetOut();
			WheelMotor[1] -= RobotControl->ChasisControl.LoopYaw.GetOut();
			
			
			/*Leg motors*/
			/*FT = [PendulumForce PendulumTorque]   Torque = [Motor3Torque(backmotor)  Motor2Torque(frontmotor)] */
			float FT_L[2]={0};float TorqueL[2]={0};
			float FT_R[2]={0};float TorqueR[2]={0};
			static float Lentmp[2]={0};
			static uint8_t StartFlag;static uint8_t LastLenChangeFlag;
			if(RobotControl->CheckPartMode(ROBOTPART_ID_BLC)==ROBOTPART_STATUS_START)
			{
				
				
				static float coff[2];static uint16_t addtimes;
				static ULONG TIM;
				if(StartFlag==0)
				{
					StartFlag = 1;LastLenChangeFlag = 1;addtimes  = 0;
					RobotControl->ChasisControl.LoopLen[0].Reset();RobotControl->ChasisControl.LoopLen[1].Reset();
					Lentmp[0] = RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumLen();
					Lentmp[1] = RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumLen();
					
					TIM = tx_time_get();
				}
				else if(StartFlag==1)
				{
					Lentmp[0] = RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumLen();
					Lentmp[1] = RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumLen();
					/*Wait 1s*/
					if(tx_time_get() - TIM>100)
					{
						coff[0]=(RobotControl->ChasisControl.GetLegLen() - RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumLen())/500;
						coff[1]=(RobotControl->ChasisControl.GetLegLen() - RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumLen())/500;
						StartFlag = 2;
					}
				}
				else if(StartFlag==2)
				{
					Lentmp[0]+=coff[0];
					Lentmp[1]+=coff[1];
					if(++addtimes==500)
					{
						StartFlag = 0;
						RobotControl->SetPartMode(ROBOTPART_ID_BLC,ROBOTPART_STATUS_NORMAL);
						RobotControl->ChasisControl.LQR.RefreshLQRK(RobotControl->ChasisControl.GetLegLenFlag());
						RobotControl->ChasisControl.TargetVal.X[2] = RobotControl->ChasisControl.ObserveVal.X[2];
					}
				}
			}
			else
			{
				/*Length close loop*/
				static uint8_t LastLenFlag = RobotControl->ChasisControl.GetLegLenFlag();
				static float coff;static uint16_t addtimes;
				
				if(RobotControl->ChasisControl.GetLegLenFlag()!=LastLenFlag)
				{StartFlag = 0;LastLenChangeFlag = 1;}

				
				if(LastLenChangeFlag==1)
				{
					LastLenChangeFlag=2;addtimes=0;coff=(RobotControl->ChasisControl.GetLegLen()-Lentmp[0])/250;
					RobotControl->ChasisControl.LoopLen[0].Reset();RobotControl->ChasisControl.LoopLen[1].Reset();
				}
				else if(LastLenChangeFlag==2)
				{
					Lentmp[0]+=coff;
					Lentmp[1]+=coff;
					if(++addtimes==250)
					{LastLenChangeFlag=0;RobotControl->ChasisControl.LQR.RefreshLQRK(RobotControl->ChasisControl.GetLegLenFlag());}
				}
				else
				{Lentmp[0] = RobotControl->ChasisControl.GetLegLen();Lentmp[1]=Lentmp[0];}
				
				LastLenFlag = RobotControl->ChasisControl.GetLegLenFlag();
			}
			
			/*Robot roll*/
			RobotControl->ChasisControl.LoopRoll.PID_Cal(QCS.Roll(INS->Q));
			
			RobotControl->ChasisControl.LoopLen[0].SetRef(Lentmp[0] - RobotControl->ChasisControl.LoopRoll.GetOut());
			RobotControl->ChasisControl.LoopLen[1].SetRef(Lentmp[1] + RobotControl->ChasisControl.LoopRoll.GetOut());
			RobotControl->ChasisControl.LoopLen[0].PID_Cal(RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumLen());
			RobotControl->ChasisControl.LoopLen[1].PID_Cal(RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumLen());
			
			
			/*LQR*/
			static float ThetaLast,ThetaLastLP;
			float Tlqr[2]={0};
			
			/*Set observation*/
			RobotControl->ChasisControl.ObserveVal.X[0] = 0.5f*(RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumRadian()+RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumRadian()) + RobotControl->ChasisControl.TargetVal.ChasisPitch - PI_Half ;
			RobotControl->ChasisControl.ObserveVal.X[1] = 500.0f*(RobotControl->ChasisControl.ObserveVal.X[0] - ThetaLast);
			RobotControl->ChasisControl.ObserveVal.X[2] = RobotControl->ChasisControl.MotorUnits->GetDis();
			RobotControl->ChasisControl.ObserveVal.X[3] = RobotControl->ChasisControl.MotorUnits->GetVel();
			RobotControl->ChasisControl.ObserveVal.X[4] = -RobotControl->ChasisControl.ObserveVal.ChasisPitch;
			RobotControl->ChasisControl.ObserveVal.X[5] = INS->Gyro[0];/*This is the really pitch. Different placement should be treated differently*/
			
			
			/*Set reference*/
			RobotControl->ChasisControl.TargetVal.X[0] = 0;
			RobotControl->ChasisControl.TargetVal.X[1] = 0;
			
			/*While moving, process D denamatic*/
			if(RobotControl->ChasisControl.GetForwardVelocity()!=0.0f)
			{RobotControl->ChasisControl.TargetVal.X[2] = DisplacementBTW.BTW2Cal(RobotControl->ChasisControl.ObserveVal.X[2] + 0.002f*RobotControl->ChasisControl.GetForwardVelocity());}
			RobotControl->ChasisControl.TargetVal.X[2] = DisplacementBTW.BTW2Cal(RobotControl->ChasisControl.TargetVal.X[2] + 0.002f*RobotControl->ChasisControl.GetForwardVelocity());
			RobotControl->ChasisControl.TargetVal.X[3] = VelocityBTW.BTW2Cal(RobotControl->ChasisControl.GetForwardVelocity());
			RobotControl->ChasisControl.TargetVal.X[4] = -RobotControl->ChasisControl.TargetVal.ChasisPitch;
			RobotControl->ChasisControl.TargetVal.X[5] = 0;
			
			ThetaLast = RobotControl->ChasisControl.ObserveVal.X[0];
			
			if(RobotControl->CheckPartMode(ROBOTPART_ID_BLC)==ROBOTPART_STATUS_NORMAL)
			{RobotControl->ChasisControl.LQR.LQRCal(Tlqr);}
			
			WheelMotor[0] += Tlqr[0];
			WheelMotor[1] -= Tlqr[0];
			
			/*Theta different control*/
			RobotControl->ChasisControl.LoopTheta.PID_Cal(RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumRadian() - RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumRadian());
			
			/*VMC*/
			/*Force*/
			FT_L[0] = RobotControl->ChasisControl.LoopLen[0].GetOut();
			FT_R[0] = RobotControl->ChasisControl.LoopLen[1].GetOut();
			FT_L[1] = -Tlqr[1] - RobotControl->ChasisControl.LoopTheta.GetOut();
			FT_R[1] = -Tlqr[1] + RobotControl->ChasisControl.LoopTheta.GetOut();
			
			/*Torque*/
			RobotControl->ChasisControl.MotorUnits->LinkSolver[0].VMCCal(FT_L,TorqueL);
			RobotControl->ChasisControl.MotorUnits->LinkSolver[1].VMCCal(FT_R,TorqueR);
			
			/*¼Ù×°»áÐý×ª*/
			
			float zzz[2];
			zzz[0] =  Msg_Remoter->RC_Data->rmt.CH0;
			zzz[1] =  Msg_Remoter->RC_Data->rmt.CH0;
			
			/*Output!*/
			RobotControl->ChasisControl.MotorUnits->Motor[0].MITUpdate(0,0,0,0,-TorqueL[1]);
			RobotControl->ChasisControl.MotorUnits->Motor[1].MITUpdate(0,0,0,0,-TorqueL[0]);
			RobotControl->ChasisControl.MotorUnits->Motor[2].MITUpdate(0,0,0,0,TorqueR[0]);
			RobotControl->ChasisControl.MotorUnits->Motor[3].MITUpdate(0,0,0,0,TorqueR[1]);
			RobotControl->ChasisControl.MotorUnits->Motor[4].MITUpdate(0,0,0,0,WheelMotor[0]+zzz[0]);
			RobotControl->ChasisControl.MotorUnits->Motor[5].MITUpdate(0,0,0,0,WheelMotor[1]+zzz[1]);
			
		}
		
		tx_thread_sleep_until(&timer,2);
	}
}