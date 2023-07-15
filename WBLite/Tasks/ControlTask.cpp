#include "ControlTask.h"
#include "IMUTask.h"

#include "Filter.h"
#include "RemoterTask.h"

cRef refdemo;
extern cRemoter *Msg_Remoter;
extern cMotorUnit *MotorUnit;
cRef *Msg_Refer = &refdemo;

float ForwardSpeed = 0.0f;
float SpeedTarget = 0.0f;
float TargetAngel = 0.0f;
float Lens  = LEGMID;


TX_SEMAPHORE COMSem;

SRAM_SET_DTCM static cRobotControl RobotControlt;
cRobotControl *RobotControl = 0;

TX_THREAD RoboCTRThread;
uint8_t RoboCTRThreadStack[2048]={0};

void RoboCTRThreadFun(ULONG initial_input)
{
	RobotControl = &RobotControlt;
	RobotControl->ChasisControl.MotorUnits = MotorUnit;
	
	tx_thread_sleep(100);

	/*Input refer system data point*/
	RobotControl->SetRefSysDataSource(&Msg_Refer->robot_referee_status->ext_game_robot_status,&Msg_Refer->robot_referee_status->ext_power_heat_data);
	
	/*Used to determind RobotMode change*/
	uint8_t LastKey_SW1=Msg_Remoter->RC_Data->rmt.SW1;
	
	/*This value is determind by cool rate*/
	uint8_t RobotPart_TroggleMMode = ROBOTPART_TRIGGLE_MMODE0;
	/*This value is determind by ammo speed limit*/
	uint8_t RobotPart_BoosterMode  = ROBOTPART_BOOSTER_15MPS;
	
	ULONG timer = tx_time_get();
	for(;;)
	{
		tx_thread_sleep_until(&timer,5);
		timer = tx_time_get();
		/*Check if remoter offline*/
		if(Msg_Remoter->IsRCOffline)
		{
			/*if remote offline, RobotMode and all part will get into IDLE*/
			RobotControl->SetRobotMode(ROBOTMODE_IDLE);
			
			RobotControl->SetPartMode(ROBOTPART_ID_LEG,ROBOTPART_STATUS_IDLE);
			RobotControl->SetPartMode(ROBOTPART_ID_BLC,ROBOTPART_STATUS_IDLE);
			RobotControl->SetPartMode(ROBOTPART_ID_BST,ROBOTPART_STATUS_IDLE);
			RobotControl->SetPartMode(ROBOTPART_ID_GIM,ROBOTPART_STATUS_IDLE);
		}
		
		/*Error Detection*/
		
		
		/*Refresh RobotDeath*/
		if(!RobotControl->GetRobotHP())
		{RobotControl->ComDown.RobotDeath = 1;}
		else
		{RobotControl->ComDown.RobotDeath = 0;}
		
/*********************************************************************************/
		/*RobotMode Change*/
		/*Press Z to emergancy turn to RobotIDLE*/
		uint8_t IsModeChangeValid = 0;//To show is mode change happened.
		if(	(LastKey_SW1!=Msg_Remoter->RC_Data->rmt.SW1)||(Msg_Remoter->RC_Data->key.Z==1) )
		{
			/*SW1->1, robot in SafeMode*/
			if( (Msg_Remoter->RC_Data->rmt.SW1==1)||(Msg_Remoter->RC_Data->key.Z==1) )
			{
				RobotControl->SetRobotMode(ROBOTMODE_IDLE);
				/*
					机器人空闲模式
				
					Leg:		IDLE
					Blance:		IDLE
					Gimbal:		IDLE
					Booster:	IDLE
				*/
				/*检查状态,设置为IDLE,异常状态将保留*/
				RobotControl->SetPartMode(ROBOTPART_ID_LEG,ROBOTPART_STATUS_IDLE);
				RobotControl->SetPartMode(ROBOTPART_ID_BLC,ROBOTPART_STATUS_IDLE);
				RobotControl->SetPartMode(ROBOTPART_ID_BST,ROBOTPART_STATUS_IDLE);
				RobotControl->SetPartMode(ROBOTPART_ID_GIM,ROBOTPART_STATUS_IDLE);	
			}
			
			/*逃脱模式*/
			else if(Msg_Remoter->RC_Data->rmt.SW1==2)
			{
				RobotControl->SetRobotMode(ROBOTMODE_ESCAPE);
				/*
					机器人逃脱模式
				
					Leg:		IDLE
					Blance:		IDLE
					Gimbal:		NORMAL
					Booster:	IDLE
				*/
				/*检查状态*/
				RobotControl->SetPartMode(ROBOTPART_ID_LEG,ROBOTPART_STATUS_IDLE);
				RobotControl->SetPartMode(ROBOTPART_ID_BLC,ROBOTPART_STATUS_IDLE);
				
				if(RobotControl->CheckPartMode(ROBOTPART_ID_BST)!=ROBOTPART_STATUS_NORMAL)
				{RobotControl->SetPartMode(ROBOTPART_ID_BST,ROBOTPART_STATUS_START);}
				if(RobotControl->CheckPartMode(ROBOTPART_ID_GIM)!=ROBOTPART_STATUS_NORMAL)
				{RobotControl->SetPartMode(ROBOTPART_ID_GIM,ROBOTPART_STATUS_START);}
				
			}
			/*正常模式*/
			else if(Msg_Remoter->RC_Data->rmt.SW1==3)
			{	
				RobotControl->SetRobotMode(ROBOTMODE_NORMAL);
				/*
					机器人正常模式
				
					Leg:		NORMAL 需要启动模式
					Blance:		NORMAL 需要启动模式
					Gimbal:		NORMAL
					Booster:	NORMAL
				*/
				
				/*检查状态*/
				if((RobotControl->CheckPartMode(ROBOTPART_ID_LEG)&0x03)!= ROBOTPART_STATUS_NORMAL)
				{RobotControl->SetPartMode(ROBOTPART_ID_LEG,ROBOTPART_STATUS_START);}
				if((RobotControl->CheckPartMode(ROBOTPART_ID_BLC)&0x03)!= ROBOTPART_STATUS_NORMAL)
				{RobotControl->SetPartMode(ROBOTPART_ID_BLC,ROBOTPART_STATUS_START);}
				if((RobotControl->CheckPartMode(ROBOTPART_ID_BST)&0x03)!= ROBOTPART_STATUS_NORMAL)
				{RobotControl->SetPartMode(ROBOTPART_ID_BST,ROBOTPART_STATUS_START);}
				if((RobotControl->CheckPartMode(ROBOTPART_ID_GIM)&0x03)!= ROBOTPART_STATUS_NORMAL)
				{RobotControl->SetPartMode(ROBOTPART_ID_GIM,ROBOTPART_STATUS_START);}
			
				
			}
			
			IsModeChangeValid = 1;
			/*Refresh*/
			LastKey_SW1 = Msg_Remoter->RC_Data->rmt.SW1;
		}
			
/*********************************************************************************/
		/*Robot input activity*/	
		
		/*Chasis activity*/
		static uint8_t Lastkey_CV;
		static uint8_t LastKey_Wheel_Zero;
		switch(RobotControl->CheckRobotMode()&0x03)
		{	
			case ROBOTMODE_IDLE:
			{
				/*Check if Chasis error happened*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_BLC)&0x04)
				{
				
				}
					
				/*Reset open-loop value*/
				RobotControl->ChasisControl.OpenLoopWheelI[0] = 0.0f;
				RobotControl->ChasisControl.OpenLoopWheelI[1] = 0.0f;
				
				/*Yaw security*/
				RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw = RobotControl->ChasisControl.ObserveVal.ChasisPsaiYaw;
				
				/*Displacement setting to observated value*/
				RobotControl->ChasisControl.TargetVal.X[2] = RobotControl->ChasisControl.ObserveVal.X[2];
				/*Velocity should be set to zero*/
				RobotControl->ChasisControl.TargetVal.X[4] = 0.0f;
				break;
			}
			case ROBOTMODE_ESCAPE:
			{
				/*Yaw security*/
				RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw = RobotControl->ChasisControl.ObserveVal.ChasisPsaiYaw;
				
				/*前后控制*/
				/*This if...else... is designed to make sure remoter has higher priority than keyboard*/
				if(Msg_Remoter->RC_Data->rmt.CH3!=0.0f)
				{	/*遥控器直接赋值即可*/
					RobotControl->ChasisControl.OpenLoopWheelI[0] =  Msg_Remoter->RC_Data->rmt.CH3*3;
					RobotControl->ChasisControl.OpenLoopWheelI[1] = -Msg_Remoter->RC_Data->rmt.CH3*3;	
				}
				else//鼠标键盘权限
				{	/*鼠键做下积分*/
					static int16_t Tem_KeyV = 0;
					Tem_KeyV = (Msg_Remoter->RC_Data->key.W||Msg_Remoter->RC_Data->key.S) ? Tem_KeyV+3*(Msg_Remoter->RC_Data->key.W - Msg_Remoter->RC_Data->key.S) : 0;
					if(Tem_KeyV>ESCAPE_MAXI_FORWARD){Tem_KeyV = ESCAPE_MAXI_FORWARD;}
					else if(Tem_KeyV<-ESCAPE_MAXI_FORWARD){Tem_KeyV = -ESCAPE_MAXI_FORWARD;}
					
					RobotControl->ChasisControl.OpenLoopWheelI[0] =  Tem_KeyV;
					RobotControl->ChasisControl.OpenLoopWheelI[1] = -Tem_KeyV;
				}
				/*旋转控制*/
				if(Msg_Remoter->RC_Data->rmt.CH2!=0.0f)
				{
					RobotControl->ChasisControl.OpenLoopWheelI[0] = Msg_Remoter->RC_Data->rmt.CH2*3;
					RobotControl->ChasisControl.OpenLoopWheelI[1] = Msg_Remoter->RC_Data->rmt.CH2*3;
				}
				else
				{
					/*鼠键做下积分*/
					static int16_t Tem_KeyV = 0;
					Tem_KeyV = (Msg_Remoter->RC_Data->key.A||Msg_Remoter->RC_Data->key.D) ? Tem_KeyV+1*(Msg_Remoter->RC_Data->key.A - Msg_Remoter->RC_Data->key.D) : 0;
					if(Tem_KeyV>ESCAPE_MAXI_ROLL){Tem_KeyV = ESCAPE_MAXI_ROLL;}
					else if(Tem_KeyV<-ESCAPE_MAXI_ROLL){Tem_KeyV = -ESCAPE_MAXI_ROLL;}
					
					RobotControl->ChasisControl.OpenLoopWheelI[0] +=  Tem_KeyV;
					RobotControl->ChasisControl.OpenLoopWheelI[1] +=  Tem_KeyV;
				}				
				break;
			}
			case ROBOTMODE_NORMAL:
			{	
				/*Check if Chasis error happened*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_BLC)&0x04)
				{

				}
				
				/*Get into Chasis StartMode*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_BLC)==ROBOTPART_STATUS_START)
				{
					/*Wait until banlance is OK*/
					if(RobotControl->ChasisControl.GetBalanceFlag())
					{RobotControl->SetPartMode(ROBOTPART_ID_BLC,ROBOTPART_STATUS_NORMAL);}
				}
				else
				{
					/*SHUTTLE MODE*/
					if(RobotControl->ChasisControl.GetChasisMode()==ROBOTPART_CHASIS_SHUTTLE)
					{
						/*Mode Change Check*/
						if(fabs(Msg_Remoter->RC_Data->rmt.CH2==1.0f)||Msg_Remoter->RC_Data->key.Q)
						{
							RobotControl->ChasisControl.SetChasisMode(ROBOTPART_CHASIS_SIDEMODE);
							RobotControl->ChasisControl.ChasisForwardTargetVelocity = 0;
							RobotControl->ChasisControl.RefreshChasisHead();
							RobotControl->ChasisControl.SetCFGENStatue(1);
							RobotControl->ChasisControl.SetCFGSStatue(1);
							break;
						}

						/*Click E to enable chasis follow gimbal and keep chasis coincidiently follow gimbal*/
						if(Msg_Remoter->RC_Data->key.E)
						{RobotControl->ChasisControl.SetCFGENStatue(1);}
						RobotControl->ChasisControl.SetCFGSStatue(Msg_Remoter->RC_Data->key.E);
						
						/*Leg length control*/
						if(Msg_Remoter->RC_Data->rmt.SW2==1)/*Booster safty mode can do*/
						{
							if((fabs(Msg_Remoter->RC_Data->rmt.WHEEL)==1.0f)&&LastKey_Wheel_Zero)
							{
								Lastkey_CV = 1;LastKey_Wheel_Zero=0;
								int8_t tmp = RobotControl->ChasisControl.GetLegLenFlag() + ((Msg_Remoter->RC_Data->rmt.WHEEL==1.0f)?1:-1);
								if(tmp<LQR_BOTTOM){tmp = LQR_BOTTOM;}
								else if(tmp>LQR_TOP){tmp=LQR_TOP;}

								RobotControl->ChasisControl.SetLegLen((eRobotLQRID)tmp);
							}

						}
						if((Msg_Remoter->RC_Data->key.C||Msg_Remoter->RC_Data->key.V) && (!Lastkey_CV))
						{
							Lastkey_CV = 1;
							int8_t tmp = RobotControl->ChasisControl.GetLegLenFlag() + ((Msg_Remoter->RC_Data->key.V - Msg_Remoter->RC_Data->key.C));
							if(tmp<LQR_BOTTOM){tmp = LQR_BOTTOM;}
							else if(tmp>LQR_TOP){tmp=LQR_TOP;}
							
							RobotControl->ChasisControl.SetLegLen((eRobotLQRID)tmp);
						}
						else
						{Lastkey_CV = 0;}
						
						/*前后控制*/
						/*This if...else... is designed to make sure remoter has higher priority than keyboard*/
						if(Msg_Remoter->RC_Data->rmt.CH3!=0.0f)
						{
							if(!RobotControl->ChasisControl.GetCFGENFlag())
							{
								/*CFCEN == 0, we should check head*/
								RobotControl->ChasisControl.RefreshChasisHead();
								/*Enable chasis follow gimbal*/
								RobotControl->ChasisControl.SetCFGENStatue(1);
								RobotControl->ChasisControl.SetCFGSStatue(1);
							}

							RobotControl->ChasisControl.ChasisForwardTargetVelocity =  Msg_Remoter->RC_Data->rmt.CH3*CHASIS_SPEED_MAX;
						}/*Close-loop control with fix this*/
						else//鼠标键盘权限
						{	
							
							/*CFCEN == 0, we should check head*/
							if( (!RobotControl->ChasisControl.GetCFGENFlag()) && (Msg_Remoter->RC_Data->key.W||Msg_Remoter->RC_Data->key.S))
							{
								/*Check head*/
								RobotControl->ChasisControl.RefreshChasisHead();
								/*Enable chasis follow gimbal*/
								RobotControl->ChasisControl.SetCFGENStatue(1);
								RobotControl->ChasisControl.SetCFGSStatue(1);
							}
							
							/*鼠键做下积分*/
							static float keyval = 0;
							keyval = (Msg_Remoter->RC_Data->key.W||Msg_Remoter->RC_Data->key.S) ? keyval+CHASIS_SPEED_ACCEL*(Msg_Remoter->RC_Data->key.W - Msg_Remoter->RC_Data->key.S) : 0;
							if(keyval>CHASIS_SPEED_MAX){keyval = CHASIS_SPEED_MAX;}
							else if(keyval<-CHASIS_SPEED_MAX){keyval = -CHASIS_SPEED_MAX;}
							
							RobotControl->ChasisControl.ChasisForwardTargetVelocity =  keyval;
						}
						
						/*没有前进后退发生时允许进行旋转*/
						if(RobotControl->ChasisControl.ChasisForwardTargetVelocity==0.0f)
						{
						
							/*Directly add to ref psai*/
							if(Msg_Remoter->RC_Data->rmt.CH2!=0.0f)
							{
								/*Disbale Chasis follow gimbal*/
								RobotControl->ChasisControl.SetCFGENStatue(0);
							
								RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw -= CHASIS_ANGYLAR_VELOCITY_MAX*Msg_Remoter->RC_Data->rmt.CH2;							
							}
							else
							{
								if(Msg_Remoter->RC_Data->key.A||Msg_Remoter->RC_Data->key.D)
								{
									/*Disbale Chasis follow gimbal*/
									RobotControl->ChasisControl.SetCFGENStatue(0);
								}

								
								/*鼠键做下积分*/
								static int16_t keyval = 0;
								keyval = (Msg_Remoter->RC_Data->key.A||Msg_Remoter->RC_Data->key.D) ? keyval+CHASIS_ANGYLAR_VELOCITY_MAX*(Msg_Remoter->RC_Data->key.A - Msg_Remoter->RC_Data->key.D) : 0;
								
								RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw += keyval;
							}		
							
						}	
					}
					/*SIDE MODE*/
					else if(RobotControl->ChasisControl.GetChasisMode()==ROBOTPART_CHASIS_SIDEMODE)
					{
						/*Mode Change Check*/
						if(Msg_Remoter->RC_Data->key.A||Msg_Remoter->RC_Data->key.D||Msg_Remoter->RC_Data->key.E||(fabs(Msg_Remoter->RC_Data->rmt.CH3)>0.2f))
						{
							RobotControl->ChasisControl.SetChasisMode(ROBOTPART_CHASIS_SHUTTLE);
							RobotControl->ChasisControl.RefreshChasisHead();
							RobotControl->ChasisControl.ChasisForwardTargetVelocity = 0;
							break;
						}
						
						/*Leg length control*/
						if(Msg_Remoter->RC_Data->rmt.SW2==1)/*Booster safty mode can do*/
						{
							if((fabs(Msg_Remoter->RC_Data->rmt.WHEEL)==1.0f)&&LastKey_Wheel_Zero)
							{
								Lastkey_CV = 1;LastKey_Wheel_Zero=0;
								int8_t tmp = RobotControl->ChasisControl.GetLegLenFlag() + ((Msg_Remoter->RC_Data->rmt.WHEEL==1.0f)?1:-1);
								if(tmp<LQR_BOTTOM){tmp = LQR_BOTTOM;}
								else if(tmp>LQR_TOP){tmp=LQR_TOP;}

								RobotControl->ChasisControl.SetLegLen((eRobotLQRID)tmp);
							}
						}
						if((Msg_Remoter->RC_Data->key.C||Msg_Remoter->RC_Data->key.V) && (!Lastkey_CV))
						{
							Lastkey_CV = 1;
							int8_t tmp = RobotControl->ChasisControl.GetLegLenFlag() + ((Msg_Remoter->RC_Data->key.V - Msg_Remoter->RC_Data->key.C));
							if(tmp<LQR_BOTTOM){tmp = LQR_BOTTOM;}
							else if(tmp>LQR_TOP){tmp=LQR_TOP;}
							
							RobotControl->ChasisControl.SetLegLen((eRobotLQRID)tmp);
						}
						else
						{Lastkey_CV = 0;}
						
						/*左右控制*/
						/*This if...else... is designed to make sure remoter has higher priority than keyboard*/
						if(Msg_Remoter->RC_Data->rmt.CH2!=0.0f)
						{
							RobotControl->ChasisControl.ChasisForwardTargetVelocity =  Msg_Remoter->RC_Data->rmt.CH2*CHASIS_SPEED_SIDE_MAX;
						}/*Close-loop control with fix this*/
						else//鼠标键盘权限
						{	
							/*鼠键做下积分*/
							static float keyval = 0;
							keyval = (Msg_Remoter->RC_Data->key.A||Msg_Remoter->RC_Data->key.D) ? keyval+CHASIS_SPEED_SIDE_ACCEL*(Msg_Remoter->RC_Data->key.A - Msg_Remoter->RC_Data->key.D) : 0;
							if(keyval>CHASIS_SPEED_ACCEL){keyval = CHASIS_SPEED_ACCEL;}
							else if(keyval<-CHASIS_SPEED_ACCEL){keyval = -CHASIS_SPEED_ACCEL;}
							
							RobotControl->ChasisControl.ChasisForwardTargetVelocity =  keyval;
						}
					}
				}
				break;
			}
		}
		if(fabs(Msg_Remoter->RC_Data->rmt.WHEEL)<0.1f)
		{LastKey_Wheel_Zero = 1;}
			
		/*Gimbal activity*/
		switch(RobotControl->CheckRobotMode()&0x03)
		{
			case ROBOTMODE_IDLE:
				RobotControl->ComDown.YawInc		= 0;
				RobotControl->ComDown.PitInc		= 0;
				RobotControl->ComDown.GimDeathFlag	= 1;
				
				/*Error in IDLE mode*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_GIM)&0x04)
				{
				
				}
			break;
			
			/*Gimbal has same activity when robot be in Escape and Normal mode*/
			case ROBOTMODE_ESCAPE:
			case ROBOTMODE_NORMAL:	
				/*Check if Gimbal error happened*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_GIM)&0x04)
				{
					/*Change Normal into Start*/
					if(RobotControl->CheckPartMode(ROBOTPART_ID_GIM)==ROBOTPART_STATUS_NORMAL)
					{RobotControl->SetPartMode(ROBOTPART_ID_GIM,ROBOTPART_STATUS_START);}
					
					/*Set GimDeathFlag*/
					RobotControl->ComDown.GimDeathFlag = 1;
				}
				
				/*Get into Gimbal StartMode*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_GIM)==ROBOTPART_STATUS_START)
				{
					/*Return to normal mode. Robot is alive and no Gimbal error*/
					if( RobotControl->GetRobotHP() && !(RobotControl->CheckPartMode(ROBOTPART_ID_GIM)&0x04)  )
					{
						RobotControl->SetPartMode(ROBOTPART_ID_GIM,ROBOTPART_STATUS_NORMAL);
						RobotControl->ComDown.GimDeathFlag = 0;
					}
				}
				/*Get into Gimbal NormalMode*/
				else if(RobotControl->CheckPartMode(ROBOTPART_ID_GIM)==ROBOTPART_STATUS_NORMAL)
				{	
					/*Gimbal normal activity here*/					
					/*Yaw and Pithc radian increase*/
					RobotControl->ComDown.PitInc =	-GIMBAL_PITCH_ANGYLAR_VELOCITY	* (Msg_Remoter->RC_Data->rmt.CH1 + (float)(Msg_Remoter->RC_Data->mouse.x)/32768.0f );//Pitch need to reverse
					RobotControl->ComDown.YawInc =	-GIMBAL_YAW_ANGYLAR_VELOCITY 	* (Msg_Remoter->RC_Data->rmt.CH0 + (float)(Msg_Remoter->RC_Data->mouse.y)/32768.0f );//Pitch need to reverse
				}		
		}
		
		/*Booster activity*/
		switch(RobotControl->CheckRobotMode()&0x03)
		{
			case ROBOTMODE_IDLE:
				RobotControl->ComDown.Fire			= 0;
				RobotControl->ComDown.BoosterMode	= ROBOTPART_BOOSTER_STOP;
				RobotControl->ComDown.TriggleMode	= ROBOTPART_TRIGGLE_STOP;
			
				/*Error in IDLE mode*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_BST)&0x04)
				{
				
				}
			break;
			
			case ROBOTMODE_ESCAPE:
			case ROBOTMODE_NORMAL:
				
				/*Check if Booster error happened*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_BST)&0x04)
				{
					/*Jam is not a very important error*/
				
				}
				
				/*Update real-time paramaters*/
				
				/*Update Troggle mode*/
				if(RobotControl->GetRobotCoolRate()<ROBOTPART_TROOGLEMODE_VALUE)
				{RobotPart_TroggleMMode = ROBOTPART_TRIGGLE_MMODE0;}
				else
				{RobotPart_TroggleMMode = ROBOTPART_TRIGGLE_MMODE1;}
				
				/*Update Booster Velocity*/
				switch(RobotControl->GetRobotAmmoSpeed())
				{
					case 15:
						RobotPart_BoosterMode = ROBOTPART_BOOSTER_15MPS;
					break;
					case 18:
						RobotPart_BoosterMode = ROBOTPART_BOOSTER_18MPS;
					break;
					case 30:
						RobotPart_BoosterMode = ROBOTPART_BOOSTER_30MPS;
					break;
					default:
						RobotPart_BoosterMode = ROBOTPART_BOOSTER_15MPS;
					break;
				}
				
				/*Control Cap*/
				static uint8_t LastKey_R;
				static float LastKey_Wheel;
				/*KeyBoard Control*/
				if( (Msg_Remoter->RC_Data->key.R)&&(!LastKey_R) )
				{RobotControl->ComDown.CapOpen = RobotControl->ComDown.CapOpen?0:1;}
				/*Remoter control. Push Timewise*/
				else if((Msg_Remoter->RC_Data->rmt.WHEEL<-0.5f)&&(LastKey_Wheel>-0.5f))
				{RobotControl->ComDown.CapOpen = RobotControl->ComDown.CapOpen?0:1;}					
				/*Close Cap when mode change*/
				if(IsModeChangeValid)
				{RobotControl->ComDown.CapOpen = 0;}	
				
				LastKey_R = Msg_Remoter->RC_Data->key.R;
				LastKey_Wheel = Msg_Remoter->RC_Data->rmt.WHEEL;
				
				/*Get into Ammobooster StartMode*/
				if(RobotControl->CheckPartMode(ROBOTPART_ID_BST)==ROBOTPART_STATUS_START)
				{
					/*Return to normal mode. Robot is alive and no Booster error*/
					if( RobotControl->GetRobotHP() && !(RobotControl->CheckPartMode(ROBOTPART_ID_BST)&0x04)  )
					{
						RobotControl->ComDown.TriggleMode	= ROBOTPART_TRIGGLE_SINGLE;
						RobotControl->SetPartMode(ROBOTPART_ID_BST,ROBOTPART_STATUS_NORMAL);
					}
					else
					{
						RobotControl->ComDown.Fire			= 0;
						RobotControl->ComDown.BoosterMode	= ROBOTPART_BOOSTER_STOP;
					}
				}	
				/*Get into Ammobooster NormalMode*/
				else if(RobotControl->CheckPartMode(ROBOTPART_ID_BST)==ROBOTPART_STATUS_NORMAL)
				{	
					/*Boooster normal activity here*/
					
					static uint8_t LastKey_B = 0;
					static uint8_t LastKey_SW2 = 1;
					
					/* When SW2->1, safy lock enable, robot booster disable and can not open fire. */
					if(Msg_Remoter->RC_Data->rmt.SW2==1)
					{
						RobotControl->ComDown.Fire			= 0;
						RobotControl->ComDown.BoosterMode	= ROBOTPART_BOOSTER_STOP;
					}
					else
					{	
						/*Singel-Shot or Multi-Shot mode change*/
						/*switch change*/
						if(Msg_Remoter->RC_Data->rmt.SW2!=LastKey_B)
						{
							RobotControl->ComDown.TriggleMode = (Msg_Remoter->RC_Data->rmt.SW2==2)?ROBOTPART_TRIGGLE_SINGLE:RobotPart_TroggleMMode;
						}
						/*keyboard change*/
						if( (Msg_Remoter->RC_Data->key.B) && (!LastKey_B) )
						{
							RobotControl->ComDown.TriggleMode = (RobotControl->ComDown.TriggleMode	== RobotPart_TroggleMMode)?ROBOTPART_TRIGGLE_SINGLE:RobotPart_TroggleMMode;
						}
						
						/*Update BoosterMode*/
						RobotControl->ComDown.BoosterMode	=	RobotPart_BoosterMode;
						
						/*Fire Flag Set*/
						/*Check is threr enough heat remain*/					
						if(RobotControl->GetRobotRemainHeat()<ROBOTPART_HEATSAFT_VALUE)
						{RobotControl->ComDown.Fire = 0;}
						else
						{
							/*OpenFire Control. Singel-Shot or Multi-Shot should be processed in gimbal-control-board*/						
							/*Mouse control*/
							RobotControl->ComDown.Fire = Msg_Remoter->RC_Data->mouse.press_l;
							/*Remoter control. Push Anti-Timewise*/
							if(Msg_Remoter->RC_Data->rmt.WHEEL>0.5f)
							{RobotControl->ComDown.Fire = 1;}								
						}
					}
					LastKey_B = Msg_Remoter->RC_Data->key.B;
					LastKey_SW2 = Msg_Remoter->RC_Data->rmt.SW2;
				}

				/*Send CAN message*/
				//HAL_FDCAN_AddMessageToTxFifoQ();
				
		}		
	}
}