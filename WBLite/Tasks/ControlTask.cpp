#include "ControlTask.h"
#include "IMUTask.h"

#include "Filter.h"
#include "RemoterTask.h"

cRef refdemo;
extern cRemoter *Msg_Remoter;
cRef *Msg_Refer = &refdemo;

float ForwardSpeed = 0.0f;
float SpeedTarget = 0.0f;
float TargetAngel = 0.0f;
float Lens  = LEGMID;


TX_SEMAPHORE COMSem;
cRobotControl *RobotControl = 0;

TX_THREAD RoboCTRThread;
uint8_t RoboCTRThreadStack[2048]={0};

void RoboCTRThreadFun(ULONG initial_input)
{
	cRobotControl RobotControlt;
	RobotControl = &RobotControlt;
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
				
				/*Displacement setting to observated value*/
				RobotControl->ChasisControl.TargetVal.X[2] = RobotControl->ChasisControl.ObserveVal.X[2];
				/*Velocity should be set to zero*/
				RobotControl->ChasisControl.TargetVal.X[4] = 0.0f;
				break;
			}
			case ROBOTMODE_ESCAPE:
			{
				
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
							RobotControl->ChasisControl.ChasisLeftwardTargetVelocity = 0;
							break;
						}
						
						/*左右控制*/
						/*This if...else... is designed to make sure remoter has higher priority than keyboard*/
						if(Msg_Remoter->RC_Data->rmt.CH2!=0.0f)
						{
							RobotControl->ChasisControl.ChasisLeftwardTargetVelocity =  Msg_Remoter->RC_Data->rmt.CH2*CHASIS_SPEED_SIDE_MAX;
						}/*Close-loop control with fix this*/
						else//鼠标键盘权限
						{	
							/*鼠键做下积分*/
							static float keyval = 0;
							keyval = (Msg_Remoter->RC_Data->key.A||Msg_Remoter->RC_Data->key.D) ? keyval+CHASIS_SPEED_SIDE_ACCEL*(Msg_Remoter->RC_Data->key.A - Msg_Remoter->RC_Data->key.D) : 0;
							if(keyval>CHASIS_SPEED_ACCEL){keyval = CHASIS_SPEED_ACCEL;}
							else if(keyval<-CHASIS_SPEED_ACCEL){keyval = -CHASIS_SPEED_ACCEL;}
							
							RobotControl->ChasisControl.ChasisLeftwardTargetVelocity =  keyval;
						}
					}
				}
				break;
			}
		}
		
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


//TX_THREAD Loop9025Thread;
//uint8_t   Loop9025ThreadStack[1280]={0};

//cLoop9025Stand	*Loop9025Stand;
//cLoop9025Yaw	*Loop9025Yaw;

//extern cINS *INS;
//float MultiY = 0.0f;

//extern TX_BYTE_POOL MotorPool;

//void Loop9025ThreadFun(ULONG initial_input)
//{

//	Loop9025Stand = new cLoop9025Stand;
//	Loop9025Yaw  = new cLoop9025Yaw;
//	
//	Loop9025Stand->SetRef(0.0f);
//	Loop9025Yaw->SetRef(0.0f);
//	
//	while(!INS){tx_thread_sleep(50);}
//	while(INS->QChasis[0]==1.0f){tx_thread_sleep(50);}
//	
//	static float LastY;
////	static float MultiY;
//	tx_thread_sleep(1000);
//	ULONG timer = 0;
//	for(;;)
//	{
//		timer = tx_time_get();
//		float angel_t =  QCS.Yaw(INS->QChasis) - LastY;
//		angel_t = (angel_t>PI) ? angel_t-2*PI :angel_t;
//		angel_t = (angel_t<-PI)? angel_t+2*PI :angel_t;
//		MultiY += angel_t;
//		LastY	= QCS.Yaw(INS->QChasis);
//		
//		if(Msg_Remoter->RC_Data->rc.s1==2 &&! Msg_Remoter->IsRCOffline)
//		{
//			Loop9025Yaw->SetRef(TargetAngel);
//			
//			Loop9025Stand->PID_Cal(QCS.Pitch(INS->QChasis));
//			Loop9025Yaw->PID_Cal(MultiY);
//			

//			float WheelTorI[2] ={
//				Loop9025Stand->GetOut() + Loop9025Yaw->GetOut() + ForwardSpeed + Loop9025Stand->Mcorrect,
//				Loop9025Stand->GetOut() - Loop9025Yaw->GetOut() + ForwardSpeed + Loop9025Stand->Mcorrect};
//			
//				
//			/*动态调整最大值以保持运动趋势*/
//			int16_t maxval = (abs(WheelTorI[0])>abs(WheelTorI[1])) ? abs(WheelTorI[0]) : abs(WheelTorI[1]);
//			if(maxval>3000)
//			{
//				float calibK = 3000.0f/maxval;
//				WheelTorI[0]*=calibK;
//				WheelTorI[1]*=calibK;
//			}
//			
//			KF9025L->Params->iqcontrol =  -WheelTorI[0];
//			KF9025R->Params->iqcontrol =   WheelTorI[1];
//	
//		}
//		else
//		{
//			tx_thread_sleep(100);
//			Loop9025Stand->Reset();
//			Loop9025Yaw->Reset();
//			TargetAngel = MultiY;
//		}
//		tx_thread_sleep_until(&timer,2);
//	}
//}




//TX_THREAD Loop8016IThread;
//uint8_t   Loop8016IThreadStack[4096]={0};
//cLinkSolver *LinkL = 0;
//cLinkSolver *LinkR = 0;

//cFilterBTW2_5Hz *FliWheelSpeed = 0;
//cFilterBTW2_5Hz *FliSpeedIpt = 0;

//cVMC_Rad *VMC_Rad_L=0;
//cVMC_Len *VMC_Len_L=0;
//cVMC_Rad *VMC_Rad_R=0;
//cVMC_Len *VMC_Len_R=0;

//cVMC_Roll *VMC_Roll = 0;
//cVMC_Speed*VMC_Speed= 0;


//void Loop8016IThreadFun(ULONG initial_input)
//{
//	cLinkSolver tLinkL;
//	cLinkSolver tLinkR;
//	cFilterBTW2_5Hz tBTW2W;
//	cFilterBTW2_5Hz tBTW2I;
//	
//	LinkL = &tLinkL;
//	LinkR = &tLinkR;
//	FliWheelSpeed = &tBTW2W;
//	FliSpeedIpt = &tBTW2I;
//	
//	cVMC_Rad tVMC_Rad_L;
//	cVMC_Len tVMC_Len_L;
//	cVMC_Rad tVMC_Rad_R;
//	cVMC_Len tVMC_Len_R;
//	cVMC_Roll	tVMC_Roll;
//	cVMC_Speed	tVMC_Speed;
//	
//	
//	VMC_Rad_L = &tVMC_Rad_L; 
//	VMC_Len_L = &tVMC_Len_L;
//	VMC_Rad_R = &tVMC_Rad_R; 
//	VMC_Len_R = &tVMC_Len_R;
//	VMC_Roll  = &tVMC_Roll;
//	VMC_Speed = &tVMC_Speed;
//	
//	VMC_Rad_L->SetRef(PI/2);
//	VMC_Len_L->SetRef(LEGMID);
//	VMC_Rad_R->SetRef(PI/2);
//	VMC_Len_R->SetRef(LEGMID);
//	VMC_Roll ->SetRef(0);
//	VMC_Speed->SetRef(0);
//	
//	
//	uint8_t StartMode = 0;
//	float FSpeed_t = 0;
//	float FSpeed = 0;
//	
//	
//	tx_thread_sleep(1000);
//	ULONG timer = 0;
//	for(;;)
//	{
//		timer = tx_time_get();
//		LinkL->InputLink(MG8016L3->GetRadian(),MG8016L2->GetRadian());
//		LinkL->Resolve();

//		LinkR->InputLink(MG8016R3->GetRadian(),MG8016R2->GetRadian());
//		LinkR->Resolve();
//		
//		FSpeed_t = 0.5f*(KF9025L->GetSpeed() - KF9025R->GetSpeed());
//		FSpeed = FliWheelSpeed->BTW2Cal(FSpeed_t);
//		

//		
//		if(Msg_Remoter->RC_Data->rc.s1==2 &&! Msg_Remoter->IsRCOffline)
//		{
//			if(StartMode)//起摆
//			{
//				Loop9025Stand->Mcorrect = 300;
//				tx_thread_sleep(1000);
//				Loop9025Stand->Mcorrect = 0;
//				StartMode=0;
//			}
//			else
//			{
//				VMC_Roll ->SetRef(0);
//				VMC_Roll ->PID_Cal(QCS.Roll(INS->QChasis));
//				
//				VMC_Speed->SetRef(FliSpeedIpt->BTW2Cal(SpeedTarget));
//				VMC_Speed->PID_Cal(FSpeed);
//				
//				VMC_Rad_L->SetRef(VMC_Speed->GetOut());
//				VMC_Rad_R->SetRef(VMC_Speed->GetOut());
//				
//				VMC_Len_L->SetRef(
//									Lens/arm_sin_f32(VMC_Speed->GetOut())\
//									+VMC_Roll->GetOut());
//				VMC_Len_R->SetRef(
//									Lens/arm_sin_f32(VMC_Speed->GetOut())\
//									-VMC_Roll->GetOut());
//				
//				
//				VMC_Len_L->PID_Cal(LinkL->GetPendulumLen());
//				VMC_Rad_L->PID_Cal(LinkL->GetPendulumRadian());
//				
//				                                                                                                                                                                                                                                                                                                                                                                                                                                                      
//				float FL[2]={VMC_Len_L->GetOut(),-VMC_Rad_L->GetOut()};
//				float TL[2];
//				LinkL->VMCCal(FL,TL);

//	//			/*电机保护*/
//				
//				/*-20*/
//				if(MG8016L2->GetRadian()>3.054326f)
//				{MG8016L2->Params->iqcontrol = (TL[1]<-PWR_LIMIT) ? -PWR_LIMIT : TL[1];}
//				else
//				{MG8016L2->Params->iqcontrol = TL[1];}
//				

//				if((MG8016L3->GetRadian()>PI)||(MG8016L3->GetRadian()<0.1047197f))
//				{MG8016L3->Params->iqcontrol = (TL[0]> PWR_LIMIT) ? PWR_LIMIT : TL[0];}
//				else
//				{MG8016L3->Params->iqcontrol = TL[0];}
//					
//				
//				VMC_Len_R->PID_Cal(LinkR->GetPendulumLen());
//				VMC_Rad_R->PID_Cal(LinkR->GetPendulumRadian());
//				
//				float FR[2]={VMC_Len_R->GetOut(),-VMC_Rad_R->GetOut()};
//				float TR[2];
//				LinkR->VMCCal(FR,TR);
//	

//	//			/*电机保护*/
//				
//				/*-20*/
//				if(MG8016R2->GetRadian()>3.054326f)
//				{MG8016R2->Params->iqcontrol = (-TR[1]>PWR_LIMIT) ? PWR_LIMIT : -TR[1];}
//				else
//				{MG8016R2->Params->iqcontrol = -TR[1];}
//				

//				if((MG8016R3->GetRadian()>PI)||(MG8016R3->GetRadian()<0.1047197f))
//				{MG8016R3->Params->iqcontrol = (-TR[0]<-PWR_LIMIT) ? -PWR_LIMIT : -TR[0];}
//				else
//				{MG8016R3->Params->iqcontrol = -TR[0];}
//				

//			}
//		}
//		else
//		{
//			StartMode = 1;
//			VMC_Len_L->Reset();
//			VMC_Len_R->Reset();
//			Lens = LEGMID;
//			tx_thread_sleep(100);
//		}
//		
//		tx_thread_sleep_until(&timer,2);
//	}
//}
//                       