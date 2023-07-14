/*
	2023/7/8
	Update Message package
	Benefit for wheelbotlite
*/
#ifndef	CONTROLTASK_H
#define	CONTROLTASK_H
#ifdef __cplusplus
#include "main.h"
#include "arm_math.h"
#include "PID.h"
#include "QCSLite.h"
#include "CloseLoopFun.h"

#include "LinkSolver.h"
#include "RemoterTask.h"
#include "MotorTask.h"
#include "IMUTask.h"
#include "ReferDriver.h"


#define LEGMAX 190.0f
#define LEGMID 145.0f
#define LEGMIN 100.0f

#define RADMAX  2.6f  //PI/3
#define RADMIN  0.52f
#define RADINIT 1.570796f // PI/2

#define RADCOR 0.068f

#define ESCAPE_MAXI_FORWARD 1500
#define ESCAPE_MAXI_ROLL 1500

#define GIMBAL_PITCH_MAX
#define GIMBAL_YAW_MIN


#define STATUS_ERROR_BIT (uint8_t)4

#define ROBOTPART_HEATSAFT_VALUE 20
#define ROBOTPART_TROOGLEMODE_VALUE 20

#define CHASIS_FOLLOW_GIMBAL_RANGE 			0.5235988f // 30 dgrees 底盘追踪云台不严格范围
#define CHASIS_FOLLOW_GIMBAL_SAFE_RANGE		0.5235988f // 30 dgrees 底盘位置闭环异常值 超出此值后期望角度将不再继续增加

#define CHASIS_SPEED_MAX 			2.0f // 2m/S
#define CHASIS_SPEED_SIDE_MAX 		1.0f // 1m/S
#define CHASIS_SPEED_ACCEL			0.01f 			/*  2m/s^2 (val = accel/200)	*/
#define CHASIS_SPEED_SIDE_ACCEL		0.005f 			/*  1m/s^2 (val = accel/200)	*/
#define CHASIS_ANGYLAR_VELOCITY_MAX	0.03141592f 	/*	1r/s (val = rps*2pi/200)	*/

#define GIMBAL_PITCH_ANGYLAR_VELOCITY 50			/*	PI rad/s	Radian*10000/(PI*200) */
#define GIMBAL_YAW_ANGYLAR_VELOCITY	  50			/*	PI rad/s	Radian*10000/(PI*200) */

enum eRobotLQRID
{
	LQR_BOTTOM=0,
	LQR_MIDDLE,		
	LQR_TOP,	
};

/*机器人整体状态*/
/*
	Robot total mode.
	
	ROBOTMODE_IDLE means robot in IDLE mode, all functions will be disabled.
	
	ROBOTMODE_ESCAPE means robot in escape mode. Gimbal and Ammobooster will be enabled.
	Balance will disabel, but 2 wheel motors will work at open loop mode to help robot escape danger.
	
	MDOE_NORMAL means robot in normal mode. Gimbal,Ammobooster.Leg,Banlance will be enbaled.
*/
enum eRobotMode
{
	ROBOTMODE_IDLE=0,
	ROBOTMODE_ESCAPE,
	ROBOTMODE_NORMAL,
};


/*机器人部件序号*/
enum eRobotPart
{
	ROBOTPART_ID_LEG = 0,
	ROBOTPART_ID_BLC,
	ROBOTPART_ID_GIM,
	ROBOTPART_ID_BST,
};


/*机器人部件状态*/
/*
	Robot part statue.
	
	ROBOTPART_STATUS_IDLE 	means part  work in IDLE statue. Functions will be disabled.
	ROBOTPART_STATUS_START 	means part is going to work in normal statue. Start mode is a buffer besides in IDLE and Normal.
	ROBOTPART_STATUS_NORMAL 	means part work i normal statue. This statue shouldn't change in directly.
*/
enum eRobotStatus
{
	ROBOTPART_STATUS_IDLE=1,
	ROBOTPART_STATUS_START,
	ROBOTPART_STATUS_NORMAL,
};


/*Some functions' flag*/
/*Flag of BoosterMode*/
enum eRobotPartBoosterMode
{
	ROBOTPART_BOOSTER_STOP=0,	/* Booster Stop */
	ROBOTPART_BOOSTER_15MPS,	/* Boost ammos at 15m/s */
	ROBOTPART_BOOSTER_18MPS,	/* Boost ammos at 18m/s */
	ROBOTPART_BOOSTER_30MPS,	/* Boost ammos at 30m/s */
};

enum eRobotPartTriggleMode
{
	ROBOTPART_TRIGGLE_STOP=0,	/* Triggle Stop */
	ROBOTPART_TRIGGLE_MMODE0,	/* Triggle Muliti Mode 0 */
	ROBOTPART_TRIGGLE_MMODE1,	/* Triggle Muliti Mode 1 */
	ROBOTPART_TRIGGLE_SINGLE,	/* Triggle Single	Mode */
};

enum eRobotPartShotStatus
{
	ROBOTPART_SHOT_FREE=0,		/* Free to Shoot */
	ROBOTPART_SHOT_SHOOTING,	/* Shooting */
	ROBOTPART_SHOT_JAM,			/* Ammo Jam */
	ROBOTPART_SHOT_JAMSOLVING,	/* Ammo Jam Solving */
};

enum eRobotPartClipStatus
{
	ROBOTPART_CLIP_OVER75=0,	/* More than 75% ammos in Clip */
	ROBOTPART_CLIP_OVER50,		/* More than 50% ammos in Clip */
	ROBOTPART_CLIP_OVER25,	/* More than 25% ammos in Clip */
	ROBOTPART_CLIP_NONE,		/* No ammos in Clip */
};

enum eRobotPartChasisMode
{
	ROBOTPART_CHASIS_SHUTTLE=0,	/* Normal moving */
	ROBOTPART_CHASIS_FLYING,	/* Feipo */
	ROBOTPART_CHASIS_SIDEMODE,	/* Side mode */
	ROBOTPART_CHASIS_ROLLING,	/* Xiaotuoluo */
};

/*
	Please read README.md to confirm the exactly functions
	
	This struct is the control package from CHASIS to GIMBAL.
	COMDOWNSIZE sizes of package in byte
	YawInc		Increase value of Yaw 	in radian.
	PitInc		Increase value of Pitch in radian.
	Fire		Open fire flag. Should be treat differently in one-shot or free-fire mode.
	BoosterMode Boost mode. Show ammo's velocity.
	TriggleMode	Triggle mode. Shows shoot frequence.
	CapOpen		Flag of open cap.
	DeathFlag	Is robot 0 HP.
	RSTFLAG		Robot reboot flag.May be not use.
*/
#define COMDOWNSIZE 5
typedef __PACKED_STRUCT
{
	int16_t YawInc;            //Yaw增量  -10000~10000表示-PI~PI 其他表示失能
	int16_t PitInc;            //Pitch增量 -10000~10000表示-PI~PI 其他表示失能
	uint8_t Fire  : 1;         //开火标志位
	uint8_t BoosterMode  : 2;  //摩擦轮射速模式
	uint8_t TriggleMode  : 2;  //拨盘模式
	uint8_t CapOpen      : 1;  //是否开启弹舱盖
	uint8_t GimDeathFlag : 1;  //云台关断标记位
	uint8_t RobotDeath   : 1;  //机器人阵亡标志位
}tComDown;

/*
	Please read README.md to confirm the exactly functions
	
	This struct is the message package from GIMBAL to CHASIS.
	
*/
#define COMUPSIZE 1
typedef __PACKED_STRUCT
{
	uint8_t ShootStatue : 2;	//发射状态
	uint8_t AmmoCap		: 2;	//弹舱容量
	uint8_t Reserved	: 4;	//resevered
}tComUP;


struct tChasisVal
{
	float ChasisPsaiYaw;	//Chasis yaw angel
	float ChasisRoll;	//Chasis roll angel
	float ChasisPitch;	//Chasis roll angel
		
	float ChasisLegLen[2];	//Chasis leg length
	float X[6]={0};			//Theta ThetaDot Xbody XbodyDot Phi PhitDot
							//PendulumRadian PendulumRadianSpeed Distance Velocity BodyPitchRadian BodyPitchRadianSpeed 
	arm_matrix_instance_f32 MatX = {6, 1, X};
};

class cChasisControl
{
	protected:
		
	/*This flag is designed to make sure chasis control mode*/
	eRobotPartChasisMode ChasisMode;
	
	/*Leg length level*/
	eRobotLQRID LegLen;
	float LeglengthVal[3]={LEGMIN,LEGMID,LEGMAX};
	
	/*Is blance ok*/
	uint8_t BanlanOKFlag = 0;
	
	
	/*Variates about chasis follow gimbal */
	/*0 for X+, 1 for X-*/
	uint8_t ChasisHead = 0;
	float ChasisGimbalPositionCor = 0.0f;	/*Correct value*/
	float ChasisGimbalPositionErr=0.0f;		/*Really Error. Chasis way is Zero.Always from -PI to Pi*/
	uint8_t ChasisFollowGimbalConincidetly = 0; /*Is Chasis strict tracking*/
	uint8_t ChasisFollowGimbalEn = 0; 	/*Is chasis folow gimbal enable*/
	
	
	public:
		
	int16_t OpenLoopWheelI[2];
	tChasisVal TargetVal;	/*Refer value*/
	tChasisVal ObserveVal;	/*Observed value*/
	
	cLQR LQR;
	cLoopCFG LoopCFG;
	cLoopYaw LoopYaw;
	cLoopRoll LoopRoll;
	cLoopLen LoopLen[2];/*Left Right*/
	cMotorUnit *MotorUnits;
		
	/*This velocity should be changed to X[3] in different situation*/
	float ChasisForwardTargetVelocity=0.0f;	

	
	cChasisControl()
	{
		this->ChasisMode = ROBOTPART_CHASIS_SHUTTLE;
		this->LegLen = LQR_BOTTOM;
		this->LQR.InitMatX(&this->TargetVal.MatX,&this->ObserveVal.MatX);
	}
	
	/*To make sure is banlance ok*/
	inline void SetBalanceFlag(uint8_t IsOK)
	{this->BanlanOKFlag = IsOK?1:0;}
	inline uint8_t GetBalanceFlag(void)
	{return this->BanlanOKFlag;}
	
	/*To make sure in shuttle side or other mods*/
	inline eRobotPartChasisMode GetChasisMode(void)
	{return this->ChasisMode;}
	inline void SetChasisMode(eRobotPartChasisMode Mode)
	{this->ChasisMode = Mode;}
	
	
	/*Functions about chasis head.(Postive direction) */
	/*
		Set raw error of chasis and gimbal.
		This value should be absolutely error of chasis and gimbal.
		Chasis is zero.
		-PI ~ PI
	*/
	inline void SetErrWithGim(float Radian)
	{this->ChasisGimbalPositionErr = Radian;}
	
	/*Get head direction*/
	inline uint8_t GetChasisHead(void)
	{return this->ChasisHead;}
	/*Get Forward velocity*/
	inline float GetForwardVelocity(void)
	{return this->ChasisHead?-ChasisForwardTargetVelocity:ChasisForwardTargetVelocity;}

	
	/*Change chasis and gimbal error's direction*/
	void RefreshChasisHead(void)
	{
		/*Find minimum radian to change head*/
		if(this->ChasisGimbalPositionErr < -PI_Half)
		{this->ChasisGimbalPositionCor = PI;this->ChasisHead=1;}
		else if(this->ChasisGimbalPositionErr > PI_Half)
		{this->ChasisGimbalPositionCor = -PI;this->ChasisHead=1;}
		else{this->ChasisGimbalPositionCor = 0;this->ChasisHead=0;}
	}
	/*Get chasis follow gimbal error*/
	float GetCFGErr(void)
	{ 
		float tmp = this->ChasisGimbalPositionErr + this->ChasisGimbalPositionCor;
		
		if(tmp>PI){tmp-=PI;}
		else if(tmp<-PI){tmp+=PI;}
		
		return tmp;
	}
	/*Set chasis follow gimbal conincidently enable or not*/
	inline void SetCFGSStatue(uint8_t IsEnable)
	{this->ChasisFollowGimbalConincidetly = IsEnable?1:0;}
	/*Check is strict tracking*/
	inline uint8_t GetCFGSFlag(void)
	{return this->ChasisFollowGimbalConincidetly;}
	/*Set is chasis folow gimbal enable*/
	inline void SetCFGENStatue(uint8_t IsEnable)
	{this->ChasisFollowGimbalEn = IsEnable?1:0;}
	/*Check is chasis folow gimbal enable*/
	inline uint8_t GetCFGENFlag(void)
	{return this->ChasisFollowGimbalEn;}
	

	inline void SetLegLen(eRobotLQRID ID)
	{this->LegLen = ID;}
	inline eRobotLQRID GetLegLenFlag(void)
	{return this->LegLen;}
	inline float GetLegLen(void)
	{return this->LeglengthVal[(uint8_t) this->LegLen];}
	
};


class cRobotControl
{
	protected:
	/*[0,1]表示状态 [2]表示有无异常*/
	/*状态设置与异常标志位完全分离*/
	uint8_t	RobotMode:3;			//整机器人
	uint8_t LegStatus:3;			//腿状态
	uint8_t BlcStatus:3;			//平衡状态
	uint8_t GimbalStatus:3;			//云台状态
	uint8_t BoosterStatus:3;		//发射状态
	
	robot_referee_status_t::ext_game_robot_status_t *ext_game_robot_status;//Robot status 0x0202
	robot_referee_status_t::ext_power_heat_data_t *ext_power_heat_data;//Robot power heat data 0x0203
	
	
	public:
	cRobotControl()
	{
		this->LegStatus = ROBOTPART_STATUS_IDLE;
		this->BlcStatus = ROBOTPART_STATUS_IDLE;
		this->GimbalStatus = ROBOTPART_STATUS_IDLE;
		this->BoosterStatus = ROBOTPART_STATUS_IDLE;
	}
		
	cINS *INS=0;
	
	tComDown ComDown;
	tComUP	 ComUP;
	cChasisControl ChasisControl;
	
	
	void SetINS(cINS *INS);
	void UpdataABSGimbal(void);
	void UpdateABSChasis(void);
	
	/*Set refer system data*/
	void SetRefSysDataSource(robot_referee_status_t::ext_game_robot_status_t *robot_status,robot_referee_status_t::ext_power_heat_data_t *power_heat_data)
	{
		this->ext_game_robot_status = robot_status;
		this->ext_power_heat_data = power_heat_data;
	}
	uint16_t GetRobotHP(void)
	{return this->ext_game_robot_status->remain_HP;}
	uint16_t GetRobotRemainHeat(void)
	{return (this->ext_game_robot_status->shooter_id1_17mm_cooling_limit - this->ext_power_heat_data->shooter_id1_17mm_cooling_heat);}
	uint16_t GetRobotCoolRate(void)
	{return this->ext_game_robot_status->shooter_id1_17mm_cooling_rate;}
	uint16_t GetRobotAmmoSpeed(void)
	{return this->ext_game_robot_status->shooter_id1_17mm_speed_limit;}
	
	/**@Input RobotMode which follow eRobotMode*/
	/*This function will set robot mode, which will not change error flag*/
	inline void SetRobotMode(eRobotMode mode)
	{
		this->RobotMode &= 0x04;
		this->RobotMode |= mode;
	}
	
	/**@Input part follow eRobotPart*/
	/**@Input mode follow eRobotStatus*/
	/*This function will set robot mode, which will not change error flag*/
	void SetPartMode(eRobotPart Part, eRobotStatus mode)
	{
		switch (Part)
		{
			case ROBOTPART_ID_LEG:
			{this->LegStatus &= 0x04;this->LegStatus |= mode;}
			break;
			case ROBOTPART_ID_BLC:
			{this->BlcStatus &= 0x04;this->BlcStatus |= mode;}
			break;
			case ROBOTPART_ID_GIM:
			{this->GimbalStatus &= 0x04;this->GimbalStatus |= mode;}
			break;
			case ROBOTPART_ID_BST:
			{this->BoosterStatus &= 0x04;this->BoosterStatus |= mode;}
			break; 
		}
	}
		
	inline uint8_t CheckRobotMode(void)
	{return this->RobotMode;}
	
	uint8_t CheckPartMode(eRobotPart Part)
	{
		switch (Part)
		{
			case ROBOTPART_ID_LEG:
			{return this->LegStatus;}
			break;
			case ROBOTPART_ID_BLC:
			{return this->BlcStatus;}
			break;
			case ROBOTPART_ID_GIM:
			{return this->GimbalStatus;}
			break;
			case ROBOTPART_ID_BST:
			{return this->BoosterStatus;}
			break; 
		}
	}	
};


//class cVMC_Len : public cPIDPla
//{
//	public:
//	float MaxLen = LEGMAX;
//	float MinLen = LEGMIN;
//	
//	cVMC_Len(void)
//	{PID_Init();}
//	
//	void PID_Init(void)
//	{
//		this->Ref = LEGMID;
//		this->Fs = 0;//500.0f;
//		this->Kp = 0.15f;//0.15f;//10.0f;
//		this->Ki = 0.0f;//1.0;
//		this->Kd = 7.0f;//7.0f;
//		this->Kf = 0;//-0.002f;
//		this->IN_RANGE_EN_D = 0.0f;// Pi/3
//		this->IN_RANGE_EN_I = 0.0f;
//		this->MaxOutValue = 100.0f;
//		this->MinOutValue = -100.0f;
//		this->Maxintegral = 0.0f;
//		this->Minintegral = -0.0f;
//	}
//	
//	void SetRef(float Ref)
//	{
//		if(Ref>this->MaxLen)
//		{this->Ref = this->MaxLen;}
//		else if(Ref<this->MinLen)
//		{this->Ref = this->MinLen;}
//		else{this->Ref = Ref;}
//	}
//};


//class cVMC_Rad : public cPIDPla
//{
//	public:
//	float MaxRad = RADMAX;
//	float MinRad = RADMIN;
//	
//	cVMC_Rad(void)
//	{PID_Init();}
//	
//	void PID_Init(void)
//	{
//		this->Ref = RADINIT;
//		this->Fs = 0.0f;//500.0f;
//		this->Kp = 15000.0f;//16000.0f;//2400.0f;//10.0f;
//		this->Ki = 0.0f;//1.0;
//		this->Kd = 180000.0f;//160000.0f;
//		this->Kf = 0;//0.3f;//-0.002f;
//		this->IN_RANGE_EN_D = 0.0f;// Pi/3
//		this->IN_RANGE_EN_I	= 0.0f;
//		this->MaxOutValue = 1200.0f;//500.0f;
//		this->MinOutValue = -1200.0f;//-500.0f;
//		this->Maxintegral = 0.0f;
//		this->Minintegral = -0.0f;
//	}
//	
//	void SetRef(float xRef)
//	{		
//		if(xRef>this->MaxRad)
//		{this->Ref = this->MaxRad;}
//		else if(xRef<this->MinRad)
//		{this->Ref = this->MinRad;}
//		else{this->Ref = xRef;}
//	}
//	
//	void SetPD(float xKp, float xKd)
//	{
//		this->Kp = xKp;
//		this->Kd = xKd;
//	}
//};


//class cVMC_Roll : public cPIDPla
//{
//	public:
//	
//	float MidLen = LEGMID;
//	cVMC_Roll(void)
//	{PID_Init();}
//	
//	void PID_Init(void)
//	{
//		this->Fs = 0.0f;
//		this->Kp = 170.0f;
//		this->Ki = 1.0f;
//		this->Kd = 120.0f;
//		this->Kf = 0.0f;
//		this->IN_RANGE_EN_D = 0.0f;// Pi/3
//		this->IN_RANGE_EN_I = 0.0f;
//		this->MaxOutValue = LEGMAX - LEGMID;
//		this->MinOutValue = LEGMIN - LEGMID;
//		this->Maxintegral = 10.0f;
//		this->Minintegral = -10.0f;
//	}
//	
//	inline float GetMidLen(void)
//	{return this->MidLen;}
//};

//class cVMC_Speed : public cPIDPla
//{
//	public:
//	
//	float MaxSpeed=4.0f;
//	float MinSpeed=-4.0f;
//	float RadCor = RADCOR;
//	
//	cVMC_Speed(void)
//	{PID_Init();}
//	
//	void PID_Init(void)
//	{
//		this->Fs = 500.0f;
//		this->Kp = 0.0675f;
//		this->Ki = 0.0005f;
//		this->Kd = 0.01f;
//		this->Kf = 0.000001f;
//		this->IN_RANGE_EN_D = 0.0f;// Pi/3
//		this->IN_RANGE_EN_I = 0.0f;
//		this->Maxintegral = 100.0f;
//		this->Minintegral = -100.0f;
//		this->MaxOutValue = (RADMAX - RADMIN);
//		this->MinOutValue = -(RADMAX - RADMIN);
//	}
//	
//	void SetRef(float Ref)
//	{
//		if(Ref>this->MaxSpeed)
//		{this->Ref = this->MaxSpeed;}
//		else if(Ref<this->MinSpeed)
//		{this->Ref = this->MinSpeed;}
//		else{this->Ref = Ref;}
//	}
//	
//	inline float GetOut(void)
//	{return (this->Out+this->RadCor+RADINIT);}
//};




extern "C"{

}
#endif
#endif
