#ifndef	CLOSELOOPFUN_H
#define	CLOSELOOPFUN_H
#ifdef __cplusplus
#include "ControlTask.h"

/*ÐÐ½ø¿ØÖÆ*/
class cLQR
{		
	protected:
	//Bottom Middle Top 3 in 1
	//Bot :LQRKbuf[0][]
	//Mid :LQRKbuf[1][]
	//Bot :LQRKbuf[2][]
	float LQRKbuf[3][12]=
	{
		//Order: K00 K01 K02 K03 K04 K05 K10 K11 K12 K13 K14 K15
		{7.8698,   0.6602,   2.1559,   3.7122,  -5.9936,  -0.6556,  -2.8884,  -0.2665,  -0.6851,  -1.1447, -15.7729,  -1.0864},	//K BOT 100 
		{8.7097,   0.7137,   2.1965,   3.6629,  -4.3517,  -0.5257,  -2.1968,  -0.1799,  -0.4837,  -0.7718, -16.4165,  -1.1484},	//K MID 145 
		{9.3048,   0.7603,   2.2161,   3.6346,  -3.2732,  -0.4387,  -1.7164,  -0.1241,  -0.3447,  -0.5315, -16.7164,  -1.1804},	//K TOP 190
	};
	
	float LQROutBuf[2]={0};
	float LQRXerrorBuf[6]={0};
	
	arm_matrix_instance_f32 *LQRXRefX;
	arm_matrix_instance_f32 *LQRXObsX;
	
	
	arm_matrix_instance_f32 MatLQRNegK = {2, 6, (float*)LQRKbuf[0]};
	arm_matrix_instance_f32 MatLQRErrX = {6, 1, LQRXerrorBuf};
	arm_matrix_instance_f32 MatLQROutU = {2, 1, LQROutBuf};
	
	
	
	public:
		
	/*Calculate X. Output is u (T,Tp)`*/
	void LQRCal(float* Tout)
	{
		//Calculate error
		arm_mat_sub_f32(this->LQRXObsX,this->LQRXRefX,&this->MatLQRErrX);
		//Calculate output value
		arm_mat_mult_f32(&this->MatLQRNegK,&this->MatLQRErrX,&this->MatLQROutU);
		//return Value
		Tout[0] = this->LQROutBuf[0];
		Tout[1] = this->LQROutBuf[1];
	}
	
	/* SetLQR -K paramaters*/
	inline void RefreshLQRK(uint8_t ID)
	{this->MatLQRNegK.pData = (float*)LQRKbuf[ID];}
	
	/* Set LQR Error Variate*/
	void InitMatX(arm_matrix_instance_f32* pMatXRef, arm_matrix_instance_f32* pMatXObs)
	{
		this->LQRXRefX = pMatXRef;
		this->LQRXObsX = pMatXObs;
	}
};

/*º½Ïò½Ç×·×Ù*/
class cLoopYaw : public cPIDPla
{
	public:

	cLoopYaw(void)
	{PID_Init();}
	
	void PID_Init(void)
	{
		this->Fs = 0.0f;
		this->Kp = 0.0f;
		this->Ki = 0;
		this->Kd = 0;
		this->Kf = 0.0f;
		this->IN_RANGE_EN_D = 1.05;// Pi/3
		this->IN_RANGE_EN_I = 0.6;
		this->MaxOutValue = 1;
		this->MinOutValue = -1;
		this->Maxintegral = 0;
		this->Minintegral = 0;
	}
};

/*µ¹Á¢°Ú³¤¶È*/
class cLoopLen : public cPIDPla
{
	public:
	cLoopLen(void)
	{PID_Init();}
	
	void PID_Init(void)
	{
		this->Fs = 0.0f;
		this->Kp = 0.0030f;
		this->Ki = 0.0f;
		this->Kd = 0.02f;
		this->Kf = 0.0f;
		this->IN_RANGE_EN_D = 0;// Pi/3
		this->IN_RANGE_EN_I = 0;
		this->MaxOutValue = 0.2f;
		this->MinOutValue = -0.2f;
		this->Maxintegral = 3.0f;
		this->Minintegral = -3.0f;
	}
};

/*Ð±ÆÂÆ½ÎÈ*/
class cLoopRoll : public cPIDPla
{
	public:
	cLoopRoll(void)
	{PID_Init();}
	
	void PID_Init(void)
	{
		this->Fs = 0.0f;
		this->Kp = 0.0f;
		this->Ki = 0;
		this->Kd = 0;
		this->Kf = 0.0f;
		this->IN_RANGE_EN_D = 0;// Pi/3
		this->IN_RANGE_EN_I = 0;
		this->MaxOutValue = 1;
		this->MinOutValue = -1;
		this->Maxintegral = 0;
		this->Minintegral = 0;
	}
};

/*µ×ÅÌ¸úËæÔÆÌ¨*/
class cLoopCFG : public cPIDPla
{
	public:
	cLoopCFG(void)
	{PID_Init();}
	
	void PID_Init(void)
	{
		this->Fs = 0.0f;
		this->Kp = 0.0f;
		this->Ki = 0;
		this->Kd = 0;
		this->Kf = 0.0f;
		this->IN_RANGE_EN_D = 0;// Pi/3
		this->IN_RANGE_EN_I = 0;
		this->MaxOutValue = 1;
		this->MinOutValue = -1;
		this->Maxintegral = 0;
		this->Minintegral = 0;
	}
};

/*±ÜÃâÅü²æ*/
class cLoopTheta : public cPIDPla
{
	public:
	cLoopTheta(void)
	{PID_Init();}
	
	void PID_Init(void)
	{
		this->Fs = 0.0f;
		this->Kp = 0.7f;
		this->Ki = 0;
		this->Kd = 0.2;
		this->Kf = 0.0f;
		this->IN_RANGE_EN_D = 0;// Pi/3
		this->IN_RANGE_EN_I = 0;
		this->MaxOutValue = 1;
		this->MinOutValue = -1;
		this->Maxintegral = 0;
		this->Minintegral = 0;
	}
};


#endif
#endif
