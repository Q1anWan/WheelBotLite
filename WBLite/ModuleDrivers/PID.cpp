/*********************************************************************************
  *FileName:		PID.cpp
  *Author:  		qianwan
  *Detail: 			PID算法基类,PID_Init为纯虚函数
  
  *Version:  		1.3
  *Date:  			2023/05/17
  *Describe:		添加前馈环
  
  *Version:  		1.2
  *Date:  			2023/03/13
  *Describe:		修补BUG
  
  *Version:  		1.1
  *Date:  			2023/02/26
  *Describe:		调整积分微分范围判定条件表达式

  *Version:  		1.0
  *Date:  			2023/02/10
  *Describe:		基于测试数据,取消对CMSIS-DSP的依赖
**********************************************************************************/
#include "PID.h"
#include "arm_math.h"
/*增量式PID*/
float cPIDInc::PID_Cal(float fdb)
{
	/*中间量*/
	float _Kp =0.0f;
	float _Ki =0.0f;
	float _Kd =0.0f;
	float _OUT =0.0f;
	
	/*前期准备*/
	this->FeedBack = fdb;
	this->Error 	= this->Ref 	- this->FeedBack;	
	this->DError 	= this->Error 	- this->PreError;
	this->DDError = this->DError	- this->PreDError;			
	this->PreError = this->Error;
	this->PreDError = this->DError;
			
	/*比例积分微分运算*/
	//pid->Out = pid->Out + (pid->Kp * pid->DError + pid->Ki * pid->Error + pid->Kd * pid->DDError);
	_Kp = this->Kp * this->DError;		
	//I 积分分离
	if((IN_RANGE_EN_I==0.0f)?1:(fabs(this->Error)<this->IN_RANGE_EN_I))
	{_Ki = this->Ki * this->Error;}		
	//D 微分分离
	if((IN_RANGE_EN_D==0.0f)?1:(fabs(this->Error)<this->IN_RANGE_EN_D))
	{_Kd = this->Kd * this->DDError;}
	
	//求和
	_OUT = this->Out + _Kp + _Ki + _Kd + this->ForwardLoop()*this->Kf;
	
	/*后期处理*/
	//输出限幅
	_OUT = (_OUT > this->MaxOutValue)?this->MaxOutValue:_OUT;
	_OUT = (_OUT < this->MinOutValue)?this->MinOutValue:_OUT;
	//赋值
	this->Out = _OUT;
	return _OUT;
}
/* 1阶惯性环节前馈环 G(s)=(1+s)/Ts */
float cPIDInc::ForwardLoop(void)
{
	return (this->Ref - this->LastRef + 1)*this->Fs; 
}
void cPIDInc::Reset(void)
{
	this->FeedBack 	= 0;
	this->Error 	= 0;
	this->DError 	= 0;
	this->DDError 	= 0;	
	this->PreError 	= 0;
	this->PreDError = 0;
	this->Out		= 0;
	this->LastRef	= 0;
}


/*位置式PID*/
float cPIDPla::PID_Cal(float fdb)
{
	/*中间量*/
	float _Kp =0.0f;
	float _Ki =0.0f;
	float _Kd =0.0f;
	float _OUT =0.0f;
	
	/*前期准备*/
	this->FeedBack = fdb;
	this->Error = this->Ref - this->FeedBack;
	this->integral	+=	this->Error;

	//积分限幅
	this->integral = (this->integral > this->Maxintegral)?this->Maxintegral:this->integral;
	this->integral = (this->integral < this->Minintegral)?this->Minintegral:this->integral;
	
	/*比例积分微分运算*/
	//pid->Out = pid->Kp * pid->Error + pid->Ki * pid->integral  + pid->Kd * (pid->Error - pid->DError);		
	//P	
	_Kp = this->Kp * this->Error;		
	//I 积分分离
	if((IN_RANGE_EN_I==0)?1:(fabs(this->Error)<this->IN_RANGE_EN_I))	
	{_Ki = this->Ki * this->integral;}		
	//D 微分分离
	if((IN_RANGE_EN_D==0)?1:(fabs(this->Error)<this->IN_RANGE_EN_D))
	{_Kd = this->Kd * (this->Error - this->DError);}		
	//求和
	_OUT = _Kp + _Ki + _Kd + this->ForwardLoop()*this->Kf;
			
	/*后期处理*/
	//输出限幅
	_OUT = (_OUT > this->MaxOutValue)?this->MaxOutValue:_OUT;
	_OUT = (_OUT < this->MinOutValue)?this->MinOutValue:_OUT;
	//赋值
	this->DError = this->Error;
	this->Out = _OUT;
	return _OUT;
}
/* 1阶惯性环节前馈环 G(s)=(1+s)/Ts */
float cPIDPla::ForwardLoop(void)
{
	return (this->Ref - this->LastRef + 1)*this->Fs; 
}
void cPIDPla::Reset(void)
{
	this->FeedBack 	= 0;
	this->LastRef	= 0;
	this->Error 	= 0;
	this->DError 	= 0;
	this->integral 	= 0;	
	this->Out		= 0;
}