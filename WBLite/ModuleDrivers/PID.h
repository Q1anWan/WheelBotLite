/*********************************************************************************
  *FileName:		PID.h
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
#ifndef PID_H
#define PID_H
#include "main.h"
#ifdef __cplusplus
class cPIDInc
{
	public:
	
	void Reset(void);
	
	virtual void PID_Init() = 0;
	/*PID计算函数*/
	float PID_Cal(float fdb);
	
	inline void SetRef(float Ref)
	{this->LastRef = this->Ref;this->Ref = Ref;}
	
	inline float GetRef(void)
	{return this->Ref;}
	
	inline float  GetError(void)
	{return this->Error;}
	 
	inline float  GetOut(void)
	{return this->Out;}
	
	protected:
	/*前馈环*/
	float ForwardLoop(void);
	
	float	Fs = 0.0f;//频率 单位Hz
	
	float	Ref = 0.0f;
	float	LastRef = 0.0f;
	float 	FeedBack = 0.0f;
	
	float 	Error = 0.0f; //误差	
	float 	DError = 0.0f;
	float 	DDError = 0.0f;
	float 	PreError = 0.0f;
	float 	PreDError = 0.0f;
		
	float Kp=0.0f; //PID参数
	float Ki=0.0f;
	float Kd=0.0f;
	float Kf=0.0f; //前馈增益系数

	float IN_RANGE_EN_D = 0.0f;//开启微分项范围 值为0时始终开启
	float IN_RANGE_EN_I = 0.0f;//开启积分项范围 值为0时始终开启
	
	float MaxOutValue=0; //输出限幅
	float MinOutValue=0;
	
	float Out = 0.0f; //输出值
};

class cPIDPla
{
	//位置式
	public:
	
	void Reset(void);
	
	virtual void PID_Init() = 0;
	/*PID计算函数*/
	float PID_Cal(float fdb);
	
	inline void SetRef(float Ref)
	{this->LastRef = this->Ref;this->Ref = Ref;}
	
	inline float GetRef(void)
	{return this->Ref;}
	
	inline float  GetError(void)
	{return this->Error;}
	 
	inline float  GetOut(void)
	{return this->Out;}
	
	protected:
	/*前馈环*/

	
	protected:
	
	virtual float ForwardLoop(void);
	
	float	Fs = 0.0f;//频率 单位Hz
	
	float	Ref = 0.0f;
	float	LastRef = 0.0f;
	float 	FeedBack = 0.0f;	
	
	float 	Error = 0.0f; //误差
	float 	DError = 0.0f;

	float integral = 0.0f;//误差的积分
	float Maxintegral = 0.0f; //积分限幅
	float Minintegral = 0.0f;
	
	float Kp = 0.0f; //PID参数
	float Ki = 0.0f;
	float Kd = 0.0f;
	float Kf = 0.0f; //前馈增益系数
	
	float IN_RANGE_EN_D = 0.0f;//开启微分项范围 值为0时始终开启
	float IN_RANGE_EN_I = 0.0f;//开启积分项范围 值为0时始终开启
	
	float MaxOutValue = 0; //输出限幅
	float MinOutValue = 0;

	float Out = 0.0f; //输出值
};

#endif
#endif