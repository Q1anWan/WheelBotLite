/*********************************************************************************
  *FileName:		PID.cpp
  *Author:  		qianwan
  *Detail: 			PID�㷨����,PID_InitΪ���麯��
  
  *Version:  		1.3
  *Date:  			2023/05/17
  *Describe:		���ǰ����
  
  *Version:  		1.2
  *Date:  			2023/03/13
  *Describe:		�޲�BUG
  
  *Version:  		1.1
  *Date:  			2023/02/26
  *Describe:		��������΢�ַ�Χ�ж��������ʽ

  *Version:  		1.0
  *Date:  			2023/02/10
  *Describe:		���ڲ�������,ȡ����CMSIS-DSP������
**********************************************************************************/
#include "PID.h"
#include "arm_math.h"
/*����ʽPID*/
float cPIDInc::PID_Cal(float fdb)
{
	/*�м���*/
	float _Kp =0.0f;
	float _Ki =0.0f;
	float _Kd =0.0f;
	float _OUT =0.0f;
	
	/*ǰ��׼��*/
	this->FeedBack = fdb;
	this->Error 	= this->Ref 	- this->FeedBack;	
	this->DError 	= this->Error 	- this->PreError;
	this->DDError = this->DError	- this->PreDError;			
	this->PreError = this->Error;
	this->PreDError = this->DError;
			
	/*��������΢������*/
	//pid->Out = pid->Out + (pid->Kp * pid->DError + pid->Ki * pid->Error + pid->Kd * pid->DDError);
	_Kp = this->Kp * this->DError;		
	//I ���ַ���
	if((IN_RANGE_EN_I==0.0f)?1:(fabs(this->Error)<this->IN_RANGE_EN_I))
	{_Ki = this->Ki * this->Error;}		
	//D ΢�ַ���
	if((IN_RANGE_EN_D==0.0f)?1:(fabs(this->Error)<this->IN_RANGE_EN_D))
	{_Kd = this->Kd * this->DDError;}
	
	//���
	_OUT = this->Out + _Kp + _Ki + _Kd + this->ForwardLoop()*this->Kf;
	
	/*���ڴ���*/
	//����޷�
	_OUT = (_OUT > this->MaxOutValue)?this->MaxOutValue:_OUT;
	_OUT = (_OUT < this->MinOutValue)?this->MinOutValue:_OUT;
	//��ֵ
	this->Out = _OUT;
	return _OUT;
}
/* 1�׹��Ի���ǰ���� G(s)=(1+s)/Ts */
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


/*λ��ʽPID*/
float cPIDPla::PID_Cal(float fdb)
{
	/*�м���*/
	float _Kp =0.0f;
	float _Ki =0.0f;
	float _Kd =0.0f;
	float _OUT =0.0f;
	
	/*ǰ��׼��*/
	this->FeedBack = fdb;
	this->Error = this->Ref - this->FeedBack;
	this->integral	+=	this->Error;

	//�����޷�
	this->integral = (this->integral > this->Maxintegral)?this->Maxintegral:this->integral;
	this->integral = (this->integral < this->Minintegral)?this->Minintegral:this->integral;
	
	/*��������΢������*/
	//pid->Out = pid->Kp * pid->Error + pid->Ki * pid->integral  + pid->Kd * (pid->Error - pid->DError);		
	//P	
	_Kp = this->Kp * this->Error;		
	//I ���ַ���
	if((IN_RANGE_EN_I==0)?1:(fabs(this->Error)<this->IN_RANGE_EN_I))	
	{_Ki = this->Ki * this->integral;}		
	//D ΢�ַ���
	if((IN_RANGE_EN_D==0)?1:(fabs(this->Error)<this->IN_RANGE_EN_D))
	{_Kd = this->Kd * (this->Error - this->DError);}		
	//���
	_OUT = _Kp + _Ki + _Kd + this->ForwardLoop()*this->Kf;
			
	/*���ڴ���*/
	//����޷�
	_OUT = (_OUT > this->MaxOutValue)?this->MaxOutValue:_OUT;
	_OUT = (_OUT < this->MinOutValue)?this->MinOutValue:_OUT;
	//��ֵ
	this->DError = this->Error;
	this->Out = _OUT;
	return _OUT;
}
/* 1�׹��Ի���ǰ���� G(s)=(1+s)/Ts */
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