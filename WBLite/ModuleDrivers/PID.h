/*********************************************************************************
  *FileName:		PID.h
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
#ifndef PID_H
#define PID_H
#include "main.h"
#ifdef __cplusplus
class cPIDInc
{
	public:
	
	void Reset(void);
	
	virtual void PID_Init() = 0;
	/*PID���㺯��*/
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
	/*ǰ����*/
	float ForwardLoop(void);
	
	float	Fs = 0.0f;//Ƶ�� ��λHz
	
	float	Ref = 0.0f;
	float	LastRef = 0.0f;
	float 	FeedBack = 0.0f;
	
	float 	Error = 0.0f; //���	
	float 	DError = 0.0f;
	float 	DDError = 0.0f;
	float 	PreError = 0.0f;
	float 	PreDError = 0.0f;
		
	float Kp=0.0f; //PID����
	float Ki=0.0f;
	float Kd=0.0f;
	float Kf=0.0f; //ǰ������ϵ��

	float IN_RANGE_EN_D = 0.0f;//����΢���Χ ֵΪ0ʱʼ�տ���
	float IN_RANGE_EN_I = 0.0f;//���������Χ ֵΪ0ʱʼ�տ���
	
	float MaxOutValue=0; //����޷�
	float MinOutValue=0;
	
	float Out = 0.0f; //���ֵ
};

class cPIDPla
{
	//λ��ʽ
	public:
	
	void Reset(void);
	
	virtual void PID_Init() = 0;
	/*PID���㺯��*/
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
	/*ǰ����*/

	
	protected:
	
	virtual float ForwardLoop(void);
	
	float	Fs = 0.0f;//Ƶ�� ��λHz
	
	float	Ref = 0.0f;
	float	LastRef = 0.0f;
	float 	FeedBack = 0.0f;	
	
	float 	Error = 0.0f; //���
	float 	DError = 0.0f;

	float integral = 0.0f;//���Ļ���
	float Maxintegral = 0.0f; //�����޷�
	float Minintegral = 0.0f;
	
	float Kp = 0.0f; //PID����
	float Ki = 0.0f;
	float Kd = 0.0f;
	float Kf = 0.0f; //ǰ������ϵ��
	
	float IN_RANGE_EN_D = 0.0f;//����΢���Χ ֵΪ0ʱʼ�տ���
	float IN_RANGE_EN_I = 0.0f;//���������Χ ֵΪ0ʱʼ�տ���
	
	float MaxOutValue = 0; //����޷�
	float MinOutValue = 0;

	float Out = 0.0f; //���ֵ
};

#endif
#endif