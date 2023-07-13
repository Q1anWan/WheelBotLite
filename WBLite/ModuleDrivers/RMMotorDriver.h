#ifndef RMMOTORDRIVER_H
#define RMMOTORDRIVER_H
#ifdef __cplusplus
#include "main.h"
#include "cmsis_compiler.h"

#define RMECDRESLUTON 8192

#ifndef PI
  #define PI               3.14159265358979f
#endif

typedef __PACKED_STRUCT
{
	float angle; // ת�ӽ�λ�ƣ�rad��
	float polar_angle;//����(-pi~pi)
	float angular_velocity; // ת�ӽ��ٶ�
	float velocity; // ���ٶȣ�m/s��
}Motor_RM_Output;


class cRMMotor
{
	protected:
	int16_t	ecdoffset = 0;
	int16_t	ecdcorrect = 0;
	
	int16_t setcurrent = 0;
	
	struct
	{
		int16_t ecd;
		int16_t rpm;
		int16_t current;
		uint8_t temperature;
	}MotorRec;
	
	struct
	{
		int64_t multiecd = 0;
		int32_t loops	 = 0;
		int16_t lastecd	 = 0; 
	}MultiLoopData;
	
	struct 
	{
		int16_t lastecd	 = 0;
		
		float fs		= 1000.0f;// Ƶ��
		float reduction_ratio_rev = 1.0f;	// ���ٱȵ���
		float wheelR	= 1.0f;			// �ְ뾶
		float filterk	= 0.63f;// �˲������� 1KƵ��ʱ100Hz����
		
		float LineVel	= 0.0f;
		float RadianVel = 0.0f;
	}SpcltData;//�Ʋ�����
	
	
	public:
	
	inline void SetCurrent(int16_t current)
	{this->setcurrent = current;}
	
	inline int16_t GetSetCurrent(void)
	{return this->setcurrent;}
		
	void UpdateMotorRec(uint8_t* data);
			
	inline int16_t GetEcd(void)
	{return this->MotorRec.ecd;}
	
	inline int16_t GetEcdCorrect(void)
	{return this->ecdcorrect;}
	
	inline void SetEcdOffset(int16_t offeset)
	{this->ecdoffset = offeset;}
	
	inline int16_t GetRpm(void)
	{return this->MotorRec.rpm;}
	
	inline int16_t GetCurrent(void)
	{return this->MotorRec.current;}
	
	inline uint8_t GetTemperature(void)
	{return this->MotorRec.temperature;}
	
	
	
	void UpdateMultiLoopData(void);
	
	void ResetMultiLoopData(void);
	
	inline int64_t GetMultiEcd(void)
	{return this->MultiLoopData.multiecd;}
	
	inline int32_t GetMultiLoops(void)
	{return this->MultiLoopData.loops;}
	
	
	void ConfigSpcltPara(float fs,float CutoffF, float revreduceratio, float wheelR);
	
	void UpdateSpcltData(void);
	
	void ResetSpcltData(void);
	
	inline float GetLineVel(void)
	{return this->SpcltData.LineVel;}
	inline float GetRadianVel(void)
	{return this->SpcltData.RadianVel;}
	
};


extern "C"{
	
}

#endif
#endif
