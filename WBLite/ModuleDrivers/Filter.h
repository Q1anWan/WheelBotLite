/*
	2023.7.12
	V1.2
	Add BufClean function
*/
#ifndef FILTER_H
#define FILTER_H
#include "main.h"
#include "arm_math.h"
#ifdef __cplusplus
class cFilterBTW3_30Hz
{
	arm_biquad_casd_df1_inst_f32 BTW3;
	float IIRBuf[8]={0};
	float IIRCoeff[10]={1.0f, 2.0f, 1.0f, 1.570486f, -0.6891003f, 1.0f, 1.0f, 0.0f, 0.6795993f, 0.0f};
	float Gain = 0.029653640486155225658437828428759530652*0.160200350887736708838104959795600734651;
	
	public:
	cFilterBTW3_30Hz(){arm_biquad_cascade_df1_init_f32(&this->BTW3, 2, this->IIRCoeff, this->IIRBuf);}
	float BTW3Cal(float data);
	void CleanBuf(void)
	{IIRBuf[0]=0;IIRBuf[1]=0;IIRBuf[2]=0;IIRBuf[3]=0;IIRBuf[4]=0;IIRBuf[5]=0;IIRBuf[6]=0;IIRBuf[7]=0;}
};

class cFilterBTW2_5Hz
{
	arm_biquad_casd_df1_inst_f32 BTW2;
	float IIRBuf[4]={0};
	float IIRCoeff[5]={1.0f, 2.0f, 1.0f, 1.911197f, -0.9149758f};
	float Gain = 0.000944691843840150748297379568185760945f;
	
	
	public:
	cFilterBTW2_5Hz(){arm_biquad_cascade_df1_init_f32(&this->BTW2, 1, this->IIRCoeff, this->IIRBuf);}
	float BTW2Cal(float data);
	void CleanBuf(void)
	{IIRBuf[0]=0;IIRBuf[1]=0;IIRBuf[2]=0;IIRBuf[3]=0;}
};

class cFilterBTW2_40Hz
{
	arm_biquad_casd_df1_inst_f32 BTW2;
	float IIRBuf[4]={0};
	float IIRCoeff[5]={1.0f, 2.0f, 1.0f, 0.3695273f, -0.1958157f};
	float Gain = 0.20657208382614791752907024147134507075f;
	
	
	public:
	cFilterBTW2_40Hz(){arm_biquad_cascade_df1_init_f32(&this->BTW2, 1, this->IIRCoeff, this->IIRBuf);}
	float BTW2Cal(float data);
	void CleanBuf(void)
	{IIRBuf[0]=0;IIRBuf[1]=0;IIRBuf[2]=0;IIRBuf[3]=0;}
};

class cFilterBTW4_5Hz
{
	arm_biquad_casd_df1_inst_f32 BTW4;
	float IIRBuf[8]={0};
	float IIRCoeff[10]={1.0f, 2.0f, 1.0f, 1.949216f, -0.9530699f, 1.0f, 2.0f, 1.0f, 1.886610f, -0.8903397f};
	float Gain = 0.000963484325512290753178168412063087089*0.000932538415629474422112454856659269353;
	
	public:
	cFilterBTW4_5Hz(){arm_biquad_cascade_df1_init_f32(&this->BTW4, 2, this->IIRCoeff, this->IIRBuf);}
	float BTW4Cal(float data);
	void CleanBuf(void)
	{IIRBuf[0]=0;IIRBuf[1]=0;IIRBuf[2]=0;IIRBuf[3]=0;IIRBuf[4]=0;IIRBuf[5]=0;IIRBuf[6]=0;IIRBuf[7]=0;}
};




#endif
#endif