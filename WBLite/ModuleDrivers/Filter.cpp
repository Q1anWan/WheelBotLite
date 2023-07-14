#include "Filter.h"
/*
	2023.7.12
	V1.2
	Add BufClean function
*/

/*
	cFilterBTW3_30Hz
	滤波器结构  : 直接 I 型，二阶节                                                                            
	节数     : 2 
	fs=500Hz,fc=30Hz,阶数=3

	下面是CMSIS函数需要的内容
	b0 b1 b2 a0 a1 a2
	1  2  1  1  1.570485783501867516065431118477135896683  -0.689100345446488571354848318151198327541
	b0 b1 b2 a0 a1 a2
	1  1  0  1  0.679599298224526471301487617893144488335  0      

	增益:                                                                                             
	0.029653640486155225658437828428759530652                                                        
	0.160200350887736708838104959795600734651
*/


/*
	系数矩阵
	b00 b01 b02 a01 a02
	b10 b11 b12 a11 a12
*/

/*
const float32_t IIRCoeffs32LP[10] = {                                                                                 
1.0f, 2.0f, 1.0f, 1.570486f, -0.6891003f,
1.0f, 1.0f, 0.0f, 0.6795993f, 0.0f
};
*/

/*运算速度 240Mhz下810ns*/
float cFilterBTW3_30Hz::BTW3Cal(float data)
{
	float Output = 0;
	arm_biquad_cascade_df1_f32(&this->BTW3, &data, &Output ,1);
	return Output*this->Gain;
}


/*
	cFilterBTW2_5Hz
	滤波器结构  : 直接 I 型，二阶节                                                                            
	节数     : 1 
	fs=500Hz,fc=5Hz,阶数=2
*/
float cFilterBTW2_5Hz::BTW2Cal(float data)
{
	float Output = 0;
	arm_biquad_cascade_df1_f32(&this->BTW2, &data, &Output ,1);
	return Output*this->Gain;
}

/*
	cFilterBTW2_40Hz
	滤波器结构  : 直接 I 型，二阶节                                                                            
	节数     : 1 
	fs=500Hz,fc=40Hz,阶数=2
*/
float cFilterBTW2_40Hz::BTW2Cal(float data)
{
	float Output = 0;
	arm_biquad_cascade_df1_f32(&this->BTW2, &data, &Output ,1);
	return Output*this->Gain;
}

/*
	cFilterBTW4_5Hz
	滤波器结构  : 直接 I 型，二阶节                                                                            
	节数     : 2 
	fs=500Hz,fc=5Hz,阶数=4
*/
float cFilterBTW4_5Hz::BTW4Cal(float data)
{
	float Output = 0;
	arm_biquad_cascade_df1_f32(&this->BTW4, &data, &Output ,1);
	return Output*this->Gain;
}