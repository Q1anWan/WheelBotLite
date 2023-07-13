#include "LinkSolver.h"

void cLinkSolver::Resolve(void)
{	
	float xdb = this->MotoDistance + this->L1 * (arm_cos_f32(this->Theta3) - arm_cos_f32(this->Theta2));
	float ydb = this->L1 * (arm_sin_f32(this->Theta3) - arm_sin_f32(this->Theta2));
	
	float A0 = 2 * L2 * xdb;
	float B0 = 2 * L2 * ydb;
	float C0 = xdb*xdb + ydb*ydb;
	float lBD = _sqrtf(C0);
	
	/*计算u2*/
	float u2t=0.0f;
	arm_atan2_f32( (B0+_sqrtf(A0*A0 + B0*B0 - C0*C0)) , (A0+C0) , &u2t);
	this->U2=2.0f*(u2t+PI);
	
	/*计算B坐标*/
	this->CoorB[0] = L1 * arm_cos_f32(this->Theta2) - this->HalfMotoDistance;
	this->CoorB[1] = L1 * arm_sin_f32(this->Theta2);
	/*计算C坐标*/
	this->CoorC[0] = this->CoorB[0] + L2 * arm_cos_f32(this->U2);
	this->CoorC[1] = this->CoorB[1] + L2 * arm_sin_f32(this->U2);
	/*计算D坐标*/
	this->CoorD[0] = L1 * arm_cos_f32(this->Theta3) - this->HalfMotoDistance;
	this->CoorD[1] = L1 * arm_sin_f32(this->Theta3);
	/*计算u3*/
	arm_atan2_f32(this->CoorD[1]-this->CoorC[1], this->CoorD[0]-this->CoorC[0], &this->U3);
	
	
	/*输出摆长摆角*/
	arm_atan2_f32(this->CoorC[1],this->CoorC[0],&this->PendulumRadian);
	this->PendulumLength = _sqrtf(this->CoorC[0]*this->CoorC[0] + this->CoorC[1]*this->CoorC[1]);
}

uint8_t	cLinkSolver::InputLink(float Theta3, float Theta2)
{
	this->Theta3 = Theta3;
	this->Theta2 = Theta2;
	
	if(Theta3>this->Theta3Max)
	{this->LinkStatue = LINK_ERROR;}
	else if(Theta2<this->Theta2Min)
	{this->LinkStatue = LINK_ERROR;}
	else
	{this->LinkStatue = LINK_NORMAL;}
	
	return (uint8_t)this->LinkStatue;
}

void cLinkSolver::VMCCal(float *FT, float *Tmotor)
{
	/*计算VMC*/	
	volatile float q00,q01,q10,q11;
	/*中间变量*/
	volatile float sin32 = arm_sin_f32(this->U3-this->U2);
	volatile float sin12 = arm_sin_f32(this->Theta2 - this->U2);
	volatile float sin34 = arm_sin_f32(this->U3 - this->Theta3);
	
	
	q00 = this->L1 * arm_sin_f32(this->PendulumRadian - this->U3) * sin12 / sin32;
	q01 = this->L1 * arm_cos_f32(this->PendulumRadian - this->U3) * sin12 / (this->PendulumLength *sin32);
	q10 = this->L1 * arm_sin_f32(this->PendulumRadian - this->U2) * sin34 / sin32;
	q11 = this->L1 * arm_cos_f32(this->PendulumRadian - this->U2) * sin34 / (this->PendulumLength *sin32);
	
	/*矩阵乘法*/
	Tmotor[0] = q00*FT[0] + q01*FT[1];
	Tmotor[1] = q10*FT[0] + q11*FT[1];
}