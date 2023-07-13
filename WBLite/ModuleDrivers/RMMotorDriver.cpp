#include "RMMotorDriver.h"

void cRMMotor::UpdateMotorRec(uint8_t* data)
{
	/*解析原始数据*/
	this->MotorRec.ecd			=(data[0] <<8| data[1]);
	this->MotorRec.rpm			=(data[2] <<8| data[3]);
	this->MotorRec.current		=(data[4] <<8| data[5]);
	this->MotorRec.temperature	= data[6];
	
	//修正编码器数值修正
	int16_t tmp = this->MotorRec.ecd + this->ecdoffset;
	if(tmp>RMECDRESLUTON){this->ecdcorrect = tmp - RMECDRESLUTON;}
	else if(tmp<0){this->ecdcorrect = tmp + RMECDRESLUTON;}
	else{this->ecdcorrect = tmp;}
}	

void cRMMotor::UpdateMultiLoopData(void)
{
	this->MultiLoopData.multiecd 	= 0;
	this->MultiLoopData.loops		= 0;
	this->MultiLoopData.lastecd 	= this->MotorRec.ecd;
}

void cRMMotor::ResetMultiLoopData(void)
{
	this->MultiLoopData.multiecd += (this->MotorRec.ecd - this->MultiLoopData.lastecd);
	this->MultiLoopData.loops  	=  this->MultiLoopData.multiecd>>13;
	this->MultiLoopData.lastecd = this->MotorRec.ecd;
}

void cRMMotor::ConfigSpcltPara(float fs,float CutoffF, float revreduceratio, float wheelR)
{
	this->SpcltData.fs = fs;
	this->SpcltData.filterk = (2*PI*CutoffF/fs) / (1+2*PI*CutoffF/fs);
	this->SpcltData.reduction_ratio_rev = revreduceratio;
	this->SpcltData.wheelR = wheelR;
}
	
void cRMMotor::UpdateSpcltData(void)
{
	
	/*编码器插值经过低通滤波后转为角速度*/
	this->SpcltData.RadianVel = this->SpcltData.fs * this->SpcltData.reduction_ratio_rev * ( this->SpcltData.filterk * (0.0007669904f* (this->MotorRec.ecd - this->SpcltData.lastecd )) + (1-this->SpcltData.filterk) * this->SpcltData.RadianVel );
	/*由角速度计算线速度*/
	this->SpcltData.LineVel	  = this->SpcltData.RadianVel * this->SpcltData.wheelR;
	
	this->SpcltData.lastecd = this->MotorRec.ecd;
}

	
void cRMMotor::ResetSpcltData(void)
{
	this->SpcltData.RadianVel = 0.0f;
	this->SpcltData.LineVel = 0.0f;
	this->SpcltData.lastecd = this->MotorRec.ecd;
}