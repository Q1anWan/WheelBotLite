#ifndef 	LINKSOLVER_H
#define		LINKSOLVER_H

#include "main.h"
#include "arm_math.h"
#ifdef __cplusplus

enum eLinkStatue
{
	LINK_NORMAL	= 0,
	LINK_ERROR	= 1
};

class cLinkSolver
{
	protected:
	/*单位mm*/
	float L1 = 75.0f; 
	float L2 = 141.0f;
	float MotoDistance = 72.0f;
	float HalfMotoDistance = MotoDistance/2.0f;
	
	/*关节电机弧度*/
	float Theta2 = 0.0f;
	float Theta3 = 0.0f;
	
	/*极限值*/
	float Theta2Min = PI/2;
	float Theta3Max = PI/2;
	/*杆状态*/
	eLinkStatue LinkStatue = LINK_ERROR;
	
	/*倒立摆长度*/
	float PendulumLength = 0.0f;
	/*倒立摆角度*/
	float PendulumRadian = PI/2;
	/*倒立摆坐标*/
	float CoorC[2]={0.0f,0.0f};
	/*第二象限节点坐标*/
	float CoorB[2]={0.0f,0.0f};
	float U2 = 0.0f;
	/*第二象限节点坐标*/
	float CoorD[2]={0.0f,0.0f};
	float U3 = 0.0f;

	
	public:
	void	Resolve(void);
	void	VMCCal(float *F, float *T);
	
	void	SetRadLimit(float Theta3Max, float Theta2Min)
	{
		this->Theta3Max = Theta3Max;
		this->Theta2Min = Theta2Min;
	}
	
	uint8_t	InputLink(float Theta3, float Theta2);
	
	inline uint8_t GetLinkStatue(void)
	{return (uint8_t)this->LinkStatue;}
	
	inline float GetPendulumLen(void)
	{return PendulumLength;}
	
	inline float GetPendulumRadian(void)
	{return PendulumRadian;}
	
	inline void GetPendulumCoor(float* Coor) 
	{Coor[0]=this->CoorC[0];Coor[1]=this->CoorC[1];}
};

#endif
#endif