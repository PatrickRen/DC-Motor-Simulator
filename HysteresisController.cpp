// HysteresisController.cpp : 定义类RKMotorSimu、CurrentController、SpeedController
#include "stdafx.h"
#include "parameters.h"
#include "HysteresisController.h"

HysCtrler::HysCtrler(void)
/*------------------------------------------------------------
	@HysCtrler(void)
	@功能：类HysCtrler的构造函数，初始状态默认开通功率管
------------------------------------------------------------*/
{
	Enable();
}

bool HysCtrler::JudgeState(double vari)
/*------------------------------------------------------------
	@bool JudgeState(double vari)
	@功能：根据值vari判断控制器开通、关断、维持现状，返回当前状态
	@参数：
	1. double vari: 控制量的当前值
------------------------------------------------------------*/
{
	if (vari > UpperLimit) Disable();
	else if (vari < LowerLimit) Enable();
	return State;
}

void HysCtrler::Enable(void)
/*------------------------------------------------------------
	@void Enable(void)
	@功能：开通功率管
------------------------------------------------------------*/
{
	State = 1;
}

void HysCtrler::Disable(void)
/*------------------------------------------------------------
	@void Disable(void)
	@功能：关断功率管
------------------------------------------------------------*/
{
	State = 0;
}

void HysCtrler::SetLimit(double LLimit, double ULimit)
/*------------------------------------------------------------
	@void SetLimit(double LLimit, double ULimit)
	@功能：设置控制器上下限
	@参数：
	1. double LLimit: 设定的控制器下限值
	2. double ULimit: 设定的控制器上限值
------------------------------------------------------------*/
{
	UpperLimit = ULimit;
	LowerLimit = LLimit;
}

bool HysCtrler::GetState()
/*------------------------------------------------------------
	@bool GetState()
	@功能：读取当前控制器开通关断状态
------------------------------------------------------------*/
{
	return State;
}

CurrentCtrler::CurrentCtrler(Motor M)
/*------------------------------------------------------------
	@CurrentCtrler(Motor M)
	@功能：类CurrentCtrler的构造函数，从电机对象M中读取电流上下限
	@参数：
	1. Motor M: 电机对象M
------------------------------------------------------------*/
{
	UpperLimit = M.GetParameter(I_h);
	LowerLimit = M.GetParameter(I_l);
}

SpeedCtrler::SpeedCtrler(Motor M)
/*------------------------------------------------------------
	@SpeedCtrler(Motor M)
	@功能：类SpeedCtrler的构造函数，从电机对象M中读取转速设定值、
	滞环宽度，并求出相应的上下限
	@参数：
	1. Motor M: 电机对象M
------------------------------------------------------------*/
{
	Base = M.GetParameter(Speed_1);
	DeltaSpeed = M.GetParameter(Delta);
	UpperLimit = Base + DeltaSpeed;
	LowerLimit = Base - DeltaSpeed;
}

void SpeedCtrler::SetBase(double BaseSpeed)
/*------------------------------------------------------------
	@void SetBase(double BaseSpeed)
	@功能：设定转速值
	@参数：
	1. double BaseSpeed： 设定的转速值
------------------------------------------------------------*/
{
	Base = BaseSpeed;
	UpperLimit = Base + DeltaSpeed;
	LowerLimit = Base - DeltaSpeed;
	return;
}
