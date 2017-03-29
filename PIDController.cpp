// RongeKutta.cpp : 定义类PIDCtrler
#include "stdafx.h"
#include "PIDController.h"
#include "RongeKutta.h"

PIDCtrler::PIDCtrler(Motor M, RKMotorSimu S)
/*------------------------------------------------------------
	@PIDCtrler(Motor M, RKMotorSimu S)
	@功能：类PIDCtrler的构造函数，初始化PID控制器参数
	@参数：
	1. Motor M: 电机对象M
	2. RKMotorSimu S: 仿真对象S
------------------------------------------------------------*/
{
	Gain_P = 1;
	Gain_I = 0;
	Gain_D = 0;
	SpeedSet = M.GetParameter(Speed_2);
	PreError = 0;
	Integral = 0;
	dt = S.GetStep();
}

void PIDCtrler::SetGain(double Kp, double Ki, double Kd)
/*------------------------------------------------------------
	@void SetGain(double Kp, double Ki, double Kd)
	@功能：设置PID控制器各控件增益
	@参数：
	1. double Kp: 设定的比例控件增益
	2. double Ki: 设定的积分控件增益
	3. double Kd: 设定的微分控件增益
------------------------------------------------------------*/
{
	Gain_P = Kp;
	Gain_I = Ki;
	Gain_D = Kd;
}

void PIDCtrler::SetSpeed(double Speed)
/*------------------------------------------------------------
	@void SetSpeed(double Speed)
	@功能：设置转速
	@参数：
	1. double Speed: 设置的转速值
------------------------------------------------------------*/
{
	SpeedSet = Speed;
}

double PIDCtrler::PIDCalc(double SpeedNow)
/*------------------------------------------------------------
	@double PIDCalc(double SpeedNow)
	@功能：PID控制器输出计算，输出PWM波占空比
	@参数：
	1. double SpeedNow: 当前转速值
------------------------------------------------------------*/
{
	double Error = SpeedSet - SpeedNow;
	Integral += Error*dt;
	double Deri = (Error - PreError) / dt;
	double output = Gain_P*Error + Gain_I*Integral + Gain_D*Deri;
	PreError = Error;
	double DutyCycle = output / 144.0;
	if (DutyCycle > 1) DutyCycle = 1;
	if (DutyCycle < 0) DutyCycle = 0;
	return DutyCycle;
}

PWMCtrler::PWMCtrler(RKMotorSimu S)
/*------------------------------------------------------------
	@PWMCtrler(RKMotorSimu S)
	@功能：类PWMCtrler的构造函数，初始化PWM参数
	@参数：
	1. RKMotorSimu S: 仿真对象S
------------------------------------------------------------*/
{
	DutyCycle = 1;
	Period = S.GetStep()*10.0;
	State = 1;
}

bool PWMCtrler::JudgeState(double TimeNow)
/*------------------------------------------------------------
	@bool JudgeState(double vari)
	@功能：根据当前时间判断PWM波为0或1，返回当前状态
	@参数：
	1. double TimeNow: 当前仿真时间
------------------------------------------------------------*/
{
	int N = static_cast<int> (TimeNow / Period);
	double TimeInPeriod = TimeNow - static_cast<double>(N)*Period;
	if (TimeInPeriod < Period*DutyCycle) State = 1;
	else State = 0;
	return State;
}

void PWMCtrler::SetDutyCycle(double d)
/*------------------------------------------------------------
	@void SetDutyCycle(double d)
	@功能：设置PWM占空比
	@参数：
	1. double d: 设置的占空比值
------------------------------------------------------------*/
{
	DutyCycle = d;
}