// PIDController.h : 类PIDCtrler声明文件
#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include "parameters.h"
#include "RongeKutta.h"
class PIDCtrler
/*-------------------------------------------------------
	@PIDCtrler
	@描述：PID控制器类
-------------------------------------------------------*/
{
public:
	PIDCtrler(Motor M, RKMotorSimu S);  //构造函数，用于初始化
	void SetGain(double Kp, double Ki, double Kd);  //设置PID控制器增益方法
	void SetSpeed(double Speed);  //设定转速值方法
	double PIDCalc(double SpeedNow);  //PID控制器计算PWM占空比方法
private:
	double Gain_P;  //比例控件增益
	double Gain_I;  //积分控件增益
	double Gain_D;  //微分控件增益
	double SpeedSet;  //转速设定值
	double PreError;  //误差累积
	double Integral;  //积分项
	double dt;  //时间微分元
};

class PWMCtrler
/*-------------------------------------------------------
	@PIDCtrler
	@描述：PWM波发生类
-------------------------------------------------------*/
{
public:
	PWMCtrler(RKMotorSimu S);  //构造函数，用于初始化
	bool JudgeState(double TimeNow);  //状态判断方法
	void SetDutyCycle(double d);  //设定占空比方法
private:
	bool State;  //PWM状态
	double DutyCycle;  //占空比
	double Period;  //PWM周期
};
#endif