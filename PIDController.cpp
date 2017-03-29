// RongeKutta.cpp : ������PIDCtrler
#include "stdafx.h"
#include "PIDController.h"
#include "RongeKutta.h"

PIDCtrler::PIDCtrler(Motor M, RKMotorSimu S)
/*------------------------------------------------------------
	@PIDCtrler(Motor M, RKMotorSimu S)
	@���ܣ���PIDCtrler�Ĺ��캯������ʼ��PID����������
	@������
	1. Motor M: �������M
	2. RKMotorSimu S: �������S
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
	@���ܣ�����PID���������ؼ�����
	@������
	1. double Kp: �趨�ı����ؼ�����
	2. double Ki: �趨�Ļ��ֿؼ�����
	3. double Kd: �趨��΢�ֿؼ�����
------------------------------------------------------------*/
{
	Gain_P = Kp;
	Gain_I = Ki;
	Gain_D = Kd;
}

void PIDCtrler::SetSpeed(double Speed)
/*------------------------------------------------------------
	@void SetSpeed(double Speed)
	@���ܣ�����ת��
	@������
	1. double Speed: ���õ�ת��ֵ
------------------------------------------------------------*/
{
	SpeedSet = Speed;
}

double PIDCtrler::PIDCalc(double SpeedNow)
/*------------------------------------------------------------
	@double PIDCalc(double SpeedNow)
	@���ܣ�PID������������㣬���PWM��ռ�ձ�
	@������
	1. double SpeedNow: ��ǰת��ֵ
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
	@���ܣ���PWMCtrler�Ĺ��캯������ʼ��PWM����
	@������
	1. RKMotorSimu S: �������S
------------------------------------------------------------*/
{
	DutyCycle = 1;
	Period = S.GetStep()*10.0;
	State = 1;
}

bool PWMCtrler::JudgeState(double TimeNow)
/*------------------------------------------------------------
	@bool JudgeState(double vari)
	@���ܣ����ݵ�ǰʱ���ж�PWM��Ϊ0��1�����ص�ǰ״̬
	@������
	1. double TimeNow: ��ǰ����ʱ��
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
	@���ܣ�����PWMռ�ձ�
	@������
	1. double d: ���õ�ռ�ձ�ֵ
------------------------------------------------------------*/
{
	DutyCycle = d;
}