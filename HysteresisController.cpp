// HysteresisController.cpp : ������RKMotorSimu��CurrentController��SpeedController
#include "stdafx.h"
#include "parameters.h"
#include "HysteresisController.h"

HysCtrler::HysCtrler(void)
/*------------------------------------------------------------
	@HysCtrler(void)
	@���ܣ���HysCtrler�Ĺ��캯������ʼ״̬Ĭ�Ͽ�ͨ���ʹ�
------------------------------------------------------------*/
{
	Enable();
}

bool HysCtrler::JudgeState(double vari)
/*------------------------------------------------------------
	@bool JudgeState(double vari)
	@���ܣ�����ֵvari�жϿ�������ͨ���ضϡ�ά����״�����ص�ǰ״̬
	@������
	1. double vari: �������ĵ�ǰֵ
------------------------------------------------------------*/
{
	if (vari > UpperLimit) Disable();
	else if (vari < LowerLimit) Enable();
	return State;
}

void HysCtrler::Enable(void)
/*------------------------------------------------------------
	@void Enable(void)
	@���ܣ���ͨ���ʹ�
------------------------------------------------------------*/
{
	State = 1;
}

void HysCtrler::Disable(void)
/*------------------------------------------------------------
	@void Disable(void)
	@���ܣ��ضϹ��ʹ�
------------------------------------------------------------*/
{
	State = 0;
}

void HysCtrler::SetLimit(double LLimit, double ULimit)
/*------------------------------------------------------------
	@void SetLimit(double LLimit, double ULimit)
	@���ܣ����ÿ�����������
	@������
	1. double LLimit: �趨�Ŀ���������ֵ
	2. double ULimit: �趨�Ŀ���������ֵ
------------------------------------------------------------*/
{
	UpperLimit = ULimit;
	LowerLimit = LLimit;
}

bool HysCtrler::GetState()
/*------------------------------------------------------------
	@bool GetState()
	@���ܣ���ȡ��ǰ��������ͨ�ض�״̬
------------------------------------------------------------*/
{
	return State;
}

CurrentCtrler::CurrentCtrler(Motor M)
/*------------------------------------------------------------
	@CurrentCtrler(Motor M)
	@���ܣ���CurrentCtrler�Ĺ��캯�����ӵ������M�ж�ȡ����������
	@������
	1. Motor M: �������M
------------------------------------------------------------*/
{
	UpperLimit = M.GetParameter(I_h);
	LowerLimit = M.GetParameter(I_l);
}

SpeedCtrler::SpeedCtrler(Motor M)
/*------------------------------------------------------------
	@SpeedCtrler(Motor M)
	@���ܣ���SpeedCtrler�Ĺ��캯�����ӵ������M�ж�ȡת���趨ֵ��
	�ͻ���ȣ��������Ӧ��������
	@������
	1. Motor M: �������M
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
	@���ܣ��趨ת��ֵ
	@������
	1. double BaseSpeed�� �趨��ת��ֵ
------------------------------------------------------------*/
{
	Base = BaseSpeed;
	UpperLimit = Base + DeltaSpeed;
	LowerLimit = Base - DeltaSpeed;
	return;
}
