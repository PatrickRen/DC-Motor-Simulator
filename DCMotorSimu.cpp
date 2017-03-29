// DCMotorSimu.cpp : ����������ڵ�
#include "stdafx.h"
#include "parameters.h"
#include "HysteresisController.h"
#include "RongeKutta.h"
#include "PIDController.h"
#include <iostream>
#include <cstdio>
#include <cstdlib>
using namespace std;
ifstream InitFile;  //�����ļ�����
ofstream ResultFileHys;  //����ļ�����
ofstream ResultFilePID;

int main()
{
	cout << "DC Motor Simulator" << endl << endl;
	InitFile.open("d:\\parameters.txt");  //�������ļ�
	ResultFileHys.open("d:\\hysdata.txt");  //������ļ�
	ResultFilePID.open("d:\\piddata.txt");
	Motor DCMotor;  //�½��������DCMotor���ɹ��캯���Զ���ɵ��������ȡ
	RKMotorSimu Simu;  //�½��������Simu���ɹ��캯���Զ���ɷ��������ȡ
	CurrentCtrler ICtrler(DCMotor);  //�½���������������ICtrler���ɹ��캯���Զ���ɲ�������
	SpeedCtrler SCtrler(DCMotor);  //�½�ת�ٿ���������SCtrler���ɹ��캯���Զ���ɲ�������
	PIDCtrler PID(DCMotor,Simu);  ////�½�ת�ٿ���������SCtrler���ɹ��캯���Զ���ɲ�������
	PWMCtrler PWM(Simu);
	double Iq0 = 0, w0 = 0;  //�����ʼֵ�趨
	cout << "Simulation Time: " << Simu.GetSimuTime() << 's'<< endl<<endl;
	cout << "Simulating..." << endl << endl;
	cout << "PID Controller" << endl;
	for (int i = 0; (static_cast<double>(i)*Simu.GetStep()) <= Simu.GetSimuTime(); i++)  //����ѭ��
	{
		double TimeNow = static_cast<double>(i)*Simu.GetStep();  //��ǰ����ʱ��
		if ((static_cast<int>(TimeNow * 10.0 / 2.0)) % 2 == 0)  //0~0.2s��Ϊת��1, 0.2~0.4s��Ϊת��2
		{
			PID.SetGain(15, 35, 0.06);
			PID.SetSpeed(DCMotor.GetParameter(Speed_1));
		}
		else
		{
			PID.SetGain(10, 35, 0.03);
			PID.SetSpeed(DCMotor.GetParameter(Speed_2));
		}
		PWM.SetDutyCycle(PID.PIDCalc(w0));
		bool IGBT = ICtrler.JudgeState(Iq0) * PWM.JudgeState(w0);  //������ת�ٿ���������������õ����ʹ�״̬
		//������д������ļ�
		ResultFilePID << TimeNow << ' ' << Iq0 << ' ' << w0 << ' ' << ICtrler.GetState()<<' ' << PWM.JudgeState(PID.PIDCalc(w0)) << ' ' << IGBT << endl;
		//printf("Process: %.5fs\r", TimeNow);  //���������ʾ
		Simu.RKCalc(DCMotor, Iq0, w0, IGBT);  //������һ�μ���
		//��ȡ������
		Iq0 = Simu.GetIResult();
		w0 = Simu.GetSpeedResult();  
	}
	printf("Process: %.5fs\n\n", Simu.GetSimuTime());  //���������ʾ
	Iq0 = 0, w0 = 0;  //�����ʼֵ�趨
	cout << "Hysteresis Controller" << endl;
	for (int i = 0; (static_cast<double>(i)*Simu.GetStep()) <= Simu.GetSimuTime(); i++)  //����ѭ��
	{
		double TimeNow = static_cast<double>(i)*Simu.GetStep();  //��ǰ����ʱ��
		if ((static_cast<int>(TimeNow * 10.0 / 2.0)) % 2 == 0)  //0~0.2s��Ϊת��1, 0.2~0.4s��Ϊת��2
		{
			SCtrler.SetBase(DCMotor.GetParameter(Speed_1));
		}
		else
		{
			SCtrler.SetBase(DCMotor.GetParameter(Speed_2));
		}
		bool IGBT = ICtrler.JudgeState(Iq0) * SCtrler.JudgeState(w0);  //������ת�ٿ���������������õ����ʹ�״̬
		//������д������ļ�
		ResultFileHys << TimeNow << ' ' << Iq0 << ' ' << w0 << ' ' << ICtrler.GetState() << ' ' << SCtrler.GetState() << ' ' << IGBT << endl;
		//printf("Process: %.4fs\r", TimeNow);  //���������ʾ
		Simu.RKCalc(DCMotor, Iq0, w0, IGBT);  //������һ�μ���
		//��ȡ������
		Iq0 = Simu.GetIResult();
		w0 = Simu.GetSpeedResult();
	}
	printf("Process: %.5fs\n\n", Simu.GetSimuTime());
	InitFile.close();
	ResultFilePID.close();
	ResultFileHys.close();
    return 0;
}