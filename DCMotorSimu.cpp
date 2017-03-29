// DCMotorSimu.cpp : 仿真程序的入口点
#include "stdafx.h"
#include "parameters.h"
#include "HysteresisController.h"
#include "RongeKutta.h"
#include "PIDController.h"
#include <iostream>
#include <cstdio>
#include <cstdlib>
using namespace std;
ifstream InitFile;  //输入文件对象
ofstream ResultFileHys;  //输出文件对象
ofstream ResultFilePID;

int main()
{
	cout << "DC Motor Simulator" << endl << endl;
	InitFile.open("d:\\parameters.txt");  //打开输入文件
	ResultFileHys.open("d:\\hysdata.txt");  //打开输出文件
	ResultFilePID.open("d:\\piddata.txt");
	Motor DCMotor;  //新建电机对象DCMotor，由构造函数自动完成电机参数读取
	RKMotorSimu Simu;  //新建仿真对象Simu，由构造函数自动完成仿真参数读取
	CurrentCtrler ICtrler(DCMotor);  //新建电流控制器对象ICtrler，由构造函数自动完成参数设置
	SpeedCtrler SCtrler(DCMotor);  //新建转速控制器对象SCtrler，由构造函数自动完成参数设置
	PIDCtrler PID(DCMotor,Simu);  ////新建转速控制器对象SCtrler，由构造函数自动完成参数设置
	PWMCtrler PWM(Simu);
	double Iq0 = 0, w0 = 0;  //仿真初始值设定
	cout << "Simulation Time: " << Simu.GetSimuTime() << 's'<< endl<<endl;
	cout << "Simulating..." << endl << endl;
	cout << "PID Controller" << endl;
	for (int i = 0; (static_cast<double>(i)*Simu.GetStep()) <= Simu.GetSimuTime(); i++)  //仿真循环
	{
		double TimeNow = static_cast<double>(i)*Simu.GetStep();  //当前仿真时刻
		if ((static_cast<int>(TimeNow * 10.0 / 2.0)) % 2 == 0)  //0~0.2s设为转速1, 0.2~0.4s设为转速2
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
		bool IGBT = ICtrler.JudgeState(Iq0) * PWM.JudgeState(w0);  //电流、转速控制器进行与运算得到功率管状态
		//计算结果写入输出文件
		ResultFilePID << TimeNow << ' ' << Iq0 << ' ' << w0 << ' ' << ICtrler.GetState()<<' ' << PWM.JudgeState(PID.PIDCalc(w0)) << ' ' << IGBT << endl;
		//printf("Process: %.5fs\r", TimeNow);  //仿真进度显示
		Simu.RKCalc(DCMotor, Iq0, w0, IGBT);  //进行下一次计算
		//读取计算结果
		Iq0 = Simu.GetIResult();
		w0 = Simu.GetSpeedResult();  
	}
	printf("Process: %.5fs\n\n", Simu.GetSimuTime());  //仿真进度显示
	Iq0 = 0, w0 = 0;  //仿真初始值设定
	cout << "Hysteresis Controller" << endl;
	for (int i = 0; (static_cast<double>(i)*Simu.GetStep()) <= Simu.GetSimuTime(); i++)  //仿真循环
	{
		double TimeNow = static_cast<double>(i)*Simu.GetStep();  //当前仿真时刻
		if ((static_cast<int>(TimeNow * 10.0 / 2.0)) % 2 == 0)  //0~0.2s设为转速1, 0.2~0.4s设为转速2
		{
			SCtrler.SetBase(DCMotor.GetParameter(Speed_1));
		}
		else
		{
			SCtrler.SetBase(DCMotor.GetParameter(Speed_2));
		}
		bool IGBT = ICtrler.JudgeState(Iq0) * SCtrler.JudgeState(w0);  //电流、转速控制器进行与运算得到功率管状态
		//计算结果写入输出文件
		ResultFileHys << TimeNow << ' ' << Iq0 << ' ' << w0 << ' ' << ICtrler.GetState() << ' ' << SCtrler.GetState() << ' ' << IGBT << endl;
		//printf("Process: %.4fs\r", TimeNow);  //仿真进度显示
		Simu.RKCalc(DCMotor, Iq0, w0, IGBT);  //进行下一次计算
		//读取计算结果
		Iq0 = Simu.GetIResult();
		w0 = Simu.GetSpeedResult();
	}
	printf("Process: %.5fs\n\n", Simu.GetSimuTime());
	InitFile.close();
	ResultFilePID.close();
	ResultFileHys.close();
    return 0;
}