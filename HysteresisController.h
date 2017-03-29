// HysteresisController.h : 类HysCtrler、CurrentCtrler、SpeedCtrler声明文件
#ifndef HYSTERESIS_CONTROLLER_H
#define HYSTERESIS_CONTROLLER_H
class HysCtrler
/*-------------------------------------------------------
	@HysCtrler
	@描述：滞环控制器基类
-------------------------------------------------------*/
{
protected:
	bool State;  //控制器状态
	double UpperLimit;  //上限
	double LowerLimit;  //下限
	void Enable(void);  //开通方法
	void Disable(void);  //关短方法
public:
	HysCtrler(void);  //构造函数，用于初始化
	bool JudgeState(double vari);  //状态判断方法
	void SetLimit(double LLimit, double ULimit);  //设置上下限方法
	bool GetState();  // 获取当前控制器状态
};

class CurrentCtrler : public HysCtrler
/*------------------------------------------------------
	@CurrentCtrler
	@描述：电流滞环控制器类，继承自滞环控制器基类HysCtrler
-------------------------------------------------------*/
{
public:
	CurrentCtrler(Motor DCMotor);  //构造函数，用于初始化
};

class SpeedCtrler : public HysCtrler
/*------------------------------------------------------
	@SpeedCtrler
	@描述：转速滞环控制器类，继承自滞环控制器基类HysCtrler
-------------------------------------------------------*/
{
public:
	SpeedCtrler(Motor DCMotor);  //构造函数，用于初始化
	void SetBase(double BaseSpeed);  //设定转速方法
protected:
	double DeltaSpeed;  //转速滞环宽度
	double Base;  //转速设定值
};
#endif
