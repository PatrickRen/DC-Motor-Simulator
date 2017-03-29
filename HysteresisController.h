// HysteresisController.h : ��HysCtrler��CurrentCtrler��SpeedCtrler�����ļ�
#ifndef HYSTERESIS_CONTROLLER_H
#define HYSTERESIS_CONTROLLER_H
class HysCtrler
/*-------------------------------------------------------
	@HysCtrler
	@�������ͻ�����������
-------------------------------------------------------*/
{
protected:
	bool State;  //������״̬
	double UpperLimit;  //����
	double LowerLimit;  //����
	void Enable(void);  //��ͨ����
	void Disable(void);  //�ض̷���
public:
	HysCtrler(void);  //���캯�������ڳ�ʼ��
	bool JudgeState(double vari);  //״̬�жϷ���
	void SetLimit(double LLimit, double ULimit);  //���������޷���
	bool GetState();  // ��ȡ��ǰ������״̬
};

class CurrentCtrler : public HysCtrler
/*------------------------------------------------------
	@CurrentCtrler
	@�����������ͻ��������࣬�̳����ͻ�����������HysCtrler
-------------------------------------------------------*/
{
public:
	CurrentCtrler(Motor DCMotor);  //���캯�������ڳ�ʼ��
};

class SpeedCtrler : public HysCtrler
/*------------------------------------------------------
	@SpeedCtrler
	@������ת���ͻ��������࣬�̳����ͻ�����������HysCtrler
-------------------------------------------------------*/
{
public:
	SpeedCtrler(Motor DCMotor);  //���캯�������ڳ�ʼ��
	void SetBase(double BaseSpeed);  //�趨ת�ٷ���
protected:
	double DeltaSpeed;  //ת���ͻ����
	double Base;  //ת���趨ֵ
};
#endif
