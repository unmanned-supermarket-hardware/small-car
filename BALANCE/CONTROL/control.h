#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#include "malloc.h"
#include "cJSON.h"

  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
#define PI 3.14159265
#define ZHONGZHI 0 
#define DIFFERENCE 100
#define GOALlDISTANCETOL 1000  // ���ף�С��������  ������ľ���


// ȫ�ִ洢  С����������
struct CarDistance {
	double distanceF;
	double distanceL1;
	double distanceL2;
	u8 leftPositionOK;   // 1: �Խ���ok ,0:�Խ���δ���
	u8 start;  // 1: �Ѿ���ʼ���        0����δ��ʼ���
}  ;

extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
extern struct CarDistance carDistance;
extern int intoCurve;

int EXTI15_10_IRQHandler(void);
void Set_Pwm(int motor_a,int motor_b,int motor_c);
void Kinematic_Analysis(float Vx,float Vy,float Vz);
void Kinematic_Analysis_SpeedMode_Aiwac(float Vx,float Vy,float Vz);

void Kinematic_Analysis2(float Vx,float Vy,float Vz);
void Key(void);
void Xianfu_Pwm(int amplitude);
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C);
u8 Turn_Off( int voltage);
u32 myabs(long int a);
int Incremental_PI_A (int Encoder,int Target);
int Incremental_PI_B (int Encoder,int Target);
int Incremental_PI_C (int Encoder,int Target);

int Position_PID_A (int Encoder,int Target);
int Position_PID_B (int Encoder,int Target);
int Position_PID_C (int Encoder,int Target);
void Get_RC(u8 mode);
void Count_Velocity(void);
void CAN_N_Usart_Control(void);
void AiwacPositionCorrection(void);
void AiwacSupermarketCarControl(void);

#endif
