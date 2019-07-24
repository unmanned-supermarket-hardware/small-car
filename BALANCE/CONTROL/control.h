#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#include "malloc.h"
#include "cJSON.h"
#include "string.h"

  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#define PI 3.14159265
#define ZHONGZHI 0 
#define DIFFERENCE 100
#define GOALlDISTANCETOL 70  // 毫米，小车中心离  轨道左侧的距离

// 小车的运动状态  
#define STATE_STOP 0
#define STATE_STRAIGHT 1
#define STATE_TURN_RIGHT 2
#define STATE_TURN_LEFT 3


#define CORRECTION_Y 40  // 小车自校正  Y方向速度 
#define CORRECTION_Z 30  // 小车自校正  Z方向速度 

#define CORRECTION_Y_BIG 40  // 小车自校正  Y方向速度     		快速矫正 
#define CORRECTION_Z_BIG 40  // 小车自校正  Z方向速度      		快速矫正


#define CORRECTION_Z_DISTANCE 6    // L1  L2  差多少开始矫正  平行


// 全局存储  小车测距的数据
struct CarDistance {
	double distanceF;
	double distanceL1;
	double distanceL2;
	int leftPositionOK;   // 1: 自矫正ok ,0:自矫正未完成
	int start;  // 1: 已经开始测距        0：还未开始测距
}  ;

extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
extern struct CarDistance carDistance;
extern int intoCurve;
extern u8 jsonParseBuF[300], USART2_jsonParseBuF[300];



extern unsigned char const crc8_tab[256];
unsigned	char crc8_calculate(unsigned char * ucPtr, unsigned char ucLen) ;


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

\
void AiwacPositionCorrection(void);
void AiwacSupermarketCarControl(void);
void AiwacParseDistanceJson(void);
void AiwacParseMOVEOrder(void);
void  AiwacSendState2Master(void);
double myabs_double(double a);


#endif
