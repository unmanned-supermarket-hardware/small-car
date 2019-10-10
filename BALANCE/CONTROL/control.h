#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#include "malloc.h"
#include "cJSON.h"
#include "string.h"

// 区分上下车
#define CAR_UP 1 
#define CAR_DOWN 2
#define CAR_ID CAR_DOWN

#define PI 3.14159265
#define ZHONGZHI 0 
#define DIFFERENCE 100

#define GOALlDISTANCETOL 200  // 毫米，测距模块离  轨道左侧的距离
#define OUT_TURING_DISTANCE 1  //出弯阶段的检测距离
#define OUT_TURING_GAP 30 // 出弯时，前面测距模块的值  与 边距的差值
#define CORRECTION_Z_DISTANCE 15    // L1  L2  差多少开始矫正  平行
#define CORRECTION_Y_DISTANCE 15    //  小车边距矫正的误差


// 小车的运动状态  
#define STATE_STOP 0
#define STATE_STRAIGHT 1
#define STATE_TURN_RIGHT 2
#define STATE_TURN_LEFT 3




extern int timeAiwac;
extern int timeSys;
extern int timePrintf;


#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)      
#define L_PARAMETER            (1.0f)        



#define CORRECTION_Y 30  // 小车自校正  Y方向速度 
#define CORRECTION_Y_BIG 40  // 小车自校正  Y方向速度     		快速矫正 

#define CORRECTION_Z 30  // 小车自校正  Z方向速度 
#define CORRECTION_Z_BIG 40  // 小车自校正  Z方向速度      		快速矫正





// 全局存储  小车测距的数据
struct CarDistance {
	double distanceF;
	double distanceB;
	double distanceL1;
	double distanceL2;
	int leftPositionOK;   // 1: 自矫正ok ,0:自矫正未完成
	int start;  // 1: 已经开始测距        0：还未开始测距
}  ;

extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
extern struct CarDistance carDistance;
extern int intoCurve;
extern u8 jsonParseBuF[300], USART2_jsonParseBuF[300];
extern int VoltageFlag;


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
void  AiwacFeedback2Master(void);


#endif
