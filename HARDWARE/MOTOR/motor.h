#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#define PWMD   TIM8->CCR3  
#define PWMC   TIM8->CCR4  
#define PWMB   TIM8->CCR1 
#define PWMA   TIM8->CCR2 
#define INA1   PCout(12)  
#define INB1   PDout(2)  
#define INC1   PCout(5)  
#define IND1   PCout(4)  

#define INA2   PBout(5)  
#define INB2   PBout(4)  
#define INC2   PBout(1)  
#define IND2   PBout(0)  
#define EN     PAin(12)  
void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
#endif
