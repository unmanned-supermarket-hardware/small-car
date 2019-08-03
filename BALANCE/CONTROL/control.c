#include "control.h"	
#include "filter.h"	
#include <math.h>


u8 Flag_Target,Flag_Change;                             //相关标志位
u8 temp1;                                               //临时变量
float Voltage_Count,Voltage_All;  //电压采样相关变量
float Gyro_K=0.00;       //陀螺仪比例系数
int j;


//AIWAC  无人超市
struct CarDistance carDistance;  // 存储 三个 测量的距离
float AIWAC_R_vehicle = 310;  //小车半径，单位：mm
float AIWAC_R_gui = 400;  // 轨道半径，单位：mm
float AIWAC_Move_X  = 0, AIWAC_Move_Y = 0, AIWAC_Move_Z = 0;   //三轴角度和XYZ轴目标速度
float AIWAC_MOVE_Xtemp = 0;  // 保存  主控下发的 X 速度
float AIWAC_V_sum = 100;  // 当前的速度，单位  mm/s
int AIWACTuringTime = 0;     
  // 转弯的时间控制
int moveState = STATE_STOP; // 小车运动 状态，  0：停止 （刚上电  或 刚出弯道），  1：  直走  2： 顺时针转  3：逆时针转 
int AIWACStop = 0;		//当三方距离  危险时，紧急停止   重新上电才行








unsigned char const crc8_tab[256]  = 
{ 
	0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D, 
	0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D, 
	0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD, 
	0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD, 
	0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA, 
	0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A, 
	0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A, 
	0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A, 
	0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4, 
	0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4, 
	0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44, 
	0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34, 
	0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63, 
	0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
	0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83, 
	0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3 
}; 


unsigned	char crc8_calculate(unsigned char * ucPtr, unsigned char ucLen) 
{ 
	unsigned char ucCRC8 = 0;

	while(ucLen--)
	{ 
		ucCRC8 = crc8_tab[ucCRC8^(*ucPtr++)]; 
	} 
	return (~ucCRC8)&0xFF; 

} 



/**************************************************************************
函数功能：小车运动数学模型
入口参数：X Y Z 三轴速度或者位置
返回  值：无
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
			Target_A   = Vx + L_PARAMETER*Vz+gyro[2]*Gyro_K;
			Target_B   = -X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz+gyro[2]*Gyro_K;
			Target_C   = -X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz+gyro[2]*Gyro_K;
			
}


/**************************************************************************
函数功能：	小车运动数学模型，将 X Y方向和 Z(自旋的速度) 单位mm/s，转换为三个
			轮的线速度，再转化为  三个轮的 在5ms内的脉冲数（频率）
入口参数：X Y Z 三轴速度   mm/s
返回  值：无
**************************************************************************/
void Kinematic_Analysis_SpeedMode_Aiwac(float Vx,float Vy,float Vz)
{
	float Target_A_Speed, Target_B_Speed, Target_C_Speed;

	// 上车需要反过来，Y 需要反
	if(CAR_ID == CAR_UP)
	{
		Vy = -Vy;
	}
	
	Target_A_Speed   = Vx + L_PARAMETER*Vz;
	Target_B_Speed   = -X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz;
	Target_C_Speed   = -X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz;


	/**************************************************************************
	 经计算  车轮转一圈  需要	1800 个脉冲,一圈 周长 R直径 = 127mm,C = 127mm * PI
	 则一个脉冲的  对应 轮转动的  距离  为    	C/1800，即  127mm*PI/1800
	 目标 变量 Target_A 代表 在 10ms  内 电机A转动的脉冲，设为 M
	 则 每秒      100M个脉冲，对应的 距离是     100M*127mm*PI/1800
	 即 速度  100M*127mm*PI/1800/s = 127*M*PI/ 18  mm/s
	 其中 M 就是 我们需要赋值给Target_A的值

	 现在目标速度 V mm/s = 127*M*PI/ 18  mm/s,   那么 M = V*18/(127*PI)
	**************************************************************************/

	Target_A = Target_A_Speed*18/(127*PI);
	Target_B = Target_B_Speed*18/(127*PI);
	Target_C = Target_C_Speed*18/(127*PI);

}

/**************************************************************************
函数功能：获取位置控制过程速度值
入口参数：X Y Z 三轴位置变化量
返回  值：无
**************************************************************************/
void Kinematic_Analysis2(float Vx,float Vy,float Vz)
{
			Rate_A   = Vx + L_PARAMETER*Vz;
			Rate_B   =-X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz;
			Rate_C   =-X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz;
}
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	 if(INT==0)		
	{     
		  EXTI->PR=1<<15;                                                      //清除LINE5上的中断标志位  		
		  Flag_Target=!Flag_Target;
		  if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //给主函数提供50ms的精准延时
			 }
		  if(Flag_Target==1)                                                  //5ms读取一次陀螺仪和加速度计的值
			{
			if(Usart_Flag==0&&PS2_ON_Flag==0&&Usart_ON_Flag==1)  memcpy(rxbuf,Urxbuf,8*sizeof(u8));	//如果解锁了串口控制标志位，进入串口控制模式
			Read_DMP();                                                           //===更新姿态		
			Key();//扫描按键变化	
			return 0;	                                               
			}                                                                      //===10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据

		  
			Encoder_A=Read_Encoder(2)/20;                                          //===读取编码器的值
			Position_A+=Encoder_A;                                                 //===积分得到位置 
			Encoder_B=Read_Encoder(3)/20;                                          //===读取编码器的值
			Position_B+=Encoder_B;                                                 //===积分得到位置 
			Encoder_C=Read_Encoder(4)/20;                                          //===读取编码器的值
			Position_C+=Encoder_C;                                                 //===积分得到位置     
	  	Read_DMP();                                                            //===更新姿态	
  		Led_Flash(100);                                                        //===LED闪烁;常规模式 1s改变一次指示灯的状态	
  		
		Voltage_All+=Get_battery_volt();                                       //多次采样累积
		if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//求平均值 获取电池电压	       
		 if(Turn_Off(Voltage)!=0)               //===如果电池电压存在异常
		 { 	
			return;
		 }

		 
		  //if(PS2_KEY==4)PS2_ON_Flag=1,CAN_ON_Flag=0,Usart_ON_Flag=0;						
		  //if(CAN_ON_Flag==1||Usart_ON_Flag==1||PS2_ON_Flag==1) CAN_N_Usart_Control();       //接到串口或者CAN遥控解锁指令之后，使能CAN和串口控制输入
		  //if(CAN_ON_Flag==0&&Usart_ON_Flag==0&&PS2_ON_Flag==0)  Get_RC(Run_Flag);  //===串口和CAN控制都未使能，则接收蓝牙遥控指
		 
	



		Motor_A=Incremental_PI_A(Encoder_A,Target_A);                         //===速度闭环控制计算电机A最终PWM
		Motor_B=Incremental_PI_B(Encoder_B,Target_B);                         //===速度闭环控制计算电机B最终PWM
		Motor_C=Incremental_PI_C(Encoder_C,Target_C);                         //===速度闭环控制计算电机C最终PWM
				
		 Xianfu_Pwm(6500);                     //===PWM限幅
		 Set_Pwm(Motor_B,Motor_A,Motor_C);     //===赋值给PWM寄存器  
		 
 	}
	 return 0;	 
} 

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c)
{
   	if(motor_a<0)			INA2=1,			INA1=0;
			else 	          INA2=0,			INA1=1;
		PWMA=myabs(motor_a);
	
		if(motor_b<0)			INB2=1,			INB1=0;
		else 	            INB2=0,			INB1=1;
		PWMB=myabs(motor_b);
	
		if(motor_c<0)			INC2=1,			INC1=0;
		else 	            INC2=0,			INC1=1;
		PWMC=myabs(motor_c);
	

}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{	
    if(Motor_A<-amplitude) Motor_A=-amplitude;	
		if(Motor_A>amplitude)  Motor_A=amplitude;	
	  if(Motor_B<-amplitude) Motor_B=-amplitude;	
		if(Motor_B>amplitude)  Motor_B=amplitude;		
	  if(Motor_C<-amplitude) Motor_C=-amplitude;	
		if(Motor_C>amplitude)  Motor_C=amplitude;		
}
/**************************************************************************
函数功能：位置PID控制过程中速度的设置
入口参数：无、幅值
返回  值：无
**************************************************************************/
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C)
{	
    if(Motor_A<-amplitude_A) Motor_A=-amplitude_A;	//位置控制模式中，A电机的运行速度
		if(Motor_A>amplitude_A)  Motor_A=amplitude_A;	  //位置控制模式中，A电机的运行速度
	  if(Motor_B<-amplitude_B) Motor_B=-amplitude_B;	//位置控制模式中，B电机的运行速度
		if(Motor_B>amplitude_B)  Motor_B=amplitude_B;		//位置控制模式中，B电机的运行速度
	  if(Motor_C<-amplitude_C) Motor_C=-amplitude_C;	//位置控制模式中，C电机的运行速度
		if(Motor_C>amplitude_C)  Motor_C=amplitude_C;		//位置控制模式中，C电机的运行速度
}
/**************************************************************************
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double(100); 
	if(tmp==2)Flag_Show=!Flag_Show;//双击控制显示模式                  
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<2000||EN==0)//电池电压低于20V关闭电机
			{	                                                
      temp=1;      
      PWMA=0;
      PWMB=0;
      PWMC=0;						
      }
			else
      temp=0;
      return temp;			
}

/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
函数功能：绝对值函数
入口参数：double
返回  值：unsigned int
**************************************************************************/
double myabs_double(double a)
{ 		   
	  double temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;  // 
	 Bias=Encoder-Target;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
int Incremental_PI_C (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
/**************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
int Position_PID_A (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 if(Integral_bias>100000)Integral_bias=10000;
	 if(Integral_bias<-100000)Integral_bias=-10000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
int Position_PID_B (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 if(Integral_bias>100000)Integral_bias=10000;
	 if(Integral_bias<-100000)Integral_bias=-10000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
int Position_PID_C (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 if(Integral_bias>100000)Integral_bias=10000;
	 if(Integral_bias<-100000)Integral_bias=-10000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}
/**************************************************************************
函数功能：通过串口指令对小车进行遥控
入口参数：串口指令
返回  值：无
当前版本说明：转90°自动停止
**************************************************************************/
#define STOP 0
#define MOVING_STRAIGHT 1
#define TURNING 2

float V_sum = 300;  // 当前的速度，单位  mm/s
u8 state_flag = 0;    //目前状态，有STOP，MOVING_STRAIGHT，TURNING 三种取值
u8 last_state_flag = 0;
u8 last_flag_direction = 0;
//-------------------------
void Get_RC(u8 mode)
{
	float step=0.25;  //设置速度控制步进值。
	int yaw = Yaw;  // 传感器测的偏角
	int angle = Yaw;
	int pitch = Pitch;
	int roll = Roll;
	u8 Flag_Move=1;
	float R_vehicle = 310;  //小车半径，单位：mm
	float R_gui = 400;  // 轨道半径，单位：mm
	
	char strTemp[64];
	  if(mode==0)//速度
		{	

			 switch(Flag_Direction)   //方向控制
			 {
				 case 1:  Move_X=0;           Move_Y=V_sum;  Flag_Move=1;               break;
				 case 2:  
						Move_X = -V_sum;
					 	Move_Y = 0;	
						Move_Z = (R_vehicle/R_gui)*(-Move_X);	
						Flag_Move=1;
						state_flag = TURNING;
						//sprintf(strTemp, "case 2	Move_X == %f\r\n",Move_X);
						//usart1_sendString(strTemp,strlen(strTemp));	
				 		break;
				 case 5:  Move_X=0;           Move_Y=0;	 Move_Z = 0;Flag_Move=0; state_flag =STOP;               break;  //stop

				 default: 
							if(state_flag == TURNING)   //正在转，判断是否转完
							{

								//debug
								
								//顺时针/右转，Yaw--
								if(angle >-90) //没转完，接着转
								{

									Move_X = -V_sum;
									Move_Y = 0;	
									Move_Z = (R_vehicle/R_gui)*(-Move_X);	
									Flag_Move=1;
									state_flag = TURNING;
									
								}
								else
								{
									//sprintf(strTemp, "yaw>90\r\n");
									//usart1_sendString(strTemp,strlen(strTemp));	
									Move_X = 0;
									Move_Y=0;
									Move_Z=0;
									state_flag = STOP;
									Flag_Move=0;
								}
								
							}
//							else
//							{
//									Move_X = 0;
//									Move_Y=0;
//									Move_Z=0;
//									state_flag = STOP;
//									Flag_Move=0;
//							}
		     }


			 //  一定频率时间会进入  Get_RC ，不是每次都检测到有  外界控制
			if(Flag_Move==0)		//如果无方向控制指令	 ，检查转向控制状态
			{	
				if(Flag_Left==1)       Move_Z-=step,Gyro_K=0;    //左自旋   
				else if(Flag_Right==1) Move_Z+=step,Gyro_K=0;    //右自旋		
				else 		               Move_Z=0,Gyro_K=0.00;    //停止
			}	
			sprintf(strTemp, "default	pitch ==%d , roll == %d ,angle == %d\r\n",pitch,roll,angle);
			//usart1_sendString(strTemp,strlen(strTemp));	

			
			
				//if(Flag_Move==1)	Flag_Left=0,Flag_Right=0,Move_Z=0;

			/*
				if(Move_X<-RC_Velocity) Move_X=-RC_Velocity;	   //速度控制限幅
				if(Move_X>RC_Velocity)  Move_X=RC_Velocity;	     
				if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
				if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
				if(Move_Z<-RC_Velocity) Move_Z=-RC_Velocity;	
				if(Move_Z>RC_Velocity)  Move_Z=RC_Velocity;	 
			*/

		 }/*
		 else if(mode==1)//位置模式
		{	
				 switch(Flag_Direction)   //方向控制
				 {
				 case 1:  Move_Y+=RC_Position; Flag_Change=1;break;
				 case 2:  Move_X+=RC_Position; Flag_Change=2; Move_Y+=RC_Position; break;
				 case 3:  Move_X+=RC_Position; Flag_Change=3;break;
				 case 4:  Move_X+=RC_Position; Flag_Change=4;Move_Y-=RC_Position;break;
				 case 5:  Move_Y-=RC_Position; Flag_Change=5;break;
				 case 6:  Move_X-=RC_Position; Flag_Change=6;Move_Y-=RC_Position; break;
				 case 7:  Move_X-=RC_Position; Flag_Change=7; break;
				 case 8:  Move_X-=RC_Position; Flag_Change=8;Move_Y+=RC_Position;break;			 
				 case 9:  Move_Z-=RC_Position; Flag_Change=9; break;
				 case 10: Move_Z+=RC_Position; Flag_Change=10;break;			 
				 default: break;	 
			 }
	 	}*/


		//Kinematic_Analysis(Move_X,Move_Y,Move_Z);//得到控制目标值，进行运动学分析
		Kinematic_Analysis_SpeedMode_Aiwac(Move_X,Move_Y,Move_Z);//得到控制目标值，进行运动学分析
}
/**************************************************************************
函数功能：每个电机位置控制过程速度计算
入口参数：无
返回  值：无
**************************************************************************/
void Count_Velocity(void)
{
	static double Last_Target_X,Last_Target_Y,Last_Target_Z,Divider;
	double Bias_X,Bias_Y,Bias_Z;
	Bias_X=(Move_X-Last_Target_X);  //求X轴位移量
	Bias_Y=(Move_Y-Last_Target_Y);	//求Y轴位移量
	Bias_Z=(Move_Z-Last_Target_Z);	//求Z轴位移量
	if(Bias_X!=0||Bias_Y!=0||Bias_Z!=0)Divider=sqrt(Bias_X*Bias_X+Bias_Y*Bias_Y+Bias_Z*Bias_Z);
	if(Bias_X!=0||Bias_Y!=0||Bias_Z!=0) Kinematic_Analysis2(Bias_X,Bias_Y,Bias_Z);

	Xianfu_Velocity(RC_Velocity*myabs(Rate_A)/Divider,RC_Velocity*myabs(Rate_B)/Divider,RC_Velocity*myabs(Rate_C)/Divider); 
	Last_Target_X=Move_X;   //保存X轴上一次的位置信息，便于调用
	Last_Target_Y=Move_Y;   //保存Y轴上一次的位置信息，便于调用
	Last_Target_Z=Move_Z;   //保存Z轴上一次的位置信息，便于调用
}
/**************************************************************************
函数功能：接收CAN或者串口控制指令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void CAN_N_Usart_Control(void)
{
   int flag_X, flag_Y,flag_Z;
	 int Yuzhi=20;
	 int LX,LY,RX;
	 if(Run_Flag==0)//速度模式
	 {
		  if(CAN_ON_Flag==1||Usart_ON_Flag==1) 
		{
				 if((rxbuf[7]&0x04)==0)flag_X=1;  else flag_X=-1;  //方向控制位
				 if((rxbuf[7]&0x02)==0)flag_Y=1;  else flag_Y=-1;  //方向控制位
				 if((rxbuf[7]&0x01)==0)flag_Z=1;  else flag_Z=-1;  //方向控制位
				 Move_X=flag_X*(rxbuf[1]*256+rxbuf[2]);
				 Move_Y=flag_Y*(rxbuf[3]*256+rxbuf[4]);	
				 Move_Z=flag_Z*(rxbuf[5]*256+rxbuf[6]);	
				 if(rxbuf[0]==1)Kinematic_Analysis(Move_X,Move_Y,Move_Z),Gyro_K=0;    //进行运动学分析
				 if(rxbuf[0]==2)Target_A=Move_X,Target_B=Move_Y,Target_C=Move_Z;      //单独对每个电机进行控制
		 }
			else if (PS2_ON_Flag==1)
			{
				LX=PS2_LX-128;
				LY=PS2_LY-128;
				RX=PS2_RX-128;
				if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
				if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
				if(RX>-Yuzhi&&RX<Yuzhi)RX=0;
				 Move_X=LX*RC_Velocity/200;
				 Move_Y=-LY*RC_Velocity/200;	
				 Move_Z=-RX*RC_Velocity/200;		 
//		 if(Move_X<-RC_Velocity) Move_X=-RC_Velocity;	   //速度控制限幅
//		if(Move_X>RC_Velocity)  Move_X=RC_Velocity;	     
//		if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
//		if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
//		if(Move_Z<-RC_Velocity) Move_Z=-RC_Velocity;	
//		if(Move_Z>RC_Velocity)  Move_Z=RC_Velocity;	 
				Kinematic_Analysis(Move_X,Move_Y,Move_Z),Gyro_K=0;    //进行运动学分析 
			}
	 }
}


/**************************************************************************
函数功能：		小车自矫正  矫正 平行姿态， 边距距离
入口参数：		无
返回  值：		无
**************************************************************************/
void AiwacPositionCorrection(void)
{

	float distanceDvalueToL = 0 ;  
	u8 PositionFlag1 = 0;  //  当前小车  边距状态，1：矫正Ok,			  0：未完成
	u8 PositionFlag2 = 0;  //  当前小车  平行状态，1：矫正Ok,		  0：未完成



	// 紧急制动
	if ( ((carDistance.distanceF >0) && (carDistance.distanceL1  >0) && (carDistance.distanceL2 >0))  //  已经开始测量
		&& ((carDistance.distanceF < 0.2) || (carDistance.distanceL1  < 0.06) || (carDistance.distanceL2 < 0.06)) )  //判断危险的情况
	{
		AIWACStop = 1;
	}

		
		
	// 还未获得距离值，不进行矫正
	if (carDistance.start == 0 )
	{
		return;
	}


	// 正在入弯道，不进行矫正
	if ((moveState  == STATE_TURN_LEFT ) || (moveState  == STATE_TURN_RIGHT ) )
	{
		return;
	}


	//小车离轨道边距的矫正
	distanceDvalueToL = (carDistance.distanceL1 * 1000 + carDistance.distanceL2 * 1000)/2 - GOALlDISTANCETOL ; 
	if (distanceDvalueToL >7) // 离轨道过远，超过10mm
	{
		//  轨道  垂直方向  提供下速度
		AIWAC_Move_Y = -(CORRECTION_Y);  // 向轨道 靠近， mm/s
		printf("\r\n close to track");

		if (distanceDvalueToL > 50)
		{
			AIWAC_Move_Y = -(CORRECTION_Y_BIG);
		}
		else
		{
			//  轨道  垂直方向  提供下速度
			AIWAC_Move_Y = -(CORRECTION_Y);  // 向轨道 靠近， mm/s
		}
		PositionFlag1 = 0;
	}
	else if (distanceDvalueToL <-7) // 离轨道过近，太近  m
	{
		printf("\r\n go far from  track");
		if (distanceDvalueToL < -50)
		{
			AIWAC_Move_Y = (CORRECTION_Y_BIG);
		}
		else
		{
			//  轨道  垂直方向  提供下速度
			AIWAC_Move_Y = (CORRECTION_Y);  // 向轨道 离远， mm/s
		}
		
		PositionFlag1 = 0;
	}else{

		//  轨道  垂直方向 速度为 0
		AIWAC_Move_Y = 0;
		PositionFlag1 = 1;
	}


	

	// 小车与轨道平行姿态矫正
	if (carDistance.distanceL1 * 1000- carDistance.distanceL2 * 1000 >CORRECTION_Z_DISTANCE )  //该逆时针旋转
	{
		// Z轴加上 逆时针的  速度
		printf("\r\n need ni");
		AIWAC_Move_Z =  -(CORRECTION_Z);   // mm/s
		PositionFlag2 = 0;
	}
	else if ((carDistance.distanceL1 * 1000- carDistance.distanceL2 * 1000) < -(CORRECTION_Z_DISTANCE)) //该顺时针旋转
	{
		printf("\r\n need shun"); 
		// Z轴加上 顺时针的  速度 
		AIWAC_Move_Z =  (CORRECTION_Z);  // mm/s
		PositionFlag2= 0;
	}else {

		// Z轴加上 顺时针的  速度  为0
		AIWAC_Move_Z = 0;
		PositionFlag2 = 1;
	}


	// 自校正 状态
	if ((PositionFlag1 == 1) && (PositionFlag2 == 1) )
	{

		carDistance.leftPositionOK = 1;
		printf("\r\n leftPositionOK");
	}else
	{
		carDistance.leftPositionOK = 0;
	}
}



/**************************************************************************
函数功能：		小车 轮控制 函数， 包含 自校正  、转弯、
入口参数：		无
返回  值：		无
**************************************************************************/
void AiwacSupermarketCarControl(void)
{
	AIWAC_Move_Y = 0;
	AIWAC_Move_Z = 0;
	AIWAC_Move_X = 0;


	/*	
	// 还未获得距离值，不进行矫正
	if (carDistance.start == 0 )
	{
		return;
	}

	*/

	// X 前进速度  由  主控下发指令
	AIWAC_Move_X = -(AIWAC_MOVE_Xtemp);
	
	if ((moveState == STATE_STOP) || (moveState == STATE_STRAIGHT)) // 停止或 直线运动
	{

		AiwacPositionCorrection();  // 会产生 Y    Z轴的速度
	}  
	else if (moveState == STATE_TURN_RIGHT) // 开始右转弯
	{
		
		if ((carDistance.distanceF > 0.7) && (myabs_double(carDistance.distanceL1 - carDistance.distanceL2) <0.01) )
		{
		
			//send()  // 发送  转弯结束的情况
			moveState = STATE_STOP;
			printf("\r\n turing over!!!");
			AIWACTuringTime = 0;
		}
		else {
			printf("\r\n turing  begining!!");

			AIWAC_Move_X = -AIWAC_V_sum;
			AIWAC_Move_Y = 0;	
			AIWAC_Move_Z = (AIWAC_R_vehicle/AIWAC_R_gui)*(-AIWAC_Move_X);	

		}

	}
	else if (moveState == STATE_TURN_LEFT)  // 向左转弯
	{
		if ((carDistance.distanceB > 0.7) && (myabs_double(carDistance.distanceL1 - carDistance.distanceL2) <0.01) )
		{
		
			//send()  // 发送  转弯结束的情况
			moveState = STATE_STOP;
		}
		else {

			AIWAC_Move_X = AIWAC_V_sum;
			AIWAC_Move_Y = 0;	
			AIWAC_Move_Z = (AIWAC_R_vehicle/AIWAC_R_gui)*(-AIWAC_Move_X);	
			
		}

	}

	
/*

	if (AIWACStop == 1)  //  强制停止
	{
		AIWAC_Move_X = 0;
		AIWAC_Move_Y = 0;
		AIWAC_Move_Z = 0;
		
		moveState = STATE_STOP;
		printf("\r\n stop ,need to restart!!");
	}
*/
 

	Kinematic_Analysis_SpeedMode_Aiwac(AIWAC_Move_X,AIWAC_Move_Y,AIWAC_Move_Z);//得到控制目标值，进行运动学分析
}






/**************************************************************************
函数功能：		解析从串口  获取的  三个方向的  距离
入口参数：		无
返回  值：		无
**************************************************************************/
void AiwacParseDistanceJson(void)
{
	cJSON *rootDistance, *DistanceValue;  


	if (jsonParseBuF[0] == '-' ) //  还未收到 距离信息
	{
		return;
	}
	
	rootDistance = cJSON_Parse(jsonParseBuF);
		if (!rootDistance) 
		{
			printf("Error before: [%s]\n",cJSON_GetErrorPtr());
		return;
	}

	DistanceValue = cJSON_GetObjectItem(rootDistance, "F");  //  需要确定  距离 标签       	前方的 
	if (!DistanceValue) {
	   // printf("get name faild !\n");
	    //printf("Error before: [%s]\n", cJSON_GetErrorPtr());
	    goto end;
	}
	//printf("\r\n  (DistanceValue->valuedouble < 0) :%d  DistanceValue->valuedouble:%f",(DistanceValue->valuedouble < 0), DistanceValue->valuedouble);
	if (DistanceValue->valuedouble > 0)
	{
		carDistance.distanceF = DistanceValue->valuedouble;  //前方的距离
	}
	else
	{
		printf("\r\nF:%f",DistanceValue->valuedouble);
	}



	DistanceValue = cJSON_GetObjectItem(rootDistance, "B");  //  需要确定  距离 标签       	后方的 
	if (!DistanceValue) {
	   // printf("get name faild !\n");
	    //printf("Error before: [%s]\n", cJSON_GetErrorPtr());
	    goto end;
	}
	//printf("\r\n  (DistanceValue->valuedouble < 0) :%d  DistanceValue->valuedouble:%f",(DistanceValue->valuedouble < 0), DistanceValue->valuedouble);
	if (DistanceValue->valuedouble > 0)
	{
		carDistance.distanceB = DistanceValue->valuedouble;  //后方的距离
	}
	else
	{
		printf("\r\nB:%f",DistanceValue->valuedouble);
	}


	



	DistanceValue = cJSON_GetObjectItem(rootDistance, "L1");  //  需要确定  距离 标签			左1
	if (!DistanceValue) {
	    //printf("get name faild !\n");
	    //printf("Error before: [%s]\n", cJSON_GetErrorPtr());
	    goto end;
	}
	//printf("\r\n  (DistanceValue->valuedouble < 0) :%d	DistanceValue->valuedouble:%f",(DistanceValue->valuedouble < 0), DistanceValue->valuedouble);
	if (DistanceValue->valuedouble >0)
	{
		carDistance.distanceL1 = DistanceValue->valuedouble;  //左1的距离
	}
	else
	{
		printf("\r\nL1:%f",DistanceValue->valuedouble);
	}




	DistanceValue = cJSON_GetObjectItem(rootDistance, "L2");  //  需要确定  距离 标签			左2
	if (!DistanceValue) {
	    //printf("get name faild !\n");
	    //printf("Error before: [%s]\n", cJSON_GetErrorPtr());
	    goto end;
	}
	//printf("\r\n  (DistanceValue->valuedouble < 0) :%d	DistanceValue->valuedouble:%f",(DistanceValue->valuedouble < 0), DistanceValue->valuedouble);
	if (DistanceValue->valuedouble > 0)
	{
		carDistance.distanceL2 = DistanceValue->valuedouble;  //左2的距离
	}
	else
	{
		printf("\r\nL2:%f",DistanceValue->valuedouble);
	}



// 测试
	DistanceValue = cJSON_GetObjectItem(rootDistance, "d2str");  //  需要确定  距离 标签			左2
	if (!DistanceValue) {
	    //printf("get name faild !\n");
	    //printf("Error before: [%s]\n", cJSON_GetErrorPtr());
	}
	printf("\r\nd2str:%s",DistanceValue->valuestring);

		DistanceValue = cJSON_GetObjectItem(rootDistance, "d3str");  //  需要确定  距离 标签			左2
	if (!DistanceValue) {
	    //printf("get name faild !\n");
	    //printf("Error before: [%s]\n", cJSON_GetErrorPtr());
	}
	printf("\r\nd3str:%s",DistanceValue->valuestring);

		DistanceValue = cJSON_GetObjectItem(rootDistance, "d5str");  //  需要确定  距离 标签			左2
	if (!DistanceValue) {
	    //printf("get name faild !\n");
	    //printf("Error before: [%s]\n", cJSON_GetErrorPtr());
	}
	printf("\r\nd5str:%s",DistanceValue->valuestring);


end :
	cJSON_Delete(rootDistance);
}




/**************************************************************************
函数功能：		解析从串口  获取的  主控下发的  运动指令
入口参数：		无
返回  值：		无
**************************************************************************/
void AiwacParseMOVEOrder(void)
{
	cJSON *rootMoveOrder, *orderValue;  //  不晓得name需不需要回收

	if (USART2_jsonParseBuF[0] == '-' ) //  还未收到运动命令
	{
		return;
	}

	rootMoveOrder = cJSON_Parse(USART2_jsonParseBuF);
	if (!rootMoveOrder) 
	{
		printf("Error before: [%s]\n",cJSON_GetErrorPtr());
		return;
	}


	orderValue = cJSON_GetObjectItem(root, "businessType");  //  businessType
	if (!orderValue) {
		//printf("get name faild !\n");
		//printf("Error before: [%s]\n", cJSON_GetErrorPtr());
		goto end;
	}


	// 主控查询小车情况,小车反馈
	if ((strcmp(orderValue.valuestring, "0007")==0) || (strcmp(orderValue.valuestring, "0008")==0)) 
		{
			AiwacFeedback2Master();  // 给主控反馈小车情况
		}
	else if ((strcmp(orderValue.valuestring, "0009")==0)  || (strcmp(orderValue.valuestring, "0010")==0) ) //  主控控制小车运动及状态
		{
			orderValue = cJSON_GetObjectItem(rootMoveOrder, "X_V");  //  X轴速度 
			if (!orderValue) {
			    //printf("get name faild !\n");
			    //printf("Error before: [%s]\n", cJSON_GetErrorPtr());
			    goto end ;
			}
			AIWAC_MOVE_Xtemp = orderValue->valuedouble;  //X轴速度 


			orderValue = cJSON_GetObjectItem(rootMoveOrder, "mo");  //  运动指令
			if (!orderValue) {
			   // printf("get name faild !\n");
			   // printf("Error before: [%s]\n", cJSON_GetErrorPtr());
			   goto end ;
			}
			moveState = orderValue->valueint;  //运动指令
		}
	

end :
	cJSON_Delete(rootMoveOrder);
}



void  AiwacSendState2Master(void)
{

	u16 jsonSize;
	cJSON *root;
	char *strJson;
	char strSend[300];
	
	strSend[0] = '#';
	strSend[1] = '!';


	root=cJSON_CreateObject();
	
	if (CAR_ID == CAR_UP)
	{
		cJSON_AddStringToObject(root, "businessType", "0011");
	}
	else
	{
		cJSON_AddStringToObject(root, "businessType", "0012");
	}


	cJSON_AddNumberToObject(root,"Co", carDistance.leftPositionOK);

	cJSON_AddNumberToObject(root,"FD", carDistance.distanceF);
	cJSON_AddNumberToObject(root,"BD", carDistance.distanceB);
	cJSON_AddNumberToObject(root,"mo", moveState);

	strJson=cJSON_PrintUnformatted(root);

	
	cJSON_Delete(root); 
//printf("\r\n strJson:%s",strJson);
	jsonSize = strlen(strJson);

	strSend[2] = jsonSize >> 8;
	strSend[3] = jsonSize;

	strncpy(strSend+4,strJson,jsonSize);

	strSend[jsonSize+4] = '*';
	strSend[jsonSize+5] = crc8_calculate(strJson, jsonSize);
	strSend[jsonSize+6] = '&';


	// 需要打开

	usart2_sendString(strSend,7 + jsonSize);
	myfree(strJson);

}



/**************************************************************************
函数功能：		小车给主控反馈当前复位情况
入口参数：		无
返回  值：		无
**************************************************************************/
void  AiwacFeedback2Master(void)
{

	u16 jsonSize;
	cJSON *root;
	char *strJson;
	char strSend[300];
	
	strSend[0] = '#';
	strSend[1] = '!';


	root=cJSON_CreateObject();

	if (CAR_ID == CAR_UP)
	{
		cJSON_AddStringToObject(root, "businessType", "0007");
	}
	else
	{
		cJSON_AddStringToObject(root, "businessType", "0008");
	}

	if ((carDistance.start == 1))
		{
			cJSON_AddNumberToObject(root, "errorCode", 200);
			cJSON_AddStringToObject(root, "errorDesc", "ok");
		}
	else
		{
			cJSON_AddNumberToObject(root, "errorCode", 199);
			cJSON_AddStringToObject(root, "errorDesc", "the distance measurement  does not work!!!");
		}
	


	strJson=cJSON_PrintUnformatted(root);

	
	cJSON_Delete(root); 
//printf("\r\n strJson:%s",strJson);
	jsonSize = strlen(strJson);

	strSend[2] = jsonSize >> 8;
	strSend[3] = jsonSize;

	strncpy(strSend+4,strJson,jsonSize);

	strSend[jsonSize+4] = '*';
	strSend[jsonSize+5] = crc8_calculate(strJson, jsonSize);
	strSend[jsonSize+6] = '&';


	// 需要打开

	usart2_sendString(strSend,7 + jsonSize);
	myfree(strJson);

}


