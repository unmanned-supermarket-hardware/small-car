#include "control.h"	
#include "filter.h"	
#include <math.h>


u8 Flag_Target,Flag_Change;                             //��ر�־λ
u8 temp1;                                               //��ʱ����
float Voltage_Count,Voltage_All;  //��ѹ������ر���
float Gyro_K=0.00;       //�����Ǳ���ϵ��
int j;


//AIWAC  ���˳���
struct CarDistance carDistance;  // �洢 ���� �����ľ���
float AIWAC_R_vehicle = 310;  //С���뾶����λ��mm
float AIWAC_R_gui = 400;  // ����뾶����λ��mm
float AIWAC_Move_X  = 0, AIWAC_Move_Y = 0, AIWAC_Move_Z = 0;   //����ǶȺ�XYZ��Ŀ���ٶ�
float AIWAC_MOVE_Xtemp = 0;  // ����  �����·��� X �ٶ�
float AIWAC_V_sum = 100;  // ��ǰ���ٶȣ���λ  mm/s
int AIWACTuringTime = 0;     
  // ת���ʱ�����
int moveState = STATE_STOP; // С���˶� ״̬��  0��ֹͣ �����ϵ�  �� �ճ��������  1��  ֱ��  2�� ˳ʱ��ת  3����ʱ��ת 
int AIWACStop = 0;		//����������  Σ��ʱ������ֹͣ   �����ϵ����








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
�������ܣ�С���˶���ѧģ��
��ڲ�����X Y Z �����ٶȻ���λ��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
			Target_A   = Vx + L_PARAMETER*Vz+gyro[2]*Gyro_K;
			Target_B   = -X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz+gyro[2]*Gyro_K;
			Target_C   = -X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz+gyro[2]*Gyro_K;
			
}


/**************************************************************************
�������ܣ�	С���˶���ѧģ�ͣ��� X Y����� Z(�������ٶ�) ��λmm/s��ת��Ϊ����
			�ֵ����ٶȣ���ת��Ϊ  �����ֵ� ��5ms�ڵ���������Ƶ�ʣ�
��ڲ�����X Y Z �����ٶ�   mm/s
����  ֵ����
**************************************************************************/
void Kinematic_Analysis_SpeedMode_Aiwac(float Vx,float Vy,float Vz)
{
	float Target_A_Speed, Target_B_Speed, Target_C_Speed;

	// �ϳ���Ҫ��������Y ��Ҫ��
	if(CAR_ID == CAR_UP)
	{
		Vy = -Vy;
	}
	
	Target_A_Speed   = Vx + L_PARAMETER*Vz;
	Target_B_Speed   = -X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz;
	Target_C_Speed   = -X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz;


	/**************************************************************************
	 ������  ����תһȦ  ��Ҫ	1800 ������,һȦ �ܳ� Rֱ�� = 127mm,C = 127mm * PI
	 ��һ�������  ��Ӧ ��ת����  ����  Ϊ    	C/1800����  127mm*PI/1800
	 Ŀ�� ���� Target_A ���� �� 10ms  �� ���Aת�������壬��Ϊ M
	 �� ÿ��      100M�����壬��Ӧ�� ������     100M*127mm*PI/1800
	 �� �ٶ�  100M*127mm*PI/1800/s = 127*M*PI/ 18  mm/s
	 ���� M ���� ������Ҫ��ֵ��Target_A��ֵ

	 ����Ŀ���ٶ� V mm/s = 127*M*PI/ 18  mm/s,   ��ô M = V*18/(127*PI)
	**************************************************************************/

	Target_A = Target_A_Speed*18/(127*PI);
	Target_B = Target_B_Speed*18/(127*PI);
	Target_C = Target_C_Speed*18/(127*PI);

}

/**************************************************************************
�������ܣ���ȡλ�ÿ��ƹ����ٶ�ֵ
��ڲ�����X Y Z ����λ�ñ仯��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis2(float Vx,float Vy,float Vz)
{
			Rate_A   = Vx + L_PARAMETER*Vz;
			Rate_B   =-X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz;
			Rate_C   =-X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz;
}
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	 if(INT==0)		
	{     
		  EXTI->PR=1<<15;                                                      //���LINE5�ϵ��жϱ�־λ  		
		  Flag_Target=!Flag_Target;
		  if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //���������ṩ50ms�ľ�׼��ʱ
			 }
		  if(Flag_Target==1)                                                  //5ms��ȡһ�������Ǻͼ��ٶȼƵ�ֵ
			{
			if(Usart_Flag==0&&PS2_ON_Flag==0&&Usart_ON_Flag==1)  memcpy(rxbuf,Urxbuf,8*sizeof(u8));	//��������˴��ڿ��Ʊ�־λ�����봮�ڿ���ģʽ
			Read_DMP();                                                           //===������̬		
			Key();//ɨ�谴���仯	
			return 0;	                                               
			}                                                                      //===10ms����һ�Σ�Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ����������

		  
			Encoder_A=Read_Encoder(2)/20;                                          //===��ȡ��������ֵ
			Position_A+=Encoder_A;                                                 //===���ֵõ�λ�� 
			Encoder_B=Read_Encoder(3)/20;                                          //===��ȡ��������ֵ
			Position_B+=Encoder_B;                                                 //===���ֵõ�λ�� 
			Encoder_C=Read_Encoder(4)/20;                                          //===��ȡ��������ֵ
			Position_C+=Encoder_C;                                                 //===���ֵõ�λ��     
	  	Read_DMP();                                                            //===������̬	
  		Led_Flash(100);                                                        //===LED��˸;����ģʽ 1s�ı�һ��ָʾ�Ƶ�״̬	
  		
		Voltage_All+=Get_battery_volt();                                       //��β����ۻ�
		if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//��ƽ��ֵ ��ȡ��ص�ѹ	       
		 if(Turn_Off(Voltage)!=0)               //===�����ص�ѹ�����쳣
		 { 	
			return;
		 }

		 
		  //if(PS2_KEY==4)PS2_ON_Flag=1,CAN_ON_Flag=0,Usart_ON_Flag=0;						
		  //if(CAN_ON_Flag==1||Usart_ON_Flag==1||PS2_ON_Flag==1) CAN_N_Usart_Control();       //�ӵ����ڻ���CANң�ؽ���ָ��֮��ʹ��CAN�ʹ��ڿ�������
		  //if(CAN_ON_Flag==0&&Usart_ON_Flag==0&&PS2_ON_Flag==0)  Get_RC(Run_Flag);  //===���ں�CAN���ƶ�δʹ�ܣ����������ң��ָ
		 
	



		Motor_A=Incremental_PI_A(Encoder_A,Target_A);                         //===�ٶȱջ����Ƽ�����A����PWM
		Motor_B=Incremental_PI_B(Encoder_B,Target_B);                         //===�ٶȱջ����Ƽ�����B����PWM
		Motor_C=Incremental_PI_C(Encoder_C,Target_C);                         //===�ٶȱջ����Ƽ�����C����PWM
				
		 Xianfu_Pwm(6500);                     //===PWM�޷�
		 Set_Pwm(Motor_B,Motor_A,Motor_C);     //===��ֵ��PWM�Ĵ���  
		 
 	}
	 return 0;	 
} 

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
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
�������ܣ�����PWM��ֵ 
��ڲ�������ֵ
����  ֵ����
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
�������ܣ�λ��PID���ƹ������ٶȵ�����
��ڲ������ޡ���ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C)
{	
    if(Motor_A<-amplitude_A) Motor_A=-amplitude_A;	//λ�ÿ���ģʽ�У�A����������ٶ�
		if(Motor_A>amplitude_A)  Motor_A=amplitude_A;	  //λ�ÿ���ģʽ�У�A����������ٶ�
	  if(Motor_B<-amplitude_B) Motor_B=-amplitude_B;	//λ�ÿ���ģʽ�У�B����������ٶ�
		if(Motor_B>amplitude_B)  Motor_B=amplitude_B;		//λ�ÿ���ģʽ�У�B����������ٶ�
	  if(Motor_C<-amplitude_C) Motor_C=-amplitude_C;	//λ�ÿ���ģʽ�У�C����������ٶ�
		if(Motor_C>amplitude_C)  Motor_C=amplitude_C;		//λ�ÿ���ģʽ�У�C����������ٶ�
}
/**************************************************************************
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double(100); 
	if(tmp==2)Flag_Show=!Flag_Show;//˫��������ʾģʽ                  
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<2000||EN==0)//��ص�ѹ����20V�رյ��
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
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����double
����  ֵ��unsigned int
**************************************************************************/
double myabs_double(double a)
{ 		   
	  double temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;  // 
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_C (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ���������������λ����Ϣ��Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ 
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,,k;
pwm�������
**************************************************************************/
int Position_PID_A (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 if(Integral_bias>100000)Integral_bias=10000;
	 if(Integral_bias<-100000)Integral_bias=-10000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
int Position_PID_B (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 if(Integral_bias>100000)Integral_bias=10000;
	 if(Integral_bias<-100000)Integral_bias=-10000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
int Position_PID_C (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 if(Integral_bias>100000)Integral_bias=10000;
	 if(Integral_bias<-100000)Integral_bias=-10000;
	 Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
/**************************************************************************
�������ܣ�ͨ������ָ���С������ң��
��ڲ���������ָ��
����  ֵ����
��ǰ�汾˵����ת90���Զ�ֹͣ
**************************************************************************/
#define STOP 0
#define MOVING_STRAIGHT 1
#define TURNING 2

float V_sum = 300;  // ��ǰ���ٶȣ���λ  mm/s
u8 state_flag = 0;    //Ŀǰ״̬����STOP��MOVING_STRAIGHT��TURNING ����ȡֵ
u8 last_state_flag = 0;
u8 last_flag_direction = 0;
//-------------------------
void Get_RC(u8 mode)
{
	float step=0.25;  //�����ٶȿ��Ʋ���ֵ��
	int yaw = Yaw;  // ���������ƫ��
	int angle = Yaw;
	int pitch = Pitch;
	int roll = Roll;
	u8 Flag_Move=1;
	float R_vehicle = 310;  //С���뾶����λ��mm
	float R_gui = 400;  // ����뾶����λ��mm
	
	char strTemp[64];
	  if(mode==0)//�ٶ�
		{	

			 switch(Flag_Direction)   //�������
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
							if(state_flag == TURNING)   //����ת���ж��Ƿ�ת��
							{

								//debug
								
								//˳ʱ��/��ת��Yaw--
								if(angle >-90) //ûת�꣬����ת
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


			 //  һ��Ƶ��ʱ������  Get_RC ������ÿ�ζ���⵽��  ������
			if(Flag_Move==0)		//����޷������ָ��	 �����ת�����״̬
			{	
				if(Flag_Left==1)       Move_Z-=step,Gyro_K=0;    //������   
				else if(Flag_Right==1) Move_Z+=step,Gyro_K=0;    //������		
				else 		               Move_Z=0,Gyro_K=0.00;    //ֹͣ
			}	
			sprintf(strTemp, "default	pitch ==%d , roll == %d ,angle == %d\r\n",pitch,roll,angle);
			//usart1_sendString(strTemp,strlen(strTemp));	

			
			
				//if(Flag_Move==1)	Flag_Left=0,Flag_Right=0,Move_Z=0;

			/*
				if(Move_X<-RC_Velocity) Move_X=-RC_Velocity;	   //�ٶȿ����޷�
				if(Move_X>RC_Velocity)  Move_X=RC_Velocity;	     
				if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
				if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
				if(Move_Z<-RC_Velocity) Move_Z=-RC_Velocity;	
				if(Move_Z>RC_Velocity)  Move_Z=RC_Velocity;	 
			*/

		 }/*
		 else if(mode==1)//λ��ģʽ
		{	
				 switch(Flag_Direction)   //�������
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


		//Kinematic_Analysis(Move_X,Move_Y,Move_Z);//�õ�����Ŀ��ֵ�������˶�ѧ����
		Kinematic_Analysis_SpeedMode_Aiwac(Move_X,Move_Y,Move_Z);//�õ�����Ŀ��ֵ�������˶�ѧ����
}
/**************************************************************************
�������ܣ�ÿ�����λ�ÿ��ƹ����ٶȼ���
��ڲ�������
����  ֵ����
**************************************************************************/
void Count_Velocity(void)
{
	static double Last_Target_X,Last_Target_Y,Last_Target_Z,Divider;
	double Bias_X,Bias_Y,Bias_Z;
	Bias_X=(Move_X-Last_Target_X);  //��X��λ����
	Bias_Y=(Move_Y-Last_Target_Y);	//��Y��λ����
	Bias_Z=(Move_Z-Last_Target_Z);	//��Z��λ����
	if(Bias_X!=0||Bias_Y!=0||Bias_Z!=0)Divider=sqrt(Bias_X*Bias_X+Bias_Y*Bias_Y+Bias_Z*Bias_Z);
	if(Bias_X!=0||Bias_Y!=0||Bias_Z!=0) Kinematic_Analysis2(Bias_X,Bias_Y,Bias_Z);

	Xianfu_Velocity(RC_Velocity*myabs(Rate_A)/Divider,RC_Velocity*myabs(Rate_B)/Divider,RC_Velocity*myabs(Rate_C)/Divider); 
	Last_Target_X=Move_X;   //����X����һ�ε�λ����Ϣ�����ڵ���
	Last_Target_Y=Move_Y;   //����Y����һ�ε�λ����Ϣ�����ڵ���
	Last_Target_Z=Move_Z;   //����Z����һ�ε�λ����Ϣ�����ڵ���
}
/**************************************************************************
�������ܣ�����CAN���ߴ��ڿ���ָ����д���
��ڲ�������
����  ֵ����
**************************************************************************/
void CAN_N_Usart_Control(void)
{
   int flag_X, flag_Y,flag_Z;
	 int Yuzhi=20;
	 int LX,LY,RX;
	 if(Run_Flag==0)//�ٶ�ģʽ
	 {
		  if(CAN_ON_Flag==1||Usart_ON_Flag==1) 
		{
				 if((rxbuf[7]&0x04)==0)flag_X=1;  else flag_X=-1;  //�������λ
				 if((rxbuf[7]&0x02)==0)flag_Y=1;  else flag_Y=-1;  //�������λ
				 if((rxbuf[7]&0x01)==0)flag_Z=1;  else flag_Z=-1;  //�������λ
				 Move_X=flag_X*(rxbuf[1]*256+rxbuf[2]);
				 Move_Y=flag_Y*(rxbuf[3]*256+rxbuf[4]);	
				 Move_Z=flag_Z*(rxbuf[5]*256+rxbuf[6]);	
				 if(rxbuf[0]==1)Kinematic_Analysis(Move_X,Move_Y,Move_Z),Gyro_K=0;    //�����˶�ѧ����
				 if(rxbuf[0]==2)Target_A=Move_X,Target_B=Move_Y,Target_C=Move_Z;      //������ÿ��������п���
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
//		 if(Move_X<-RC_Velocity) Move_X=-RC_Velocity;	   //�ٶȿ����޷�
//		if(Move_X>RC_Velocity)  Move_X=RC_Velocity;	     
//		if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
//		if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
//		if(Move_Z<-RC_Velocity) Move_Z=-RC_Velocity;	
//		if(Move_Z>RC_Velocity)  Move_Z=RC_Velocity;	 
				Kinematic_Analysis(Move_X,Move_Y,Move_Z),Gyro_K=0;    //�����˶�ѧ���� 
			}
	 }
}


/**************************************************************************
�������ܣ�		С���Խ���  ���� ƽ����̬�� �߾����
��ڲ�����		��
����  ֵ��		��
**************************************************************************/
void AiwacPositionCorrection(void)
{

	float distanceDvalueToL = 0 ;  
	u8 PositionFlag1 = 0;  //  ��ǰС��  �߾�״̬��1������Ok,			  0��δ���
	u8 PositionFlag2 = 0;  //  ��ǰС��  ƽ��״̬��1������Ok,		  0��δ���



	// �����ƶ�
	if ( ((carDistance.distanceF >0) && (carDistance.distanceL1  >0) && (carDistance.distanceL2 >0))  //  �Ѿ���ʼ����
		&& ((carDistance.distanceF < 0.2) || (carDistance.distanceL1  < 0.06) || (carDistance.distanceL2 < 0.06)) )  //�ж�Σ�յ����
	{
		AIWACStop = 1;
	}

		
		
	// ��δ��þ���ֵ�������н���
	if (carDistance.start == 0 )
	{
		return;
	}


	// ����������������н���
	if ((moveState  == STATE_TURN_LEFT ) || (moveState  == STATE_TURN_RIGHT ) )
	{
		return;
	}


	//С�������߾�Ľ���
	distanceDvalueToL = (carDistance.distanceL1 * 1000 + carDistance.distanceL2 * 1000)/2 - GOALlDISTANCETOL ; 
	if (distanceDvalueToL >7) // ������Զ������10mm
	{
		//  ���  ��ֱ����  �ṩ���ٶ�
		AIWAC_Move_Y = -(CORRECTION_Y);  // ���� ������ mm/s
		printf("\r\n close to track");

		if (distanceDvalueToL > 50)
		{
			AIWAC_Move_Y = -(CORRECTION_Y_BIG);
		}
		else
		{
			//  ���  ��ֱ����  �ṩ���ٶ�
			AIWAC_Move_Y = -(CORRECTION_Y);  // ���� ������ mm/s
		}
		PositionFlag1 = 0;
	}
	else if (distanceDvalueToL <-7) // ����������̫��  m
	{
		printf("\r\n go far from  track");
		if (distanceDvalueToL < -50)
		{
			AIWAC_Move_Y = (CORRECTION_Y_BIG);
		}
		else
		{
			//  ���  ��ֱ����  �ṩ���ٶ�
			AIWAC_Move_Y = (CORRECTION_Y);  // ���� ��Զ�� mm/s
		}
		
		PositionFlag1 = 0;
	}else{

		//  ���  ��ֱ���� �ٶ�Ϊ 0
		AIWAC_Move_Y = 0;
		PositionFlag1 = 1;
	}


	

	// С������ƽ����̬����
	if (carDistance.distanceL1 * 1000- carDistance.distanceL2 * 1000 >CORRECTION_Z_DISTANCE )  //����ʱ����ת
	{
		// Z����� ��ʱ���  �ٶ�
		printf("\r\n need ni");
		AIWAC_Move_Z =  -(CORRECTION_Z);   // mm/s
		PositionFlag2 = 0;
	}
	else if ((carDistance.distanceL1 * 1000- carDistance.distanceL2 * 1000) < -(CORRECTION_Z_DISTANCE)) //��˳ʱ����ת
	{
		printf("\r\n need shun"); 
		// Z����� ˳ʱ���  �ٶ� 
		AIWAC_Move_Z =  (CORRECTION_Z);  // mm/s
		PositionFlag2= 0;
	}else {

		// Z����� ˳ʱ���  �ٶ�  Ϊ0
		AIWAC_Move_Z = 0;
		PositionFlag2 = 1;
	}


	// ��У�� ״̬
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
�������ܣ�		С�� �ֿ��� ������ ���� ��У��  ��ת�䡢
��ڲ�����		��
����  ֵ��		��
**************************************************************************/
void AiwacSupermarketCarControl(void)
{
	AIWAC_Move_Y = 0;
	AIWAC_Move_Z = 0;
	AIWAC_Move_X = 0;


	/*	
	// ��δ��þ���ֵ�������н���
	if (carDistance.start == 0 )
	{
		return;
	}

	*/

	// X ǰ���ٶ�  ��  �����·�ָ��
	AIWAC_Move_X = -(AIWAC_MOVE_Xtemp);
	
	if ((moveState == STATE_STOP) || (moveState == STATE_STRAIGHT)) // ֹͣ�� ֱ���˶�
	{

		AiwacPositionCorrection();  // ����� Y    Z����ٶ�
	}  
	else if (moveState == STATE_TURN_RIGHT) // ��ʼ��ת��
	{
		
		if ((carDistance.distanceF > 0.7) && (myabs_double(carDistance.distanceL1 - carDistance.distanceL2) <0.01) )
		{
		
			//send()  // ����  ת����������
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
	else if (moveState == STATE_TURN_LEFT)  // ����ת��
	{
		if ((carDistance.distanceB > 0.7) && (myabs_double(carDistance.distanceL1 - carDistance.distanceL2) <0.01) )
		{
		
			//send()  // ����  ת����������
			moveState = STATE_STOP;
		}
		else {

			AIWAC_Move_X = AIWAC_V_sum;
			AIWAC_Move_Y = 0;	
			AIWAC_Move_Z = (AIWAC_R_vehicle/AIWAC_R_gui)*(-AIWAC_Move_X);	
			
		}

	}

	
/*

	if (AIWACStop == 1)  //  ǿ��ֹͣ
	{
		AIWAC_Move_X = 0;
		AIWAC_Move_Y = 0;
		AIWAC_Move_Z = 0;
		
		moveState = STATE_STOP;
		printf("\r\n stop ,need to restart!!");
	}
*/
 

	Kinematic_Analysis_SpeedMode_Aiwac(AIWAC_Move_X,AIWAC_Move_Y,AIWAC_Move_Z);//�õ�����Ŀ��ֵ�������˶�ѧ����
}






/**************************************************************************
�������ܣ�		�����Ӵ���  ��ȡ��  ���������  ����
��ڲ�����		��
����  ֵ��		��
**************************************************************************/
void AiwacParseDistanceJson(void)
{
	cJSON *rootDistance, *DistanceValue;  


	if (jsonParseBuF[0] == '-' ) //  ��δ�յ� ������Ϣ
	{
		return;
	}
	
	rootDistance = cJSON_Parse(jsonParseBuF);
		if (!rootDistance) 
		{
			printf("Error before: [%s]\n",cJSON_GetErrorPtr());
		return;
	}

	DistanceValue = cJSON_GetObjectItem(rootDistance, "F");  //  ��Ҫȷ��  ���� ��ǩ       	ǰ���� 
	if (!DistanceValue) {
	   // printf("get name faild !\n");
	    //printf("Error before: [%s]\n", cJSON_GetErrorPtr());
	    goto end;
	}
	//printf("\r\n  (DistanceValue->valuedouble < 0) :%d  DistanceValue->valuedouble:%f",(DistanceValue->valuedouble < 0), DistanceValue->valuedouble);
	if (DistanceValue->valuedouble > 0)
	{
		carDistance.distanceF = DistanceValue->valuedouble;  //ǰ���ľ���
	}
	else
	{
		printf("\r\nF:%f",DistanceValue->valuedouble);
	}



	DistanceValue = cJSON_GetObjectItem(rootDistance, "B");  //  ��Ҫȷ��  ���� ��ǩ       	�󷽵� 
	if (!DistanceValue) {
	   // printf("get name faild !\n");
	    //printf("Error before: [%s]\n", cJSON_GetErrorPtr());
	    goto end;
	}
	//printf("\r\n  (DistanceValue->valuedouble < 0) :%d  DistanceValue->valuedouble:%f",(DistanceValue->valuedouble < 0), DistanceValue->valuedouble);
	if (DistanceValue->valuedouble > 0)
	{
		carDistance.distanceB = DistanceValue->valuedouble;  //�󷽵ľ���
	}
	else
	{
		printf("\r\nB:%f",DistanceValue->valuedouble);
	}


	



	DistanceValue = cJSON_GetObjectItem(rootDistance, "L1");  //  ��Ҫȷ��  ���� ��ǩ			��1
	if (!DistanceValue) {
	    //printf("get name faild !\n");
	    //printf("Error before: [%s]\n", cJSON_GetErrorPtr());
	    goto end;
	}
	//printf("\r\n  (DistanceValue->valuedouble < 0) :%d	DistanceValue->valuedouble:%f",(DistanceValue->valuedouble < 0), DistanceValue->valuedouble);
	if (DistanceValue->valuedouble >0)
	{
		carDistance.distanceL1 = DistanceValue->valuedouble;  //��1�ľ���
	}
	else
	{
		printf("\r\nL1:%f",DistanceValue->valuedouble);
	}




	DistanceValue = cJSON_GetObjectItem(rootDistance, "L2");  //  ��Ҫȷ��  ���� ��ǩ			��2
	if (!DistanceValue) {
	    //printf("get name faild !\n");
	    //printf("Error before: [%s]\n", cJSON_GetErrorPtr());
	    goto end;
	}
	//printf("\r\n  (DistanceValue->valuedouble < 0) :%d	DistanceValue->valuedouble:%f",(DistanceValue->valuedouble < 0), DistanceValue->valuedouble);
	if (DistanceValue->valuedouble > 0)
	{
		carDistance.distanceL2 = DistanceValue->valuedouble;  //��2�ľ���
	}
	else
	{
		printf("\r\nL2:%f",DistanceValue->valuedouble);
	}



// ����
	DistanceValue = cJSON_GetObjectItem(rootDistance, "d2str");  //  ��Ҫȷ��  ���� ��ǩ			��2
	if (!DistanceValue) {
	    //printf("get name faild !\n");
	    //printf("Error before: [%s]\n", cJSON_GetErrorPtr());
	}
	printf("\r\nd2str:%s",DistanceValue->valuestring);

		DistanceValue = cJSON_GetObjectItem(rootDistance, "d3str");  //  ��Ҫȷ��  ���� ��ǩ			��2
	if (!DistanceValue) {
	    //printf("get name faild !\n");
	    //printf("Error before: [%s]\n", cJSON_GetErrorPtr());
	}
	printf("\r\nd3str:%s",DistanceValue->valuestring);

		DistanceValue = cJSON_GetObjectItem(rootDistance, "d5str");  //  ��Ҫȷ��  ���� ��ǩ			��2
	if (!DistanceValue) {
	    //printf("get name faild !\n");
	    //printf("Error before: [%s]\n", cJSON_GetErrorPtr());
	}
	printf("\r\nd5str:%s",DistanceValue->valuestring);


end :
	cJSON_Delete(rootDistance);
}




/**************************************************************************
�������ܣ�		�����Ӵ���  ��ȡ��  �����·���  �˶�ָ��
��ڲ�����		��
����  ֵ��		��
**************************************************************************/
void AiwacParseMOVEOrder(void)
{
	cJSON *rootMoveOrder, *orderValue;  //  ������name�費��Ҫ����

	if (USART2_jsonParseBuF[0] == '-' ) //  ��δ�յ��˶�����
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


	// ���ز�ѯС�����,С������
	if ((strcmp(orderValue.valuestring, "0007")==0) || (strcmp(orderValue.valuestring, "0008")==0)) 
		{
			AiwacFeedback2Master();  // �����ط���С�����
		}
	else if ((strcmp(orderValue.valuestring, "0009")==0)  || (strcmp(orderValue.valuestring, "0010")==0) ) //  ���ؿ���С���˶���״̬
		{
			orderValue = cJSON_GetObjectItem(rootMoveOrder, "X_V");  //  X���ٶ� 
			if (!orderValue) {
			    //printf("get name faild !\n");
			    //printf("Error before: [%s]\n", cJSON_GetErrorPtr());
			    goto end ;
			}
			AIWAC_MOVE_Xtemp = orderValue->valuedouble;  //X���ٶ� 


			orderValue = cJSON_GetObjectItem(rootMoveOrder, "mo");  //  �˶�ָ��
			if (!orderValue) {
			   // printf("get name faild !\n");
			   // printf("Error before: [%s]\n", cJSON_GetErrorPtr());
			   goto end ;
			}
			moveState = orderValue->valueint;  //�˶�ָ��
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


	// ��Ҫ��

	usart2_sendString(strSend,7 + jsonSize);
	myfree(strJson);

}



/**************************************************************************
�������ܣ�		С�������ط�����ǰ��λ���
��ڲ�����		��
����  ֵ��		��
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


	// ��Ҫ��

	usart2_sendString(strSend,7 + jsonSize);
	myfree(strJson);

}


