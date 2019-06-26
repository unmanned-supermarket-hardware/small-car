#include "control.h"	
#include "filter.h"	
#include <math.h>
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
u8 Flag_Target,Flag_Change;                             //��ر�־λ
u8 temp1;                                               //��ʱ����
float Voltage_Count,Voltage_All;  //��ѹ������ر���
float Gyro_K=0.00;       //�����Ǳ���ϵ��
int j;
#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)      
#define L_PARAMETER            (1.0f)         
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
         �ϸ�֤���������ݴ�����ʱ��ͬ��				 
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
		  if(PS2_KEY==4)PS2_ON_Flag=1,CAN_ON_Flag=0,Usart_ON_Flag=0;						
		  if(CAN_ON_Flag==1||Usart_ON_Flag==1||PS2_ON_Flag==1) CAN_N_Usart_Control();       //�ӵ����ڻ���CANң�ؽ���ָ��֮��ʹ��CAN�ʹ��ڿ�������

		 if(Turn_Off(Voltage)==0)               //===�����ص�ѹ�������쳣
		 { 			 
		  if(Run_Flag==0)//�ٶ�ģʽ
			{		
				if(CAN_ON_Flag==0&&Usart_ON_Flag==0&&PS2_ON_Flag==0)  Get_RC(Run_Flag);  //===���ں�CAN���ƶ�δʹ�ܣ����������ң��ָ
				Motor_A=Incremental_PI_A(Encoder_A,Target_A);                         //===�ٶȱջ����Ƽ�����A����PWM
				Motor_B=Incremental_PI_B(Encoder_B,Target_B);                         //===�ٶȱջ����Ƽ�����B����PWM
				Motor_C=Incremental_PI_C(Encoder_C,Target_C);                         //===�ٶȱջ����Ƽ�����C����PWM
			}
			 else//λ��ģʽ
			{
					if(CAN_ON_Flag==0&&Usart_ON_Flag==0&&PS2_ON_Flag==0) //===���ں�CAN���ƶ�δʹ�ܣ����������ң��ָ��
				 {	
					if(Turn_Flag==0) 	Flag_Direction=click_RC();     
					Get_RC(Run_Flag);
				 }
					Motor_A=Position_PID_A(Position_A,Target_A)>>8;//λ�ñջ����ƣ�������A�ٶ��ڻ���������
					Motor_B=Position_PID_B(Position_B,Target_B)>>8;//λ�ñջ����ƣ�������B�ٶ��ڻ���������
					Motor_C=Position_PID_C(Position_C,Target_C)>>8;//λ�ñջ����ƣ�������C�ٶ��ڻ���������
				
			    if(rxbuf[0]!=2)  Count_Velocity();   //���ǵ���λ�ÿ��ƹ��̵��ٶȴ�С
					else 	
					Xianfu_Velocity(RC_Velocity,RC_Velocity,RC_Velocity); 
					Motor_A=Incremental_PI_A(Encoder_A,-Motor_A);         //===�ٶȱջ����Ƽ�����A����PWM
					Motor_B=Incremental_PI_B(Encoder_B,-Motor_B);         //===�ٶȱջ����Ƽ�����B����PWM
					Motor_C=Incremental_PI_C(Encoder_C,-Motor_C);         //===�ٶȱջ����Ƽ�����C����PWM
			}	 
		 Xianfu_Pwm(6500);                     //===PWM�޷�
		 Set_Pwm(Motor_B,Motor_A,Motor_C);     //===��ֵ��PWM�Ĵ���  
		 }
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
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)��������ƫ�� 
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
e(k)��������ƫ�� 
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