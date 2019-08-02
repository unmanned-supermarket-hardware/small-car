#include "sys.h"
#include "control.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/ 
u8 Flag_Left,Flag_Right,Flag_Direction=0;   //蓝牙遥控相关的变量
u8 Flag_Stop=1,Flag_Show=0;                 //停止标志位和 显示标志位 默认停止 显示打开
int Encoder_A,Encoder_B,Encoder_C,Encoder_D;          //编码器的脉冲计数
long int Position_A,Position_B,Position_C,Position_D,Rate_A,Rate_B,Rate_C,Rate_D; //PID控制相关变量
int Encoder_A_EXTI;                       //通过外部中断读取的编码器数据                       
long int Motor_A,Motor_B,Motor_C,Motor_D;        //电机PWM变量
long int Target_A,Target_B,Target_C,Target_D;     //电机目标值
int Voltage;                             //电池电压采样相关的变量
float Show_Data_Mb;                      //全局显示变量，用于显示需要查看的数据                         
u8 delay_50,delay_flag;                          //延时相关变量
u8 Run_Flag=0;  //蓝牙遥控相关变量和运行状态标志位
u8 rxbuf[8],Urxbuf[8],CAN_ON_Flag=0,Usart_ON_Flag=0,PS2_ON_Flag=0,Usart_Flag,PID_Send,Flash_Send;  //CAN和串口控制相关变量
u8 txbuf[8],txbuf2[8],Turn_Flag;             //CAN发送相关变量
float Pitch,Roll,Yaw,Move_X  = 0,Move_Y = 0,Move_Z = 0;   //三轴角度和XYZ轴目标速度
u16 PID_Parameter[10],Flash_Parameter[10];  //Flash相关数组
float	Position_KP=6,Position_KI=0,Position_KD=3;  //位置控制PID参数
float Velocity_KP=10,Velocity_KI=10;	          //速度控制PID参数
int RC_Velocity=30,RC_Position=1000;         //设置遥控的速度和位置值
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;  
int Gryo_Z;


int main(void)
{ 

	int timeNumDistance = 0; // 计划500ms 进行 自校准  主要是  左侧轨道的情况，要保证平行

	Stm32_Clock_Init(9);            //=====系统时钟设置
	delay_init(72);                 //=====延时初始化
	JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
	LED_Init();                     //=====初始化与 LED 连接的硬件接口
	KEY_Init();                     //=====按键初始化
	MiniBalance_PWM_Init(7199,0);   //=====初始化PWM 10KHZ，用于驱动电机
	if(MODE==0)Run_Flag=1;          //=====启动的过程中，根据模式选择开关确定进入位置模式还是速度模式
	else Run_Flag=0;                //=====启动的过程中，根据模式选择开关确定进入位置模式还是速度模式

	OLED_Init();                    //=====OLED初始化
	
	uart_init(72,115200);           //=====串口1初始化，下载 程序和  打印日志
	uart2_init(36,115200);            //=====串口2初始化，连主控   
	uart3_init(36,115200);          //=====串口3初始化  ， 连测距的单片机
 	 
	Encoder_Init_TIM2();            //=====编码器接口
	Encoder_Init_TIM3();            //=====编码器接口
	Encoder_Init_TIM4();            //=====初始化编码器C
	Encoder_Init_TIM5();            //=====初始化编码器D
	Adc_Init();                     //=====adc初始化
	IIC_Init();                     //=====IIC初始化
  MPU6050_initialize();           //=====MPU6050初始化	
  DMP_Init();                     //=====初始化DMP     
	if(KEY==0) Flash_Read();        //=====读取Flash里面的参数
	PS2_Init();											//=====ps2驱动端口初始化
	PS2_SetInit();		 							//=====ps2配置初始化,配置“红绿灯模式”，并选择是否可以修改
  EXTI_Init_R();                    //=====MPU6050 5ms定时中断初始化
  CAN1_Mode_Init(1,2,3,6,0);      //=====CAN初始化


	// 初始化  测距的  结构体
	carDistance.start = 0;  // 代表 还未得到距离值
	carDistance.distanceF = 0;
	carDistance.distanceB = 0;
	carDistance.distanceL1 = 0;
	carDistance.distanceL2 = 0;
	carDistance.leftPositionOK = 0;
	
	jsonParseBuF[0] = '-';
	USART2_jsonParseBuF[0] = '-';



	while(1)
	{		
		if(Flash_Send==1)          //写入PID参数到Flash,由app控制该指令
		{
			Flash_Write();	
			Flash_Send=0;	
		}	
		/*
		if(Flag_Show==0)           //使用MiniBalance APP和OLED显示屏
		{
			APP_Show();	              
			oled_show();             //===显示屏打开
		}*/
		CAN1_SEND();                   //CAN发送	
		PS2_Receive();            //PS2接收
		//USART_TX();                //串口发送
		delay_flag=1;	
		delay_50=0;
		while(delay_flag);	       //通过MPU6050的INT中断实现的50ms精准延时				


		// 解析并更新存储的 四个方向的距离
		AiwacParseDistanceJson();
		//解析并更新 主控下发的指令
		AiwacParseMOVEOrder();

		AiwacSupermarketCarControl(); // 里面会对小车状态进行改变，  必须在  给主控发送  数据前
		//  给主控发小车 的  状态，
		AiwacSendState2Master();


		// 第一次红外测距采集完成
		if ( (carDistance.distanceB != 0) && (carDistance.distanceF != 0) && (carDistance.distanceL1 != 0) && (carDistance.distanceL2 != 0))
		{
			carDistance.start = 1;
		}

		// 500ms 矫正一次  当前位置
		timeNumDistance++;
		if (timeNumDistance == 10)
		{
		//usart2_sendString("~1234",5);
		 //usart2_send('1');
			printf("\r\n F:%f B：%f  L1:%f   L2:%f",carDistance.distanceF, carDistance.distanceB, carDistance.distanceL1, carDistance.distanceL2);


			timeNumDistance = 0;
		}
	} 
}
