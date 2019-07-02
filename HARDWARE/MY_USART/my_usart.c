#include "my_usart.h"
/*********************************************************************************************************
这里负责自定义普通io口模拟串口的相关代码
暂时自定义两个串口,如下：
TX		RX		定时器		外部中断
PC1		PC2		TIM6		EXIT2
PC3		PC5		TIM7		EXIT5

定时器7似乎也可用

代码来自：TonyIOT
https://blog.csdn.net/tonyiot/article/details/82502953
**********************************************************************************************************/
/*
#define OI_TXD_1	PCout(1)
#define OI_RXD_1	PCin(2)

#define BuadRate_9600	100
*/
u8 len = 0;	//接收计数
u8 USART_buf[11];  //接收缓冲区

enum{
	COM_START_BIT,
	COM_D0_BIT,
	COM_D1_BIT,
	COM_D2_BIT,
	COM_D3_BIT,
	COM_D4_BIT,
	COM_D5_BIT,
	COM_D6_BIT,
	COM_D7_BIT,
	COM_STOP_BIT,
};

u8 recvStat = COM_STOP_BIT;
u8 recvData = 0;

void IO_TXD_1(u8 Data)
{
	u8 i = 0;
	OI_TXD_1 = 0;  
	delay_us(BuadRate_9600);
	for(i = 0; i < 8; i++)
	{
		if(Data&0x01)
			OI_TXD_1 = 1;  
		else
			OI_TXD_1 = 0; 	
		
		delay_us(BuadRate_9600);
		Data = Data>>1;
	}
	OI_TXD_1 = 1;
	delay_us(BuadRate_9600);
}
	
void USART_Send_1(u8 *buf, u8 len)
{
	u8 t;
	for(t = 0; t < len; t++)
	{
		IO_TXD_1(buf[t]);
	}
}
	
 void IOConfig_1(void)
 {
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 	EXTI_InitTypeDef EXTI_InitStruct;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);	 //使能PB,PC端口时钟    //我们这里只用了PC端口，PB也使能了应该问题也不大吧
	 
	 //SoftWare Serial TXD_1 PC1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz	 
  GPIO_Init(GPIOC, &GPIO_InitStructure);	  				
  GPIO_SetBits(GPIOC,GPIO_Pin_1); 						
	 
	 
	//SoftWare Serial RXD_1 PC 2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOC, &GPIO_InitStructure);	 

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);
	EXTI_InitStruct.EXTI_Line = EXTI_Line2;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Falling; //下降沿触发中断
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStruct);


	//PC2 查表值对应总断服务函数为EXTI2_IRQHandler
	NVIC_InitStructure.NVIC_IRQChannel= EXTI2_IRQn ; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;  
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  
	NVIC_Init(&NVIC_InitStructure);  
	
}
 
//
void TIM6_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //时钟使能
	
	//定时器TIM6初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
	TIM_ClearITPendingBit(TIM6, TIM_FLAG_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE ); //使能指定的TIM6中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  //TIM6中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级1级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级1级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器			 
}
 
void My_Usart1_Init()
{
		IOConfig_1();
		TIM6_Int_Init(107, 71);	 //1M计数频率
}
// int main(void)
// {		
//	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
//	 delay_init(72);
//----------------------------
//	 IOConfig();
//  TIM6_Int_Init(107, 71);	 //1M计数频率
//	 
//  while(1)
//	{
//		if(len > 10)
//		{
//			len = 0;
//			USART_Send(USART_buf,11);
//		}
//	}
//----------------------------
//}

void EXTI2_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line2) != RESET)
	{
		if(OI_RXD_1 == 0) 
		{
			if(recvStat == COM_STOP_BIT)
			{
				recvStat = COM_START_BIT;
				TIM_Cmd(TIM6, ENABLE);
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

void TIM6_IRQHandler(void)
{  
	if(TIM_GetFlagStatus(TIM6, TIM_FLAG_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM6, TIM_FLAG_Update);	
		 recvStat++;
		if(recvStat == COM_STOP_BIT)
		{
			TIM_Cmd(TIM6, DISABLE);
			//将字节数据存入buffer
//			USART_buf[len++] = recvData;	
			USART_buf[0] = recvData;	

			//发送该字节
			USART_Send_1(USART_buf, 1);
			usart1_send(recvData);
			return;
		}
		//获得该字节收到的串口数据
		if(OI_RXD_1)
		{
			recvData |= (1 << (recvStat - 1));
		}else{
			recvData &= ~(1 << (recvStat - 1));
		}	
  }		
}
