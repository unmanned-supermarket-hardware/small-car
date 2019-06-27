#include "my_usart.h"
/*********************************************************************************************************
���︺���Զ�����ͨio��ģ�⴮�ڵ���ش���
��ʱ�Զ�����������,���£�
TX		RX		��ʱ��		�ⲿ�ж�
PC1		PC2		TIM1		EXIT2
PC3		PC5		TIM6		EXIT5

��ʱ��7�ƺ�Ҳ����

�������ԣ�TonyIOT
https://blog.csdn.net/tonyiot/article/details/82502953
**********************************************************************************************************/
#define OI_TXD	PCout(13)
#define OI_RXD	PBin(14)

#define BuadRate_9600	100

u8 len = 0;	//���ռ���
u8 USART_buf[11];  //���ջ�����

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

void IO_TXD(u8 Data)
{
	u8 i = 0;
	OI_TXD = 0;  
	delay_us(BuadRate_9600);
	for(i = 0; i < 8; i++)
	{
		if(Data&0x01)
			OI_TXD = 1;  
		else
			OI_TXD = 0; 	
		
		delay_us(BuadRate_9600);
		Data = Data>>1;
	}
	OI_TXD = 1;
	delay_us(BuadRate_9600);
}
	
void USART_Send(u8 *buf, u8 len)
{
	u8 t;
	for(t = 0; t < len; t++)
	{
		IO_TXD(buf[t]);
	}
}
	
 void IOConfig(void)
 {
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 	EXTI_InitTypeDef EXTI_InitStruct;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);	 //ʹ��PB,PC�˿�ʱ�� 
	 
	 //SoftWare Serial TXD
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;	    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz	 
  GPIO_Init(GPIOC, &GPIO_InitStructure);	  				
  GPIO_SetBits(GPIOC,GPIO_Pin_13); 						
	 
	 
	//SoftWare Serial RXD
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOB, &GPIO_InitStructure);	 

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);
	EXTI_InitStruct.EXTI_Line = EXTI_Line14;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Falling; //�½��ش����ж�
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStruct);


	NVIC_InitStructure.NVIC_IRQChannel= EXTI15_10_IRQn ; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;  
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  
	NVIC_Init(&NVIC_InitStructure);  
	
}
 
void TIM4_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM4��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���			 
}
 

// int main(void)
// {		
//	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
//	 delay_init(72);
//	 IOConfig();
//  TIM4_Int_Init(107, 71);	 //1M����Ƶ��
//	 
//  while(1)
//	{
//		if(len > 10)
//		{
//			len = 0;
//			USART_Send(USART_buf,11);
//		}
//	}
//}

void EXTI3_10_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line14) != RESET)
	{
		if(OI_RXD == 0) 
		{
			if(recvStat == COM_STOP_BIT)
			{
				recvStat = COM_START_BIT;
				TIM_Cmd(TIM4, ENABLE);
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line14);
	}
}

void TIM1_IRQHandler(void)
{  
	if(TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update);	
		 recvStat++;
		if(recvStat == COM_STOP_BIT)
		{
			TIM_Cmd(TIM4, DISABLE);
			USART_buf[len++] = recvData;	
			return;
		}
		if(OI_RXD)
		{
			recvData |= (1 << (recvStat - 1));
		}else{
			recvData &= ~(1 << (recvStat - 1));
		}	
  }		
}
