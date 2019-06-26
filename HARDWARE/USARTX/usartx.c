
#include "usartx.h"

/**************************实现函数**********************************************
*功    能:		usart3发送一个字节
*********************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}
/**************************实现函数**********************************************
*功    能:		usart3发送一个字符串
*********************************************************************************/
void usart3_sendString(char *data,u8 len)
{
	int i=0;
	for(i=0;i<len;i++)
	{
		USART3->DR = data[i];
		while((USART3->SR&0x40)==0);	
	}
	
}
/**************************************************************************
函数功能：串口3初始化
入口参数：pclk2:PCLK2 时钟频率(Mhz)    bound:波特率
返回  值：无
**************************************************************************/
void uart3_init(u32 pclk2,u32 bound)
{  	 
float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
  mantissa<<=4;
	mantissa+=fraction; 
	

	RCC->APB2ENR|=1<<0;    //开启辅助时钟
	RCC->APB2ENR|=1<<4;   //使能PORTC口时钟  
	RCC->APB1ENR|=1<<18;  //使能串口时钟 
	GPIOC->CRH&=0XFFFF00FF; 
	GPIOC->CRH|=0X00008B00;//IO状态设置
	GPIOC->ODR|=1<<10;	 
  AFIO->MAPR|=1<<4;      //部分重映射

	RCC->APB1RSTR|=1<<18;   //复位串口1
	RCC->APB1RSTR&=~(1<<18);//停止复位	   	   
	//波特率设置
 	USART3->BRR=mantissa; // 波特率设置	 
	USART3->CR1|=0X200C;  //1位停止,无校验位.
	//使能接收中断
	USART3->CR1|=1<<8;    //PE中断使能
	USART3->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(0,1,USART3_IRQn,2);//组2，最低优先级 
}

/**************************************************************************
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/

#define dD 1
#define dColon 2
#define dComma 3

#define oO 1
#define oEnd 2

int USART3_IRQHandler(void)
{	
	if(USART3->SR&(1<<5))//接收到数据
	{	      
				u8 temp;
				u8 myBuffer[64];
				double distance;
				char strTemp[64];
				char strDistance[7];
				static u8 count,last_data,last_last_data,dState,oState,index;
				//count = last_data = last_last_data = dState = oState = index = 0;
				//
				temp=USART3->DR;
				usart1_send(temp);

				//sprintf(strTemp," %d\r\n",dState);
				//usart1_sendString(strTemp,strlen(strTemp));
			
				switch(dState)
				{
					case (0): {if (temp == 'D') dState = dD; break;}    //还没出现'D' ，判断现在是否为‘D’
					case (dD): {if (temp == ':') {dState = dColon; index =0;}break;}    //上一个是'D'，判断是否出现':'
					case(dColon):   //已经出现':'，如果下面不是空格就是数字了。
					{
							if(temp == 'm')
							{
								strDistance[index] = '\0';
								distance = atof(strDistance);
								dState = 0;
								
								sprintf(strTemp,"\r\n%f\r\n",distance);
								usart1_sendString(strTemp,strlen(strTemp));
							}
							else if(temp!=' ')
							{
								strDistance[index] = temp;
								index++;
							}
							break;
					}
				}
				
				
		
				//--------------------------
				last_data=temp;
				last_last_data=last_data;
			
		}
			
   
return 0;	
}



