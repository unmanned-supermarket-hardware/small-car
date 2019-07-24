#include "usart.h"	
#include "control.h"

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//__use_no_semihosting was requested, but _ttywrch was 
_ttywrch(int ch) 
{ 
ch = ch; 
} 

//重定义fputc函数 
int fputc(int ch, FILE *f)
{      

	while((USART1->SR&0X40)==0);
	USART1->DR = (u8) ch;      
  return ch;
}
#endif 

int Usart_Receive;
/**************************实现函数**********************************************
*功    能:		usart1发送一个字节
*********************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}


/**************************实现函数**********************************************
*功    能:		usart1发送一个字符串
*********************************************************************************/
void usart1_sendString(char *data,u8 len)
{
	int i=0;
	for(i=0;i<len;i++)
	{
		USART1->DR = data[i];
		while((USART1->SR&0x40)==0);	
	}
	
}

void uart_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB2ENR|=1<<14;  //使能串口时钟 
	GPIOA->CRH&=0XFFFFF00F;//IO状态设置
	GPIOA->CRH|=0X000008B0;//IO状态设置
		  
	RCC->APB2RSTR|=1<<14;   //复位串口1
	RCC->APB2RSTR&=~(1<<14);//停止复位	   	   
	//波特率设置
 	USART1->BRR=mantissa; // 波特率设置	 
	USART1->CR1|=0X200C;  //1位停止,无校验位.
	USART1->CR1|=1<<8;    //PE中断使能
	USART1->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(0,1,USART1_IRQn,2);//组2，最低优先级 
}

/**************************************************************************
函数功能：串口1接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART1_IRQHandler(void)
{	
	if(USART1->SR&(1<<5))//接收到数据
	{	      
				u8 temp;
				char strTemp[64];
					static u8 count,last_data,last_last_data,Usart_ON_Count;
					if(Usart_ON_Flag==0)
					{	
						if(++Usart_ON_Count>10)Usart_ON_Flag=1;
					}
					temp=USART1->DR;
				//------------------------------------
				//sprintf(strTemp,"USART1 收到：%c\r\n",temp);
				//usart1_sendString(strTemp,strlen(strTemp));
				//printf(strTemp, "串口1收到数据： %c\r\n",temp);
				if(temp == 'O')
				{
					//usart3_send('O');
				}
				else if(temp =='D')
				{
					usart3_send('D');
				}
//-------------------------				
				   if(Usart_Flag==0)
						{	
						if(last_data==0xfe&&last_last_data==0xff) 
						Usart_Flag=1,count=0;	
						}
					 if(Usart_Flag==1)
						{	
							Urxbuf[count]=temp;     
							count++;                
							if(count==8)Usart_Flag=0;
						}
						last_last_data=last_data;
						last_data=temp;
   }
return 0;	             
}

//////////////////////////////////////////////////////////////////
/**************************实现函数**********************************************
*功    能:		usart2发送一个字节
*********************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}


/**************************实现函数**********************************************
*功    能:		usart2发送一个字符串
*********************************************************************************/
void usart2_sendString(char *data,u8 len)
{
	int i=0;
	for(i=0;i<len;i++)
	{
		USART2->DR = data[i];
		while((USART2->SR&0x40)==0);	
	}
	
}

void uart2_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
  mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB1ENR|=1<<17;  //使能串口时钟 
	GPIOA->CRL&=0XFFFF00FF; 
	GPIOA->CRL|=0X00008B00;//IO状态设置
	GPIOA->ODR|=1<<10;	  
	RCC->APB1RSTR|=1<<18;   //复位串口1
	RCC->APB1RSTR&=~(1<<18);//停止复位	   	   
	//波特率设置
 	USART2->BRR=mantissa; // 波特率设置	 
	USART2->CR1|=0X200C;  //1位停止,无校验位.
	//使能接收中断
	USART2->CR1|=1<<8;    //PE中断使能
	USART2->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(0,1,USART2_IRQn,2);//组2，最低优先级 
}



int USART2_dataLen = -1; // json字符串的长度
u8 USART2_jsonBuF[300]; // 在中断的时候 存储接收的json 字符串
int USART2_jsonDataCount = 0; //当前接收的 json 字符串数
u8 USART2_jsonParseBuF[300]; 
int uart2ByteNum = 0; // 串口2 接收符合协议的字节数目
u8 uart2CRC =0;

void USART2StateTo0(void)
{
	// 恢复初始化
	USART2_dataLen = -1; // json字符串的长度
	memset(USART2_jsonBuF, 0, sizeof(USART2_jsonBuF));
	USART2_jsonDataCount = 0; //当前接收的 json 字符串数
	uart2ByteNum = 0;
	uart2CRC =0;

}


/**************************************************************************
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART2_IRQHandler(void)
{	
	if(USART2->SR&(1<<5))//接收到数据
	{	      
		u8 temp;
		
		temp=USART2->DR;

		// 第一个字节
		if (uart2ByteNum == 0)
		{
			if (temp == '#')
			{
				uart2ByteNum++;
				//printf("\r\n get 1!!");
			}
			return ;
		}
		
		// 第二个字节
		if (uart2ByteNum == 1)
		{
			if (temp == '!')
			{
				uart2ByteNum++;
				//printf("\r\n get 2!!");
				return ;
			}
			else
			{
				uart2ByteNum = 0;
				//printf("\r\n get 2 FAILED!!");
				return ;
			}
		}
		
		
		// 接收 json Len 高字节
		// 第3个字节
		if (uart2ByteNum == 2)
		{
			USART2_dataLen = temp*256;
			uart2ByteNum++;
			return ;
		}
		
		// 接收 json Len 低字节
		// 第4个字节
		if (uart2ByteNum == 3)
		{
			USART2_dataLen = USART2_dataLen + temp;
			uart2ByteNum++;
			//printf("\r\n get 6!!");
			return ;
		}
		
		// 开始接收
		if (uart2ByteNum == 4)
		{
			USART2_jsonDataCount++;
			
			if (USART2_jsonDataCount>250)  //  可能的超出情况
			{
				USART2StateTo0();
				return;
			}
						
			if (USART2_jsonDataCount <= USART2_dataLen)
			{
				USART2_jsonBuF[USART2_jsonDataCount-1] = temp;
				return;
			}


			// 末尾第一次校验标签
			if (USART2_jsonDataCount ==(USART2_dataLen + 1))
				{
					if (temp != '*')
						{
							USART2StateTo0();
							return;
						}
				}


			// 末尾CRC
			if (USART2_jsonDataCount ==(USART2_dataLen + 2))
				{
					uart2CRC = temp;
				}


			// 末尾第二次校验标签
			if (USART2_jsonDataCount ==(USART2_dataLen + 3))
				{
					if (temp != '+')
					{
						USART2StateTo0();
						return;
					}
					else
					{
						if ( uart2CRC == crc8_calculate(USART2_jsonBuF, USART2_dataLen) )
							{
								memset(USART2_jsonParseBuF, 0, sizeof(USART2_jsonParseBuF));
								strcpy(USART2_jsonParseBuF,USART2_jsonBuF);

							}
						
						USART2StateTo0();	
						
					}
		
				}
			return ;
		}

   }
return 0;	
}


/**************************************************************************
函数功能：串口扫描
**************************************************************************/
u8 click_RC (void)
{
			static u8 flag_key=1;//按键按松开标志
	    u8 temp;
			if(flag_key&&Usart_Receive!=0x5A)
			{
			flag_key=0;
		  if(Usart_Receive>=0x01&&Usart_Receive<=0x08)temp=Usart_Receive;
		  else	if(Usart_Receive>=0x41&&Usart_Receive<=0x48)temp=Usart_Receive-0x40;	
		//	else 	temp=0;
			return temp;	// 按键按下
			}
			else if(Usart_Receive==0x5A)			flag_key=1;
			return 0;//无按键按下
}

void USART_TX(void)
{
        u8 Direction_A,Direction_B,Direction_C;
	      u16 Temp_GZ;
	           if(Encoder_A>0) Direction_A=0;
        else if(Encoder_A<0) Direction_A=2;
	      else                 Direction_A=1;
		         if(Encoder_B>0) Direction_B=0;
        else if(Encoder_B<0) Direction_B=2;
	      else                 Direction_B=1;     
		         if(Encoder_C>0) Direction_C=0;
        else if(Encoder_C<0) Direction_C=2;
	      else                 Direction_C=1;
      	Temp_GZ=Gryo_Z+32768;
	
				usart1_send(0xff);		
				usart1_send(0xfe);	
				usart1_send(abs(Encoder_A));		
				usart1_send(Direction_A);	
				usart1_send(abs(Encoder_B));		
				usart1_send(Direction_B);	
				usart1_send(abs(Encoder_C));		
				usart1_send(Direction_C);	
				usart1_send(Temp_GZ>>8);		
				usart1_send(Temp_GZ&0x00ff);	
				
				usart3_send(0xff);		
				usart3_send(0xfe);	
				usart3_send(abs(Encoder_A));		
				usart3_send(Direction_A);	
				usart3_send(abs(Encoder_B));		
				usart3_send(Direction_B);	
				usart3_send(abs(Encoder_C));		
				usart3_send(Direction_C);	
				usart3_send(Temp_GZ>>8);		
				usart3_send(Temp_GZ&0x00ff);	
}
