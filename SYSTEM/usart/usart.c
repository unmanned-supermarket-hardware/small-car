#include "usart.h"	
#include "control.h"

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//__use_no_semihosting was requested, but _ttywrch was 
_ttywrch(int ch) 
{ 
ch = ch; 
} 

//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      

	while((USART1->SR&0X40)==0);
	USART1->DR = (u8) ch;      
  return ch;
}
#endif 

int Usart_Receive;
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1����һ���ֽ�
*********************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}


/**************************ʵ�ֺ���**********************************************
*��    ��:		usart1����һ���ַ���
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
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  
	RCC->APB2ENR|=1<<14;  //ʹ�ܴ���ʱ�� 
	GPIOA->CRH&=0XFFFFF00F;//IO״̬����
	GPIOA->CRH|=0X000008B0;//IO״̬����
		  
	RCC->APB2RSTR|=1<<14;   //��λ����1
	RCC->APB2RSTR&=~(1<<14);//ֹͣ��λ	   	   
	//����������
 	USART1->BRR=mantissa; // ����������	 
	USART1->CR1|=0X200C;  //1λֹͣ,��У��λ.
	USART1->CR1|=1<<8;    //PE�ж�ʹ��
	USART1->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(0,1,USART1_IRQn,2);//��2��������ȼ� 
}

/**************************************************************************
�������ܣ�����1�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int USART1_IRQHandler(void)
{	
	if(USART1->SR&(1<<5))//���յ�����
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
				//sprintf(strTemp,"USART1 �յ���%c\r\n",temp);
				//usart1_sendString(strTemp,strlen(strTemp));
				//printf(strTemp, "����1�յ����ݣ� %c\r\n",temp);
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
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart2����һ���ֽ�
*********************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}


/**************************ʵ�ֺ���**********************************************
*��    ��:		usart2����һ���ַ���
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
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
  mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  
	RCC->APB1ENR|=1<<17;  //ʹ�ܴ���ʱ�� 
	GPIOA->CRL&=0XFFFF00FF; 
	GPIOA->CRL|=0X00008B00;//IO״̬����
	GPIOA->ODR|=1<<10;	  
	RCC->APB1RSTR|=1<<18;   //��λ����1
	RCC->APB1RSTR&=~(1<<18);//ֹͣ��λ	   	   
	//����������
 	USART2->BRR=mantissa; // ����������	 
	USART2->CR1|=0X200C;  //1λֹͣ,��У��λ.
	//ʹ�ܽ����ж�
	USART2->CR1|=1<<8;    //PE�ж�ʹ��
	USART2->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(0,1,USART2_IRQn,2);//��2��������ȼ� 
}



int USART2_dataLen = -1; // json�ַ����ĳ���
u8 USART2_jsonBuF[300]; // ���жϵ�ʱ�� �洢���յ�json �ַ���
int USART2_jsonDataCount = 0; //��ǰ���յ� json �ַ�����
u8 USART2_jsonParseBuF[300]; 
int uart2ByteNum = 0; // ����2 ���շ���Э����ֽ���Ŀ
u8 uart2CRC =0;

void USART2StateTo0(void)
{
	// �ָ���ʼ��
	USART2_dataLen = -1; // json�ַ����ĳ���
	memset(USART2_jsonBuF, 0, sizeof(USART2_jsonBuF));
	USART2_jsonDataCount = 0; //��ǰ���յ� json �ַ�����
	uart2ByteNum = 0;
	uart2CRC =0;

}


/**************************************************************************
�������ܣ�����2�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int USART2_IRQHandler(void)
{	
	if(USART2->SR&(1<<5))//���յ�����
	{	      
		u8 temp;
		
		temp=USART2->DR;

		// ��һ���ֽ�
		if (uart2ByteNum == 0)
		{
			if (temp == '#')
			{
				uart2ByteNum++;
				//printf("\r\n get 1!!");
			}
			return ;
		}
		
		// �ڶ����ֽ�
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
		
		
		// ���� json Len ���ֽ�
		// ��3���ֽ�
		if (uart2ByteNum == 2)
		{
			USART2_dataLen = temp*256;
			uart2ByteNum++;
			return ;
		}
		
		// ���� json Len ���ֽ�
		// ��4���ֽ�
		if (uart2ByteNum == 3)
		{
			USART2_dataLen = USART2_dataLen + temp;
			uart2ByteNum++;
			//printf("\r\n get 6!!");
			return ;
		}
		
		// ��ʼ����
		if (uart2ByteNum == 4)
		{
			USART2_jsonDataCount++;
			
			if (USART2_jsonDataCount>250)  //  ���ܵĳ������
			{
				USART2StateTo0();
				return;
			}
						
			if (USART2_jsonDataCount <= USART2_dataLen)
			{
				USART2_jsonBuF[USART2_jsonDataCount-1] = temp;
				return;
			}


			// ĩβ��һ��У���ǩ
			if (USART2_jsonDataCount ==(USART2_dataLen + 1))
				{
					if (temp != '*')
						{
							USART2StateTo0();
							return;
						}
				}


			// ĩβCRC
			if (USART2_jsonDataCount ==(USART2_dataLen + 2))
				{
					uart2CRC = temp;
				}


			// ĩβ�ڶ���У���ǩ
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
�������ܣ�����ɨ��
**************************************************************************/
u8 click_RC (void)
{
			static u8 flag_key=1;//�������ɿ���־
	    u8 temp;
			if(flag_key&&Usart_Receive!=0x5A)
			{
			flag_key=0;
		  if(Usart_Receive>=0x01&&Usart_Receive<=0x08)temp=Usart_Receive;
		  else	if(Usart_Receive>=0x41&&Usart_Receive<=0x48)temp=Usart_Receive-0x40;	
		//	else 	temp=0;
			return temp;	// ��������
			}
			else if(Usart_Receive==0x5A)			flag_key=1;
			return 0;//�ް�������
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
