
#include "usartx.h"
#include "control.h"
#include <string.h>

/**************************ʵ�ֺ���**********************************************
*��    ��:		usart3����һ���ֽ�
*********************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart3����һ���ַ���
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
�������ܣ�����3��ʼ��
��ڲ�����pclk2:PCLK2 ʱ��Ƶ��(Mhz)    bound:������
����  ֵ����
**************************************************************************/
void uart3_init(u32 pclk2,u32 bound)
{  	 
float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
  mantissa<<=4;
	mantissa+=fraction; 
	

	RCC->APB2ENR|=1<<0;    //��������ʱ��
	RCC->APB2ENR|=1<<4;   //ʹ��PORTC��ʱ��  
	RCC->APB1ENR|=1<<18;  //ʹ�ܴ���ʱ�� 
	GPIOC->CRH&=0XFFFF00FF; 
	GPIOC->CRH|=0X00008B00;//IO״̬����
	GPIOC->ODR|=1<<10;	 
  AFIO->MAPR|=1<<4;      //������ӳ��

	RCC->APB1RSTR|=1<<18;   //��λ����1
	RCC->APB1RSTR&=~(1<<18);//ֹͣ��λ	   	   
	//����������
 	USART3->BRR=mantissa; // ����������	 
	USART3->CR1|=0X200C;  //1λֹͣ,��У��λ.
	//ʹ�ܽ����ж�
	USART3->CR1|=1<<8;    //PE�ж�ʹ��
	USART3->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(0,1,USART3_IRQn,2);//��2��������ȼ� 
}

/**************************************************************************
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/

#define dD 1
#define dColon 2
#define dComma 3

#define oO 1
#define oEnd 2


char startMS = '+';	//����Э��ǰ���ֽ�			#��
u8 startGetMS = 0;		// 0�������ܿ�ʼ��1������  ���ݳ���λ 2����ʼ����json��
int	dataLen = -1;		// json�ַ����ĳ���
u8 jsonBuF[500]; 			// ���жϵ�ʱ�� �洢���յ�json �ַ���
int jsonDataCount = 0;  //��ǰ���յ�  json �ַ�����
u8 jsonParseBuF[500]; 			//������ʱ���� �洢���յ�json �ַ�������ֹ���жϹ���һ��  �ַ��� ��д ������
int uart3GetLen = 0; 


void  USART3StateTo0(void )
{
	// �ָ���ʼ��
	startMS = '+';  //����Э��ǰ���ֽ�          #��
	startGetMS = 0;		// 0�������ܿ�ʼ��1������  ���ݳ���λ 2����ʼ����json��
	dataLen = -1;  		// json�ַ����ĳ���
	jsonDataCount = 0;	//��ǰ���յ�  json �ַ�����
	memset(jsonBuF, 0, sizeof(jsonBuF));
	uart3GetLen = 0; 
}

int USART3_IRQHandler(void)
{	
	if(USART3->SR&(1<<5))//���յ�����
	{	      		
		u8 temp;
	
		temp=USART3->DR;

		// �ж�Э�����ݵĿ�ͷ

		if (startGetMS == 0)
		{
			if (temp == '#')
			{
				startMS = '#';		
				uart3GetLen++; 
			}
			else if ((temp == '!') && (startMS == '#') && (uart3GetLen == 1  )) 
			{
				startGetMS = 1;// Э���־ ǰ���ֽ� ����ok	
			}else if((temp != '!')  && (startMS == '#')  && (uart3GetLen == 1))
			{
				USART3StateTo0();
			}
		}
		else if (startGetMS == 1)// ���� Э������  �� json �ַ����ĳ���
		{
			if (dataLen == -1)
			{
				dataLen = temp*256;
			}else if(dataLen != -1)
			{
				dataLen = dataLen + temp;
				startGetMS =2;				
			}		
		}else if (startGetMS == 2)  // // ��ʼ����  Json ��
		{
			
			jsonBuF[jsonDataCount] = temp;
			jsonDataCount++;
			
			if (jsonDataCount == dataLen)  //  ���ν������
			{

				//usart3_sendString(jsonBuF, dataLen);
				memset(jsonParseBuF, 0, sizeof(jsonParseBuF));

				strcpy(jsonParseBuF,jsonBuF);
				
				USART3StateTo0();
	
			}


			if (jsonDataCount>499)  //  ���ܵĳ������
			{
					USART3StateTo0();

			}
		}
		

	}
			
   
return 0;	
}



