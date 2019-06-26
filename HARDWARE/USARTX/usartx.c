
#include "usartx.h"

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

int USART3_IRQHandler(void)
{	
	if(USART3->SR&(1<<5))//���յ�����
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
					case (0): {if (temp == 'D') dState = dD; break;}    //��û����'D' ���ж������Ƿ�Ϊ��D��
					case (dD): {if (temp == ':') {dState = dColon; index =0;}break;}    //��һ����'D'���ж��Ƿ����':'
					case(dColon):   //�Ѿ�����':'��������治�ǿո���������ˡ�
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



