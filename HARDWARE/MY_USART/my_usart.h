#ifndef __MY_USRAT_H
#define __MY_USRAT_H
#include "sys.h"	  	


#include "my_usart.h"
/*********************************************************************************************************
���︺���Զ�����ͨio��ģ�⴮�ڵ���ش���
��ʱ�Զ�����������,���£�
TX		RX		��ʱ��		�ⲿ�ж�
PC1		PC2		TIM6		EXIT2
PC3		PC5		TIM7		EXIT5

��ʱ��7�ƺ�Ҳ����

�������ԣ�TonyIOT
https://blog.csdn.net/tonyiot/article/details/82502953
**********************************************************************************************************/
#define OI_TXD_1	PCout(1)
#define OI_RXD_1	PCin(2)

#define BuadRate_9600	100

void My_Usart1_Init(void );
	
void USART_Send_1(u8 *buf, u8 len);
	
#endif
