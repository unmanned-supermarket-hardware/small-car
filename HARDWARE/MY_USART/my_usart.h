#ifndef __MY_USRAT_H
#define __MY_USRAT_H
#include "sys.h"	  	


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
#define OI_TXD_1	PCout(1)
#define OI_RXD_1	PCin(2)

#define BuadRate_9600	100

void My_Usart1_Init(void );
	
void USART_Send_1(u8 *buf, u8 len);
	
#endif
