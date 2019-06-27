#ifndef __MY_USRAT_H
#define __MY_USRAT_H
#include "sys.h"	  	
void usart3_send(u8 data);
void usart3_sendString(char *data,u8 len);
void uart3_init(u32 pclk2,u32 bound);
int USART3_IRQHandler(void);

#endif
