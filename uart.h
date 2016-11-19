#ifndef __UART_H__ 
#define __UART_H__

#include "stc12c5a60s2.h"
#include "datatype.h"

#define S2RI  0x01          //S2CON.0
#define S2TI  0x02          //S2CON.1

extern void uart1_init(void);
extern void send1_char(uchar txd);
extern void uart2_init(void);
extern void send2_char(uchar txd);

#endif