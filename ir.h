#ifndef __IR_H__
#define __IR_H__

#include "stc12c5a60s2.h"
#include "datatype.h"

sbit IR=P3^2;           //��IRλ����ΪP3.2����

extern uchar IrCode[4]; 
extern uint  LowTime;
extern uint  HighTime;
extern void  int0_init(void);
extern bit 	 decode_ir(void);

#endif