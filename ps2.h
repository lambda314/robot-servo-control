#ifndef __PS2_H__ 
#define __PS2_H__

#include<stc12c5a60s2.h>
#include"datatype.h"
 
extern uchar  HAND;                       
extern uchar  xdata RES[6]; 
extern uchar  keybuf0;  				//手柄按键编码存储单元
extern uchar  keybuf1;

extern uint t;                                       
               
sbit  DAT = P5^0;                  
sbit  CMND= P5^1;
sbit  ATT = P5^2;
sbit  CLK = P5^3;
  
extern void delay(uchar k);
extern void ps2_out();		//主机接收子程序 
extern void ps2_inout();   
extern void key_scan();		//键扫描


#endif

