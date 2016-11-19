#ifndef __EEPROM_H__
#define	__EEPROM_H__

#include <intrins.h>
#include "stc12c5a60s2.h"
#include "datatype.h"
#include "delay.h"
		   
sbit SDA = P3^4;   //p34 sda
sbit SCL = P3^5;   //p35 scl


#define delay_4nop(); {_nop_();_nop_();_nop_();_nop_();}
#define delay_9nop(); {_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();}
#define addr_write 0xa0          // 器件地址以及写入操作 
#define addr_read  0xa1          // 器件地址以及读取操作 
#define addr_write1 0xa2         // 器件地址1以及写入操作 
#define addr_read1  0xa3         // 器件地址1以及读取操作 

extern void i2c_init(void);
extern void i2c_start(void);
extern void i2c_stop(void);
extern uchar read_eeprom(void);
extern bit write_eeprom(uchar write_data);
extern bit write_eeprom_interrupt(uchar write_datai);
extern void write_byte(uint addr, uchar write_data);
extern void write_byte1(uint addr, uchar write_data);
extern uchar read_current(void);
extern uchar read_current1(void);
extern uchar read_random(uint random_addr);
extern uchar read_random1(uint random_addr);


#endif