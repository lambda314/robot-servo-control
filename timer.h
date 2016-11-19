#ifndef __TIMER_H__
#define __TIMER_H__

#include "stc12c5a60s2.h"
#include "datatype.h"

extern bit timer0_end;
extern void timer0_init(void);
extern void timer0_start(uint THTL);
extern void timer1_init(void);

#endif