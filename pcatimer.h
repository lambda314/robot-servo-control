#ifndef __PCATIMER_H__
#define __PCATIMER_H__

#include "stc12c5a60s2.h"
#include "datatype.h"

extern bit  pcatimer_end;
extern uint PcaValue;
extern void pcatimer_init(void);
extern void pcatimer_start(void);

#endif