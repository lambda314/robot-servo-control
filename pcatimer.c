#include "pcatimer.h"

uint T2500us = 0x1200;// 4608

bit  pcatimer_end = 0;
uint PcaValue;

/**********************************************<+><+><+><+><+>★【robot】★<+><+><+><+><+>************************************************
函数名称：pcatimer_init
函数功能：pca定时器初始化，用于舵机2.5ms定时
输入    ：无 
输出    ：无
备注    ：没有开启定时
 **********************************************<<<<<<<<<<<<<<<★【robot】★>>>>>>>>>>>>>>>************************************************/

void pcatimer_init(void)
{
    CCON = 0;                       //Initial PCA control register
                                    //PCA timer stop running
                                    //Clear CF flag
                                    //Clear all module interrupt flag
    CL = 0;                         //Reset PCA base timer
    CH = 0;
    CMOD = 0x00;                    //Set PCA timer clock source as Fosc/12
                                    //Disable PCA timer overflow interrupt
    PcaValue = T2500us;
    CCAP0L = PcaValue;
    CCAP0H = PcaValue >> 8;         //Initial PCA module-0
    PcaValue += T2500us;
    CCAPM0 = 0x49;                  //PCA module-0 work in 16-bit timer mode and enable PCA interrupt

    EA = 1;
	pcatimer_end = 1;

}
/**********************************************<+><+><+><+><+>★【robot】★<+><+><+><+><+>************************************************
函数名称：pcatimer_start
函数功能：同周期pca定时器设置及启动程序，使每变化一个变化量的周期相同	 给寄存器填值
输入    ：无 
输出    ：无
 **********************************************<<<<<<<<<<<<<<<★【robot】★>>>>>>>>>>>>>>>************************************************/

void pcatimer_start(void)
{
    PcaValue = T2500us;
    CCAP0L = PcaValue;
    CCAP0H = PcaValue >> 8;            //Initial PCA module-0
    PcaValue += T2500us;
    CR = 1;                         //PCA timer start run
    EA = 1;
	pcatimer_end = 0;
	
}
/**********************************************<+><+><+><+><+>★【robot】★<+><+><+><+><+>************************************************
函数名称：void pcatimer_Interrupt(void) interrupt 7
函数功能：pca定时器中断服务函数函数	中断一次填一次值 标记一次一个2.5ms到了
输入    ：无
输出    ：无
***********************************************<<<<<<<<<<<<<<<★【robot】★>>>>>>>>>>>>>>>************************************************/

void pcatimer_interrupt() interrupt 7 
{

    CCF0 = 0;                       //Clear interrupt flag
    CCAP0L = PcaValue;
    CCAP0H = PcaValue >> 8;         //Update compare value
    PcaValue += T2500us;
	pcatimer_end = 1; 
}
