#include "lvd.h"
#include "delay.h"
/**********************************************<+><+><+><+><+>★【robot】★<+><+><+><+><+>************************************************
函数名称：lvd_interrupt
函数功能；低压中断检测中断服务程序
输入参数：无
输出参数：无
***********************************************<<<<<<<<<<<<<<<★【robot】★>>>>>>>>>>>>>>>************************************************/
void lvd_interrupt(void) interrupt 6           
{
	uint f;
	P4=0xfe;
    PCON &= 0xdf;                   //clear LVD flag
	if(PCON & 0x20)
		PCON &= 0xdf;               //clear LVD flag
	for(f=0;f<2;f++)
	{
		P4=0xff;
		delay10ms(5);
		P4=0xfe;
		delay10ms(5);
	}
}
/**********************************************<+><+><+><+><+>★【robot】★<+><+><+><+><+>************************************************
函数名称：lvd_init
函数功能；低压中断检测初始化
输入参数：无
输出参数：无
***********************************************<<<<<<<<<<<<<<<★【robot】★>>>>>>>>>>>>>>>************************************************/
void lvd_init(void)
{
	PCON &= 0xdf;					//clear LVD flag
	if(PCON & 0x20)
		PCON &= 0xdf;				//clear LVD flag
	P4&=0xfe;
	IP &= 0xbf;	 					//PLVD=0;
	IPH|= 0x40;	 					//PLVDH=1;PLVDH:PLVD=1:0低压检测为较高优先级（优先级2）
    P4SW &= 0xbf;                   //Set P4.6 as LVD function pin
    WAKE_CLKO = 0x08;               //enable LVD signal wakeup MCU from power-down mode
    ELVD = 1;                       //enable LVD interrupt
    EA = 1;                         //open global interrupt switch
}
