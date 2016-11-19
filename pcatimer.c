#include "pcatimer.h"

uint T2500us = 0x1200;// 4608

bit  pcatimer_end = 0;
uint PcaValue;

/**********************************************<+><+><+><+><+>�robot����<+><+><+><+><+>************************************************
�������ƣ�pcatimer_init
�������ܣ�pca��ʱ����ʼ�������ڶ��2.5ms��ʱ
����    ���� 
���    ����
��ע    ��û�п�����ʱ
 **********************************************<<<<<<<<<<<<<<<�robot����>>>>>>>>>>>>>>>************************************************/

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
/**********************************************<+><+><+><+><+>�robot����<+><+><+><+><+>************************************************
�������ƣ�pcatimer_start
�������ܣ�ͬ����pca��ʱ�����ü���������ʹÿ�仯һ���仯����������ͬ	 ���Ĵ�����ֵ
����    ���� 
���    ����
 **********************************************<<<<<<<<<<<<<<<�robot����>>>>>>>>>>>>>>>************************************************/

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
/**********************************************<+><+><+><+><+>�robot����<+><+><+><+><+>************************************************
�������ƣ�void pcatimer_Interrupt(void) interrupt 7
�������ܣ�pca��ʱ���жϷ���������	�ж�һ����һ��ֵ ���һ��һ��2.5ms����
����    ����
���    ����
***********************************************<<<<<<<<<<<<<<<�robot����>>>>>>>>>>>>>>>************************************************/

void pcatimer_interrupt() interrupt 7 
{

    CCF0 = 0;                       //Clear interrupt flag
    CCAP0L = PcaValue;
    CCAP0H = PcaValue >> 8;         //Update compare value
    PcaValue += T2500us;
	pcatimer_end = 1; 
}
