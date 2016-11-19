#include "lvd.h"
#include "delay.h"
/**********************************************<+><+><+><+><+>�robot����<+><+><+><+><+>************************************************
�������ƣ�lvd_interrupt
�������ܣ���ѹ�жϼ���жϷ������
�����������
�����������
***********************************************<<<<<<<<<<<<<<<�robot����>>>>>>>>>>>>>>>************************************************/
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
/**********************************************<+><+><+><+><+>�robot����<+><+><+><+><+>************************************************
�������ƣ�lvd_init
�������ܣ���ѹ�жϼ���ʼ��
�����������
�����������
***********************************************<<<<<<<<<<<<<<<�robot����>>>>>>>>>>>>>>>************************************************/
void lvd_init(void)
{
	PCON &= 0xdf;					//clear LVD flag
	if(PCON & 0x20)
		PCON &= 0xdf;				//clear LVD flag
	P4&=0xfe;
	IP &= 0xbf;	 					//PLVD=0;
	IPH|= 0x40;	 					//PLVDH=1;PLVDH:PLVD=1:0��ѹ���Ϊ�ϸ����ȼ������ȼ�2��
    P4SW &= 0xbf;                   //Set P4.6 as LVD function pin
    WAKE_CLKO = 0x08;               //enable LVD signal wakeup MCU from power-down mode
    ELVD = 1;                       //enable LVD interrupt
    EA = 1;                         //open global interrupt switch
}
