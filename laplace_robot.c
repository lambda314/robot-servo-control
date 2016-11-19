/***********************************************************************************************************************************
文件名称：robot_test.c
文件内容：
创建人  ：拉普拉斯开源智能机器人
创建时间：2014
************************************************************************************************************************************/

#include<math.h>
#include<intrins.h>
#include"stc12c5a60s2.h"
#include"datatype.h"
#include"delay.h"
#include"ps2.h"
#include"lvd.h"
#include"timer.h"
#include"uart.h" 
#include"eeprom.h"
#include"ir.h"

sbit Fmq = P4^0;
sbit Qdj = P4^1;


#define swap(x,y) {int t; t = x; x = y; y = t;}


uchar xdata position[24]={0};
uchar xdata robotrun[25];
uchar xdata runmore[30][26]; 	 //100 
uchar xdata sort[24];
uchar xdata pick_up[24];
schar xdata changepos[25]={0};
uchar m;
uchar n;

uchar posaim[26];

uint  xdata eeprom_addr1  = 64;
uint  xdata eeprom_addr2  = 5440;
uint  xdata eeprom_addr3  = 10880;
uint  xdata eeprom_addr4  = 16320;
uint  xdata eeprom_addr5  = 21760;
uint  xdata eeprom_addr6  = 27200;
uint  xdata eeprom_addr7  = 0;
uint  xdata eeprom_addr8  = 5440;
uint  xdata eeprom_addr9  = 10880;
uint  xdata eeprom_addr10 = 16320;
uint  xdata eeprom_addr11 = 21760;
uint  xdata eeprom_addr12 = 27200;
			    
bit flag_run=0;
bit flag_ispr=0;
bit flag_initpos=0;
bit flag_runmore=0;
bit flag_swj=0;

/**********************************************<+><+><+><+><+>★【robot】★<+><+><+><+><+>************************************************
函数名称：void low_level_500u
函数功能：PWM信号低电平时间子程序，控制舵机PWM信号的低电平时间决定舵机转动的速度  
输入    ：  time
输出    ：无
***********************************************<<<<<<<<<<<<<<<★【robot】★>>>>>>>>>>>>>>>************************************************/
void low_level_500u(uint time)
{
	uint i; 
	for(i=0;i<time;i++)
	{
  		delay500us(1);
	}
}
/**********************************************<+><+><+><+><+>★【robot】★<+><+><+><+><+>************************************************
函数名称：sort0()
函数功能：排序子程序，将各个口的8位根据时间的长短排序 提供sort[i]=sort[i]- sort[i+1]相邻差值
输入    ：无
输出    ：无
***********************************************<<<<<<<<<<<<<<<★【robot】★>>>>>>>>>>>>>>>************************************************/
void sort0(void)
{ 
	uchar i=0,j=0,x=0;

	pick_up[0]=0xFE;
    pick_up[1]=0xFD;
    pick_up[2]=0xFB;
    pick_up[3]=0xF7;
    pick_up[4]=0xEF;
    pick_up[5]=0xDF;
    pick_up[6]=0xBF;
    pick_up[7]=0x7F;	
	

		
	for(i=0;i<7;i++)
	{    for(j=i+1;j<8;j++)								 
         {
		 	if(sort[i]<sort[j])
			{
				swap(sort[i],sort[j]);
				swap(pick_up[i],pick_up[j]);	  
			}
         }
	}	
	for(i=0;i<7;i++)
    {
		sort[i]= sort[i]- sort[i+1];
    } 		
}

/**********************************************<+><+><+><+><+>★【robot】★<+><+><+><+><+>************************************************
函数名称：sort1()
函数功能：排序子程序，将各个口的8位根据时间的长短排序 提供sort[i]=sort[i]- sort[i+1]相邻差值
输入    ：无
输出    ：无
***********************************************<<<<<<<<<<<<<<<★【robot】★>>>>>>>>>>>>>>>************************************************/
void sort1(void)
{ 
	uchar i=0,j=0,x=0;
	
	pick_up[8]=0xFE;
    pick_up[9]=0xFD;
    pick_up[10]=0xFB;
    pick_up[11]=0xF7;
    pick_up[12]=0xEF;
    pick_up[13]=0xDF;
    pick_up[14]=0xBF;
    pick_up[15]=0x7F;	


	for(i=8;i<15;i++)
	{    for(j=i+1;j<16;j++)
         {
		 	if(sort[i]<sort[j])
			{
				swap(sort[i],sort[j]);
				swap(pick_up[i],pick_up[j]);	  	  
            }
         }
	}	
	for(i=8;i<15;i++)
    {
		sort[i]= sort[i]- sort[i+1];
    } 		
}

/**********************************************<+><+><+><+><+>★【robot】★<+><+><+><+><+>************************************************
函数名称：sort2()
函数功能：排序子程序，将各个口的8位根据时间的长短排序 提供sort[i]=sort[i]- sort[i+1]相邻差值
输入    ：无
输出    ：无
***********************************************<<<<<<<<<<<<<<<★【robot】★>>>>>>>>>>>>>>>************************************************/
void sort2(void)
{ 
	uchar i=0,j=0,x=0;

	pick_up[16]=0xFE;
    pick_up[17]=0xFD;
    pick_up[18]=0xFB;
    pick_up[19]=0xF7;
    pick_up[20]=0xEF;
    pick_up[21]=0xDF;
    pick_up[22]=0xBF;
    pick_up[23]=0x7F;	


	for(i=16;i<23;i++)
	{    for(j=i+1;j<24;j++)
         {
		 	if(sort[i]<sort[j])
			{
				swap(sort[i],sort[j]);
				swap(pick_up[i],pick_up[j]);	  
            }
         }
	}	
	for(i=16;i<23;i++)
    {
		sort[i]= sort[i]- sort[i+1];
    } 		
}
/**********************************************<+><+><+><+><+>★【robot】★<+><+><+><+><+>************************************************																			 
函数功能：单片机初始化程序
函数名称：read_changepos
输入    ：无
输出    : 无
***********************************************<<<<<<<<<<<<<<<★【robot】★>>>>>>>>>>>>>>>************************************************/
void read_changepos(void)
{
	uchar r;
	for(r=0;r<24;r++)
		changepos[r]=(schar)(read_random(32+r));		
	changepos[24] = 0x00;
}
/**********************************************<+><+><+><+><+>★【robot】★<+><+><+><+><+>************************************************
函数名称：pwm24_out
函数功能：24路舵机PWM同时输出
输入    ：无
输出    ：无
***********************************************<<<<<<<<<<<<<<<★【robot】★>>>>>>>>>>>>>>>************************************************/

void pwm24_out(void)
{
	uchar i=0;

	for(i=0;i<24;i++)          
	sort[i]=position[i];

    sort0();                 
	sort1();	   
   	sort2();
	

	timer0_start(0xee00);	  			 	   		   
    P0=0xff;                  
    delay500us(1);            
    for(i=0;i<8;i++)          
    {      	  
		delay8us(sort[7-i]);
        P0=P0&pick_up[7-i];		  
    } 
    while(!timer0_end)
	{
		_nop_();
	}		  	   	
	TR0=0;	 					   
	
	
	timer0_start(0xee00);	 			 	   		   
    P1=0xff;                 
    delay500us(1);            
    for(i=0;i<8;i++)          
    {      	  
		delay8us(sort[15-i]);
        P1=P1&pick_up[15-i];		  
    } 
    while(!timer0_end)
	{
		_nop_();
	}		  	   	
	TR0=0;	 					   
	
	
	timer0_start(0xee00);	  			 	   		   
    P2=0xff;                  
    delay500us(1);           

    for(i=0;i<8;i++)          
    {      	  
		delay8us(sort[23-i]);
        P2=P2&pick_up[23-i];		  

    } 
    while(!timer0_end)
	{
		_nop_();
	}		  	   	
	TR0=0;	 					   
}

/**********************************************<+><+><+><+><+>★【robot】★<+><+><+><+><+>************************************************
函数名称：control_speed_24pwm
函数功能：同时控制24路舵机的速度
输入    ：uchar servo[25]
输出    ：无
***********************************************<<<<<<<<<<<<<<<★【robot】★>>>>>>>>>>>>>>>************************************************/

void control_speed_24pwm(uchar servo[25])
{
		uchar p;
		uchar q;

		xdata uchar div[24];
		xdata uchar rem[24];
		
		read_changepos();

		for(p=0;p<24;p++)
		{
			servo[p]+=changepos[p];			
			if(servo[p]>250)
				servo[p] = 250;
			if(servo[p]<0)
				servo[p] = 0;
		}
					
		for(p=0;p<24;p++)
		{

			if(servo[p]>position[p])
			{
				div[p] = (servo[p]-position[p])/10;
				rem[p] = (servo[p]-position[p])%10;
			}
			if(servo[p]<position[p])
			{
				div[p] = (position[p]-servo[p])/10;
				rem[p] = (position[p]-servo[p])%10;
			}
		}

		for(q=0;q<24;q++)
		{
			if(servo[q]>position[q])
				position[q] += rem[q];			
			if(servo[q]<position[q])
				position[q] -= rem[q];
		}
		pwm24_out();					
		for(p=0;p<10;p++)
		{

			for(q=0;q<24;q++)
			{
				if(servo[q]>position[q])
					position[q] += div[q];
				if(servo[q]<position[q])
					position[q] -= div[q];
			}	
			pwm24_out();
			delay1ms(servo[24]);
		}

} 

/**********************************************<+><+><+><+><+>★【robot】★<+><+><+><+><+>************************************************
函数名称：initial_position();
函数功能：初始位置子程序，根据各个舵机的不同位置设置初始位置
输入    ：无
输出    ：无
***********************************************<<<<<<<<<<<<<<<★【robot】★>>>>>>>>>>>>>>>************************************************/

void position_initial(void)                 
{
	uchar i,j,k,x=0;
	for(i=0;i<30;i++)	  
	{	
		x++;
		for(k=0;k<24;k++)
			position[k]=read_random(k);
			  			
		pwm24_out();
		if(x<10)
        {
			j=20;
			j--;				 
			low_level_500u(j);
		}
		else 
        {	
			j=10;				 
		    j++;
           	low_level_500u(j);
        }				       
	}
}


/**********************************************<+><+><+><+><+>★【robot】★<+><+><+><+><+>************************************************
函数名称：uart1_reinter();
函数功能：串口1中断服务程序
输入    ：无
输出    ：无
***********************************************<<<<<<<<<<<<<<<★【robot】★>>>>>>>>>>>>>>>************************************************/
void uart1_interrupt(void) interrupt 4 
{
	static char n;
	uchar k;	 

	if(RI)		  			
	{   
		RI = 0; 
		posaim[n] = SBUF; 
	 	n++;
		if(n==26)
		{
			n=0; 	

			switch(posaim[25])
			{
				case 0x01:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte(eeprom_addr1+k,posaim[k]);
								eeprom_addr1 += 32;
							}			
							break;
				case 0x02:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte(eeprom_addr2+k,posaim[k]);
								eeprom_addr2 += 32;
							}			
							break;
				case 0x03:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte(eeprom_addr3+k,posaim[k]);
								eeprom_addr3 += 32;
							}			
							break;
				case 0x04:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte(eeprom_addr4+k,posaim[k]);
								eeprom_addr4 += 32;
							}			
							break;
				case 0x05:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte(eeprom_addr5+k,posaim[k]);
								eeprom_addr5 += 32;
							}			
							break;
				case 0x06:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte(eeprom_addr6+k,posaim[k]);
								eeprom_addr6 += 32;
							}			
							break;
				case 0x07:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte1(eeprom_addr7+k,posaim[k]);
								eeprom_addr7 += 32;
							}			
							break;
				case 0x08:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte1(eeprom_addr8+k,posaim[k]);
								eeprom_addr8 += 32;
							}			
							break;
				case 0x09:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte1(eeprom_addr9+k,posaim[k]);
								eeprom_addr9 += 32;
							}			
							break;
				case 0x0a:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte1(eeprom_addr10+k,posaim[k]);
								eeprom_addr10 += 32;
							}			
							break;
				case 0x0b:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte1(eeprom_addr11+k,posaim[k]);
								eeprom_addr11 += 32;
							}			
							break;
				case 0x0c:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte1(eeprom_addr12+k,posaim[k]);
								eeprom_addr12 += 32;
							}			
							break;
				case 0xea:									//舵机偏差
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<24;k++)
								write_byte(32+k,(uchar)posaim[k]);
							}
/********************************************/
				case 0xeb:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte(eeprom_addr1+k,0xff);
							}			
							break;
				case 0xec:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte(eeprom_addr2+k,0xff);
							}			
							break;
				case 0xed:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte(eeprom_addr3+k,0xff);
							}			
							break;
				case 0xef:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte(eeprom_addr4+k,0xff);
							}			
							break;
				case 0xf0:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte(eeprom_addr5+k,0xff);
							}			
							break;
				case 0xf1:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte(eeprom_addr6+k,0xff);
							}			
							break;
				case 0xf2:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte1(eeprom_addr7+k,0xff);
							}			
							break;
				case 0xf3:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte1(eeprom_addr8+k,0xff);
							}			
							break;
				case 0xf4:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte1(eeprom_addr9+k,0xff);
							}			
							break;
				case 0xf5:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte1(eeprom_addr10+k,0xff);
							}			
							break;
				case 0xf6:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte1(eeprom_addr11+k,0xff);
							}			
							break;
				case 0xf7:
							{
								RI = 0;
								posaim[25] = 0x00;
								for(k=0;k<26;k++)
								write_byte1(eeprom_addr12+k,0xff);
							}			
							break;

/********************************************/
				case 0xf8:
							{
							   RI = 0;
							   posaim[25] = 0x00;
							   flag_swj = 0;
							}
							break;		

				case 0xf9:
							{
							   RI = 0;
							   posaim[25] = 0x00;
							   flag_swj = 1;
							}
							break;		
				case 0xfa:										  //发送初始位置到上位机
					 		{
								RI = 0;
								posaim[25]=0x00;
								flag_initpos=1;																								
							}
							break;
				case 0xfb:										  //在线运行动作
					 		{
								RI = 0;
								posaim[25]=0x00;
								flag_ispr=1;				
					 		}
				case 0xfc:										  //在线动作组运行动作
					 		{
								RI = 0;
								posaim[25]=0x00;
								for(n=0;n<25;n++)
									runmore[m][n]=posaim[n];
								m++;
								if(posaim[0]==0xff)
								{
									flag_runmore=1;	
									break;
								}			
					 		}
							break;
				case 0xfd:										  //在线调试动作
					 		{
								RI = 0;
								posaim[25]=0x00;
								flag_run=1;				
					 		}
							break;
				case 0xfe:										  //初始位置下载到AT24256 0
					 		{
								RI = 0;
								posaim[25]=0x00;
								for(k=0;k<26;k++)
									write_byte(k,posaim[k]);					    															
							}
							break;			
				default:
					break;

			} 	
		
		} 
	} 	
}

/**********************************************<+><+><+><+><+>★【robot】★<+><+><+><+><+>************************************************																			 
函数功能：单片机初始化程序
函数名称：mcu_init
输入    ：无
输出    ：无
***********************************************<<<<<<<<<<<<<<<★【robot】★>>>>>>>>>>>>>>>************************************************/
void mcu_init(void)
{ 											  
	timer0_init();
	//timer1_init();
	uart1_init();
	//uart2_init();
	//int0_init();
    i2c_init();
	lvd_init();
	Fmq = 0;
	Qdj = 0;					
	delay500us(1);
	read_changepos();
}
/*****************************************************************************************************************************************
函数名称：main
函数功能：入口函数
输入    ：无
输出    ：无
*****************************************************************************************************************************************/
void main(void)
{   

	uint s;
	uint read_addr;

    SP=0x70;     			  
	P0M1 = 0x00;
	P0M0 = 0x00;  
   	P1M1 = 0x00;
	P1M0 = 0x00;   
	P2M1 = 0x00;
	P2M0 = 0x00;  
	  
	mcu_init();

    P0 = 0xff;
	P1 = 0xff;
	P2 = 0xff;
	delay500ms(1);
	position_initial();
	delay500ms(1);

	while(1)
	{
 
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*====================================如果调试请按遥控器START按钮===============================*/	

		if(flag_ispr)
		{
			flag_ispr=0;
			control_speed_24pwm(posaim);
		} 
		if(flag_run)  
		{
			
			flag_run=0;
			control_speed_24pwm(posaim);
		}
		if(flag_initpos)
		{
			flag_initpos=0;

			send1_char(0xfb);		  				//以前发的是0xfe，由于断电的时候可能存在干扰，单片机自己发送0xfe;
			for(s=0;s<24;s++)									
				send1_char(read_random(s));
		}

		if(flag_runmore)
		{
			flag_runmore=0;
			for(n=0;n<30;n++)
			{
				for(m=0;m<25;m++)
					robotrun[m] = runmore[n][m];
				if(robotrun[0]==0xff)
					break;
				control_speed_24pwm(robotrun);
			}
			m=0;
			n=0;
		}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*=================================+++++++++++++===========================================*/

		if(!flag_swj)
		{
		    for(s=0;s<3;s++)	
			{
	    		key_scan();
	
			    send1_char(4);
				send1_char(keybuf0);
				send1_char(5);
				send1_char(keybuf1);
					
			}
			if((keybuf0==0xfe)||(Qdj))		   //SELECT
			{			
				for(read_addr=5440;read_addr<10880;read_addr+=32)
				{
					for(s=0;s<25;s++)
						robotrun[s]=read_random(read_addr+s);
					if(robotrun[0]==0xff) 
						break;
					else
						control_speed_24pwm(robotrun);
				}
				for(read_addr=64;read_addr<5440;read_addr+=32)
				{
					for(s=0;s<25;s++)
						robotrun[s]=read_random(read_addr+s);
					if(robotrun[0]==0xff) 
						break;
					else
					control_speed_24pwm(robotrun);
				}
				for(read_addr=5440;read_addr<10880;read_addr+=32)
				{
					for(s=0;s<25;s++)
						robotrun[s]=read_random1(read_addr+s);
					if(robotrun[0]==0xff) 
						break;
					else
						control_speed_24pwm(robotrun);
				}
				for(read_addr=0;read_addr<5440;read_addr+=32)
				{
					for(s=0;s<25;s++)
						robotrun[s]=read_random1(read_addr+s);
					if(robotrun[0]==0xff) 
						break;
					else
						control_speed_24pwm(robotrun);
				}							 
		
			}  
			switch(keybuf0)
			{
				case 0xef :

							for(read_addr=10880;read_addr<16320;read_addr+=32)
							{
								for(s=0;s<25;s++)
									robotrun[s]=read_random(read_addr+s);
								if(robotrun[0]==0xff) 
									break;
								else
									control_speed_24pwm(robotrun);
							}
							break;
				case 0xdf :
							for(read_addr=16320;read_addr<21760;read_addr+=32)
							{
								for(s=0;s<25;s++)
									robotrun[s]=read_random(read_addr+s);
								if(robotrun[0]==0xff) 
									break;
								else
									control_speed_24pwm(robotrun);
							}
							break;
				case 0xbf:
							for(read_addr=21760;read_addr<27200;read_addr+=32)
							{
								for(s=0;s<25;s++)
										robotrun[s]=read_random(read_addr+s);
								if(robotrun[0]==0xff)
									break;
								else
									control_speed_24pwm(robotrun);
							}
							break;
				case 0x7f:
							for(read_addr=27200;read_addr<32640;read_addr+=32)
							{
								for(s=0;s<25;s++)
										robotrun[s]=read_random(read_addr+s);
								if(robotrun[0]==0xff) 
									break;
								else
									control_speed_24pwm(robotrun);
							}
							break;
	 
								 
	
				default:
						break;
							
			}	 
	 		switch(keybuf1)
			{
				case 0xfe :
							for(read_addr=64;read_addr<5440;read_addr+=32)
							{
								for(s=0;s<25;s++)
										robotrun[s]=read_random(read_addr+s);
								if(robotrun[0]==0xff) 
									break;
								else
									control_speed_24pwm(robotrun);
							}
							break;
				case 0xfb :
							for(read_addr=5440;read_addr<10880;read_addr+=32)
							{
								for(s=0;s<25;s++)
										robotrun[s]=read_random(read_addr+s);
								if(robotrun[0]==0xff) break;
								else
								control_speed_24pwm(robotrun);
							}
							break;
				case 0xfd:
							for(read_addr=0;read_addr<5440;read_addr+=32)
							{
								for(s=0;s<25;s++)
									robotrun[s]=read_random1(read_addr+s);
						
								if(robotrun[0]==0xff) break;
								else
								control_speed_24pwm(robotrun);
							}
							break;
				case 0xf7:
							for(read_addr=5440;read_addr<10880;read_addr+=32)
							{
								for(s=0;s<25;s++)
										robotrun[s]=read_random1(read_addr+s);
								if(robotrun[0]==0xff) break;
								else
								control_speed_24pwm(robotrun);
							}
							break;
				case 0xef:
							for(read_addr=10880;read_addr<16320;read_addr+=32)
							{
								for(s=0;s<25;s++)
									robotrun[s]=read_random1(read_addr+s);

								if(robotrun[0]==0xff) break;
								else
								control_speed_24pwm(robotrun);
							}
							break;
	
				case 0xdf:
							for(read_addr=16320;read_addr<21760;read_addr+=32)
							{
								for(s=0;s<25;s++)
										robotrun[s]=read_random1(read_addr+s);
								if(robotrun[0]==0xff) break;
								else
								control_speed_24pwm(robotrun);
							}
							break;
	
				case 0xbf:
							for(read_addr=21760;read_addr<27200;read_addr+=32)
							{
								for(s=0;s<25;s++)
										robotrun[s]=read_random1(read_addr+s);
								if(robotrun[0]==0xff) break;
								else
								control_speed_24pwm(robotrun);
							}
							break;
						 
				case 0x7f:
							for(read_addr=27200;read_addr<32640;read_addr+=32)
							{
								for(s=0;s<25;s++)
										robotrun[s]=read_random1(read_addr+s);
								if(robotrun[0]==0xff) break;
								else
								control_speed_24pwm(robotrun);
							}
							break;
	
				default:
						break;
							
			}
		} 
	} 


}


 

