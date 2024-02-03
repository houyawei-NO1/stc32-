#include "./comm/STC32G.h"  //包含此头文件后，不需要再包含"reg51.h"头文件
#include "./comm/usb.h"     //USB调试及复位所需头文件
#include "intrins.h"
#include "stdio.h"


/****************************** 用户定义宏 ***********************************/

#define MAIN_Fosc       12000000L   //定义主时钟
#define Timer0_Reload   (65536UL -(MAIN_Fosc / 1000))       //Timer 0 中断频率, 3333次/秒 1S/3333=300us  , 1000  1ms

/*****************************************************************************/
#define PWM1_1      0x00	//P:P1.0  N:P1.1
#define PWM1_2      0x01	//P:P2.0  N:P2.1
#define PWM1_3      0x02	//P:P6.0  N:P6.1

#define PWM2_1      0x00	//P:P1.2/P5.4  N:P1.3
#define PWM2_2      0x04	//P:P2.2  N:P2.3
#define PWM2_3      0x08	//P:P6.2  N:P6.3

#define PWM3_1      0x00	//P:P1.4  N:P1.5
#define PWM3_2      0x10	//P:P2.4  N:P2.5
#define PWM3_3      0x20	//P:P6.4  N:P6.5

#define PWM4_1      0x00	//P:P1.6  N:P1.7
#define PWM4_2      0x40	//P:P2.6  N:P2.7
#define PWM4_3      0x80	//P:P6.6  N:P6.7
#define PWM4_4      0xC0	//P:P3.4  N:P3.3

#define ENO1P       0x01
#define ENO1N       0x02
#define ENO2P       0x04
#define ENO2N       0x08
#define ENO3P       0x10
#define ENO3N       0x20
#define ENO4P       0x40
#define ENO4N       0x80

#define PWM5_1      0x00	//P2.0
#define PWM5_2      0x01	//P1.7
#define PWM5_3      0x02	//P0.0
#define PWM5_4      0x03	//P7.4

#define PWM6_1      0x00	//P2.1
#define PWM6_2      0x04	//P5.4
#define PWM6_3      0x08	//P0.1
#define PWM6_4      0x0C	//P7.5

#define PWM7_1      0x00	//P2.2
#define PWM7_2      0x10	//P3.3
#define PWM7_3      0x20	//P0.2
#define PWM7_4      0x30	//P7.6

#define PWM8_1      0x00	//P2.3
#define PWM8_2      0x40	//P3.4
#define PWM8_3      0x80	//P0.3
#define PWM8_4      0xC0	//P7.7

#define ENO5P       0x01
#define ENO6P       0x04
#define ENO7P       0x10
#define ENO8P       0x40


sbit fule_m2 = P6^1;			
sbit fule_m3 = P6^3;
sbit zs_m2=P6^5;
sbit zs_m3=P6^7;
sbit water_m2=P4^4;
sbit water_m3=P4^3;

sbit	LCD_B7  = P2^7;


#define DB07  P2

sbit RS=P7^6;
sbit RW=P7^5;
sbit E=P7^4;

/*************  本地常量声明    **************/
u8  SPN[8]={
	0x53,0x50,0x4e,0x31,0x32,0x33,0x34,0x35};
u8  FMI[8]={
	0x46,0x4D,0x49,0x20,0x20,0x36,0x37,0x38};
u8  hour[8]={
	0x20,0x20,0x20,0x20,0x38,0x2e,0x39,0x68};
u8  power[8]={
	0x20,0x20,0x20,0x31,0x32,0x2e,0x33,0x76};
u8 code border_inf[16] = {
  0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,
  0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa
  };

u8 code string[]={
	0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x35,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,
	0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x34,0x34,0x34,0x34,0x34,0x34,0x34,0x34,
	0xd5,0xd6,0xe3,0xe4,0xe5,0xf3,0xf4,0xf5
};

u8 count=0;
u8 cnt=0;
u8 a=0;




u16 motor_pwm1[48] = {354,374,394,400,410,400,394,374,
				354,286,220,184,148,112,82,36,
				512,476,430,400,364,328,292,226,
				158,138,118,112,102,112,118,138,
				158,226,292,328,364,400,430,476,
				2,36,82,112,148,184,220,288,};
u8 code motor_pwm2[48]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
								   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
								   0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
								   0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
								   0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
								   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};

u8 code motor_pwm3[48]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
								   0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
								   0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
								   0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
								   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
								   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};

u16 motor_pwm4[48] = {
				354,286,220,184,148,112,82,36,//1
				512,476,430,400,364,328,292,226,//2
				158,138,118,112,102,112,118,138,//3
				158,226,292,328,364,400,430,476,//4
				2,36,82,112,148,184,220,288,//5
				354,374,394,400,410,400,394,374,//6
				};

void delay_ms(u16 ms);
void Delay150us();
void Delay10us();		
void init_stepmotor(void);
				
void CheckBusy(void)
{
	u16	i;
	for(i=0; i<5000; i++)	
	{
		if(!LCD_B7)	
			{
			//P35=1;
			break;
			}
		else
			P35=0;
	}		//check the LCD busy or not. With time out
//	while(LCD_B7);			//check the LCD busy or not. Without time out
}
	
void LcdIntWriteCom(u8 com)//不检查忙
{
	u8 com1,com2;
	com1=com&0xf0;
	com2=(com<<4)&0xf0;
	E=0;
	RW=0;
	RS=0;
	DB07=com1;
	Delay10us();
	
	E=1;
	Delay150us();
	E=0;
	
	RW=0;
	RS=0;
	DB07=com2;
	Delay10us();
	
	E=1;
	Delay150us();
	E=0;
	
	DB07=0xff;
}

void LcdWriteCom(u8 com)//检查忙
{
	u8 com1,com2;
	RS = 0;
	RW = 1;
	DB07=0xff;
	Delay150us();
	E = 1;
	CheckBusy();			//check the LCD busy or not.
	E = 0;

	com1=com&0xf0;
	com2=(com<<4)&0xf0;
	E=0;
	RW=0;
	RS=0;
	DB07=com1;
	Delay10us();
	
	E=1;
	Delay150us();
	E=0;
	
	RW=0;
	RS=0;
	DB07=com2;
	Delay10us();
	
	E=1;
	Delay150us();
	E=0;
	
	DB07=0xff;
}

void LcdWriteData(u8 dat)//检查忙
{
	u8 dat1,dat2;
	
	RS = 0;
	RW = 1;
	DB07=0xff;
	Delay150us();
	E = 1;
	CheckBusy();			//check the LCD busy or not.
	E = 0;
	
	dat1=dat&0xf0;
	dat2=(dat<<4)&0xf0;
	E=0;
	RW=0;
	RS=1;
	DB07=dat1;
	Delay10us();
	
	E=1;
	Delay150us();
	E=0;
	
	RW=0;
	RS=1;
	DB07=dat2;
	Delay10us();
	
	E=1;
	Delay150us();
	E=0;
	
	DB07=0xff;
}

void LcdInit()
{

	delay_ms(100);
	LcdIntWriteCom(0x20);//功能设置，四位接线
	delay_ms(10);
	LcdIntWriteCom(0x20);//功能设置，四位接线
	delay_ms(6);
	LcdWriteCom(0x28);//功能设置，四位接线且两行显示
	delay_ms(6);
//	LcdWriteCom(0x08);  // 关闭显示器
	LcdWriteCom(0x01);//清除
	delay_ms(5);
	LcdWriteCom(0x06);//写一个指针加1
	delay_ms(5);
	LcdWriteCom(0x0C);//显示开，不显示光标
	delay_ms(5);
	

	
	delay_ms(50);
}

void Show_string1(u8 *a)
{
	u8 i;
	LcdWriteCom(0x80);
	for(i=0;a[i]!='\0';i++)
	//for(i=0;i<8;i++)
	{
		LcdWriteData(a[i]);
	}
}

void Show_string2(u8 *a)
{
	u8 i;
	LcdWriteCom(0xc0);
	for(i=0;i<8;i++)
	{
		LcdWriteData(a[i]);
	}
}


void main() 
{ 
	u8 i,j;

	EAXFR = 1; 
	//使能访问 XFR 
	CKCON = 0x00; //设置外部数据总线速度为最快 
	WTST = 0x00;
	//设置程序代码等待参数， 
	//赋值为 0 可将 CPU 执行程序的速度设置为最快 
	
	//  Timer0初始化
    AUXR = 0x80;    //Timer0 set as 1T, 16 bits timer auto-reload, 
    TH0 = (u8)(Timer0_Reload / 256);
    TL0 = (u8)(Timer0_Reload % 256);
	ET0 = 1;    //Timer0 interrupt enable
	TR0 = 1;    //Tiner0 run
    
	
	
	P0M0 = 0x00; P0M1 = 0x00; 
	P1M0 = 0x00; P1M1 = 0x00; 
	P2M0 = 0x03; P2M1 = 0x00; //2.4567双向口  2.0 2.1推挽输出
	P3M0 = 0x20; P3M1 = 0x00; //3.5 led
	P4M0 = 0x18; P4M1 = 0x00; //p4.3,p4.4推挽输出
	P5M0 = 0x00; P5M1 = 0x00; 
	P6M0 = 0xff; P6M1 = 0x00;   
	P7M0 = 0x70; P7M1 = 0x00;   
	PWMA_CCER1 = 0x00; //写 CCMRx 前必须先清零 CCERx 关闭通道 
	PWMA_CCER2 = 0x00; //写 CCMRx 前必须先清零 CCERx 关闭通道
	PWMB_CCER1 = 0x00; //写 CCMRx 前必须先清零 CCxE 关闭通道
    PWMB_CCER2 = 0x00;
	
	PWMA_CCMR1 = 0x60; //设置 CC1 为 PWMA 输出模式 
	PWMA_CCMR2 = 0x60; //设置 CC1 为 PWMA 输出模式 
	PWMA_CCMR3 = 0x60;
    PWMA_CCMR4 = 0x60;
	PWMB_CCMR1 = 0x60; //通道模式配置
    PWMB_CCMR2 = 0x60;
    PWMB_CCMR3 = 0x60;
    PWMB_CCMR4 = 0x60;
	
	PWMA_CCER1 = 0x55; //使能 CC1，2,3 通道 ，允许输入，下降沿   //0x01 使能cc1输出，高电平为有效电平    0x03 使能cc1输出，低电平为有效电平
	PWMA_CCER2 = 0x55; //使能 CC2 通道 
	PWMB_CCER1 = 0x55; //配置通道输出使能和极性
    PWMB_CCER2 = 0x55;
	
	PWMA_CCR1H = 0x00;//设置占空比时间 
	PWMA_CCR1L = 0xAA; 
	PWMA_CCR2H = 0x00;//设置占空比时间 
	PWMA_CCR2L = 0xAA; 
	PWMA_CCR3H = 0x00;//设置占空比时间 
	PWMA_CCR3L = 0xAA; 
	PWMA_CCR4H = 0x00;//设置占空比时间 
	PWMA_CCR4L = 0xAA; 
	PWMB_CCR5H = 0x00;//设置占空比时间 
	PWMB_CCR5L = 0xAA; 
	PWMB_CCR6H = 0x00;//设置占空比时间 
	PWMB_CCR6L = 0xAA; 
	
	PWMA_ARRH = 0x02; //设置周期时间 
	PWMA_ARRL = 0x00; 
	PWMB_ARRH = 0x02; //设置周期时间 
    PWMB_ARRL = 0x00; 
	
//	PWMA_ENO = 0x01; //使能 PWM1P 端口输出 
	PWMA_ENO = 0x00; 
	PWMB_ENO = 0x00; 
	PWMA_ENO |= ENO1P; //使能输出 
	PWMA_ENO |= ENO2P; //使能输出
	PWMA_ENO |= ENO3P; //使能输出 
	PWMA_ENO |= ENO4P; //使能输出
	PWMB_ENO |= ENO5P; //使能输出
    PWMB_ENO |= ENO6P; //使能输出
	
	 PWMA_PS = 0x00; //高级 PWM 通道输出脚选择位 
	 PWMA_PS |= PWM1_3; //选择 PWM1-3 通道  p6.0
	 PWMA_PS |= PWM2_3; //选择 PWM2-3 通道  p6.2
 	 PWMA_PS |= PWM3_3; //选择 PWM3-3 通道  p6.4
	 PWMA_PS |= PWM4_3; //选择 PWM4-3 通道  p6.6
	 PWMB_PS |= PWM5_1; //选择 PWM5_1 通道  p2.0
     PWMB_PS |= PWM6_1; //选择 PWM6_1 通道  p2.1
	 
	PWMA_BKR = 0x80; //使能主输出 
	PWMA_CR1 = 0x01; //开始计时 
	PWMB_BKR = 0x80;   //使能主输出
    PWMB_CR1 |= 0x01;  //开始计时
	
	
	EA = 1;     //打开总中断

	
//	PWMB_ENO |= ENO5P; //关闭输出
//    PWMB_ENO |= ENO6P; //关闭输出
//	P2M0 = 0xf3; P2M1 = 0x00; //2.4567  2.0 2.1推挽输出
	init_stepmotor();
//	PWMB_ENO &= ENO5P; //使能输出
//    PWMB_ENO &= ENO6P; //使能输出
//	P2M0 = 0xf0; P2M1 = 0x03; //2.4567推挽输出  2.0 2.1高阻输入

	
	

	delay_ms(1000);
	LcdInit();
	
//	 LcdWriteCom(0x40);//设定CGRAM地址
//	 for(i=0;i<16;i++)//把自定义字符存储到设定地址

//	  {
//	   LcdWriteData(border_inf[i]);
//	   }

	
	while (1)
	{
	
//	LcdWriteCom(0x01);//清除显示	
	delay_ms(1000);	
	
		
	Show_string1(hour);	
	Show_string2(power);	
	delay_ms(3000);
		
	Show_string1(SPN);	
	Show_string2(FMI);
	delay_ms(1000);	
	
	}
}

/********************** Timer0 1ms中断函数 ************************/
void timer0(void) interrupt 1
{

	if(count==0) count = 47;
	PWMA_CCR1H = (u8)(motor_pwm1[count] >> 8);//设置占空比时间 
	PWMA_CCR1L = (u8)(motor_pwm1[count]); 
	fule_m2=(bit)motor_pwm2[count];
	fule_m3=(bit)motor_pwm3[count];
	PWMA_CCR2H = (u8)(motor_pwm4[count] >> 8);//设置占空比时间 
	PWMA_CCR2L = (u8)(motor_pwm4[count]); 
	
	PWMA_CCR3H = (u8)(motor_pwm1[count] >> 8);//设置占空比时间 
	PWMA_CCR3L = (u8)(motor_pwm1[count]); 
	zs_m2=(bit)motor_pwm2[count];
	zs_m3=(bit)motor_pwm3[count];
	PWMA_CCR4H = (u8)(motor_pwm4[count] >> 8);//设置占空比时间 
	PWMA_CCR4L = (u8)(motor_pwm4[count]); 
	
	PWMB_CCR5H = (u8)(motor_pwm1[count] >> 8);//设置占空比时间 
	PWMB_CCR5L = (u8)(motor_pwm1[count]); 
	water_m2=(bit)motor_pwm2[count];
	water_m3=(bit)motor_pwm3[count];
	PWMB_CCR6H = (u8)(motor_pwm4[count] >> 8);//设置占空比时间 
	PWMB_CCR6L = (u8)(motor_pwm4[count]); 

	count --;		

}

void init_stepmotor(void)
{
	
	unsigned int d,e;

	for(d=0;d<5760;d++)
	{
		//------------------------------------------------
		if(a>47) a=0;
		
		PWMA_CCR1H = (u8)(motor_pwm1[a] >> 8);//设置占空比时间 
		PWMA_CCR1L = (u8)(motor_pwm1[a]); 
		fule_m2=(bit)motor_pwm2[a];
		fule_m3=(bit)motor_pwm3[a];
		PWMA_CCR2H = (u8)(motor_pwm4[a] >> 8);//设置占空比时间 
		PWMA_CCR2L = (u8)(motor_pwm4[a]); 
		
		PWMA_CCR3H = (u8)(motor_pwm1[a] >> 8);//设置占空比时间 
		PWMA_CCR3L = (u8)(motor_pwm1[a]); 
		zs_m2=(bit)motor_pwm2[a];
		zs_m3=(bit)motor_pwm3[a];
		PWMA_CCR4H = (u8)(motor_pwm4[a] >> 8);//设置占空比时间 
		PWMA_CCR4L = (u8)(motor_pwm4[a]); 
		
		PWMB_CCR5H = (u8)(motor_pwm1[a] >> 8);//设置占空比时间 
		PWMB_CCR5L = (u8)(motor_pwm1[a]); 
		water_m2=(bit)motor_pwm2[a];
		water_m3=(bit)motor_pwm3[a];
		PWMB_CCR6H = (u8)(motor_pwm4[a] >> 8);//设置占空比时间 
		PWMB_CCR6L = (u8)(motor_pwm4[a]); 
		a++;		
		Delay150us();

	}
	
	
    
}

//========================================================================
// 函数: void delay_ms(u16 ms)
// 描述: 延时函数。
// 参数: ms,要延时的ms数, 这里只支持1~65535ms. 自动适应主时钟.
// 返回: none.
// 版本: VER1.0
// 日期: 2013-4-1
// 备注: 
//========================================================================
void delay_ms(u16 ms)
{
     u16 i;
     do{
          i = MAIN_Fosc / 6000;
          while(--i);
     }while(--ms);
}		
void Delay150us()		//@12.000MHz
{
	unsigned long edata i;

	_nop_();
	_nop_();
	_nop_();
	i = 448UL;
	while (i) i--;
}

void Delay10us()		//@12.000MHz
{
	unsigned long edata i;

	_nop_();
	_nop_();
	_nop_();
	i = 28UL;
	while (i) i--;
}



