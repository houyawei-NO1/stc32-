#include "./comm/STC32G.h"  //������ͷ�ļ��󣬲���Ҫ�ٰ���"reg51.h"ͷ�ļ�
#include "./comm/usb.h"     //USB���Լ���λ����ͷ�ļ�
#include "intrins.h"
#include "stdio.h"


/****************************** �û������ ***********************************/

#define MAIN_Fosc       12000000L   //������ʱ��
#define Timer0_Reload   (65536UL -(MAIN_Fosc / 1000))       //Timer 0 �ж�Ƶ��, 3333��/�� 1S/3333=300us  , 1000  1ms

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

/*************  ���س�������    **************/
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
	
void LcdIntWriteCom(u8 com)//�����æ
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

void LcdWriteCom(u8 com)//���æ
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

void LcdWriteData(u8 dat)//���æ
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
	LcdIntWriteCom(0x20);//�������ã���λ����
	delay_ms(10);
	LcdIntWriteCom(0x20);//�������ã���λ����
	delay_ms(6);
	LcdWriteCom(0x28);//�������ã���λ������������ʾ
	delay_ms(6);
//	LcdWriteCom(0x08);  // �ر���ʾ��
	LcdWriteCom(0x01);//���
	delay_ms(5);
	LcdWriteCom(0x06);//дһ��ָ���1
	delay_ms(5);
	LcdWriteCom(0x0C);//��ʾ��������ʾ���
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
	//ʹ�ܷ��� XFR 
	CKCON = 0x00; //�����ⲿ���������ٶ�Ϊ��� 
	WTST = 0x00;
	//���ó������ȴ������� 
	//��ֵΪ 0 �ɽ� CPU ִ�г�����ٶ�����Ϊ��� 
	
	//  Timer0��ʼ��
    AUXR = 0x80;    //Timer0 set as 1T, 16 bits timer auto-reload, 
    TH0 = (u8)(Timer0_Reload / 256);
    TL0 = (u8)(Timer0_Reload % 256);
	ET0 = 1;    //Timer0 interrupt enable
	TR0 = 1;    //Tiner0 run
    
	
	
	P0M0 = 0x00; P0M1 = 0x00; 
	P1M0 = 0x00; P1M1 = 0x00; 
	P2M0 = 0x03; P2M1 = 0x00; //2.4567˫���  2.0 2.1�������
	P3M0 = 0x20; P3M1 = 0x00; //3.5 led
	P4M0 = 0x18; P4M1 = 0x00; //p4.3,p4.4�������
	P5M0 = 0x00; P5M1 = 0x00; 
	P6M0 = 0xff; P6M1 = 0x00;   
	P7M0 = 0x70; P7M1 = 0x00;   
	PWMA_CCER1 = 0x00; //д CCMRx ǰ���������� CCERx �ر�ͨ�� 
	PWMA_CCER2 = 0x00; //д CCMRx ǰ���������� CCERx �ر�ͨ��
	PWMB_CCER1 = 0x00; //д CCMRx ǰ���������� CCxE �ر�ͨ��
    PWMB_CCER2 = 0x00;
	
	PWMA_CCMR1 = 0x60; //���� CC1 Ϊ PWMA ���ģʽ 
	PWMA_CCMR2 = 0x60; //���� CC1 Ϊ PWMA ���ģʽ 
	PWMA_CCMR3 = 0x60;
    PWMA_CCMR4 = 0x60;
	PWMB_CCMR1 = 0x60; //ͨ��ģʽ����
    PWMB_CCMR2 = 0x60;
    PWMB_CCMR3 = 0x60;
    PWMB_CCMR4 = 0x60;
	
	PWMA_CCER1 = 0x55; //ʹ�� CC1��2,3 ͨ�� ���������룬�½���   //0x01 ʹ��cc1������ߵ�ƽΪ��Ч��ƽ    0x03 ʹ��cc1������͵�ƽΪ��Ч��ƽ
	PWMA_CCER2 = 0x55; //ʹ�� CC2 ͨ�� 
	PWMB_CCER1 = 0x55; //����ͨ�����ʹ�ܺͼ���
    PWMB_CCER2 = 0x55;
	
	PWMA_CCR1H = 0x00;//����ռ�ձ�ʱ�� 
	PWMA_CCR1L = 0xAA; 
	PWMA_CCR2H = 0x00;//����ռ�ձ�ʱ�� 
	PWMA_CCR2L = 0xAA; 
	PWMA_CCR3H = 0x00;//����ռ�ձ�ʱ�� 
	PWMA_CCR3L = 0xAA; 
	PWMA_CCR4H = 0x00;//����ռ�ձ�ʱ�� 
	PWMA_CCR4L = 0xAA; 
	PWMB_CCR5H = 0x00;//����ռ�ձ�ʱ�� 
	PWMB_CCR5L = 0xAA; 
	PWMB_CCR6H = 0x00;//����ռ�ձ�ʱ�� 
	PWMB_CCR6L = 0xAA; 
	
	PWMA_ARRH = 0x02; //��������ʱ�� 
	PWMA_ARRL = 0x00; 
	PWMB_ARRH = 0x02; //��������ʱ�� 
    PWMB_ARRL = 0x00; 
	
//	PWMA_ENO = 0x01; //ʹ�� PWM1P �˿���� 
	PWMA_ENO = 0x00; 
	PWMB_ENO = 0x00; 
	PWMA_ENO |= ENO1P; //ʹ����� 
	PWMA_ENO |= ENO2P; //ʹ�����
	PWMA_ENO |= ENO3P; //ʹ����� 
	PWMA_ENO |= ENO4P; //ʹ�����
	PWMB_ENO |= ENO5P; //ʹ�����
    PWMB_ENO |= ENO6P; //ʹ�����
	
	 PWMA_PS = 0x00; //�߼� PWM ͨ�������ѡ��λ 
	 PWMA_PS |= PWM1_3; //ѡ�� PWM1-3 ͨ��  p6.0
	 PWMA_PS |= PWM2_3; //ѡ�� PWM2-3 ͨ��  p6.2
 	 PWMA_PS |= PWM3_3; //ѡ�� PWM3-3 ͨ��  p6.4
	 PWMA_PS |= PWM4_3; //ѡ�� PWM4-3 ͨ��  p6.6
	 PWMB_PS |= PWM5_1; //ѡ�� PWM5_1 ͨ��  p2.0
     PWMB_PS |= PWM6_1; //ѡ�� PWM6_1 ͨ��  p2.1
	 
	PWMA_BKR = 0x80; //ʹ������� 
	PWMA_CR1 = 0x01; //��ʼ��ʱ 
	PWMB_BKR = 0x80;   //ʹ�������
    PWMB_CR1 |= 0x01;  //��ʼ��ʱ
	
	
	EA = 1;     //�����ж�

	
//	PWMB_ENO |= ENO5P; //�ر����
//    PWMB_ENO |= ENO6P; //�ر����
//	P2M0 = 0xf3; P2M1 = 0x00; //2.4567  2.0 2.1�������
	init_stepmotor();
//	PWMB_ENO &= ENO5P; //ʹ�����
//    PWMB_ENO &= ENO6P; //ʹ�����
//	P2M0 = 0xf0; P2M1 = 0x03; //2.4567�������  2.0 2.1��������

	
	

	delay_ms(1000);
	LcdInit();
	
//	 LcdWriteCom(0x40);//�趨CGRAM��ַ
//	 for(i=0;i<16;i++)//���Զ����ַ��洢���趨��ַ

//	  {
//	   LcdWriteData(border_inf[i]);
//	   }

	
	while (1)
	{
	
//	LcdWriteCom(0x01);//�����ʾ	
	delay_ms(1000);	
	
		
	Show_string1(hour);	
	Show_string2(power);	
	delay_ms(3000);
		
	Show_string1(SPN);	
	Show_string2(FMI);
	delay_ms(1000);	
	
	}
}

/********************** Timer0 1ms�жϺ��� ************************/
void timer0(void) interrupt 1
{

	if(count==0) count = 47;
	PWMA_CCR1H = (u8)(motor_pwm1[count] >> 8);//����ռ�ձ�ʱ�� 
	PWMA_CCR1L = (u8)(motor_pwm1[count]); 
	fule_m2=(bit)motor_pwm2[count];
	fule_m3=(bit)motor_pwm3[count];
	PWMA_CCR2H = (u8)(motor_pwm4[count] >> 8);//����ռ�ձ�ʱ�� 
	PWMA_CCR2L = (u8)(motor_pwm4[count]); 
	
	PWMA_CCR3H = (u8)(motor_pwm1[count] >> 8);//����ռ�ձ�ʱ�� 
	PWMA_CCR3L = (u8)(motor_pwm1[count]); 
	zs_m2=(bit)motor_pwm2[count];
	zs_m3=(bit)motor_pwm3[count];
	PWMA_CCR4H = (u8)(motor_pwm4[count] >> 8);//����ռ�ձ�ʱ�� 
	PWMA_CCR4L = (u8)(motor_pwm4[count]); 
	
	PWMB_CCR5H = (u8)(motor_pwm1[count] >> 8);//����ռ�ձ�ʱ�� 
	PWMB_CCR5L = (u8)(motor_pwm1[count]); 
	water_m2=(bit)motor_pwm2[count];
	water_m3=(bit)motor_pwm3[count];
	PWMB_CCR6H = (u8)(motor_pwm4[count] >> 8);//����ռ�ձ�ʱ�� 
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
		
		PWMA_CCR1H = (u8)(motor_pwm1[a] >> 8);//����ռ�ձ�ʱ�� 
		PWMA_CCR1L = (u8)(motor_pwm1[a]); 
		fule_m2=(bit)motor_pwm2[a];
		fule_m3=(bit)motor_pwm3[a];
		PWMA_CCR2H = (u8)(motor_pwm4[a] >> 8);//����ռ�ձ�ʱ�� 
		PWMA_CCR2L = (u8)(motor_pwm4[a]); 
		
		PWMA_CCR3H = (u8)(motor_pwm1[a] >> 8);//����ռ�ձ�ʱ�� 
		PWMA_CCR3L = (u8)(motor_pwm1[a]); 
		zs_m2=(bit)motor_pwm2[a];
		zs_m3=(bit)motor_pwm3[a];
		PWMA_CCR4H = (u8)(motor_pwm4[a] >> 8);//����ռ�ձ�ʱ�� 
		PWMA_CCR4L = (u8)(motor_pwm4[a]); 
		
		PWMB_CCR5H = (u8)(motor_pwm1[a] >> 8);//����ռ�ձ�ʱ�� 
		PWMB_CCR5L = (u8)(motor_pwm1[a]); 
		water_m2=(bit)motor_pwm2[a];
		water_m3=(bit)motor_pwm3[a];
		PWMB_CCR6H = (u8)(motor_pwm4[a] >> 8);//����ռ�ձ�ʱ�� 
		PWMB_CCR6L = (u8)(motor_pwm4[a]); 
		a++;		
		Delay150us();

	}
	
	
    
}

//========================================================================
// ����: void delay_ms(u16 ms)
// ����: ��ʱ������
// ����: ms,Ҫ��ʱ��ms��, ����ֻ֧��1~65535ms. �Զ���Ӧ��ʱ��.
// ����: none.
// �汾: VER1.0
// ����: 2013-4-1
// ��ע: 
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



