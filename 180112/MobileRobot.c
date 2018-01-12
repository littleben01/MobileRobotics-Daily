#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include "Interface.h"
#include "Move.h"
#include "Motor.h"
#include  "Sensor.h"

/*************************************************************************************/

#define BZ_PORT		PORTB
#define BZ			3

#define BZ_ON()		(BZ_PORT |=  (1 << BZ));
#define BZ_OFF() 	(BZ_PORT &= ~(1 << BZ));




#define SW1            (~PINB&0x10)
#define SW2            (~PINB&0x20)
#define SW3            (~PINB&0x40)

#define LED_ON(X)       (PORTB |=  (1<<X))
#define LED_OFF(X)      (PORTB &= ~(1<<X))
#define READ_SENSOR() 		((~(PING>>3)&0x03) | (~PINE&0x1C))

#define PSD       0x01
#define IDT       0x02
#define PHT       0x04
#define EN        0x08
#define DISTANCE  0x10  
#define PSD_Y     0x20   
#define PSD_N     0x40

#define x					0
#define X					0
#define y					1
#define Y					1
#define w					2
#define W					2
#define L					0
#define l					0
#define R					1
#define r					1
#define B                   2
#define b                   2
#define F					3
#define f					3

#define ENX				165.4f
#define ENY				191.1f
#define ENA				415.0f

#define ADC_VREF_TYPE 0x20


void HolonomicW(int f_agl, int f_speed, int fw_speed);

void non_Holonomic(long Fx, long Fy, long Fw);
//�� �ΰ��� �Լ��� ��� �Լ��� ���� 

volatile unsigned int sec=0;
volatile float speed=0, next_speed=0, acc=5, wspeed=0, next_wspeed=0, accW=4;
volatile float disX=0, disY=0, disW=0, disMD=0, speedX=0, speedY=0, speedW=0;



////////////////////////�Լ�/////////////////////////


void Holonomic_distance(int f_agl,int f_speed,unsigned int distance,unsigned stop){
//1.����	2.�ӵ�	3.�Ÿ�	4.��������	
	next_speed=f_speed;
	acc=5;
	TCNT1H=0xFF; TCNT1L=0x70;	//0.01��
	sec=0;
	disMD=0;	//�Ÿ� �ʱ�ȭ
	
	while(1){
	
		HolonomicW(f_agl,speed,0);
	
		if(disMD>=distance) break;
		else if(disMD>=stop) next_speed=50;
	}
}


int LINE(void)
{
	unsigned char SENSOR=0;
	int err=0;
	unsigned char dir;
	sec=0;
	while(1)
	{
		SENSOR=READ_SENSOR();	
		if( SENSOR&0x08 )
		{
			sec=0;
			err=3;
		}
		else if (!(SENSOR&0x08) )err=-15;

		if(sec>50||(SENSOR&0x1C)==0x1C)
		{
			dir=0;
			break;
			
		}		
		if((SENSOR&0x04))
		{	
			dir=2;
			break;
		}
		else if((SENSOR&0x10))
		{
			dir=3;
			break;
		}
		HolonomicW(0,150,err);
	}
	non_Holonomic(0,0,0);
	return dir;
}

int main(void)
{    

    Interface_init(); //�������̽� �ʱ�ȭ

	LM629_HW_Reset();
	MCU_init();	   // MCU �ʱ�ȭ
	Motor_init();  // Motor ����̹� �ʱ�ȭ

	Sensor_init();

	DDRB=0x0F;		//LED, BZ, SW
	TCCR1A=0x00;	// Clock value: 14.400 kHz
	TCCR1B=0x05;

	TCNT1H=0xFF;	//0.01��
	TCNT1L=0x70;
	TIMSK=0x04;

	sei();
	while(1){


		if(SW1)
		{
			LINE();
			non_Holonomic(0,0,100);
			_delay_ms(900);
			non_Holonomic(0,0,0);
			while(!(READ_SENSOR()&0x08)){
				non_Holonomic(0,-150,0);
			}
			LINE();
			non_Holonomic(0,0,-100);
			_delay_ms(900);
			while(!(READ_SENSOR()&0x08)){
				non_Holonomic(0,150,0);
			}			
			LINE();
			non_Holonomic(0,0,100);
			_delay_ms(900);
			while(!(READ_SENSOR()&0x08)){
				non_Holonomic(0,-150,0);
			}
			LINE();
			HolonomicW(0,150,0);
			_delay_ms(500);
			LINE();
			non_Holonomic(0,0,-100);
			_delay_ms(900);
			while(!(READ_SENSOR()&0x08)){
				non_Holonomic(0,150,0);
			}
			LINE();
			HolonomicW(0,150,0);
			_delay_ms(200);
			LINE();
			BZ_ON();
			_delay_ms(100);
			BZ_OFF();
			non_Holonomic(0,0,0);
		}

		if(SW2)
		{	
			Holonomic_distance(0,500,500,250);
			non_Holonomic(0,0,0);

		}

		if(SW3)
		{
	
		}

	}		
}



void HolonomicW(int f_agl, int f_speed, int fw_speed){
	long Fx=0, Fy=0, Fw=0; //�ӵ�	
	double radianA=0, radianB=0;
	int DesiredDirection = 0;
	double V[3]={0,0,0};

	unsigned char i=0;

	// ��ǥ�� ��ȯ
	if(f_agl <= 180) DesiredDirection = 180 - f_agl;
	else if(f_agl > 180) DesiredDirection = 540 - f_agl;
	
	// ���� ��ȯ
	radianA = (180-(double)DesiredDirection) * 0.017453; // 0.017453 = 3.141592 / 180
	radianB = (90-(double)DesiredDirection) * 0.017453;

	
	// 120�� ���� ������ �� �������� �ۿ��ϴ� ���� �������� ���ϱ� ���� �� wheel�� Motor contribution ���ϱ�.. 
	Fx = f_speed * cos(radianA);
	Fy = f_speed * cos(radianB);
	Fw=fw_speed;

	if(f_agl<0 || f_agl>=360){
		if(f_agl<0)Fw = -f_speed;
		else if(f_agl>=360)Fw = f_speed;
		Fx=0;
		Fy=0;
	}
		
	V[0]=((( 0.057*Fx)+(0.033*Fy)+(0.141*Fw)));		//100 1 100 1		?
	V[1]=(((-0.065*Fy)+(0.141*Fw)));				//200 1 100 1       ?
	V[2]=(((-0.057*Fx)+(0.033*Fy)+(0.141*Fw)));		//100 1 100 1
	
    for(i=0;i<3;++i){
		if(V[i]>=40)V[i]=40;
		if(V[i]<=(-40))V[i]=-40;
		SetVelocity(i, V[i]*65536);
	}
	StartMotion();
}

void non_Holonomic(long Fx, long Fy, long Fw){

	unsigned char i=0;
	double V[3]={0,0,0};

	if(Fx==0 && Fy==0 && Fw==0)StopMotion(STOP_ABRUPTLY);

	V[0]=((( 0.056*Fx)+(0.033*Fy)+(0.14*Fw)));
	V[1]=(((-0.065*Fy)+(0.14*Fw)));
	V[2]=(((-0.056*Fx)+(0.033*Fy)+(0.14*Fw)));

	for(i=0;i<3;++i){
		if(V[i]>=40)V[i]=40;
		if(V[i]<=(-40))V[i]=-40;
		SetVelocity(i, V[i]*65536);
	}
	StartMotion();
}


ISR (TIMER1_OVF_vect)
{
	TCNT1H=0xFF; TCNT1L=0x70; //0.01��
	++sec;
	
	disMD+=speed*0.01;
	disW+=speedW*0.01;
	disX+=(speedX*0.01);
	disY+=(speedY*0.01);

	if(next_speed>speed){
		speed+=acc;
		if(next_speed<=speed)speed=next_speed;
	}
	else if(next_speed<speed){
		speed-=acc;
		if(next_speed>=speed)speed=next_speed;
	}
	if(next_wspeed>wspeed){
		wspeed+=accW;
		if(next_wspeed<=wspeed)wspeed=next_wspeed;
	}
	else if(next_wspeed<wspeed){
		wspeed-=accW;
		if(next_wspeed>=wspeed)wspeed=next_wspeed;
	}
}
