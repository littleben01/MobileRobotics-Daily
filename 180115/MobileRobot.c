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
//이 두가지 함수가 모든 함수의 기초 

volatile unsigned int sec=0;
volatile float speed=0, next_speed=0, acc=5, wspeed=0, next_wspeed=0, accW=4;
volatile float disX=0, disY=0, disW=0, disMD=0, speedX=0, speedY=0, speedW=0;



////////////////////////함수/////////////////////////


void Holonomic_distance(int f_agl,int f_speed,unsigned int distance,unsigned stop){
//1.각도	2.속도	3.거리	4.감속지점	
	next_speed=f_speed;
	acc=5;
	TCNT1H=0xFF; TCNT1L=0x70;	//0.01초
	sec=0;
	disMD=0;	//거리 초기화
	
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
		else if (!(SENSOR&0x08) )err=-3;

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
		if(SENSOR&0x01){
			dir=4;
			break;
		}
		HolonomicW(0,250,err);
	}
	non_Holonomic(0,0,0);
	return dir;
}

int LINE_back(){
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
		else if (!(SENSOR&0x08) )err=-3;

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
		if(SENSOR&0x01){
			dir=4;
			break;
		}
		HolonomicW(0,-250,err);
	}
	non_Holonomic(0,0,0);
	return dir;

}

///제작///

unsigned char LINE_iron(){ //센서 가운데로 모아야함 
	unsigned char SENSOR=0;
	int err = 0;
	while(1){
		SENSOR=READ_SENSOR();
		if(SENSOR&0x01)	   break;
		if((READ_SENSOR()&0x1C)==0x1C)	break;
		

		if		(!(SENSOR&0x04) && (SENSOR&0x08) && !(SENSOR&0x10)) err = 0;
		else if	( (SENSOR&0x04) && (SENSOR&0x08) && !(SENSOR&0x10)) err = -5;
		else if	( (SENSOR&0x04) &&!(SENSOR&0x08) && !(SENSOR&0x10)) err = -10;
		else if	(!(SENSOR&0x04) && (SENSOR&0x08) &&  (SENSOR&0x10)) err = 5;
		else if	(!(SENSOR&0x04) &&!(SENSOR&0x08) &&  (SENSOR&0x10)) err = 10;

		HolonomicW(0,200,err);
	}
	non_Holonomic(0,0,0);
	return 0;
}

void irontracer() 
{
	unsigned char SENSOR=0;
//	int err=0;
	while(1){
		SENSOR=READ_SENSOR();
		if(	(SENSOR&0x1C)==0x1C )	break;
		if( SENSOR&0x01 )	HolonomicW(0,150,5);
		else 				HolonomicW(0,150,-5);
	}
	non_Holonomic(0,0,0);
}


//direction = 1.왼쪽 / 2.오른쪽
void ladder_iron(int direction){ //센서 가운데로 모아야함 

	while(1){
		LINE_iron();

		Holonomic_distance(0,250,150,100);
		if(direction == 1){
			while(!(READ_SENSOR()&0x01))	non_Holonomic(0,0,65);
			while( (READ_SENSOR()&0x01))	non_Holonomic(0,0,65);
				//irontracer가 센서의 왼쪽으로 금속선을 타기 때문에 더 회전해서 왼쪽에 걸쳐줘야 한다
		} else if(direction == 2){
			while(!(READ_SENSOR()&0x01))	non_Holonomic(0,0,-65);
		} 
		irontracer();
		Holonomic_distance(0,250,150,100);
		if(direction == 1){
			while(!(READ_SENSOR()&0x08))	non_Holonomic(0,0,-65);
			non_Holonomic(0,0,-65);
			_delay_ms(50);
			direction = 2;
		} else if (direction == 2){
			while(!(READ_SENSOR()&0x08))	non_Holonomic(0,0,65);
			direction = 1;
		}
	}

}

unsigned char ladder_down(){ //미완성  

	unsigned char SENSOR=0;	
	unsigned char direction=0;	//1=왼쪽 2=가운데 3=오른쪽 
	while(!(SENSOR&0x01)){
		SENSOR = READ_SENSOR();
		LINE();
		speed=250;
		Holonomic_distance(0,250,150,100);
		if(psd_value[0]>165){
			if(psd_value[2]>90)		direction = 3;
			if(psd_value[7]>90)		direction = 1;
		} else direction = 2;

		if(direction == 1)	{
			while(!(SENSOR&0x08))	non_Holonomic(0,0,-65);
		}
		
		if(direction == 3)	{
			while(!(SENSOR&0x08))	non_Holonomic(0,0, 65);	
		}
		
		LINE();
		speed=200;
		Holonomic_distance(0,200,150,100);
		if(direction == 1) 	while(!(SENSOR&0x08))	non_Holonomic(0,0, 65);		
		if(direction == 1)	while(!(SENSOR&0x08))	non_Holonomic(0,0,-65);
	}

	non_Holonomic(0,0,0);
	return 0;

}

unsigned char READ_barcode(unsigned int distance,int f_speed){
	unsigned char count = 0;
	next_speed=f_speed;
	acc=5;
	TCNT1H=0xFF; TCNT1L=0x70;	//0.01초
	sec=0;
	disMD=0;	//거리 초기화
	while(!(disMD>=distance)){
		HolonomicW(0,speed,0);
		if(READ_SENSOR()&0x08) {
			count++;
			while(READ_SENSOR()&0x08);	
		}
	}
	non_Holonomic(0,0,0);
	return count;
}


unsigned char miro (int startx) {
	int x = startx;
	int y = 0;
	while(!(READ_SENSOR()&0x01)){
		if(psd_value[0]>165){
			if( (psd_value[2]>90) && (psd_value[7]>90))	{
			
			}
		}

		

	}
	return 0;
}

int main(void)
{    

    Interface_init(); //인터페이스 초기화

	LM629_HW_Reset();
	MCU_init();	   // MCU 초기화
	Motor_init();  // Motor 드라이버 초기화

	Sensor_init();

	DDRB=0x0F;		//LED, BZ, SW
	TCCR1A=0x00;	// Clock value: 14.400 kHz
	TCCR1B=0x05;

	TCNT1H=0xFF;	//0.01초
	TCNT1L=0x70;
	TIMSK=0x04;

	sei();
	while(1){
			
//		int left,right;

		if(SW1)
		{
			int ladderdirection = 0;
			//ladder_iron(1);
			non_Holonomic(150,0,0);
			_delay_ms(500);	 
			speed = 150;//홀로노믹 distance 사이에 끝김이 없게 하기 위해 넣어야함
			int barcode = READ_barcode(300,150);			
			display_char(0,7,barcode);	
			
			non_Holonomic(150,0,0);
			_delay_ms(500);

			non_Holonomic(0,0,100);
			_delay_ms(900);
			
			while(!((READ_SENSOR()&0x1C)==0x1C))	non_Holonomic(150,0,0);	//센서 전체가 모두 온 되었을때는 ==0x1C를 붙여야 한다. 
			while((READ_SENSOR()&0x1C))		non_Holonomic(150,0,0);
			
			//3개 오른쪽 2개 왼쪽 
			non_Holonomic(0,0,0);
			if(barcode == 2){
				ladderdirection = 1;
				while(!(READ_SENSOR()&0x08)){
					non_Holonomic(0,200,0);
				}
			} else if (barcode == 3){
				ladderdirection = 2;
				while(!(READ_SENSOR()&0x08)){
					non_Holonomic(0,200,0);
				}
				while((READ_SENSOR()&0x08))	non_Holonomic(0,200,0);
			} else {
				BZ_ON();
				_delay_ms(200);
				BZ_OFF();
			}

			ladder_iron(ladderdirection);
			
			non_Holonomic(0,0,0);

		}

		if(SW2)
		{
			LINE();
			speed=200;
			Holonomic_distance(0,200,150,100);
			non_Holonomic(0,0,0);
			while(1){
				display_char(0,5,psd_value[0]);
				display_char(1,2,psd_value[2]);
				display_char(1,8,psd_value[7]);
			}	

		}

		if(SW3)
		{	
//			ladder_down();

			

		}

	}
				
}



void HolonomicW(int f_agl, int f_speed, int fw_speed){
	long Fx=0, Fy=0, Fw=0; //속도	
	double radianA=0, radianB=0;
	int DesiredDirection = 0;
	double V[3]={0,0,0};

	unsigned char i=0;

	// 좌표계 변환
	if(f_agl <= 180) DesiredDirection = 180 - f_agl;
	else if(f_agl > 180) DesiredDirection = 540 - f_agl;
	
	// 라디안 변환
	radianA = (180-(double)DesiredDirection) * 0.017453; // 0.017453 = 3.141592 / 180
	radianB = (90-(double)DesiredDirection) * 0.017453;

	
	// 120도 방향 간격의 휠 방향으로 작용하는 힘의 벡터합을 구하기 위한 각 wheel의 Motor contribution 구하기.. 
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
	TCNT1H=0xFF; TCNT1L=0x70; //0.01초
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
