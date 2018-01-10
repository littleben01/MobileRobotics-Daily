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




/////////////////////////////////////////////////만든 함수 ////////////////////////////////////////////////

//전진하다가 앞벽감지시 멈추는 함수 
void Holonomic_psd(int sensor,int length,int speed){
	while(psd_value[sensor]<length){
		HolonomicW(0,speed,0);
	}
	non_Holonomic(0,0,0);
}

//왼쪽, 뒤쪽 벽 맞춰서 위치 보정하는 함수 
void left_fix(int length){
	while(1){
		display_char(1,0,psd_value[1]);
		display_char(2,0,psd_value[3]);
		display_char(3,5,psd_value[4]);
		display_char(3,8,psd_value[5]);
		non_Holonomic(psd_value[4]-250,psd_value[1]-(length),(psd_value[1]+52)-psd_value[3]);
		_delay_ms(10);
		
		if((((psd_value[1]<137)&&(psd_value[1]>135))&&((psd_value[3]<188)&&(psd_value[3]>186)))&&((psd_value[4]<250)&&(psd_value[4]>247))&&((psd_value[5]>220)&&(psd_value[5]<223))){
			BZ_ON();
			_delay_ms(100);
			BZ_OFF();
			BZ_ON();
			_delay_ms(100);
			BZ_OFF();
			non_Holonomic(0,0,0);
			return;
		}

	}
}

//벽타는 함수
// mode - 모드1:왼쪽벽 모드 / 2:오른쪽 벽 
// end  - 1:앞벽 감지시 멈춤 / 2:타던 벽 사라질 시 멈춤
void wall_follow(int mode, int end){
	if(mode==1){
		while(1){
			if(psd_value[0] < 90 && end == 1) break;
			if(psd_value[1] < 100 && end == 2) break;
			display_char(0,0,psd_value[1]);
			HolonomicW(0,200,psd_value[1]-135);
		}
		non_Holonomic(0,0,0);
	} else {
		while(1){
			if(psd_value[0] < 90 && end == 1) break;
			if(psd_value[8] < 100 && end == 2) break;
			display_char(0,0,psd_value[8]);
			HolonomicW(0,200,135-psd_value[8]);
		}
		non_Holonomic(0,0,0);
	}
}

//뒷벽 타는 함수 
//먼저 뒷벽에 대해 위치를 보정하고 옆으로 움직임 
void backwall_follow(int speed){
	while(1){
		display_char(3,5,psd_value[4]);
		display_char(3,8,psd_value[5]);
		if((psd_value[5]>216)&&(psd_value[5]<220)) break;
		non_Holonomic(psd_value[5]-217,0,(psd_value[4])-(psd_value[5]+30));
	}
//	while(1){
//		if(psd_value[7]>150) break;
//		
//		display_char(3,5,psd_value[4]);
//		display_char(3,8,psd_value[5]);
//		non_Holonomic(0,speed,(psd_value[4])-(psd_value[5]+20));
//		_delay_ms(10);
//
//	}
	BZ_ON();
	_delay_ms(100);
	BZ_OFF();
	non_Holonomic(0,0,0);
	while(1){
		if(psd_value[7]>150) break;
		non_Holonomic(psd_value[5]-215,speed,(psd_value[4])-(psd_value[5]+30));
	}
	non_Holonomic(0,0,0);
	BZ_ON();
	_delay_ms(100);
	BZ_OFF();
}

//두개의 벽 사이를 지나가는 함수 
void Holonomic_BW(){
	int direction=0;
	while(1){
		display_char(0,0,psd_value[1]);
		display_char(0,5,psd_value[8]);
		non_Holonomic(200,direction,0);
		while(psd_value[1]>100){
			non_Holonomic(200,150,0);
		}
		while(psd_value[8]>80){
			non_Holonomic(200,-150,0);
		}
	}
}

//벽을 빠꾸치면서 타는 함수
void wall_reverse(int mode, int end){
	if(mode==1){
		while(1){
			display_char(0,0,psd_value[3]);
			if(((psd_value[4] > 150)&&(psd_value[5] > 150)) && end == 1) break;
//			if(psd_value[1] < 100 && end == 2) break;
			
			non_Holonomic(-200,0,150-psd_value[3]);
//			HolonomicW(0,-200,psd_value[3]-225);
		}
		non_Holonomic(0,0,0);
	} else {
		while(1){
			if(psd_value[0] < 90 && end == 1) break;
			if(psd_value[8] < 100 && end == 2) break;
			display_char(0,0,psd_value[8]);
			HolonomicW(0,200,140-psd_value[1]);
		}
		non_Holonomic(0,0,0);
	}
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
//	lcd_display_str(0,0,"START");
	sei();
	while(1){


		if(SW1)
		{
			//curve(90,400,10);
//			while(1){
//				display_char(1,0,psd_value[1]);
//				display_char(2,0,psd_value[3]);
				//1번에 50 보정
				//5번에 25 보정 
//			}
			while(1){
				display_char(1,0,psd_value[1]);
				display_char(2,0,psd_value[3]);
				display_char(3,5,psd_value[4]);
				display_char(3,8,psd_value[5]);
//				non_Holonomic(0,psd_value[1]-135,(psd_value[1]+52)-psd_value[3]);	
				_delay_ms(10);	
			}
		}

		if(SW2)
		{	
//			while(psd_value[0]<70){
//				HolonomicW(0,200,0);
//			}
//			non_Holonomic(0,0,0);
			left_fix(135);

		}

		if(SW3)
		{
//			Holonomic_psd(0,70,200);

			
//			wall_follow(2,2);
			
//			backwall_follow(200);
			
//			Holonomic_BW();					
			
			wall_reverse(1,1);		
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



