#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdio.h>
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


#define EAST 0
#define NORTH 1
#define WEST 2
#define SOUTH 3

	int tape = 0;
void Turn_and_Drive(double f_agl, int f_speed, int fw_speed, unsigned int mm,int dgree, unsigned int stop, unsigned int wstop);



////////////////////////함수/////////////////////////

//1. 이동각도 (방향)
//2. 이동속도
//3. 회전이동속도 마이너스 일경우 왼쪽회전
//4. 이동거리
//5. 회전이동거리 (회전 각도)
//6. 정지 시점
//7. 회전정지시점

void Turn_and_Drive(double f_agl, int f_speed, int fw_speed, unsigned int mm,int dgree, unsigned int stop, unsigned int wstop)
{
	double distance=0, distanceW=0;
	float S_distance=0, S_distanceW=0;
	unsigned char flg0=0, flg1=0;

	TCNT1H=0xFF; TCNT1L=0x70;
	sec=1;

	acc=5;	accW=3;
	next_speed=f_speed;
	next_wspeed=fw_speed;

	while(1){

		if(sec){
			sec=0;

			S_distance=speed*0.01;	//0.01 순간 이동거리
			S_distanceW=wspeed*0.01;	//0.01 순간 이동거리 

			f_agl=f_agl-S_distanceW;

			if(f_agl<0)f_agl+=360;
			else if(f_agl>=360)f_agl-=360;

			HolonomicW((int)(f_agl),speed,wspeed);

			distance+=S_distance;
			distanceW+=S_distanceW;
			if(distance>=stop && stop!=0)next_speed=100;
			if(fabs(distanceW)>=wstop && wstop!=0){
				next_wspeed=20;
				if(wspeed<=0)next_wspeed=-20;
			}

			if(distance>=mm || (distance*-1)>=mm){
				flg0=1;
				next_speed=0;
				speed=0;
			}
			if(fabs(distanceW)>=dgree){
				flg1=1;
				next_wspeed=0;
				wspeed=0;
			}
		}
		if(flg0 && flg1)
			break;
	}
}

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


void between_fix(){
	sec = 0;
	while(sec!=300){
		non_Holonomic(0,psd_value[2]-psd_value[7],0);
	}
}




unsigned int find_way(){

	while(1){
			if(psd_value[0] > 90 ) return 1;
			if(psd_value[1] < 50 && psd_value[2] < 50) return 2;
			if(READ_SENSOR()&0x08){
			tape = 1;
				while(READ_SENSOR()&0x08){
					HolonomicW(0,200,135-psd_value[8]);
				}
			}
			if(READ_SENSOR()&0x01){
			tape = 2;
				while(READ_SENSOR()&0x01){
					HolonomicW(0,200,135-psd_value[8]);
				}
			}
			HolonomicW(0,200,135-psd_value[8]);
		}
		non_Holonomic(0,0,0);
}
//return 
//1=앞벽감지,그냥 돌면 됨 
//2=구멍 찾음, 보정 후 나가면 됨 



void PUSH(){
	while(psd_value[0]<150){
		non_Holonomic(100,0,0);
	}
	while(psd_value[5]<225){
		non_Holonomic(-100,0,0);
	}

}

void left_fix(){
	sec = 0;
	while(sec != 500){
		non_Holonomic(psd_value[5]-170,psd_value[3]-215,(psd_value[4]-30)-psd_value[5]);
	}
	non_Holonomic(0,0,0);
}

void right_fix(){
	sec = 0;
	while(sec != 500){
		non_Holonomic(-(200-psd_value[5]),-(psd_value[7]-190),(psd_value[4]-30)-psd_value[5]);
	}
}

void display_sensor(){
while(1){
				display_char(0,3,psd_value[0]);
				display_char(1,1,psd_value[1]);
				display_char(2,1,psd_value[2]);
				display_char(3,1,psd_value[3]);
				display_char(3,5,psd_value[4]);
				display_char(3,10,psd_value[5]);
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

	sei();



	while(1){
		if(SW1)
		{
			find_way();
			non_Holonomic(0,0,0);
			if(tape == 1){
				Turn_and_Drive(0,150,-50,225,90,0,0);
				Turn_and_Drive(0,250,-30,700,95,0,0);
				while(psd_value[5]<120){
					non_Holonomic(-150,0,0);
				}
				left_fix();
				while(psd_value[0]<90){
					non_Holonomic(250,0,0);
				}

				non_Holonomic(0,0,0);
			}
			

		}
//1. 이동각도 (방향)
//2. 이동속도
//3. 회전이동속도 마이너스 일경우 왼쪽회전
//4. 이동거리
//5. 회전이동거리 (회전 각도)
//6. 정지 시점
//7. 회전정지시점
int camera = 0;
		if(SW2)
		{
			find_way();
			Turn_and_Drive(0,100,-60,225,90,0,0);
			display_char(0,0,tape);
			Turn_and_Drive(0,200,0,500,0,0,0);
			
			
			if(tape == 1){
				Turn_and_Drive(0,150,50,150,90,0,0);
				while(psd_value[5]<200){
					non_Holonomic(-200,0,0);
				}

				while(psd_value[0]<70){
					non_Holonomic(200,0,0);
				}

				Turn_and_Drive(0,150,-50,235,90,0,0);

				BZ_ON();
				_delay_ms(50);
				BZ_OFF();
				
				sec = 0;
				while(sec!=300){
					camera = Camera_Cmd(2,102);
					non_Holonomic(0,(camera-116)*5,0);
					display_char(0,2,camera);
				}
				PUSH();
				Turn_and_Drive(0,0,-35,0,90,0,0);
				while(psd_value[0]<70){
					non_Holonomic(200,0,0);
				}

				Turn_and_Drive(0,150,50,235,90,0,0);
				sec = 0;
				while(sec!=300){
					camera = Camera_Cmd(1,102);
					non_Holonomic(0,(camera-116)*5,0);
					display_char(0,2,camera);
				}
				PUSH();


			}
			if(tape == 2){
				Turn_and_Drive(0,150,-50,250,90,0,0);
				while(psd_value[5]<200){
					non_Holonomic(-150,0,0);
				}
				left_fix();
				while(psd_value[0]<70){
					non_Holonomic(150,0,0);
				}
				Turn_and_Drive(0,150,-50,235,90,0,0);
			}

			non_Holonomic(0,0,0);
		}


		if(SW3)
		{	
			
			display_sensor();
				
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
