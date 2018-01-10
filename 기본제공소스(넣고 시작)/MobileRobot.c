#include <avr/io.h>
#include <util/delay.h>
#include "Interface.h"
#include "Move.h"
#include "Motor.h"


int main(void)
{    

    Interface_init(); //인터페이스 초기화
	
	LM629_HW_Reset();

	MCU_init();	   // MCU 초기화
	
	Motor_init();  // Motor 드라이버 초기화

	while(1){
		MOTOR_CTR(30,0,-30);
		_delay_ms(5000);
		StopMotion(STOP_ABRUPTLY);
		break;
	}   		
}
