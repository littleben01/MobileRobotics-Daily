#include <avr/io.h>
#include <util/delay.h>
#include "Interface.h"
#include "Move.h"
#include "Motor.h"


int main(void)
{    

    Interface_init(); //�������̽� �ʱ�ȭ
	
	LM629_HW_Reset();

	MCU_init();	   // MCU �ʱ�ȭ
	
	Motor_init();  // Motor ����̹� �ʱ�ȭ

	while(1){
		MOTOR_CTR(30,0,-30);
		_delay_ms(5000);
		StopMotion(STOP_ABRUPTLY);
		break;
	}   		
}
