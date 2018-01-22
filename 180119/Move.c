#include "Move.h"
#include "Interface.h"
#include <math.h>

#define READ_STATUS(X, Y)   ((Y) = MOTOR_CMD(X))
#define	CHECK_BUSY(X, Y)	while(((Y) = MOTOR_CMD(X)) & 0x01)

unsigned char status1, status2, status3;


// Function  : ��� ������ ����� �����Ѵ�.
// Parameter : ����
// Return    : ����
void StartMotion(void)
{
    WriteCommand(MOTOR0, STT);
    WriteCommand(MOTOR1, STT);
    WriteCommand(MOTOR2, STT);
}


// Function  : ��� ������ ����� �����Ѵ�.
// Parameter : ����
// Return    : ����
void StopMotion(uint16_t mode)
{
    // Motor1 ����
    // LTRJ Ŀ�ǵ� ���
    WriteCommand(MOTOR0, LTRJ);
    // ��Ʈ�� ���� ���
    WriteDataTwoByte(MOTOR0, (1 << mode));

    // Motor2 ����
    // LTRJ Ŀ�ǵ� ���
    WriteCommand(MOTOR1, LTRJ);
    // ��Ʈ�� ���� ���
    WriteDataTwoByte(MOTOR1, (1 << mode));

    // Motor3 ����
    // LTRJ Ŀ�ǵ� ���
    WriteCommand(MOTOR2, LTRJ);
    // ��Ʈ�� ���� ���
    WriteDataTwoByte(MOTOR2, (1 << mode));

    // ��� ����
    WriteCommand(MOTOR0, STT);
    WriteCommand(MOTOR1, STT);
    WriteCommand(MOTOR2, STT);
}

void MOTOR_CTR(long MT0,long MT1,long MT2){
	
	unsigned char i=0;
	double V[3]={MT0,MT1,MT2};

	for(i=0;i<3;++i){
		
		V[i]=V[i]*0.2728;

		SetVelocity(i, V[i]*65536);
	}
	StartMotion();	
}
