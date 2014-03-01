#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     gyro,           sensorAnalogInactive)
#pragma config(Sensor, S3,     IR,             sensorI2CCustom)
#pragma config(Sensor, S4,     HTAC,           sensorI2CCustom)
#pragma config(Motor,  motorA,          flag,          tmotorNXT, openLoop, encoder)
#pragma config(Motor,  motorB,          jawsr,         tmotorNXT, openLoop, encoder)
#pragma config(Motor,  motorC,          jawsl,         tmotorNXT, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     fr,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     br,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     lift,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motorG,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     bl,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     fl,            tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    armr,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    arml,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)

void drive45(int millisec) {
	motor[fr] = 0;
	motor[br] = 30;
	motor[bl] = 0;
	motor[fl] = -30;

	wait1Msec(millisec);

	motor[fr] = 0;
	motor[br] = 0;
	motor[bl] = 0;
	motor[fl] = 0;
}

void drive0(int millisec) {
	motor[fr] = -30;
	motor[br] = 30;
	motor[bl] = 30;
	motor[fl] = -30;

	wait1Msec(millisec);

	motor[fr] = 0;
	motor[br] = 0;
	motor[bl] = 0;
	motor[fl] = 0;
}

task main()
{
	drive0(1000);
	while(nNxtButtonPressed == -1) {}
	drive0(1000);
	while(nNxtButtonPressed == -1) {}
	drive0(2000);
	while(nNxtButtonPressed == -1) {}
	drive0(2000);
	while(nNxtButtonPressed == -1) {}
	drive45(1000);
	while(nNxtButtonPressed == -1) {}
	drive45(1000);
}
