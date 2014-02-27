#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     gyro,           sensorAnalogInactive)
#pragma config(Sensor, S3,     IR,             sensorI2CCustom)
#pragma config(Sensor, S4,     HTAC,                sensorI2CCustom)
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

#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.
#include "hitechnic-gyro.h"
#include "hitechnic-irseeker-v2.h"
#include "hitechnic-accelerometer-float.h"

int numass = 10;
int assignment[10][4]; //Assignments array: [n assignments][n parameters]
int curass = 1;

void initializeRobot() {
	HTGYROstartCal(gyro);

	return;
}

void assignmentset(int n, int x, int y, int r, int action) {
	assignment[n][1] = x;
	assignment[n][2] = y;
	assignment[n][3] = r;

	//nothing = 0, IR = 1, armup = 2, armdown = 3, dispose = 4, upslope = 5
	/*switch (action) {
	case "nothing":
		assignment[n][4] = 0;
		break;
	case "IR":
		assignment[n][4] = 1;
		break;
	case "armup":
		assignment[n][4] = 2;
		break;
	case "armdown":
		assignment[n][4] = 3;
		break;
	case "dispose":
		assignment[n][4] = 4;
		break;
	case "upslope":
		assignment[n][4] = 5;
		break;
	}*/
}

void initializeAssignments()
{
	assignmentset(1,10,10,90,0); //(10,10,90) nothing
}


task main()
{
	initializeRobot();
	initializeAssignments();

	waitForStart(); // Wait for the beginning of autonomous phase.

	while(curass <= numass) {

	}
}
