#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     gyro,           sensorAnalogInactive)
#pragma config(Sensor, S3,     IR,             sensorI2CCustom)
#pragma config(Sensor, S4,     HTAC,                sensorI2CCustom)
#pragma config(Motor,  motorA,          flag,          tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          jaw,         tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorC,          nonee,         tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     fr,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     br,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     noone,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     none,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     bl,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     fl,            tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    armr,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    arml,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"

//Make sure these drivers and "common.h" are present in *\Robomatter Inc\ROBOTC Development Environment\Includes
#include "hitechnic-gyro.h"
#include "hitechnic-irseeker-v2.h"
#include "hitechnic-accelerometer-float.h"

float TC = 0.1;

float gyrospd = 0;
float gyroheading = 0;
float dt = 0;
float a = 0;

float heading = 0;

float xacc = 0;
float yacc = 0;
float zacc = 0;

float xcal = 0;
float ycal = 0;
float zcal = 0;

//task headingcalc()
//{

//}


task main()
{
	HTGYROstartCal(gyro);
	time1[T1] = 0;
	HTACreadAllAxes(HTAC, xcal, ycal, zcal);
	//StartTask(headingcalc);
	while(true)
	{
		//Getting sensor values
		gyrospd = HTGYROreadRot(gyro);
		HTACreadAllAxes(HTAC, xacc, yacc, zacc);
		xacc -= xcal;
		yacc -= ycal;
		zacc -= zcal;

		dt = time1[T1]; //Getting time passed in ms

		a = 0.909; //TC/(TC + (dt/1000)); //Getting filter coefficient (s)

		gyroheading = heading + gyrospd * (dt/1000); //Jawohl, ein Riemannsumm fur die integrale!

		heading = a*gyroheading + (1-a)*(yacc); //The filter

		nxtDisplayCenteredBigTextLine(3, "%2.0f", heading);

		time1[T1] = 0; //Reset time
	}
}
