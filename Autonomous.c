#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     gyro,           sensorAnalogInactive)
#pragma config(Sensor, S3,     IR,             sensorI2CCustom)
#pragma config(Motor,  motorA,          flag,          tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          jawsr,         tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorC,          jawsl,         tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     fr,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     br,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     lift,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     bl,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     fl,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    arml,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    armr,                 tServoStandard)

#include "JoystickDriver.c"

//Make sure these drivers and "common.h" are present in *\Robomatter Inc\ROBOTC Development Environment\Includes
#include "hitechnic-gyro.h"
#include "hitechnic-irseeker-v2.h"

int turnspd = 80;
float heading;


void initializeRobot()
{
  HTGYROstartCal(gyro); //Calibrate gyro sensor, make sure robot is still

  return;
}


//Function for turning, use: turnto(heading goal for robot relative to starting position in degrees)
void turnto(float headinggoal)
{
	float rotspd = 0;
	int motorspd = 0;

	//Check which way should be turned and adjust motor direction
	if(heading < headinggoal)
	{motorspd = turnspd;}
	else if(heading > headinggoal)
	{motorspd = -turnspd;}

	//Turn on motors
	motor[fl] = motorspd;
	motor[fr] = motorspd;
	motor[bl] = motorspd;
	motor[br] = motorspd;

	while(heading != headinggoal)
	{
		//This is where the motors actually turn, 1 ms at the time
		wait1Msec(1);

		//Get rotationspeed value from gyrosensor
		rotspd = HTGYROreadRot(gyro);

		//Jawohl, ein Riemannsumm fur die integrale!
		heading += rotspd * 0.001;

		//Display heading on the display for testing
		nxtDisplayCenteredBigTextLine(3, "%2.0f", heading);
	}

	//Turn off motors when the while loop ends, so when the heading = the heading goal
	motor[fl] = 0;
	motor[fr] = 0;
	motor[bl] = 0;
	motor[br] = 0;
}

task main()
{
  initializeRobot(); //Calibrate sensors

  waitForStart(); //Wait for the beginning of autonomous phase
}
