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

//Assignments: change numass and assignment[...][] to number of assignments
int numass = 1;
int assignment[1][4]; //Assignments array: [n assignments][n parameters(don't touch)]

//Run Calibrate.c and enter distance travelled here in cm
float d0_1s_1 = 32; //Straight, one second
float d0_1s_2 = 32;
float d0_2s_1 = 70; //Straight, two seconds
float d0_2s_2 = 70;
float d45_1s_1 = 26; //At 45 degrees, one second
float d45_1s_2 = 26;

//Correction constants (don't touch)
float c_d = 0.81;
float c_t = 2.18;

float distancecm;

//Movement
float alfa;
float beta;
float xcur;
float ycur;
float rcur;



//Filling assignmentarray: don't touch
void assignmentSet(int n, int x, int y, int r, int action) {
	assignment[n][0] = x;
	assignment[n][1] = y;
	assignment[n][2] = r;
	assignment[n][3] = action;
}

//Assignments for the robot: change this
void initializeAssignments()
{
	//assignmentset(n,x,y,r,action), nothing = 0, seekIR = 1, armup = 2, armdown = 3, dispose = 4, upslope = 5
	assignmentSet(1,10,10,90,0); //(10,10,90) nothing
}

//Calculating correction constants: don't touch
void distanceCalibration() {
	c_d = ((d45_1s_1+d45_1s_2)/2)/((d0_1s_1+d0_1s_2)/2);
	c_t = 0.5*(((d0_2s_1+d0_2s_2)/2)/((d0_1s_1+d0_1s_2)/2));
}

//Initialization: don't touch
void initializeRobot() {
	HTGYROstartCal(gyro);
	distanceCalibration();
	initializeAssignments();

	return;
}

void slitherto(int xgoal, int ygoal, int rgoal) {
	float betacor;

	float spd = 30;

	float _x1;
	float _y1;
	float _x2 = 0;

	float rotspeed;

	beta = atan2(ygoal-ycur,xgoal-xcur);
	betacor = beta + degreesToRadians(rcur);

	_x1 = cos(betacor);
	_y1 = sin(betacor);

	time1[T1] = 0;

	while (round(xcur) != xgoal && round(ycur) != ygoal && round(rcur) != rgoal) {
		if (round(rgoal) == rcur) {
			_x2 = 0;
		}
		else if (rgoal > rcur && rgoal - rcur <= 180)	{
			if (rgoal > rcur + 5)
				_x2 = 1;
			else
				_x2 = 0.3;
		}
		else if (rgoal < rcur && rcur - rgoal > 180)	{
			if (rgoal < rcur - 5)
				_x2 = 1;
			else
				_x2 = 0.3;
		}
		else	{
			if (abs(rgoal - rcur) > 5 )//?
				_x2 = -1;
			else
				_x2 = -0.3;
		}

		if (beta/1.571 - floor(beta/1.571) > 0.785) //Oh radians....
			alfa = 1.571 - (beta/1.571 - floor(beta/1.571));
		else
			alfa = beta/1.571 - floor(beta/1.571);

		wait1Msec(20);

		distancecm = ((1-c_d)/(-0.785*alfa) + 1)*c_t*time1[T1]*0.001;

		xcur += distancecm*cos(beta);
		ycur += distancecm*sin(beta);

		rotspeed = HTGYROreadRot(gyro); //Read the current rotation speed
		rcur += rotspeed * time1[T1]*0.001; //Magic
		nxtDisplayCenteredBigTextLine(3, "%2.0f", rcur); //Display our current heading on the screen

		motor[fr] = spd*(-_x1+_y1-_x2);
		motor[br] = spd*(_x1+_y1-_x2);
		motor[bl] = spd*(_x1-_y1-_x2);
		motor[fl] = spd*(-_x1-_y1-_x2);

		time1[T1] = 0; //Reset timer
	}
}

void seekIR(){
}

void armup() {
}

void armdown() {
}

void dispose() {
}

void upslope() {
}

task main()
{
	initializeRobot();

	waitForStart(); // Wait for the beginning of autonomous phase.

	for (int curass = 1; curass <= numass; curass++)	{
		slitherto(assignment[curass][0],assignment[curass][1],assignment[curass][2]);

		switch (assignment[curass][3]) {
		case 1:
			seekIR();
			break;
		case 2:
			armup();
			break;
		case 3:
			armdown();
			break;
		case 4:
			dispose();
			break;
		case 5:
			upslope();
			break;
		}
	}
}
