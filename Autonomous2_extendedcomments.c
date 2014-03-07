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
//^This code configures the sensors, motors and servos^//

//Make sure these files and common.h are present in ...\Robomatter Inc\ROBOTC Development Environment\Includes
#include "JoystickDriver.c" //Include file to "handle" the Bluetooth messages.
#include "hitechnic-gyro.h" //Gyroscope sensor driver
#include "hitechnic-irseeker-v2.h" //IR-seeker V2 driver
#include "hitechnic-accelerometer-float.h" //Accelerometer driver, modified so HTACreadallaxes(x,y,z) outputs floats instead of integers

//*Variables and constants declaration*//

/*Edit these*/
//Assignments: change numass and assignment[...][] to number of assignments
int numass = 2;
int assignment[2][4]; //Assignments array: [n assignments][n parameters(don't touch)]

//Run Calibrate.c and enter distance travelled here in cm
float d0_1s_1 = 35; //Straight, one second
float d0_1s_2 = 34;
float d0_2s_1 = 79; //Straight, two seconds
float d0_2s_2 = 79;
float d45_1s_1 = 29; //At 45 degrees, one second
float d45_1s_2 = 29;

/*Constants*/
//Constants for arm movement (don't touch)
int ARMUPL = 190; //Up position for left servo
int ARMUPR = 50; //Up position for right servo
int ARMDOWNL = 10; //Down position for left servo
int ARMDOWNR = 230; //Down position for right servo

//Correction constants (don't touch)
float C_D = 0.81; //Initial correction value for distance travelled in a certain direction
float C_T = 1.09; //Initial correction value for distance travelled during a certain time

/*Variables*/
//Movement
float xcur = 0; //Current x-position in coordinate system
float ycur = 0; //Current y-position in coordinate system
float rcur = 0; //Current heading relative to starting position (clockwise) in degrees

//Accelerometer
float xcal, ycal, zcal; //Calibration values
float xacc, yacc, zacc; //Measured acceleration


/*Functions*/
//Filling assignmentarray: don't touch
void assignmentSet(int n, float x, float y, float r, int action) {
	assignment[n][0] = x; //Setting xgoal for slitherto
	assignment[n][1] = y; //Setting ygoal for slitherto
	assignment[n][2] = r; //Setting rgoal for sliterto
	assignment[n][3] = action; //Setting action number to be performed
}

//Assignments for the robot: change this
void initializeAssignments()
{
	//assignmentset(n,x,y,r,action), nothing = 0, seekIR = 1, armup = 2, armdown = 3, dispose = 4, upslope = 5, calibrate = 6
	assignmentSet(0,0,40,0,0);
	assignmentSet(1,60,40,0,0);
}

//Calculating correction constants: don't touch
void distanceCalibration() {
	C_D = ((d45_1s_1+d45_1s_2)/2)/((d0_1s_1+d0_1s_2)/2);
	C_T = 0.5*(((d0_2s_1+d0_2s_2)/2)/((d0_1s_1+d0_1s_2)/2));
}

//Initialization: don't touch
void initializeRobot() {
	HTGYROstartCal(gyro); //Gyro calibration
	distanceCalibration(); //Calculating correction constants
	initializeAssignments(); //Filling assignment array
	HTACreadAllAxes(HTAC, xcal, ycal, zcal); //Calibrating accelerometer

	return;
}

//Moving function: enter coordinates and the robot does the rest. Keeps track of position by time-based dead reckoning.
//Calibration while running a longer program is possible by assigning negative x- and y-coordinates (the robot will push itself into a corner of the field),
//and then running calibrate (6)
void slitherto(float xgoal, float ygoal, float rgoal) {
	float alfa;
	float beta;
	float _x1 = 0;
	float _y1 = 0;
	float _x1s = 0;
	float _y1s = 0;
	float dirx;
	float diry;

	bool atgoal = false;
	float distancecm;

	float betacor;
	float spd = 30;
	float _x2 = 0;
	float rotspeed;

	atgoal = !((dirx == 1 && xgoal > xcur) || (dirx == -1 && xgoal < xcur) || (diry == 1 && ygoal > ycur) || (diry == -1 && ygoal < ycur) || round(rcur) != rgoal);

	beta = atan2(ygoal-ycur,xgoal-xcur);
	betacor = beta + degreesToRadians(rcur);

	_x1 = cos(betacor);
	_y1 = sin(betacor);

	dirx = sgn(xgoal-floor(xcur));
	diry = sgn(ygoal-floor(ycur));

	time1[T1] = 0;

	while(atgoal == false) {
		{
			if ((dirx == 1 && xgoal > xcur) || (dirx == -1 && xgoal < xcur)) {
				_x1s = 1;
			}
			else {
				_x1s = 0;
			}
			if ((diry == 1 && ygoal > ycur) || (diry == -1 && ygoal < ycur)) {
				_y1s = 1;
			}
			else {
				_y1s = 0;
			}

			if (round(rcur) < rgoal - 5 && (_x1s || _y1s))
				_x2 = 1;
			else if (round(rcur) > rgoal + 5 && (_x1s || _y1s))
				_x2 = -1;
			else if (round(rcur) < rgoal && (_x1s || _y1s))
				_x2 = 0.3;
			else if (round(rcur) > rgoal && (_x1s || _y1s))
				_x2 = -0.3;
			else if (round(rcur) < rgoal && !(_x1s || _y1s))
				_x2 = 1;
			else if (round(rcur) > rgoal && !(_x1s || _y1s))
				_x2 = -1;

			if (beta/1.571 - floor(beta/1.571) > 0.785) //Oh radians....
				alfa = 1.571 - (beta/1.571 - floor(beta/1.571));
			else
				alfa = beta/1.571 - floor(beta/1.571);

			wait1Msec(10);

			distancecm = abs(((1-C_D)/(-0.785*alfa) + 1)*((d0_1s_1+d0_1s_2)/2)*C_T*time1[T1]*0.001);

			xcur += distancecm*cos(beta);
			ycur += distancecm*sin(beta);

			HTACreadAllAxes(HTAC, xacc, yacc, zacc);
			xacc -= xcal;
			yacc -= ycal;
			zacc -= zcal;
			writeDebugStreamLine("%d,%d",xacc,yacc);

			rotspeed = HTGYROreadRot(gyro); //Read the current rotation speed
			rcur += rotspeed * time1[T1]*0.001; //Magic
			nxtDisplayCenteredBigTextLine(3, "%2.0f", rcur); //Display our current heading on the screen

			if (abs(xacc) > 120 || abs(yacc) > 120) {
				PlaySound(soundBeepBeep);

				//Move in opposite direction
				motor[fr] = spd*(_x1*_x1s-_y1*_y1s);
				motor[br] = spd*(-_x1*_x1s-_y1*_y1s);
				motor[bl] = spd*(-_x1*_x1s+_y1*_y1s);
				motor[fl] = spd*(_x1*_x1s+_y1*_y1s);

				time1[T1] = 0;

				while (time1[T1] < 1000) {
					time1[T2] = 0;

					wait1Msec(10);

					rotspeed = HTGYROreadRot(gyro); //Read the current rotation speed
					rcur += rotspeed * time1[T2]*0.001; //Magic
					nxtDisplayCenteredBigTextLine(3, "%2.0f", rcur);
				}

				motor[fr] = 0;
				motor[br] = 0;
				motor[bl] = 0;
				motor[fl] = 0;

				distancecm = abs(((1-C_D)/(-0.785*alfa) + 1)*((d0_1s_1+d0_1s_2)/2)*C_T*time1[T1]*0.001);

				xcur += distancecm*cos(-beta);
				ycur += distancecm*sin(-beta);

				wait1Msec(1000);
			}

			motor[fr] = spd*(-_x1*_x1s+_y1*_y1s-_x2);
			motor[br] = spd*(_x1*_x1s+_y1*_y1s-_x2);
			motor[bl] = spd*(_x1*_x1s-_y1*_y1s-_x2);
			motor[fl] = spd*(-_x1*_x1s-_y1*_y1s-_x2);

			atgoal = !((dirx == 1 && xgoal > xcur) || (dirx == -1 && xgoal < xcur) || (diry == 1 && ygoal > ycur) || (diry == -1 && ygoal < ycur) || round(rcur) != rgoal);

			time1[T1] = 0; //Reset timer
		}

		motor[fr] = 0;
		motor[br] = 0;
		motor[bl] = 0;
		motor[fl] = 0;
	}
}

//This function assumes that the robot only needs to move left and right to find the beacon:
//Make sure x,y,r is a correct starting position
void seekIR(){
	int _dirEnh;
	int _strEnh;
	float dir;
	float time = 0;

	HTIRS2readEnhanced(IR, _dirEnh, _strEnh);

	time1[T1] = 0;

	while (_dirEnh != 6) {
		time1[T1] = 0;

		HTIRS2readEnhanced(IR, _dirEnh, _strEnh);

		if (_dirEnh == 0) {
			motor[fr] = 0;
			motor[br] = 0;
			motor[bl] = 0;
			motor[fl] = 0;
			PlaySound(soundShortBlip);
		}
		else if (_dirEnh < 6){
			motor[fr] = 30;
			motor[br] = -30;
			motor[bl] = -30;
			motor[fl] = 30;
			dir = -1;
		}
		else if (_dirEnh > 6) {
			motor[fr] = -30;
			motor[br] = 30;
			motor[bl] = 30;
			motor[fl] = -30;
			dir = 1;
		}

		time += dir*time1[T1];
	}

	//Edit this according to basket position
	motor[fr] = -30; //Sideways
	motor[br] = 30;
	motor[bl] = 30;
	motor[fl] = -30;
	wait1Msec(500);

	motor[fr] = -30; //Backwards
	motor[br] = -30;
	motor[bl] = 30;
	motor[fl] = 30;
	wait1Msec(1200);

	motor[fr] = 0; //Arm up
	motor[br] = 0;
	motor[bl] = 0;
	motor[fl] = 0;
	PlaySound(soundBeepBeep);
	servo[arml] = ARMUPL;
	servo[armr] = ARMUPR;
	wait1Msec(1000);	motor[fr] = 0; //Arm down
	motor[br] = 0;
	motor[bl] = 0;
	motor[fl] = 0;

	motor[fr] = 30; //Forward
	motor[br] = 30;
	motor[bl] = -30;
	motor[fl] = -30;
	wait1Msec(1200);

	motor[fr] = 0; //Dispose
	motor[br] = 0;
	motor[bl] = 0;
	motor[fl] = 0;
	motor[jaw] = 40;
	wait1Msec(1500);
	motor[jaw] = 0;

	motor[fr] = -30; //Backward
	motor[br] = -30;
	motor[bl] = 30;
	motor[fl] = 30;
	wait1Msec(1200);

	motor[fr] = 0; //Arm down
	motor[br] = 0;
	motor[bl] = 0;
	motor[fl] = 0;
	PlaySound(soundBeepBeep);
	servo[arml] = ARMDOWNL;
	servo[armr] = ARMDOWNR;
	wait1Msec(1000);

	motor[fr] = 30; //Forward
	motor[br] = 30;
	motor[bl] = -30;
	motor[fl] = -30;
	wait1Msec(1200);

	motor[fr] = 30; //Sideways
	motor[br] = -30;
	motor[bl] = -30;
	motor[fl] = 30;
	wait1Msec(500);

	dir = sgn(time);
	motor[fr] = dir*30; //Back to coordinates
	motor[br] = dir*-30;
	motor[bl] = dir*-30;
	motor[fl] = dir*30;
	wait1Msec(time);
	motor[fr] = 0;
	motor[br] = 0;
	motor[bl] = 0;
	motor[fl] = 0;

}

void armup() {
}

void armdown() {
}

void dispose() {
}

//This function assumes a correct starting position (right in front of slope)
//It also assumes it is the last action of the robot in autonomous (as it should be), so x,y,r are incorrect after this
void upslope() {
	motor[fr] = 50; //Forward (fast)
	motor[br] = 50;
	motor[bl] = -50;
	motor[fl] = -50;
	wait1Msec(1500);
	motor[fr] = 0;
	motor[br] = 0;
	motor[bl] = 0;
	motor[fl] = 0;
}

//To be used when calibrating (=jamming into a corner) to reset coordinates (make sure the coordinates match the desired calibration position)
void calibrate() {
	xcur = 0;
	ycur = 0;
	rcur = 0;
}

task main()
{
	initializeRobot();

	//waitForStart(); // Wait for the beginning of autonomous phase.

	for (int curass = 0; curass < numass; curass++)	{
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
		case 6:
			calibrate();
			break;
		}
	}
}
