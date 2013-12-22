#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     GYRO,           sensorAnalogInactive)
#pragma config(Sensor, S3,     IRSEEKER,       sensorI2CCustom)
#pragma config(Motor,  motorA,          flag,          tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          jawsr,         tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorC,          jawsl,         tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     frontright,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     backleft,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     backright,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     frontleft,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     motorH,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     motorI,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)

#include "JoystickDriver.c"

//Make sure these drivers and "common.h" are present in *\Robomatter Inc\ROBOTC Development Environment\Includes
#include "hitechnic-gyro.h"
#include "hitechnic-irseeker-v2.h"

void initializeRobot()
{
  HTGYROstartCal(GYRO); //Calibrate gyro sensor, make sure robot is still

  return;
}

task main()
{
  initializeRobot();

  waitForStart(); //Wait for the beginning of autonomous phase


}
