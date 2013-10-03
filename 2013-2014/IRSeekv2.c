#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, MatrxRbtcs, none,     none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     SMUX,            sensorI2CCustom)
#pragma config(Sensor, S4,     gyro,            sensorI2CCustom)
#pragma config(Motor,  motorA,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C1_1,     lift,          tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     driveL,        tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     driveR,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     intake,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_Matrix_S2_1, flagSpinner,   tmotorMatrix, openLoop)
#pragma config(Motor,  mtr_Matrix_S2_2, motorI,        tmotorMatrix, openLoop)
#pragma config(Motor,  mtr_Matrix_S2_3, motorJ,        tmotorMatrix, openLoop)
#pragma config(Motor,  mtr_Matrix_S2_4, motorK,        tmotorMatrix, openLoop)
#pragma config(Servo,  srvo_Matrix_S2_1, servo1,               tServoNone)
#pragma config(Servo,  srvo_Matrix_S2_2, servo2,               tServoNone)
#pragma config(Servo,  srvo_Matrix_S2_3, servo3,               tServoNone)
#pragma config(Servo,  srvo_Matrix_S2_4, servo4,               tServoNone)

#include "JoystickDriver.c"
#include "hitechnic-sensormux.h"
#include "hitechnic-irseeker-v2.h"


int dir_left = 0;
int dir_right = 0;
int S1_left, S2_left, S3_left, S4_left, S5_left = 0;
int S1_right, S2_right, S3_right, S4_right, S5_right = 0;

void startSensors(tHTIRS2DSPMode mode);
void readSensors();
void raiseLift();
void blargLarg();
void driveToBeacon();

const tMUXSensor irL = msensor_S1_2;
const tMUXSensor irR = msensor_S1_1;

task main()
{
	nMotorEncoder[lift] = 0;
	tHTIRS2DSPMode _mode = DSP_1200;
	startSensors(_mode);
	waitForStart();
	raiseLift();
	driveToBeacon();
	blargLarg();
}

void blargLarg() {
	motor[intake] = -100;
	wait1Msec(5000);
	motor[intake] = 0;
}

void raiseLift() {
	while(nMotorEncoder[lift] > -10500) {
		motor[lift] = 100;
	}
	motor[lift] = 0;
}

void driveToBeacon() {
	while(S3_left + S3_right + S4_left + S2_right < 200) {
		readSensors();
		motor[driveL] = 20 + 1.0 / ((S4_left + S5_left) / 50.0);
		motor[driveR] = 20 + 1.0 / ((S1_right + S2_right) / 50.0);
	}
	motor[driveL] = motor[driveR] = 0;
}

void readSensors() {
	dir_left = HTIRS2readACDir(irL);
	dir_right = HTIRS2readACDir(irR);
	HTIRS2readAllDCStrength(irL, S1_left, S2_left, S3_left, S4_left, S5_left);
	HTIRS2readAllDCStrength(irR, S1_right, S2_right, S3_right, S4_right, S5_right);
}

void startSensors(tHTIRS2DSPMode mode) {
	//while(true) {
	//	PlaySound(soundShortBlip);
	//	if(HTIRS2setDSPMode(irL, mode)) {
	//		break;
	//	}
	//}
	//while(true) {
	//	PlaySound(soundShortBlip);
	//	if(HTIRS2setDSPMode(irR, mode)) {
	//		break;
	//	}
	//}
}
