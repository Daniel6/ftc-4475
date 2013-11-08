#pragma config(Hubs,  S1, HTServo,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     irL,            sensorI2CCustom)
#pragma config(Motor,  motorA,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     driveR,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     driveL,        tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     lift,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     motorG,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     intake,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     motorI,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C1_1,    autoArm,              tServoStandard)
#pragma config(Servo,  srvo_S1_C1_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_3,    liftR,                tServoStandard)
#pragma config(Servo,  srvo_S1_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_6,    liftL,                tServoStandard)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"
#include "hitechnic-sensormux.h"
#include "hitechnic-irseeker-v2.h"

void readSensors();
void startSensors(tHTIRS2DSPMode mode);
void dumpBrick();
void driveToEnd();
void driveBy();
void turnOntoRamp();
void selectMode();

bool leftSide;
int dir_left = 0;
//int dir_right = 0;
int S1_left, S2_left, S3_left, S4_left, S5_left = 0;
//int S1_right, S2_right, S3_right, S4_right, S5_right = 0;

tHTIRS2DSPMode _mode = DSP_1200;

//const tMUXSensor irL = msensor_S1_2;
//const tMUXSensor irR = msensor_S1_1;
//-----------------------------------------Encoder / Other constants
const int sensorTrigger = 70;
const int encoderTurnAmount = 5000;
const int driveLength = 10000;
const int encoderRampDistance = 4000;
//------------------------------------------------------------------

task main()
{
	disableDiagnosticsDisplay();
	nMotorEncoder[driveL] = 0;
	nMotorEncoder[driveR] = 0;
	startSensors(_mode);
	selectMode();
  waitForStart(); // Wait for the beginning of autonomous phase.
  driveBy();
	dumpBrick();
	driveToEnd();
	turnOntoRamp();

}

void selectMode() {
	eraseDisplay();
	nxtDisplayCenteredTextLine(0, "Choose which way");
	nxtDisplayCenteredTextLine(1, "to drive.");
	while(nNxtButtonPressed != 3) {
		if(nNxtButtonPressed == 1) { //Right arrow
			leftSide = true;
			nxtDisplayCenteredBigTextLine(3, "Forwards");
		}
		if(nNxtButtonPressed == 2) { //Left arrow
			leftSide = false;
			nxtDisplayCenteredBigTextLine(3, "Backwards");
		}
	}
	eraseDisplay();
	nxtDisplayCenteredBigTextLine(1, "DONE");
	nxtDisplayCenteredBigTextLine(2, ";)");
}

void turnOntoRamp() {
	nMotorEncoder[driveL] = 0;
	nMotorEncoder[driveR] = 0;
	if(leftSide) {
		while(abs(nMotorEncoder[driveR]) <= encoderTurnAmount) {
			motor[driveL] = 20;
			motor[driveR] = 100;
		}
		motor[driveL] = motor[driveR] = 0;
	} else {
		while(abs(nMotorEncoder[driveL]) <= encoderTurnAmount) {
			motor[driveL] = 100;
			motor[driveR] = 20;
		}
		nMotorEncoder[driveL] = 0;
		nMotorEncoder[driveR] = 0;
		while( (abs(nMotorEncoder[driveL]) + abs(nMotorEncoder[driveR])) / 2 < encoderRampDistance) {
			motor[driveL] = motor[driveR] = 80;
		}
		motor[driveL] = motor[driveR] = 0;
	}
}

void driveBy() {
	while(S3_left < sensorTrigger) { //Drive forwards until detect beacon to left
		if(leftSide) {
			motor[driveL] = motor[driveR] = 40;
		} else {
			motor[driveL] = motor[driveR] = -40;
		}
  	readSensors();
	}
	motor[driveL] = motor[driveR] = 0;
}

void driveToEnd() {
	while( (abs(nMotorEncoder[driveL]) + abs(nMotorEncoder[driveR])) / 2 <= driveLength) { //Drive to end of wall
		if(leftSide) {
			motor[driveL] = motor[driveR] = 100;
		} else {
			motor[driveL] = motor[driveR] = -100;
		}
	}
	motor[driveL] = motor[driveR] = 0;
}

void dumpBrick() {
	servo[autoArm] = 255;
	wait10Msec(100);
	servo[autoArm] = 10;
}

void readSensors() {
	dir_left = HTIRS2readACDir(irL);
	//dir_right = HTIRS2readACDir(irR);
	HTIRS2readAllDCStrength(irL, S1_left, S2_left, S3_left, S4_left, S5_left);
	//HTIRS2readAllDCStrength(irR, S1_right, S2_right, S3_right, S4_right, S5_right);
}

void startSensors(tHTIRS2DSPMode mode) {
	while(true) {
		PlaySound(soundShortBlip);
		if(HTIRS2setDSPMode(irL, mode)) {
			break;
		}
	}
	//while(true) {
	//	PlaySound(soundShortBlip);
	//	if(HTIRS2setDSPMode(irR, mode)) {
	//		break;
	//	}
	//}
}
