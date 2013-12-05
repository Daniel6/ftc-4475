#pragma config(Hubs,  S1, HTServo,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Sensor, S2,     gyro,          sensorI2CCustom)
#pragma config(Sensor, S3,     irL,            sensorI2CCustom)
#pragma config(Sensor, S4,     mux,          sensorI2CCustom)
#pragma config(Motor,  motorA,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     driveR,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     driveL,        tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     lift,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     motorG,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     intake,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     spinner,       tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C1_1,    autoArm,              tServoStandard)
#pragma config(Servo,  srvo_S1_C1_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_3,    liftR,                tServoStandard)
#pragma config(Servo,  srvo_S1_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_6,    liftL,                tServoStandard)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
#include "hitechnic-sensormux.h"
#include "hitechnic-irseeker-v2.h"
#include "hitechnic-gyro.h"
#include "hitechnic-colour-v1.h"
#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.

const tMUXSensor lineL = msensor_S4_1;
const tMUXSensor lineR = msensor_S4_2;

//Encoder values
int driveToLineLength = 600;
int driveToLineLengthOffset = 400;
int turnAmount = 5000;
int rampAmount = 1000;

int waitTime = 0;
float driveMult = 1.0;

//Line Sensor variables
int redL = 0;
int greenL = 0;
int blueL = 0;
int redR = 0;
int greenR = 0;
int blueR = 0;

//IRSeeker variables
int dir_left = 0;
int S1_left, S2_left, S3_left, S4_left, S5_left = 0;
tHTIRS2DSPMode _mode = DSP_1200;

//Gyro variables
float heading = 0.0;
float error = 0.0;
float turnPower;

bool leftSide = true;
bool retraceSteps = false;
bool driveFurtherOnRamp = false;

void selectMode();
void driveToLine();
void followLine();
void driveToEnd();
void driveForwards(int t);
void driveBackwards(int t);
void turnOntoRamp();
void turnOntoRampWithGyro();
void score();
void nudge();
bool leftOn();
bool rightOn();
void turnDegrees(int deg);
void startSensors(tHTIRS2DSPMode mode);

void initializeRobot() {
	startSensors(_mode);
	HTGYROstartCal(gyro);
  return;
}

task readSensors() {
	while(true) {
		dir_left = HTIRS2readACDir(irL);
		HTIRS2readAllDCStrength(irL, S1_left, S2_left, S3_left, S4_left, S5_left);
		HTCSreadRGB(lineL, redL, greenL, blueL);
		HTCSreadRGB(lineR, redR, greenR, blueR);
	}
}

task readGyro() {
	time1[T1] = 0;
	while(true) {
		while (time1[T1] < 20) {
  		wait1Msec(1);
  	}
  	time1[T1] = 0;

  	heading += HTGYROreadRot(gyro) * 0.02;
	}
}

task main() {
	disableDiagnosticsDisplay();
	selectMode();
  initializeRobot();
  StartTask(readSensors);
  StartTask(readGyro);
  waitForStart();
  wait1Msec(waitTime);
  driveToLine();
  followLine();
  driveToEnd();
  nudge();
  turnOntoRampWithGyro();
}

void driveToLine() {
	nMotorEncoder[driveL] = 0;
	nMotorEncoder[driveR] = 0;
	if(!leftSide) {
		driveToLineLength += driveToLineLengthOffset;
	}
	while( (abs(nMotorEncoder[driveL]) + abs(nMotorEncoder[driveR])) / 2.0 <= driveToLineLength) {
		if(leftSide) {
			motor[driveL] = motor[driveR] = 100;
		} else {
			motor[driveL] = motor[driveR] = -100;
		}
	}
	motor[driveL] = motor[driveR] = 0;
}

void followLine() {
	if(!leftOn() && !rightOn()) {
		while(!leftOn() && !rightOn()) {
			motor[driveR] = 30;
		}
	}
	motor[driveR] = 0;

	while(S3_left < 70) {
		if(leftOn() && rightOn()) {
			if(leftSide) {
				motor[driveL] = motor[driveR] = 20;
			} else {
				motor[driveL] = motor[driveR] = -20;
			}
		}
		if(leftOn() && !rightOn()) {
			if(leftSide) {
				motor[driveR] = 30;
				motor[driveL] = 0;
			} else {
				motor[driveL] = 30;
				motor[driveR] = 0;
			}
		}
		if(!leftOn() && rightOn()) {
			if(leftSide) {
				motor[driveR] = 0;
				motor[driveL] = 30;
			} else {
				motor[driveL] = 0;
				motor[driveR] = 30;
			}
		}
		if(!leftOn() && !rightOn()) {
			motor[driveL] = motor[driveR] = 0;
			//break;
		}
	}

	motor[driveL] = motor[driveR] = 0;
	wait1Msec(100);
	score();
}

void score() {
	for(int i = 0; i < 630; i++) {
		if(leftOn() && rightOn()) {
			if(leftSide) {
				motor[driveL] = motor[driveR] = 20;
			} else {
				motor[driveL] = motor[driveR] = -20;
			}
		}
		if(leftOn() && !rightOn()) {
			if(leftSide) {
				motor[driveR] = 30;
				motor[driveL] = 0;
			} else {
				motor[driveL] = 30;
				motor[driveR] = 0;
			}
		}
		if(!leftOn() && rightOn()) {
			if(leftSide) {
				motor[driveR] = 0;
				motor[driveL] = 30;
			} else {
				motor[driveL] = 0;
				motor[driveR] = 30;
			}
		}
		if(!leftOn() && !rightOn()) {
			motor[driveL] = motor[driveR] = 0;
			//break;
		}
	}
	motor[driveL] = motor[driveR] = 0;
	wait1Msec(200);
	servo[autoArm] = 10;
	wait10Msec(100);
	servo[autoArm] = 255;
	wait10Msec(100);
}

void driveToEnd() {
	if(retraceSteps) {
		driveMult *= -1.0;
	}

	while(leftOn() || rightOn()) {
		if(leftOn() && rightOn()) {
			if(leftSide) {
				motor[driveL] = motor[driveR] = 20;
			} else {
				motor[driveL] = motor[driveR] = -20;
			}
		}
		if(leftOn() && !rightOn()) {
			if(leftSide) {
				motor[driveR] = 30;
				motor[driveL] = 0;
			} else {
				motor[driveL] = 30;
				motor[driveR] = 0;
			}
		}
		if(!leftOn() && rightOn()) {
			if(leftSide) {
				motor[driveR] = 0;
				motor[driveL] = 30;
			} else {
				motor[driveL] = 0;
				motor[driveR] = 30;
			}
		}
	}
	motor[driveL] = motor[driveR] = 0;
}

void nudge() {
	if(leftSide && !retraceSteps) {
		driveForwards(500);
	} else if (leftSide && retraceSteps) {
		driveBackwards(200);
	} else if (!leftSide && !retraceSteps) {
		driveBackwards(200);
	} else if (!leftSide && retraceSteps) {
		driveForwards(500);
	}
}

void turnOntoRampWithGyro() {
	if(leftSide) { //FORWARDS
		if(retraceSteps) {
			turnDegrees(85);
			driveBackwards(900);
			turnDegrees(90);
			if(driveFurtherOnRamp) {
				driveBackwards(2100);
			} else {
				driveBackwards(1500);
			}
		} else {
			turnDegrees(85);
			driveBackwards(900);
			turnDegrees(-85);
			if(driveFurtherOnRamp) {
				driveBackwards(2100);
			} else {
				driveBackwards(1500);
			}
		}
	} else { //BACKWARDS
		if(retraceSteps) {
			turnDegrees(-85);
			driveForwards(900);
			turnDegrees(85);
			if(driveFurtherOnRamp) {
				driveBackwards(2100);
			} else {
				driveBackwards(1500);
			}
		} else {
			turnDegrees(85);
			driveBackwards(90);
			turnDegrees(90);
			if(driveFurtherOnRamp) {
				driveBackwards(2100);
			} else {
				driveBackwards(1500);
			}
		}
	}
}

void turnOntoRamp() {
	motor[driveL] = motor[driveR] = 0;
	nMotorEncoder[driveL] = 0;
	nMotorEncoder[driveR] = 0;
	wait1Msec(500);
	while( (abs(nMotorEncoder[driveL]) + abs(nMotorEncoder[driveR])) / 2.0 < turnAmount) {
		if(leftSide && !retraceSteps) {
			motor[driveL] = 2;
			motor[driveR] = 100;
		}
		if(leftSide && retraceSteps) {
			motor[driveL] = -2;
			motor[driveR] = -100;
		}
		if(!leftSide && !retraceSteps) {
			motor[driveL] = -2;
			motor[driveR] = -100;
		}
		if(!leftSide && retraceSteps) {
			motor[driveL] = 2;
			motor[driveR] = 100;
		}
	}
	nMotorEncoder[driveL] = 0;
	nMotorEncoder[driveR] = 0;
	while( (abs(nMotorEncoder[driveL]) + abs(nMotorEncoder[driveR])) / 2.0 < rampAmount) {
		if(leftSide && !retraceSteps) {
			motor[driveL] = motor[driveR] = 100;
		}
		if(!leftSide && retraceSteps) {
			motor[driveL] = motor[driveR] = 100;
		}
		if(leftSide && retraceSteps) {
			motor[driveL] = motor[driveR] = -100;
		}
		if(!leftSide && !retraceSteps) {
			motor[driveL] = motor[driveR] = -100;
		}
	}
	motor[driveL] = motor[driveR] = 0;
}

void startSensors(tHTIRS2DSPMode mode) {
	while(true) {
		PlaySound(soundShortBlip);
		if(HTIRS2setDSPMode(irL, mode)) {
			break;
		}
	}
}

bool leftOn() {
	if(redL > 90 || blueL > 90) {
		return true;
	} else {
		return false;
	}
}

bool rightOn() {
	if(redR > 90 || blueR > 90) {
		return true;
	} else {
		return false;
	}
}

void selectMode() {
	//eraseDisplay();
	//nxtDisplayCenteredTextLine(0, "Choose which way");
	//nxtDisplayCenteredTextLine(1, "to drive.");
	//while(nNxtButtonPressed != 3) {
	//	if(nNxtButtonPressed == 1) { //Right arrow
	//		leftSide = true;
	//		nxtDisplayCenteredBigTextLine(3, "Forwards");
	//	}
	//	if(nNxtButtonPressed == 2) { //Left arrow
	//		leftSide = false;
	//		nxtDisplayCenteredBigTextLine(3, "Backwards");
	//	}
	//}
	//eraseDisplay();
	//nxtDisplayCenteredTextLine(0, "Do you want to");
	//nxtDisplayCenteredTextLine(1, "retrace your steps?");
	//wait1Msec(500);
	//while(nNxtButtonPressed != 3) {
	//	if(nNxtButtonPressed == 1) { //Right arrow
	//		retraceSteps = true;
	//		nxtDisplayCenteredBigTextLine(3, "YES");
	//	}
	//	if(nNxtButtonPressed == 2) { //Left arrow
	//		retraceSteps = false;
	//		nxtDisplayCenteredBigTextLine(3, "NO");
	//	}
	//}
	eraseDisplay();
	nxtDisplayCenteredTextLine(0, "How long do you");
	nxtDisplayCenteredTextLine(1, "want to wait?");
	nxtDisplayCenteredBigTextLine(2, "%d", waitTime);
	wait1Msec(500);
	while(nNxtButtonPressed != 3) {
		nxtDisplayCenteredBigTextLine(2, "%dms", waitTime);
		if(nNxtButtonPressed == 1) { //Right arrow
			waitTime += 100;
		}
		if(nNxtButtonPressed == 2) { //Left arrow
			waitTime -= 100;
		}
		if(waitTime < 0) {
			waitTime = 0;
		}
		wait1Msec(100);
	}
	eraseDisplay();
	nxtDisplayCenteredTextLine(0, "How Far do you want");
	nxtDisplayCenteredTextLine(1, "to go on the ramp?");
	while(nNxtButtonPressed != 3) {
		if(nNxtButtonPressed == 1) { //Right arrow
			nxtDisplayCenteredBigTextLine(2, "Further");
			driveFurtherOnRamp = true;
		}
		if(nNxtButtonPressed == 2) { //Left arrow
			nxtDisplayCenteredBigTextLine(2, "Normal");
			driveFurtherOnRamp = false;
		}
		wait1Msec(100);
	}
	eraseDisplay();
	nxtDisplayCenteredTextLine(1, "DONE");
	nxtDisplayCenteredTextLine(2, "dont forget to");
	nxtDisplayCenteredTextLine(3, "turn on the robot");
	nxtDisplayCenteredTextLine(4, ";)");
}

void turnDegrees(int deg) {
	heading = 0;
	error = heading + deg;
	while(error > 5 || error < -5) {
		error = heading + deg; //(-) means you are too far left
		//turnPower = BOUND((int)(error), -100, 100);
		turnPower = 60 * (error/abs(error));
		motor[driveL] = turnPower;
		motor[driveR] = -turnPower;
	}
	motor[driveL] = motor[driveR] = 0;
}

void driveBackwards(int t) {
	motor[driveL] = motor[driveR] = -100;
	wait1Msec(t);
	motor[driveL] = motor[driveR] = 0;
}

void driveForwards(int t) {
	motor[driveL] = motor[driveR] = 100;
	wait1Msec(t);
	motor[driveL] = motor[driveR] = 0;
}
