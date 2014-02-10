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

#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.
#include "hitechnic-gyro.h"
#include "hitechnic-sensormux.h"
#include "hitechnic-irseeker-v2.h"
#include "hitechnic-colour-v1.h"

const tMUXSensor lineL = msensor_S4_1;
const tMUXSensor lineR = msensor_S4_2;
const tMUXSensor rangefinder = msensor_S4_3;

//IRSeeker variables
int dir_left = 0;
int S1_left, S2_left, S3_left, S4_left, S5_left;

//Line Sensor variables
int redL = 0;
int greenL = 0;
int blueL = 0;
int redR = 0;
int greenR = 0;
int blueR = 0;

int waitTime = 0;
const float kDriveDistance1 = 100; //The distance that we drive to when initially approaching the line/board.
const float kDriveDistance2 = 100; //The distance that we drive to when going to get blocks.
const float kDriveDistance3 = 1000; //The distance that we drive backwards to get on the ramp after getting blocks.
const int kTurnAmount1 = 45; //The amount that we turn left when we go to get blocks after getting to the end of the line.
const int kTurnAmount2 = -45; //The amount that we turn left after getting blocks to make our butt face the ramp.
float heading = 0;
bool scored = false;
float gyroOffset = 0;

task getHeading();
task readSensors();
void startup();
void calibrateGyroOffset();
void getSettings();
void driveToDistance(float dist);
void encoderDriveToDistance(float dist, bool forwards);
void driveOnHeading(float dir);
void turnDegrees(int deg);
void turnOntoLine();
void driveBy();
void pickupBlocks();
void getOnRamp();
void block();
void score();
void followLineBackwards();
void followLineForwards();
bool onTarget();
bool frontOnLine();
bool backOnLine();
bool frontLeftOnLine();
bool frontRightOnLine();
bool backLeftOnLine();
bool backRightOnLine();

task main() {
	StartTask(getHeading);
	StartTask(readSensors);
	startup(); //Initialize and calibrate
	getSettings(); //Set Wait timer.
  waitForStart(); //Wait for the beginning of autonomous phase.
  wait10Msec(waitTime); //Do the wait that you set in getSettings();
	driveToDistance(kDriveDistance1); //Drive until front sensor is on line
	turnOntoLine(); //Turn on center until both sensors on line
	driveBy(); //Drive backwards, then drive along whole line, scoring, then move to the end of the line
	pickupBlocks(); //Drive to the corner and pick up blocks?
	getOnRamp(); //Drive backwards up onto the ramp
	block(); //Try to block other team from getting on the ramp too.
}

task readSensors() {
	while(true) {
		dir_left = HTIRS2readACDir(irL);
		HTIRS2readAllDCStrength(irL, S1_left, S2_left, S3_left, S4_left, S5_left);
		HTCSreadRGB(lineL, redL, greenL, blueL);
		HTCSreadRGB(lineR, redR, greenR, blueR);
	}
}

task getHeading() {
	time1[T1] = 0;
	while(true) { //Recalculate heading every 20ms
		while (time1[T1] < 20) {
  		wait1Msec(1);
  	}
  	time1[T1] = 0;

  	heading += (HTGYROreadRot(gyro) - gyroOffset) * 0.02; //Subtract the offset error caused by skewed sensor
	}
}

void startup() {
	while(true) {
		if(HTIRS2setDSPMode(irL, DSP_1200)) {
			break;
		}
	}
	HTGYROstartCal(gyro);
	wait1Msec(1000);
	calibrateGyroOffset();
}

void calibrateGyroOffset() {
	float accGyroVal = 0;
	for(int i = 0; i < 10; i++) {
		accGyroVal += HTGYROreadRot(gyro);
		wait1Msec(100);
	}
	gyroOffset = accGyroVal / 10.0; //Take average gyro reading over 1 seconds w/ 10 samples
}

void turnOntoLine() { //Turn on center until both front sensors and back sensors are on the line
	while(!frontOnLine() && !backOnLine()) {
		motor[driveL] = 100;
		motor[driveR] = -100;
	}
	motor[driveL] = motor[driveR] = 0;
}

void driveBy() {
	followLineBackwards();
	followLineForwards();
}

void pickupBlocks() {
	motor[intake] = 100;
	turnDegrees(heading - kTurnAmount1); //Turn to face the blocks
	driveToDistance(kDriveDistance2); //Drive to the blocks
}

void getOnRamp() {
	turnDegrees(heading - kTurnAmount2); //Turn so butt is facing the ramp
	encoderDriveToDistance(kDriveDistance3, false);
}

void block() {

}

void getSettings() {
	disableDiagnosticsDisplay();
	eraseDisplay();
	nxtDisplayCenteredTextLine(0, "Delay Time:");
	while (nNxtButtonPressed != 3) {
		if(nNxtButtonPressed == 1) { //Right arrow
			waitTime += 1000;
		}
		if(nNxtButtonPressed == 2) { //Left arrow
			waitTime -= 1000;
		}
		nxtDisplayCenteredTextLine(1, "%i ms", waitTime);
		wait10Msec(200);
	}
}

void turnDegrees(int deg) {
	while (abs(heading - deg) > 5) {
		float err = heading - deg; //Neg err means we need to turn right
		motor[driveL] = err * -2.0;
		motor[driveR] = err * 2.0;
	}
	motor[driveL] = motor[driveR] = 0;
}

void driveToDistance(float dist) {
	float straight = heading;
	while(SensorValue[rangefinder] > dist) {
		driveOnHeading(straight);
	}
	motor[driveL] = motor[driveR] = 0;
}

void encoderDriveToDistance(float dist, bool forwards) { //Pass distance and direction. True means drive forwards, false means drive backwards.
	while (abs(nMotorEncoder[driveL]) + abs(nMotorEncoder[driveR]) < dist*2) {
		if (forwards) {
			motor[driveL] = motor[driveR] = 100;
		} else {
			motor[driveL] = motor[driveR] = -100;
		}
	}
	motor[driveL] = motor[driveR] = 0;
}

void driveOnHeading(float dir) {
	float err = heading - dir; //Neg err means we need to turn right
	motor[driveL] = 90 - err;  //Pos err means we need to turn left
	motor[driveR] = 90 + err;
}

bool frontOnLine() {
	return frontLeftOnLine() && frontRightOnLine();
}

bool frontLeftOnLine() {

}

bool frontRightOnLine() {

}

bool backOnLine() {
	return backLeftOnLine() && backRightOnLine();
}

bool backLeftOnLine() {

}

bool backRightOnLine() {

}

void followLineBackwards() {
	while (backLeftOnLine() || backRightOnLine()) {
		if (backLeftOnLine() && backRightOnLine()) {
			motor[driveL] = motor[driveR] = -100;
		} else if (backLeftOnLine()) {
			motor[driveL] = 0;
			motor[driveR] = -100;
		} else if (backRightOnLine()) {
			motor[driveL] = -100;
			motor[driveR] = 0;
		}	else {
			motor[driveL] = motor[driveR] = 0;
		}
	}
	motor[driveL] = motor[driveR] = 0;
}

void followLineForwards() {
	while (frontLeftOnLine() || frontRightOnLine()) {
		if (onTarget() && !scored) {
			motor[driveL] = motor[driveR] = 0;
			score();
		}
		if (frontLeftOnLine() && frontRightOnLine()) {
			motor[driveL] = motor[driveR] = 100;
		} else if (frontLeftOnLine()) {
			motor[driveL] = 100;
			motor[driveR] = 0;
		} else if (frontRightOnLine()) {
			motor[driveL] = 0;
			motor[driveR] = 100;
		}	else {
			motor[driveL] = motor[driveR] = 0;
		}
	}
	motor[driveL] = motor[driveR] = 0;
}

bool onTarget() {
	if (S3_left > 70) {
		return true;
	} else {
		return false;
	}
}

void score() {
	servo[autoArm] = 255;
	scored = true;
	wait1Msec(500);
	servo[autoArm] = 10;
	wait1Msec(100);
}
