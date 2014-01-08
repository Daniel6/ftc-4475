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

const float kDriveDistance1 = 100;
float heading = 0;
float gyroOffset = 0;

task getHeading();
task readSensors();
void startup();
void calibrateGyroOffset();
void getSettings();
void driveToDistance(float dist);
void driveOnHeading(float dir);
void turnOntoLine();
void driveBy();
void pickupBlocks();
void getOnRamp();
void block();

task main() {
	StartTask(getHeading);
	StartTask(readSensors);
	startup();
	getSettings();
  waitForStart(); // Wait for the beginning of autonomous phase.
	driveToDistance(kDriveDistance1);
	turnOntoLine();
	driveBy();
	pickupBlocks(); //???
	getOnRamp();
	block();
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

void turnOntoLine() {

}

void driveBy() {

}

void pickupBlocks() {

}

void getOnRamp() {

}

void block() {

}

void getSettings() {

}

void driveToDistance(float dist) {
	float straight = heading;
	while(SensorValue[rangefinder] > dist) {
		driveOnHeading(straight);
	}
	motor[driveL] = motor[driveR] = 0;
}

void driveOnHeading(float dir) {
	float err = heading - dir; //Neg err means we need to turn right
	motor[driveL] = 90 - err;  //Pos err means we need to turn left
	motor[driveR] = 90 + err;
}
