#pragma config(Hubs,  S1, HTServo,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Sensor, S2,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Sensor, S3,     irL,            sensorI2CCustom)
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

#include "JoystickDriver.c"
#include "hitechnic-sensormux.h"
#include "hitechnic-irseeker-v2.h"
#include "hitechnic-gyro.h"

#define BOUND(n, l, h) (((n) < (l))? (l): ((n) > (h))? (h): (n))

void turnDegrees(int deg);
void readSensors();
void startSensors(tHTIRS2DSPMode mode);
void dumpBrick();
void driveToEnd();
void driveBy();
void turnOntoRamp();
void selectMode();

bool leftSide;
bool retraceSteps;
int dir_left = 0;
//int dir_right = 0;
int S1_left, S2_left, S3_left, S4_left, S5_left = 0;
//int S1_right, S2_right, S3_right, S4_right, S5_right = 0;

tHTIRS2DSPMode _mode = DSP_1200;

//const tMUXSensor irL = msensor_S1_2;
//const tMUXSensor irR = msensor_S1_1;
float heading = 0.0;
float initialHeading = 0.0;
float error;
float turnPower;

//-----------------------------------------Encoder / Other constants
int reversedDriveLength = 200;
int encoderRampDistance = 1800;
const int sensorTrigger = 70;
const int encoderTurnAmount = 4600;
const int initialTurnAmount = encoderTurnAmount / 4.0; //How long the initial phase of turning is
const int driveLength = 4600;
const int slowDrive = 5; //Value for the inside motor when turning onto ramp during initial phase
const int fastDrive = 100; //Value for the outside motor when turning onto ramp during initial phase
const int reverseDrive = -10;
//------------------------------------------------------------------

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

//--------------------------------MAIN TASK--------------------------------------------------------------
task main()
{
	disableDiagnosticsDisplay();
	heading = 0.0;
	StartTask(readGyro);
	nMotorEncoder[driveL] = 0;
	nMotorEncoder[driveR] = 0;
	startSensors(_mode);

	selectMode();
  waitForStart(); // Wait for the beginning of autonomous phase.
  HTGYROstartCal(gyro);
  initialHeading = heading;
 // motor[lift] = -30;
 // wait1Msec(800);
 // motor[lift] = 0;
  driveBy();
	dumpBrick();
	driveToEnd();
	turnOntoRamp();

}

void turnDegrees(int deg) {
	heading = 0;
	error = heading - deg;
	while(error > 5 || error < -5) {
		error = heading - deg; //(-) means you are too far left
		//turnPower = BOUND((int)(error), -100, 100);
		turnPower = 40 * (error/abs(error));
		if(leftSide && !retraceSteps) {
			turnPower = 60 * (error/abs(error));
		} else if (!leftSide && !retraceSteps) {
			turnPower = 50 * (error/abs(error));
		}
		motor[driveL] = -turnPower;
		motor[driveR] = turnPower;
	}
	motor[driveL] = motor[driveR] = 0;
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
	wait1Msec(500);
	nxtDisplayCenteredTextLine(0, "Do you want to");
	nxtDisplayCenteredTextLine(1, "retrace your steps?");
	while(nNxtButtonPressed != 3) {
		if(nNxtButtonPressed == 1) { //Right arrow
			retraceSteps = true;
			nxtDisplayCenteredBigTextLine(3, "YES");
		}
		if(nNxtButtonPressed == 2) { //Left arrow
			retraceSteps = false;
			nxtDisplayCenteredBigTextLine(3, "NO");
		}
	}
	eraseDisplay();
	nxtDisplayCenteredBigTextLine(1, "DONE");
	nxtDisplayCenteredBigTextLine(2, ";)");
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

void turnOntoRamp() {
	nMotorEncoder[driveL] = 0;
	nMotorEncoder[driveR] = 0;
	wait1Msec(1000);
	if(leftSide) { //FORWARDS
		if(retraceSteps) {
			turnDegrees(-100);
			driveForwards(700);
			turnDegrees(-90);
		} else {
			turnDegrees(-75);
			driveForwards(700);
			turnDegrees(75);
		}
	} else { //BACKWARDS
		if(retraceSteps) {
			turnDegrees(-85);
			driveForwards(700);
			turnDegrees(85);
		} else {
			turnDegrees(-100);
			driveForwards(700);
			turnDegrees(-90);
		}
	}

	//if(leftSide) {
	//	while(abs(nMotorEncoder[driveR]) <= encoderTurnAmount) {
	//		if(abs(nMotorEncoder[driveR]) < initialTurnAmount) {
	//			if(retraceSteps) {
	//				motor[driveL] = -slowDrive;
	//				motor[driveR] = -fastDrive;
	//			} else {
	//				motor[driveL] = slowDrive;
	//				motor[driveR] = fastDrive;
	//			}
	//		} else {
	//			if(retraceSteps) {
	//				motor[driveL] = reverseDrive;
	//				motor[driveR] = -fastDrive;
	//			} else {
	//				motor[driveL] = -reverseDrive;
	//				motor[driveR] = fastDrive;
	//			}
	//		}
	//	}
	//	motor[driveL] = motor[driveR] = 0;
	//} else {
	//	while(abs(nMotorEncoder[driveL]) <= encoderTurnAmount) {
	//		if(abs(nMotorEncoder[driveL]) < initialTurnAmount) {
	//			if(retraceSteps) {
	//				motor[driveR] = -slowDrive;
	//				motor[driveL] = -fastDrive;
	//			} else {
	//				motor[driveR] = slowDrive;
	//				motor[driveL] = fastDrive;
	//			}
	//		} else {
	//			if(retraceSteps) {
	//				motor[driveR] = reverseDrive;
	//				motor[driveL] = -fastDrive;
	//			} else {
	//				motor[driveR] = -reverseDrive;
	//				motor[driveL] = fastDrive;
	//			}
	//		}
	//	}
		//motor[driveL] = motor[driveR] = 0;



	//Drive up onto the ramp!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	nMotorEncoder[driveL] = 0;
	nMotorEncoder[driveR] = 0;
	wait1Msec(1000);
	if(leftSide && !retraceSteps) {
		encoderRampDistance += 100;
	}
	while( (abs(nMotorEncoder[driveL]) + abs(nMotorEncoder[driveR])) / 2 < encoderRampDistance) {
		if(retraceSteps) {
			motor[driveL] = motor[driveR] = -80;
		} else {
			motor[driveL] = motor[driveR] = -80;
		}
	}
	motor[driveL] = motor[driveR] = 0;
}

void driveBy() {
	if(leftSide) {
		while(S3_left < sensorTrigger && (abs(nMotorEncoder[driveL]) + abs(nMotorEncoder[driveR])) / 2 <= driveLength - 500) { //Drive forwards until detect beacon to left
			motor[driveL] = motor[driveR] = 40;
  		readSensors();
		}
	} else {
		while(S3_left < sensorTrigger && (abs(nMotorEncoder[driveL]) + abs(nMotorEncoder[driveR])) / 2 <= driveLength) {
  		motor[driveL] = motor[driveR] = -23;
  		readSensors();
		}

		int setpoint = (abs(nMotorEncoder[driveL]) + abs(nMotorEncoder[driveR])) / 2;
		while((abs(nMotorEncoder[driveL]) + abs(nMotorEncoder[driveR])) / 2 > setpoint - 40) {
			motor[driveL] = motor[driveR] = 18;
		}
	}
	motor[driveL] = motor[driveR] = 0;
}

void driveToEnd() {
	if(!retraceSteps) {
		if(leftSide) {
			while( (abs(nMotorEncoder[driveL]) + abs(nMotorEncoder[driveR])) / 2 <= driveLength - 650) { //Drive to end of wall
				motor[driveL] = motor[driveR] = 25;
			}
		} else {
			while( (abs(nMotorEncoder[driveL]) + abs(nMotorEncoder[driveR])) / 2 <= driveLength - 100) { //Drive to end of wall
				motor[driveL] = motor[driveR] = -25;
			}
		}
	} else {
		reversedDriveLength = BOUND(((abs(nMotorEncoder[driveL]) + abs(nMotorEncoder[driveR])) / 2) - 400, 0, driveLength);
		nMotorEncoder[driveL] = 0;
		nMotorEncoder[driveR] = 0;
		while( (abs(nMotorEncoder[driveL]) + abs(nMotorEncoder[driveR])) / 2 <= reversedDriveLength) { //Drive back to start
			if(leftSide) {
				motor[driveL] = motor[driveR] = -25;
			} else {
				motor[driveL] = motor[driveR] = 25;
			}
		}
	}
	motor[driveL] = motor[driveR] = 0;
}

void dumpBrick() {
	servo[autoArm] = 255;
	wait10Msec(100);
	servo[autoArm] = 10;
	wait10Msec(100);
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
