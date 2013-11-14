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
#pragma config(Motor,  mtr_S1_C4_2,     spinner,       tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C1_1,    autoArm,              tServoStandard)
#pragma config(Servo,  srvo_S1_C1_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_3,    liftR,                tServoStandard)
#pragma config(Servo,  srvo_S1_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_6,    liftL,                tServoStandard)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.

const int kUpperLiftLimit = -9000;
bool tankDriveEnabled = true;
float driveDivisor = 1.0;

void manualControlLift();

bool moveArmTo(int pos) {
	if(abs(pos - nMotorEncoder[lift]) < 50) {
		return true;
		} else {
		motor[lift] = (pos - nMotorEncoder[lift]) / 16.0;
	}
	return false;
}

void initializeRobot()
{
	nMotorEncoder[lift] = 0;
	nMotorEncoder[driveL] = 0;
	nMotorEncoder[driveR] = 0;
	servo[liftL] = 255;
	servo[liftR] = 0;
	servo[autoArm] = 20;
	return;
}

task main()
{
	initializeRobot();
	waitForStart();   // wait for start of tele-op phase
	servo[liftL] = 0;
	servo[liftR] = 255;

	while (true)
	{

	eraseDisplay();
	nxtDisplayString(2, "Lift %d", nMotorEncoder[lift]);

		if(joy1Btn(3)) {
			motor[spinner] = 100;
		} else {
			motor[spinner] = 0;
		}

		//Control power ratios for drive train
		if(joy1Btn(8) || joy1Btn(6)) {
			driveDivisor = 4.0;
		} else {
			driveDivisor = 1.0;
		}
		//Tank Drive
		if(tankDriveEnabled) {
			motor[driveL] = joystick.joy1_y1 / driveDivisor;
			motor[driveR] = joystick.joy1_y2 / driveDivisor;
		} else { //Arcade Drive
			motor[driveL] = (joystick.joy1_y1 + joystick.joy1_x2) / driveDivisor;
			motor[driveR] = (joystick.joy1_y1 - joystick.joy1_x2) / driveDivisor;
		}

		if(joy2Btn(1)) {
				//moveArmTo(0); //Bottom
		} else if(joy2Btn(2)) {
				//moveArmTo(-11000); //Top
		} else if(joy2Btn(3)) {
				manualControlLift();
		} else {
			if(nMotorEncoder[lift] >= 0) {
				if(joystick.joy2_y1 > 0) {
					manualControlLift();
				} else {
					motor[lift] = 0;
				}
			} else if(nMotorEncoder[lift] <= kUpperLiftLimit) {
				if(joystick.joy2_y1 < 0) {
					manualControlLift();
				} else {
					motor[lift] = 0;
				}
			} else {
				manualControlLift();
			}
		}

		if(joy1Btn(9) && joy1Btn(10)) {
			tankDriveEnabled = !tankDriveEnabled;
			wait10Msec(30);
		}

		if(joy2Btn(6)) {
			motor[intake] = 100;
		} else if(joy2Btn(8)) {
			motor[intake] = -100;
		} else {
			motor[intake] = 0;
		}

		if(joy2Btn(9) && joy2Btn(10)) {
			nMotorEncoder[lift] = 0;
		}
	}
}

void manualControlLift() {
	if(joy2Btn(5) || joy2Btn(7)) {
		if(joy2Btn(5) && joy2Btn(7)) {
			motor[lift] = -joystick.joy2_y1 / 8.0;
		} else {
			motor[lift] = -joystick.joy2_y1 / 4.0;
		}
	} else {
		motor[lift] = -joystick.joy2_y1;
	}
}
