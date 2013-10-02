#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, MatrxRbtcs, none,     none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     irL,            sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S4,     irR,            sensorHiTechnicIRSeeker1200)
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
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.

const bool tankDriveEnabled = true;
float driveDivisor = 1.0;

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
  return;
}

task main()
{
  initializeRobot();

  waitForStart();   // wait for start of tele-op phase

  while (true)
  {
  	//Control power ratios for drive train
  	if(joy1Btn(8)) {
	  		driveDivisor = 4.0;
	  	} else {
	  		driveDivisor = 1.0;
	  	}
  	//Tank Drive
  	if(tankDriveEnabled) {
	  	motor[driveL] = joystick.joy1_y1 / driveDivisor;
	  	motor[driveR] = joystick.joy1_y2 / driveDivisor;
	  } else { //Arcade Drive
	  	motor[driveL] = (joystick.joy1_y1 - joystick.joy1_x2) / driveDivisor;
	  	motor[driveR] = (joystick.joy1_y1 + joystick.joy1_x2) / driveDivisor;
	  }

	  if(joy2Btn(2)) {
	  	moveArmTo(0); //Bottom
	  } else if(joy2Btn(3)) {
	  	moveArmTo(10000); //Top
	  } else {
	  	motor[lift] = joystick.joy2_y1;
	  }

	  if(joy2Btn(8)) {
	  	motor[intake] = 100;
	  } else if(joy2Btn(6)) {
	  	motor[intake] = -100;
	  } else {
	  	motor[intake] = 0;
	  }

	  if(joy2Btn(9) || joy2Btn(10)) {
	  	nMotorEncoder[lift] = 0;
	  }
  }
}
