#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, MatrxRbtcs, none,     none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     irL,            sensorI2CCustom)
#pragma config(Sensor, S4,     irR,            sensorI2CCustom)
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

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                           Autonomous Mode Code Template
//
// This file contains a template for simplified creation of an autonomous program for an TETRIX robot
// competition.
//
// You need to customize two functions with code unique to your specific robot.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.
#include "hitechnic-irseeker-v2.h"

string sTextLines[8];

void displayText(int nLineNumber, const string cChar, int nValueDC, int nValueAC);

void initializeRobot()
{
	return;
}

task main()
{
	initializeRobot();

	int _dirDC_left = 0;
	int _dirAC_left = 0;

	int _dirDC_right = 0;
	int _dirAC_right = 0;

	int dcS1_left, dcS2_left, dcS3_left, dcS4_left, dcS5_left = 0;
	int acS1_left, acS2_left, acS3_left, acS4_left, acS5_left = 0;

	int dcS1_right, dcS2_right, dcS3_right, dcS4_right, dcS5_right = 0;
	int acS1_right, acS2_right, acS3_right, acS4_right, acS5_right = 0;

	eraseDisplay();
	for (int i = 0; i < 8; ++i)
		sTextLines[i] = "";

	waitForStart();

	tHTIRS2DSPMode _mode = DSP_1200;

	while(true) {
		PlaySound(soundShortBlip);
		if(HTIRS2setDSPMode(irL, _mode)) {
			break;
		}
	}
	while(true) {
		PlaySound(soundShortBlip);
		if(HTIRS2setDSPMode(irR, _mode)) {
			break;
		}
	}

	while (true) {
		nxtDisplayTextLine(1, "  L       R");

		_dirDC_left = HTIRS2readDCDir(irL);
		if (_dirDC_left < 0)
			break; // I2C read error occurred
		// read the current irL);
		_dirAC_left = HTIRS2readACDir(irR);
		if (_dirAC_left < 0)
			break; // I2C read error occurred

		_dirDC_right = HTIRS2readDCDir(irR);
		if (_dirDC_right < 0)
			break; // I2C read error occurred
		// read the current modulated signal direction
		_dirAC_right = HTIRS2readACDir(irR);
		if (_dirAC_right < 0)
			break; // I2C read error occurred

		if (!HTIRS2readAllDCStrength(irL, dcS1_left, dcS2_left, dcS3_left, dcS4_left, dcS5_left))
			break; // I2C read error occurred
		if (!HTIRS2readAllACStrength(irL, acS1_left, acS2_left, acS3_left, acS4_left, acS5_left))
			break; // I2C read error occurred
		if (!HTIRS2readAllDCStrength(irR, dcS1_right, dcS2_right, dcS3_right, dcS4_right, dcS5_right))
			break; // I2C read error occurred
		if (!HTIRS2readAllACStrength(irR, acS1_right, acS2_right, acS3_right, acS4_right, acS5_right))
			break; // I2C read error occurred

		displayText(1, "D", _dirAC_left, _dirAC_right);
		displayText(2, "0", acS1_left, acS1_right);
		displayText(3, "1", acS2_left, acS2_right);
		displayText(4, "2", acS3_left, acS3_right);
		displayText(5, "3", acS4_left, acS4_right);
		displayText(6, "4", acS5_left, acS5_right);
	}
}

// Minimize LCD screen flicker by only updating LCD when data has changed
void displayText(int nLineNumber, const string cChar, int nValueDC, int nValueAC)
{
	string sTemp;

	StringFormat(sTemp, "%4d %4d", nValueDC, nValueAC);

	// Check if the new line is the same as the previous one
	// Only update screen if it's different.
	if (sTemp != sTextLines[nLineNumber])
	{
		string sTemp2;

		sTextLines[nLineNumber] = sTemp;
		StringFormat(sTemp2, "%s:%s", cChar, sTemp);
		nxtDisplayTextLine(nLineNumber, sTemp2);
	}
}
