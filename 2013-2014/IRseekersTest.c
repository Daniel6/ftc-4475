#pragma config(Sensor, S3,     irL,              sensorI2CCustom)
#pragma config(Sensor, S4,     irR,              sensorI2CCustom)

#include "JoystickDriver.c"
#include "hitechnic-irseeker-v2.h"

string sTextLines[8];
int dir_left = 0;
int dir_right = 0;
int S1_left, S2_left, S3_left, S4_left, S5_left = 0;
int S1_right, S2_right, S3_right, S4_right, S5_right = 0;

void displayText(int nLineNumber, const string cChar, int nValueDC, int nValueAC);

void initializeRobot()
{
	return;
}

void startSensors(tHTIRS2DSPMode mode) {
	while(true) {
		PlaySound(soundShortBlip);
		if(HTIRS2setDSPMode(irL, mode)) {
			break;
		}
	}
	while(true) {
		PlaySound(soundShortBlip);
		if(HTIRS2setDSPMode(irR, mode)) {
			break;
		}
	}
}

task main()
{


	tHTIRS2DSPMode _mode = DSP_1200;

	startSensors(_mode);

	for (int i = 0; i < 8; ++i) {
		sTextLines[i] = "";
	}

	waitForStart();

	while (true) {

		dir_left = HTIRS2readACDir(irL);
		if (dir_left < 0) {
			break;
		}
		dir_right = HTIRS2readACDir(irR);
		if (dir_right < 0) {
			break;
		}

		if (!HTIRS2readAllDCStrength(irL, S1_left, S2_left, S3_left, S4_left, S5_left)) {
			break;
		}
		if (!HTIRS2readAllDCStrength(irR, S1_right, S2_right, S3_right, S4_right, S5_right)) {
			break;
		}

		displayText(1, "D", dir_left, dir_right);
		displayText(2, "1", S1_left, S1_right);
		displayText(3, "2", S2_left, S2_right);
		displayText(3, "3", S3_left, S3_right);
		displayText(3, "4", S4_left, S4_right);
		displayText(3, "5", S5_left, S5_right);
	}
}

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
