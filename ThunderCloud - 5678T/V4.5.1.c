#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, dgtl1,  PELevel,        sensorDigitalIn)
#pragma config(Sensor, I2C_1,  leftE,          sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  lArmE,          sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  rArmE,          sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_4,  rightE,         sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           grabR,         tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           armR3,         tmotorVex393_MC29, openLoop, encoderPort, I2C_3)
#pragma config(Motor,  port3,           right,         tmotorVex393_MC29, openLoop, encoderPort, I2C_4)
#pragma config(Motor,  port4,           armR1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           armL2,         tmotorVex393_MC29, openLoop, encoderPort, None)
#pragma config(Motor,  port6,           armL3,         tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_2)
#pragma config(Motor,  port7,           left,          tmotorVex393_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port8,           armL1,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           armR2,         tmotorVex393_MC29, openLoop, reversed, encoderPort, None)
#pragma config(Motor,  port10,          grabL,         tmotorVex393_HBridge, openLoop, encoderPort, None)
#pragma config(DatalogSeries, 0, "Battery Status", Sensors, Sensor, in8, 500)
#pragma config(DatalogSeries, 1, "Battery Level", Properties, immediateBatteryLevel, , 500)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//Important Stuff, Don't Mess with!//////
#pragma platform(VEX2)///////////////////
#pragma competitionControl(Competition)//
#include "Vex_Competition_Includes.c"////
/////////////////////////////////////////
bool testI2C()
{
	if(nI2CStatus == 0 || nI2CStatus == 6)
	{
		return false;
	}
	else
	{
		return true;
	}
}
bool getI2C = testI2C();
void pre_auton()
{
	bLCDBacklight = true;                                    // Turn on LCD Backlight
	bStopTasksBetweenModes = true;
	bDisplayCompetitionStatusOnLcd = false;
	clearLCDLine(0);                                            // Clear line 1 (0) of the LCD
	clearLCDLine(1);                                            // Clear line 2 (1) of the LCD
	string mainBattery, I2C_ErrorCode;
	if(nImmediateBatteryLevel/1000.0>7 && getI2C == true)
	{
		displayLCDString(0, 0, "Systems: GREEN");
		displayLCDString(1, 0, "Ready to Begin!");
	}
	else if(nImmediateBatteryLevel/1000.0>7 && getI2C == false)
	{
		displayLCDCenteredString(0, "I2C Fault!!!");
		displayLCDCenteredString(1, "Error #: ");
		sprintf(I2C_ErrorCode, "%1.2f%c", nI2CStatus);
		displayNextLCDString(I2C_ErrorCode);
	}
	else if(getI2C == false && nImmediateBatteryLevel/1000.0<7)
	{
		displayLCDCenteredString(0, "I2C Fault!!!");
		displayLCDCenteredString(1, "Battery Fault!!!");
	}
	else
	{
		displayLCDCenteredString(0, "REPLACE BATT!!!");
		displayLCDString(1, 0, "Main V: ");
		sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V');
		displayNextLCDString(mainBattery);
	}
}
//auton methods//////////////////////////
/////////////////////////////////////////
float mult = 156.8; //changes inches to ticks, motors should be torque, if not this will be off, test this
void drive(int speed, float forHowLong)
{
	float distance = forHowLong*mult;
	do
	{
		motor[left]=speed;
		motor[right]=speed;
	}
	while(getMotorEncoder(left)< distance && getMotorEncoder(right)<distance);
	motor[left] = motor[right] = 0;
}
/////////////////////////////////////////
void rotateR (int speed, int forHowLong)
{
	float distance = forHowLong*mult;
	do
	{
		motor[left]=speed;
		motor[right]=-speed;
	}
	while(getMotorEncoder(left) < distance && getMotorEncoder(right) > -distance);
	motor[left] = motor[right] = 0;
}
/////////////////////////////////////////
void rotateL (int speed, int forHowLong)
{
	float distance = forHowLong*mult;
	do
	{
		motor[left]=-speed;
		motor[right]=speed;
	}
	while(getMotorEncoder(left) > -distance && getMotorEncoder(right) < distance);
	motor[left] = motor[right] = 0;
}
/////////////////////////////////////////
void resetIME ()
{
	resetMotorEncoder(right);
	resetMotorEncoder(left);
	resetMotorEncoder(grabL);
	resetMotorEncoder(armR3);
	resetMotorEncoder(armL3);
	clearTimer(T1);
}
/////////////////////////////////////////
void armAuto (int speed, int forHowLong)
{
	do
	{
		motor [armL1] = speed;
		motor [armL2] = speed;
		motor [armL3] = speed;
		motor [armR1] = speed;
		motor [armR2] = speed;
		motor [armR3] = speed;
	}
	while(getMotorEncoder(armL3)<forHowLong && getMotorEncoder(armR3)<forHowLong);
	motor[left] = motor[right] = 0;
}
/////////////////////////////////////////
void grabAuto (int speed, int forHowLong)
{
	do
	{
		motor [grabL] = motor[grabR] = speed;
	}
	while(time1[T1]<= forHowLong);
	motor [grabL] = motor[grabR] = 0;
}
////////////////////////////////////////
//autonomous
task autonomous()
{
	slaveMotor(armL2, armL3);
	slaveMotor(armL1, armL2);
	slaveMotor(armR1, armR3);
	slaveMotor(armR2, armR1);
	resetIME();
	grabAuto(127, 1250);//clutch preload
	resetIME();
	drive(127, 1000); //drive forward
	resetIME();
	rotateL(127, 1000);//turn 180 degrees
	resetIME();
	drive(-127, -850);//reverse to wall
	resetIME();
	armAuto(127, 250);//lift arm
	resetIME();
	grabAuto(-127, 1500);//release preload
	resetIME();
	armAuto(-127, -800);//arm back down
	resetIME();
	rotateR(127, 400);//rotate towards cube
	resetIME();
	drive(127,500);//drive towards cube
	resetIME();
	grabAuto(127, 1250);//clutch cube
	resetIME();
	rotateL(127, 400);//rotate towards fence
	resetIME();
	drive(-127, -850);//backup
	resetIME();
	armAuto(127,250);//raise arm
	resetIME();
	grabAuto(-127, 1250);//dump cube
	resetIME();
	armAuto(-127, -800);//lower arm
	resetIME();
}
//User Control Methods////////////////////////
void grab(int speed)
{
	motor [grabL] = motor[grabR] = speed;
}
/////////////////////////////////////////
void arm (int speed)//fallback simple method
{
	motor [armL1] = speed;
	motor [armL2] = speed;
	motor [armL3] = speed;
	motor [armR1] = speed;
	motor [armR2] = speed;
	motor [armR3] = speed;
}
/////////////////////////////////////////
task errorReset()//May work, or may cause catastrophic failue
{
	hogCPU();
	wait1Msec(1500);
	releaseCPU();
	startTask(usercontrol);
}
/////////////////////////////////////////
task usercontrol()
{
	while (true)
	{
		do
		{
			float x1 = vexRT[Ch4];
			float y1 = vexRT[Ch3];
			float x2 = vexRT[Ch1];
			float y2 = vexRT[Ch2];
			//Left Joystick Arcade Drive//
			motor [left] = y1 + x1;     //
			motor [right] = y2 - x2;    //
			//////////////////////////////
			//Arm Control/////////////////
			if (vexRT[Btn6U] == 1)			//
			{														//
				arm(127);									//
			}														//
			else if (vexRT[Btn6D] == 1 )//
			{														//
				arm(-127);								//
			}														//
			else												//
			{														//
				arm(0);										//
			}														//
			//////////////////////////////
			//Grabber Control/////////////
			if (vexRT[Btn8U] == 1)			//
			{														//
				grab(127);								//
			}														//
			else if (vexRT[Btn8D] == 1) //
			{														//
				grab(-127);								//
			}														//
			else												//
			{														//
				grab(0);									//
			}														//
			//////////////////////////////
			//LCD Display/////////////////
			string mainBattery, PEBattery;
			clearLCDLine(0);                                            // Clear line 1 (0) of the LCD
			clearLCDLine(1);                                            // Clear line 2 (1) of the LCD

			//Display the Primary Robot battery voltage
			displayLCDString(0, 0, "Primary: ");
			sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V');
			displayNextLCDString(mainBattery);

			//Display the Backup battery voltage
			displayLCDString(1, 0, "Count: ");
			sprintf(PEBattery, "%1.2f%c", BackupBatteryLevel/1000.0, 'V');
			displayNextLCDString(PEBattery);
			//Deadband////////////////////
			int deadBand = 10;
			if(x1 > deadBand || x1 < -deadBand)
			{
				x1 = x1;
			}
			else
			{
				x1 = 0;
			}
			if(y1 > deadBand || y1 < -deadBand)
			{
				y1 = y1;
			}
			else
			{
				y1 = 0;
			}
			if(x2 > deadBand || x2 < -deadBand)
			{
				x2 = x2;
			}
			else
			{
				x2 = 0;
			}
			if(y2 > deadBand || y2 < -deadBand)
			{
				y2 = y2;
			}
			else
			{
				y2 = 0;
			}
		}
		while(getI2C == false);
		startTask(errorReset);
	}
}
