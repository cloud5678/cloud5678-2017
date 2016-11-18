#pragma config(UART_Usage, UART2, uartNotUsed, baudRate4800, IOPins, None, None)
#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  backRIME,       sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  backLIME,       sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  armLIME,        sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_4,  armRIME,        sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_5,  grabIME,        sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           grabR,         tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           frontRight,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           backRight,     tmotorVex393_MC29, PIDControl, driveRight, encoderPort, I2C_2)
#pragma config(Motor,  port4,           armL1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           armL2,         tmotorVex393_MC29, PIDControl, reversed, encoderPort, I2C_4)
#pragma config(Motor,  port6,           frontLeft,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           backLeft,      tmotorVex393_MC29, PIDControl, reversed, driveLeft, encoderPort, I2C_1)
#pragma config(Motor,  port8,           armR1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           armR2,         tmotorVex393_MC29, PIDControl, reversed, encoderPort, I2C_3)
#pragma config(Motor,  port10,          grabL,         tmotorVex393_HBridge, PIDControl, encoderPort, I2C_5)
#pragma config(DatalogSeries, 0, "", Motors, MotorPWM, port5, 50)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
//Important Stuff, Don't Mess with!//////
#pragma platform(VEX2)///////////////////
#pragma competitionControl(Competition)//
#include "Vex_Competition_Includes.c"////
/////////////////////////////////////////
void pre_auton()
{
	bStopTasksBetweenModes = true;
	bDisplayCompetitionStatusOnLcd = false;
	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
}
void drive(int speed)//Some of these might need to be made negative
{
	motor [frontLeft] = speed;
	motor [frontRight] = speed;
	motor [backRight] = speed;
	motor [backLeft] = speed;
}
void arm (int speed)//fallback simple method
{
	motor [armL1] = speed;
	motor [armL2] = speed;
	motor [armR1] = speed;
	motor [armR2] = speed;
}
void grab (int speed)
{
	motor [grabL] = motor[grabR] = speed;
}
task autonomous()
{
	bool a = true;
	clearTimer(T1);
	while(a)
	{
		if(time1(T1)<3000)
			//	arm(-127);
		if(time1(T1)>60000)
			a=false;
	}
}
task usercontrol()
{
	while (true)
	{
		float x = vexRT[Ch4];
		float y = vexRT[Ch3];
		//Left Joystick Arcade Drive/
		motor [frontLeft] = y + x;///
		motor [backLeft] = y + x;////
		motor [frontRight] = y - x;//
		motor [backRight]  = y - x;//
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
		else if (vexRT[Btn8D] == 1 )//
		{														//
			grab(-127);								//
		}														//
		else												//
		{														//
			grab(0);									//
		}														//
		//////////////////////////////
		//Deadband////////////////////
		int deadBand = 10;
		if(x > deadBand || x < -deadBand)
		{
			x = x;
		}
		else
		{
			x = 0;
		}
		if(y > deadBand || y < -deadBand)
		{
			y = y;
		}
		else
		{
			y = 0;
		}
	}
}
