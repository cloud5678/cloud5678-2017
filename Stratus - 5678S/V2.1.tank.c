#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  catIME,         sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  liftIME,        sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           frontLeft,     tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           frontRight,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           rearLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           rearRight,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           gb1,           tmotorVex393_MC29, PIDControl, encoderPort, I2C_1)
#pragma config(Motor,  port6,           gb2,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           gb3,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           gb4,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           lift1,         tmotorVex393_MC29, PIDControl, encoderPort, I2C_2)
#pragma config(Motor,  port10,          lift2,         tmotorVex393_HBridge, openLoop)
#pragma config(DatalogSeries, 0, "", Motors, MotorPWM, port5, 50)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton()
{
	// Set bStopTasksBetweenModes to false if you want to keep user created tasks
	// running between Autonomous and Driver controlled modes. You will need to
	// manage all user created tasks if set to false.
	bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task autonomous()
{
	// ..........................................................................
	// Insert user code here.
	// ..........................................................................
	slaveMotor(gb2,gb1);
	slaveMotor(gb3, gb2);
	slaveMotor(gb4, gb3);
	bool a = true;
	clearTimer(T1);
	while(a)
	{
		if(time1(T1)<3000)
			cat(-127);
		if(time1(T1)>60000)
			a=false
	}
	// Remove this function call once you have "real" code.
	AutonomousCodePlaceholderForTesting();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void driveR1ght(int speed)//controlls the speed of the right side motors
{

	motor [frontRight] = -speed;
	motor [rearRight] = -speed;

}
void driveL3ft(int speed)//controlls the speed of the the left side motors
{
	motor [frontLeft] = speed;
	motor [rearLeft] = speed;
}
void cat(int speed)//controlls the speed of the motor that power the arm
									 //if speed is negative the arm goes up, if postivie the arm goes down.
{
	motor[gb1] = speed;
}

task usercontrol()
{
	// User control code here, inside the loop
	slaveMotor(gb2,gb1);
	slaveMotor(gb3, gb2);
	slaveMotor(gb4, gb3);
	while (true)
	{
		/*motor[frontLeft] = vexRT[Ch1] + vexRT[Ch2];//x drive arcade
		motor[rearRight] =  vexRT[Ch1] - vexRT[Ch2];
		motor[rearLeft] = vexRT[Ch4] + vexRT[Ch3];
		motor[frontRight] =  vexRT[Ch4] - vexRT[Ch3];*/
		/*motor[frontLeft] =vexRT[Ch3];//tank drive arcade
		motor[rearLeft]=vexRT[Ch3];
		motor[frontRight]= -vexRT[Ch2];
		motor[rearRight]=-vexRT[Ch2];*/
		driveL3ft(Ch3);
		driveR1ght(Ch2);
		if (vexRT[Btn6U]==1)
		{
			cat(-127);//up
		}
		else if(vexRT[Btn6D] == 1)
		{
			cat(127);//down
		}
		//for catapult with high tension
		/*else if(vexRT[Btn5D]==1)
		{
		cat(10);//hold
		}*/
		else
		{
			cat(0);//no power
		}
	}
}
