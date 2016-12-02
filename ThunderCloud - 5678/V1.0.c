#pragma config(Motor,  port1,           left1,         tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           left2,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           left3,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           right1,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           right2,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           left4,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           left5,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           right3,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           right4,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          right5,        tmotorVex393_HBridge, openLoop)
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

task usercontrol()
{
  // User control code here, inside the loop

  while (true)
  {
  float x1 = vexRT[Ch4];
		float y1 = vexRT[Ch3];
		float x2 = vexRT[Ch1];
		float y2 = vexRT[Ch2];
		//Left Joystick Arcade Drive///
		motor [left1] = motor [left2] = motor [left3] = motor [left4] = motor [left5] = y1 + x1;///
		motor [right1] = motor [right2] = motor [right3] = motor[right4] = motor[right5]  = y2 - x2;//
  }
}
