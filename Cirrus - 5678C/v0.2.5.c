#pragma config(Motor,  port1,           clawR,         tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           topRight,      tmotorVex393_MC29, openLoop, reversed, encoderPort, None)
#pragma config(Motor,  port3,           topLeft,       tmotorVex393_MC29, openLoop, reversed, encoderPort, None)
#pragma config(Motor,  port4,           bottomRight,   tmotorVex393_MC29, openLoop, encoderPort, None)
#pragma config(Motor,  port5,           bottomLeft,    tmotorVex393_MC29, openLoop, encoderPort, None)
#pragma config(Motor,  port6,           armR1,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           armR2,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           armL1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           armL2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          clawL,         tmotorVex393_HBridge, openLoop)
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
/*                             Global Functions                              */
/*---------------------------------------------------------------------------*/
void drive(int speed, int angle) { //this may or may not work; attempt to merge turn and drive code together
	int x = cosDegrees(angle) * speed;
	int y = sinDegrees(angle) * speed;
	motor[topLeft] = x + y;
	motor[bottomLeft] = x + y;
	motor[topRight] = x - y;
	motor[bottomRight] = x - y;
}

void clawOpen() {
	motor[clawR] = motor[clawL] = 127;
	wait1Msec(200);
	motor[clawR] = motor[clawL] = 0;
}

void clawClose() {
	motor[clawR] = motor[clawL] = -127;
	wait1Msec(200);
	motor[clawR] = motor[clawL] = 0;
}

void throw() {
	motor[armR1] = motor[armR2] = motor[armL1] = motor[armL2] = 127;
	wait1Msec(800);
	clawOpen();
	motor[armR1] = motor[armR2] = motor[armL1] = motor[armL2] = -127;
	wait1Msec(800);
	clawClose();
	motor[armR1] = motor[armR2] = motor[armL1] = motor[armL2] = 0;
}


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton() {
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

task autonomous() {
	drive(-127, 0);
	wait1Msec(3000);
	drive(0, 0);
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

task usercontrol() {
	// User control code here, inside the loop
	bool manual = true;
	while (true) {
		// This is the main execution loop for the user control program.
		// Each time through the loop your program should update motor + servo
		// values based on feedback from the joysticks.

		// ........................................................................
		// Insert user code here. This is where you use the joystick values to
		// update your motors, etc.
		// ........................................................................
		if(vexRT[Btn8R] == 1) {
			throw();
			manual = false;
		}
		if(vexRT[Btn6U] == 1) {
			clawOpen();
			manual = false;
		} else if(vexRT[Btn6D] == 1) {
			clawClose();
			manual = false;
		}

		if(manual) {
			motor[armR1] = motor[armR2] = motor[armL1] = motor[armL2] = vexRT[Ch3];
			if(vexRT[Btn5U] == 1) {
				motor[clawR] = motor[clawL] = 127;
				wait1Msec(50);
				motor[clawR] = motor[clawL] = 0;
			} else if(vexRT[Btn5D] == 1) {
				motor[clawR] = motor[clawL] = -127;
				wait1Msec(50);
				motor[clawR] = motor[clawL] = 0;
			}
		}
		//arcade drive
		motor[topLeft] = vexRT[Ch2] - vexRT[Ch1];
		motor[bottomLeft] = vexRT[Ch2] - vexRT[Ch1];
		motor[topRight] = vexRT[Ch2] + vexRT[Ch1];
		motor[bottomRight] = vexRT[Ch2] + vexRT[Ch1];
		manual = true;
	}
}