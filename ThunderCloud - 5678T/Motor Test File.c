#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, dgtl1,  PELevel,        sensorDigitalIn)
#pragma config(Sensor, I2C_1,  leftE,          sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  lArmE,          sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  rArmE,          sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_4,  rightE,         sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           grabR,         tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           armR3,         tmotorVex393_MC29, openLoop, encoderPort, I2C_3)
#pragma config(Motor,  port3,           right,         tmotorVex393_MC29, openLoop, driveRight, encoderPort, I2C_4)
#pragma config(Motor,  port4,           armR1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           armL2,         tmotorVex393_MC29, openLoop, encoderPort, None)
#pragma config(Motor,  port6,           armL3,         tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_2)
#pragma config(Motor,  port7,           left,          tmotorVex393_MC29, openLoop, driveLeft, encoderPort, I2C_1)
#pragma config(Motor,  port8,           armL1,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           armR2,         tmotorVex393_MC29, openLoop, reversed, encoderPort, None)
#pragma config(Motor,  port10,          grabL,         tmotorVex393_HBridge, openLoop, reversed, encoderPort, None)
#pragma config(DatalogSeries, 0, "Battery Status", Sensors, Sensor, in8, 500)
#pragma config(DatalogSeries, 1, "Battery Level", Properties, immediateBatteryLevel, , 500)
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
void stopAll ()
{
	motor[armL2] = 0;
	motor[armR2] = 0;
	motor[armL1] = 0;
	motor[armL3] = 0;
	motor[armR1] = 0;
	motor[armR3] = 0;
}
task autonomous()
{
	motor[armR3] = 127;//correct
	wait1Msec(500);
	stopAll();
	wait1Msec(10);
	motor[armR2] = 127;//correct
	wait1Msec(500);
	stopAll();
	wait1Msec(10);
	motor[armR1] = 127;//Left1
	wait1Msec(500);
	stopAll();
	wait1Msec(10);
	motor[armL1] = 127;//Right1
	wait1Msec(500);
	stopAll();
	wait1Msec(10);
	motor[armL2] = 127;//Correct
	wait1Msec(500);
	stopAll();
	wait1Msec(10);
	motor[armL3] = 127;//Correct
	wait1Msec(500);
	stopAll();
	wait1Msec(10);

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
    motor[armL1] = vexRT[Btn7L]*127;//Correct
    motor[armL2] = vexRT[Btn7U]*127;//Check
    motor[armL3] = vexRT[Btn7R]*127;//Correct
    motor[armR1] = vexRT[Btn8L]*127;//Correct
    motor[armR2] = vexRT[Btn8U]*127;//Correct
    motor[armR3] = vexRT[Btn8R]*127;//Correct

  }
}
