/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2013                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*        Module:     ImeLib.c                                                 */
/*        Author:     James Pearman                                            */
/*        Created:    6 March 2013                                             */
/*                                                                             */
/*        Revisions:  V0.1                                                     */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*        Description:                                                         */
/*                                                                             */
/*        Library to monitor all IMEs and flywheel over lost values if the     */
/*        I2C bus is reset for any reason.                                     */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */

#ifndef __IME_LIB__
#define __IME_LIB__

// comment out this line if you want to see the variables from this file
// in the globals window. (you may have to restart ROBOTC).
#pragma systemFile

// We need features only available in V3.60 and greater
#include "FirmwareVersion.h"
#if kRobotCVersionNumeric < 360
#error "IME Library requires ROBOTC Version 3.60 or newer"
// We cannot compile but want to stop other errors so define
// the core functions
void    IMEInit() {}
long    IMEGetEncoder( tMotor index ) {}
short   IMESetEncoder( tMotor index, long count = 0 ) {}
#else

// structure to hold the encoder data
// should probably try and reduce memory usage but for now
// it's ok, tried char for the delta array but it did not store
// negative values.  32 bytes used for each motor, 320 bytes for
// all 10.
typedef struct _ime_data
{
	unsigned char   installed;
	unsigned char   delta_ptr;
	short           deltas[10];
	long            value;
	long            offset;
} ime_data;

// storage for all possible encoders
static ime_data _imes[ kNumbOfRealMotors ];
// a semaphore so we can set the encoder synchronously
static TSemaphore  _ImeSemaphore;

// forward ref
task IMEMonitorTask();

/*-----------------------------------------------------------------------------*/
/*  Init the encoder flywheel task, runa at a higher than "normal" priority  */
/*-----------------------------------------------------------------------------*/

void
IMEInit()
{
	if( getTaskState( IMEMonitorTask ) == taskStateStopped )
	{
		startTask( IMEMonitorTask, 10 );
		// Give new task time to start
		wait1Msec(5);
	}
}

/*-----------------------------------------------------------------------------*/
/*  Get the semaphore we use to allow several encoder structure vaiables to be */
/*  set without the main monitoring task using them in a partially set state.  */
/*  timeout is set at 2mS, far lomger than we should ever need                 */
/*-----------------------------------------------------------------------------*/

int
IMEGetSemaphore()
{
	semaphoreLock( _ImeSemaphore, 2);

	short s = getSemaphoreTaskOwner(_ImeSemaphore);

	if ( s == nCurrentTask )
		return(1);
	else
		return(0);
}

/*-----------------------------------------------------------------------------*/
/*  Release the semaphore, check that we own it first                          */
/*-----------------------------------------------------------------------------*/

void
IMEReleaseSemaphore()
{
	short s = getSemaphoreTaskOwner(_ImeSemaphore);
	if ( s == nCurrentTask )
		semaphoreUnlock(_ImeSemaphore);
}

/*-----------------------------------------------------------------------------*/
/*  Get a motor encoder value - use this instead of nMotorEncoder for reading  */
/*  encoder values.  index should be the motor port                            */
/*-----------------------------------------------------------------------------*/

long
IMEGetEncoder( tMotor index )
{
	return( _imes[index].value + _imes[index].offset );
}

/*-----------------------------------------------------------------------------*/
/*  Set motor encoder value                                                    */
/*  One advantage of this over using nMotorEncoder (or SensorValue) is that    */
/*  non zero values can also be set                                            */
/*-----------------------------------------------------------------------------*/

short
IMESetEncoder( tMotor index, long count = 0 )
{
	if( IMEGetSemaphore() )
	{
		_imes[index].value  = 0;
		nMotorEncoder[ index ] = 0;
		_imes[index].offset = count;
		IMEReleaseSemaphore();
		return(1);
	}
	else
		return(0);
}

/*-----------------------------------------------------------------------------*/
/*  task to monitor all installed IMEs and flywheel over lost values if the    */
/*  I2C bus is reset.                                                          */
/*-----------------------------------------------------------------------------*/

task
IMEMonitorTask()
{
	TI2cStatistics i2c_stats;

	int     i, j;
	short   delta;
	long    value;
	long    lost_comms_timer;
	int     resume_comms_timeout = 0;
	ime_data *p;

	// Initialize resource semaphore
	semaphoreInitialize( _ImeSemaphore );

	IMEGetSemaphore();

	// Clear data
	for(i=0;i<kNumbOfRealMotors;i++)
	{
		_imes[ i ].installed = 0;
		_imes[ i ].delta_ptr = 0;
		_imes[ i ].value     = 0;
		_imes[ i ].offset    = 0;

		for(j=0;j<10;j++)
			_imes[ i ].deltas[j] = 0;
	}

	// Learn which motors have IMEs
	// cannot do this in a loop as getEncoderForMotor only accepts constants
	if( getEncoderForMotor( port1 ) >= 20 ) _imes[ port1 ].installed = 1;
	if( getEncoderForMotor( port2 ) >= 20 ) _imes[ port2 ].installed = 1;
	if( getEncoderForMotor( port3 ) >= 20 ) _imes[ port3 ].installed = 1;
	if( getEncoderForMotor( port4 ) >= 20 ) _imes[ port4 ].installed = 1;
	if( getEncoderForMotor( port5 ) >= 20 ) _imes[ port5 ].installed = 1;
	if( getEncoderForMotor( port6 ) >= 20 ) _imes[ port6 ].installed = 1;
	if( getEncoderForMotor( port7 ) >= 20 ) _imes[ port7 ].installed = 1;
	if( getEncoderForMotor( port8 ) >= 20 ) _imes[ port8 ].installed = 1;
	if( getEncoderForMotor( port9 ) >= 20 ) _imes[ port9 ].installed = 1;
	if( getEncoderForMotor( port10) >= 20 ) _imes[ port10].installed = 1;

	IMEReleaseSemaphore();

	while(1)
	{
		// 10mS delay, don't change this
		wait1Msec(10);

		// Get the I2C bus statistics
		getI2CStatistics(&i2c_stats, sizeof(i2c_stats));

		// skip if cannot get the semaphore
		if( !IMEGetSemaphore() )
			continue;

		// Check I2C communication good flag
		if( !i2c_stats.bI2CNeverResponded )
		{
			if(resume_comms_timeout == 0)
			{
				if( lost_comms_timer != 0 )
				{
					// Only add if comms lost for less than 1 second
					//writeDebugStreamLine("Comms lost for %dmS", lost_comms_timer * 10);
					if( lost_comms_timer < 100 )
					{
						// compensate for lost communications
						for(i=0;i<kNumbOfRealMotors;i++)
						{
							p = &_imes[i];

							if(p->installed)
							{
								// calculate delta for last 100mS of known encoder use
								delta = 0;
								for(j=0;j<10;j++)
									delta += p->deltas[j];

								// add to offset an estimate of how many counts we lost
								p->offset += ((delta * lost_comms_timer) / 10);
								//writeDebugStreamLine("motor %d had %d added to offset", i, ((delta * lost_comms_timer) / 10) );
								nMotorEncoder[i] = 0;
							}
						}
					}

					// clear timer
					lost_comms_timer = 0;
				}

				// For each motor
				for(i=0;i<kNumbOfRealMotors;i++)
				{
					p = &_imes[i];

					// Does this motor have an IME ?
					if( p->installed )
					{
						// Read encoder
						value = nMotorEncoder[i];

						// Calculate delta from last time
						delta = value - p->value;
						p->value = value;

						// store in the delta history array
						p->deltas[p->delta_ptr] = delta;

						// increment delta history index
						if(++p->delta_ptr == 10)
							p->delta_ptr = 0;
					}
				}
			}
			else
			{
				resume_comms_timeout--;
				lost_comms_timer++;
			}
		}
		else
		{
			if(resume_comms_timeout == 0)
			{
				// First time around store new offset
				for(i=0;i<kNumbOfRealMotors;i++)
				{
					p = &_imes[i];

					// If IME installed
					if( p->installed )
					{
						// Calculate new offset
						p->offset = p->value + p->offset;
						p->value  = 0;
					}
				}
			}

			// Wait 100mS after comms is resumed.
			resume_comms_timeout = 10;

			// 10mS more of lost comms
			lost_comms_timer++;
		}

		// done for now
		IMEReleaseSemaphore();
	}
}

#endif  // version test
#endif  // __IME_LIB__
