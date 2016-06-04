/*
 * motorControl.c
 *
 * Created: 12/6/2014 1:52:45 AM
 *  Author: chitvan
 */ 
 
#include "m_general.h"
#include "m_usb.h"
#include <avr/io.h>
#include "motorControl.h"

//#define MaxSpeed 255
volatile char MotorStatus = 0;
//#define LeftMotor 0
//#define RightMotor 1

// ***************************************************************************************//
//									Control Code

void motorControl(int theta, int *motorPWMValues, char MaxSpeed)
{
/* Function Information:	
	Explanation
		This function takes in the value of theta and a 2 element array, and sets the value required for the motor in the 
		array. The input is 1st element LeftMotor (#define), and second element as RightMotor
	Input variables
		theta is an integer giving value of direction of drive (puck or goal) in degrees.
		motorPWMValues is a 2 element array which the function will save data in.
*/
	if( (theta>=0) && (theta<=90) )
	{
		*(motorPWMValues + LeftMotor)	= MaxSpeed - (MaxSpeed/45.0)*theta;
		*(motorPWMValues + RightMotor) = MaxSpeed;
	}
	else if( (theta>=90) && (theta<=180) )
	{
		*(motorPWMValues + LeftMotor)	= -MaxSpeed;
		*(motorPWMValues + RightMotor) = MaxSpeed - (MaxSpeed/90.0)*(theta-90);
	}
	else if( (theta>=-90) && (theta<=0) )
	{
		*(motorPWMValues + LeftMotor)	= MaxSpeed;
		*(motorPWMValues + RightMotor) = MaxSpeed + (MaxSpeed/45.0)*theta;
	}
	else if( (theta>=-180) && (theta<=-90) )
	{
		*(motorPWMValues + LeftMotor)	= MaxSpeed + (MaxSpeed/90.0)*(theta+90);
		*(motorPWMValues + RightMotor) = -MaxSpeed;
	}
}

// ***************************************************************************************//
//									Motor Running Code

void MotorSpeed(char Motor, int Speed)
{
/* Function Information:	
	Explanation
		This function takes in the motor and its respective speeds, and sets the PWM setting for Timer 0
		running in Mode zero, as well as the respective direction setting.
		Frequency for Motor is 61.27Hz
	Input variables
		Motor input will be L/l for left Motor, or R/r for right motor
		Speed is the speed in -255 to 255 where -255 is the full speed of motor in reverse direction 
			and 255 is full speed of motor in positive direction
	
*/
	if(~check(MotorStatus,FirstGo))
	{
		Timer0Init();					// Initialize only first time
		set(MotorStatus,FirstGo);		// First go complete
	}
	
	switch (Motor)
	{
		case 'L':
		case 'l':
			set(MotorStatus,Active);										// Define motorPWMValues Variable,  #define Active bit 1 for active
			if (Speed<0)
			{
				Speed = -Speed;
				// Motor define and direction define for ISR subroutine
				clear(MotorStatus,LMDirection);								// #define LMDirection bit: 1 for forward 0 for backward
			}
			else
			{
				set(MotorStatus,LMDirection);								// Left Motor Direction : Forward
			}
			OCR0A = Speed;
			break;
			
		case 'R':
		case 'r':
			set(MotorStatus,Active);										// Define motorPWMValues Variable,  #define Active bit 1 for active
			if (Speed<0)
			{
				Speed = -Speed;
				// Motor define and direction define for ISR subroutine
				clear(MotorStatus,RMDirection);							// #define LMDirection bit: 1 for forward 0 for backward
			}
			else
			{
				set(MotorStatus,RMDirection);							// Left Motor Direction : Forward
			}
			OCR0B = Speed;
			break;
			
		default:
			clear(MotorStatus,Active);
			break;
	}
}

void Timer0Init()
{
	/*	Timer 0 initialization
		Mode 0 count upto 255
		Set pins LMPos, LMNeg, RMPos, RMNeg, LMPin, RMPin to output in Microcontroller
		Frequency low (around 60Hz) : set Timer prescaler (/1024) to adjust to that value
		ISR subroutine for OCR0A and OCR0B
		
	*/
		MPortSelect |= (1<<LMPin) | (1<<RMPin);		// Make the Motor pins to Output
		DirPortSelect |= (1<<LMPos) | (1<<LMNeg);	// Left Motor Direction Pin Enabled
		DirPortSelect |= (1<<RMPos) | (1<<RMNeg);	// Right Motor Direction Pin Enabled
		TCCR0B  &=	~(1<<WGM02);					// Mode 0
		TCCR0A	&=	~( (1<<WGM01) | (1<<WGM00) );	// Count upto FF and overflow
		sei();
		// Interrupts Timer Overflow, OCR0A compare and OCR0B compare
		TIMSK0	|=	(1<<TOIE0) | (1<<OCIE0A) | (1<<OCIE0B);
		
		TCCR0B  |=	((1<<CS02) | (1<<CS00));		// Prescaler set to /1024 & Start clock
		TCCR0B  &=	~(1<<CS01);						// Frequency around 60Hz Hz
}

ISR(TIMER0_OVF_vect)
{
	/*	Interrupt Explanation
		Checks the status of Register motorPWMValues
		Checks whether the motors need to be active
		Check the directions of the motors and sets the direction bits
		Checks the PWM high of the motors
	*/
	if (check(MotorStatus,Active))					// If the Motors are Active and supposed to run
	{
		MotorPWMPort |= (1<<LMPin) | (1<<RMPin);	// Start Both the Motors
		if (check(MotorStatus,LMDirection))			// Left Motor Direction Pins - Forward Direction
		{
			MotorDirectionPort	|=	(1<<LMPos);
			MotorDirectionPort	&=	~(1<<LMNeg);
		}
		else										// Left Motor Direction Pins - Reverse Direction
		{
			MotorDirectionPort	&=	~(1<<LMPos);
			MotorDirectionPort	|=	(1<<LMNeg);
		}
		if (check(MotorStatus,RMDirection))			// Right Motor Direction Pins - Forward Direction
		{
			MotorDirectionPort	|=	(1<<RMPos);
			MotorDirectionPort	&=	~(1<<RMNeg);
		}
		else										// Right Motor Direction Pins - Reverse Direction
		{
			MotorDirectionPort	&=	~(1<<RMPos);
			MotorDirectionPort	|=	(1<<RMNeg);
		}
	}
	else
	{
		MotorPWMPort &= ~((1<<LMPin) | (1<<RMPin));	// Stop Both the motors
	}
	
	/*	Debug
	*/
	//m_red(ON); m_green(ON);
}

ISR(TIMER0_COMPA_vect)
{
	/* Interrupt Explanation
		Resets the Left Motor Pin back to zero for PWM
	*/	
	MotorPWMPort &= ~(1<<LMPin);					// PWM clear part
	
	/*	Debug
	*/
		//m_red(OFF);				// Red intensity determines RM power
	
}

ISR(TIMER0_COMPB_vect)
{
	/* Interrupt Explanation
		Resets the Left Motor Pin back to zero for PWM
	*/
	MotorPWMPort &= ~(1<<RMPin);					// PWM clear part
	
	/*	Debug */
		//m_green(OFF);			// Green intensity determines LM power
	
}