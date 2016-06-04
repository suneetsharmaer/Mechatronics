/*
 * motorControl.h
 *
 * Created: 12/6/2014 1:42:45 AM
 *  Author: chitvan
 */ 
#ifndef motorControl__
#define motorControl__

// 								Motor Running code definitions 
// ---------------------------------------------------------------------------------------//
//									Function Definitions
void MotorSpeed(char Motor, int Speed);
void Timer0Init();
//int MotorControl(float );

// ---------------------------------------------------------------------------------------//
//									Variable Definitions
									
#define MPortSelect DDRC
#define DirPortSelect DDRB
#define Active 0				// Bit in MotorStatus
#define LMDirection 1			// Bit in MotorStatus
#define RMDirection 2			// Bit in MotorStatus
#define FirstGo	3				// Bit in MotorStatus to check for first run
#define Active 0				// Bit in MotorStatus
#define MotorPWMPort PORTC
#define LMPin PORTC6
#define RMPin PORTC7
#define MotorDirectionPort PORTB
#define LMPos PORTB4
#define LMNeg PORTB5
#define RMPos PORTB6
#define RMNeg PORTB7

// ---------------------------------------------------------------------------------------//
//									Motor Control Code
// ---------------------------------------------------------------------------------------//
//									Function Definitions
void motorControl(int theta, int *motorPWMValues, char maxSpeed);
// initialize the USB subsystem

// ---------------------------------------------------------------------------------------//
//									Variable Definitions
#define LeftMotor 0
#define RightMotor 1

#endif