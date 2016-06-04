/*
 * Acrobat_v2.c
 *
 * Created: 12/22/2014 2:35:09 PM
 *  Author: chitvan
 */ 


#include <avr/io.h>
#include "m_imu.h"
#include "m_usb.h"
#include "m_rf.h"
#include "motorControl.h"

#define Kp 80
#define Kd -5
#define Ki 0
#define tiltPosition 1		// ay changing with tilting
#define rotationAxis 3		// Rotation along x axis
#define AccelScale 0
#define GyroScale 0
#define gValue 16384
#define pi 3.14159265
#define del_t 0.01			// time period of readings in seconds
#define alpha 0.1			// Acceleration
#define beta 0.8

void Timer1Init();
void printMIMU();
void printMatlab();
void findTheta();


// Global Variables
int rawData[9] , i , speed,	count;
float data[9],mean_data[6] = {140.258,-229.11,276.73,-92.614,181.6275,-129.1410},theta;
float theta_accel, theta_gyro=0, omega_gyro, theta_gyro_prev;
float theta_err, prev_err=0,theta_prev = 0, sum_err=0, diff_err;
volatile int flag;

int main(void)
{
	m_clockdivide(0);
	m_red(ON);
	m_bus_init();
	m_usb_init();
	Timer1Init();
	while( !( m_imu_init(AccelScale, GyroScale) ) );
	m_red(OFF);
	while(1)
	{
//		m_usb_tx_int(flag);
		if(flag>0)
		{
			//cli();
/*			count = count+flag;*/
			findTheta();
			m_green(ON);
//  			m_usb_tx_string("\nError in Theta : ");
//  			m_usb_tx_int(180*theta_err/pi);
		}
		else continue;
		
		sum_err += theta_err;
		
		
// 		if (count>20)
// 		{
// 			count = 0;
			diff_err = theta_err-prev_err;
//			speed = Kp*theta_err + Kd*diff_err + Ki*sum_err;
			if (theta_err<0)
			speed = -Kp*sqrt(-theta_err) + Kd*diff_err + Ki*sum_err;
			else
			speed = Kp*sqrt(theta_err) + Kd*diff_err + Ki*sum_err;
			if(speed>240)
			speed = 255;
			else if(speed<-240)
			speed = -255;
			else if(speed>0)
			speed += 12;
			else if(speed<0)
			speed -= 12;
			MotorSpeed('l',speed);
			MotorSpeed('r',speed);
			m_usb_tx_string("\nError in Theta : ");
			m_usb_tx_int(10000*theta_err);
			m_usb_tx_string("\nMotor Speeds : ");
			m_usb_tx_int(speed);
			prev_err = theta_err;
//		}
		
		
		//sei();
		
	}
}

void Timer1Init()
{
	sei();
	clear(TCCR1B,WGM13);						// Upto OCR1A
	set(TCCR1B,WGM12);							// Upto OCR1A
	TCCR1A &= ~( (1<<WGM11) | (1<<WGM10) );		// ^^
	set(TIMSK1,OCIE1A);							// OCR1A compare interrupt
	OCR1A = 625;								// 
	TCCR1B |= ( (1<<CS12));						// freq/256
	TCCR1B &= ~((1<<CS10) | (1<<CS11));			// ^^
}

ISR(TIMER1_COMPA_vect)
{
	cli();
	m_green(OFF);
	flag++;
	sei();
}

void printMIMU()
{
	m_usb_tx_string("\nAcceleration : (");
	m_usb_tx_int(data[0]);
	m_usb_tx_string(" , ");
	m_usb_tx_int(data[1]);
	m_usb_tx_string(" , ");
	m_usb_tx_int(data[2]);
	m_usb_tx_string(")");
	m_usb_tx_string("\t\tGyroscope : (");
	m_usb_tx_int(data[3]);
	m_usb_tx_string(" , ");
	m_usb_tx_int(data[4]);
	m_usb_tx_string(" , ");
	m_usb_tx_int(data[5]);
	m_usb_tx_string(")");
	m_usb_tx_string("\t\tMagnetometer : (");
	m_usb_tx_int(data[6]);
	m_usb_tx_string(" , ");
	m_usb_tx_int(data[7]);
	m_usb_tx_string(" , ");
	m_usb_tx_int(data[8]);
	m_usb_tx_string(")");
}

void printMatlab()
{
	char rx_buffer; //computer interactions
	while(!m_usb_rx_available());  	//wait for an indication from the computer
	rx_buffer = m_usb_rx_char();  	//grab the computer packet
	
	m_usb_rx_flush();  				//clear buffer
	
	if(rx_buffer == 1)
	{  			//computer wants ir data
		//write ir data as concatenated hex:  i.e. f0f1f4f5
		for (i = 0 ; i < 9 ; i++)
		{
			m_usb_tx_int(data[i]);
			m_usb_tx_char('\t');
		}
		m_usb_tx_char('\n');  //MATLAB serial command reads 1 line at a time
	}
}

void findTheta()
{
	while(!m_imu_raw(rawData));
	for(i=1;i<6;i++)
	{
		data[i] = rawData[i]-mean_data[i];
	}
	//		printMIMU();
	//		printMatlab();
	m_green(TOGGLE);
	theta_accel = atan2(*(data+2),*(data+tiltPosition));
//  	m_usb_tx_string("\nTheta Accel : ");
//  	m_usb_tx_int(180*theta_accel/pi);
	if(theta_gyro==0)
	{
		theta_gyro = theta;
	}
	else
	theta_gyro = theta - data[rotationAxis]*pi*125*flag*del_t/(16384*180.0);
	flag=0;
//  	m_usb_tx_string("\t\tTheta Gyro : ");
//  	m_usb_tx_int(180*theta_gyro/pi);
	theta = alpha*theta_accel + (1-alpha)*theta_gyro;
	theta_gyro_prev = theta_gyro;
	theta_err = beta*(pi/2-theta) + (1-beta)*theta_prev;
	theta_prev = theta_err;
	//return(theta_err)
}