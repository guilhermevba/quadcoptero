/*
* quadcopter_control.cpp
*
* Created: 29/5/2015 9:43:11
*  Author: Guilherme
*/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "motor.h"
#include "mpu6050.h"
#include "Arduino.h"
#include "interrupt.h"
#include "control.h"
#include "WString.h"

#define maxMin 0 // 1 para configurar os maximos e minimos dos ESC
#define ensaio 0 //1 em modo aquisição, 0 em modo controle

#define t 10 //mili seconds
//#define T 0.01 //seconds
#define UART_BAUD_RATE 115200
#define kinit 100
#define kmax kinit+1500
#define toRad 500/(32768*57.3)
#define gravity 9.81/16384
#define a_te   0.9
int k=0; //sample counter

#pragma region motor control
#define MAX_SIGNAL 255//2100
#define MIN_SIGNAL 127//980
int DC_VALUE =190;
int u1 = DC_VALUE;
int u2 = DC_VALUE;
float ur=0;
#pragma endregion motor control


#pragma region sensors auxiliary variables
float ayF=0, azF=0, axF=0;
float Ax=0, Ay=0,Az=0;
float Gx=0, Gy=0, Gz=0;
float xAcc=0,yAcc=0;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float x=0, dx=0, y=0, dy=0;
#pragma endregion sensors auxiliary variables

bool amostra=false;

void newSample();
void loop();
void setup();
void z_gyro();
void sensor_read();
void aplicaCtrl();
void normalizeUr();
void calibraESC();
void identifica();


void newSample()
{
	TCNT1 = interruptionPresetValue;
	mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
	k += 1;
	if (k<=kinit)
	{
		z_gyro();
	}
	else if(k>kmax)
	{
		stopInterrupt();
		Serial.println("Fim do programa...");
		setSpeed(motorXfront,MIN_SIGNAL);
		setSpeed(motorXback,MIN_SIGNAL);
		setSpeed(motorYfront,MIN_SIGNAL);
		setSpeed(motorYback,MIN_SIGNAL);
	}
	else
	{
		sensor_read();
		#if ensaio
		identifica();
		#else
		if(k==1000) setRef(1,0);
		aplicaCtrl();
		#endif
		
	}
}

void calibraESC()
{
	motorConfig(cpPWM_500,false);
	#if maxMin
	setSpeed(motorxfront,max_signal);
	setspeed(motorxback, max_signal);
	setspeed(motoryfront,max_signal);
	setspeed(motoryback, max_signal);
	serial.print("again...");
	while (!serial.available());
	serial.read();
	#endif

	setSpeed(motorXfront,MIN_SIGNAL);
	setSpeed(motorXback, MIN_SIGNAL);
	setSpeed(motorYfront,MIN_SIGNAL);
	setSpeed(motorYback, MIN_SIGNAL);
	Serial.flush();
	Serial.println(F("Wait the bits and send something again...\n amostras=["));
	while (!Serial.available());
	Serial.read();
}

void normalizeUr()
{
	u1 = DC_VALUE + ur;
	u2 = DC_VALUE - ur;
	if(u1 < MIN_SIGNAL) u1 = MIN_SIGNAL;
	else if(u1 > MAX_SIGNAL) u1 = MAX_SIGNAL;
	if(u2 < MIN_SIGNAL) u2 = MIN_SIGNAL;
	else if(u2 > MAX_SIGNAL) u2 = MAX_SIGNAL;
	
	//Serial.println(String(x)+" "+String(y)+" "+String(u1));
	
}

void aplicaCtrl()
{
	ur=execX(x,dx);
	normalizeUr();
	setSpeed(motorXfront,u2);
	setSpeed(motorXback,u1);
	//Serial.println(String(refx)+" "+String(x));
	ur=execY(y,dy);
	normalizeUr();
	setSpeed(motorYfront,u2);
	setSpeed(motorYback,u1);
	Serial.println(String(refy)+" "+String(y));
}

void identifica()
{
	//if(k <= 250) ur = 0;
	//if(k > 400)  ur= 1;
	//if(k > 600)  ur= -1;
	//if(k > 800) ur = 3;
	//if(k > 950) ur = -3;
	//if(k > 1100) ur = 5;
	//if(k > 1200) ur = -5;
	//if(k > 1300) ur = 7;
	//if(k > 1350) ur = -7;
	//if(k > 1400) ur = 0;
	  	if(k <= 300) ur = 0;
	  	if(k > 300)  ur= 2;
	  	if(k > 470)  ur= -3;
	  	if(k > 780) ur = 2;
	  	if(k > 1050) ur = -1;
	  	if(k > 1240) ur = 3;
	  	if(k > 1375) ur = -2;
	  	if(k > 1450) ur = 0;
	normalizeUr();
	setSpeed(motorXfront,u1);
	setSpeed(motorXback,u2);
}

void sensor_read()
{

	// gyro lido em [rad/s]
	dy  = (-gx-Gx)*toRad;
	dx  = (gy-Gy)*toRad;
	#if ensaio
	Serial.println(String(dx*1000)+" "+String(ur));
	return;
	#endif
	axF = -(ax-Ax)*gravity;
	ayF = -(ay-Ay)*gravity;
	azF = (az-Az)*gravity;
	xAcc=atan2(axF,hypot(ayF,azF));
	x = a_te*(T*dx+x) + (1-a_te)*xAcc;
	//x=(T*dx+x);
	yAcc= atan2(ayF,hypot(axF,azF));
	y = a_te*(T*dy+y) + (1-a_te)*yAcc;
	//y=(T*dy+y);
}

void z_gyro()
{
	if(k<kinit)
	{
		Ax+=ax;
		Ay+=ay;
		Az+=az;
		Gx+=gx;
		Gy+=gy;
		Gz+=gz;
	}
	else if (k==kinit)
	{
		Serial.println("zerando sensor...");
		Ax=Ax/kinit;
		Ay=Ay/kinit;
		Az=Az/kinit-MPU6050_ACCEL_LSB_2;
		
		Gx=Gx/kinit;
		Gy=Gy/kinit;
		Gz=Gz/kinit;
	}
}

void initRFCtrl()
{
	
	DDRD  &= ~(1<<PD4);								//PD4 set as input
	PORTD |= (1<<PD4);								//Turn on the pull-up
	DDRC  &= ~(1<<PC5);								//PC5 set as input
	PORTD |= (1<<PC5);								//Turn on the pull-up
	DDRB  &= ~(1<<PB5);								//PB5 set as input
	PORTB |= (1<<PB5);								//Turn on the pull-up
	EICRA |= (1<<ISC00);							//Any logical change to INT0 generetes an interrupt 
	EIMSK=(1<<INT0);
	PCICR |= ((1<<PCIE2)|(1<<PCIE1)|(1<<PCIE0));	//Pin Change Interrupt Control Register
	PCMSK0 = (1<<PCINT5);							//PB5 pin
	PCMSK1 = (1<<PCINT13);							//PC5 pin
	PCMSK2 = (1<<PCINT20);							//PD4 pin
}

int main(void)
{
	setup();
	while(1)
	{
		loop();
	}
	
}

void setup()
{
	interruptPointer= &newSample;
	Serial.begin(UART_BAUD_RATE);
	Serial.flush();
	#if ensaio
	Serial.println(F("[ENSAIO] Initializing..."));
	#else
	Serial.println(F("Initializing..."));
	#endif
	mpu6050_init();
	Serial.println(mpu6050_testConnection()?F("MPU OK") : F("MPU NOK"));
	calibraESC();
	initInterrupt(T);
};

void loop()
{
};

/*
 *	Interrupt that handle logical change on pin PB5 attached on the throttle of our RF controller
 */
ISR(PCINT0_vect)
{
}

/*
 *	Interrupt that handle logical change on pin PC5 attached on the pitch of our RF controller
 */
ISR(PCINT1_vect)
{
}

/*
 *	Interrupt that handle logical change on pin PB5 attached on the yaw of our RF controller
 */
ISR(PCINT2_vect)
{
}