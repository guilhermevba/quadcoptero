/*
* control.cpp
*
* Created: 1/6/2015 10:57:57
* Author: Guilherme
*/


#include "control.h"
#include <math.h>
#include "Arduino.h"
// default constructor


void setRef(float rx,float ry)
{
	refx=rx;
	refy=ry;
}
float execX(float pos,float w)
{
	float erro=0,e=0;
	e=refx-pos;
	// PD control
	//ur=KP*(e)-KD*w;
	// PD control with Sigmoid function on Kd
	if (refx!=rx0)
	{
		e0=e;
		rx0=refx;
	}
	if (e0==0)
	{
		e0=e;
	}
	
	erro=e/e0;
	
	sigKD=(1-1/(1+exp(beta*(-abs(erro)+gama))))*KD*w;
	
	itx=itx+e*T*KI;
	
	U=e*KP-sigKD+itx;
	
	qu=(U2*nQ1-dQ2*qu0-dQ3*qu1-dQ4*qu2)/dQ1;
	
	qgu=(nQG1*w+nQG2*y1+nQG3*y2+nQG4*y3-dQG4*qgu2-dQG3*qgu1-dQG2*qgu0)/dQG1;
	
	

	qgu2=qgu1;
	qgu1=qgu0;
	qgu0=qgu;

	qu2=qu1;
	qu1=qu0;
	qu0=qu;

	y3=y2;
	y2=y1;
	y1=w;

	U2=U1;
	U1=U0;
	U0=U;
	
	ud=qgu-qu;
	return U-ud;
	
}

float execY(float pos,float w)
{
	float erro=0,e=0;
	e=refy-pos;
	// PD control
	//ur=KP*(refy-y)-KD*dy;
	
	//calculo do erro percentual
	if (refy!=ry0)
	{
		e0y=e;
		ry0=refy;
	}
	if (e0y==0)
	{
		e0=e;
	}
	erro=e/e0y;
	// PD control with Sigmoid function on Kd
	sigKD=(1-1/(1+exp(beta*(-abs(erro)+gama))))*KD*w;
	
	ity=ity+e*T*KI;
	
	U=e*KP-sigKD+ity;
	
	//compensador de disturbio
	qu=(U2y*nQ1-dQ2*qu0y-dQ3*qu1y-dQ4*qu2y)/dQ1;

	qgu=(nQG1*w+nQG2*y1y+nQG3*y2y+nQG4*y3y-dQG4*qgu2y-dQG3*qgu1y-dQG2*qgu0y)/dQG1;


	qgu2y=qgu1y;
	qgu1y=qgu0y;
	qgu0y=qgu;

	qu2y=qu1y;
	qu1y=qu0y;
	qu0y=qu;

	y3y=y2y;
	y2y=y1y;
	y1y=w;

	U2y=U1y;
	U1y=U0y;
	U0y=U;
	ud=qgu-qu;
	return U-ud;
	
}