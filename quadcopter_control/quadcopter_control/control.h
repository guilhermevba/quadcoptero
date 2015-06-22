/*
* control.h
*
* Created: 1/6/2015 10:57:57
* Author: Guilherme
*/


#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <math.h>
#include "Arduino.h"
#define simple 0
#define KD 6.8//10
#define KP 22//32
#define KI 5//
#define gama 0.75//0.58
#define beta 20//12
#define T 0.01

#define nQG1  0.000300547041582888
#define nQG2 -0.000885300309052029
#define nQG3  0.000869961019663900
#define nQG4 -0.000285204856060866

#define dQG1 -0.0000200092680446592
#define dQG2  0.0000587706363984809
#define dQG3 -0.0000575526521438772
#define dQG4  0.0000187906824174240

#define nQ1   0.000030054704158

#define dQ1   1
#define dQ2  -2.937170728449891
#define dQ3   2.876299723479332
#define dQ4  -0.939098940325283

float refx=0,refy=0,sig=0, r0=0,e0=0,e0y=0,rx0=0,ry0=0;
//float x=0, dx=0, y=0, dy=0;
float  qu0=0, qu1=0, qu2=0, qgu0=0, qgu1=0, qgu2=0, y1=0, y2=0, y3=0, U1=0, U2=0, U0=0;
float  qu0y=0, qu1y=0, qu2y=0, qgu0y=0, qgu1y=0, qgu2y=0, y1y=0, y2y=0, y3y=0, U1y=0, U2y=0, U0y=0;
float itx=0,ity=0;

float U=0, qu=0, qgu=0, sigKD=0, ud=0;


int execX(float pos, float speed);
int execY(float pos, float speed);
void setRef(float rx,float ry);



void setRef(float rx,float ry)
{
	refx=rx;
	refy=ry;
}
int execX(float pos,float w)
{
	float erro=0,e=0;
	e=refx-pos;
	//e=pos-refx;
	// PD control
	#if simple
	return (int)(KP*e-KD*w);
	#else
// 	PD control with Sigmoid function on Kd
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
 		
 		sigKD=(1-1/(1+exp(beta*(-abs(e)+gama))))*KD*w;
 		
 		itx=itx+e*T*KI;
 		
 		U=e*KP-sigKD+itx;
 		
 		//qu=(U2*nQ1-dQ2*qu0-dQ3*qu1-dQ4*qu2)/dQ1;
 		//
 		//qgu=(nQG1*w+nQG2*y1+nQG3*y2+nQG4*y3-dQG4*qgu2-dQG3*qgu1-dQG2*qgu0)/dQG1;
 		//
 		//
 		//
 		//qgu2=qgu1;
 		//qgu1=qgu0;
 		//qgu0=qgu;
 		//
 		//qu2=qu1;
 		//qu1=qu0;
 		//qu0=qu;
 		//
 		//y3=y2;
 		//y2=y1;
 		//y1=w;
 		//
 		//U2=U1;
 		//U1=U0;
 		//U0=U;
 		//
 		//ud=qgu-qu;
 		return (int)(U);//-ud);
 		#endif
}

int execY(float pos,float w)
{
	float erro=0,e=0;
	e=refy-pos;
	// PD control
	#if simple
	return (int)(KP*(e)-KD*w);
	
	#else
	
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
	sigKD=(1-1/(1+exp(beta*(-abs(e)+gama))))*KD*w;
	
	//ity=ity+e*T*KI;
	
	U=e*KP-sigKD;//+ity;
	
	//compensador de disturbio
	//qu=(U2y*nQ1-dQ2*qu0y-dQ3*qu1y-dQ4*qu2y)/dQ1;
//
	//qgu=(nQG1*w+nQG2*y1y+nQG3*y2y+nQG4*y3y-dQG4*qgu2y-dQG3*qgu1y-dQG2*qgu0y)/dQG1;
//
//
	//qgu2y=qgu1y;
	//qgu1y=qgu0y;
	//qgu0y=qgu;
//
	//qu2y=qu1y;
	//qu1y=qu0y;
	//qu0y=qu;
//
	//y3y=y2y;
	//y2y=y1y;
	//y1y=w;
//
	//U2y=U1y;
	//U1y=U0y;
	//U0y=U;
	//ud=qgu-qu;
	return (int)(U);//-ud);
	#endif
}

#endif //__CONTROL_H__
