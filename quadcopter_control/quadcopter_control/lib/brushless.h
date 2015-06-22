/*
 * brushless.h
 *
 * Created: 3/6/2015 0:30:35
 *  Author: Guilherme
 */ 


#ifndef BRUSHLESS_H_
#define BRUSHLESS_H_

void (*_tim0compA)();
void (*_tim0compB)();
void (*_tim2compA)();
void (*_tim2compB)();

void timer0CTCConfig()
{
	cli();
	PRR    &= ~(1<<PRTIM0);
	TCCR0A |=  (1<<WGM01);	// Clear Time Compare (CTC) mode
	TCCR0A |=  (1<<COM0A0);	// Toggle OC0A on Compare Match
	TCCR0A |=  (1<<COM0B0);	// Toggle OC0B on Compare Match
	TCCR0B |=  (1<<CS00);	// clk/64
	TCCR0B |=  (1<<CS01);	// clk/64
	TIMSK0 |= (1<<OCIE0A);  //enable output compare A interrupt
	TIMSK0 |= (1<<OCIE0B);  //enable output compare B interrupt
	sei();
	
}
void timer2CTCConfig()
{
	cli();
	PRR    &= ~(1<<PRTIM0);
	TCCR2A |=  (1<<WGM21);	// Clear Time Compare (CTC) mode
	TCCR2A |=  (1<<COM2A0);	// Toggle OC0A on Compare Match
	TCCR2A |=  (1<<COM2B0);	// Toggle OC0B on Compare Match
	TCCR2B |=  (1<<CS22);	// clk/64
	TIMSK2 |= (1<<OCIE2A);  //enable output compare A interrupt
	TIMSK2 |= (1<<OCIE2B);  //enable output compare B interrupt
	sei();
}

ISR(TIM2_COMPA)
{
	(*_tim2compA)();
}
ISR(TIM2_COMPB)
{
	(*_tim2compB)();
}
ISR(TIM0_COMPA)
{
	(*_tim0compA)();
}
ISR(TIM0_COMPB)
{
	(*_tim0compB)();
}
#endif /* BRUSHLESS_H_ */