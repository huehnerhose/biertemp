/*
 * rotary.c
 *
 * Created: 04.12.2013 11:03:34
 *  Author: huehnerhose
 */ 

#include <inttypes.h>
#include <avr/interrupt.h>
#include "rotary.h"

volatile int8_t enc_delta;          // -128 ... 127
int8_t last;

ISR( TIMER0_OVF_vect )             // 1ms for manual movement
{
	int8_t new, diff;
	new = 0;
	if( PHASE_A )
	new = 3;
	if( PHASE_B )
	new ^= 1;                   // convert gray to binary
	diff = last - new;                // difference last - new
	if( diff & 1 ){               // bit 0 = value (1)
		last = new;                 // store new as next last
		enc_delta += (diff & 2) - 1;        // bit 1 = direction (+/-)
	}
}

void rotary_encode_init( void )
{
	//Timer0, mit Interrupt ca. jede 1000/s
	TCCR0 |= ((1<<CS01));
	TIMSK |= (1<<TOIE0);
	
	DDRC &= ~((1<<PC2) | (1<<PC3));
	
	int8_t new;
	
	new = 0;
	if( PHASE_A )
	new = 3;
	if( PHASE_B )
	new ^= 1;                   // convert gray to binary
	last = new;                   // power on state
	enc_delta = 0;
}

int8_t rotary_encode_read4( void )         // read four step encoders
{
	int8_t val;
	
	cli();
	val = enc_delta;
	enc_delta = val & 3;
	sei();
	return val >> 2;
}