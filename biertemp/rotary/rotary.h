/*
 * rotary.h
 *
 * Created: 04.12.2013 11:03:22
 *  Author: huehnerhose
 */ 


#ifndef ROTARY_H_
#define ROTARY_H_

#define ROTARY_DDR DDRD
#define ROTARY_PIN PIND
#define ROTARY_A PD0
#define ROTARY_B PD1

#define PHASE_A     (ROTARY_PIN & 1<<ROTARY_A)
#define PHASE_B     (ROTARY_PIN & 1<<ROTARY_B)
//importierter Drehimpulscode
extern volatile int8_t enc_delta;          // -128 ... 127
extern int8_t last;
int8_t rotary_encode_read4( void ) /* read four step encoders */;
void rotary_encode_init( void );
#endif /* ROTARY_H_ */