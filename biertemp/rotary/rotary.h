/*
 * rotary.h
 *
 * Created: 04.12.2013 11:03:22
 *  Author: huehnerhose
 */ 


#ifndef ROTARY_H_
#define ROTARY_H_


#define PHASE_A     (PINC & 1<<PC3)
#define PHASE_B     (PINC & 1<<PC2)
//importierter Drehimpulscode
extern volatile int8_t enc_delta;          // -128 ... 127
extern int8_t last;
int8_t rotary_encode_read4( void ) /* read four step encoders */;
void rotary_encode_init( void );
#endif /* ROTARY_H_ */