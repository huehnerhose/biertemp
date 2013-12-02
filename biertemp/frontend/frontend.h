/*
 * frontend.h
 *
 * Created: 02.12.2013 19:10:38
 *  Author: huehnerhose
 */ 


#ifndef FRONTEND_H_
#define FRONTEND_H_

#define MAIN			0

#define MENU_AIM		1
#define MENU_TEMP		2
#define MENU_MAIN		3

#define TEMP_AIM_MIN	10
#define TEMP_AIM_MAX	11
#define TEMP_CONFIRM	12
#define TEMP_CANCEL		13

#define TEMP_DETAILS	200


#define UP				0
#define BOTTLE			1
#define STALL			2
#define CELCIUS			3
#define AVERAGE			4


#include <avr/io.h>
#include <stdio.h>

typedef struct Flag{
	unsigned char heaterOn : 1;
	unsigned char change : 1;
	unsigned char measStarted : 1;
	unsigned char int0 : 1;
};

extern void frontend_init(uint8_t nSensors);

extern void frontend_main(uint8_t **wheel_target, uint8_t *next_state, uint16_t measMiddle, uint8_t rangeMin, uint8_t rangeMax, struct Flag Flags, uint8_t timerCounter, uint8_t timerTarget);

extern void frontend_menu_aim(uint8_t **wheel_target, uint8_t *next_state, uint8_t *state, uint8_t *wheel_min, uint8_t *wheel_max);

extern void frontend_menu_temp(uint8_t **wheel_target, uint8_t *next_state, uint8_t *state, uint8_t *wheel_min, uint8_t *wheel_max);

extern void frontend_menu_main(uint8_t **wheel_target, uint8_t *next_state, uint8_t *state, uint8_t *wheel_min, uint8_t *wheel_max);

extern void frontend_tempdetails(uint8_t **wheel_target, uint8_t *next_state, int16_t *measVal, uint16_t measMiddle, uint8_t nSensors);



extern void frontend_else(uint8_t **wheel_target, uint8_t *next_state, uint8_t *state, uint8_t *wheel_min, uint8_t *wheel_max);

#endif /* FRONTEND_H_ */