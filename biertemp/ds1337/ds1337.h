/*
 * ds1337.h
 *
 * Created: 02.12.2013 23:41:24
 *  Author: huehnerhose
 */ 


#ifndef DS1337_H_
#define DS1337_H_

/************************************************************************/
/* Defines DS1337                                                       */
/************************************************************************/
#define DS1337 0xD0

#define SECONDS		0x00
#define MINUTES		0x01
#define HOURS		0x02
#define DAY			0x03
#define DATE		0x04
#define MONTH		0x05
#define YEAR		0x06

#define ALARM1_SEC	0x07
#define ALARM1_MIN	0x08
#define ALARM1_H	0x09
#define ALARM1_D	0x0A

#define ALARM2_MIN	0x0B
#define ALARM2_H	0x0C
#define ALARM2_D	0x0D

#define CTRL		0x0E
#define STAT		0x0F

//Control Register
#define A1IE		0
#define A2IE		1
#define INTCN		2
#define RS1			3
#define RS2			4
#define EOSC		7

//Status Register
#define A1F			0
#define A2F			1
#define OSF			7

//ALARM Register
#define DYDT		6

//HourMode
#define HOUR_MODE	6 //HIGH = 12H Mode, LOW = 24H Mode


#define BCD2BIN(val) (((val)&15) + ((val)>>4)*10)
#include "i2cmaster.h"

extern uint8_t getHours();
extern uint8_t getMinutes();
extern uint8_t getMinutesSum();
extern uint8_t getDate();
extern void setDate(uint8_t date);
extern void setAlarmMinutes(uint8_t min);
extern void setHour(uint8_t hour);

#endif /* DS1337_H_ */