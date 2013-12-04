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

#define DS1337_SECONDS		0x00
#define DS1337_MINUTES		0x01
#define DS1337_HOURS		0x02
#define DS1337_DAY			0x03
#define DS1337_DATE			0x04
#define DS1337_MONTH		0x05
#define DS1337_YEAR			0x06

#define DS1337_ALARM1_SEC	0x07
#define DS1337_ALARM1_MIN	0x08
#define DS1337_ALARM1_HOUR	0x09
#define DS1337_ALARM1_DAY	0x0A

#define DS1337_ALARM2_MIN	0x0B
#define DS1337_ALARM2_HOUR	0x0C
#define DS1337_ALARM2_DAY	0x0D

#define DS1337_CTRL			0x0E
#define DS1337_STAT			0x0F

//Control Register
#define DS1337_A1IE			0
#define DS1337_A2IE			1
#define DS1337_INTCN		2
#define DS1337_RS1			3
#define DS1337_RS2			4
#define DS1337_EOSC			7

//Status Register
#define DS1337_A1F			0
#define DS1337_A2F			1
#define DS1337_OSF			7

//ALARM Register
#define DS1337_DYDT			6

//HourMode
#define DS1337_HOUR_MODE	6 //HIGH = 12H Mode, LOW = 24H Mode


#define DS1337_BCD2BIN(val) (((val)&15) + ((val)>>4)*10)
#define DS1337_BIN2BCD(val) (((val)/10)<<4) + (val)%10 
#include "i2cmaster.h"

extern uint8_t ds1337_getHours();
extern uint8_t ds1337_getMinutes();
extern uint8_t ds1337_getMinutesSum();
extern uint8_t ds1337_getDate();
//extern void ds1337_setDate(uint8_t date);
extern void ds1337_setAlarmMinutes(uint8_t min);
//extern void ds1337_setHour(uint8_t hour);
extern void ds1337_init();
//extern void set(uint8_t val, uint8_t addr);
extern void ds1337_setAlarm1();
extern void ds1337_resetAlarm1();
extern uint8_t ds1337_debug();
#endif /* DS1337_H_ */