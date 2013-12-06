/*
 * ds1337.c
 *
 * Created: 02.12.2013 23:41:16
 *  Author: huehnerhose
 */ 
#include <inttypes.h>
#include "ds1337.h"
#include "i2cmaster.h"


static uint8_t get(uint8_t addr){
	i2c_start_wait(DS1337+I2C_WRITE);
	i2c_write(addr);
	i2c_rep_start(DS1337+I2C_READ);
	addr = i2c_readAck();
	return addr;
}

static void set(uint8_t val, uint8_t addr){
	i2c_rep_start(DS1337+I2C_WRITE);
	i2c_write(addr);
	i2c_write( val );
	i2c_stop();
}

static uint8_t getHourRaw(){
	i2c_start_wait(DS1337+I2C_WRITE);
	i2c_write(DS1337_MINUTES);
	i2c_rep_start(DS1337+I2C_READ);
	return i2c_readAck();
}

static void resetTime(){
	set(0, DS1337_HOURS); //automaticly set HOUR_MODE to 24h
	set(0, DS1337_MINUTES);
	set(0, DS1337_SECONDS);
}

void ds1337_init(){
	i2c_init();
	//PD3 für INT1 für INTA von DS1337
	DDRD &= (1<<DDD3);
	PORTD |= (1<<PD3);
	GICR |= (1<<INT1);
	resetTime();
/*	
	uint8_t t;
	t = get(DS1337_CTRL);
	t &= ~((1<<DS1337_RS2) | (1<<DS1337_INTCN));
	t |= (1<<DS1337_RS1);
	set(t, DS1337_CTRL);
*/
}

uint8_t ds1337_getHours(){
	uint8_t ret;
	ret = get(DS1337_HOURS);
	ret = ret & (0b0011111);
	return DS1337_BCD2BIN(ret);
}

uint8_t ds1337_getMinutes(){
	uint8_t ret;
	ret = get(DS1337_MINUTES);
	return DS1337_BCD2BIN(ret);
}

uint8_t ds1337_getMinutesSum(){
	return ds1337_getMinutes() + 60*ds1337_getHours();
}

uint8_t ds1337_getDate(){
	i2c_start_wait(DS1337+I2C_WRITE);
	i2c_write(DS1337_DATE);
	i2c_rep_start(DS1337+I2C_READ);
	return i2c_readAck();
}

void ds1337_setAlarm1(){
	ds1337_resetAlarm1();
	set( (1<<7), DS1337_ALARM1_DAY ); //Set Alarm1 Mode
	set( (1<<DS1337_A1IE), DS1337_CTRL ); // enable Interrupt
}

void ds1337_resetAlarm1(){
	uint8_t t;
	t = get(DS1337_CTRL);
	t &= ~( (1<<DS1337_A1IE) );
	set(t, DS1337_CTRL);
	
	t = get(DS1337_STAT);
	t &= ~( (1<<DS1337_A1F) );
	set(t, DS1337_STAT);
}

uint8_t ds1337_debug(){
	uint8_t t = get(DS1337_CTRL);
	return t;
}

void ds1337_setAlarmMinutes( uint8_t min ){
	uint8_t hour = (uint8_t)min/60;
	min = min - (hour*60);
	set(DS1337_BIN2BCD(min), DS1337_ALARM1_MIN);
	set(hour, DS1337_ALARM1_HOUR);
	ds1337_setAlarm1();
	resetTime();
}
