/*
 * ds1337.c
 *
 * Created: 02.12.2013 23:41:16
 *  Author: huehnerhose
 */ 
#include <inttypes.h>
#include "ds1337.h"
#include "i2cmaster.h"

void ds1337_init(){
	i2c_init();
	//PD3 für INT1 für INTA von DS1337
	DDRD &= (1<<DDD3);
	PORTD |= (1<<PD3);
	GICR |= (1<<INT1);
}

uint8_t ds1337_getHours(){
	uint8_t ret;
	i2c_start_wait(DS1337+I2C_WRITE);
	i2c_write(DS1337_HOURS);
	i2c_rep_start(DS1337+I2C_READ);
	ret  = i2c_readAck();
	ret = ret & (0b0011111);
	return DS1337_BCD2BIN(ret);
}

uint8_t ds1337_getMinutes(){
	uint8_t ret;
	i2c_start_wait(DS1337+I2C_WRITE);
	i2c_write(DS1337_MINUTES);
	i2c_rep_start(DS1337+I2C_READ);
	ret = i2c_readAck();
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

void ds1337_setDate(uint8_t date){
	i2c_rep_start(DS1337+I2C_WRITE);
	i2c_write(DS1337_DATE);
	i2c_write(date);
	i2c_stop();
}

static uint8_t getHourRaw(){
	i2c_start_wait(DS1337+I2C_WRITE);
	i2c_write(DS1337_MINUTES);
	i2c_rep_start(DS1337+I2C_READ);
	return i2c_readAck();
}
//TODO
void ds1337_setHour(uint8_t hour){
	i2c_rep_start(DS1337+I2C_WRITE);
	i2c_write(DS1337_HOURS);
	i2c_write( hour );
	i2c_stop();
}

void ds1337_setAlarmMinutes(uint8_t min){
	i2c_rep_start(DS1337+I2C_WRITE);
}