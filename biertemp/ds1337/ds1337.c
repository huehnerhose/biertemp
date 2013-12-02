/*
 * ds1337.c
 *
 * Created: 02.12.2013 23:41:16
 *  Author: huehnerhose
 */ 
#include <inttypes.h>
#include "ds1337.h"

uint8_t getHours(){
	uint8_t ret;
	i2c_start_wait(DS1337+I2C_WRITE);
	i2c_write(HOURS);
	i2c_rep_start(DS1337+I2C_READ);
	ret  = i2c_readAck();
	ret = ret & (0b0011111);
	return BCD2BIN(ret);
}

uint8_t getMinutes(){
	uint8_t ret;
	i2c_start_wait(DS1337+I2C_WRITE);
	i2c_write(MINUTES);
	i2c_rep_start(DS1337+I2C_READ);
	ret = i2c_readAck();
	return BCD2BIN(ret);
}

uint8_t getMinutesSum(){
	return getMinutes() + 60*getHours();
}

uint8_t getDate(){
	i2c_start_wait(DS1337+I2C_WRITE);
	i2c_write(DATE);
	i2c_rep_start(DS1337+I2C_READ);
	return i2c_readAck();
}

void setDate(uint8_t date){
	i2c_rep_start(DS1337+I2C_WRITE);
	i2c_write(DATE);
	i2c_write(date);
	i2c_stop();
}

static uint8_t getHourRaw(){
	i2c_start_wait(DS1337+I2C_WRITE);
	i2c_write(MINUTES);
	i2c_rep_start(DS1337+I2C_READ);
	return i2c_readAck();
}
//TODO
void setHour(uint8_t hour){
	i2c_rep_start(DS1337+I2C_WRITE);
	i2c_write(HOURS);
	i2c_write( hour );
	i2c_stop();
}

void setAlarmMinutes(uint8_t min){
	i2c_rep_start(DS1337+I2C_WRITE);
}