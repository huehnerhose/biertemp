/*
 * frontend.c
 *
 * Created: 02.12.2013 19:10:51
 *  Author: huehnerhose
 */ 

#include <inttypes.h>
#include <avr/pgmspace.h>
#include "frontend.h"
#include "lcd.h"
#include "ds18x20.h"

/************************************************************************/
/* Add Arrows for scrollable views                                      */
/************************************************************************/

static void frontend_addArrows(){
	lcd_gotoxy(0,1);
	lcd_putc(LEFT);
	lcd_gotoxy(15,1);
	lcd_putc(RIGHT);
}

/************************************************************************/
/* Initialize Frontend / LCD / and stuff                                */
/************************************************************************/
void frontend_init(uint8_t nSensors){
	unsigned char i;
	char buffer[16];
	
	//Program Specialchars
	//0 - Arrow Up
	static const PROGMEM unsigned char up[] = {0x4,0xe,0x15,0x4,0x4,0x4,0x4,0x4};
	//1 - Beerbottle
	static const PROGMEM unsigned char bottle[] = {0x0,0x4,0x4,0xe,0xe,0xe,0xe,0xe};
	//2 - Stall -sign
	static const PROGMEM unsigned char stall[] = {0x0,0xa,0x11,0x15,0x11,0xa,0x0,0x0};
	//3 - °C
	static const PROGMEM unsigned char de_ce[] = {0x1c,0x14,0x1c,0x3,0x4,0x4,0x3,0x0};
	//4 - Mittelwert
	static const PROGMEM unsigned char middle[] = {0x0,0x1,0xe,0x13,0x15,0x19,0xe,0x10};
	//5 - LEFT
	static const PROGMEM unsigned char left[] = {0x0,0x2,0x6,0xe,0x6,0x2,0x0,0x0};
	//6 - RIGHT
	static const PROGMEM unsigned char right[] = {0x0,0x8,0xc,0xe,0xc,0x8,0x0,0x0};

	lcd_init(LCD_DISP_ON);

	
	
	//Schreibe CustomChar in Display
	lcd_command(_BV(LCD_CGRAM));
	for(i=0;i<8;i++){
		lcd_data((pgm_read_byte_near(&up[i])));
	}
	for(i=0;i<8;i++){
		lcd_data((pgm_read_byte_near(&bottle[i])));
	}
	for(i=0;i<8;i++){
		lcd_data((pgm_read_byte_near(&stall[i])));
	}
	for(i=0;i<8;i++){
		lcd_data((pgm_read_byte_near(&de_ce[i])));
	}
	for(i=0;i<8;i++){
		lcd_data((pgm_read_byte_near(&middle[i])));
	}
	for(i=0;i<8;i++){
		lcd_data((pgm_read_byte_near(&left[i])));
	}
	for(i=0;i<8;i++){
		lcd_data((pgm_read_byte_near(&right[i])));
	}
	
	//Initialisiere Willkommen
	lcd_gotoxy(1,0);
	lcd_puts("Here for beer");
	lcd_putc(1);
	lcd_gotoxy(0,1);
	
	sprintf(buffer, "%i Sensors found", nSensors);
	lcd_puts(buffer);

}

/************************************************************************/
/* Render Main View                                                     */
/************************************************************************/
void frontend_main(uint8_t **wheel_target, uint8_t *next_state, uint16_t measMiddle, uint8_t rangeMin, uint8_t rangeMax, struct Flag Flags, int8_t timerCounter, uint8_t timerTarget){
	//MAIN//
	char buffer[16];
	*wheel_target = NULL;
	*next_state = MENU_AIM;
	lcd_clrscr();
	lcd_gotoxy(0,0);
	lcd_putc(4);
	DS18X20_format_from_decicelsius(measMiddle, buffer, 10);
	lcd_puts(buffer);
	lcd_putc(3);
	lcd_gotoxy(9,0);
	sprintf(buffer, "R:%i-%i", rangeMin, rangeMax);
	lcd_puts(buffer);
	lcd_gotoxy(0,1);
	if(Flags.heaterOn == 1){
		lcd_putc(0); //Up
		}else{
		lcd_putc(2); //stall
	}
	
	#ifdef DEBUG_CLOCK
		#include "ds1337.h"
		uint8_t t;
		t = ds1337_getHours();
		sprintf(buffer, "%2i", t);
		lcd_puts(buffer);
		t = ds1337_getMinutes();
		sprintf(buffer, ":%2i", t);
		lcd_puts(buffer);
		t = ds1337_getSeconds();
		sprintf(buffer, ":%2i", t);
		lcd_puts(buffer);
	
	#else
	
		lcd_gotoxy(2, 1);
		sprintf(buffer, "t:%im", timerCounter);
		lcd_puts(buffer);
		lcd_gotoxy(9,1);
		sprintf(buffer, "T:%im", timerTarget);
		lcd_puts(buffer);
	
	#endif
}

/************************************************************************/
/* Render Menuview - Set Temp Aim                                       */
/************************************************************************/
void frontend_menu_aim(uint8_t **wheel_target, uint8_t *next_state, uint8_t *state, uint8_t *wheel_min, uint8_t *wheel_max){
	lcd_clrscr();
	*next_state = TEMP_AIM_MIN;
	*wheel_target = state;
	*wheel_min = MENU_AIM;
	*wheel_max = MENU_MAIN;
	lcd_gotoxy(4,0);
	lcd_puts("set next");
	lcd_gotoxy(4,1);
	lcd_puts("temp aim");
	frontend_addArrows();
}

/************************************************************************/
/* Render Menuview - Show Tempdetails                                   */
/************************************************************************/
void frontend_menu_temp(uint8_t **wheel_target, uint8_t *next_state, uint8_t *state, uint8_t *wheel_min, uint8_t *wheel_max){
	lcd_clrscr();
	*next_state = TEMP_DETAILS;
	*wheel_target = state;
	*wheel_min = MENU_AIM;
	*wheel_max = MENU_MAIN;
	lcd_gotoxy(4,0);
	lcd_puts("show temp");
	lcd_gotoxy(4,1);
	lcd_puts("details");
	frontend_addArrows();
}

/************************************************************************/
/* Render Menuview - Show Overview                                      */
/************************************************************************/
void frontend_menu_main(uint8_t **wheel_target, uint8_t *next_state, uint8_t *state, uint8_t *wheel_min, uint8_t *wheel_max){
	*next_state = MAIN;
	*wheel_target = state;
	*wheel_min = MENU_AIM;
	*wheel_max = MENU_MAIN;
	lcd_clrscr();
	lcd_gotoxy(4,0);
	lcd_puts("Overview");
	frontend_addArrows();
}

/************************************************************************/
/* Render Temperaturedetails                                            */
/************************************************************************/
void frontend_tempdetails(uint8_t **wheel_target, uint8_t *next_state, int16_t *measVal, uint16_t measMiddle, uint8_t nSensors){
	char buffer[8];
	*wheel_target = NULL;
	*next_state = MENU_MAIN;
	lcd_clrscr();
	for(uint8_t i = 0; i < nSensors; i++){
		if((i+1)%2 != 0){
			lcd_gotoxy(0,(i/2));
		}else{
			lcd_gotoxy(8, (i/2));
		}
		sprintf(buffer, "%i:", i);
		lcd_puts(buffer);
		DS18X20_format_from_decicelsius(measVal[i], buffer, 10);
		lcd_puts(buffer);
	}
	lcd_gotoxy(8,1);
	lcd_putc(AVERAGE);
	lcd_puts(":");
	DS18X20_format_from_decicelsius(measMiddle, buffer, 10);
	lcd_puts(buffer);
}

void frontend_tempAimMin(uint8_t **wheel_target, uint8_t *wheel_min, uint8_t *wheel_max, uint8_t *next_state, uint8_t *rangeMin){
	char buffer[3];
	*wheel_target = rangeMin;
	*next_state = TEMP_AIM_MAX;
	*wheel_min = 20;
	*wheel_max = 99;
	lcd_clrscr();
	lcd_gotoxy(4,0);
	lcd_puts("next temp");
	lcd_gotoxy(4,1);
	lcd_puts("min ");
	sprintf(buffer, "%2i", *rangeMin);
	lcd_puts(buffer);
	lcd_putc(CELCIUS);
	frontend_addArrows();
}

void frontend_tempAimMax(uint8_t **wheel_target, uint8_t *wheel_min, uint8_t *wheel_max, uint8_t *next_state, uint8_t *rangeMin, uint8_t *rangeMax){
	char buffer[3];
	*wheel_target = rangeMax;
	*next_state = TEMP_AIM_CONFIRM;
	*wheel_min = *rangeMin;
	*wheel_max = 99;
	lcd_clrscr();
	lcd_gotoxy(4,0);
	lcd_puts("next temp");
	lcd_gotoxy(4,1);
	lcd_puts("max ");
	sprintf(buffer, "%2i", *rangeMax);
	lcd_puts(buffer);
	lcd_putc(CELCIUS);
	frontend_addArrows();
}

void frontend_tempAimConfirm(uint8_t **wheel_target, uint8_t *wheel_min, uint8_t *wheel_max, uint8_t *state, uint8_t *next_state, uint8_t *rangeMin, uint8_t *rangeMax){
	char buffer[6];
	*wheel_target = state;
	*next_state = TIME_TARGET;
	*wheel_min = TEMP_AIM_CONFIRM;
	*wheel_max = TEMP_AIM_CANCEL;
	lcd_clrscr();
	lcd_gotoxy(2,0);
	lcd_puts("confirm range");
	lcd_gotoxy(5,1);
	sprintf(buffer, "%2i-%2i", *rangeMin, *rangeMax);
	lcd_puts(buffer);
	lcd_putc(CELCIUS);
	frontend_addArrows();
}

void frontend_tempAimCancel(uint8_t **wheel_target, uint8_t *wheel_min, uint8_t *wheel_max, uint8_t *state, uint8_t *next_state){
	*wheel_target = state;
	*next_state = TEMP_AIM_MIN;
	*wheel_min = TEMP_AIM_CONFIRM;
	*wheel_max = TEMP_AIM_CANCEL;
	lcd_clrscr();
	lcd_gotoxy(2,0);
	lcd_puts("reset range");
	frontend_addArrows();
}

void frontend_timeTarget(uint8_t **wheel_target, uint8_t *wheel_min, uint8_t *wheel_max, uint8_t *next_state, uint8_t *timerTarget){
	*wheel_target = timerTarget;
	*next_state = TIME_CONFIRM;
	*wheel_min = 0;
	*wheel_max = 120;
	lcd_clrscr();
	lcd_gotoxy(4,0);
	lcd_puts("Set Timer");
	lcd_gotoxy(5,1);
	char buffer[7];
	sprintf(buffer, "%3imin", *timerTarget);
	lcd_puts(buffer);
	frontend_addArrows();
}

void frontend_timerConfirm(uint8_t **wheel_target, uint8_t *wheel_min, uint8_t *wheel_max, uint8_t *next_state, uint8_t *state, uint8_t *timerTarget, struct Flag *flag){
	*wheel_target = state;
	*next_state = MAIN;
	*wheel_min = TIME_CONFIRM;
	*wheel_max = TIME_CANCEL;
	lcd_clrscr();
	lcd_gotoxy(2,0);
	lcd_puts("confirm timer");
	lcd_gotoxy(5,1);
	char buffer[7];
	sprintf(buffer, "%3imin", *timerTarget);
	lcd_puts(buffer);
	frontend_addArrows();
	flag->setAlarm = 1;
}

void frontend_timerCancel(uint8_t **wheel_target, uint8_t *wheel_min, uint8_t *wheel_max, uint8_t *next_state, uint8_t *state, struct Flag *flag){
	*wheel_target = state;
	*next_state = TIME_TARGET;
	*wheel_min = TIME_CONFIRM;
	*wheel_max = TIME_CANCEL;
	lcd_clrscr();
	lcd_gotoxy(3,0);
	lcd_puts("reset timer");
	frontend_addArrows();
	flag->setAlarm = 0; //Same as (*flag).setAlarm
}

/************************************************************************/
/* Render Alarm-Triggered                                               */
/************************************************************************/
void frontend_alarm(uint8_t **wheel_target, uint8_t *next_state){
	*next_state = MAIN;
	*wheel_target = NULL;
	lcd_clrscr();
	lcd_gotoxy(5,0);
	lcd_puts("Timer");
	lcd_gotoxy(4,1);
	lcd_puts("reached");
}

/************************************************************************/
/* Render When nothing else is triggered                                */
/************************************************************************/
void frontend_else(uint8_t **wheel_target, uint8_t *next_state, uint8_t *state, uint8_t *wheel_min, uint8_t *wheel_max, uint8_t minutes, uint8_t hours, uint8_t minutesSum){
	char buffer[16];
	lcd_clrscr();
	lcd_puts("else? ");
	sprintf(buffer, "%i %i %i %i", *state, *wheel_min, *wheel_max, minutesSum);
	lcd_puts(buffer);
	lcd_gotoxy(0,1);
	sprintf(buffer, "%i:%i", hours, minutes);
	lcd_puts(buffer);
	*next_state = MAIN;
}

