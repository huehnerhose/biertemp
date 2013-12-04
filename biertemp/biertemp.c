/*
 * biertemp.c
 *
 * Created: 25.11.2013 15:41:51
 *  Author: huehnerhose
 */ 

#define MAXSENSORS 3
#define NEWLINESTR ""
#define uart_puts_P

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "onewire.h"
#include "ds18x20.h"
#include "ds1337.h"
#include "lcd.h"
#include "frontend.h"
#include "debounce.h"
#include "rotary.h"

/************************************************************************/
/* Globals                                                              */
/************************************************************************/
uint8_t state, next_state;
struct Flag Flags;
uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];

/************************************************************************/
/* Interrupts                                                           */
/************************************************************************/

//////////////////////////////////////////////////////////////////////////
// Interrupt for temperature mesearument
//////////////////////////////////////////////////////////////////////////
ISR(TIMER1_OVF_vect){				// Temperaturmessung anwerfen
	DS18X20_start_meas(DS18X20_POWER_EXTERN, NULL);
	Flags.measStarted = 1;
}

//////////////////////////////////////////////////////////////////////////
// Interrupt for DS1337 alarminterrupt
//////////////////////////////////////////////////////////////////////////
ISR( INT1_vect ){
	Flags.change = 1;
	Flags.alarm = 1;
	state = ALARM;	
}

/************************************************************************/
/* Subroutine                                                           */
/************************************************************************/

//////////////////////////////////////////////////////////////////////////
// Search for DS18x20 temperaturesensors
//////////////////////////////////////////////////////////////////////////
static uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	ow_reset();

	nSensors = 0;
	
	diff = OW_SEARCH_FIRST;
	while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) {
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == OW_PRESENCE_ERR ) {
			//uart_puts_P( "No Sensor found" NEWLINESTR );
			break;
		}
		
		if( diff == OW_DATA_ERR ) {
			//uart_puts_P( "Bus Error" NEWLINESTR );
			break;
		}
		
		for ( i=0; i < OW_ROMCODE_SIZE; i++ )
		gSensorIDs[nSensors][i] = id[i];
		
		nSensors++;
	}
	
	return nSensors;
}

//////////////////////////////////////////////////////////////////////////
// Calculate average temperature over all avaible sensors
//////////////////////////////////////////////////////////////////////////
static uint16_t calcMeasMiddle(int16_t meas[], uint8_t nSensors){
	int16_t middle = 0;
	for(uint8_t i = 0; i<nSensors; i++){
		DS18X20_read_decicelsius(&gSensorIDs[i][0], &meas[i]);
		middle += meas[i];
	}
	middle = middle / nSensors;
	return middle;
}

/************************************************************************/
/* MAIN                                                                 */
/************************************************************************/
int main(void)
{
	
	//temperature range for heater target
	uint8_t rangeMax, rangeMin;
		
	//timer variables
	////sind hier überhaupt beide notwendig? Reicht nicht nur target?
	uint8_t timerTarget, timerCounter;
		
	//initialize target / range values
	rangeMax = 65;
	rangeMin = 26;
	Flags.heaterOn = 0;
	timerCounter = 23;
	timerTarget = 120;
	
	//////////////////////////////////////////////////////////////////////////
	// Initialize ds18x20 / temperature measurement
	//////////////////////////////////////////////////////////////////////////
	uint8_t nSensors;
	nSensors = search_sensors();
	int16_t measVal[nSensors];
	int16_t measMiddle = 0;
	//set timer for interrupt driven measurement
	TCCR1B |= ((1<<CS12));
	TIMSK |= (1<<TOIE1);
	
	//////////////////////////////////////////////////////////////////////////
	// Initialize peripherials
	//////////////////////////////////////////////////////////////////////////
	frontend_init(nSensors); //Frontend / LCD
	ds1337_init(); //ds1337
	rotary_encode_init(); //Rotary Encoder Initialization
	debounce_init(); //Debouncing for push button
	
	//variables for user interaction / user interface
	state = MAIN;
	uint8_t *wheel_target = NULL; //dynamicly mapped to value edited by rotary wheel
	uint8_t wheel_max = 0;
	uint8_t wheel_min = 0;
	int offset = 0;

	wdt_enable(WDTO_1S); // enable 1s watchdog timer
	sei();

	/************************************************************************/
	/* MAIN LOOP                                                            */
	/************************************************************************/
	while(1)
    {
		wdt_reset();
		
		if( debounce_get_key_short( 1<<DEBOUNCE_KEY0 )){
			Flags.change = 1;
			state = next_state;
		}
		
		//Get rotaryencoder input and check/generate user interface adjustment
		offset = rotary_encode_read4();
		if(offset != 0){
			Flags.change = 1;

			if(wheel_target != NULL){	
				if(*wheel_target == wheel_min && offset == -1){
					*wheel_target = wheel_max;
				}else if(*wheel_target == wheel_max && offset == 1){
					*wheel_target = wheel_min;
				}else{
					*wheel_target += offset;
				}
			}
		}


		//Draw Userinterface
		if(Flags.change == 1){
			Flags.change = 0;	
			if(state == MAIN){
				frontend_main(&wheel_target, &next_state, measMiddle, rangeMin, rangeMax, Flags, timerCounter, timerTarget);
			}else if(state == MENU_AIM){	
				frontend_menu_aim(&wheel_target, &next_state, &state, &wheel_min, &wheel_max);
				//wheel_target = &state;
			}else if(state == MENU_TEMP){
				frontend_menu_temp(&wheel_target, &next_state, &state, &wheel_min, &wheel_max);
			}else if(state == MENU_MAIN){
					frontend_menu_main(&wheel_target, &next_state, &state, &wheel_min, &wheel_max);
			}else if(state == TEMP_DETAILS){
				frontend_tempdetails(&wheel_target, &next_state, measVal, measMiddle, nSensors);
			}else if (state == ALARM){
				frontend_alarm(&wheel_target, &next_state);
			}else{
				frontend_else(&wheel_target, &next_state, &state, &wheel_min, &wheel_max, ds1337_getMinutes(), ds1337_getHours(), ds1337_getMinutesSum());
			}
		}
		
		//Check whether measurement is ready and update all measurement holding variables
		if(Flags.measStarted == 1){
			if(DS18X20_conversion_in_progress() == 0){
				Flags.measStarted = 0;
				measMiddle = calcMeasMiddle(measVal, nSensors);				
				if(state == MAIN || state == TEMP_DETAILS){
					Flags.change = 1;	
				}
				if(measMiddle < (rangeMin*10)){
					Flags.heaterOn = 1;
				}else if (measMiddle > (rangeMin*10))
				{
					Flags.heaterOn = 0;
				}
			}
		}
    }
}