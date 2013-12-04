/*
 * biertemp.c
 *
 * Created: 25.11.2013 15:41:51
 *  Author: huehnerhose
 */ 


#include <avr/io.h>

#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#define MAXSENSORS 3
#define NEWLINESTR ""
#define uart_puts_P
#include "onewire.h"
#include "ds18x20.h"
#include "ds1337.h"

#include "lcd.h"

#include "frontend.h"

#include "debounce.h"

uint8_t state, next_state;

struct Flag Flags;

//DS18X20
uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];

/************************************************************************/
/* Subroutines rotary                                                   */
/************************************************************************/
#define PHASE_A     (PINC & 1<<PC3)
#define PHASE_B     (PINC & 1<<PC2)
//importierter Drehimpulscode
volatile int8_t enc_delta;          // -128 ... 127
static int8_t last;
void encode_init( void )
{
	int8_t new;
	
	new = 0;
	if( PHASE_A )
		new = 3;
	if( PHASE_B )
		new ^= 1;                   // convert gray to binary
	last = new;                   // power on state
	enc_delta = 0;
}

int8_t encode_read4( void )         // read four step encoders
{
	int8_t val;
	
	cli();
	val = enc_delta;
	enc_delta = val & 3;
	sei();
	return val >> 2;
}
/************************************************************************/
/* Subroutines DS18X20                                                  */
/************************************************************************/
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

ISR( TIMER0_OVF_vect )             // 1ms for manual movement
{
	int8_t new, diff;
	new = 0;
	if( PHASE_A )
	new = 3;
	if( PHASE_B )
	new ^= 1;                   // convert gray to binary
	diff = last - new;                // difference last - new
	if( diff & 1 ){               // bit 0 = value (1)
		last = new;                 // store new as next last
		enc_delta += (diff & 2) - 1;        // bit 1 = direction (+/-)
	}
}

ISR(TIMER1_OVF_vect){				// Temperaturmessung anwerfen
	DS18X20_start_meas(DS18X20_POWER_EXTERN, NULL);
	Flags.measStarted = 1;
}

/*
ISR( TIMER2_OVF_vect )                            // every 10ms
{
	//lcd_clrscr();
	//lcd_puts("Timer2");
	static uint8_t ct0, ct1;//, rpt;
	uint8_t i;
	
	TCNT2 = 255-117;
	
	i = key_state ^ ~KEY_PIN;                       // key changed ?
	ct0 = ~( ct0 & i );                             // reset or count ct0
	ct1 = ct0 ^ (ct1 & i);                          // reset or count ct1
	i &= ct0 & ct1;                                 // count until roll over ?
	key_state ^= i;                                 // then toggle debounced state
	key_press |= key_state & i;                     // 0->1: key press detect
	
	//if( (key_state & REPEAT_MASK) == 0 )            // check repeat function
	//rpt = REPEAT_START;                          // start delay
	//if( --rpt == 0 ){
		//rpt = REPEAT_NEXT;                            // repeat delay
		//key_rpt |= key_state & REPEAT_MASK;
	//}
}
*/

ISR( INT1_vect ){
	Flags.change = 1;
	Flags.alarm = 1;
	state = ALARM;	
}

static uint16_t calcMeasMiddle(int16_t meas[], uint8_t nSensors){
	int16_t middle = 0;
	for(uint8_t i = 0; i<nSensors; i++){
		DS18X20_read_decicelsius(&gSensorIDs[i][0], &meas[i]);
		middle += meas[i];
	}
	middle = middle / nSensors;
	return middle;
}

int main(void)
{
	uint8_t rangeMax, rangeMin;
	uint8_t timerTarget, timerCounter;
	
	//Hier sollten standardwerte für das einmaischen hin
	rangeMax = 65;
	rangeMin = 26;
	Flags.heaterOn = 0;
	timerCounter = 23;
	timerTarget = 120;
	
	//init ds18x20
	uint8_t nSensors;
	nSensors = search_sensors();
	int16_t measVal[nSensors];
	int16_t measMiddle = 0;
	
	frontend_init(nSensors);
	i2c_init();
	
	//temperaturmessung in Interrupt starten
	TCCR1B |= ((1<<CS12));
	TIMSK |= (1<<TOIE1);


	//Timer0, mit Interrupt ca. jede 1000/s
	TCCR0 |= ((1<<CS01));
	TIMSK |= (1<<TOIE0);
	int offset = 0;
	encode_init();
	
	//PD2 auf Eingang für Pseudo-Interrupt vom Button
	DDRD &= ~(1 << DDD2);
	PORTD |= (1<<PD2); //interner PullUp
	//GICR |= (1<<INT0); //Interrupt für INT0 aktivieren
	TCCR2 |= ( (1<<CS22) || (1<<CS21) || (1<<CS20) );
	TCNT2 = 255-117;	//Hier wird der Timer für den Pseudo-Interrupt so eingestellt, dass alle 10ms gefeuert wird
	TIMSK |= (1<<TOIE2);
	
	//PD3 für INT1 für INTA von DS1337
	DDRD &= (1<<DDD3);
	PORTD |= (1<<PD3);
	GICR |= (1<<INT1);
	
	state = MAIN;
	uint8_t *wheel_target = NULL; //Wird auf die Variable gelenkt, die verändert werden soll
	uint8_t wheel_max = 0;
	uint8_t wheel_min = 0;

	wdt_enable(WDTO_1S); // enable 1s watchdog timer
	sei();
	while(1)
    {
		wdt_reset();
		
		if( get_key_short( 1<<KEY0 )){
			Flags.change = 1;
			state = next_state;
		}
		
		offset = encode_read4();
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
				frontend_else(&wheel_target, &next_state, &state, &wheel_min, &wheel_max, getMinutes(), getHours(), getMinutesSum());
			}
		}
			
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