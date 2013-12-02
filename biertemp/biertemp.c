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

#include "lcd.h"

#include "frontend.h"

uint8_t state, next_state;

struct Flag Flags;

//struct {
	//unsigned char heaterOn : 1;
	//unsigned char change : 1;
	//unsigned char measStarted : 1;
	//unsigned char int0 : 1;
//} Flags;

//DS18X20
uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];

/************************************************************************/
/* Entprellen                                                           */
/************************************************************************/
#define KEY_DDR         DDRD
#define KEY_PORT        PORTD
#define KEY_PIN         PIND
#define KEY0            2
//#define KEY1            4
//#define KEY2            4
#define ALL_KEYS        (1<<KEY0)

//#define REPEAT_MASK     (1<<KEY1 | 1<<KEY2)       // repeat: key1, key2
//#define REPEAT_START    50                        // after 500ms
//#define REPEAT_NEXT     20                        // every 200ms
//
volatile uint8_t key_state;                                // debounced and inverted key state:
// bit = 1: key pressed
volatile uint8_t key_press;                                // key press detect

volatile uint8_t key_rpt;                                  // key long press and repeat

///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed. Each pressed key is reported
// only once
//
uint8_t get_key_press( uint8_t key_mask )
{
	cli();                                          // read and clear atomic !
	key_mask &= key_press;                          // read key(s)
	key_press ^= key_mask;                          // clear key(s)
	sei();
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed long enough such that the
// key repeat functionality kicks in. After a small setup delay
// the key is reported being pressed in subsequent calls
// to this function. This simulates the user repeatedly
// pressing and releasing the key.
//
uint8_t get_key_rpt( uint8_t key_mask )
{
	cli();                                          // read and clear atomic !
	key_mask &= key_rpt;                            // read key(s)
	key_rpt ^= key_mask;                            // clear key(s)
	sei();
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key is pressed right now
//
uint8_t get_key_state( uint8_t key_mask )

{
	key_mask &= key_state;
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
uint8_t get_key_short( uint8_t key_mask )
{
	cli();                                          // read key state and key press atomic !
	return get_key_press( ~key_state & key_mask );
}

///////////////////////////////////////////////////////////////////
//
uint8_t get_key_long( uint8_t key_mask )
{
	return get_key_press( get_key_rpt( key_mask ));
}

/************************************************************************/
/* Defines DS1337                                                       */
/************************************************************************/
#define DS1337 0xD0
#define SECONDS 0x00
#define MINUTES 0x01
#define BCD2BIN(val) (((val)&15) + ((val)>>4)*10)
#include "i2cmaster.h"


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

ISR( INT0_vect ){
	if(Flags.change == 0){
		Flags.change = 1;
		state = next_state;
	}
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
	
	//temperaturmessung in Interrupt starten
	TCCR1B |= ((1<<CS12));
	TIMSK |= (1<<TOIE1);


	//Timer0, mit Interrupt ca. jede 1000/s
	TCCR0 |= ((1<<CS01));
	TIMSK |= (1<<TOIE0);
	int offset = 0;
	encode_init();
	
	//PD2 auf Eingang für Interrupt vom Button
	DDRD &= ~(1 << DDD2);
	PORTD |= (1<<PD2); //interner PullUp
	//GICR |= (1<<INT0); //Interrupt für INT0 aktivieren
	TCCR2 |= ( (1<<CS22) || (1<<CS21) || (1<<CS20) );
	TCNT2 = 255-117;
	TIMSK |= (1<<TOIE2);
	
	state = MAIN;
	uint8_t *wheel_target = NULL; //Wird auf die Variable gelenkt, die verändert werden soll
	uint8_t wheel_max = 0;
	uint8_t wheel_min = 0;

	//uint8_t ret;
	i2c_init();
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

//Read RTC
/*
			i2c_start_wait(DS1337+I2C_WRITE);
			i2c_write(0x00);
			i2c_rep_start(DS1337+I2C_READ);
			ret = i2c_readAck();
			ret = BCD2BIN(ret);
*/
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
			}else{
				frontend_else(&wheel_target, &next_state, &state, &wheel_min, &wheel_max);
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