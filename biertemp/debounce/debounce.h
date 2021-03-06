/*
 * debounce.h
 *
 * Created: 04.12.2013 10:16:38
 *  Author: huehnerhose
 */ 


#ifndef DEBOUNCE_H_
#define DEBOUNCE_H_

#define DEBOUNCE_KEY_DDR         DDRD
#define DEBOUNCE_KEY_PORT        PORTD
#define DEBOUNCE_KEY_PIN         PIND
#define DEBOUNCE_KEY0            2
//#define KEY1            4
//#define KEY2            4
#define DEBOUNCE_ALL_KEYS        (1<<DEBOUNCE_KEY0)

#define DEBOUNCE_TIMER TIMER2_OVF_vect

//#define REPEAT_MASK     (1<<KEY1 | 1<<KEY2)       // repeat: key1, key2
//#define REPEAT_START    50                        // after 500ms
//#define REPEAT_NEXT     20                        // every 200ms
//
extern volatile uint8_t debounce_key_state;                                // debounced and inverted key state:
extern volatile uint8_t debounce_key_press;                                // key press detect
extern volatile uint8_t debounce_key_rpt;                                  // key long press and repeat

uint8_t debounce_get_key_press( uint8_t key_mask );
uint8_t debounce_get_key_rpt( uint8_t key_mask );
uint8_t debounce_get_key_state( uint8_t key_mask );
uint8_t debounce_get_key_short( uint8_t key_mask );
uint8_t debounce_get_key_long( uint8_t key_mask );
void debounce_init();

#endif /* DEBOUNCE_H_ */