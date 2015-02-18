#define NDEBUG
#define OW_ONE_BUS
#define NDEBUG_CLOCK

#define MAXSENSORS 3
#define NEWLINESTR ""
#define uart_puts_P

#define HEATER_PORT PORTD
#define BUZZER_PORT PORTD
#define HEATER_DDR DDRD
#define HEATER_PIN PD4

#define BUZZER_DDR DDRD
#define BUZZER_PIN PIND
#define BUZZER_PIN1 PD5
#define BUZZER_PIN2 PD6


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