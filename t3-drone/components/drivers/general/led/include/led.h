
#ifndef __LED_H__
#define __LED_H__

#include <stdbool.h>

//Led polarity configuration constant
#define LED_POL_POS 0
#define LED_POL_NEG 1

/*#define LED_GPIO_BLUE  CONFIG_LED_PIN_BLUE
#define LED_POL_BLUE   LED_POL_POS
#define LED_GPIO_GREEN CONFIG_LED_PIN_GREEN  //different from pcb design
#define LED_POL_GREEN  LED_POL_POS*/
#define LED_GPIO_RED   CONFIG_LED_PIN_RED
#define LED_POL_RED    LED_POL_POS

#define ERR_LED         LED_RED
/*
#define LINK_LED         LED_GREEN
//#define CHG_LED          LED_RED
#define LOWBAT_LED       LED_RED
//#define LINK_DOWN_LED    LED_BLUE
#define SYS_LED          LED_BLUE
#define ERR_LED1         LED_RED
#define ERR_LED2         LED_RED
*/
#define LED_NUM 1

//typedef enum {LED_BLUE = 0, LED_RED, LED_GREEN} led_t;
typedef enum { LED_RED =0} led_t;

void ledInit();

bool ledTest();

// Clear all configured LEDs
void ledClearAll(void);
/*
// Set all configured LEDs
void ledSetAll(void);
*/
// Procedures to set the status of the LEDs
void ledSet(led_t led, bool value);
/*
void ledTask(void *param);
*/
//Legacy functions
#define ledSetRed(VALUE) ledSet(LED_RED, VALUE)

#endif
