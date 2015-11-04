#ifndef _SIX_STEP_H_
#define _SIX_STEP_H_

#include "asf.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "stdio.h"
#include "lcd_aux.h"

#define IRQ_PRIOR_PIO    0

/* =============== Hall sensor =============== */

#define PIN_HALL_1 PIO_PA2_IDX
#define PIN_HALL_1_MASK PIO_PA2
#define PIN_HALL_1_PIO PIOA
#define PIN_HALL_1_ID ID_PIOA
#define PIN_HALL_1_TYPE PIO_INPUT
#define PIN_HALL_1_ATTR  (PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE)

#define PIN_HALL_2 PIO_PA3_IDX
#define PIN_HALL_2_MASK PIO_PA3
#define PIN_HALL_2_PIO PIOA
#define PIN_HALL_2_ID ID_PIOA
#define PIN_HALL_2_TYPE PIO_INPUT
#define PIN_HALL_2_ATTR  (PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE)

#define PIN_HALL_3 PIO_PA4_IDX
#define PIN_HALL_3_MASK PIO_PA4
#define PIN_HALL_3_PIO PIOA
#define PIN_HALL_3_ID ID_PIOA
#define PIN_HALL_3_TYPE PIO_INPUT
#define PIN_HALL_3_ATTR  (PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE)

/* =============== Acionamento do motor =============== */

#define PIN_PWM_IN1_GPIO    PIO_PB0_IDX
#define PIN_PWM_IN1_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)
#define PIN_PWM_IN1_CHANNEL PWM_CHANNEL_0
#define PIN_PWM_EN1_GPIO    PIO_PA5_IDX
#define PIN_PWM_EN1_FLAGS	(PIO_OUTPUT_0 | PIO_DEFAULT)

#define PIN_PWM_IN2_GPIO    PIO_PB1_IDX
#define PIN_PWM_IN2_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)
#define PIN_PWM_IN2_CHANNEL PWM_CHANNEL_1
#define PIN_PWM_EN2_GPIO    PIO_PA6_IDX
#define PIN_PWM_EN2_FLAGS	(PIO_OUTPUT_0 | PIO_DEFAULT)

#define PIN_PWM_IN3_GPIO    PIO_PB14_IDX
#define PIN_PWM_IN3_FLAGS   (PIO_PERIPH_B | PIO_DEFAULT)
#define PIN_PWM_IN3_CHANNEL PWM_CHANNEL_3
#define PIN_PWM_EN3_GPIO    PIO_PA21_IDX
#define PIN_PWM_EN3_FLAGS	(PIO_OUTPUT_0 | PIO_DEFAULT)



/* =============== PWM =============== */
/** PWM frequency in Hz */
#define PWM_FREQUENCY      20000 //20kHz
/** Period value of PWM output waveform */
#define PERIOD_VALUE       10
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0

/* =============== HEADER USART =============== */
#define STRING_EOL    "\n\r"
#define STRING_HEADER	"-- six-step --\n\r" \
						"-- "BOARD_NAME" --\n\r" \
						"-- Caue Menegaldo \n\r"\
						"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/* =============== Prototypes =============== */
void Hall_Phase(void);
void configure_hall(void);
void configure_buttons(void);
void Button1_Handler(uint32_t id, uint32_t mask);
void Button2_Handler(uint32_t id, uint32_t mask);
void Hall_Handler(uint32_t id, uint32_t mask);
pwm_channel_t configure_pwm(void);
void start_lcd(uint32_t pos_lcd_x, uint32_t pos_lcd_y, uint32_t ul_duty, uint32_t hall_1, uint32_t hall_2, uint32_t hall_3, uint8_t phase);
#endif