#ifndef _SIX_STEP_H_
#define _SIX_STEP_H_

#include "asf.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "stdio.h"
#include "lcd_aux.h"
#include "touch_api.h"

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
#define PERIOD_VALUE       200
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0

/* =============== HEADER USART =============== */
#define STRING_EOL    "\n\r"
#define STRING_HEADER		STRING_EOL\
						"   ACIONAMENTO SIX-STEP DO MOTOR BLDC"STRING_EOL\
							STRING_EOL\
						"   Para utilizar o teclado:"STRING_EOL\
						"   Pressione a tecla 'T' para apresentar os dados do bldc na tela;"STRING_EOL\
						"   Pressione a tecla 'A' para aumentar o duty-cicle;"STRING_EOL\
						"   Pressione a tecla 'S' para diminuir o duty-cicle;"STRING_EOL\
						"   Pressione a tecla 'D' para acionamento em forma de rampa;"STRING_EOL\
						"   Pressione a tecla 'F' para acionamento em forma de degrau;"STRING_EOL\
						"   Pressione a tecla 'I' para inverter o sentido de rotacao;"STRING_EOL\
							STRING_EOL\
						"   Para utilizar o touch:"STRING_EOL\
						"   Pressione a seta para a direita para aumentar o duty-cicle;"STRING_EOL\
						"   Pressione a seta para a esquerda para diminuir o duty-cicle;"STRING_EOL\
						"   Utilize o barcode para aumentar e diminuir o duty-cicle;"STRING_EOL\
							STRING_EOL\


/* =============== QTOUCH =============== */
/** Qtouch key number */
#define BOARD_KEY_NUM_2
/** The PIO numbers for silder is before key */
#define BOARD_SILDER_BEFOR_KEY
/** Qtouch left key ID */
#define BOARD_LEFT_KEY_ID    1
/** Qtouch right key ID */
#define BOARD_RIGHT_KEY_ID    2
/** Qtouch left key channel */
#define BOARD_LEFT_KEY_CHANNEL    CHANNEL_4
/** Qtouch right key channel */
#define BOARD_RIGHT_KEY_CHANNEL    CHANNEL_5
/** Qtouch slider start channel */
#define BOARD_SLIDER_START_CHANNEL    CHANNEL_0
/** Qtouch slider end channel */
#define BOARD_SLIDER_END_CHANNEL    CHANNEL_2

/** Qtouch library type: Qtouch / QMatrix */
#define QTOUCH_LIB_TPYE_MASK    0x01
/** Qtouch library compiler type offset: GCC / IAR */
#define QTOUCH_LIB_COMPILER_OFFSET    2
/** Qtouch library compiler type mask */
#define QTOUCH_LIB_COMPILER_MASK    0x01
/** Qtouch library maximum channels offset */
#define QTOUCH_LIB_MAX_CHANNEL_OFFSET    3
/** Qtouch library maximum channels mask */
#define QTOUCH_LIB_MAX_CHANNEL_MASK   0x7F
/** Qtouch library supports keys only offset */
#define QTOUCH_LIB_KEY_ONLY_OFFSET    10
/** Qtouch library supports keys only mask */
#define QTOUCH_LIB_KEY_ONLY_MASK   0x01
/** Qtouch library maximum rotors/silders offset */
#define QTOUCH_LIB_ROTOR_NUM_OFFSET    11
/** Qtouch library maximum rotors/silders mask */
#define QTOUCH_LIB_ROTOR_NUM_MASK   0x1F

#define GET_SENSOR_STATE(SENSOR_NUMBER) (qt_measure_data.qt_touch_status.sensor_states[(SENSOR_NUMBER/8)] & (1 << (SENSOR_NUMBER % 8)))
#define GET_ROTOR_SLIDER_POSITION(ROTOR_SLIDER_NUMBER) qt_measure_data.qt_touch_status.rotor_slider_values[ROTOR_SLIDER_NUMBER]

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