#ifndef _SIX_STEP_H_
#define _SIX_STEP_H_

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

#define PIN_PWM_IN1_GPIO    //PIO_PA23_IDX
#define PIN_PWM_IN1_FLAGS   //(PIO_PERIPH_B | PIO_DEFAULT)
#define PIN_PWM_IN1_CHANNEL //PWM_CHANNEL_0

#define PIN_PWM_IN2_GPIO    //PIO_PB14_IDX
#define PIN_PWM_IN2_FLAGS   //(PIO_PERIPH_B | PIO_DEFAULT)
#define PIN_PWM_IN2_CHANNEL //PWM_CHANNEL_3

#define PIN_PWM_IN3_GPIO    //PIO_PA23_IDX
#define PIN_PWM_IN3_FLAGS   //(PIO_PERIPH_B | PIO_DEFAULT)
#define PIN_PWM_IN3_CHANNEL //PWM_CHANNEL_0




/* =============== Prototypes =============== */
void Hall_Phase(void);

#endif