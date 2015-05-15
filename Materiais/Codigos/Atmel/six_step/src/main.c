/**
 * Acionamento do BLDC através do método six-step.
 * 
 * Leitura de 3 sensores hall por interrupção externa,
 * verificando borda de subida e borda de descida, a
 * cada evento é verificada uma tabela gray onde inf-
 * orma quais bobinas do motor devem ser acionadas.
 * O acionamento da bobina é feito por PWM.
 *
 */

#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "stdio.h"
#include "lcd_aux.h"
#include "six_step.h"
#include <delay.h>


uint32_t pos_lcd_x;
uint32_t pos_lcd_y;
const char *pre_lcd;

uint32_t hall_1 = 0;
uint32_t hall_2 = 0;
uint32_t hall_3 = 0;

uint8_t phase_lcd = 0;

static uint8_t bt1 = 0;
static uint8_t bt2 = 0;

/** PWM channel instance for LEDs */
pwm_channel_t g_pwm_channel;

static uint32_t ul_duty = INIT_DUTY_VALUE;  /* PWM duty cycle rate */


void SPI_Handler(void)
{
	ili9225_spi_handler();
}

void PWM_Handler(void)
{
	static uint32_t ul_count = 0;  /* PWM counter value */
	
	
	uint32_t events = pwm_channel_get_interrupt_status(PWM);

	/* Interrupt on PIN_PWM_IN1_CHANNEL */
	if ((events & (1 << PIN_PWM_IN1_CHANNEL)) == (1 << PIN_PWM_IN1_CHANNEL))
	{
		ul_count++;

		if (ul_count == (PWM_FREQUENCY / (PERIOD_VALUE - INIT_DUTY_VALUE))) {

			/* Set new duty cycle */
			ul_count = 0;
			g_pwm_channel.channel = PIN_PWM_IN1_CHANNEL;
			pwm_channel_update_duty(PWM, &g_pwm_channel, ul_duty);
			g_pwm_channel.channel = PIN_PWM_IN2_CHANNEL;
			pwm_channel_update_duty(PWM, &g_pwm_channel, ul_duty);
		}
	}
}

void Button1_Handler(uint32_t id, uint32_t mask)
{
	/*Botão 1 aumenta o duty cicle (ul_duty)*/
	
	if (PIN_PUSHBUTTON_1_ID == id && PIN_PUSHBUTTON_1_MASK == mask) {

		if(ul_duty < PERIOD_VALUE) {
			ul_duty++;

			bt1++;
			
		}
	}

}

void Button2_Handler(uint32_t id, uint32_t mask)
{
	/*Botão 2 diminui o duty cicle (ul_duty)*/

	if (PIN_PUSHBUTTON_2_ID == id && PIN_PUSHBUTTON_2_MASK == mask) {
	
		if(ul_duty > INIT_DUTY_VALUE){
		ul_duty--;
		
		bt2++;
		
		}
	
	}
}

void Hall_Phase(void)
{
	static uint8_t phase = 0;
	
	hall_1 = ioport_get_pin_level(PIN_HALL_1);
	hall_2 = ioport_get_pin_level(PIN_HALL_2);
	hall_3 = ioport_get_pin_level(PIN_HALL_3);

	phase = (hall_3<<2) | (hall_2<<1) | (hall_1);

	switch (phase){
	
	case 5 : phase_lcd=1; break; //phase 1
	case 1 : phase_lcd=2; break; //phase 2
	case 3 : phase_lcd=3; break; //phase 3
	case 2 : phase_lcd=4; break; //phase 4
	case 6 : phase_lcd=5; break; //phase 5
	case 4 : phase_lcd=6; break; //phase 6
	}
}

void Hall_Handler(uint32_t id, uint32_t mask)
{
	if ((PIN_HALL_1_ID == id && PIN_HALL_1_MASK == mask) || 
		(PIN_HALL_2_ID == id && PIN_HALL_2_MASK == mask) || 
		(PIN_HALL_3_ID == id && PIN_HALL_3_MASK == mask))
	{
		Hall_Phase();
	}
}
//uint32_t six[6] = {0x4, 0x1, 0x2, 0x}
//
	//
//funcao(hall_1, hall_2, hall_3);	
//
//uint32_t valor = (hall_3 << 2) | (hall_2 << 1) |  hall_1;
//
//PIOA->PIO_CODR = 0xFFFFFF;
//PIOA->PIO_SODR = six[valor];


int main(void)
{
	sysclk_init();
	board_init();
	configure_buttons();
	configure_hall();
	configure_console();
	configure_lcd();
	g_pwm_channel = configure_pwm();

	/* Cabeçalho do lcd */
	ili9225_set_foreground_color(COLOR_BLACK);
	ili9225_draw_string(40, 20, (uint8_t *)"six-step");
	pos_lcd_x = 20;
	pos_lcd_y = 40;
	escreve_int_lcd("bt1 = ", bt1, pos_lcd_x, 40);
	escreve_int_lcd("bt2 = ", bt2, pos_lcd_x, 65);
	escreve_int_lcd("dc = ", ul_duty, pos_lcd_x, 190);
	escreve_int_lcd("hall1 = ", hall_1, pos_lcd_x, 90);
	escreve_int_lcd("hall2 = ", hall_2, pos_lcd_x, 115);
	escreve_int_lcd("hall3 = ", hall_3, pos_lcd_x, 140);
	escreve_int_lcd("phase = ", phase_lcd, pos_lcd_x, 165);


	//gpio_configure_pin(PIO_PC21_IDX, (PIO_PERIPH_B | PIO_DEFAULT));  // PIn PC21
	//gpio_configure_pin(PIN_PWM_LED0_GPIO, PIN_PWM_LED0_FLAGS);
	
	

	/* Infinite loop */
	while (1) {
		static uint8_t bt1_aux, bt2_aux, phase_lcd_aux;
		static uint32_t hall_1_aux, hall_2_aux, hall_3_aux, ul_duty_aux;

		/* Atualiza o display somente quando houver alteração nas variáveis que serão apresentadas */
		
		if(bt1_aux != bt1 || bt2_aux != bt2 || ul_duty_aux != ul_duty)
		{
			escreve_int_lcd("bt1 = ", bt1, pos_lcd_x, 40);
			escreve_int_lcd("bt2 = ", bt2, pos_lcd_x, 65);
			escreve_int_lcd("dc = ", ul_duty, pos_lcd_x, 190);
			bt1_aux = bt1;
			bt2_aux = bt2;
			ul_duty_aux = ul_duty;
		}
		
		if(phase_lcd_aux != phase_lcd || hall_1_aux != hall_1 || hall_2_aux != hall_2 || hall_3_aux != hall_3)
		{
			escreve_int_lcd("hall1 = ", hall_1, pos_lcd_x, 90);
			escreve_int_lcd("hall2 = ", hall_2, pos_lcd_x, 115);
			escreve_int_lcd("hall3 = ", hall_3, pos_lcd_x, 140);
			escreve_int_lcd("phase = ", phase_lcd, pos_lcd_x, 165);

			phase_lcd_aux = phase_lcd;
			hall_1_aux = hall_1;
			hall_2_aux = hall_2;
			hall_3_aux = hall_3;
		}
	}
}
