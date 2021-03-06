/**
 * Ensaio do BLDC para validacao do simulador.
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

uint8_t phase = 0;

static bool flag_hab_m = 0;
static bool sel_rot = 0;

static bool flag_hab_test = 0;

uint32_t vel_count = 0;
uint32_t vel_pulse = 0;

/** PWM channel instance for LEDs */
pwm_channel_t g_pwm_channel;

static uint32_t ul_duty = INIT_DUTY_VALUE;  /* PWM duty cycle rate */


void SPI_Handler(void)
{
	ili9225_spi_handler();
}


void PWM_Handler(void)
{
	//Para utilizar interrup��o do pwm configurar e habilitar em six_step.c
	
	//static uint32_t ul_count = 0;  /* PWM counter value */
	//uint32_t events = pwm_channel_get_interrupt_status(PWM);
//
	///* Interrupt on PIN_PWM_IN1_CHANNEL */
	//if ((events & (1 << PIN_PWM_IN1_CHANNEL)) == (1 << PIN_PWM_IN1_CHANNEL))
	//{
		//ul_count++;
		//if (ul_count == (PWM_FREQUENCY / (PERIOD_VALUE - INIT_DUTY_VALUE))) {
			//ul_count = 0;
		//}
	//}
}

void Button1_Handler(uint32_t id, uint32_t mask)
{
	/*Bot�o 1 aumenta o duty cicle (ul_duty)*/
	if (PIN_PUSHBUTTON_1_ID == id && PIN_PUSHBUTTON_1_MASK == mask)
	{
		ioport_toggle_pin_level(LED1_GPIO);
		flag_hab_test = 1;
	}
}

void Button2_Handler(uint32_t id, uint32_t mask)
{
	/*Bot�o 2 diminui o duty cicle (ul_duty)*/
	if (PIN_PUSHBUTTON_2_ID == id && PIN_PUSHBUTTON_2_MASK == mask)
	{
		ioport_toggle_pin_level(LED2_GPIO);
		ul_duty  = 0;
		flag_hab_test = 0;
	}
}

void Hall_Phase(void)
{
	static uint8_t hall_code = 0;
	static uint32_t ul_duty1, ul_duty2, ul_duty3, high1, high2, low1;
	
	hall_1 = ioport_get_pin_level(PIN_HALL_1);
	hall_2 = ioport_get_pin_level(PIN_HALL_2);
	hall_3 = ioport_get_pin_level(PIN_HALL_3);

	hall_code = (hall_3<<2) | (hall_2<<1) | (hall_1);
	
	if (sel_rot)
		{
			hall_code = ~hall_code;
		}

	switch (hall_code & 0b00000111){
	
	case 5 : //phase 1
		phase=1;
		ul_duty1 = ul_duty;
		ul_duty2 = 0;
		ul_duty3 = 0;
		high1 = PIN_PWM_EN1_GPIO;
		high2 = PIN_PWM_EN2_GPIO;
		low1 = PIN_PWM_EN3_GPIO;
		break; 
	case 1 : //phase 2
		phase=2;
		ul_duty1 = ul_duty;
		ul_duty2 = 0;
		ul_duty3 = 0;
		high1 = PIN_PWM_EN1_GPIO;
		high2 = PIN_PWM_EN3_GPIO;
		low1 = PIN_PWM_EN2_GPIO;
		break;
	case 3 : //phase 3
		phase=3;
		ul_duty1 = 0;
		ul_duty2 = ul_duty;
		ul_duty3 = 0;
		high1 = PIN_PWM_EN2_GPIO;
		high2 = PIN_PWM_EN3_GPIO;
		low1 = PIN_PWM_EN1_GPIO;
		break;
	case 2 : //phase 4
		phase=4;
		ul_duty1 = 0;
		ul_duty2 = ul_duty;
		ul_duty3 = 0;
		high1 = PIN_PWM_EN1_GPIO;
		high2 = PIN_PWM_EN2_GPIO;
		low1 = PIN_PWM_EN3_GPIO;
		break;
	case 6 : //phase 5
		phase=5;
		ul_duty1 = 0;
		ul_duty2 = 0;
		ul_duty3 = ul_duty;
		high1 = PIN_PWM_EN1_GPIO;
		high2 = PIN_PWM_EN3_GPIO;
		low1 = PIN_PWM_EN2_GPIO;
		break;
	case 4 : //phase 6
		phase=6;
		ul_duty1 = 0;
		ul_duty2 = 0;
		ul_duty3 = ul_duty;
		high1 = PIN_PWM_EN2_GPIO;
		high2 = PIN_PWM_EN3_GPIO;
		low1 = PIN_PWM_EN1_GPIO;
		break; 
	}
	
	g_pwm_channel.channel = PIN_PWM_IN1_CHANNEL;
	pwm_channel_update_duty(PWM, &g_pwm_channel, ul_duty1);
	g_pwm_channel.channel = PIN_PWM_IN2_CHANNEL;
	pwm_channel_update_duty(PWM, &g_pwm_channel, ul_duty2);
	g_pwm_channel.channel = PIN_PWM_IN3_CHANNEL;
	pwm_channel_update_duty(PWM, &g_pwm_channel, ul_duty3);
	
	g_pwm_channel.channel = PIN_PWM_GENERAL_CHANNEL;
	pwm_channel_update_duty(PWM, &g_pwm_channel, ul_duty);
	
	gpio_set_pin_high(high1);
	gpio_set_pin_high(high2);
	gpio_set_pin_low(low1);
	
}

void Hall_Handler(uint32_t id, uint32_t mask)
{
	if (PIN_HALL_1_ID == id && PIN_HALL_1_MASK == mask)
	{
		vel_pulse++;
		Hall_Phase();
	}
	
	else
	{
		if((PIN_HALL_2_ID == id && PIN_HALL_2_MASK == mask) ||
		(PIN_HALL_3_ID == id && PIN_HALL_3_MASK == mask))
		{
			Hall_Phase();
		}
	}
}

void TC0_Handler(void)
{
	volatile uint32_t ul_dummy;

	/* Clear status bit to acknowledge interrupt */
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	ioport_toggle_pin_level(LED0_GPIO);
	vel_count = vel_pulse*30*TC_HZ_FREQUENCY/POLE_PAIRS;
	vel_pulse = 0;
	
	if (flag_hab_test)
	{
		if (ul_duty == 0 || vel_count == 0)
		{
			flag_hab_m = 1;
		}
		
		if(ul_duty < PERIOD_VALUE)
		{
			ul_duty++;
		}
	}
}

int main(void)
{
	sysclk_init();
	board_init();
	configure_buttons();
	configure_hall();
	configure_console();
	configure_lcd();
	g_pwm_channel = configure_pwm();
	configure_tc();

	/* Cabe�alho do lcd */
	ili9225_set_foreground_color(COLOR_BLACK);
	ili9225_draw_string(40, 20, (uint8_t *)"six-step");
	pos_lcd_x = 20;
	pos_lcd_y = 40;
	escreve_int_lcd("dc = ", ul_duty, pos_lcd_x, 40);
	escreve_int_lcd("hall1 = ", hall_1, pos_lcd_x, 60);
	escreve_int_lcd("hall2 = ", hall_2, pos_lcd_x, 80);
	escreve_int_lcd("hall3 = ", hall_3, pos_lcd_x, 100);
	escreve_int_lcd("phase = ", phase, pos_lcd_x, 120);
	escreve_int_lcd("vel = ", vel_count, pos_lcd_x, 140);

	/* Infinite loop */
	while (1)
	{
		static uint8_t phase_aux;
		static uint32_t hall_1_aux, hall_2_aux, hall_3_aux, ul_duty_aux, vel_count_aux;

		/* Atualiza o display somente quando houver altera��o nas vari�veis que ser�o apresentadas */
		
		if(ul_duty_aux != ul_duty)
		{
			escreve_int_lcd("dc = ", ul_duty*100/PERIOD_VALUE, pos_lcd_x, 40);
			ul_duty_aux = ul_duty;
		}
		
		if(phase_aux != phase || hall_1_aux != hall_1 || hall_2_aux != hall_2 || hall_3_aux != hall_3 || vel_count_aux != vel_count)
		{
			escreve_int_lcd("hall1 = ", hall_1, pos_lcd_x, 60);
			escreve_int_lcd("hall2 = ", hall_2, pos_lcd_x, 80);
			escreve_int_lcd("hall3 = ", hall_3, pos_lcd_x, 100);
			escreve_int_lcd("phase = ", phase, pos_lcd_x, 120);
			escreve_int_lcd("vel = ", vel_count, pos_lcd_x, 140);

			phase_aux = phase;
			hall_1_aux = hall_1;
			hall_2_aux = hall_2;
			hall_3_aux = hall_3;
			vel_count_aux = vel_count;
		}
		
		if(flag_hab_m && ul_duty != 0)
		{
			Hall_Phase();
			flag_hab_m = 0;
		}
	}
}
