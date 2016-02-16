/**
 * Acionamento do BLDC atraves do metodo six-step com 
 * leitura de velocidade e utilizacao do qtouch da SAN3N-EK.
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
#include "touch_api.h"
#include <delay.h>

uint32_t pos_lcd_x;
uint32_t pos_lcd_y;
const char *pre_lcd;

uint32_t hall_1 = 0;
uint32_t hall_2 = 0;
uint32_t hall_3 = 0;

uint8_t phase = 0;
uint8_t motor_run = 0;
uint8_t motor_aux = 0;
uint8_t ensaio = 0;

static bool flag_hab_m = 0;
static bool sel_rot = 0;


/** PWM channel instance for LEDs */
pwm_channel_t g_pwm_channel;

static uint32_t ul_duty = INIT_DUTY_VALUE;  /* PWM duty cycle rate */

/* Flag set by timer ISR when it's time to measure touch */
static volatile uint8_t time_to_measure_touch = 0u;

/* Current time, set by timer ISR */
static volatile uint16_t current_time_ms_touch = 0u;

/* Timer period in msec */
uint16_t qt_measurement_period_msec = 25u;

void SysTick_Handler(void)
{
	/* Set flag: it's time to measure touch */
	time_to_measure_touch = 1u;

	/* Update the current time */
	current_time_ms_touch += qt_measurement_period_msec;
}

static void init_timer_isr(void)
{
	if (SysTick_Config((sysclk_get_cpu_hz() / 1000) *
					qt_measurement_period_msec)) {
		printf("-F- Systick configuration error\n\r");
	}
}

/**
 * \brief This will fill the default threshold values in the configuration
 * data structure.But User can change the values of these parameters.
 */
static void qt_set_parameters(void)
{
	qt_config_data.qt_di = DEF_QT_DI;
	qt_config_data.qt_neg_drift_rate = DEF_QT_NEG_DRIFT_RATE;
	qt_config_data.qt_pos_drift_rate = DEF_QT_POS_DRIFT_RATE;
	qt_config_data.qt_max_on_duration = DEF_QT_MAX_ON_DURATION;
	qt_config_data.qt_drift_hold_time = DEF_QT_DRIFT_HOLD_TIME;
	qt_config_data.qt_recal_threshold = DEF_QT_RECAL_THRESHOLD;
	qt_config_data.qt_pos_recal_delay = DEF_QT_POS_RECAL_DELAY;
}

/**
 * \brief Configure the sensors.
 */
static void config_sensors(void)
{
	qt_enable_slider(BOARD_SLIDER_START_CHANNEL, BOARD_SLIDER_END_CHANNEL,
			AKS_GROUP_1, 16u, HYST_6_25, RES_8_BIT, 0u);
	qt_enable_key(BOARD_LEFT_KEY_CHANNEL, AKS_GROUP_1, 18u, HYST_6_25);
	qt_enable_key(BOARD_RIGHT_KEY_CHANNEL, AKS_GROUP_1, 18u, HYST_6_25);
}

void SPI_Handler(void)
{
	ili9225_spi_handler();
}

void PWM_Handler(void)
{
	/* Para utilizar interrupção do pwm configurar e habilitar em six_step.c */
	
	static uint32_t ul_count = 0;  /* PWM counter value */
	static uint32_t ul_count2 = 0;
	static uint8_t sub = 1; // 1 -> subindo; 0 -> descendo;
	
	uint32_t events = pwm_channel_get_interrupt_status(PWM);

	/* Interrupt on PIN_PWM_IN1_CHANNEL */
	if ((events & (1 << PIN_PWM_IN1_CHANNEL)) == (1 << PIN_PWM_IN1_CHANNEL))
	{
		ul_count++;
		if (ul_count == (PWM_FREQUENCY*10 / (PERIOD_VALUE - INIT_DUTY_VALUE))) // a cada 50ms
		{
			ul_count = 0;
			ul_count2++;
			
			/*rotina para verificar se o motor está rodando*/
			if(motor_aux!=0)
			{
				motor_run = 1;
				motor_aux = 0;
			}
			else motor_run = 0;
			
			/*rotina para realizar o ensaio 1 - ensaio de rampa */
			if (ensaio == 1)
			{
				if (sub)
				{
					if(ul_duty < PERIOD_VALUE) ul_duty++;
					else sub = 0;
				}
				else
				{
					if(ul_duty > INIT_DUTY_VALUE) ul_duty--;
					else sub = 1;
				}
			}
			if (ul_count2 == 20) // a cada 1s
			{
				ul_count2 = 0;
				/*rotina para realizar ensaio 2 - acionamento degrau */
				if	(ensaio == 2)
				{
					if (sub)
					{
						if(ul_duty < PERIOD_VALUE) ul_duty = ul_duty + 20;
						else sub = 0;
					}
					else
					{
						if(ul_duty > INIT_DUTY_VALUE) ul_duty = ul_duty - 20;
						else sub = 1;
					}
				}
			}
		}
	}
}

void Button1_Handler(uint32_t id, uint32_t mask)
{
	/*Botão 1 aumenta o duty cicle (ul_duty)*/
	if (PIN_PUSHBUTTON_1_ID == id && PIN_PUSHBUTTON_1_MASK == mask) {
		if(ul_duty < PERIOD_VALUE) ul_duty++;
		
	}
}

void Button2_Handler(uint32_t id, uint32_t mask)
{
	/*Botão 2 diminui o duty cicle (ul_duty)*/
	if (PIN_PUSHBUTTON_2_ID == id && PIN_PUSHBUTTON_2_MASK == mask) {
	
		if(ul_duty > INIT_DUTY_VALUE) ul_duty--;
		
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
	
	if (sel_rot) hall_code = ~hall_code;

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
	gpio_set_pin_high(high1);
	gpio_set_pin_high(high2);
	gpio_set_pin_low(low1);
	
}

void Hall_Handler(uint32_t id, uint32_t mask)
{
	if ((PIN_HALL_1_ID == id && PIN_HALL_1_MASK == mask) || 
		(PIN_HALL_2_ID == id && PIN_HALL_2_MASK == mask) || 
		(PIN_HALL_3_ID == id && PIN_HALL_3_MASK == mask))
	{
		Hall_Phase();
		motor_aux++;
	}
}

int main(void)
{
		/*Status flags to indicate the re-burst for library */
	uint16_t status_flag = 0u;
	uint16_t burst_flag = 0u;

	uint8_t lft_pressed = 0;
	uint8_t rgt_pressed = 0;

	static uint8_t old_position = 0;

	uint8_t uc_char;
	uint8_t uc_flag;
	sysclk_init();
	board_init();
	configure_buttons();
	configure_hall();

	wdt_disable(WDT);
	pmc_enable_periph_clk(ID_PIOC);
	qt_reset_sensing();
	config_sensors();
	qt_init_sensing();
	/* Set the parameters like recalibration threshold, Max_On_Duration etc in this function by the user */
	qt_set_parameters();
	init_timer_isr();
	qt_filter_callback = 0;

	configure_console();
	printf(STRING_HEADER);
	
	configure_lcd();
	g_pwm_channel = configure_pwm();

	/* Cabeçalho do lcd */
	pos_lcd_x = 20;
	pos_lcd_y = 40;
	start_lcd(pos_lcd_x, pos_lcd_y, ul_duty, hall_1, hall_2, hall_3, phase);

	/* Infinite loop */
	while (1) {
		static uint8_t phase_aux;
		static uint32_t hall_1_aux, hall_2_aux, hall_3_aux, ul_duty_aux;

		/* Atualiza o display somente quando houver alteração nas variáveis que serão apresentadas */
		
		if(ul_duty_aux != ul_duty)
		{
			escreve_int_lcd("dc = ", ul_duty*100/PERIOD_VALUE, pos_lcd_x, 40);
			ul_duty_aux = ul_duty;
		}
		
		if(phase_aux != phase || hall_1_aux != hall_1 || hall_2_aux != hall_2 || hall_3_aux != hall_3)
		{
			escreve_int_lcd("hall1 = ", hall_1, pos_lcd_x, 60);
			escreve_int_lcd("hall2 = ", hall_2, pos_lcd_x, 80);
			escreve_int_lcd("hall3 = ", hall_3, pos_lcd_x, 100);
			escreve_int_lcd("phase = ", phase, pos_lcd_x, 120);

			phase_aux = phase;
			hall_1_aux = hall_1;
			hall_2_aux = hall_2;
			hall_3_aux = hall_3;
		}
		
		if(motor_run == 0 && ul_duty != 0)
			Hall_Phase();
		
		uc_char = 0;
		uc_flag = uart_read(CONSOLE_UART, &uc_char);
		if (!uc_flag) {
			if (uc_char == 't') {
				printf("   duty cicle = %lu \r\n",ul_duty*100/PERIOD_VALUE);
				printf("   hall1 = %lu \r\n", hall_1);
				printf("   hall2 = %lu \r\n", hall_2);
				printf("   hall3 = %lu \r\n", hall_3);
				printf("   phase = %u \r\n\n", phase);
			}
			if (uc_char == 'a'){				
				if(ul_duty < PERIOD_VALUE) ul_duty++;
				printf("   duty cicle = %lu \r\n",ul_duty*100/PERIOD_VALUE);
			}
			if (uc_char == 's'){
				if(ul_duty > INIT_DUTY_VALUE) ul_duty--;
				printf("   duty cicle = %lu \r\n",ul_duty*100/PERIOD_VALUE);
			}
			if (uc_char == 'd')
			{
				ensaio = 1;
				printf("   Ensaio de rampa\r\n");
				printf("   para parar pressione a letra 'P'\r\n");	
			}
			if (uc_char == 'f')
			{
				ensaio = 2;
				printf("   Ensaio de degrau\r\n");
				printf("   para parar pressione a letra 'P'\r\n");
			}
			if (uc_char == 'p')
			{
				ensaio = 0;
				ul_duty = 0;
			}
			if (uc_char == 'i')
			{
				sel_rot = !sel_rot;
				printf("   Rotacao invertida\r\n");
				printf("   para parar pressione a letra 'P'\r\n");
			}
		}
		
		if (time_to_measure_touch) {

			/* Clear flag: it's time to measure touch */
			time_to_measure_touch = 0u;

			do {
				/*  One time measure touch sensors    */
				status_flag = qt_measure_sensors(current_time_ms_touch);

				burst_flag = status_flag & QTLIB_BURST_AGAIN;

				/*Time critical host application code goes here */

			} while (burst_flag);
		}

		/*  Time Non-critical host application code goes here */


		if ((GET_SENSOR_STATE(BOARD_LEFT_KEY_ID) != 0)
		&& (lft_pressed == 0)) {
			lft_pressed = 1;
			if(ul_duty > INIT_DUTY_VALUE) ul_duty--;
			printf("  duty cicle = %lu \r\n",ul_duty*100/PERIOD_VALUE);
			} else {
			if ((GET_SENSOR_STATE(BOARD_LEFT_KEY_ID) == 0)
			&& (lft_pressed == 1)) {
				lft_pressed = 0;
			}
		}
		if ((GET_SENSOR_STATE(BOARD_RIGHT_KEY_ID) != 0)
		&& (rgt_pressed == 0)) {
			rgt_pressed = 1;
			if(ul_duty < PERIOD_VALUE) ul_duty++;
			printf("  duty cicle = %lu \r\n",ul_duty*100/PERIOD_VALUE);
			} else {
			if ((GET_SENSOR_STATE(BOARD_RIGHT_KEY_ID) == 0)
			&& (rgt_pressed == 1)) {
				rgt_pressed = 0;
			}
		}


		if (GET_ROTOR_SLIDER_POSITION(0) != old_position) {
			old_position = GET_ROTOR_SLIDER_POSITION(0);
			if (motor_run==0) flag_hab_m = 1;
			ul_duty = old_position*PERIOD_VALUE/255;
		}
	}
}