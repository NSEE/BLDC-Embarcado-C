#include "six_step.h"

void configure_buttons(void)
{
	pmc_enable_periph_clk(PIN_PUSHBUTTON_1_ID);
	pio_set_debounce_filter(PIN_PUSHBUTTON_1_PIO, PIN_PUSHBUTTON_1_MASK, 10);
	pio_handler_set(PIN_PUSHBUTTON_1_PIO, PIN_PUSHBUTTON_1_ID, PIN_PUSHBUTTON_1_MASK, PIN_PUSHBUTTON_1_ATTR, Button1_Handler);
	NVIC_EnableIRQ((IRQn_Type) PIN_PUSHBUTTON_1_ID);
	pio_handler_set_priority(PIN_PUSHBUTTON_1_PIO, (IRQn_Type) PIN_PUSHBUTTON_1_ID, IRQ_PRIOR_PIO);
	pio_enable_interrupt(PIN_PUSHBUTTON_1_PIO, PIN_PUSHBUTTON_1_MASK);
	
	pmc_enable_periph_clk(PIN_PUSHBUTTON_2_ID);
	pio_set_debounce_filter(PIN_PUSHBUTTON_2_PIO, PIN_PUSHBUTTON_2_MASK, 10);
	pio_handler_set(PIN_PUSHBUTTON_2_PIO, PIN_PUSHBUTTON_2_ID, PIN_PUSHBUTTON_2_MASK, PIN_PUSHBUTTON_2_ATTR, Button2_Handler);
	NVIC_EnableIRQ((IRQn_Type) PIN_PUSHBUTTON_2_ID);
	pio_handler_set_priority(PIN_PUSHBUTTON_2_PIO, (IRQn_Type) PIN_PUSHBUTTON_2_ID, IRQ_PRIOR_PIO);
	pio_enable_interrupt(PIN_PUSHBUTTON_2_PIO, PIN_PUSHBUTTON_2_MASK);
}

void configure_hall(void)
{
	//Sensor Hall 1
	pmc_enable_periph_clk(PIN_HALL_1_ID);
	pio_handler_set(PIN_HALL_1_PIO, PIN_HALL_1_ID, PIN_HALL_1_MASK, 0, Hall_Handler);
	NVIC_EnableIRQ((IRQn_Type) PIN_HALL_1_ID);
	pio_handler_set_priority(PIN_HALL_1_PIO, (IRQn_Type) PIN_HALL_1_ID, IRQ_PRIOR_PIO);
	pio_enable_interrupt(PIN_HALL_1_PIO, PIN_HALL_1_MASK);
	
	//Sensor Hall 2
	pmc_enable_periph_clk(PIN_HALL_2_ID);
	pio_handler_set(PIN_HALL_2_PIO, PIN_HALL_2_ID, PIN_HALL_2_MASK, 0, Hall_Handler);
	NVIC_EnableIRQ((IRQn_Type) PIN_HALL_2_ID);
	pio_handler_set_priority(PIN_HALL_2_PIO, (IRQn_Type) PIN_HALL_2_ID, IRQ_PRIOR_PIO);
	pio_enable_interrupt(PIN_HALL_2_PIO, PIN_HALL_2_MASK);
	
	//Sensor Hall 3
	pmc_enable_periph_clk(PIN_HALL_3_ID);
	pio_handler_set(PIN_HALL_3_PIO, PIN_HALL_3_ID, PIN_HALL_3_MASK, 0, Hall_Handler);
	NVIC_EnableIRQ((IRQn_Type) PIN_HALL_3_ID);
	pio_handler_set_priority(PIN_HALL_3_PIO, (IRQn_Type) PIN_HALL_3_ID, IRQ_PRIOR_PIO);
	pio_enable_interrupt(PIN_HALL_3_PIO, PIN_HALL_3_MASK);
}

pwm_channel_t configure_pwm(void)
{
	pwm_channel_t g_pwm_channel;
	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(ID_PWM);
	
	/* Disable PWM channels*/
	pwm_channel_disable(PWM, PIN_PWM_IN1_CHANNEL); // channel 0
	pwm_channel_disable(PWM, PIN_PWM_IN2_CHANNEL); // channel 1
	pwm_channel_disable(PWM, PIN_PWM_IN3_CHANNEL); // channel 2
	
	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};
	pwm_init(PWM, &clock_setting);
	
	/* Initialize PWM channel for U1-2 */
	/* Period is left-aligned */
	g_pwm_channel.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	g_pwm_channel.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	g_pwm_channel.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel.ul_duty = INIT_DUTY_VALUE;
	g_pwm_channel.channel = PIN_PWM_IN1_CHANNEL;
	pwm_channel_init(PWM, &g_pwm_channel);
	
	/* Enable channel counter event interrupt */
	pwm_channel_enable_interrupt(PWM, PIN_PWM_IN1_CHANNEL, 0);
	
	/* Initialize PWM channel for U2-3 */
	/* Period is center-aligned */
	g_pwm_channel.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a high level */
	g_pwm_channel.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	g_pwm_channel.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel.ul_duty = INIT_DUTY_VALUE;
	g_pwm_channel.channel = PIN_PWM_IN2_CHANNEL;
	
	pwm_channel_init(PWM, &g_pwm_channel);
	
	/* Disable channel counter event interrupt */
	pwm_channel_disable_interrupt(PWM, PIN_PWM_IN2_CHANNEL, 0);
	
	/* Initialize PWM channel for U3-1 */
	/* Period is center-aligned */
	g_pwm_channel.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a high level */
	g_pwm_channel.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	g_pwm_channel.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel.ul_duty = INIT_DUTY_VALUE;
	g_pwm_channel.channel = PIN_PWM_IN3_CHANNEL;
	
	pwm_channel_init(PWM, &g_pwm_channel);
	
	/* Disable channel counter event interrupt */
	pwm_channel_disable_interrupt(PWM, PIN_PWM_IN3_CHANNEL, 0);
	
	/* Configurar e habilitar interrupção do PWM*/
	//NVIC_DisableIRQ(PWM_IRQn);
	//NVIC_ClearPendingIRQ(PWM_IRQn);
	//NVIC_SetPriority(PWM_IRQn, 0);
	//NVIC_EnableIRQ(PWM_IRQn);
	
	/* Enable PWM channels for Motors */
	pwm_channel_enable(PWM, PIN_PWM_IN1_CHANNEL);
	pwm_channel_enable(PWM, PIN_PWM_IN2_CHANNEL);
	pwm_channel_enable(PWM, PIN_PWM_IN3_CHANNEL);
	
	return g_pwm_channel;
}

void start_lcd(uint32_t pos_lcd_x, uint32_t pos_lcd_y, uint32_t ul_duty, uint32_t hall_1, uint32_t hall_2, uint32_t hall_3, uint8_t phase)
{
	ili9225_set_foreground_color(COLOR_BLACK);
	ili9225_draw_string(40, 20, (uint8_t *)"six-step");
	escreve_int_lcd("dc = ", ul_duty, pos_lcd_x, 40);
	escreve_int_lcd("hall1 = ", hall_1, pos_lcd_x, 60);
	escreve_int_lcd("hall2 = ", hall_2, pos_lcd_x, 80);
	escreve_int_lcd("hall3 = ", hall_3, pos_lcd_x, 100);
	escreve_int_lcd("phase = ", phase, pos_lcd_x, 120);
}