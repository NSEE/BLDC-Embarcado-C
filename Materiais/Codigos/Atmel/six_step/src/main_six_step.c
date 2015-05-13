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

struct ili9225_opt_t g_ili9225_display_opt;

uint32_t pos_lcd_x;
uint32_t pos_lcd_y;
const char *pre_lcd;

uint32_t hall_1 = 0;
uint32_t hall_2 = 0;
uint32_t hall_3 = 0;

uint8_t phase_lcd = 0;

static uint32_t bt1 = 0;
static uint32_t bt2 = 0;

///** PWM frequency in Hz */
//#define PWM_FREQUENCY      1000
///** Period value of PWM output waveform */
//#define PERIOD_VALUE       10
///** Initial duty cycle value */
//#define INIT_DUTY_VALUE    0

#define IRQ_PRIOR_PIO    0



///** PWM channel instance for LEDs */
//pwm_channel_t g_pwm_channel_led;
//
////static uint8_t fade_in = 1;  /* LED fade in flag */
//static uint32_t ul_duty = INIT_DUTY_VALUE;  /* PWM duty cycle rate */

/**
 * \brief Override SPI handler.
 */
void SPI_Handler(void)
{
	ili9225_spi_handler();
}

/**
 *  Configure UART console.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY
	};
	
	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 * \brief Interrupt handler for the PWM controller.
 */
//void PWM_Handler(void)
//{
	//static uint32_t ul_count = 0;  /* PWM counter value */
	//
	//
	//uint32_t events = pwm_channel_get_interrupt_status(PWM);
//
	///* Interrupt on PIN_PWM_LED0_CHANNEL */
	//if ((events & (1 << PIN_PWM_LED0_CHANNEL)) ==
			//(1 << PIN_PWM_LED0_CHANNEL)) {
		//ul_count++;
//
		//if (ul_count == (PWM_FREQUENCY / (PERIOD_VALUE - INIT_DUTY_VALUE))) {
//
			///* Set new duty cycle */
			//ul_count = 0;
			//g_pwm_channel_led.channel = PIN_PWM_LED0_CHANNEL;
			//pwm_channel_update_duty(PWM, &g_pwm_channel_led, ul_duty);
			//g_pwm_channel_led.channel = PIN_PWM_LED1_CHANNEL;
			//pwm_channel_update_duty(PWM, &g_pwm_channel_led, ul_duty);
		//}
	//}
//}

static void Button1_Handler(uint32_t id, uint32_t mask)
{
	/*Botão 1 aumenta o duty cicle (ul_duty)*/
	
	if (PIN_PUSHBUTTON_1_ID == id && PIN_PUSHBUTTON_1_MASK == mask) {

//		if(ul_duty < PERIOD_VALUE) {
//			ul_duty++;

			bt1++;
			
//		}
	}

}

static void Button2_Handler(uint32_t id, uint32_t mask)
{
	/*Botão 2 diminui o duty cicle (ul_duty)*/

	if (PIN_PUSHBUTTON_2_ID == id && PIN_PUSHBUTTON_2_MASK == mask) {
	
//		if(ul_duty > INIT_DUTY_VALUE){
//		ul_duty--;
		
		bt2++;
		
//		}
	
	}
}

void Hall_Phase(void)
{
	static uint8_t aux_hall_1;
	static uint8_t aux_hall_2;
	static uint8_t aux_hall_3;
	static uint8_t phase = 0;
	
	aux_hall_1 = ioport_get_pin_level(PIN_HALL_1);
	aux_hall_2 = ioport_get_pin_level(PIN_HALL_2);
	aux_hall_3 = ioport_get_pin_level(PIN_HALL_3);
	

	phase = (aux_hall_3<<2) | (aux_hall_2<<1) | (aux_hall_1);

	hall_1 = aux_hall_1;
	hall_2 = aux_hall_2;
	hall_3 = aux_hall_3;
	
	switch (phase){
	
	case 5 : phase_lcd=1; break; //phase 1
	case 1 : phase_lcd=2; break; //phase 2
	case 3 : phase_lcd=3; break; //phase 3
	case 2 : phase_lcd=4; break; //phase 4
	case 6 : phase_lcd=5; break; //phase 5
	case 4 : phase_lcd=6; break; //phase 6
	}
}

static void Hall1_Handler(uint32_t id, uint32_t mask)
{
	if (PIN_HALL_1_ID == id && PIN_HALL_1_MASK == mask)
	{
		Hall_Phase();
	}
}

static void Hall2_Handler(uint32_t id, uint32_t mask)
{
	if (PIN_HALL_2_ID == id && PIN_HALL_2_MASK == mask)
	{
		Hall_Phase();					
	}	
}

static void Hall3_Handler(uint32_t id, uint32_t mask)
{
	if (PIN_HALL_3_ID == id && PIN_HALL_3_MASK == mask)
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


static void configure_buttons(void)
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

static void configure_hall (void)
{
	//Sensor Hall 1
	pmc_enable_periph_clk(PIN_HALL_1_ID);
	pio_handler_set(PIN_HALL_1_PIO, PIN_HALL_1_ID, PIN_HALL_1_MASK, 0, Hall1_Handler);
	NVIC_EnableIRQ((IRQn_Type) PIN_HALL_1_ID);
	pio_handler_set_priority(PIN_HALL_1_PIO, (IRQn_Type) PIN_HALL_1_ID, IRQ_PRIOR_PIO);
	pio_enable_interrupt(PIN_HALL_1_PIO, PIN_HALL_1_MASK);
	
	//Sensor Hall 2
	pmc_enable_periph_clk(PIN_HALL_2_ID);
	pio_handler_set(PIN_HALL_2_PIO, PIN_HALL_2_ID, PIN_HALL_2_MASK, 0, Hall2_Handler);
	NVIC_EnableIRQ((IRQn_Type) PIN_HALL_2_ID);
	pio_handler_set_priority(PIN_HALL_2_PIO, (IRQn_Type) PIN_HALL_2_ID, IRQ_PRIOR_PIO);
	pio_enable_interrupt(PIN_HALL_2_PIO, PIN_HALL_2_MASK);
	
	//Sensor Hall 3
	pmc_enable_periph_clk(PIN_HALL_3_ID);
	pio_handler_set(PIN_HALL_3_PIO, PIN_HALL_3_ID, PIN_HALL_3_MASK, 0, Hall3_Handler);
	NVIC_EnableIRQ((IRQn_Type) PIN_HALL_3_ID);
	pio_handler_set_priority(PIN_HALL_3_PIO, (IRQn_Type) PIN_HALL_3_ID, IRQ_PRIOR_PIO);
	pio_enable_interrupt(PIN_HALL_3_PIO, PIN_HALL_3_MASK);
}

int main(void)
{
	sysclk_init();
	board_init();
	configure_buttons();
	configure_hall();
	configure_console();
	

	/* Initialize display parameter */
	g_ili9225_display_opt.ul_width = ILI9225_LCD_WIDTH;
	g_ili9225_display_opt.ul_height = ILI9225_LCD_HEIGHT;
	g_ili9225_display_opt.foreground_color = COLOR_BLACK;
	g_ili9225_display_opt.background_color = COLOR_WHITE;

	/* Switch off backlight */
	aat31xx_disable_backlight();

	/* Initialize LCD */
	ili9225_init(&g_ili9225_display_opt);

	/* Set backlight level */
	aat31xx_set_backlight(AAT31XX_AVG_BACKLIGHT_LEVEL);

	/* Turn on LCD */
	ili9225_display_on();

	/* Draw filled rectangle with white color */
	ili9225_set_foreground_color(COLOR_WHITE);
	ili9225_draw_filled_rectangle(0, 0, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
		
	/* Draw text and basic shapes on the LCD */
	ili9225_set_foreground_color(COLOR_BLACK);
	ili9225_draw_string(40, 20, (uint8_t *)"six-step");
	pos_lcd_x = 20;
	
	//gpio_configure_pin(PIO_PC21_IDX, (PIO_PERIPH_B | PIO_DEFAULT));  // PIn PC21
	//gpio_configure_pin(PIN_PWM_LED0_GPIO, PIN_PWM_LED0_FLAGS);
	
	///* Enable PWM peripheral clock */
	//pmc_enable_periph_clk(ID_PWM);
//
	///* Disable PWM channels for LEDs */
	//pwm_channel_disable(PWM, PIN_PWM_LED0_CHANNEL); // channel 0
	//pwm_channel_disable(PWM, PIN_PWM_LED1_CHANNEL); // channel 3
//
	///* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	//pwm_clock_t clock_setting = {
		//.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		//.ul_clkb = 0,
		//.ul_mck = sysclk_get_cpu_hz()
	//};
	//pwm_init(PWM, &clock_setting);
//
	///* Initialize PWM channel for LED0 */
	///* Period is left-aligned */
	//g_pwm_channel_led.alignment = PWM_ALIGN_LEFT;
	///* Output waveform starts at a low level */
	//g_pwm_channel_led.polarity = PWM_LOW;
	///* Use PWM clock A as source clock */
	//g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	///* Period value of output waveform */
	//g_pwm_channel_led.ul_period = PERIOD_VALUE;
	///* Duty cycle value of output waveform */
	//g_pwm_channel_led.ul_duty = INIT_DUTY_VALUE;
	//g_pwm_channel_led.channel = PIN_PWM_LED0_CHANNEL;
	//pwm_channel_init(PWM, &g_pwm_channel_led);
//
	///* Enable channel counter event interrupt */
	//pwm_channel_enable_interrupt(PWM, PIN_PWM_LED0_CHANNEL, 0);
//
	///* Initialize PWM channel for LED1 */
	///* Period is center-aligned */
	//g_pwm_channel_led.alignment = PWM_ALIGN_CENTER;
	///* Output waveform starts at a high level */
	//g_pwm_channel_led.polarity = PWM_HIGH;
	///* Use PWM clock A as source clock */
	//g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	///* Period value of output waveform */
	//g_pwm_channel_led.ul_period = PERIOD_VALUE;
	///* Duty cycle value of output waveform */
	//g_pwm_channel_led.ul_duty = INIT_DUTY_VALUE;
	//g_pwm_channel_led.channel = PIN_PWM_LED1_CHANNEL;
	//
	//pwm_channel_init(PWM, &g_pwm_channel_led);
//
	///* Disable channel counter event interrupt */
	//pwm_channel_disable_interrupt(PWM, PIN_PWM_LED1_CHANNEL, 0);
//
	///* Configure interrupt and enable PWM interrupt */
	//NVIC_DisableIRQ(PWM_IRQn);
	//NVIC_ClearPendingIRQ(PWM_IRQn);
	//NVIC_SetPriority(PWM_IRQn, 0);
	//NVIC_EnableIRQ(PWM_IRQn);
//
	///* Enable PWM channels for LEDs */
	//pwm_channel_enable(PWM, PIN_PWM_LED0_CHANNEL);
	//pwm_channel_enable(PWM, PIN_PWM_LED1_CHANNEL);

	/* Infinite loop */
	while (1) {
		escreve_int_lcd("bt1 = ", bt1, pos_lcd_x, 40);
		escreve_int_lcd("bt2 = ", bt2, pos_lcd_x, 65);
		escreve_int_lcd("hall1 = ", hall_1, pos_lcd_x, 90);
		escreve_int_lcd("hall2 = ", hall_2, pos_lcd_x, 115);
		escreve_int_lcd("hall3 = ", hall_3, pos_lcd_x, 140);
		escreve_int_lcd("phase = ", phase_lcd, pos_lcd_x, 165);
		delay_ms(500);
		
	}
}
