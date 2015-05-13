#include "asf.h"
#include "conf_board.h"
#include "conf_clock.h"

//LED0 -> blue
//LED1 -> gren

#define IRQ_PRIOR_PIO    0

volatile bool g_b_led0_active = true;
volatile bool g_b_led1_active = true;
volatile uint32_t g_ul_ms_ticks = 0;

static void ProcessButtonEvt(uint8_t uc_button)
{
	if (uc_button == 0) {
		g_b_led0_active = !g_b_led0_active;
		if (!g_b_led0_active) {
			ioport_set_pin_level(LED0_GPIO, IOPORT_PIN_LEVEL_HIGH);
		}
	}
	else {
		g_b_led1_active = !g_b_led1_active;
		if (g_b_led1_active) {
			ioport_set_pin_level(LED1_GPIO, IOPORT_PIN_LEVEL_LOW);
			tc_start(TC0, 0);
		}
		else {
			ioport_set_pin_level(LED1_GPIO, IOPORT_PIN_LEVEL_HIGH);
			tc_stop(TC0, 0);
		}
	}
}

void SysTick_Handler(void)
{
	g_ul_ms_ticks++;
}

static void Button1_Handler(uint32_t id, uint32_t mask)
{
	if (PIN_PUSHBUTTON_1_ID == id && PIN_PUSHBUTTON_1_MASK == mask) {
		ProcessButtonEvt(0);
	}
}

static void Button2_Handler(uint32_t id, uint32_t mask)
{
	if (PIN_PUSHBUTTON_2_ID == id && PIN_PUSHBUTTON_2_MASK == mask) {
		ProcessButtonEvt(1);
	}
}

static void configure_buttons(void)
{
	pmc_enable_periph_clk(PIN_PUSHBUTTON_1_ID);
	pio_set_debounce_filter(PIN_PUSHBUTTON_1_PIO, PIN_PUSHBUTTON_1_MASK, 10);
	/* Interrupt on rising edge  */
	pio_handler_set(PIN_PUSHBUTTON_1_PIO, PIN_PUSHBUTTON_1_ID,
			PIN_PUSHBUTTON_1_MASK, PIN_PUSHBUTTON_1_ATTR, Button1_Handler);
	NVIC_EnableIRQ((IRQn_Type) PIN_PUSHBUTTON_1_ID);
	pio_handler_set_priority(PIN_PUSHBUTTON_1_PIO,
			(IRQn_Type) PIN_PUSHBUTTON_1_ID, IRQ_PRIOR_PIO);
	pio_enable_interrupt(PIN_PUSHBUTTON_1_PIO, PIN_PUSHBUTTON_1_MASK);
// [main_button1_configure]

// [main_button2_configure]
	/* Configure Pushbutton 2 */
	pmc_enable_periph_clk(PIN_PUSHBUTTON_2_ID);
	pio_set_debounce_filter(PIN_PUSHBUTTON_2_PIO, PIN_PUSHBUTTON_2_MASK, 10);
	/* Interrupt on falling edge */
	pio_handler_set(PIN_PUSHBUTTON_2_PIO, PIN_PUSHBUTTON_2_ID,
			PIN_PUSHBUTTON_2_MASK, PIN_PUSHBUTTON_2_ATTR, Button2_Handler);
	NVIC_EnableIRQ((IRQn_Type) PIN_PUSHBUTTON_2_ID);
	pio_handler_set_priority(PIN_PUSHBUTTON_2_PIO,
			(IRQn_Type) PIN_PUSHBUTTON_2_ID, IRQ_PRIOR_PIO);
	pio_enable_interrupt(PIN_PUSHBUTTON_2_PIO, PIN_PUSHBUTTON_2_MASK);
// [main_button2_configure]
}	

void TC0_Handler(void)
{
	volatile uint32_t ul_dummy;

	/* Clear status bit to acknowledge interrupt */
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Toggle LED state. */
	ioport_toggle_pin_level(LED1_GPIO);
}

static void configure_tc(void)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configure PMC */
	pmc_enable_periph_clk(ID_TC0);

	tc_find_mck_divisor(4, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC0, 0, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC0, 0, (ul_sysclk / ul_div) / 4);

	/* Configure and enable interrupt on RC compare */
	NVIC_EnableIRQ((IRQn_Type) ID_TC0);
	tc_enable_interrupt(TC0, 0, TC_IER_CPCS);
	
	if (g_b_led1_active) {
		tc_start(TC0, 0);
	}
}	
static void mdelay(uint32_t ul_dly_ticks)
{
	uint32_t ul_cur_ticks;

	ul_cur_ticks = g_ul_ms_ticks;
	while ((g_ul_ms_ticks - ul_cur_ticks) < ul_dly_ticks);
}
	
int main (void)
{
	
	sysclk_init();
	board_init();

	if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
	puts("-F- Systick configuration error\r");
	while (1);
	}
	
	configure_tc();
	configure_buttons();
	
	while (1) {
		while (!g_b_led0_active);

		if (g_b_led0_active) {
			ioport_toggle_pin_level(LED0_GPIO);
		}
		/* Wait for 500ms */
		mdelay(500);
	}

}
