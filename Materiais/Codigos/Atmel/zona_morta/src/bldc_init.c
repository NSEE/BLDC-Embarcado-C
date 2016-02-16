/*
 * bldc_init.c
 *
 * Created: 12/05/2015 15:12:41
 *  Author: Cauê
 */ 

#include "board_init.h"


void config_ios_hall(void){
		/*Configure Hall sensor*/
		gpio_configure_pin(PIO_PA2_IDX, (PIO_INPUT | PIO_PULLUP | PIO_DEBOUNCE));
		gpio_configure_pin(PIO_PA3_IDX, (PIO_INPUT | PIO_PULLUP | PIO_DEBOUNCE));
		gpio_configure_pin(PIO_PA4_IDX, (PIO_INPUT | PIO_PULLUP | PIO_DEBOUNCE));
}

void config_ios_botao(void){
	/*Configure Hall sensor*/
	gpio_configure_pin(PIO_PA2_IDX, (PIO_INPUT | PIO_PULLUP | PIO_DEBOUNCE));
	gpio_configure_pin(PIO_PA3_IDX, (PIO_INPUT | PIO_PULLUP | PIO_DEBOUNCE));
	gpio_configure_pin(PIO_PA4_IDX, (PIO_INPUT | PIO_PULLUP | PIO_DEBOUNCE));
}