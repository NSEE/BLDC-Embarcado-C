#include "lcd_aux.h"

void escreve_int_lcd(const char *stolcd, uint32_t itolcd, uint32_t ul_x, uint32_t ul_y)
{
	char buffer[20];
	
	ili9225_set_foreground_color(COLOR_WHITE);
	ili9225_draw_filled_rectangle(ul_x, ul_y, ILI9225_LCD_WIDTH, (ul_y+18));
	sprintf(buffer, "%s %u", stolcd, (uint) itolcd);
	ili9225_set_foreground_color(COLOR_BLACK);
	ili9225_draw_string(ul_x, ul_y, (uint8_t *)buffer);
}

/*  Configure UART console. */
void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY
	};
	
	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

void configure_lcd(void)
{
	static struct ili9225_opt_t g_ili9225_display_opt;
	
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
}
