#include "lcd_aux.h"

void escreve_int_lcd(const char *stolcd, uint32_t itolcd, uint32_t ul_x, uint32_t ul_y)
{
	char buffer[20];
	
	ili9225_set_foreground_color(COLOR_WHITE);
	ili9225_draw_filled_rectangle(ul_x, ul_y, ILI9225_LCD_WIDTH, (ul_y+20));
	sprintf(buffer, "%s %u", stolcd, (uint) itolcd);
	ili9225_set_foreground_color(COLOR_BLACK);
	ili9225_draw_string(ul_x, ul_y, (uint8_t *)buffer);
}
