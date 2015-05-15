#ifndef LCD_AUX_H
#define LCD_AUX_H

// include
#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "stdio.h"

// variaveis auxiliares


// prototipes
// tolcd - inteiro que deseja escrever no lcd
// ul_x - posição em x no lcd
// ul_y - posição em y no lcd
void escreve_int_lcd(const char *stolcd, uint32_t itolcd, uint32_t ul_x, uint32_t ul_y);

void configure_console(void);

void configure_lcd(void);


#endif /* LCD_AUX_H_INCLUDED */