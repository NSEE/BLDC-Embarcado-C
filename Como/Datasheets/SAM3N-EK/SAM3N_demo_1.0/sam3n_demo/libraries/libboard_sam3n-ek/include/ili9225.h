/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2010, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/**
 * \file
 *
 * Interface of ILI9225 driver.
 *
 */

#ifndef _ILI9225_
#define _ILI9225_

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

//#include <include/lcd_color.h>
typedef uint16_t LcdColor_t;
/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

extern uint32_t LCD_Initialize( void ) ;
extern void LCD_On( void ) ;
extern void LCD_Off( void ) ;
extern uint32_t LCD_SetColor(uint32_t color);
extern uint32_t LCD_DrawPixel(uint32_t x, uint32_t y);
extern uint32_t LCD_DrawLine ( uint32_t dwX1, uint32_t dwY1, uint32_t dwX2, uint32_t dwY2 );
extern uint32_t LCD_DrawCircle( uint32_t x, uint32_t y, uint32_t r );
extern uint32_t LCD_DrawFilledCircle( uint32_t dwX, uint32_t dwY, uint32_t dwRadius);
extern uint32_t LCD_DrawRectangle( uint32_t dwX1, uint32_t dwY1, uint32_t dwX2, uint32_t dwY2 );
extern uint32_t LCD_DrawFilledRectangle( uint32_t dwX1, uint32_t dwY1, uint32_t dwX2, uint32_t dwY2 );
extern uint32_t LCD_DrawPicture( uint32_t dwX1, uint32_t dwY1, uint32_t dwX2, uint32_t dwY2, const LcdColor_t *pBuffer );
extern void LCD_SetBacklight (uint32_t level);
#endif /* #ifndef ILI9225 */
