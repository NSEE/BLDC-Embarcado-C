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

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include <libfreertos.h>
#include "test_qtouch.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
extern void TSK_LcdInit( void );

#define BUTTON_RADIUS 15
#define SPRITE_WIDTH  (BUTTON_RADIUS * 2)
#define SPRITE_HEIGHT (BUTTON_RADIUS * 2)
#define SLIDER_WIDTH  (BOARD_LCD_WIDTH - 20)
#define SLIDER_Y_POSITION (BOARD_LCD_HEIGHT - 42)

static LcdColor_t gKSprite[SPRITE_WIDTH * SPRITE_HEIGHT];
static LcdColor_t gSSprite[SPRITE_WIDTH * SPRITE_HEIGHT];

extern const LcdColor_t gBackgroundPicture[];




static void DrawButton(LcdColor_t *pSprite, short delta, uint8_t threshold, uint8_t state)
{
    uint8_t radius;


    FB_SetFrameBuffer(pSprite, SPRITE_WIDTH, SPRITE_HEIGHT);

    if (delta < 0)
        delta = -delta;
    if (delta < (2 * threshold)) {
        radius = ((delta * BUTTON_RADIUS) /  threshold);
    }
    else {
        radius = 2 * BUTTON_RADIUS;
    }

    if (radius < BUTTON_RADIUS) {
        FB_SetColor(COLOR_WHITE);
        FB_DrawFilledCircle(SPRITE_WIDTH/2, SPRITE_HEIGHT/2, BUTTON_RADIUS);
        FB_SetColor((state) ? COLOR_LIGHTGREEN : COLOR_PALEBROWN);
        FB_DrawFilledCircle(SPRITE_WIDTH/2, SPRITE_HEIGHT/2, radius);
    }
    else if (radius == BUTTON_RADIUS) {
        FB_SetColor((state) ? COLOR_LIGHTGREEN : COLOR_PALEBROWN);
        FB_DrawFilledCircle(SPRITE_WIDTH/2, SPRITE_HEIGHT/2, BUTTON_RADIUS);
    }
    else if (radius < (2*BUTTON_RADIUS)) {
        FB_SetColor((state) ? COLOR_LIGHTGREEN : COLOR_PALEBROWN);
        FB_DrawFilledCircle(SPRITE_WIDTH/2, SPRITE_HEIGHT/2, BUTTON_RADIUS);
        FB_SetColor((state) ? COLOR_GREEN : COLOR_DARKBROWN);
        FB_DrawFilledCircle(SPRITE_WIDTH/2, SPRITE_HEIGHT/2, radius%BUTTON_RADIUS);
    }
    else  {
        FB_SetColor((state) ? COLOR_GREEN : COLOR_DARKBROWN);
        FB_DrawFilledCircle(SPRITE_WIDTH/2, SPRITE_HEIGHT/2, BUTTON_RADIUS);
    }
}


/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/
extern void TSK_Lcd( void *pvParameters );

extern void TSK_LcdInit( void )
{
}

/**
 * \brief Application entry point for spi_lcd example.
 *
 * \return Unused (ANSI-C compatibility).
 */
extern void TSK_Lcd( void *pvParameters )
{
    QtMsg qtMsg;
    uint16_t position = BOARD_LCD_WIDTH/2;
    uint16_t prevPosition = position;

    /* Initialize SPI driver */
    SPIM_Initialize(SPI);

    /* Initialize LCD */
    LCD_Initialize() ;
    LCD_SetBacklight(2);
    LCD_On();


    /* Draw the background */
    LCD_DrawPicture(
         0, 0,
         BOARD_LCD_WIDTH-1, BOARD_LCD_HEIGHT-1,
         gBackgroundPicture);

    /* Draw the Slider */
    LCD_SetColor(COLOR_WHITE);
    LCD_DrawFilledRectangle(
        BOARD_LCD_WIDTH/2 - SLIDER_WIDTH/2-3,                SLIDER_Y_POSITION - SPRITE_HEIGHT/2-3,
        BOARD_LCD_WIDTH/2 - SLIDER_WIDTH/2 + SLIDER_WIDTH+3, SLIDER_Y_POSITION - SPRITE_HEIGHT/2 + SPRITE_HEIGHT+3);
    LCD_SetColor(COLOR_BLACK);
    LCD_DrawRectangle(
        BOARD_LCD_WIDTH/2 - SLIDER_WIDTH/2-1,                SLIDER_Y_POSITION - SPRITE_HEIGHT/2-1,
        BOARD_LCD_WIDTH/2 - SLIDER_WIDTH/2 + SLIDER_WIDTH+1, SLIDER_Y_POSITION - SPRITE_HEIGHT/2 + SPRITE_HEIGHT+1);

    /* Draw K1 button */
    LCD_SetColor(COLOR_WHITE);
    LCD_DrawFilledCircle(BOARD_LCD_WIDTH/3, BOARD_LCD_HEIGHT/2, BUTTON_RADIUS + 6);
    LCD_SetColor(COLOR_BLACK);
    LCD_DrawCircle(BOARD_LCD_WIDTH/3, BOARD_LCD_HEIGHT/2, BUTTON_RADIUS + 3);

    /* Draw K2 button */
    LCD_SetColor(COLOR_WHITE);
    LCD_DrawFilledCircle(BOARD_LCD_WIDTH*2/3, BOARD_LCD_HEIGHT/2, BUTTON_RADIUS + 6);
    LCD_SetColor(COLOR_BLACK);
    LCD_DrawCircle(BOARD_LCD_WIDTH*2/3, BOARD_LCD_HEIGHT/2, BUTTON_RADIUS + 3);

    /* Initialize button sprites */
    memset(gSSprite, 0xFF, SPRITE_WIDTH * SPRITE_HEIGHT * sizeof(LcdColor_t));
    memset(gKSprite, 0xFF, SPRITE_WIDTH * SPRITE_HEIGHT * sizeof(LcdColor_t));
    FB_SetFrameBuffer(gKSprite, SPRITE_WIDTH, SPRITE_HEIGHT);
    FB_SetColor(COLOR_BLACK);
    FB_DrawCircle(SPRITE_WIDTH/2, SPRITE_HEIGHT/2, BUTTON_RADIUS + 3);

    while(1) {
        /* Wait for a message from the QTouch task */
        if (xQueueReceive(gMsgQueue, &qtMsg, 100/portTICK_RATE_MS) == pdFALSE)
            continue;

        /* K1: draw button in the frame buffer then update the LCD */
        DrawButton(gKSprite, qtMsg.k1_delta, qtMsg.k1_threshold, qtMsg.k1_state);
        LCD_DrawPicture(
                BOARD_LCD_WIDTH/3-SPRITE_WIDTH/2,                BOARD_LCD_HEIGHT/2-SPRITE_HEIGHT/2,
                BOARD_LCD_WIDTH/3-SPRITE_WIDTH/2+SPRITE_WIDTH-1, BOARD_LCD_HEIGHT/2-SPRITE_HEIGHT/2+SPRITE_HEIGHT-1,
                gKSprite);

        /* K2: draw button in the frame buffer then update the LCD */
        DrawButton(gKSprite, qtMsg.k2_delta, qtMsg.k2_threshold, qtMsg.k2_state);
        LCD_DrawPicture(
                BOARD_LCD_WIDTH*2/3-SPRITE_WIDTH/2,                BOARD_LCD_HEIGHT/2-SPRITE_HEIGHT/2,
                BOARD_LCD_WIDTH*2/3-SPRITE_WIDTH/2+SPRITE_WIDTH-1, BOARD_LCD_HEIGHT/2-SPRITE_HEIGHT/2+SPRITE_HEIGHT-1,
                gKSprite);
        /* Slider: compute the new position */
        position = (((SLIDER_WIDTH-(2*BUTTON_RADIUS)) * qtMsg.s1_position)/256) + BUTTON_RADIUS;
        position += (BOARD_LCD_WIDTH/2-SLIDER_WIDTH/2);
        /* Slider: erase difference */
        LCD_SetColor(COLOR_WHITE);
        if (prevPosition < position)
            LCD_DrawFilledRectangle(
                    prevPosition-SPRITE_WIDTH/2, SLIDER_Y_POSITION - SPRITE_HEIGHT/2,
                    position-SPRITE_WIDTH/2,     SLIDER_Y_POSITION - SPRITE_HEIGHT/2 + SPRITE_HEIGHT -1);
        else
            LCD_DrawFilledRectangle(
                    position-SPRITE_WIDTH/2+SPRITE_WIDTH-1,     SLIDER_Y_POSITION - SPRITE_HEIGHT/2,
                    prevPosition-SPRITE_WIDTH/2+SPRITE_WIDTH-1, SLIDER_Y_POSITION - SPRITE_HEIGHT/2 + SPRITE_HEIGHT -1);
        prevPosition = position;
        /* Slider: draw the new button */
        DrawButton(gSSprite, qtMsg.s1_delta, qtMsg.s1_threshold, qtMsg.s1_state);
        LCD_DrawPicture(
                position-SPRITE_WIDTH/2,                SLIDER_Y_POSITION - SPRITE_HEIGHT/2,
                position-SPRITE_WIDTH/2+SPRITE_WIDTH-1, SLIDER_Y_POSITION - SPRITE_HEIGHT/2 + SPRITE_HEIGHT -1,
                gSSprite);

    }

}

