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
 * Implementation of ILI9225 driver.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "board.h"

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/* LCD Register Select pin definition */
const Pin pinLcdRs = PIN_LCD_RS;

/* Pixel cache used to speed up SPI communication */
#define LCD_DATA_CACHE_SIZE BOARD_LCD_WIDTH
static LcdColor_t gLcdPixelCache[LCD_DATA_CACHE_SIZE];

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/* SPI timing configuration for ILI9225 */
#define ILI_CSR \
    SPI_CSR_NCPHA |\
    SPI_DLYBS(BOARD_LCD_DLYBS, BOARD_MCK) |\
    SPI_DLYBCT(BOARD_LCD_DLYBCT, BOARD_MCK) |\
    SPI_SCBR(BOARD_LCD_SPCK, BOARD_MCK)

#define CSR_8_BIT  ILI_CSR | SPI_CSR_BITS_8_BIT
#define CSR_16_BIT ILI_CSR | SPI_CSR_BITS_16_BIT

/**
 * \brief Send command to LCD controller.
 *
 * \param cmd   command.
 */
static void WriteCmd(uint8_t cmd)
{
    /* Configure Spi Chip select: SPI Mode 0, 8bits */
    SPIM_ConfigureCS(SPI, BOARD_LCD_NPCS, CSR_8_BIT);

    /* Transfer data */
    SPIM_LockCS(SPI, BOARD_LCD_NPCS);
    PIO_Clear(&pinLcdRs);
    SPIM_TransferData(SPI, cmd, NULL);
    SPIM_ReleaseCS(SPI);

    /* Back to the default config: SPI Mode 0, 16bits */
    SPIM_ConfigureCS(SPI, BOARD_LCD_NPCS, CSR_16_BIT);
}

/**
 * \brief Send one data to LCD controller.
 *
 * \param data   data.
 */

static void WriteData(uint16_t data)
{
    SPIM_LockCS(SPI, BOARD_LCD_NPCS);
    PIO_Set(&pinLcdRs);
    SPIM_TransferData(SPI, data, NULL);
    SPIM_ReleaseCS(SPI);

}

/**
 * \brief Write mutiple data in buffer to LCD controller.
 *
 * \param pBuf  data buffer.
 * \param size  size in pixels.
 */
static void WriteBuffer(const LcdColor_t *pBuf, uint32_t size)
{
    if (size == 0) return;
    SPIM_LockCS(SPI, BOARD_LCD_NPCS);
    PIO_Set(&pinLcdRs);
    SPIM_TransferBuffer(SPI, pBuf, NULL, size);
    SPIM_ReleaseCS(SPI);
}

/**
 * \brief Write data to LCD Register.
 *
 * \param reg   Register address.
 * \param data  Data to be written.
 */
static void WriteReg(uint8_t reg, LcdColor_t data)
{
    WriteCmd(reg);
    WriteData(data);
}

/*----------------------------------------------------------------------------
 *        Basic ILI9225 primitives
 *----------------------------------------------------------------------------*/


/**
 * \brief Check Box coordinates. Return upper left and bottom right coordinates.
 *
 * \param pX1      X-coordinate of upper-left corner on LCD.
 * \param pY1      Y-coordinate of upper-left corner on LCD.
 * \param pX2      X-coordinate of lower-right corner on LCD.
 * \param pY2      Y-coordinate of lower-right corner on LCD.
 */
static void CheckBoxCoordinates( uint32_t *pX1, uint32_t *pY1, uint32_t *pX2, uint32_t *pY2 )
{
    uint32_t dw;

    if ( *pX1 >= BOARD_LCD_WIDTH )
        *pX1=BOARD_LCD_WIDTH-1 ;

    if ( *pX2 >= BOARD_LCD_WIDTH )
        *pX2=BOARD_LCD_WIDTH-1 ;

    if ( *pY1 >= BOARD_LCD_HEIGHT )
        *pY1=BOARD_LCD_HEIGHT-1 ;

    if ( *pY2 >= BOARD_LCD_HEIGHT )
        *pY2=BOARD_LCD_HEIGHT-1 ;

    if (*pX1 > *pX2) {
        dw = *pX1;
        *pX1 = *pX2;
        *pX2 = dw;
    }
    if (*pY1 > *pY2) {
        dw = *pY1;
        *pY1 = *pY2;
        *pY2 = dw;
    }
}

/**
 * \brief Initialize the LCD controller.
 */
extern uint32_t LCD_Initialize(void)
{
    const Pin pinLcdRst = PIN_LCD_RSTN;
    volatile unsigned int i;

    /* Initialize LCD Pins */
    PIO_Configure(&pinLcdRst, 1);
    PIO_Configure(&pinLcdRs, 1);

    /* Reset LCD module */
    PIO_Set(&pinLcdRst);
    i = 2000 * (BOARD_MCK / 1000000);    /* wait for at least 2ms */
    while(i--);

    PIO_Clear(&pinLcdRst);
    i = 20000 * (BOARD_MCK / 1000000);    /* wait for at least 20ms */
    while(i--);

    PIO_Set(&pinLcdRst);
    i = 50000 * (BOARD_MCK / 1000000);    /* wait for at least 50ms */
    while(i--);


    /* Initialize SPI driver interface for LCD */
    SPIM_ConfigureCS(SPI, BOARD_LCD_NPCS, CSR_16_BIT);

    /* Turn off LCD */
    LCD_Off();

    /*======== LCD module initial code ========*/

    /* Start Initial Sequence */
    WriteReg(0x01, 0x011c);              /* Set SS ¡¢SM  GS and NL  bit */
    WriteReg(0x02, 0x0100);              /* Set 1 line inversion */
    WriteReg(0x03, 0x1030);              /* Entry Mode  set GRAM     write direction and BGR=1 */
    WriteReg(0x08, 0x0808);              /* Set BP and FP */
    WriteReg(0x0C, 0x0001);              /* RGB Input Interface Control:16-bit RGB interface */
    WriteReg(0x0F, 0x0A01);              /* set frame rate: 83Hz */
    WriteReg(0x20, BOARD_LCD_WIDTH);     /* set GRAM Address */
    WriteReg(0x21, BOARD_LCD_HEIGHT);    /* set GRAM Address */

    /* power on sequence */
    WriteReg(0x10, 0x0A00);              /* set asp DSTB,STB */
    WriteReg(0x11, 0x1038);              /* SET APON PON AON VCI1EN VC */
    i = 50000 * (BOARD_MCK / 1000000);    /* wait for at least 50ms */
    while(i--);


    WriteReg(0x12, 0x1121);              /* INTERNAL REFERENCE VOLTATE =VCI */
    WriteReg(0x13, 0x06CE);              /* SET GVDD */
    WriteReg(0x14, 0x676F);              /* SET VCOMH/VCOML VOLTAGE */

    /* SET GRAM AREA */
    WriteReg(0x30, 0x0000);
    WriteReg(0x31, 0x00DB);
    WriteReg(0x32, 0x0000);
    WriteReg(0x33, 0x0000);
    WriteReg(0x34, 0x00DB);
    WriteReg(0x35, 0x0000);
    WriteReg(0x36, BOARD_LCD_WIDTH);
    WriteReg(0x37, 0x0000);
    WriteReg(0x38, BOARD_LCD_HEIGHT);
    WriteReg(0x39, 0x0000);

    /* Set GAMMA CRUVE */
    WriteReg(0x50, 0x0000);
    WriteReg(0x51, 0x060A);
    WriteReg(0x52, 0x0D0A);
    WriteReg(0x53, 0x0303);
    WriteReg(0x54, 0x0A0D);
    WriteReg(0x55, 0x0A06);
    WriteReg(0x56, 0x0000);
    WriteReg(0x57, 0x0303);
    WriteReg(0x58, 0x0000);
    WriteReg(0x59, 0x0000);

    return 0;
}

/**
 * \brief Turn on the LCD.
 */
extern void LCD_On(void)
{
    WriteReg(0x07, 0x1017);
}

/**
 * \brief Turn off the LCD.
 */
extern void LCD_Off(void)
{
    WriteReg(0x07, 0x0000);
}

/**
 * \brief Convert 24 bit RGB color into 5-6-5 rgb color space.
 *
 * Initialize the LcdColor_t cache with the color pattern.
 * \param x  24-bits RGB color.
 * \return 0 for successfull operation.
 */
extern uint32_t LCD_SetColor(uint32_t dwRgb24Bits)
{
    uint16_t i;
    LcdColor_t wColor;

    wColor = (dwRgb24Bits & 0xF80000) >> 8 |
             (dwRgb24Bits & 0x00FC00) >> 5 |
             (dwRgb24Bits & 0x0000F8) >> 3;

    /* Fill the cache with selected color */
    for (i = 0; i < LCD_DATA_CACHE_SIZE; ++i) {
        gLcdPixelCache[i] = wColor;
    }
    return 0;
}

/**
 * \brief Draw a LcdColor_t on LCD of given color.
 *
 * \param x  X-coordinate of pixel.
 * \param y  Y-coordinate of pixel.
 */
extern uint32_t LCD_DrawPixel(
    uint32_t x,
    uint32_t y)
{
    if ((x >= BOARD_LCD_WIDTH) || (y >= BOARD_LCD_HEIGHT)) {
        return 1;
    }

    /* Set cursor */
    WriteReg(0x20, x); /* column */
    WriteReg(0x21, y); /* row */

    /* Prepare to write in GRAM */
    WriteCmd(0x22);

    WriteData(*gLcdPixelCache);

    return 0;
}

/**
 * \brief Write several pixels with the same color to LCD GRAM.
 *
 * LcdColor_t color is set by the LCD_SetColor() function.
 * This function is optimized using an sram buffer to transfer block instead of
 * individual pixels in order to limit the number of SPI interrupts.
 * \param dwX1      X-coordinate of upper-left corner on LCD.
 * \param dwY1      Y-coordinate of upper-left corner on LCD.
 * \param dwX2      X-coordinate of lower-right corner on LCD.
 * \param dwY2      Y-coordinate of lower-right corner on LCD.
 */
extern uint32_t LCD_DrawFilledRectangle( uint32_t dwX1, uint32_t dwY1, uint32_t dwX2, uint32_t dwY2 )
{
    uint32_t size, blocks;

    /* Swap coordinates if necessary */
    CheckBoxCoordinates(&dwX1, &dwY1, &dwX2, &dwY2);

    /* Determine the refresh window area */
    WriteReg( 0x36, (uint16_t)dwX2 ) ;
    WriteReg( 0x37, (uint16_t)dwX1 ) ;
    WriteReg( 0x38, (uint16_t)dwY2 ) ;
    WriteReg( 0x39, (uint16_t)dwY1 ) ;

    /* Set cursor */
    WriteReg(0x20, dwX1); /* column */
    WriteReg(0x21, dwY1); /* row */

    /* Prepare to write in GRAM */
    WriteCmd(0x22);

    size = (dwX2 - dwX1 + 1) * (dwY2 - dwY1 + 1);
    /* Send pixels blocks => one SPI IT / block */
    blocks = size / LCD_DATA_CACHE_SIZE;
    while (blocks--) {
        WriteBuffer(gLcdPixelCache, LCD_DATA_CACHE_SIZE);
    }
    /* Send remaining pixels */
    WriteBuffer(gLcdPixelCache, size % LCD_DATA_CACHE_SIZE);

    /* Reset the refresh window area */
    WriteReg( 0x36, (uint16_t)BOARD_LCD_WIDTH - 1 ) ;
    WriteReg( 0x37, (uint16_t)0 ) ;
    WriteReg( 0x38, (uint16_t)BOARD_LCD_HEIGHT - 1 ) ;
    WriteReg( 0x39, (uint16_t)0 ) ;

    return 0 ;
}

/**
 * \brief Write several pixels pre-formatted in a bufer to LCD GRAM.
 *
 * \param dwX1      X-coordinate of upper-left corner on LCD.
 * \param dwY1      Y-coordinate of upper-left corner on LCD.
 * \param dwX2      X-coordinate of lower-right corner on LCD.
 * \param dwY2      Y-coordinate of lower-right corner on LCD.
 * \param pBuffer   LcdColor_t buffer area.
 */
extern uint32_t LCD_DrawPicture( uint32_t dwX1, uint32_t dwY1, uint32_t dwX2, uint32_t dwY2, const LcdColor_t *pBuffer )
{
    uint32_t size, blocks;
    LcdColor_t currentColor;

    /* Swap coordinates if necessary */
    CheckBoxCoordinates(&dwX1, &dwY1, &dwX2, &dwY2);

    /* Determine the refresh window area */
    WriteReg( 0x36, (uint16_t)dwX2 ) ;
    WriteReg( 0x37, (uint16_t)dwX1 ) ;
    WriteReg( 0x38, (uint16_t)dwY2 ) ;
    WriteReg( 0x39, (uint16_t)dwY1 ) ;

    /* Set cursor */
    WriteReg(0x20, dwX1); /* column */
    WriteReg(0x21, dwY1); /* row */

    /* Prepare to write in GRAM */
    WriteCmd(0x22);

    size = (dwX2 - dwX1 + 1) * (dwY2 - dwY1 + 1);
    /* Check if the buffer is within the SRAM */
    if ((IRAM_ADDR <= (uint32_t)pBuffer) && ((uint32_t)pBuffer < (IRAM_ADDR+IRAM_SIZE))) {
        WriteBuffer(pBuffer, size);
    }
    /* If the buffer is not in SRAM, transfer it in SRAM first before transfer */
    else {
        /* Use color buffer as a cache */
        currentColor = gLcdPixelCache[0];
        /* Send pixels blocks => one SPI IT / block */
        blocks = size / LCD_DATA_CACHE_SIZE;
        while (blocks--) {
            memcpy(gLcdPixelCache, pBuffer, LCD_DATA_CACHE_SIZE * sizeof(LcdColor_t));
            WriteBuffer(gLcdPixelCache, LCD_DATA_CACHE_SIZE);
            pBuffer += LCD_DATA_CACHE_SIZE;
        }
        /* Send remaining pixels */
        memcpy(gLcdPixelCache, pBuffer, (size % LCD_DATA_CACHE_SIZE) * sizeof(LcdColor_t));
        WriteBuffer(gLcdPixelCache, size % LCD_DATA_CACHE_SIZE);

        /* Restore the color cache */
        LCD_SetColor(currentColor);
    }

    /* Reset the refresh window area */
    WriteReg( 0x36, (uint16_t)BOARD_LCD_WIDTH - 1 ) ;
    WriteReg( 0x37, (uint16_t)0 ) ;
    WriteReg( 0x38, (uint16_t)BOARD_LCD_HEIGHT - 1 ) ;
    WriteReg( 0x39, (uint16_t)0 ) ;

    return 0 ;
}

/*
 * \brief Draw a line on LCD, which is not horizontal or vertical.
 *
 * \param x         X-coordinate of line start.
 * \param y         Y-coordinate of line start.
 * \param length    line length.
 * \param direction line direction: 0 - horizontal, 1 - vertical.
 * \param color     LcdColor_t color.
 */
static uint32_t DrawLineBresenham( uint32_t dwX1, uint32_t dwY1, uint32_t dwX2, uint32_t dwY2 )
{
    int dx, dy ;
    int i ;
    int xinc, yinc, cumul ;
    int x, y ;

    x = dwX1 ;
    y = dwY1 ;
    dx = dwX2 - dwX1 ;
    dy = dwY2 - dwY1 ;

    xinc = ( dx > 0 ) ? 1 : -1 ;
    yinc = ( dy > 0 ) ? 1 : -1 ;
    dx = ( dx > 0 ) ? dx : -dx ;
    dy = ( dy > 0 ) ? dy : -dy ;

    LCD_DrawPixel( x, y ) ;

    if ( dx > dy )
    {
      cumul = dx / 2 ;
      for ( i = 1 ; i <= dx ; i++ )
      {
        x += xinc ;
        cumul += dy ;

        if ( cumul >= dx )
        {
          cumul -= dx ;
          y += yinc ;
        }
        LCD_DrawPixel( x, y ) ;
      }
    }
    else
    {
        cumul = dy / 2 ;
        for ( i = 1 ; i <= dy ; i++ )
        {
            y += yinc ;
            cumul += dx ;

            if ( cumul >= dy )
            {
                cumul -= dy ;
                x += xinc ;
            }

            LCD_DrawPixel( x, y ) ;
        }
    }

    return 0 ;
}


/*
 * \brief Draw a line on LCD, horizontal and vertical line are supported.
 *
 * \param dwX1      X-coordinate of line start.
 * \param dwY1      Y-coordinate of line start.
 * \param dwX2      X-coordinate of line end.
 * \param dwY2      Y-coordinate of line end.
  */
extern uint32_t LCD_DrawLine ( uint32_t dwX1, uint32_t dwY1, uint32_t dwX2, uint32_t dwY2 )
{
    /* Optimize horizontal or vertical line drawing */
    if (( dwY1 == dwY2 ) || (dwX1 == dwX2)) {
        LCD_DrawFilledRectangle( dwX1, dwY1, dwX2, dwY2 );
    }
    else {
        DrawLineBresenham( dwX1, dwY1, dwX2, dwY2 ) ;
    }

    return 0 ;
}

/**
 * \brief Draws a circle on LCD, at the given coordinates.
 *
 * \param dwX      X-coordinate of circle center.
 * \param dwY      Y-coordinate of circle center.
 * \param dwR      circle radius.
*/
extern uint32_t LCD_DrawCircle(
        uint32_t dwX,
        uint32_t dwY,
        uint32_t dwR)
{
    int32_t   d;    /* Decision Variable */
    uint32_t  curX; /* Current X Value */
    uint32_t  curY; /* Current Y Value */

    if (dwR == 0)
        return 0;
    d = 3 - (dwR << 1);
    curX = 0;
    curY = dwR;

    while (curX <= curY)
    {
        LCD_DrawPixel(dwX + curX, dwY + curY);
        LCD_DrawPixel(dwX + curX, dwY - curY);
        LCD_DrawPixel(dwX - curX, dwY + curY);
        LCD_DrawPixel(dwX - curX, dwY - curY);
        LCD_DrawPixel(dwX + curY, dwY + curX);
        LCD_DrawPixel(dwX + curY, dwY - curX);
        LCD_DrawPixel(dwX - curY, dwY + curX);
        LCD_DrawPixel(dwX - curY, dwY - curX);

        if (d < 0) {
            d += (curX << 2) + 6;
        }
        else {
            d += ((curX - curY) << 2) + 10;
            curY--;
        }
        curX++;
    }
    return 0;
}

extern uint32_t LCD_DrawFilledCircle( uint32_t dwX, uint32_t dwY, uint32_t dwRadius)
{
    signed int d ; // Decision Variable
    uint32_t dwCurX ; // Current X Value
    uint32_t dwCurY ; // Current Y Value
    uint32_t dwXmin, dwYmin;

    if (dwRadius == 0)
        return 0;
    d = 3 - (dwRadius << 1) ;
    dwCurX = 0 ;
    dwCurY = dwRadius ;

    while ( dwCurX <= dwCurY )
    {
        dwXmin = (dwCurX > dwX) ? 0 : dwX-dwCurX;
        dwYmin = (dwCurY > dwY) ? 0 : dwY-dwCurY;
        LCD_DrawFilledRectangle( dwXmin, dwYmin, dwX+dwCurX, dwYmin ) ;
        LCD_DrawFilledRectangle( dwXmin, dwY+dwCurY, dwX+dwCurX, dwY+dwCurY ) ;
        dwXmin = (dwCurY > dwX) ? 0 : dwX-dwCurY;
        dwYmin = (dwCurX > dwY) ? 0 : dwY-dwCurX;
        LCD_DrawFilledRectangle( dwXmin, dwYmin, dwX+dwCurY, dwYmin ) ;
        LCD_DrawFilledRectangle( dwXmin, dwY+dwCurX, dwX+dwCurY, dwY+dwCurX ) ;

        if ( d < 0 )
        {
            d += (dwCurX << 2) + 6 ;
        }
        else
        {
            d += ((dwCurX - dwCurY) << 2) + 10;
            dwCurY-- ;
        }

        dwCurX++ ;
    }

    return 0 ;
}
extern uint32_t LCD_DrawRectangle( uint32_t dwX1, uint32_t dwY1, uint32_t dwX2, uint32_t dwY2 )
{
    CheckBoxCoordinates(&dwX1, &dwY1, &dwX2, &dwY2);

    LCD_DrawFilledRectangle( dwX1, dwY1, dwX2, dwY1 ) ;
    LCD_DrawFilledRectangle( dwX1, dwY2, dwX2, dwY2 ) ;

    LCD_DrawFilledRectangle( dwX1, dwY1, dwX1, dwY2 ) ;
    LCD_DrawFilledRectangle( dwX2, dwY1, dwX2, dwY2 ) ;

    return 0 ;
}


/**
 * \brief Set the backlight of the LCD (AAT3193).
 *
 * \param level   Backlight brightness level [1..16], 1 means maximum brightness.
 */
extern void LCD_SetBacklight (uint32_t level)
{
    uint32_t i;
    const Pin pPins[] = {PIN_LCD_BACKLIGTH};

    /* Ensure valid level */
    level = (level < 1) ? 1 : level;
    level = (level > 16) ? 16 : level;

    /* Enable pins */
    PIO_Configure(pPins, PIO_LISTSIZE(pPins));

    /* Switch off backlight */
    PIO_Clear(pPins);
    i = 600 * (BOARD_MCK / 1000000);    /* wait for at least 500us */
    while(i--);

    /* Set new backlight level */
    for (i = 0; i < level; i++) {
        PIO_Clear(pPins);
        PIO_Clear(pPins);
        PIO_Clear(pPins);

        PIO_Set(pPins);
        PIO_Set(pPins);
        PIO_Set(pPins);
    }
}
