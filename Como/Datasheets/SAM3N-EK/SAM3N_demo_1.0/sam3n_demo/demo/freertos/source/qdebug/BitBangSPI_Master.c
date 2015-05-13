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

/*============================ INCLUDES ======================================*/
#include "board.h"
#include "BitBangSPI_Master.h"
#include "QDebugTransport.h"

/*============================ IMPLEMENTATION ================================*/
#define JOIN1( A, B, C ) A ## B ## C
#define CONCAT( A, B, C ) JOIN1( A, B, C )


#define PORT REG_PIO

/*============================================================================
Name    :   BitBangSPI_Master_Init
------------------------------------------------------------------------------
Purpose :   Initialize BitBangSPI Interface
Input   :   n/a
Output  :   n/a
Notes   :	Called from QDebug_Init in QDebug.c
============================================================================*/
void BitBangSPI_Master_Init (void)
{
    CONCAT( PORT, SPI_BB, _OER) =  (( 1 << MOSI_BB ) | ( 1 << SCK_BB ));
    CONCAT( PORT, SPI_BB, _ODR) =  ( 1 << MISO_BB );  
    CONCAT( PORT, SPI_BB, _CODR) =  (( 1 << MOSI_BB ) | ( 1 << SCK_BB ));
    CONCAT( PORT, SPI_BB, _SODR) =  ( 1 << MISO_BB ); 
}

/*============================================================================
Name    :   BitBangSPI_Send_Byte
------------------------------------------------------------------------------
Purpose :   Send and Read one byte using BitBangSPI Interface
Input   :   Data to send to slave
Output  :   Data read from slave
Notes   :	Called from BitBangSPI_Send_Message in this file
============================================================================*/
static uint8_t BitBangSPI_Send_Byte(uint8_t c)
{
    unsigned int bit;
    volatile unsigned int i;

    for (bit = 0; bit < 8; bit++) {
        /* write MOSI on trailing edge of previous clock */
        if (c & 0x80)
            CONCAT( PORT, SPI_BB, _SODR) =  ( 1 << MOSI_BB );
        else
            CONCAT( PORT, SPI_BB, _CODR) =  ( 1 << MOSI_BB );
        
        c <<= 1;
 
        /* half a clock cycle before leading/rising edge */
        DELAY1US();
        
        CONCAT( PORT, SPI_BB, _SODR) = (1 << SCK_BB );
 
        /* half a clock cycle before trailing/falling edge */
        DELAY1US();
 
        /* read MISO on trailing edge */
        c |= ((CONCAT( PORT, SPI_BB, _PDSR) >> MISO_BB) & 0x01);
        CONCAT( PORT, SPI_BB, _CODR) = (1 << SCK_BB );
    }
    
    CONCAT( PORT, SPI_BB, _CODR) =  ( 1 << MOSI_BB );
    
    for( i=0; i<50; i++)
    {
        DELAY50US();
    }

    return c;  
}


/*============================================================================
Name    :   BitBangSPI_Send_Message
------------------------------------------------------------------------------
Purpose :   Send and Read one frame using BitBangSPI Interface
Input   :   n/a
Output  :   n/a
Notes   :	Called from Send_Message in QDebugTransport.c
============================================================================*/
void BitBangSPI_Send_Message(void)
{  
    unsigned int i;
    uint8_t FrameInProgress;
    
    // Send our message upstream	
    for (i=0; i <= TX_index; i++)
    {
        FrameInProgress = RxHandler(BitBangSPI_Send_Byte(TX_Buffer[i]));
    }
    
    // Do we need to receive even more bytes?
    while (FrameInProgress)
        FrameInProgress = RxHandler(BitBangSPI_Send_Byte(0));
}
