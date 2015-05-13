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
#ifndef BITBANGSPI_MASTER_H_INCLUDED
#define BITBANGSPI_MASTER_H_INCLUDED

#include <intrinsics.h>

/*============================ PROTOTYPES ====================================*/
extern void BitBangSPI_Master_Init (void);
extern void BitBangSPI_Send_Message(void);

/*============================ MACROS ========================================*/
//#define SS_BB       4   // slave select

/* PIOA_15 */
#define SCK_BB      15
/* PIOA_16 */
#define MOSI_BB     16
/* PIOA_17 */
#define MISO_BB     17

#define SPI_BB      A


#define nop()   (__no_operation())



#define DELAY50US() do{   \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
        nop();  \
    } while(0);

#define DELAY1US(...) DELAY50US()


#endif
/* EOF */
