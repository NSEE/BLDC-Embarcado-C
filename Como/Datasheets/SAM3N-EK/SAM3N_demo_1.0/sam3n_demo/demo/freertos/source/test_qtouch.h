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

#ifndef __QTOUCH_H_
#define __QTOUCH_H_

/* Include QTouch API */
#include <libqtouch.h>

extern xQueueHandle gMsgQueue;

/** QTouch messages shared with the current task */
typedef struct _QtMsg {
    short    s1_delta;
    uint8_t  s1_state;
    uint8_t  s1_threshold;

    short    k1_delta;
    uint8_t  k1_state;
    uint8_t  k1_threshold;

    short    k2_delta;
    uint8_t  k2_state;
    uint8_t  k2_threshold;

    uint8_t  s1_position;
} QtMsg;

/** QTouchInitialize initialise the interface */
extern int TSK_QTouchInit(void);

/**
 * \brief TaskQTouch task.
 * This task periodically calls the QTouch library measurment.
 */

extern void TSK_QTouch ( void* pParameter );

#endif /* __QTOUCH_H_ */
