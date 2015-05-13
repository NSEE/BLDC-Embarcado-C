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

#include <string.h>

/* Atmel library includes. */
#include "board.h"

/* Free RTOS includes */
#include <libfreertos.h>

/* Atmel GUI includes */
#include "test_qtouch.h"

/**
 * \addtogroup SAM3N_demo SAM3N demo
 * @{
 * \addtogroup main SAM3N Demo main()
 * @{
 */

#define TASK_MONITOR_STACK_SIZE            (512/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_LCD_STACK_SIZE                (512/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName );
extern void vApplicationIdleHook( void );
extern void vApplicationTickHook( void );

extern void xPortSysTickHandler(void );
extern void xPortPendSVHandler(void);
extern void vPortSVCHandler(void);

void SysTick_Handler(void)
{
    xPortSysTickHandler();
}

void PendSV_Handler(void)
{
    xPortPendSVHandler();
}

void SVC_Handler(void)
{
    vPortSVCHandler();
}

void SPI_IrqHandler(void)
{
    SPIM_Handler();
}

/**
 * Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
    printf( "stack overflow %x %s\r\n", pxTask, (char *)pcTaskName ) ;
    /* If the parameters have been corrupted then inspect pxCurrentTCB to
       identify which task has overflowed its stack. */
    for( ;; ) ;
}

extern void vApplicationIdleHook( void )
{
}

/**
 * This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook( void )
{
}

/**
 * Configure the hardware for the demo.
 */
static void _SetupHardware( void )
{
    uint32_t i = 0;

    /* Initialize default IRQ priority for all Peripheral IRQs */
    for ( i=0 ; i < 32 ; i++ )
    {
        NVIC_SetPriority( (IRQn_Type)i, (configMAX_SYSCALL_INTERRUPT_PRIORITY>>4) + 1 ) ;
    }

    /* Initialize NVIC to get accept interrupts from PIO controllers */
    PIO_InitializeInterrupts( (configMAX_SYSCALL_INTERRUPT_PRIORITY>>4) + 1 );

}



/**
 * This task, when activated, send every ten seconds on debug UART the whole report of free heap and total tasks status
 */
static void _TaskMonitoring( void *pvParameters )
{
    static char szList[256] ;

    for ( ; ; )
    {
        /* printf( "--- free heap %u\r\n", xPortGetFreeHeapSize() ) ; */
        printf( "--- task ## %u", (unsigned int)uxTaskGetNumberOfTasks() ) ;
        vTaskList( (signed char*)szList ) ;
        printf( szList ) ;
        vTaskDelay( 1000 ) ;
    }
}

/**
 * Main entry point
 * Initialize hardware, App GUI, Monitor task, QTouch task
 * Then start scheduler....
 */
extern void TSK_LcdInit( void );
extern void TSK_Lcd( void *pvParameters );

extern int main( void )
{
    /* Disable watchdog. */
    WDT_Disable( WDT ) ;

    /* Output demo infomation. */
    printf( "-- Freertos SAM3N --\n\r" ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;

    /* Prepare the hardware. */
    _SetupHardware() ;

    TSK_LcdInit();
    TSK_QTouchInit();

    /* Start task to monitor processor activity */

    if ( xTaskCreate( _TaskMonitoring, "Monitor", TASK_MONITOR_STACK_SIZE, NULL, TASK_MONITOR_STACK_PRIORITY, NULL ) != pdPASS )
    {
        printf( "Failed to create Monitor task\r\n" ) ;
    }

    if ( xTaskCreate( TSK_QTouch, "QTouch", TASK_MONITOR_STACK_SIZE, NULL, TASK_MONITOR_STACK_PRIORITY, NULL ) != pdPASS )
    {
        printf( "Failed to create test qtouch task\r\n" ) ;
    }

    if ( xTaskCreate( TSK_Lcd, "Lcd", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL ) != pdPASS )
    {
        printf( "Failed to create test lcd task\r\n" ) ;
    }

    /* Start the scheduler. */
    vTaskStartScheduler() ;


    /* Will only get here if there was insufficient memory to create the idle task. */
    return 0 ;
}

/** @} */
/** @} */
