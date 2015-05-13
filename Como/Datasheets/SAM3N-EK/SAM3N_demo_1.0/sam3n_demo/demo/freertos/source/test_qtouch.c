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
#include <stdio.h>

/* Miscelaneous Include from Atmel */
#include <assert.h>

/* Include SAM3N-EK definitions and initialization API */
#include "board.h"

/* Include FreeRTOS API */
#include <libfreertos.h>

/* Include QTouch API */
#include <libqtouch.h>
#include <QDebug.h>
#include <QDebugTransport.h>

/* QTouch wrapper */
#include "test_qtouch.h"


/*----------------------------------------------------------------------------
  prototypes
----------------------------------------------------------------------------*/

/*  Assign the parameters values to global configuration parameter structure    */
static void qt_set_parameters( void );

/* Configure the sensors */
static void config_sensors(void);

xQueueHandle gMsgQueue;

/*----------------------------------------------------------------------------
                                    macros
----------------------------------------------------------------------------*/
/*  The timing information for timer to fire periodically to measure touch  */
#define GET_SENSOR_SIGNAL(SENSOR_NUMBER) (qt_measure_data.channel_signals[(SENSOR_NUMBER/8)])
#define GET_SENSOR_REFERENCE(SENSOR_NUMBER) (qt_measure_data.channel_references[(SENSOR_NUMBER/8)])
#define GET_SENSOR_STATE(SENSOR_NUMBER) (qt_measure_data.qt_touch_status.sensor_states[(SENSOR_NUMBER/8)] & (1 << (SENSOR_NUMBER % 8)))
#define GET_ROTOR_SLIDER_POSITION(ROTOR_SLIDER_NUMBER) qt_measure_data.qt_touch_status.rotor_slider_values[ROTOR_SLIDER_NUMBER]

#define QT_S1_THRESHOLD  64u
#define QT_K1_THRESHOLD  64u
#define QT_K2_THRESHOLD  64u


/*----------------------------------------------------------------------------
                                extern variables
----------------------------------------------------------------------------*/
/* This configuration data structure parameters if needs to be changed will be
   changed in the qt_set_parameters function */
extern qt_touch_lib_config_data_t qt_config_data;

/* measurement data */
extern qt_touch_lib_measure_data_t qt_measure_data;

/* Get sensor delta values */
extern int16_t qt_get_sensor_delta( uint8_t sensor);

/*----------------------------------------------------------------------------
  static variables
----------------------------------------------------------------------------*/
/* Timer period in msec. */
uint16_t qt_measurement_period_msec = 25u;

/* flag set by timer ISR when it's time to measure touch */
static volatile uint8_t time_to_measure_touch = 0u;

/* current time, set by timer ISR */
static volatile uint16_t current_time_ms_touch = 0u;

/*============================================================================
Name    :   qt_set_parameters
------------------------------------------------------------------------------
Purpose :   This will fill the default threshold values in the configuration
            data structure.But User can change the values of these parameters .
Input   :   n/a
Output  :   n/a
Notes   :   initialize configuration data for processing
============================================================================*/

static void qt_set_parameters( void )
{
    /* This will be modified by the user to different values */
    qt_config_data.qt_di              = DEF_QT_DI;
    qt_config_data.qt_neg_drift_rate  = DEF_QT_NEG_DRIFT_RATE;
    qt_config_data.qt_pos_drift_rate  = DEF_QT_POS_DRIFT_RATE;
    qt_config_data.qt_max_on_duration = DEF_QT_MAX_ON_DURATION;
    qt_config_data.qt_drift_hold_time = DEF_QT_DRIFT_HOLD_TIME;
    qt_config_data.qt_recal_threshold = DEF_QT_RECAL_THRESHOLD;
}

/*============================================================================
  Name : config_sensors
------------------------------------------------------------------------------
Purpose :   Configure the sensors
Input  : n/a
Output : n/a
Notes  :
               ChanelX  ChannelY
         -----------------------
    PIOC     0  1  chanel 0    : slider
    PIOC     2  3  chanel 1    : slider
    PIOC     4  5  chanel 2    : slider
    PIOC     6  7  chanel 3
    PIOC     8  9  chanel 4    : K1
    PIOC    10 11  chanel 5    : K2
    PIOC    12 13  chanel 6
    PIOC    14 15  chanel 7
    PIOC    16 17  chanel 8
    PIOC    18 19  chanel 9
    PIOC    20 21  chanel 10
    PIOC    22 23  chanel 11
    PIOC    24 25  chanel 12
    PIOC    26 27  chanel 13
    PIOC    28 29  chanel 14
    PIOC    30 31  chanel 15
    PIOA     0  1  chanel 16
    PIOA     2  3  chanel 17
    PIOA     4  5  chanel 18
    PIOA     6  7  chanel 19
    PIOA     8  9  chanel 20
    PIOA    10 11  chanel 21
    PIOA    12 13  chanel 22
    PIOA    14 15  chanel 23
    PIOA    16 17  chanel 24
    PIOA    18 19  chanel 25
    PIOA    20 21  chanel 26
    PIOA    22 23  chanel 27
    PIOA    24 25  chanel 28
    PIOA    26 27  chanel 29
    PIOA    28 29  chanel 30
    PIOA    30 31  chanel 31

============================================================================*/
static void config_sensors(void)
{
    PMC_EnablePeripheral(ID_PIOA); // QDebug
    PMC_EnablePeripheral(ID_PIOC);

    /* Slider: PC0, PC1, PC2, PC3, PC4, PC5 */
    qt_enable_slider( CHANNEL_0, CHANNEL_2, AKS_GROUP_1, QT_S1_THRESHOLD, HYST_6_25, RES_8_BIT, 0u  );

    /* K1: SNS = PC8, SNSK = PC9 */
    qt_enable_key( CHANNEL_4, AKS_GROUP_1, QT_K1_THRESHOLD, HYST_6_25 );

    /* K2: SNS = PC10, SNSK = PC11 */
    qt_enable_key( CHANNEL_5, AKS_GROUP_1, QT_K2_THRESHOLD, HYST_6_25 );

    /* Initialize debug protocol */
    QDebug_Init();	

    /* Process commands from PC */
    QDebug_ProcessCommands();
    
}

/**
 * \brief QTouchInitialize
 * QTouchInitialize initialise the QTouch interface
*/
int TSK_QTouchInit(void)
{
    /* Create a queue capable of containing 10 values. */
    gMsgQueue = xQueueCreate( 1, sizeof( QtMsg ) );
	if( gMsgQueue == 0 ) {
		/* Queue was not created and must not be used. */
		return 1;
	}

	qt_reset_sensing();

    /* Configure the Sensors as keys or Keys With Rotor/Sliders in this function */
    config_sensors();

    /* initialise touch sensing */
    qt_init_sensing();

    /*  Set the parameters like recalibration threshold, Max_On_Duration etc in this function by the user */
    qt_set_parameters( );

    /* Address to pass address of user functions */
    /* This function is called after the library has made capacitive measurements,
    * but before it has processed them. The user can use this hook to apply filter
    * functions to the measured signal values.(Possibly to fix sensor layout faults) */
    qt_filter_callback = 0;


    /* enable interrupts */
    //sei();

  return 0;
}


/**
 * \brief TaskQTouch task.
 * This task periodically update the QTouch.
 */
void TSK_QTouch ( void* pParameter )
{
    QtMsg qtMsg;
    /*status flags to indicate the re-burst for library*/
    uint16_t status_flag = 0u;
    uint16_t burst_flag = 0u;
    
    for ( ; ; ) {

        /* clear flag: it's time to measure touch */
        time_to_measure_touch = 0u;

        /* clear flag: it's time to measure touch */
        time_to_measure_touch = 0u;

        do {
            /*  one time measure touch sensors    */
            status_flag = qt_measure_sensors( current_time_ms_touch );

           burst_flag = status_flag & QTLIB_BURST_AGAIN;

            /* send debug data */
            QDebug_SendData(status_flag);

            /*Time critical host application code goes here*/

        }while (burst_flag) ;

        /* Process commands from PC */
        QDebug_ProcessCommands();

        qtMsg.s1_position  = GET_ROTOR_SLIDER_POSITION(0);
        qtMsg.s1_state = GET_SENSOR_STATE(0);
        qtMsg.s1_delta     = qt_get_sensor_delta(0);
        qtMsg.s1_threshold = sensors[CHANNEL_0].threshold;
        
        qtMsg.k1_state  = GET_SENSOR_STATE(1);
        qtMsg.k1_delta     = qt_get_sensor_delta(1);
        qtMsg.k1_threshold = sensors[1].threshold;

        qtMsg.k2_state  = GET_SENSOR_STATE(2);
        qtMsg.k2_delta     = qt_get_sensor_delta(2);
        qtMsg.k2_threshold = sensors[2].threshold;

        xQueueSend(gMsgQueue, &qtMsg, 0);
        
        vTaskDelay( 20/portTICK_RATE_MS ) ;
    }
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
