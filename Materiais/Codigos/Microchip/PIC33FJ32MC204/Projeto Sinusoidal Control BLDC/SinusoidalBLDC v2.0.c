//---------------------------------------------------------------------
//
//							 Software License Agreement
//
// The software supplied herewith by Microchip Technology Incorporated 
// (the “Company”) for its PICmicro® Microcontroller is intended and 
// supplied to you, the Company’s customer, for use solely and 
// exclusively on Microchip PICmicro Microcontroller products. The 
// software is owned by the Company and/or its supplier, and is 
// protected under applicable copyright laws. All rights are reserved. 
//  Any use in violation of the foregoing restrictions may subject the 
// user to criminal sanctions under applicable laws, as well as to 
// civil liability for the breach of the terms and conditions of this 
// license.
//
// THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES, 
// WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
// TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
// PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT, 
// IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR 
// CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
//
//---------------------------------------------------------------------
//	File:		sinusoidalBLDC v2.0c
//
// 
// The following files should be included in the MPLAB project:
//
//		sinusoidalBLDC v2.0.c	-- Main source code file
//		SVM.c					-- Space Vector Modulation file
//		SVM.h			
//		traps.c					-- Explicit definitions of dsPIC33F Traps
//		p33FJ32MC204.h			-- Header file for dsPIC33F
//					
//		p33FJ32MC204.gld		-- Linker script file
//---------------------------------------------------------------------- 
#include "p33FJ32MC204.h"
#include "svm.h"

/******************************************************************************/
/* Configuration bits                                                         */
/******************************************************************************/

_FOSCSEL(FNOSC_FRC);			// Start with FRC will switch to Primary (XT, HS, EC) Oscillator with PLL
_FOSC(FCKSM_CSECMD & POSCMD_XT);	// Clock Switching Enabled and Fail Safe Clock Monitor is disable
    				        // Primary Oscillator Mode: XT Crystal

_FBS (BSS_NO_FLASH & BWRP_WRPROTECT_OFF);
/* no Boot sector and
   write protection disabled */

_FWDT (FWDTEN_OFF);
/* Turn off Watchdog Timer */

_FGS (GSS_OFF & GCP_OFF & GWRP_OFF);
/* Set Code Protection Off for the General Segment */

_FPOR (PWMPIN_ON & HPOL_ON & LPOL_ON & FPWRT_PWR128);
/* PWM mode is Port registers
   PWM high & low active high
   alternate I2C mapped to SDA1/SCL1
   FPOR power on reset 128ms
*/

_FICD (ICS_PGD3 & JTAGEN_OFF);
/* Use PGC3/PGD3 for programming and debugging */


//----------------------------------------------------------------------
// 	Hurst Motor Terminals |
// -----------------------|---------------------------------
//	Ground Phase ---------|-- G
//	Phase Red    ---------|-- M1
//	Phase Black  ---------|-- M2
//	Phase White  ---------|-- M3
//	Hall White   ---------|-- HA
//	Hall Brown   ---------|-- HB
//	Hall Green   ---------|-- HC
typedef signed int SFRAC16;
#define CLOSED_LOOP      // if defined the speed controller will be enabled
#undef  PHASE_ADVANCE    // for extended speed ranges this should be defined
#define FCY  20000000	 // xtal = 8Mhz; with PLL -> 20 MIPS
#define FPWM 20000		 // 20 kHz, so that no audible noise is present.
#define _10MILLISEC	 10  // Used as a timeout with no hall effect sensors
                         // transitions and Forcing steps according to the
                         // actual position of the motor
#define _100MILLISEC 100 // after this time has elapsed, the motor is
                         // consider stalled and it's stopped
// These Phase values represent the base Phase value of the sinewave for each
// one of the sectors (each sector is a translation of the hall effect sensors
// reading 
#define PHASE_ZERO 	57344
#define PHASE_ONE	((PHASE_ZERO + 65536/6) % 65536)
#define PHASE_TWO	((PHASE_ONE + 65536/6) % 65536)
#define PHASE_THREE	((PHASE_TWO + 65536/6) % 65536)
#define PHASE_FOUR	((PHASE_THREE + 65536/6) % 65536)
#define PHASE_FIVE	((PHASE_FOUR + 65536/6) % 65536)
#define MAX_PH_ADV_DEG 40  // This value represents the maximum allowed phase
                           // advance in electrical degrees. Set a value from
                           // 0 to 60. This value will be used to calculate
                           // phase advance only if PHASE_ADVANCE is defined
// This is the calculation from the required phase advance to the actual
// value to be multiplied by the speed of the motor. So, if PHASE_ADVANCE is
// enabled, a certain amount of shit angle will be added to the generated
// sine wave, up to a maximum of the specified value on MAX_PH_ADV_DEG. This
// maximum phase shift will be present when the MeasuredSpeed variable is a 
// fractional 1.0 (for CW) or -1.0 (for CCW).
#define MAX_PH_ADV 		(int)(((float)MAX_PH_ADV_DEG / 360.0) * 65536.0)
#define HALLA	1	// Connected to RB1
#define HALLB	2	// Connected to RB2
#define HALLC	4	// Connected to RB3
#define CW	0		// Counter Clock Wise direction
#define CCW	1		// Clock Wise direction
#define SWITCH_S2	(!PORTAbits.RA8) // Push button S2
// Period Calculation
// Period = (TMRClock * 60) / (RPM * Motor_Poles)
// For example>
// Motor_Poles = 10
// RPM = 6000 (Max Speed)
// Period = ((20,000,000 / 64) * 60) / (6000 * 10) = 312.5
// RPM = 60 (Min Speed)
// Period = ((20,000,000 / 64) * 60) / (60 * 10) = 31250
#define MINPERIOD	313		// For 6000 max rpm and 10 poles motor
#define MAXPERIOD	31250	// For 60 min rpm and 10 poles motor
// Use this MACRO when using floats to initialize signed 16-bit fractional 
// variables
#define SFloat_To_SFrac16(Float_Value)	\
        ((Float_Value < 0.0) ? (SFRAC16)(32768 * (Float_Value) - 0.5) \
        : (SFRAC16)(32767 * (Float_Value) + 0.5))
void InitADC10(void);	// Initialization of ADC used for Speed Command
void InitMCPWM(void);	// Initialization for PWM at 20kHz, Center aligned, 
                        // Complementary mode with 2 us of deadtime
void InitTMR1(void);	// Initialization for TIMER1 used for speed control 
                        // and motor stalled protection
void InitTMR3(void);	// Initialization for TIMER3 used as a timebase 
                        // for the two input capture channels
void InitUserInt(void);	// This function initializes all ports 
                        // (inputs and outputs) for the application
void InitICandCN(void);	// Initializes input captures and change notification, 
                        // used for the hall sensor inputs
void RunMotor(void);	// This function initializes all variables 
                        // and interrupts used for starting and running 
                        // the motor
void StopMotor(void);	// This function clears all flags, and stops anything
                        // related to motor control, and also disables PWMs
void SpeedControl(void);     // This function contains all ASM and C operations
                             // for doing the PID Control loop for the speed
void ForceCommutation(void); // When motor is to slow to generate interrupts 
                             // on halls, this function forces a commutation
void ChargeBootstraps(void); // At the begining of the motor operation, the 
                             // bootstrap caps are charged with this function
void ClearFaults(void);      // This subroutine will clear faults

void Delay(unsigned int dly_cnt); // Prototype for delay function

// Constants used for properly energizing the motor depending on the 
// rotor's position
const int PhaseValues[6] =
{PHASE_ZERO, PHASE_ONE, PHASE_TWO, PHASE_THREE, PHASE_FOUR, PHASE_FIVE}; 
// In the sinewave generation algorithm we need an offset to be added to the
// pointer when energizing the motor in CCW. This is done to compensate an
// asymetry of the sinewave
int PhaseOffset = 6000;
// Flags used for the application
struct 
{
	unsigned MotorRunning	:1;  // This bit is 1 if motor running
	unsigned unused			:15;
}Flags;
 
unsigned int Phase;	// This variable is incremented by the PWM interrupt
                    // in order to generate a proper sinewave. Its value
                    // is incremented by a value of PhaseInc, which
                    // represents the frequency of the generated sinewave
signed int PhaseInc; // Delta increments of the Phase variable, calculated 
                     // in the TIMER1 interrupt (each 1 ms) and used in 
                     // the PWM interrupt (each 50 us)
signed int PhaseAdvance; // Used for extending motor speed range. This value
                         // is added directly to the parameters passed to the
                         // SVM function (the sine wave generation subroutine)
unsigned int HallValue;	 // This variable holds the hall sensor input readings
unsigned int Sector;  // This variables holds present sector value, which is 
                      // the rotor position
unsigned int LastSector; // This variable holds the last sector value. This 
                         // is critical to filter slow slew rate on the Hall
                         // effect sensors hardware
unsigned int MotorStalledCounter = 0; // This variable gets incremented each 
                                      // 1 ms, and is cleared everytime a new
                                      // sector is detected. Used for 
                                      // ForceCommutation and MotorStalled 
                                      // protection functions
// This array translates the hall state value read from the digital I/O to the
// proper sector.  Hall values of 0 or 7 represent illegal values and therefore
// return -1.
char SectorTable[] = {-1,4,2,3,0,5,1,-1};
unsigned char Current_Direction;	// Current mechanical motor direction of 
                                    // rotation Calculated in halls interrupts
unsigned char Required_Direction;	// Required mechanical motor direction of 
                                    // rotation, will have the same sign as the
									// ControlOutput variable from the Speed 
                                    // Controller
  
// Variables containing the Period of half an electrical cycle, which is an 
// interrupt each edge of one of the hall sensor input
unsigned int PastCapture, ActualCapture, Period; 
// Used as a temporal variable to perform a fractional divide operation in 
// assembly

unsigned int temp_count;	// Variable used for Delay Count

SFRAC16 _MINPERIOD = MINPERIOD - 1;
SFRAC16 MeasuredSpeed, RefSpeed = 200;	// Actual and Desired speeds for the PID 
                            // controller, that will generate the error
SFRAC16 ControlOutput = 0;	// Controller output, used as a voltage output, 
                            // use its sign for the required direction

// Absolute PID gains used by the controller. Position form implementation of 
// a digital PID. See SpeedControl subroutine for details
SFRAC16 Kp = SFloat_To_SFrac16(0.1);   // P Gain
SFRAC16 Ki = SFloat_To_SFrac16(0.01);  // I Gain
SFRAC16 Kd = SFloat_To_SFrac16(0.000); // D Gain
// Constants used by the PID controller, since a MAC operation is used, the 
// PID structure is changed (See SpeedControl() Comments)
SFRAC16 _XBSS(4) ControlDifference[3];
SFRAC16 _YBSS(4) PIDCoefficients[3];

// Used as a temporal variable to perform a fractional divide operation in 
// assembly
SFRAC16 _MAX_PH_ADV = MAX_PH_ADV;

/*********************************************************************************** 
 * Function: IOMap()
 *
 * Preconditions: None.
 *
 * Overview: This executes the necessary process to map the RPn pins to the
 *			 appropriate peripherals on the dsPIC33FJ32MC204.
 *
 * Input: None.
 *
 * Output: None.
 *
 *****************************************************************************/
void IOMap()
{

	//	Unlock the Reprogrammable Pin Mechanism
	__builtin_write_OSCCONL(OSCCON & (~(1<<6))); // clear bit 6 

	RPINR12bits.FLTA1R = 8; // Select pin-44 (RP8) as input for fault

	//  Assign IC1 to RP1 (RB1)
	RPINR7bits.IC1R = 1;

	//  Assign IC2 to RP2 (RB2)
	RPINR7bits.IC2R = 2;

	//  Assign IC7 to RP3 (RB3)
	RPINR10bits.IC7R = 3;

	//	Lock the Reprogrammable Pin Mechanism
	__builtin_write_OSCCONL(OSCCON | (1<<6)); 	 // Set bit 6 
}
 
/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _T1Interrupt (void)
  PreCondition:    The motor is running and is generating hall effect sensors
                   interrupts. Also, the actual direction of the motor used
                   in this interrupt is assumed to be previously calculated.
 
  Input:           None.
  Output:          None.
  Side Effects:    None.
  Overview:        In this ISR the Period, Phase Increment and MeasuredSpeed
                   are calculated based on the input capture of one of the
                   halls. The speed controller is also called in this ISR
                   to generate a new output voltage (ControlOutput). The 
                   Phase Advance is calculated based on the maximum allowed
                   phase advance (MAX_PH_ADV) and the actual speed of the 
                   motor. The last thing done in this ISR is the forced 
                   commutation, which happens each time the motor doesn't
                   generate a new hall interrupt after a programmed period 
                   of time. If the timeout for generating hall ISR is too much
                   (i.e. 100 ms) the motor is then stopped.
  Note:            The MeasuredSpeed Calculation is made in assembly to take 
                   advantage of the signed fractional division.
********************************************************************/
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void)
{
	IFS0bits.T1IF = 0;
	Period = ActualCapture - PastCapture;  // This is an UNsigned substraction
                                           // to get the Period between one 
                                           // hall effect sensor transition
    // These operations limit the Period value to a range from 60 to 6000 rpm
	if (Period < (unsigned int)MINPERIOD)  // MINPERIOD or 6000 rpm
		Period = MINPERIOD;
	else if (Period > (unsigned int)MAXPERIOD) // MAXPERIOD or 60 rpm
		Period = MAXPERIOD;
    // PhaseInc is a value added to the Phase variable to generate the sine
    // voltages. 1 electrical degree corresponds to a PhaseInc value of 184,
    // since the pointer to the sine table is a 16bit value, where 360 Elec
    // Degrees represents 65535 in the pointer. 
    // __builtin_divud(Long Value, Int Value) is a function of the compiler
    // to do Long over Integer divisions.
	PhaseInc = __builtin_divud(512000UL, Period);	// Phase increment is used
								 					// by the PWM isr (SVM)
    // This subroutine in assembly calculates the MeasuredSpeed using 
    // fractional division. These operations in assembly perform the following
    // formula:
    //                   MINPERIOD (in fractional) 
    //  MeasuredSpeed = ---------------------------
    //                    Period (in fractional)
    //
	__asm__ volatile("repeat #17\n\t"
                     "divf %1,%2\n\t"
                     "mov w0,%0" : /* output */ "=g"(MeasuredSpeed)  
                                 : /* input */ "r"(_MINPERIOD),
                                               "e"(Period)
                                 : /* clobber */ "w0");
    // MeasuredSpeed sign adjustment based on current motor direction of 
    // rotation
	if (Current_Direction == CCW)
		MeasuredSpeed = -MeasuredSpeed;
    // The following values represent the MeasuredSpeed values from the 
    // previous operations:
    //
	// CONDITION        RPM          SFRAC16      SINT      HEX
	// Max Speed CW  -> 6000 RPM  -> 0.996805  -> 32663  -> 0x7F97
	// Min Speed CW  -> 60 RPM    -> 0.009984  -> 327    -> 0x0147
	// Min Speed CCW -> -60 RPM   -> -0.009984 -> -327   -> 0xFEB9
	// Max Speed CCW -> -6000 RPM -> -0.996805 -> -32663 -> 0x8069
	SpeedControl(); // Speed PID controller is called here. It will use 
                    // MeasuredSpeed, RefSpeed, some buffers and will generate
                    // the new ControlOutput, which represents a new amplitude
                    // of the sinewave that will be generated by the SVM 
                    // subroutine.
#ifdef PHASE_ADVANCE
		// Calculate Phase Advance Based on Actual Speed and MAX_PH_ADV define
        // The following assembly instruction perform the following formula
        // using fractional multiplication:
        // 
        // PhaseAdvance = MAX_PH_ADV * MeasuredSpeed
        //

        register int a_reg asm("A");
        a_reg = __builtin_mpy(_MAX_PH_ADV, MeasuredSpeed, 0,0,0,0,0,0);
        PhaseAdvance = __builtin_sac(a_reg,0);

#endif
	MotorStalledCounter++;	// We increment a timeout variable to see if the
                            // motor is too slow (not generating hall effect
                            // sensors interrupts frequently enough) or if
                            // the motor is stalled. This variable is cleared
                            // in halls ISRs
    if ((MotorStalledCounter % _10MILLISEC) == 0)
	{
		ForceCommutation();	// Force Commutation if no hall sensor changes 
                            // have occured in specified timeout.
	}
	else if (MotorStalledCounter >= _100MILLISEC)
	{
		StopMotor(); // Stop motor is no hall changes have occured in 
                     // specified timeout
	}
	return;
}
/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _IC1Interrupt (void)
  PreCondition:    The inputs of the hall effect sensors should have low pass
                   filters. A simple RC network works.
 
  Input:           None.
  Output:          None.
  Side Effects:    None.
  Overview:        This interrupt represent Hall A ISR. Hall A -> RB1 -> IC1.
                   This is generated by the input change notification IC1.
                   The purpose of this ISR is to Calculate the actual 
                   mechanical direction of rotation of the motor, and to adjust
                   the Phase variable depending on the sector the rotor is in.
  Note 1:          The sector is validated in order to avoid any spurious
                   interrupt due to a slow slew rate on the halls inputs due to
                   hardware filtering.
  Note 2:          For Phase adjustment in CCW, an offset is added to 
                   compensate non-symetries in the sine table used.
********************************************************************/
void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt (void)
{
	IFS0bits.IC1IF = 0;	// Cleat interrupt flag
	HallValue = (unsigned int)((PORTB >> 1) & 0x0007);	// Read halls
	Sector = SectorTable[HallValue];	// Get Sector from table

    // This MUST be done for getting around the HW slow rate
	if (Sector != LastSector)	
	{
		// Since a new sector is detected, clear variable that would stop 
        // the motor if stalled.
		MotorStalledCounter = 0;
		// Motor current direction is computed based on Sector
		if ((Sector == 5) || (Sector == 2))	
			Current_Direction = CCW;
		else
			Current_Direction = CW;
        // Motor commutation is actually based on the required direction, not
        // the current dir. This allows driving the motor in four quadrants
		if (Required_Direction == CW)	
		{
			Phase = PhaseValues[Sector];
		}
		else
		{
			// For CCW an offset must be added to compensate difference in 
            // symmetry of the sine table used for CW and CCW
			Phase = PhaseValues[(Sector + 3) % 6] + PhaseOffset;
		}
		LastSector = Sector; // Update last sector
	}
	return;
}
/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _IC2Interrupt (void)
  PreCondition:    The inputs of the hall effect sensors should have low pass
                   filters. A simple RC network works.
 
  Input:           None.
  Output:          None.
  Side Effects:    None.
  Overview:        This interrupt represent Hall B ISR. Hall B -> RB2 -> IC2.
                   This is generated by the input Capture Channel IC2.
                   The purpose of this ISR is to Calculate the actual Period
                   between hall effect sensor transitions, calculate the actual
                   mechanical direction of rotation of the motor, and also to
                   adjust the Phase variable depending on the sector the rotor
                   is in.
  Note 1:          The sector is validated in order to avoid any spurious
                   interrupt due to a slow slew rate on the halls inputs due to
                   hardware filtering.
  Note 2:          For Phase adjustment in CCW, an offset is added to 
                   compensate non-symetries in the sine table used.
********************************************************************/
void __attribute__((interrupt, no_auto_psv)) _IC2Interrupt (void)
{
	IFS0bits.IC2IF = 0;	// Cleat interrupt flag
	HallValue = (unsigned int)((PORTB >> 1) & 0x0007);	// Read halls
	Sector = SectorTable[HallValue];	// Get Sector from table

    // This MUST be done for getting around the HW slow rate
	if (Sector != LastSector)
	{
		// Calculate Hall period corresponding to half an electrical cycle
		PastCapture = ActualCapture;
		ActualCapture = IC2BUF;
		IC2BUF;
		IC2BUF;
		IC2BUF;
		// Since a new sector is detected, clear variable that would stop 
        // the motor if stalled.
		MotorStalledCounter = 0;
		// Motor current direction is computed based on Sector
		if ((Sector == 3) || (Sector == 0))
			Current_Direction = CCW;
		else
			Current_Direction = CW;
        // Motor commutation is actually based on the required direction, not
        // the current dir. This allows driving the motor in four quadrants
		if (Required_Direction == CW)
		{
			Phase = PhaseValues[Sector];
		}
		else
		{
			// For CCW an offset must be added to compensate difference in 
            // symmetry of the sine table used for CW and CCW
			Phase = PhaseValues[(Sector + 3) % 6] + PhaseOffset;
		}
		LastSector = Sector; // Update last sector
	}

	return;
}
/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _IC7Interrupt (void)
  PreCondition:    The inputs of the hall effect sensors should have low pass
                   filters. A simple RC network works.
 
  Input:           None.
  Output:          None.
  Side Effects:    None.
  Overview:        This interrupt represent Hall C ISR. Hall C -> RB3 -> IC7.
                   This is generated by the input Capture Channel IC7.
                   The purpose of this ISR is to Calculate the actual 
                   mechanical direction of rotation of the motor, and to adjust
                   the Phase variable depending on the sector the rotor is in.
  Note 1:          The sector is validated in order to avoid any spurious
                   interrupt due to a slow slew rate on the halls inputs due to
                   hardware filtering.
  Note 2:          For Phase adjustment in CCW, an offset is added to 
                   compensate non-symetries in the sine table used.
********************************************************************/
void __attribute__((interrupt, no_auto_psv)) _IC7Interrupt (void)
{	
	IFS1bits.IC7IF = 0;	// Cleat interrupt flag
	HallValue = (unsigned int)((PORTB >> 1) & 0x0007);	// Read halls
	Sector = SectorTable[HallValue];	// Get Sector from table
    // This MUST be done for getting around the HW slow rate
	if (Sector != LastSector)
	{
		// Since a new sector is detected, clear variable that would stop 
        // the motor if stalled.
		MotorStalledCounter = 0;
		// Motor current direction is computed based on Sector
		if ((Sector == 1) || (Sector == 4))
			Current_Direction = CCW;
		else
			Current_Direction = CW;
		
        // Motor commutation is actually based on the required direction, not
        // the current dir. This allows driving the motor in four quadrants
		if (Required_Direction == CW)
		{
			Phase = PhaseValues[Sector];
		}
		else
		{
			// For CCW an offset must be added to compensate difference in 
            // symmetry of the sine table used for CW and CCW
			Phase = PhaseValues[(Sector + 3) % 6] + PhaseOffset;
		}
		LastSector = Sector; // Update last sector
	}
	return;
}
/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _MPWM1Interrupt (void)
  PreCondition:    None.
 
  Input:           None.
  Output:          None.
  Side Effects:    None.
  Overview:        in this ISR the sinewave is generated. If the current motor
                   direction of rotation is different from the required 
                   direction then the motor is operated	in braking mode and 
                   step commutation is performed. Once both directions are 
                   equal then the sinewave is fed into the motor windings.
                   If PHASE_ADVANCE is defined, a value corresponding to the
                   multiplication of the actual speed * maximum phase advance
                   is added to the sine wave phase to produce the phase shift
  Note:            None.
********************************************************************/
void __attribute__((interrupt, no_auto_psv)) _MPWM1Interrupt (void)
{
	IFS3bits.PWM1IF = 0;	// Clear interrupt flag
	if (Required_Direction == CW)
	{
		if (Current_Direction == CW)
			Phase += PhaseInc;    // Increment Phase if CW to generate the 
                                  // sinewave only if both directions are equal
		// If Required_Direction is CW (forward) POSITIVE voltage is applied
		#ifdef PHASE_ADVANCE
		SVM(ControlOutput, Phase + PhaseAdvance);	// PhaseAdvance addition
													// produces the sinewave
													// phase shift
		#else
		SVM(ControlOutput, Phase);
		#endif
	}
	else
	{
		if (Current_Direction == CCW)	
			Phase -= PhaseInc;      // Decrement Phase if CCW to generate
									// the sinewave only if both 
									// directions are equal
		// If Required_Direction is CCW (reverse) NEGATIVE voltage is applied
		#ifdef PHASE_ADVANCE
		SVM(-(ControlOutput+1), Phase + PhaseAdvance);// PhaseAdvance addition
													  // produces the sinewave
													  // phase shift
		#else
		SVM(-(ControlOutput+1), Phase);
		#endif
	}
	return;
}
/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _ADC1Interrupt (void)
  PreCondition:    None.
 
  Input:           None.
  Output:          None.
  Side Effects:    None.
  Overview:        The ADC interrupt loads the reference speed (RefSpeed) with
                   the respective value of the POT. The value will be a signed
                   fractional value, so it doesn't need any scaling.
  Note:            None.
********************************************************************/
void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt (void)
{
	IFS0bits.AD1IF = 0;		// Clear interrupt flag
	RefSpeed = ADC1BUF0; 	// Read POT value to set Reference Speed
	return;
}
/*********************************************************************
  Function:        void ChargeBootstraps(void)
  PreCondition:    None.
 
  Input:           None.
  Output:          None.
  Side Effects:    None.
  Overview:        In the topology used, it is necessary to charge the 
                   bootstrap caps each time the motor is energized for the 
                   first time after an undetermined amount of time. 
                   ChargeBootstraps subroutine turns ON	the lower transistors
                   for 10 ms to ensure voltage on these caps, and then it 
                   transfers the control of the outputs to the PWM module.
  Note:            None.
********************************************************************/
void ChargeBootstraps(void)
{
	unsigned int i;

	OVDCON = 0x0015;	// Turn ON low side transistors to charge
	for (i = 0; i < 33330; i++) // 10 ms Delay at 20 MIPs
		;
	PWMCON2bits.UDIS = 1;
	PDC1 = PTPER;	// Initialize as 0 voltage
	PDC2 = PTPER;	// Initialize as 0 voltage
	PDC3 = PTPER;	// Initialize as 0 voltage
	OVDCON = 0x3F00;	// Configure PWM0-5 to be governed by PWM module
	PWMCON2bits.UDIS = 0;
	return;
}
/*********************************************************************

  Function:        void RunMotor(void)

  PreCondition:    None.
 
  Input:           None.
  Output:          None.
  Side Effects:    None.
  Overview:        Call this subroutine when first trying to run the motor and
                   the motor is previously stopped. RunMotor will charge 
                   bootstrap caps, will initialize application variables, and
                   will enable all ISRs.
  Note:            None.
********************************************************************/
void RunMotor(void)
{
	ChargeBootstraps();
	// init variables
	ControlDifference[0] = 0;	// Error at K	(most recent)
	ControlDifference[1] = 0;	// Error at K-1
	ControlDifference[2] = 0;	// Error at K-2	(least recent)
	PIDCoefficients[0] = Kp + Ki + Kd;	// Modified coefficient for using MACs
	PIDCoefficients[1] = -(Kp + 2*Kd);	// Modified coefficient for using MACs
	PIDCoefficients[2] = Kd;			// Modified coefficient for using MACs
	
	TMR1 = 0;			// Reset timer 1 for speed control
	TMR3 = 0;			// Reset timer 3 for speed measurement
	ActualCapture = MAXPERIOD; 	// Initialize captures for minimum speed 
								//(60 RPMs)
	PastCapture = 0;
	// Initialize direction with required direction
	// Remember that ADC is not stopped.
	HallValue = (unsigned int)((PORTB >> 1) & 0x0007);	// Read halls
	LastSector = Sector = SectorTable[HallValue];	// Initialize Sector 
                                                    // variable
	// RefSpeed's sign will determine if the motor should be run at CW 
    // (+RefSpeed) or CCW (-RefSpeed) ONLY at start up, since when the motor 
    // has started, the required direction will be set by the control output
    // variable to be able to operate in the four quadrants
	if (RefSpeed < 0)
	{
		ControlOutput = 0;	// Initial output voltage
		Current_Direction = Required_Direction = CCW;
		Phase = PhaseValues[(Sector + 3) % 6] + PhaseOffset;
	}
	else
	{
		ControlOutput = 0;	// Initial output voltage
		Current_Direction = Required_Direction = CW;
		Phase = PhaseValues[Sector];
	}
	MotorStalledCounter = 0;	// Reset motor stalled protection counter
	// Set initial Phase increment with minimum value. This will change if a 
    // costing operation is required by the application
	PhaseInc = __builtin_divud(512000UL, MAXPERIOD);	
	// Clear all interrupts flags
	IFS0bits.T1IF = 0;		// Clear timer 1 flag
	IFS0bits.IC1IF = 0;		// Clear interrupt flag
	IFS0bits.IC2IF = 0;		// Clear interrupt flag
	IFS1bits.IC7IF = 0;		// Clear interrupt flag
	IFS3bits.PWM1IF = 0;	// Clear interrupt flag
	// enable all interrupts
	__asm__ volatile ("DISI #0x3FFF");
	IEC0bits.T1IE = 1;		// Enable interrupts for timer 1
	IEC0bits.IC1IE = 1;		// Enable interrupts on IC1
	IEC0bits.IC2IE = 1;		// Enable interrupts on IC2
	IEC1bits.IC7IE = 1;		// Enable interrupts on IC7
	IEC3bits.PWM1IE = 1;	// Enable PWM interrupts
    DISICNT = 0;
	Flags.MotorRunning = 1;	// Indicate that the motor is running
	return;
}
/*********************************************************************
  Function:        void StopMotor(void)
  PreCondition:    None.
 
  Input:           None.
  Output:          None.
  Side Effects:    None.
  Overview:        Call this subroutine whenever the user want to stop the 
                   motor. This subroutine will clear interrupts properly, and
                   will also turn OFF all PWM channels.
  Note:            None.
********************************************************************/
void StopMotor(void)
{
	OVDCON = 0x0000;	// turn OFF every transistor
	
	// disable all interrupts
	__asm__ volatile ("DISI #0x3FFF");
	IEC0bits.T1IE = 0;		// Disable interrupts for timer 1
	IEC0bits.IC1IE = 0;		// Disable interrupts on IC1
	IEC0bits.IC2IE = 0;		// Disable interrupts on IC2
	IEC1bits.IC7IE = 0;		// Disable interrupts on IC7
	IEC3bits.PWM1IE = 0;	// Disable PWM interrupts
    DISICNT = 0;
	Flags.MotorRunning = 0;	// Indicate that the motor has been stopped

	return;
}
/*********************************************************************
  Function:        void SpeedControl(void)
  PreCondition:    None.
 
  Input:           None.
  Output:          None.
  Side Effects:    None.
  Overview:        This subroutine implements a PID in assembly using the MAC
                   instruction of the dsPIC.
  Note:            None.
********************************************************************/
/*
                                             ----   Proportional
                                            |    |  Output
                             ---------------| Kp |-----------------
                            |               |    |                 |
                            |                ----                  |
Reference                   |                                     --- 
Speed         ---           |           --------------  Integral | + | Control   -------
     --------| + |  Error   |          |      Ki      | Output   |   | Output   |       |
             |   |----------|----------| ------------ |----------|+  |----------| Plant |--
        -----| - |          |          |  1 - Z^(-1)  |          |   |          |       |  |
       |      ---           |           --------------           | + |           -------   |
       |                    |                                     ---                      |
       | Measured           |         -------------------  Deriv   |                       |
       | Speed              |        |                   | Output  |                       |
       |                     --------| Kd * (1 - Z^(-1)) |---------                        |
       |                             |                   |                                 |
       |                              -------------------                                  |
       |                                                                                   |
       |                                                                                   |
        -----------------------------------------------------------------------------------
   ControlOutput(K) = ControlOutput(K-1) 
                    + ControlDifference(K) * (Kp + Ki + Kd)
                    + ControlDifference(K-1) * (-Ki - 2*Kd)
                    + ControlDifference(K-2) * Kd
   Using PIDCoefficients:
   PIDCoefficients[0] = Kp + Ki + Kd
   PIDCoefficients[1] = -(Kp + 2*Kd)
   PIDCoefficients[2] = Kd
   and leting:
   ControlOutput -> ControlOutput(K) and ControlOutput(K-1)
   ControlDifference[0] -> ControlDifference(K)
   ControlDifference[1] -> ControlDifference(K-1)
   ControlDifference[2] -> ControlDifference(K-2)
   ControlOutput = ControlOutput
                 + ControlDifference[0] * PIDCoefficients[0]
                 + ControlDifference[1] * PIDCoefficients[1]
                 + ControlDifference[2] * PIDCoefficients[2]
   This was implemented using Assembly with signed fractional and saturation enabled
   with MAC instruction
*/
void SpeedControl(void) {
        SFRAC16 *ControlDifferencePtr = ControlDifference;
        SFRAC16 *PIDCoefficientsPtr = PIDCoefficients;
        SFRAC16 x_prefetch;
        SFRAC16 y_prefetch;

		register int reg_a asm("A");
		register int reg_b asm("B");
		CORCONbits.SATA = 1;    // Enable Saturation on Acc A

	#if __C30_VERSION__ == 320
	#error "This Demo is not supported with v3.20"
	#endif

	#if __C30_VERSION__ < 320

		reg_a = __builtin_lac(RefSpeed,0);
		reg_b = __builtin_lac(MeasuredSpeed,0);
		reg_a = __builtin_subab();
		*ControlDifferencePtr = __builtin_sac(reg_a,0);
		reg_a = __builtin_movsac(&ControlDifferencePtr, &x_prefetch, 2,
				 &PIDCoefficientsPtr, &y_prefetch, 2, 0);
		reg_a = __builtin_lac(ControlOutput, 0);
		reg_a = __builtin_mac(x_prefetch,y_prefetch,
                              &ControlDifferencePtr, &x_prefetch, 2,
                              &PIDCoefficientsPtr, &y_prefetch, 2, 0);
		reg_a = __builtin_mac(x_prefetch,y_prefetch,
                              &ControlDifferencePtr, &x_prefetch, 2,
                              &PIDCoefficientsPtr, &y_prefetch, 2, 0);
		reg_a = __builtin_mac(x_prefetch,y_prefetch,
                              &ControlDifferencePtr, &x_prefetch, 2,
                              &PIDCoefficientsPtr, &y_prefetch, 2, 0);
        ControlOutput = __builtin_sac(reg_a, 0);

	#else

		reg_a = __builtin_lac(RefSpeed,0);
		reg_b = __builtin_lac(MeasuredSpeed,0);
		reg_a = __builtin_subab(reg_a,reg_b);
		*ControlDifferencePtr = __builtin_sac(reg_a,0);
		reg_a = __builtin_movsac(&ControlDifferencePtr, &x_prefetch, 2,
				 &PIDCoefficientsPtr, &y_prefetch, 2, 0, reg_b);
		reg_a = __builtin_lac(ControlOutput, 0);

		reg_a = __builtin_mac(reg_a,x_prefetch,y_prefetch,
                              &ControlDifferencePtr, &x_prefetch, 2,
                              &PIDCoefficientsPtr, &y_prefetch, 2, 0, reg_b);
		reg_a = __builtin_mac(reg_a,x_prefetch,y_prefetch,
                              &ControlDifferencePtr, &x_prefetch, 2,
                              &PIDCoefficientsPtr, &y_prefetch, 2, 0, reg_b);
		reg_a = __builtin_mac(reg_a,x_prefetch,y_prefetch,
                              &ControlDifferencePtr, &x_prefetch, 2,
                              &PIDCoefficientsPtr, &y_prefetch, 2, 0, reg_b);

    	ControlOutput = __builtin_sac(reg_a, 0);

	#endif

        CORCONbits.SATA = 0;    // Disable Saturation on Acc A
        // Store last 2 errors
        ControlDifference[2] = ControlDifference[1];
        ControlDifference[1] = ControlDifference[0];
        // If CLOSED_LOOP is undefined (running open loop) overide ControlOutput
        // with value read from the external potentiometer
        #ifndef CLOSED_LOOP
                ControlOutput = RefSpeed;
        #endif
        // ControlOutput will determine the motor required direction
        if (ControlOutput < 0)
                Required_Direction = CCW;
        else
                Required_Direction = CW;
        return;
}

/*********************************************************************
  Function:        void ForceCommutation(void)
  PreCondition:    None.
 
  Input:           None.
  Output:          None.
  Side Effects:    None.
  Overview:        This function is called each time the motor doesn't 
                   generate	hall change interrupt, which means that the motor
                   running too slow or is stalled. If it is stalled, the motor
                   is stopped, but if it is only slow, this function is called
                   and forces a commutation based on the actual hall sensor 
                   position and the required direction of rotation.
  Note:            None.
********************************************************************/
void ForceCommutation(void)
{
	HallValue = (unsigned int)((PORTB >> 1) & 0x0007);	// Read halls
	Sector = SectorTable[HallValue];	// Read sector based on halls
	if (Sector != -1)	// If the sector is invalid don't do anything
	{
		// Depending on the required direction, a new phase is fetched
		if (Required_Direction == CW)
		{
			// Motor is required to run forward, so read directly the table
			Phase = PhaseValues[Sector];
		}
		else
		{
			// Motor is required to run reverse, so calculate new phase and 
            // add offset to compensate asymmetries
			Phase = PhaseValues[(Sector + 3) % 6] + PhaseOffset;
		}
	}
	return;
}
/*---------------------------------------------------------------------
  Function Name: InitADC
  Description:   Initialize ADC module
  Inputs:        None
  Returns:       None
-----------------------------------------------------------------------*/
void InitADC( void )
{
    /* ADPCFG: ADC Port Configuration Register */
    // Set all ports digital
	AD1PCFGL = 0xFFFF;
    AD1PCFGLbits.PCFG8 = 0;   // AN8 analog

	/* Signed Fractional */
	AD1CON1bits.FORM = 3;
	/* Internal Counter Ends Sampling and Starts Conversion */
	AD1CON1bits.SSRC = 3;
	/* Sampling begins immediately after last conversion */
	AD1CON1bits.ASAM = 1;
	/* Select 12-bit, 1 channel ADC operation */
	AD1CON1bits.AD12B = 1;
	
	/* No channel scan for CH0+, Use MUX A, SMPI = 1 per interrupt, Vref = AVdd/AVss */
	AD1CON2 = 0x0000;
	
	/* Set Samples and bit conversion time */
	AD1CON3 = 0x032F; 
        	
	/* No Channels to Scan */
	AD1CSSL = 0x0000;
	
	/* Channel select AN8 */
	AD1CHS0 = 0x0008;
	
	/* Reset ADC interrupt flag */
	IFS0bits.AD1IF = 0;           

	/* Enable ADC interrupts, disable this interrupt if the DMA is enabled */	  
	IEC0bits.AD1IE = 1;       

	/* Turn on ADC module */
	AD1CON1bits.ADON = 1;          	
}
/*********************************************************************
  Function:        void InitMCPWM(void)
  PreCondition:    None.
 
  Input:           None.
  Output:          None.
  Side Effects:    None.
  Overview:        InitMCPWM, intializes the PWM as follows:
                   1. FPWM = 20000 hz
                   2. Complementary PWMs with center aligned
                   3. Set Duty Cycle to 0 for complementary, which is half 
                      the period
                   4. Set ADC to be triggered by PWM special trigger
                   5. Configure deadtime to be 2 us	
  Note:            None.
********************************************************************/
void InitMCPWM(void)
{

	PTPER = (FCY/FPWM - 1) >> 1;	// Compute Period based on CPU speed and 
                                    // required PWM frequency (see defines)
	OVDCON = 0x0000;	// Disable all PWM outputs.
	DTCON1 = 0x0028;	// 2 us of dead time
	PWMCON1 = 0x0077;	// Enable PWM output pins and configure them as 
                        // complementary mode		 
	PDC1 = PTPER;	    // Initialize as 0 voltage
	PDC2 = PTPER;	    // Initialize as 0 voltage
	PDC3 = PTPER;	    // Initialize as 0 voltage
	SEVTCMP = 1;	    // Enable triggering for ADC
	PWMCON2 = 0x0F02;	// 16 postscale values, for achieving 20 kHz
	PTCON = 0x8002;		// start PWM as center aligned mode
    FLTACON = 0x0087; 	// All PWM into inactive state, cycle-by-cycle, 

	return;				 
}
/*********************************************************************
  Function:        void InitIC(void)
  PreCondition:    None.
 
  Input:           None.
  Output:          None.
  Side Effects:    None.
  Overview:        Configure Hall sensor inputs, on IC1,IC2,IC7 input captuers.
				   On IC2 the actual capture value is used for further period 
                   calculation
  Note:            None.
********************************************************************/
void InitIC(void)
{
	//Hall A -> IC1. Hall A is only used for commutation.
	//Hall B -> IC2. Hall B is used for Speed measurement and commutation.
	//Hall C -> IC7. Hall C is only used for commutation.

	TRISB |= 0x000E;	// Ensure that hall connections are inputs 

	// Init Input Capture 1
	IC1CON = 0x0001;	// Input capture every edge with interrupts and TMR3
	IFS0bits.IC1IF = 0;	// Clear interrupt flag
	// Init Input Capture 2
	IC2CON = 0x0001;	// Input capture every edge with interrupts and TMR3
	IFS0bits.IC2IF = 0;	// Clear interrupt flag
	// Init Input Capture 7
	IC7CON = 0x0001;	// Input capture every edge with interrupts and TMR3
	IFS1bits.IC7IF = 0;	// Clear interrupt flag
	return;
}
/*********************************************************************
  Function:        void InitTMR1(void)
  PreCondition:    None.
 
  Input:           None.
  Output:          None.
  Side Effects:    None.
  Overview:        Initialization of Timer 1 as a periodic interrupt each 1 ms
                   for speed control, motor stalled protection which includes:
                   forced commutation if the motor is too slow, or motor 
                   stopped if the motor is stalled.
  Note:            None.
********************************************************************/
void InitTMR1(void)
{
	T1CON = 0x0020;			// internal Tcy/64 clock
	TMR1 = 0;
	PR1 = 313;				// 1 ms interrupts for 20 MIPS
	T1CONbits.TON = 1;		// turn on timer 1 
	return;
}
/*********************************************************************
  Function:        void InitTMR3(void)
  PreCondition:    None.
 
  Input:           None.
  Output:          None.
  Side Effects:    None.
  Overview:        Initialization of Timer 3 as the timebase for the capture 
                   channels for calculating the period of the halls.
  Note:            None.
********************************************************************/
void InitTMR3(void)
{
	T3CON = 0x0020;			// internal Tcy/64 clock
	TMR3 = 0;
	PR3 = 0xFFFF;
	T3CONbits.TON = 1;		// turn on timer 3 
	return;
}
/*********************************************************************
  Function:        int main(void)
  PreCondition:    None.
 
  Input:           None.
  Output:          None.
  Side Effects:    None.
  Overview:        main function of the application. Peripherals are 
                   initialized, and then, depending on the motor status 
                   (running or stopped) and	if the push button is pressed, 
                   the motor is started or stopped.	All other operations and
                   state machines are performed with interrupts.
  Note:            None.
********************************************************************/
int main(void)
{
    //The settings below set up the oscillator and PLL for 20 MIPS as
    //follows:
    //            Crystal Frequency  * (DIVISOR+2)
    // Fcy =     ---------------------------------
    //              PLLPOST * (PRESCLR+2) * 4	
	// Crystal  = 8 MHz
	// Fosc		= 40 MHz
	// Fcy		= 20 MIPs 

	PLLFBD =  18;		        // M=20
	CLKDIVbits.PLLPOST = 0;		// N1=2
	CLKDIVbits.PLLPRE = 0;		// N2=2
	__builtin_write_OSCCONH(0x03);
	__builtin_write_OSCCONL(0x01);

	while(OSCCONbits.COSC != 0b011);
	// Wait for PLL to lock
	while(OSCCONbits.LOCK != 1);


	TRISAbits.TRISA8 = 1; //S2 on RA8
	TRISBbits.TRISB4 = 1; //S3 on RB4

	IOMap();		// Map the Peripheral Pin Select Functions
	InitADC();		// Initialize ADC to be signed fractional
	InitTMR1();		// Initialize TMR1 for 1 ms periodic ISR
	InitTMR3();		// Initialize TMR3 for timebase of capture
	InitIC();		// Initialize Hall sensor inputs ISRs	
	InitMCPWM();	// Initialize PWM @ 20 kHz, center aligned, 2 us of deadtime

	for(;;)
	{
		if ((SWITCH_S2) && (!Flags.MotorRunning))
		{
			Delay(200);
			if(SWITCH_S2)
			{
				RunMotor();	    // Run motor if button is pressed and motor is stopped
				while(SWITCH_S2);
			}
		}
		else if ((SWITCH_S2) && (Flags.MotorRunning))
		{
			Delay(200);
			if(SWITCH_S2)
			{
				StopMotor();    // Stop motor if button is pressed and motor is running 
				while(SWITCH_S2);
			}
		}
	
	}
	return 0;
}

void Delay(unsigned int delay_count)
{
	int i;
	while (delay_count-- > 0)
	{
		for (i = 0;i < 1000;i++);
	}
	return;
}

// End of SinusoidalBLDC v2.0.c

