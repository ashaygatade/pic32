/* 
 * File:   main.c
 * Author: 10740
 * Description: Configure type B Timer at 16-bit counter mode. With this functionality, toggle LED after 1 sec delay
 * Created on 22 May, 2023, 11:52 AM
 */


/*-----------------Start of configuration generated code-----------------------------------------*/
// 'C' source line config statements

// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config FMIIEN = ON              // Ethernet RMII/MII Enable (MII Enabled)
#pragma config FETHIO = ON              // Ethernet I/O Pin Select (Default Ethernet I/O)
#pragma config PGL1WAY = ON             // Permission Group Lock One Way Configuration (Allow only one reconfiguration)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USBID Selection (Controlled by the USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // System PLL Input Divider (3x Divider)
#pragma config FPLLRNG  = RANGE_BYPASS // System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK = PLL_FRC      // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_1        // System PLL Multiplier (PLL Multiply by 50)
#pragma config FPLLODIV = DIV_4         // System PLL Output Clock Divider (2x Divider)
#pragma config UPLLFSEL = FREQ_24MHZ    // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)

// DEVCFG1
#pragma config FNOSC = SPLL             // Oscillator Selection Bits (System PLL)
#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = ON             // Secondary Oscillator Enable (Enable SOSC)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = HS             // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WDTSPGM = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31           // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = OFF             // Deadman Timer Enable (Deadman Timer is disabled)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (Communicate on PGEC2/PGED2)
#pragma config TRCEN = ON               // Trace Enable (Trace features in the CPU are enabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = OFF_UNLOCKED   // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = PG_ALL          // Debug Mode CPU Access Permission (Allow CPU access to all permission regions)
#pragma config SMCLR = MCLR_NORM        // Soft Master Clear Enable bit (MCLR pin generates a normal system Reset)
#pragma config SOSCGAIN = GAIN_2X       // Secondary Oscillator Gain Control bits (2x gain setting)
#pragma config SOSCBOOST = ON           // Secondary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCGAIN = GAIN_2X       // Primary Oscillator Gain Control bits (2x gain setting)
#pragma config POSCBOOST = ON           // Primary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config EJTAGBEN = NORMAL        // EJTAG Boot (Normal EJTAG functionality)

// DEVCP0
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// SEQ3
#pragma config TSEQ = 0xFFFF            // Boot Flash True Sequence Number (Enter Hexadecimal value)
#pragma config CSEQ = 0xFFFF            // Boot Flash Complement Sequence Number (Enter Hexadecimal value)


// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

#define SYSCLK 1000000                  //in Hz
#define PBCLK (SYSCLK/2)                  //in Hz

//-----------------End of configuration generated code-----------------------------------------

//The oscillator has been configured at 1 MHz. Timer type-B uses PBCLK3. PBCLK3 is SYSCLK divided by 2 

//Function prototypes
void init();
void delay();
void timer2_init(float);
uint8_t get_TCKPS_value(int);
void check_PBxDIV();

//Global variables
float delay_in_sec = 1;               //delay duration in seconds
int prescaler = 64;                   //At prescaler = 64, we are getting the frequency of 15.625 kHz.


/*
 * FUNCTION NAME: check_PBxDIV
 * DESCRIPTION: Function written to verify the PRxDIV values.
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: Implemented to check the default value of PBDIV bit 
 */

void check_PBxDIV()
{
    if(PB3DIVbits.PBDIV == 0b0000001)
    {
        LATEbits.LATE1 = ~LATEbits.LATE1;
        delay();
    }
}


/*
 * FUNCTION NAME: timer2_init
 * DESCRIPTION: Timer2 specific configuration
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: Note: We should decide SYSCLK and Prescaler such that PRx register value should not exceed 0xFFFF 
 *      (PRx is 16-bit register) 
 */

void timer2_init(float duration_in_sec)
{
    T2CONbits.ON = 0;                   //Clear the ON control bit (TxCON<15> = 0) to disable the time
    //PB3DIVbits.PBDIV = 0b0000001;
    T2CONbits.TCS = 0;                  //Clear the TCS control bit (TxCON<1> = 0) to select the internal PBCLK source (PBCLK3)
    TMR2 = (uint16_t) 0;            //TMRx (16-bit register) holds the current value of counter. Initialise it with 0.
    T2CONbits.TCKPS = get_TCKPS_value(prescaler);
    //PR2 = (uint16_t) 0x3D09;        //As per the decided prescaler, the 16-bit PRx value = 15625
    PR2 = (uint16_t)((SYSCLK / prescaler) * duration_in_sec);
    //PB3DIVbits.ON = 1;            //
    T2CONbits.ON = 1;               //start timer
}


/*
 * FUNCTION NAME: get_TCKPS_value
 * DESCRIPTION: To chose a corresponding TCKPS value for the chosen PRESCALER
 * ARGUMENTS: No of arguments: 1, Type: int
 * RETURNS: Returns the corresponding TCKPS value for the chosen PRESCALER
 */

uint8_t get_TCKPS_value(int prescaler)
{
    switch(prescaler)
    {
        case 1:
            return 0b000;
        case 2:
            return 0b001;
        case 4:
            return 0b010;
        case 8:
            return 0b011;
        case 16:
            return 0b100;
        case 32:
            return 0b101;
        case 64:
            return 0b110;
        case 128:
            return 0b111;     
    }
}


/*
 * FUNCTION NAME: delay
 * DESCRIPTION: delay using timer2
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: timer2_init should also be called because delay is implemented using timer2
 */

void delay()
{
    //Flag will be set when TMRx and PRx matches
    while(!IFS0bits.T2IF);                      //break the loop when timer flag is set.
    IFS0bits.T2IF = 0;
    //for(int i=0; i < 10000; i++);
}


/*
 * FUNCTION NAME: init
 * DESCRIPTION: Do the required timer initialisation
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: Timer is configured to implement the delay of 1 sec
 */

void init()
{
    TRISEbits.TRISE1 = 0;
    TRISEbits.TRISE5 = 0;
    timer2_init(delay_in_sec);
}

int main(int argc, char** argv) 
{
    /* Implementing the timer program using TIMER2 as using 16 bit configuration
     */
    init();
    while(1) 
    {
        //check_PBxDIV();                           //for verification purpose
        delay();
        LATEbits.LATE5 = ~LATEbits.LATE5;         //write HIGH to IO pin
        //LATEbits.LATE1 = ~LATEbits.LATE1;        
    }
    return (EXIT_SUCCESS);
}





/*
 * NOTES:
 * 1) CALCULATION OF PRx value and Prescaler
 * Note: PRx and TCKPS are 16-bit registers, therefore pick a value which will fit in 16-bit register (65535)
 * 
 * SYSCLK = 1000000 (1 MHz)
 * PRESCALER = 64
 * [1 / (SYSCLK / PRESCALER)] * PRx = delay_duration
 *  
 * 2) TxIF will be set when TRMx matches PRx
 * 
 * 3)
 * PBCLK3 = PBCLKx is SYSCLK divided by 2 (Verified)
 * PBCLK1 = PBCLKx is SYSCLK divided by 2 (Verified)
 * PBCLK7 = PBCLKx is SYSCLK divided by 1 (Verified)
 */
 