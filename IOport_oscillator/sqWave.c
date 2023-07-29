/* 
 * File:   main.c
 * Author: 10740
 * Description: Generate a square wave with 75% duty cycle.
 * Created on 18 May, 2023, 11:52 AM
 */

#include <stdio.h>
#include <stdlib.h>

/*-----------------Start of configuration generated code-----------------------------------------*/

// PIC32MZ2048EFH064 Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config FMIIEN = OFF             // Ethernet RMII/MII Enable (RMII Enabled)
#pragma config FETHIO = OFF             // Ethernet I/O Pin Select (Alternate Ethernet I/O)
#pragma config PGL1WAY = OFF            // Permission Group Lock One Way Configuration (Allow multiple reconfigurations)
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = OFF           // USB USBID Selection (Controlled by Port Function)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // System PLL Input Divider (2x Divider)
#pragma config FPLLRNG = RANGE_BYPASS   // System PLL Input Range (Bypass)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_60        // System PLL Multiplier (PLL Multiply by 60)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (2x Divider)
#pragma config UPLLFSEL = FREQ_12MHZ    // USB PLL Input Frequency Selection (USB PLL input is 12 MHz)

// DEVCFG1
#pragma config FNOSC = SPLL             // Oscillator Selection Bits (System PLL)
#pragma config DMTINTV = WIN_0          // DMT Count Window Interval (Window/Interval value is zero)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disable SOSC)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = EC             // Primary Oscillator Configuration (External clock mode)
#pragma config OSCIOFNC = ON            // CLKO Output Signal Active on the OSCO Pin (Enabled)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disabled, FSCM Disabled)
#pragma config WDTPS = PS1              // Watchdog Timer Postscaler (1:1)
#pragma config WDTSPGM = RUN            // Watchdog Timer Stop During Flash Programming (WDT runs during Flash programming)
#pragma config WINDIS = WINDOW          // Watchdog Timer Window Mode (Watchdog Timer is in Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled)
#pragma config FWDTWINSZ = WINSZ_75     // Watchdog Timer Window Size (Window size is 75%)
#pragma config DMTCNT = DMT8            // Deadman Timer Count Selection (2^8 (256))
#pragma config FDMTEN = OFF             // Deadman Timer Enable (Deadman Timer is disabled)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (Communicate on PGEC2/PGED2)
#pragma config TRCEN = OFF              // Trace Enable (Trace features in the CPU are disabled)
#pragma config BOOTISA = MICROMIPS      // Boot ISA Selection (Boot code and Exception code is microMIPS)
#pragma config FECCCON = ON             // Dynamic Flash ECC Configuration (Flash ECC is enabled (ECCCON bits are locked))
#pragma config FSLEEP = VREGS           // Flash Sleep Mode (Flash power down is controlled by the VREGS bit)
#pragma config DBGPER = PG_NONE         // Debug Mode CPU Access Permission (Deny CPU access to all permission regions)
#pragma config SMCLR = MCLR_POR         // Soft Master Clear Enable bit (MCLR pin generates an emulated POR Reset)
#pragma config SOSCGAIN = GAIN_0_5X     // Secondary Oscillator Gain Control bits (0.5x gain setting)
#pragma config SOSCBOOST = OFF          // Secondary Oscillator Boost Kick Start Enable bit (Normal start of the oscillator)
#pragma config POSCGAIN = GAIN_0_5X     // Primary Oscillator Gain Control bits (0.5x gain setting)
#pragma config POSCBOOST = OFF          // Primary Oscillator Boost Kick Start Enable bit (Normal start of the oscillator)
#pragma config EJTAGBEN = REDUCED       // EJTAG Boot (Reduced EJTAG functionality)

// DEVCP0
#pragma config CP = ON                  // Code Protect (Protection Enabled)

// SEQ3
#pragma config TSEQ = 0xFFFF            // Boot Flash True Sequence Number (Enter Hexadecimal value)
#pragma config CSEQ = 0xFFFF            // Boot Flash Complement Sequence Number (Enter Hexadecimal value)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>


/*-----------------End of configuration generated code-----------------------------------------*/

//In the above clock cofiguration, we have set the SYSCLK as 120MHz


//Function prototypes
void init(void);
void delay(float);


int main(int argc, char** argv) 
{
    float duty_cycle = 75;              //Change duty cycle from here
    init();                             //configure the data direction registers
    while(1) 
    {   
        LATEbits.LATE1 = 1;                 //RE1 is connected to LED D4
        LATEbits.LATE5 = 1;                 //RE5 is connected to oscilloscope
        delay(duty_cycle);                  //on-duration will be duty-cycle
        
        LATEbits.LATE1 = 0;             
        LATEbits.LATE5 = 0;               
        delay(100-duty_cycle);              //off-duration will be (period - duty-cycle)
    }
    return (EXIT_SUCCESS);
}

/*
 * FUNCTION NAME: init
 * DESCRIPTION: This function is used to configure the data direction registers of the IO pin used
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: IO pins are configured to see output on on-board LED and oscillator
 */

void init()
{
    TRISEbits.TRISE1 = 0;       //configuring pin RE1 as output
    TRISEbits.TRISE5 = 0;       //configuring pin RE5 as output
}

/*
 * FUNCTION NAME: delay
 * DESCRIPTION: This function is used to create a delay using a for loop.
 * ARGUMENTS: No of arguments: 1, Type: float
 * RETURNS: void
 * NOTE: Since we are looking to have a duty cycle as per our choice, so we will use a generic delay function
 * which can create delay for ON time as well as for OFF time
 */


void delay(float delay_percent)
{
    int time_period = 24000000;
    for(int i = 0; i < (int)((delay_percent/100)*time_period); i++);
    //For general blinking, Put condition i < 1 Lakh for visible results
}