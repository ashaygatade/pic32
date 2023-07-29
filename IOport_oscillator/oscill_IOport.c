/* 
 * File:   main.c
 * Author: 10740
 * Description: Assignment 1 --> Configure oscillator frequency at 120MHz
 *              Assignment 2 --> Blink LED. It should be turned ON for 1 sec and turned OFF for 1 sec
 * Created on 17 May, 2023, 11:15 AM
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
#include <proc/p32mz2048efh064.h>



//-----------------End of configuration generated code-----------------------------------------

//As per above oscillator configuration, we have achieved frequency of 120 MHz


//*************Function prototpes*******************
void init();
void delay();
void port_pin_input();              //function to check input port functionality
void port_pin_output();              //function to check output port functionality



int main(int argc, char** argv) 
{
    init();                         //do the initialisation
    while(1) 
    {
        port_pin_output();
        //port_pin_input();         //Input port output not exactly as per expectation
        delay();
    }
    
    return (EXIT_SUCCESS);
}

/*
 * FUNCTION NAME: init
 * DESCRIPTION: Configure the IO port pin registers
 * ARGUMENTS: None
 * RETURNS: void
 */

void init()
{
    TRISEbits.TRISE1 = 0;               //configuring pin RE1 as output     //an
    TRISEbits.TRISE5 = 0;               //configuring pin RE5 as output     //an
    TRISEbits.TRISE6 = 1;               //configuring pin RE6 as input
    
    ANSELEbits.ANSE6 = 0;               //clear ANSEL for RE6 to configure it as digital input
    //OSCCONbits.FRCDIV = 0x4;          //experimenting with FRC with FRCDIV
}

/* *************CALCULATING THE NUMBER OF ITERARIONS TO BE DONE BY 'for' loop****************
 * 
 * Oscillator frequency = 120 MHz
 * No of ticks taken by an empty for loop iteration = 5
 * No of iterations to be done by for loop to complete 1 sec delay = [1 / (120 * 1000000)] / 5 = 24000000
 */

/*
 * FUNCTION NAME: delay
 * DESCRIPTION: Function to implement 1 sec delay
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: Refer calculation given above to decide the terminating condition of the for loop
 */

void delay()
{
    for(int i = 0; i < 24000000; i++);
    //for(int i = 0; i < 200000; i++);          //for FRC with FRCDIV
    //For general blinking, Put condition i < 1 Lakh for visible results
}

/*
 * FUNCTION NAME: port_pin_input
 * DESCRIPTION: To implement input port functionality
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: ANSELx bit for that corresponding pin has to be cleared to set the IO pin as digital input.
 * By default, they are set, thus configured for Analog input.
 */


void port_pin_input()               //input functionality of port
{
    if(PORTEbits.RE6 == 1)
    {
        PORTEbits.RE6 = 0;          //clear the port pin as it retains/latches the previous value (if prev value is HIGH)
        LATEbits.LATE1 = 1;         //LED
        LATEbits.LATE5 = 1;
    }
    else if(PORTEbits.RE6 == 0)
    {
        LATEbits.LATE1 = 0;         //LED
        LATEbits.LATE5 = 0; 
    }
}

/*
 * FUNCTION NAME: port_pin_output
 * DESCRIPTION: To implement outport port functionality
 * ARGUMENTS: None
 * RETURNS: void
 */


void port_pin_output()              //ouput port functionality
{
    LATEbits.LATE1 = ~LATEbits.LATE1;           //toggle LED
    LATEbits.LATE5 = ~LATEbits.LATE5;
}







//---------------------------------------NOTES----------------------------------------------------

    /*Note:
     1) Format of API:    REGISTERbits.BITFIELD = <value>
     * where
     * REGISTER: LAT, PORT, etc
     * BITFIELD: bitfield in the said REGISTER
     * value: value to be assigned at that bit
     * 
     * 2) TRISx (Tristate): x is the port number A to K
     * Writing 0 --> will configure it as OUTPUT
     * Writing 1 --> will configure it as INPUT
     * 
     * 3) Assumption:
     * In a for loop iteration,
     * initialisation: 3 cycles
     * condition check: 2 cycles
     * increment/decrement: 2 cycles
     * Empty body execution: 1 cycle
     * 
     * 4) ANSELx register
     * Value at a PORTx register is latched onto the LATx register. Therefore programmer needs to clear the bit 
     * if input value read via PORTx register is 1
     */
    
