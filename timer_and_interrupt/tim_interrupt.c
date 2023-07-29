/* 
 * File:   main.c
 * Author: 10740
 * Application: To implement Timer with Interrupt. With this functionality, toggle LED after 1 sec delay
 * Created on 22 May, 2023, 11:41 PM
 */



//-----------------Start of configuration generated code-----------------------------------------

// PIC32MZ2048EFH064 Configuration Bit Settings

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
#pragma config FPLLIDIV = DIV_2         // System PLL Input Divider (2x Divider)
#pragma config FPLLRNG = RANGE_BYPASS   // System PLL Input Range (Bypass)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_1         // System PLL Multiplier (PLL Multiply by 1)
#pragma config FPLLODIV = DIV_4         // System PLL Output Clock Divider (4x Divider)
#pragma config UPLLFSEL = FREQ_24MHZ    // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)

// DEVCFG1
#pragma config FNOSC = SPLL             // Oscillator Selection Bits (System PLL)
#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disable SOSC)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WDTSPGM = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = ON              // Watchdog Timer Enable (WDT Enabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31           // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = ON              // Deadman Timer Enable (Deadman Timer is enabled)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = ON              // JTAG Enable (JTAG Port Enabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
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
#include <sys/attribs.h>

#define PRESCALER 64
#define SYSCLK 1000000
//-----------------End of configuration generated code-----------------------------------------

//Oscillator has been configured at 1 MHz

//Function prototypes
void init();
void timer_init();
uint8_t get_TCKPS_value(int );

//Global variables
float delay_in_sec = 1;               //delay duration in seconds


/*
 * FUNCTION NAME: timer_init
 * DESCRIPTION: Timer specific configuration
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: PRx register is 16-bit. Chose PRESCALER and delay_in_sec such that the PRx register does not overflow
 */

void timer_init()
{
    T2CONbits.TCS = 0;              //Clear the TCS control bit (TxCON<1> = 0) to select the internal PBCLK source
    T2CONbits.TCKPS = get_TCKPS_value(PRESCALER);   //At prescaler, SYSCLK = 15.625 kHz
    TMR2 = (uint16_t) 0;            //TMRx holds the current value of counter. Initialise it with 0.
    
    //Note the value of PRx should not exceed 0xFFFF.
    PR2 = (uint16_t)((float)(SYSCLK / PRESCALER) * delay_in_sec);   //PRx value = 0x3D09 (15625)  
}

/*
 * FUNCTION NAME: init
 * DESCRIPTION: Perform all the configuration here
 * ARGUMENTS: None
 * RETURNS: void
 */

void init()
{
    //Configuration is in the sequence as per the example code in Section 14 Timers
    asm volatile ("di");                //Disable global interrupts
    asm volatile ("ehb");
    TRISEbits.TRISE1 = 0;               //configure RE1 pin as output
    TRISEbits.TRISE5 = 0;               //configure RE5 pin as output
    
    T2CONbits.ON = 0;                   //Stop the timer
    timer_init();                       //configure timer
    IEC0bits.T2IE = 0;                  //Disable Interrupt before doing configuration
    IPC2bits.T2IP = 7;              //set the user defined priority level to 7   (Range: 0,1 to 7)
    IPC2bits.T2IS = 1;              //set the sub-priority level to 1 (Range: 0 to 3)
    IFS0bits.T2IF = 0;                  //clear the Interrupt status flag
    IEC0bits.T2IE = 1;                  //Enable Interrupt
    T2CONbits.ON = 1;                  //start timer    
    asm volatile ("ei");                //Enable global interrupts
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

int main(int argc, char** argv) 
{
    //Implementing the 16-bit timer (TIMER2) program with Interrupt
    init();
    while(1);
    
    return (EXIT_SUCCESS);
}

//ISR
void __ISR(_TIMER_2_VECTOR,IPL7AUTO) Timer2Handler(void)                //source: example code family reference manual
{
    if(IFS0bits.T2IF == 1)
    {
        LATEbits.LATE5 = ~LATEbits.LATE5;           //toggle LED
        LATEbits.LATE1 = ~LATEbits.LATE1;           //toggle pin output
        IFS0bits.T2IF = 0;                  //clear the set flag
    }
}