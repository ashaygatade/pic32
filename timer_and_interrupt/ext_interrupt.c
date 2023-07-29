/* 
 * File:   main.c
 * Author: 10740
 * Description: To implement the functionality of External Interrupt. Using this functionality, 
 *              toggle the LED on arrival of External Interrupt
 * Created on 23 May, 2023, 3:34 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/attribs.h>                //this header file contains important macro definitions used in writing ISR

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

//-----------------End of configuration generated code-----------------------------------------

//Oscillator has been configured at 1 MHz

void init(void);
void interrupt_init(void);



int main(int argc, char** argv) 
{   
    init();                             //do all the configuration and initialisations
    while(1);
    return (EXIT_SUCCESS);
}

/*
 * FUNCTION NAME: init
 * DESCRIPTION: init function for initialisation
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: ANSELx register needs to be cleared to configure the pin as digital input
 */

void init()
{
    asm volatile ("di");
    asm volatile ("ehb");
    //Input pins
    TRISGbits.TRISG9 = 1;                   //configure RG9 port pin as input
    ANSELGbits.ANSG9 = 0;                   //clear the ANSEL register to configure it for digital input
    
    //Enable pull-down functionality of input pin from CNPDx (Change Notification pull-down)register
    CNPDGbits.CNPDG9 = 1;
    
    //Output pins
    TRISEbits.TRISE5 = 0;                   //configure RE5 pin as output
    TRISEbits.TRISE1 = 0;                   //configure RE1 pin as Output
    
    //Initialisation for interrupt
    interrupt_init();
    asm volatile ("ei");
}

/*
 * FUNCTION NAME: interrupt_init
 * DESCRIPTION: Configuration specific to External Interrupt
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: There are 5 external interrupt sources (EXT0, EXT1, .., EXT4). Only EXT0 has a dedicated IO pin, other sources 
 *         are required to be mapped using PPS functionality
 */

void interrupt_init()
{
    //PPS pin for External Interrupt
    INT1R = 0b0001;                     //PPS : 0001 --> RPG9, chose pin G9 port pin as interrupt pin (INTR1)
    //IEC0bits.INT1IE = 0;                //Disable the Interrupt for EXTI1 before doing configurations
    IPC2bits.INT1IP = 0b100;            //setting the priority to 4
    IPC2bits.INT1IS = 0b000;             //Setting the Sub-priority to 0
    //INTCONbits.INT3EP = 1;              //Interrupt on positive (rising) edge
    INTCONbits.MVEC = 1;                //Interrupt controller configured for multi vectored mode
    IEC0bits.INT1IE = 1;                //Enable the Interrupt for EXTI1
    IFS0bits.INT1IF = 0;                // Clear the interrupt flag status bit associated for the interrupt source in the IFSn Status register
    
    //PIC32MZ2048EFG100 has 7 Shadow register sets, we have used set 1 in below instruction
    //PRISSbits.PRI7SS = 0b0001;          //Interrupt with a priority level of 7 uses Shadow Set 1   
}

/*
 * NOTES WITH RESPECT TO ISR:
 * 
 * 1) __ISR_AT_VECTOR: Macro for defining an interrupt service routine such that the entire handler is placed at the vector address
 * 2) _EXTERNAL_1_VECTOR: Name of the XC32 Vector name. In definition, it actually has the value of the IRQ. 
 *                      This links the function with the base address of handler.
 * 3) IPL4AUTO: In IPLxAUTO, x should be equal to the group priority of the Interrupt.
 * 
 * Extra observation: See definition of __ISR_AT_VECTOR, you will get to know that the arguments to the macro are varargs (variable arguments)
 * Probably, variable arguments are optional, therefore removing the optional ones will not give error
 */

void __ISR_AT_VECTOR(_EXTERNAL_1_VECTOR , IPL4AUTO) ISRExtInt(void)
{
    //logic to be implemented in ISR
    LATEbits.LATE1 = ~LATEbits.LATE1;
    LATEbits.LATE5 = ~LATEbits.LATE5;
    for(int i=0; i<1000; i++);         //delay
    IFS0bits.INT1IF = 0;               //clear Interrupt at end of ISR
}



/*
 * NOTE:
 * 1) Selecting RE9 on INT3 and implementing the same code pattern did not give any result, however selecting RG9 on
 * INT1 is giving results.
 * 2) Every IO pins have weak pull-up and pull-down resistors. This functionality can be enabled  by corresponding
 *  CNPUx & CNPDx resistor
 */