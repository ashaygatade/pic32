/* 
 * File:   adc.c
 * Author: 10740
 * Description: Program to read the values of Internal ADC
 * Created on 11 June, 2023, 3:38 PM
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

/*-----------------End of configuration generated code-----------------------------------------*/
#include<sys/attribs.h>
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <proc/p32mz2048efh064.h>

//Function prototypes
void init_UART1(void);
void tx_UART1( char * str);
void adc_init();
void init();
void convert();


//Global variables
uint32_t result;
char disp_str[50];


/*
 * FUNCTION NAME: init_UART1
 * DESCRIPTION: Function to do the initialisation of UART1
 * ARGUMENTS: None
 * RETURNS: void
 */

void init_UART1(void)
{   
    //Setting the TX pins
    TRISGbits.TRISG7 = 0;                // RG7 as digital o/p 
    RPG7Rbits.RPG7R = 0b0001;            // U1 TX G7
    
    //Setting the RX pin
    TRISGbits.TRISG8 = 1;                // RG8 as digital i/p
    U1RXRbits.U1RXR = 0b0001;            // U1 RX G8
    
    //Setting the Baud rate 9600
    U1MODEbits.BRGH = 0;                 // selecting standard baud rate
    U1BRG = 6;
    
    U1MODEbits.SIDL = 0;                 // continue uart operation     
    U1MODEbits.IREN = 0;                 // lrDA (encoder or decoder) disable    
    U1MODEbits.RTSMD = 0;                // U1RTS pin is in flow control mode   
    U1MODEbits.WAKE = 1;                 // Wake-up on Start bit Detect During Sleep Mode bit    
    U1MODEbits.LPBACK = 0;               // Loopback mode is disabled    
    U1MODEbits.RXINV = 0;                // Receive Polarity Inversion bit UxRX Idle state is ?1?
    
    //Selecting the 8N1 settings
    U1MODEbits.PDSEL = 0b00;             // 8-bit data, no parity   
    U1MODEbits.STSEL = 0;                // 1 stop bit
    
    //Enabling UART
    U1MODEbits.ON = 1;                   // UART1 is enabled
    U1STAbits.URXEN = 1;                 // UART1 receiver is enabled     
    U1STAbits.UTXEN = 1;                 // UART1 transmitter is enabled
}

/*
 * FUNCTION NAME: tx_UART1
 * DESCRIPTION: Write data onto transmit buffer
 * ARGUMENTS: No of arguments: 1, Type: char *
 * RETURNS: void
 */

void tx_UART1( char * str)
{
    for(int i = 0; str[i] != '\0'; i++)
    {  
        while(U1STAbits.UTXBF);             // Wait while buffer is full
        U1TXREG = str[i]; 
    }    
}

/*
 * FUNCTION NAME: adc_init
 * DESCRIPTION: Function to do all the required configuration of ADC
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: AN1 is selected as input source. AN1 is part of ADC1 which is here used in dedicated input mode
 */

void adc_init()
{
    //DEVADC1bits.ADCFG = ;
    ADCCON1bits.ON = 0;                     //Disable ADC
    
    //Analog input pin RB1 (AN1)
    TRISBbits.TRISB1 = 1;                   //configured for input
    ANSELBbits.ANSB1 = 1;                   //configured for analog IO
    
    //Initialize the ADC calibration values by copying them from DEVADCx Flash registers into the corresponding ADCxCFG registers
    ADC0CFG = DEVADC0;
    ADC1CFG = DEVADC1;
    ADC2CFG = DEVADC2;
    ADC3CFG = DEVADC3;
    ADC4CFG = DEVADC4;
    ADC7CFG = DEVADC7;
    
    ADC1TIMEbits.SELRES = 0b11;             //Select 12-bit resolution
    
    /* Configure ADCCSSx */
    ADCCSS1 = 0; // No scanning is used
    ADCCSS2 = 0;
    
    /* Configure ADCCMPCONx */
    ADCCMPCON1 = 0; // No digital comparators are used. Setting the ADCCMPCONx
    ADCCMPCON2 = 0; // register to '0' ensures that the comparator is disabled.
    ADCCMPCON3 = 0; // Other registers are ?don't care?.
    ADCCMPCON4 = 0;
    ADCCMPCON5 = 0;
    ADCCMPCON6 = 0;
    
    /* Configure ADCFLTRx */
    ADCFLTR1 = 0; // No oversampling filters are used.
    ADCFLTR2 = 0;
    ADCFLTR3 = 0;
    ADCFLTR4 = 0;
    ADCFLTR5 = 0;
    ADCFLTR6 = 0;
    
    //Selecting the ADC Multiplexer Analog Inputs
    //SELECTION OF POSITIVE INPUTS
    ADCTRGMODEbits.SH1ALT = 0b00;           //ADC1 Analog Input (+ve) Select bit (AN1 selected)
    //SELECTION OF NEGATIVE INPUTS
    ADCIMCON1bits.DIFF1 = 0;                //AN1 is using single-ended mode (not Differential mode)
    
    //Selecting the Format of the ADC Result: Unsigned integer
    ADCCON1bits.FRACT = 0;                  //Integer (not Fractional) Data Output Format bit for all ADCs
    ADCIMCON1bits.SIGN1 = 0;                //AN1 is using Unsigned Data mode
    
    //Disabling Scan Trigger Source
    ADCCON1bits.STRGSRC = 0b00000;          //No trigger in Scan Trigger Source
    
    //Selecting the trigger source
    ADCCON3bits.GSWTRG = 1;                 //Manually trigger a conversion by setting the Global Software Trigger bit
    ADCTRG1bits.TRGSRC1 = 0b00001;          //Global software edge Trigger (GSWTRG); Trigger Source for Conversion of Analog Input AN1 Select bits
    
    //Note: When Synchronous sampling is used (SSAMPEN = 1), the presynchronized trigger value is ignored (STRGEN = ?don?t care?)
    
    //ADCTRGMODE: ADC TRIGGERING MODE FOR DEDICATED ADC REGISTER
    ADCTRGMODEbits.STRGEN1 = 1;             //ADC1 uses presynchronized triggers
    ADCTRGMODEbits.SSAMPEN1 = 1;            //Enable synchronous sampling for ADC1
    
    //ADCx Clock Divisor bits. These bits divide the ADC control clock with period TQ to generate the clock for ADCx (TADx)
    ADC1TIMEbits.ADCDIV = 0b0000001;        //2 * TQ = TADx
    ADC1TIMEbits.SAMC = 0b0000000000;       //ADC Sample time bits 2 TADx    
    
    ADCCON1bits.FSSCLKEN = 1;               //Fast synchronous system clock to ADC control clock is enabled
    ADCCON1bits.FSPBCLKEN = 0;              //Fast synchronous peripheral clock to ADC control clock is disabled
    
    //Selecting the reference voltage for ADC
    ADCCON3bits.VREFSEL = 0b000;            //ADREF+ = AVdd; ADREF- = AVss
    ADCCON1bits.AICPMPEN = 0;               //Analog input charge pump is disabled
    
    //Selecting the ADC clock source and prescaler
    ADCCON3bits.ADCSEL = 0b01;              //Clock source is SYSCLK
    ADCCON3bits.CONCLKDIV = 0b000001;       //--> 2 * TCLK = TQ. ADC Control Clock (TQ) Divider bits
    
    /* Early interrupt */
    ADCEIEN1 = 0; // No early interrupt
    ADCEIEN2 = 0;
    
    /*
     * Note: On disabling the ADC, DIGENx and ANENx pins are cleared (and hence digital and analog circuitry are disabled).
     *      Hence those bits need to be set to start the digital and analog circuitry
     */
    
    //Turning ON the ADC
    ADCANCONbits.ANEN1 = 1;                 //Analog and bias circuitry enabled
    ADCCON1bits.ON = 1;                     //Enable the ADC module
    ADCCON3bits.DIGEN1 = 1;                 //ADC1 is digital enabled
    
    ADCCON1bits.AICPMPEN = 0;               //Analog input charge pump disable
}

/*
 * FUNCTION NAME: init
 * DESCRIPTION: Primary init function
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: UART is used for printing ADC result data on putty
 */

void init()
{
    init_UART1();
    adc_init();
}

/*
 * FUNCTION NAME: convert
 * DESCRIPTION: Functionality of triggering input which stops sampling and starts conversion
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: Wait till the input source is stable. Trigger the conversion and read the conversion result and transmit on putty using UART
 */

void convert()
{
    //Code reference taken from reference manual
        
    /* Wait for voltage reference to be stable */
    while(!ADCCON2bits.BGVRRDY); // Wait until the reference voltage is ready
    while(ADCCON2bits.REFFLT); // Wait if there is a fault with the reference voltage
        
    //Selecting the trigger source
    ADCCON3bits.GSWTRG = 1;                 //Manually trigger a conversion by setting the Global Software Trigger bit
    
    while(!ADCDSTAT1bits.ARDY1);
        
    for(int i=0; i<500000; i++);
    result = ADCDATA1;
    sprintf(disp_str, "Result of ADC = %d\r\n", result);
    tx_UART1(disp_str);
}


int main(int argc, char** argv) 
{
    init();
    while(1)
    {
        convert();                          //triggering and collection of result is done periodically
    }

    return (EXIT_SUCCESS);
}

