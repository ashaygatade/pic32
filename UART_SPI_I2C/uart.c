/* 
 * File:   putty2pic.c
 * Author: 10740
 * Description: Transmission and reception functionality of UART at Baud rate = 9600.
 *          Working successfully within loopback (manual connections) mode
 *          Working successfully with 2 different UART on same PIC32
 *          Not working as expected in interrupt mode. U2RXREG gets the data for the first time. Control is also going in U1 handler
 * Created on 16 June, 2023, 3:54 PM
 */

/*
 * Status: As on date 26-06-2023, transmit and recieve functionality is working
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

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <proc/p32mz2048efh064.h>
#include<sys/attribs.h>



//Function prototypes
void init();
void init_UART1();
void init_UART2();
void tx_UART1(char *);              //general purpose transmit function with char * as parameter
void rx_UART1();
void U1RX_INT_init();               //U1RX is configured in interrupt mode
void delay();



//Global variables
uint8_t tx_data = 0xAA;        //data to be transmitted
char disp_str[50];
uint8_t rx_data;
uint16_t tx_count = 0;
volatile uint16_t isr_count = 0;

/*
 * FUNCTION NAME: init_UART1
 * DESCRIPTION: Initialise UART1 TX and RX functionality. RX is in interrupt mode
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: Transmit logic is working. Recieve logic is working. Loopback bit is not set, but physically TX & RX of the same UART are shorted
 */

void init_UART1(void)
{   
    //TX pin init
    TRISGbits.TRISG7 = 0;                // RG7 as digital o/p 
    RPG7Rbits.RPG7R = 0b0001;            // U1 TX G7
    
//    //RX pin init
//    TRISGbits.TRISG8 = 1;                // RG8 as digital i/p
//    ANSELGbits.ANSG8 = 0;
//    U1RXRbits.U1RXR = 0b0001;            // U1 RX G8
 
    //Disable UART
    U1MODEbits.ON = 0;                   // UART1 is enabled
    U1STAbits.URXEN = 0;                 // UART1 receiver is enabled     
    U1STAbits.UTXEN = 0;                 // UART1 transmitter is enabled
    
    //Baud-rate configured for 9600
    U1MODEbits.BRGH = 0;                 // selecting standard baud rate
    U1BRG = 6;                              //Baud rate is set considering SYSCLK as reference
    
//    U1MODEbits.SIDL = 0;                 // continue uart operation     
//    U1MODEbits.IREN = 0;                 // lrDA (encoder or decoder) disable    
//    U1MODEbits.RTSMD = 0;                // U1RTS pin is in flow control mode   
//    U1MODEbits.WAKE = 1;                 // Wake-up on Start bit Detect During Sleep Mode bit    
//    U1MODEbits.LPBACK = 0;               // Loopback mode is disabled    
//    U1MODEbits.RXINV = 0;                // Receive Polarity Inversion bit UxRX Idle state is ?1?
    
    //Selecting 8N1 setting
    U1MODEbits.PDSEL = 0b00;             // 8-bit data, no parity   
    U1MODEbits.STSEL = 0;                // 1 stop bit
    
    //init rx_interrupt
    //U1RX_INT_init();                     
    
    //Enable UART
    //U1STAbits.URXEN = 1;                 // UART1 receiver is enabled     
    U1STAbits.UTXEN = 1;                 // UART1 transmitter is enabled
    U1MODEbits.ON = 1;                   // UART1 is enabled     
}


/*
 * FUNCTION NAME: init_UART2
 * DESCRIPTION: Initialise UART2 TX and RX functionality. RX is in interrupt mode
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: 
 */

void init_UART2(void)
{   
//    //TX pin init
//    TRISGbits.TRISG9 = 0;                // RG9 as digital o/p 
//    RPG9R = 0b0010;                     // U2 TX G9
    
    //RX pin init
    TRISBbits.TRISB0 = 1;                // RB0 as digital i/p
    ANSELBbits.ANSB0 = 0;
    U2RXR = 0b0101;                     // U2 RX B0
 
    //Disable UART
    U2MODEbits.ON = 0;                   // UART2 is enabled
    U2STAbits.URXEN = 0;                 // UART2 receiver is enabled     
    U2STAbits.UTXEN = 0;                 // UART2 transmitter is enabled
    
    //Baud-rate configured for 9600
    U2MODEbits.BRGH = 0;                 // selecting standard baud rate
    U2BRG = 6;                              //Baud rate is set considering SYSCLK as reference
    
//    U2MODEbits.SIDL = 0;                 // continue uart operation     
//    U2MODEbits.IREN = 0;                 // lrDA (encoder or decoder) disable    
//    U2MODEbits.RTSMD = 0;                // U2RTS pin is in flow control mode   
//    U2MODEbits.WAKE = 1;                 // Wake-up on Start bit Detect During Sleep Mode bit    
//    U2MODEbits.LPBACK = 0;               // Loopback mode is disabled    
//    U2MODEbits.RXINV = 0;                // Receive Polarity Inversion bit UxRX Idle state is ?1?
    
    //Selecting 8N1 setting
    U2MODEbits.PDSEL = 0b00;             // 8-bit data, no parity   
    U2MODEbits.STSEL = 0;                // 1 stop bit
    
    //init rx_interrupt
    U2RX_INT_init();                     
    
    //Enable UART
    U2STAbits.URXEN = 1;                 // UART2 receiver is enabled     
    //U2STAbits.UTXEN = 1;                 // UART2 transmitter is enabled
    U2MODEbits.ON = 1;                   // UART2 is enabled     
}

/*
 * FUNCTION NAME: U1RX_INT_init
 * DESCRIPTION: COnfigure the RX of UART1 in interrupt mode
 * ARGUMENTS: None
 * RETURNS: void
 */

void U1RX_INT_init()
{
    asm volatile ("di");                //Disable global interrupts
    //asm volatile ("ehb");
    
    IEC3bits.U1RXIE = 0;                    //Disable interrupt
    IFS3bits.U1RXIF = 0;                    //clear flag
    IPC28bits.U1RXIP = 0b100;               //Prio 4
    IPC28bits.U1RXIS = 0b10;                //Prio 2    
    U1STAbits.URXISEL = 0b00;               //Interrupt is triggered when there is atleast 1 character
    IEC3bits.U1RXIE = 1;                    //Enable interrupt
    
    asm volatile ("ei");                //Disable global interrupts
}

/*
 * FUNCTION NAME: U2RX_INT_init
 * DESCRIPTION: Configure the RX of UART2 in interrupt mode
 * ARGUMENTS: None
 * RETURNS: void
 */

void U2RX_INT_init()
{
    asm volatile ("di");                //Disable global interrupts
    //asm volatile ("ehb");
    
    IEC4bits.U2RXIE = 0;                    //Disable interrupt
    IFS4bits.U2RXIF = 0;                    //clear flag
    IPC36bits.U2RXIP = 0b100;               //Prio 4
    IPC36bits.U2RXIS = 0b10;                //Prio 2    
    U2STAbits.URXISEL = 0b00;               //Interrupt is triggered when there is atleast 1 character
    IEC4bits.U2RXIE = 1;                    //Enable interrupt
    
    asm volatile ("ei");                //Disable global interrupts
}

void UART1_puts(char *str) 
{
    while (*str) {
        UART1_putch(*str);
        str++;
    }
}

/*
 * FUNCTION NAME: tx_UART1
 * DESCRIPTION: Function to write data on transmit buffer
 * ARGUMENTS: No of arguments: 1, Type: char *
 * RETURNS: void
 * NOTE: UxTXREG is 9-bit register. We are writing 8-bit data
 */

void tx_UART1(char *str)
{    
    for(int i = 0; str[i] != '\0'; i++)
    {
        while(U1STAbits.UTXBF);             // Wait while buffer is full
        U1TXREG = str[i];
        delay();
    }     
}

void UART1_putch(uint8_t ch) 
{
    while (U1STAbits.UTXBF); //wait buffer empty
    U1TXREG = ch; //transmit
    tx_count++;
//    U2TXREG = 0x55; //transmit
}
/*
 * FUNCTION NAME: delay
 * DESCRIPTION: Delay functionality
 * ARGUMENTS: None
 * RETURNS: void
 */

void delay()
{
    for(int i = 0; i<10000; i++);
}

/*
 * FUNCTION NAME: rx_UART1
 * DESCRIPTION: Function to read data from RX pin in polling mode
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: 
 * 1) For arrival of data, Data available flag (URXDA) is being polled. 
 * 2) There is a possibiility that data is being shifted from UxRSR shift register to UxRXR when UxRXR is full. At that moment,
 *      Overrun error flag (OERR) will be set. Read the RX register and clear the OERR flag in that case
 * 3) Reading from UxRXREG
 * 
 *  empties the register
 */

void rx_UART1()
{
    //in polling mode
    //if(U1STAbits.URXDA == 1)            
    while(!U1STAbits.URXDA);            //Recieve buffer Data available (DA) flag is set
    
    if(U1STAbits.OERR == 1)         //This overrun error is set if UxRSR has recieved some data but UxRXREG is full.
    {
        rx_data = U1RXREG;            //Read the U1RXREG and clear the OERR bit
//        sprintf(disp_str,"RX_data after OERR error = %x\r\n",rx_data);
//        tx_UART1(disp_str);  
        //memset(disp_str," ",strlen(disp_str));
        //This overrun error bit if set, UxRSR will not be able to shift its data to UxRXREG, therefore clear the bit in software
        U1STAbits.OERR = 0;
        return;                     //return as reading is already done
    }
    
    rx_data = U1RXREG;                //read if OERR is not set
    //sprintf(disp_str,"RX_data = %x\r\n",rx_data);
    //tx_UART1(disp_str);
    //tx_UART1(disp_str);             //transmit back the recieved data
    //memset(disp_str," ",strlen(disp_str));
}

/*
 * FUNCTION NAME: rx_UART2
 * DESCRIPTION: Function to read data from RX pin in polling mode
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: 
 * 1) For arrival of data, Data available flag (URXDA) is being polled. 
 * 2) There is a possibiility that data is being shifted from UxRSR shift register to UxRXR when UxRXR is full. At that moment,
 *      Overrun error flag (OERR) will be set. Read the RX register and clear the OERR flag in that case
 * 3) Reading from UxRXREG empties the register
 */

void rx_UART2()
{
    //in polling mode           
    while(!U2STAbits.URXDA);            //Recieve buffer Data available (DA) flag is set
    
    if(U2STAbits.OERR == 1)         //This overrun error is set if UxRSR has recieved some data but UxRXREG is full.
    {
        rx_data = U2RXREG;            //Read the U2RXREG and clear the OERR bit
        U2STAbits.OERR = 0;
        return;                     //return as reading is already done
    }
    
    rx_data = U2RXREG;                //read if OERR is not set
}



int main(int argc, char** argv) 
{
    TRISEbits.TRISE1 = 0;
    init_UART1();                       //UART1 will trasnmit
    init_UART2();                       //UART2 will recieve
    while(1)
    {
        //U1TXREG = 0xAA;
        //tx_UART1("Ashay");
        delay();
        UART1_putch(++tx_data);
        //delay();
        //rx_UART1();
        //rx_UART2();
    }    
    
    return (EXIT_SUCCESS);
}

//ISR for UART1
void __ISR(_UART1_RX_VECTOR,IPL4AUTO) U1RX_Handler(void)
{
    //if(IFS3bits.U1RXIF == 1)
    //{
        //LATEbits.LATE1 = ~LATEbits.LATE1;
        isr_count++;
        rx_data = U1RXREG;                        //read recieve register
//        sprintf(disp_str,"RX_INT_data = %x\r\n",rx_data);
//        tx_UART1(disp_str);
        IFS3bits.U1RXIF = 0;                  //clear the set flag
    //}
}

//ISR for UART2
void __ISR(_UART2_RX_VECTOR,IPL4AUTO) U2RX_Handler(void)
{
    //if(IFS4bits.U2RXIF == 1)
    //{
        //LATEbits.LATE1 = ~LATEbits.LATE1;
        isr_count++;
        rx_data = U2RXREG;                        //read recieve register
//        sprintf(disp_str,"RX_INT_data = %x\r\n",rx_data);
//        tx_UART1(disp_str);
        IFS4bits.U2RXIF = 0;                  //clear the set flag
    //}
}
