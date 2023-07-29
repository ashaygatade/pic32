/* 
 * File:   spi_intrapic.c
 * Author: 10740
 * Description: To implement the SPI communication protocol within 2 SPIs of same PIC32
 *          Status: Working properly
 * Created on 20 June, 2023, 4:43 PM
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

//In the above clock cofiguration, we have set the SYSCLK as 1 MHz
//SPI uses PBCLK2 which is SYSCLK/2


#include <xc.h>
#include <stdio.h>                       //for sprintf
#include <stdlib.h>
#include <sys/attribs.h>
#include <string.h>
#include <bits/alltypes.h>       


//Macros
#define DEF_DELAY 500000                //Default delay
#define CS_PIN LATGbits.LATG6
#define WRITE_TO_RE1 LATEbits.LATE1
#define WRITE_TO_RC13 LATCbits.LATC13


//Function prototypes
void init();
void SPI1_master_mode_init();
void SPI1_transfer(uint32_t );
void SPI2_slave_mode_init();
void SPI2_recieve();
void delay();
void init_UART1(void);
void tx_UART1( char * );
void data_size_select(uint8_t );            //8, 16 or 32 bit


//Global variables
int read_byte_count = 0;
uint32_t tx_data = 0x80FF80FF;
uint32_t rx_data_arr[100], rx_data;         //array to store the data recieved on SPIRXBUF
uint8_t data_size = 32;         //communication takes place in 32-bit
volatile int idx = 0;
uint32_t temp_u16_buf;          //temporary variable to read buffer to empty it
char disp_str[100];              //String to transmit on UART
uint8_t togg_val1 = 0, togg_val2 = 0;
uint8_t isr_count;



//On master end, SPI1 is to be configured at Master mode, 32-bit, Standard SPI mode


//Start of Function definitions

/*
 * FUNCTION NAME: init_UART1
 * DESCRIPTION: Do the configuration for UART1. Here UART1 is used to print the SPI data recieved
 * ARGUMENTS: None
 * RETURNS: void
 */

void init_UART1(void)
{   
    TRISGbits.TRISG7 = 0;                // RG7 as digital o/p 
    RPG7Rbits.RPG7R = 0b0001;            // U1 TX G7
    
    TRISGbits.TRISG8 = 1;                // RG8 as digital i/p
    U1RXRbits.U1RXR = 0b0001;            // U1 RX G8
    
    U1MODEbits.BRGH = 0;                 // selecting standard baud rate
    U1BRG = 6;
    
    U1MODEbits.SIDL = 0;                 // continue uart operation     
    U1MODEbits.IREN = 0;                 // lrDA (encoder or decoder) disable    
    U1MODEbits.RTSMD = 0;                // U1RTS pin is in flow control mode   
    U1MODEbits.WAKE = 1;                 // Wake-up on Start bit Detect During Sleep Mode bit    
    U1MODEbits.LPBACK = 0;               // Loopback mode is disabled    
    U1MODEbits.RXINV = 0;                // Receive Polarity Inversion bit UxRX Idle state is ?1?
    
    
    U1MODEbits.PDSEL = 0b00;             // 8-bit data, no parity   
    U1MODEbits.STSEL = 0;                // 1 stop bit
    
    U1MODEbits.ON = 1;                   // UART1 is enabled
    U1STAbits.URXEN = 1;                 // UART1 receiver is enabled     
    U1STAbits.UTXEN = 1;                 // UART1 transmitter is enabled
}

/*
 * FUNCTION NAME: tx_UART1
 * DESCRIPTION: Function to transmit the data for display purpose. Tx line of UART1 is connected to Rx of USB to TTL converter
 * ARGUMENTS: No of arguments: 1, Type: char *
 * RETURNS: void
 */

void tx_UART1( char *str)
{
    for(int i = 0; str[i] != '\0'; i++)
    {
        while(U1STAbits.UTXBF);             // Wait while buffer is full
        U1TXREG = str[i]; 
    }    
}

/*
 * FUNCTION NAME: common_gpio_init
 * DESCRIPTION: Do the initialisation of GPIOs which are going to be used in common
 * ARGUMENTS: None
 * RETURNS: void
 */

void common_gpio_init()
{
    TRISEbits.TRISE1 = 0;               //Red LED for indication purpose
    TRISCbits.TRISC13 = 0;              //Green LED for indication purpose
}

/*
 * FUNCTION NAME: data_size_select
 * DESCRIPTION: Select the data-size in which communication is to be done
 * ARGUMENTS: No of arguments: 1, Type: uint8_t
 * RETURNS: void
 * NOTE: Arguments can be 8, 16 or 32 
 */

void data_size_select(uint8_t data_size)
{
    switch(data_size)
    {
        case 8:
            SPI1CONbits.MODE32 = 0;
            SPI1CONbits.MODE16 = 0;
            break;
        case 16:
            SPI1CONbits.MODE32 = 0;
            SPI1CONbits.MODE16 = 1;
            break;
        case 32:
            SPI1CONbits.MODE32 = 1;
            SPI1CONbits.MODE16 = 0;
            break;
    }
}

/*
 * FUNCTION NAME: SPI1_master_mode_init
 * DESCRIPTION: SPI1 initialisation to act in master mode.
 * ARGUMENTS: None
 * RETURNS: void
 */

void SPI1_master_mode_init()
{
    //-------------------Configuring the pins for SPI1------------------------
    //dedicated SCK pin. No need to do PPS
    TRISDbits.TRISD1 = 0;               //Dedicated SCK1 for master, configured as output
    
    //SDO pin
    TRISFbits.TRISF1 = 0;               //SDO1 for master configured output
    RPF1R = 0b0101;            //Mapping the pin using PPS
    
    //SDI1 pin for master
    TRISBbits.TRISB9 = 1;               //SDI1 for slave configured input
    SDI1R = 0b0101;     // B9 Pin- MISO (SDI)- J3-2. Mapping the pin using PPS
    ANSELBbits.ANSB9 = 0;               //Configuring for digital input
    
    TRISDbits.TRISD9 = 0;               //SS1 for master configured output. Controlled by port pin functionality
    RPD9R = 0b0101;                     //Mapping the pin using PPS
    
    //-------------Doing the SPI specific configuration-----------------
    
   //Disable the SPI1 interrupts in the respective IECx register (Recommended in reference manual)
    IEC3bits.SPI1EIE = 0;           //Disable _SPI1_FAULT_VECTOR
    IEC3bits.SPI1RXIE = 0;          //Disable _SPI1_RX_VECTOR
    IEC3bits.SPI1TXIE = 0;          //Disable _SPI1_TX_VECTOR 
    
    SPI1CONbits.ON = 0;             //Stop and reset the SPI module by clearing the ON bit
    
    //SPI1BUF = 0;
    temp_u16_buf =  SPI1BUF;        //clear the buffer by reading it
    
    SPI1CONbits.ENHBUF = 0;         //Clear the ENHBUF bit if using Standard Buffer mode
    
    data_size_select(data_size);

    SPI1STATbits.SPIROV = 0;        //Clear the SPIROV (Recieve overflow) bit
    
    SPI1BRG = 0;                    //if SPIxBRG = 0, Frequency for SCK = Peripheral clock / 2. Formula written at the end of program
    
    SPI1CONbits.MSTEN = 1;          //Set to chose Master mode; Clear to chose slave mode
    
    //Clock configuration (configured for mode 0)
    SPI1CONbits.MCLKSEL = 0;        //(Master Clock Select bit) PBCLK is used by the Baud Rate Generator
    SPI1CONbits.CKE = 0;            //Output data changes on transition from idle clock state to active clock state
    SPI1CONbits.CKP = 0;            //Idle state for clock is a low level
    
    //SPI Data Input Sample Phase bit
    SPI1CONbits.SMP = 0;            //Input data sampled at middle of data output time
    
    //Enable MSSEN bit for ease, otherwise controlled by port pin
    SPI1CONbits.MSSEN = 0;          //Slave select (in Master mode) is disabled
    
    SPI1CONbits.ON = 1;             //Enable SPI1
    
    //Data to be transmitted. Actual transmission (and reception) will start now after filling transmit buffer
}

/*
 * FUNCTION NAME: SPI2_rx_INT_enable
 * DESCRIPTION: SPI2 recieve interrupt enable
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: Recieve interrupt is triggered when there is new data available at SPIxBUF recieve buffer
 */

void SPI2_rx_INT_enable()
{
    IFS4bits.SPI2RXIF = 0;                      //clear flag
    IPC36bits.SPI2TXIP = 0b100;               //Prio 4
    IPC36bits.SPI2TXIS = 0b10;                //Prio 2
    IEC4bits.SPI2RXIE = 1;                      //Set the Enable bit
}


/*
 * FUNCTION NAME: SPI2_slave_mode_init
 * DESCRIPTION: SPI2 initialisation to act in slave mode
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: 	(optional)
 */

void SPI2_slave_mode_init()
{
    // SPI2 PIN Configuration for slave mode
    
    //MOSI => SDO2 as output    Controller PIN = 11 (RB5)
    TRISBbits.TRISB5 = 0;
    RPB5Rbits.RPB5R = 6;        
    
    //SS  => SS2 as input     Controller PIN = 29 (RB14)
    TRISBbits.TRISB14 = 1;
    SS2R = 0b0010;
    ANSELBbits.ANSB14 = 0;
    
    //MISO => SDI2 as input     Controller PIN = 15 (RB1)
    TRISBbits.TRISB1 = 1;
    SDI2Rbits.SDI2R = 5; 
    ANSELBbits.ANSB1 = 0;
    
    // SCK => Set pin direction as a input. Controller PIN = 4 (RG6)
    TRISGbits.TRISG6 = 1;
    ANSELGbits.ANSG6 = 0;  
    
    
    //-------------Doing the SPI specific configuration-----------------
    
   //Disable the SPI2 interrupts in the respective IECx register (Recommended in reference manual)
    IEC4bits.SPI2EIE = 0;           //Disable _SPI2_FAULT_VECTOR
    IEC4bits.SPI2RXIE = 0;          //Disable _SPI2_RX_VECTOR
    IEC4bits.SPI2TXIE = 0;          //Disable _SPI2_TX_VECTOR
    
    SPI2CONbits.ON = 0;             //Stop and reset the SPI module by clearing the ON bit
    
    //SPI2BUF = 0;                    //Clear the receive buffer
    temp_u16_buf =  SPI2BUF;        //clear the buffer by reading it
    
    SPI2CONbits.ENHBUF = 0;         //Clear the ENHBUF bit if using Standard Buffer mode
    
    //Enable SPI recieve interrupt
    SPI2_rx_INT_enable();
    
    data_size_select(data_size);
    
    SPI2BRG = 0;                    //Frequency for SCK = 50 MHz (Peripheral clock / 2)
    
    SPI2STATbits.SPIROV = 0;        //Clear the SPIROV (Recieve overflow) bit
    SPI2CONbits.MSTEN = 0;          //Set to chose Master mode; Clear to chose slave mode
    
    /*
     * Note: All CLK settings are done for master mode. For slave mode, settings are not done (thus are at default settings)
     *      CLK settings on master mode are also exactly same as default (but written for understanding purpose)
     *      SPIxBRG has to be initialised on slave end as well
     */
    
    //SPI Data Input Sample Phase bit
    SPI2CONbits.SMP = 0;            //Input data sampled at middle of data output time    
    
    //Enable SSEN bit for ease, otherwise controlled by port pin
    SPI2CONbits.SSEN = 0;           //SSx pin used for Slave mode, otherwise controlled by port pin
    SPI2CONbits.ON = 1;             //Enable SPI2
}

/*
 * FUNCTION NAME: init
 * DESCRIPTION: Primary initialisation function which carries out all required configuration
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: As the code flashing between Master and Slave is managed by conditional compilation, init_UART1 
 *      and corresponding SPI init functions are called seperately
 */

void init()
{
    init_UART1();
    SPI1_master_mode_init();             //set up the SPI module for the Master mode operation
    SPI2_slave_mode_init();              //set up the SPI module for the Slave mode operation
}

/*
 * FUNCTION NAME: SPI1_transfer
 * DESCRIPTION: Function to write data onto SPIxBUF register
 * ARGUMENTS: No of arguments: 1, Type: uint32_t
 * RETURNS: void
 * NOTE: 
 * 1) SPIxTXBUF and SPIxRXBUF are mere instances of SPIxBUF. Both transmit and recieve buffer share the address with SPIxBUF.
 *    SPIxBUF is only accessible buffer to user level. 
 * 2) Transmission is immediately started once data is filled in register
 * 3) Clock is triggered only when data is trasnmitted
 */

void SPI1_transfer(uint32_t data)
{
    //Wait till the Transmit buffer empty flag is set (indicating the data is shifted out to Shift register)
    while(SPI1STATbits.SPITBE == 0);
    
    //Write data on SPIxBUF
    SPI1BUF = data;     //0xFB;//                    //store the data to be transmitted in transmit buffer 
}

/*
 * FUNCTION NAME: SPI2_recieve
 * DESCRIPTION: Function to read the recieved data on SDI pin. This data is stored in a variable and transmitted via UART for display purpose
 * ARGUMENTS: None
 * RETURNS: void
 * NOTE: Recieve is done in polling mode
 */

void SPI2_recieve()
{
    while(SPI2STATbits.SPIRBF)      //This bit if set indicates that the receive buffer is full. Hardware will clear this bit once software reads the buffer
    {
        rx_data_arr[idx] = SPI2BUF;
        idx++;
//            if((rx_data & 0x000000FF) == 0xFF)
//            {
//                togg_val1 = ~togg_val1;
//                WRITE_TO_RE1 = togg_val1;                //write can be 0 or 1
//                //stringising the received data and transmit via UART
////                sprintf(disp_str,"I recieved %x, i am sending => %x\r\n",rx_data_arr[idx],0xBB);
////                tx_UART1(disp_str);        
//            }
//            else
//            {
//                togg_val2 = ~togg_val2;
//                WRITE_TO_RC13 = togg_val2;                //write can be 0 or 1                
//                //stringising the received data and transmit via UART
////                sprintf(disp_str,"I recieved %x, i am sending => %x\r\n",rx_data_arr[idx],0xCC);
////                tx_UART1(disp_str);       
//            }
            
        if(idx >= 99)
        {
            idx = 0;
        }
    }
}

/*
 * FUNCTION NAME: delay
 * DESCRIPTION: Implement delay functionality by iterating a finite for loop
 * ARGUMENTS: None
 * RETURNS: void
 */

void delay()
{
    for(int i = 0; i < DEF_DELAY; i++);
}

int main(int argc, char** argv) 
{
    //init();
    //init_UART1();
    SPI1_master_mode_init();
    SPI2_slave_mode_init();              //set up the SPI module for the Slave mode operation
    common_gpio_init();
    
    while(1)
    {
        //Note: Operating CS pin manually by port pin functionality is valid only if MSSEN = 0 (Slave select functionality is disabled)
        
        //CS_PIN = 0;                 //Pull CS pin low. 
        SPI1_transfer(++tx_data);
        delay();
        SPI2_recieve();
        //CS_PIN = 1;                 //Leave CS pin high
    }
    
    return (EXIT_SUCCESS);
}

//ISR for SPI2
void __ISR(_SPI2_RX_VECTOR,IPL4AUTO) SPI2RX_Handler(void)
{
    //if(IFS4bits.SPI2RXIF == 1)
    //{
        //LATEbits.LATE1 = ~LATEbits.LATE1;
        isr_count++;
        rx_data = SPI2BUF;                        //read recieve register
//        sprintf(disp_str,"RX_INT_data = %x\r\n",rx_data);
//        tx_UART1(disp_str);
        IFS4bits.SPI2RXIF = 0;                  //clear the set flag
    //}
}

/*
 * NOTES:
 * 1) Calculation of Frequency (SCK)
 * Frequency (SCK) = (Frequency of Peripheral clock) / [2 * (SPIxBRG + 1)]
 * 
 * 2) Clock is not running continuously. It is only clocked when there is transfer of data between SPIxTXREG and SPIxRXREG
 * 
 * 3) Data on UxTREG register cannot be seen in debug mode
 */


