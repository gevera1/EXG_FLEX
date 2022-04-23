/************************************************************************
 * File:            main.c
 *
 * Authors:         Gevero, Adrian-James
 *
 * Date Created:    02-26-2022
 * Last Updated:    04-22-2022
 *
 * Summary:
 *  Main source program file, implementing new scheduling
 *  techniques, and properly interfacing with peripherals.
 *  Adapted from Jason Case's previous source code.
 *
 * Architecture:
 *  MSP4305229
 *
 * Notes:
 *  In order to facilitate moving away from harmful/
 *  discriminatory terminology, this project will document
 *  any serial communications protocols (SPI, I2C, etc.)
 *  using the terms "Controller and Peripheral" to sub out
 *  for the out-dated "Slave and Master" terminology.
 *
 *      Reference:
 *      MOSI    = COPI
 *      MISO    = CIPO
 *      SIMO    = PICO
 *      SOMI    = POCI
 *
 *
 * PINOUT
 * ======
 * ( CAN BE ALTERED AT DISCRETION OF HARDWARE SUBSYSTEM )
 * [See MSP430f552x Mixed-Signal uC datasheet (rev.P)]
 *
 * P3.0     - UCB0SIMO      (I2C SDA)
 * P3.1     - UCB0SCLK      (I2C SCLK)
 *
 * P3.3     - UCA0TXD       (UART TX)
 *
 * P1.0     - EXGCLK
 * P1.1     - EXGIFG            (ExG Interrupt)
 * P6.2-6.0 - EXG_DATA[0:2]-+   (ExG Data)
 * P2.6-2.0 - EXG_DATA[3:9]-+
 *
 *+========================== EXG PINOUT ===================================+
 *|   CLK [1]    INT [1]                DATA [10]                           ||
 *| +-------------------------------------------------------------------+   ||
 *| | D6(P1.7) | D2(P1.1) |  E3 - E0 (P6.2-0,P2.6), C5 - C0 (P2.5-0)    |   ||
 *| +-------------------------------------------------------------------+   ||
 *+=========================================================================+|
 *
 *+========================= PACKET LAYOUT ===================================+
 *|     ExG [2]         ACL_X [2]          ACL_Y [2]        ACL_Z [2]         ||
 *| +-----------------------------------------------------------------------+ ||
 *| | ExG_H, ExG_L | ACL_X_H, ACL_X_L | ACL_Y_H; ACL_Y_L | ACL_Z_H; ACL_Z_L | ||
 *| +-----------------------------------------------------------------------+ ||
 *|                       --------TIME------->                                ||
 *+===========================================================================+|
 *  Formatted Data: "EXG,ACL_X,ACL_Y,ACL_Z" (csv)
 *
 */
/***********************************************************************/

/*      INCLUDES        */
#include <main.h>

/*      DEFINES         */


/*      MACROS          */

/* GLOBAL VARIABLES     */
volatile        char    EXG_CPLT, ACL_CPLT, ACL_ADDR;
static volatile char    DAT_PKT[EXGn + ACLn];    // 2-byte packet (Front-end IC) + 6-byte packet (SPI)
                                                    // = 8-byte packet
    /* Demo Structures */
volatile        char    RAND_PKT[8] = {0};    

/* FUNCTION PROTOTYPES  */
void Init_SYS   ( void );
void Init_TMR   ( void );
void Init_IO    ( void );
void Init_uSCI  ( void );
void Init_DMA   ( void );

void I2C_write  ( uint8_t addr, char val );
char I2C_read   ( uint8_t addr );
void I2C_control_recv   ( uint8_t start_addr );

/******************************************************************************
 * MAIN FUNCTION
 *****************************************************************************/
/*
 *  main( void )
 *
 *  Description:
 *      Primary control function.
 *
 *  Returns:
 *      (int) value of exit code
 *
 *  Parameters:
 *      N/A
 *
 *  Usage: Performed automatically by uC
 *
 *  Notes:
 *      - Architecture is 16-bit, so int will be an 16-bit value (WORD)
 *      - Any returns are an indication of catastrophic failure
 *      - All data manipulations are handled by DMA and ISRs
 */
int main( void )
{
    __disable_interrupt();          // Disables interrupts

    /* Initialization functions */
    Init_SYS();     // Ports, clock, global variables
    Init_TMR();     // Timer0 A1
    Init_IO();      // UART, I2C, ExG
    Init_uSCI();    // UART, I2C
    Init_DMA();     // I2C DMA

    /* Interrupt every ~1 ms */
    TA0CCR0 = (uint16_t) (TMR_PERIOD-1);    // Start timer count up

    __low_power_mode_3();   // Enables interrupts and enter LPM3

    while(1);               // Infinite Loop

    return EXIT_FAILURE;    // If we got here, something TERRIBLE has happened
}

/******************************************************************************
 * SYSTEM INITIALIZATION FUNCTION
 *****************************************************************************/
/*
 *  Init_SYS()
 *
 *  Description:
 *      Performs necessary operations to configure system to the
 *      Low-Power Operating Mode. Setup timers, interrupts, and
 *      clocks
 *
 *  Returns:
 *      void
 *
 *  Parameters:
 *      N/A
 *
 *  Usage:
 *      Init_SYS();
 *
 *  Notes:
 *      N/A
 */
void Init_SYS( void )
{
    WDTCTL  = WDTPW | WDTHOLD;      // Kill Watchdog Timer

    /* Initialize Ports to Optimize power usage */
    PADIR=0xFFFF; PAOUT=0x0000;     // Ports 1 and 2
    PBDIR=0xFFFF; PBOUT=0x0000;     // Ports 3 and 4
    PCDIR=0xFFFF; PCOUT=0x0000;     // Ports 5 and 6
    PDDIR=0xFFFF; PDOUT=0x0000;     // Ports 5 and 6
    PJDIR=0xFFFF; PJOUT=0x0000;     // JTAG Port

    P5SEL   |= BIT4 | BIT5;         // 32.768 kHz ON

    UCSCTL4 |= SELA_0;              // ACLK using XT1CLK
    UCSCTL5 |= DIVPA_0 | DIVA_0;    // ACLK/1 = ACLK

    EXG_CPLT = 0;
    ACL_CPLT = 0;        // Interrupt Flags

    return;
}

/******************************************************************************
 * TIMER FUNCTIONS
 *****************************************************************************/
/*
 *  Init_TMR()
 *
 *  Description:
 *
 *  Returns:
 *      void
 *
 *  Parameters:
 *      N/A
 *
 *  Usage:
 *      Init_TMR();
 *
 *  Notes:
 */
void Init_TMR( void )
{
    TA0CCR0 = 0;                        // No special configs
    TA0CTL  |= TASSEL_1 | MC_1 | TAIE;  // Setup ACLK

    return;
}


//======================================================================
// TIMER INTERRUPT SERVICE ROUTINE
//======================================================================
#pragma vector = TIMER0_A1_VECTOR
__interrupt void TMR_ISR( void )
{
    uint8_t i;
    TA0CTL  &= ~TAIFG;          // Clear interrupt flag
    if (EXG_CPLT && ACL_CPLT)   // Is Packet Filled?
    {
        P4OUT   ^= BIT7;    // LED2 ON
        for (i = 0; i < (EXGn+ACLn); i++)   // Send data packet over UART
        {
            while (!(UCA0IFG&UCTXIFG));     // Wait for UART TX to free
            UCA0TXBUF = DAT_PKT[i];         // Send byte
        }
        P4OUT   ^= BIT7;    // LED2 OFF
        EXG_CPLT = ACL_CPLT = 0;
    }
}

/******************************************************************************
 * GPIO FUNCTIONS
 *****************************************************************************/
/*
 *  Init_IO()
 *
 *  Description:
 *
 *  Returns:
 *      void
 *
 *  Parameters:
 *      N/A
 *
 *  Usage:
 *      Init_IO();
 *
 *  Notes:
 *      - SEE PINOUT
 */
void Init_IO( void )
{
    P4DIR   |= BIT7;        // LED
    P4OUT   &= ~BIT7;

    /* UART TX */
    P3SEL   |= BIT3;        // Set 3.3 to Output
    P3DIR   |= BIT3;

    /* I2C */
    P3SEL   |= BIT0 | BIT1; // Select 3.0 (SDA) + 3.1 (SCL)

    /* ExG Clock */
    P1DIR   |= BIT0;        // Set 1.0 to ACLK (Output)
    P1SEL   |= BIT0;

    /* ExG Interrupt */
    P1DIR   &= ~(BIT1);     // Set 1.1 to Input (Interrupt)
    P1IE    |= BIT1;

    /* ExG Data */
    P6DIR   &= ~(BIT2 | BIT1 | BIT0);                               // Set 6.2-0 to Input
    P2DIR   &= ~(BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0);   // Set 2.6-0 to Input

    return;
}

//======================================================================
// I/O (ExG) INTERRUPT SERVICE ROUTINE
//======================================================================
#pragma vector = PORT1_VECTOR
__interrupt void ExG_ISR( void )
{
    switch(__even_in_range(P1IV, 4))
    {
    case 4: // ExG Interrupt Detected
        if (EXG_CPLT) break;    // ExG data still in packet

        DAT_PKT[EXG_L_IND]  = (P2IN & (0x7F)) & ( (P6IN & 1) << 7 );  // Read BIT10 - BIT03
        DAT_PKT[EXG_H_IND]  = (P6IN & (0x06)) >> 1;  // Read BIT02 and BIT01
        EXG_CPLT    = 1;        // Set Flag
        break;

    default:
        break;
    }
}

/******************************************************************************
 * uSCI FUNCTIONS
 *****************************************************************************/
/*
 *  Init_uSCI()
 *
 *  Description:
 *      Configures and enables USCI_A0 in UART Mode and """_B0 in SPI/I2C Mode.
 *
 *  Returns:
 *      void
 *
 *  Parameters:
 *      N/A
 *
 *  Usage:
 *      Init_uSCI();
 *
 *  Notes:
 *      N/A
 */
void Init_uSCI( void )
{
    /*============Configure uSCI A0 in UART mode============*/      /* WORKING! */
    /* UCA0TX (PICO) [SIMO] */
    UCA0CTL1    |= UCSWRST;         // Set SW reset bit
                                    // LSB first;

    /* BAUD: 9600; 32 kHz (from Table 36-4) */
    UCA0CTL1    |= UCSSEL_1;        // ACLK
    UCA0BR0     |= 3;
    UCA0BR1     |= 0;
    UCA0MCTL    |= UCBRS_3| UCBRF_0;

    /* Finish uSCI A0 Init */
    UCA0CTL1    &= ~UCSWRST;        // Clear SW reset bit

    /*============Configure uSCI B0 in I2C mode============*/   /* TO DO: Fix HW */
    // "Peripheral Mode; UCxPICO [SIMO] is the Data Input"
    UCB0CTL1    |= UCSWRST;         // Set SW reset bit

    /* Enable I/O ports */
    P3SEL       |= BIT0 | BIT1;     // P3.0 (SDA) + P3.1 (SCLK)

    /* I2C mode */
    UCB0CTL0    = UCMST | UCMODE_3 | UCSYNC;    // Controller I2C Mode
    UCB0CTL1    |= UCSSEL_1;                    // Choose 32kHz ACLK
    UCB0BR0     = 1;                            // ACLK/1 = ACLK
    UCB0BR1     = 0;

    /* Finish uSCI B0 Init */
    UCB0CTL1    &= ~UCSWRST;        // Clear SW reset bit
    UCB0I2CSA   = MPU_6050_ADDR;

    I2C_write(PWR_MGMT_1, 0);
    I2C_write(SIG_PTH_RST, 0x07);
    I2C_write(PWR_MGMT_1, (CLKSEL_4 | TEMP_DIS));
    I2C_write(PWR_MGMT_2, STBY_G);

    I2C_control_recv(ACCEL_STT);

    UCB0IE      = UCRXIE;           // Enable RX interrupts

    return;
}

void I2C_write( uint8_t addr, char val )
{
    while (UCB0STAT & UCBUSY);
    UCB0CTL1    |= UCTXSTT | UCTR;

    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF   = addr;
    while (!(UCB0IFG & UCTXIFG));
    UCB0TXBUF   = val;

    UCB0CTL1    |= UCTXSTP;
    while (UCB0CTL1 & UCTXSTP);
}

char I2C_read( uint8_t addr )
{
    char data = 0;

    while(UCB0STAT & UCBUSY);
    UCB0CTL1 |= UCTXSTT + UCTR;         //Start Condition sends address of slave

    while (!(UCB0IFG & UCTXIFG));     //Wait until the data has been shifted out
    UCB0TXBUF = addr;                   //Load address of slave register
    while (UCB0CTL1 & UCTXSTT);         //Wait until the acknowledge message has been received

    while (UCB0STAT & UCBUSY);
    UCB0CTL1 &= ~UCTR;                  //Sets the master as a receiver
    UCB0CTL1 |= UCTXSTT;                //Start Condition sends address of slave
    while (UCB0CTL1 & UCTXSTT);

    UCB0CTL1 |= UCTXSTP;                //Send stop message

    while (!(UCB0IFG & UCRXIFG));
    data = UCB0RXBUF;                   //Save data to memory

    while (UCB0CTL1 & UCTXSTP);

    return data;
}


void I2C_control_recv( uint8_t start_addr )
{
    while(UCB0STAT & UCBUSY);
    UCB0CTL1 |= UCTXSTT + UCTR;         //Start Condition sends address of slave

    while (!(UCB0IFG & UCTXIFG));     //Wait until the data has been shifted out
    UCB0TXBUF = start_addr;                   //Load address of slave register
    while (UCB0CTL1 & UCTXSTT);         //Wait until the acknowledge message has been received

    while (UCB0STAT & UCBUSY);
    UCB0CTL1 &= ~UCTR;                  //Sets the master as a receiver
    UCB0CTL1 |= UCTXSTT;                //Start Condition sends address of slave
    while (UCB0CTL1 & UCTXSTT);
}


//======================================================================
// I2C INTERRUPT SERVICE ROUTINE
//======================================================================
#pragma vector = USCI_B0_VECTOR
__interrupt void I2C_uSCI_ISR( void )
{
    __disable_interrupt();
    switch(__even_in_range(UCB0IV, 10))
    {
    case 4:     // NACK (SHOULDN'T GET HERE)

        UCB0IFG     &= ~UCNACKIFG;
        break;
    case 10:    // RX
        // RECEIVE HANDLED BY DMA!

        /* Change register address */
        if (ACL_ADDR == ACCEL_STP)        // Increment address
            ACL_ADDR    = ACCEL_STT;
        else
            ACL_ADDR++;
        I2C_read(ACL_ADDR);
        break;
    default:
        break;
    }
    __enable_interrupt();
}

/******************************************************************************
 * DMA FUNCTIONS
 *****************************************************************************/
void Init_DMA( void )
{
    /* Configure DMA channels */
    DMACTL0     = DMA0TSEL_18;      // DMA0 triggered by UCRXIFG

    /* Configure DMA0 */
    DMA0CTL     = DMADT_4 | DMADSTINCR0 | DMADSTINCR1 | DMADSTBYTE | DMASRCBYTE | DMAIE;
            // Repeated single transfer; Dest addr. incremented; Byte; Interrupt enable
    DMA0SZ      = ACLn; // DMA0 (ACL): 6 Bytes (48b)
    __data20_write_long((uint32_t) &DMA0SA, (uint32_t) &UCB0RXBUF);         // SRC: UCB0RXBUF
    __data20_write_long((uint32_t) &DMA0DA, (uint32_t) DAT_PKT[ACL_IND]);  // DST: DAT_PKT

    /* Enable DMAs */
    DMA0CTL     |= DMAEN;
}

//======================================================================
// DMA INTERRUPT SERVICE ROUTINE
//======================================================================
#pragma vector = DMA_VECTOR
__interrupt void DMA_ISR( void )
{
    switch(__even_in_range(DMAIV, 2))
    {
    case 0: break;
    case 2: // ACL buffer filled; 6 Bytes
        if (ACL_CPLT) break;
        else
            ACL_CPLT    = 1;
        break;
    default:
        break;
    }
}
