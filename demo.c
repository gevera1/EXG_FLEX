/************************************************************************
 * File:            demo.c
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
 * [See MSP430f552x Mixed-Signal uC datasheet (rev.P)]
 *
 * P3.0     - UCB0SIMO      (I2C SDA)
 * P3.1     - UCB0SCLK      (I2C SCLK)
 *
 * P3.3     - UCA0TXD       (UART TX)
 *
 * P1.1     - EXGIFG            (ExG Interrupt)
 * P6.2-6.0 - EXG_DATA[0:2]-+   (ExG Data)
 * P2.6-2.0 - EXG_DATA[3:9]-+
 *
 *+========================== EXG LAYOUT ===================================+
 *|   CLK [1]    INT [1]                DATA [10]                           ||
 *| +-------------------------------------------------------------------+   ||
 *| | D6(P1.7) | D2(P1.1) |  E3 - E0 (P6.2-0,P2.6), C5 - C0 (P2.5-0)    |   ||
 *| +-------------------------------------------------------------------+   ||
 *+=========================================================================+|
 *
 */
/***********************************************************************/

/*      INCLUDES        */
#include <main.h>

/*      DEFINES         */


/*      MACROS          */

/* GLOBAL VARIABLES     */
static  volatile char   EXG_PKT[EXGn];  // 2-Byte packet
static  volatile char   ACL_PKT[ACLn];  // 6-Byte packet (SPI)
        volatile char   send_flag = 0;
        volatile char   ACL_CPLT = 0;

        volatile char   DMA_PKT[8]  = {0, 0, 0, 0, 0, 0, 0, 0};

                                        //     -1,          2,        255,        128
        volatile char   TEST_PKT[8] = {0x00, 0xFF, 0x00, 0x02, 0x00, 0xFF, 0x00, 0x80};


/* FUNCTION PROTOTYPES  */
void Init_SYS   ( void );
void Init_TMR   ( void );
void Init_IO    ( void );
void Init_uSCI  ( void );
void Init_DMA   ( void );

void I2C_Send_Byte( uint8_t addr, uint8_t data );
void I2C_Read_Byte( uint8_t addr );

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
 */
int main( void )
{
    Init_SYS();
    Init_TMR();     // Timer0 A1
    Init_IO();
    Init_uSCI();
    Init_DMA();

    /* MENTION TIMING CONSIDERATIONS */
//    TA0CCR0 = (uint16_t) (((uint16_t)32768)-1);      // every 2 s
//    TA0CCR0 = (uint16_t) (((uint16_t)32768/2)-1);   // every 1 s
    TA0CCR0 = (uint16_t) (((uint16_t)32768/10)-1);   // every 0.25 s
//    TA0CCR0 = (uint16_t) (328-1);    // every 0.005 s
//    TA0CCR0 = (uint16_t) (100-1);    // every clock cycle

    __low_power_mode_3();

    while(1);

    return EXIT_FAILURE;
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

    P5SEL   |= BIT4 | BIT5; // 32.768 kHz ON

    UCSCTL4 |= SELA_0;  // ACLK using XT1CLK
    UCSCTL5 |= DIVPA_0 | DIVA_0;    // ACLK/1 = ACLK

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
    TA0CCR0 = 0;
    TA0CTL  |= TASSEL_1 | MC_1 | TAIE;

    return;
}

/*
 * TMR ISR
 *
 */
#pragma vector = TIMER0_A1_VECTOR
__interrupt void TMR_ISR( void )
{
    uint8_t i;
    TA0CTL  &= ~TAIFG;          // Clear interrupt flag
    switch (send_flag)
    {
    case (0):   // Fudge Data   and DMA!
            for (i = 0; i < 8; i++)
            {
                DMA_PKT[i] = (char) (rand() % 256); // 0 - 255
                if (i == 0)
                    DMA_PKT[i] &= 0x03; // | 0b00000011
            }

            send_flag ^= 0x01;
            break;

    case (1):   // Send Data
            while (!ACL_CPLT);  // TEST_PKT NOT UPDATED!

            P4OUT   ^= BIT7;        // LED2 ON
            for (i = 0; i < 8; i++)
            {
                while (!(UCA0IFG&UCTXIFG));
                UCA0TXBUF = (char) TEST_PKT[i];
            }

            send_flag ^= 0x01;
            ACL_CPLT = 0;
            P4OUT   ^= BIT7;        // LED2 OFF
            break;
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
 *      Init_EXG();
 *
 *  Notes:
 *      PINOUT
 *      ======
 * [See MSP430f552x Mixed-Signal uC datasheet (rev.P)]
 *
 * P3.0     - UCB0SIMO      (I2C SDA)
 * P3.1     - UCB0SCLK      (I2C SCLK)
 *
 * P3.3     - UCA0TXD       (UART TX)
 *
 * P1.0     - EXGCLK            (ExG Clk. src.)
 * P1.1     - EXGIFG            (ExG Interrupt)
 * P6.2-6.0 - EXG_DATA[0:2]-+   (ExG Data)
 * P2.6-2.0 - EXG_DATA[3:9]-+
 *
 *+========================== EXG LAYOUT ===================================+
 *|   CLK [1]    INT [1]                DATA [10]                           ||
 *| +-------------------------------------------------------------------+   ||
 *| | D6(P1.0) | D2(P1.1) |  E3 - E0 (P6.2-0,P2.6), C5 - C0 (P2.5-0)    |   ||
 *| +-------------------------------------------------------------------+   ||
 *+=========================================================================+|
 *
 */
void Init_IO( void )
{
    P4DIR   |= BIT7;    // LED
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

/*
 * ExG ISR
 *
 */
#pragma vector = PORT1_VECTOR
__interrupt void ExG_ISR( void )
{
    int i;
    switch(__even_in_range(P1IV, 4))
    {
    case 4: // ExG Interrupt Detected
        EXG_PKT[0]  = (P2IN & (0x7F)) & ( (P6IN & 1) << 7 );  // Read BIT10 - BIT03
        EXG_PKT[1]  = (P6IN & (0x06)) >> 1;  // Read BIT02 and BIT01

        for (i = 1; i != 0; i--)
        {
            while (UCB0CTL1 & UCTXIFG);
            UCB0TXBUF   = EXG_PKT[i];
        }
        while (UCB0CTL1 & UCTXIFG);
        UCB0TXBUF   = EXG_PKT[0];
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
    /*============Configure uSCI A0 in UART mode============*/
    /* UCA0TX (PICO) [SIMO] */
    UCA0CTL1    |= UCSWRST;       // Set SW reset bit
                                            // LSB first;

    /* BAUD: 9600; 32 kHz (from Table 36-4) */
    UCA0CTL1    |= UCSSEL_1;    // ACLK
    UCA0BR0     |= 3;
    UCA0BR1     |= 0;
    UCA0MCTL    |= UCBRS_3| UCBRF_0;

    /* Finish uSCI A0 Init */
    UCA0CTL1    &= ~UCSWRST;   // Clear SW reset bit


    /*============Configure uSCI B0 in I2C mode============*/

    /* First byte after START is a 7-bit peripheral address */

    /* TO DO: PULL UP SDA AND SCL WITH 10K RESISTORS! */
//    __disable_interrupt();


//    UCB0CTL1    |= UCSWRST;     // ENABLE SW Reset
//    UCB0CTL0    = UCMST | UCMODE_3 | UCSYNC;
//    UCB0CTL1    |= UCSSEL_1;   // ACLK
//    UCB0BR0     = 1;                    // fSCL = ACLK/1 = 32.768 kHz
//    UCB0BR1     = 0;
//
//    UCB0CTL1    &= ~UCSWRST;
//
//    /* WAKE UP MPU-6050 */
//    UCB0I2CSA   = MPU_6050_ADDR;
//
//    while (UCB0CTL1 & UCTXSTP);
//    UCB0CTL1    |= UCTR | UCTXSTT;
//    while (!(UCB0IFG & UCTXIFG));
//    UCB0TXBUF   = SIG_PTH_RST;
//    while (!(UCB0IFG & UCTXIFG));
//    UCB0TXBUF   = (uint8_t) 0x07;
//    while (!(UCB0IFG & UCTXIFG));
//    UCB0CTL1    |= UCTXSTP;
//    UCB0IFG     &= ~UCTXIFG;

//    WHO AM I TEST
//    while (UCB0CTL1 & UCTXSTP);
//    UCB0CTL1    |= UCTR | UCTXSTT;
//    while (!(UCB0IFG & UCTXIFG));
//    UCB0TXBUF   = (uint8_t) WHO_AM_I;
//    while (!(UCB0IFG & UCTXIFG));
//    UCB0CTL1    &= ~UCTR;
//    UCB0CTL1    |= UCTXSTT | UCTXNACK;
//    while (UCB0CTL1 & UCTXSTT);
//
//    while (!(UCB0IFG & UCRXIFG));
//    test_byte = UCB0RXBUF;
//
//    UCB0CTL1    |= UCTXSTP;

    return;
}

/*
 * UART ISR
 *
 */
#pragma vector = USCI_A0_VECTOR
__interrupt void UART_uSCI_ISR( void )
{
//    switch(__even_in_range(UCA0IV, 4))
//    {
//    case 2:
//        break;
//    case 4:
//        break;
//    default:
//        break;
//    }
}

/*
 * I2C ISR
 *
 */
#pragma vector = USCI_B0_VECTOR
__interrupt void I2C_uSCI_ISR( void )
{
    uint8_t ind;


    switch(__even_in_range(UCB0IV, 10))
    {
    case 4:     // NACK (SHOULDN'T GET HERE)
//        UCB0CTL1    |= UCSWRST | UCTXSTP;     // STOP signal
//        UCB0CTL1    &= ~UCSWRST;
        break;
    case 10:    // RX
        ind = (UCB0I2CSA - ACL_START);
        ACL_PKT[ind] = UCB0RXBUF;

        if (UCB0I2CSA == ACL_STOP)   { UCB0I2CSA = ACL_START; } // Reset addr.
        else                         { UCB0I2CSA++; }   // Increment addr.
        break;
    case 12:    // TX
        UCB0CTL1    |= UCTXSTP;
        UCB0IFG     &= ~UCTXIFG;
        break;
    default:
        break;
    }
}

/******************************************************************************
 * DMA FUNCTIONS
 *****************************************************************************/
void Init_DMA( void )
{
    /* Configure DMA channels */
//    DMACTL0     = DMA0TSEL_18;      // DMA0 triggered by UCRXIFG
    DMACTL0     = DMA1TSEL_2;       // DMA1 triggered by Timer
//
//    /* Configure DMA0 */
//    DMA0CTL     = DMADT_4 | DMADSTINCR0 | DMADSTINCR1 | DMADSTBYTE | DMASRCBYTE | DMAIE;
//            // Repeated single transfer; Dest addr. incremented; Byte; Interrupt enable
//    DMA0SZ      = ACLn; // DMA0 (ACL): 3 Words (48b)
//    __data20_write_long((uint32_t) &DMA0SA, (uint32_t) &UCB0I2CSA);     // SRC: UCB0I2CSA
//    __data20_write_long((uint32_t) &DMA0DA, (uint32_t) &ACL_PKT[0]);    // DST: ACL_PKT

    /* Configure DMA1 */
    DMA1CTL     = DMADT_5 | DMASRCINCR1 | DMASRCINCR0 | DMADSTINCR1 | DMADSTINCR0 | DMADSTBYTE | DMASRCBYTE | DMAIE;
            // Repeated block transfer
    DMA1SZ      = ACLn + EXGn;  // 8 bytes
    __data20_write_long((uint32_t) &DMA1SA, (uint32_t) DMA_PKT);     // SRC: DMA_PKT
    __data20_write_long((uint32_t) &DMA1DA, (uint32_t) TEST_PKT);    // DST: TEST_PKT

    /* Enable DMAs */
    DMA1CTL     |= DMAEN;
}

/*
 * DMA ISR
 */
#pragma vector = DMA_VECTOR
__interrupt void DMA_ISR( void )
{
    switch(__even_in_range(DMAIV, 4))
    {
    case 0: break;
    case 2:
    case 4: // I2C buffer filled; 3 Bytes
        if (ACL_CPLT) break;
        else
            ACL_CPLT    = 1;
    default:
        break;
    }
}
