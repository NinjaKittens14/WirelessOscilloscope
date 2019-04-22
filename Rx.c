// ========================================================================= BOF
// FILE FUNC:  receive ADC Data over the cc2500, then send it via SPI to the
//             DAC.
// -----------------------------------------------------------------------------

#include "stdint.h"                // MSP430 data type definitions
#include "msp430x22x4.h"           // Mammoth MSP430-specific macros & defs
#include "wireless-v2.h"           // Mammoth cc2500 setup & function defs

union {
    uint16_t u16;
    uint8_t u8[2];
} adcValue;

#define CHANNEL 9

//------------------------------------------------------------------------------
// Func:  Packet Received ISR:  triggered by falling edge of GDO0.
//        Parses pkt & prints entire packet data bytes to the host PC.
// Args:  None
// Retn:  None
//------------------------------------------------------------------------------
#pragma vector=PORT2_VECTOR        // cc2500 pin GDO0 is hardwired to P2.6
__interrupt void PktRxedISR(void)
{
    // Buffer Len for rxPkt = only data bytes;
    // pkt size byte not incl b/c it is stripped off by RFReceivePacket func.
    // IF address check was enabled, THEN would need 1 more byte for device addr

    static uint8_t len = 2;          // Packet Len = 3 bytes (data only)
    uint8_t status[2];               // Buffer to store pkt status bytes
    uint8_t rxPkt[len];

    if(TI_CC_GDO0_PxIFG & TI_CC_GDO0_PIN)            // chk GDO0 bit of P2 IFG Reg
    {
        P1OUT ^= 0x01;								   // Toggle LED
        RFReceivePacket(rxPkt, &len, status);  		   // Get packet from cc2500
    }
    TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;             // Reset GDO0 IRQ flag

    TI_CC_SPIStrobe(TI_CCxxx0_SIDLE);      // Set cc2500 to idle mode.
    TI_CC_SPIStrobe(TI_CCxxx0_SRX);        // Set cc2500 to RX mode.
                                           // AutoCal @ IDLE to RX Transition

    // Parse rxPkt into adcValue
    adcValue.u8[1] = rxPkt[1];			  // MSb
    adcValue.u8[0] = rxPkt[0];			  // LSb

    // Wait for SPI Bus
    while(UCB0STAT & UCBUSY){};       // Wait for SPI Bus to be clear
    P4OUT &= ~0x20;                   // Assert SS (active low)

    UCB0TXBUF = adcValue.u8[1];    // send byte
    while(!(IFG2 & UCB0TXIFG)){};  // wait for TxBUF to be empty
    UCB0TXBUF = adcValue.u8[0];    // send byte
    while(!(IFG2 & UCB0TXIFG)){};  // wait for TxBuf to be empty

    while (UCB0STAT & UCBUSY){};     // wait for quiet bus
    P4OUT |= 0x20;                   // de-assert SS (active low)
}

//------------------------------------------------------------------------------
// FUNC:  Setup MSP430 Ports & Clocks, reset & config cc2500
// ARGS:  none
// RETN:  none
//------------------------------------------------------------------------------
void SetupAll(void)
{   
    volatile uint16_t delay;

    for(delay=0; delay<650; delay++);     // Empirical: cc2500 Pwr up settle

    // set up clock system
    BCSCTL1 = CALBC1_8MHZ;                // set DCO = 8MHz
    DCOCTL  = CALDCO_8MHZ;                // set DCO = 8MHz
    BCSCTL2 |= DIVS_0;                    // SMCLK = MCLK/8 = 1MHz

    // Config built-in LED
    P1DIR |= 0x01;						// P1.0 = output
    P1OUT &= ~0x01;						// Set LED as Off

    // Config SMCLK Output to pin P2.1
    P2SEL |= 0x02;						// P2.1 alt mode (SMCLK Output)
    P2DIR |= 0x02;						// P2.1 Output dir

    // Config. USCA0 for SPI Master
    P3SEL = 0x10;                         // P3.4 alt modes (MOSI & SCLK)
    P3DIR = 0x10;                         // P3.4 output dir
    // Leave P3.0 alone for UCB RF SPI SS*, use SMCLK output pin P2.1 for UCLK

    P4SEL &= ~0x20;                       // P4.5 GPIO (SS)
    P4DIR |= 0x20;                        // P4.5 Output dir
    P4OUT |= 0x20;                        // clear SS (active low)

    UCA0CTL0 |= UCSYNC | UCMST | UCMSB;   // synchronous, 3-pin Master, MSB first
    UCA0CTL1 |= UCSSEL_2;        			// SCLK source = SMCLK (8sMHz)
    UCA0CTL1 &= ~UCSWRST;

    // Wireless Initialization
    P2SEL = 0;                            // P2.6, P2.7 = GDO0, GDO2 (GPIO)
    TI_CC_SPISetup();                     // Initialize SPI port
    TI_CC_PowerupResetCCxxxx();           // Reset cc2500
    writeRFSettings();                    // Send RF settings to config regs
    UCB0BR0 = 0x02;                       // UCLK/2 = SMCLK/2 = 4MHz

    TI_CC_GDO0_PxIES |=  TI_CC_GDO0_PIN;  // IRQ on GDO0 fall. edge (end of pkt)
    TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;  // Clear  GDO0 IRQ flag
    TI_CC_GDO0_PxIE  |=  TI_CC_GDO0_PIN;  // Enable GDO0 IRQ
    TI_CC_SPIStrobe(TI_CCxxx0_SRX);       // Init. cc2500 in RX mode.

    TI_CC_SPIWriteReg(TI_CCxxx0_CHANNR, CHANNEL);    // Set Your Own Channel Number
                                               // AFTER writeRFSettings (???)

    for(delay=0; delay<650; delay++);     // Empirical: Let cc2500 finish setup
}

// -----------------------------------------------------------------------------
void main()
{   WDTCTL = WDTPW + WDTHOLD;         // halt watchdog

    volatile uint8_t rxPkt[3];        // Buffer to store received pkt payload
    SetupAll();                       // Setup clocks, ports & cc2500 settings

    __bis_SR_register(CPUOFF + GIE);  // sleep til Pkt RX
}

// ========================================================================= EOF