#include "include_ez430.h"
#include "msp430x22x4.h"
#include "stdint.h"
#include "wireless-v2.h"

//------------------------------------------------------------------------------
// Tx.c
// Receive Analog signal and input into ADC to convert to Digital data.
// Send Digital data to wireless card and transmit to Receiver card.
// 4/8/19
//------------------------------------------------------------------------------

/*
 * Clock needs better config (clock for cc2250 and then maybe use it for ADC10 as well)
 * ADC10 needs better config (continuous sampling, sample hold times)
 * I think Wireless settings are okay but I havent checked
 * Also could sample on timer interupt and sync sample with transmit
 * or could sample constantly and send out packet every sample (if wireless could keep up)
 */

union {
    uint16_t u16;
    uint8_t[2] u8;
} adcValue;

#define CHANNEL 0

#pragma vector=TIMERA0_VECTOR
__interupt void TimerISR(void)
//------------------------------------------------------------------------------
// Func:  Read value of ADC and send out to cc2250
// Args:  None
// Retn:  None
//------------------------------------------------------------------------------
{
    while(!(ADC10CTL0 & ADC10IFG)){};   // wait for conversion to be ready
    adcValue.u16 = ADC10MEM;            // read value of ADC
    P1OUT ^= 0x03;                 // toggle LED

    uint8_t pktLen = 3;
    uint8_t pkData = {0x02, adcValue.u8[1], adcValue.u8[0]};  // set packets
    // IDK what order these values should be sent I think as long as we are
    // consistent it should be fine

    RFSendPacket(pktData, pktLen);   // Activate TX mode & transmit packet

    TI_CC_SPIStrobe(TI_CCxxx0_SIDLE); // Set cc2250 to IDLE mode
                                      // Tx mode re-activates in RFSendPacket
    ADC10CTL0 &= ~(ADC10IFG);         // clear conversion ready flag
    __bic_SR_register_on_exit(LPM0_bits);  // Clr previous Low Pwr bits on stack
}



void Setup()
//-----------------------------------------------------------------------------
// Func:  Setup MSP430 Ports & Clocks, and reset & config cc2500 chip
// Args:  none
// Retn:  none
//------------------------------------------------------------------------------
{
    volatile uint16_t delay;

    for(delay=0; delay<650; delay++){};   // Empirical: cc2500 Pwr up settle

    //Crystal Setup
    DCOCTL  = CALDCO_8MHZ;      // DCO = 8 MHz;
    BCSCTL1 = CALBC1_8MHZ;      // DCO = 8 MHz;
    BCSCTL2 |= DIVS_3;          // SMCLK = MCLK/8 = 1MHz

    // ADC Setup
    ADC10AE0  |= 0x01;          // Enable chnl A0 = P2.0;
    ADC10CTL0 |= ADC10ON        // Turn ON ADC10
              //|  ADC10IE        // enable ADC IRQ
              |  ADC10SHT_2;    // samp-hold tim = 16 cyc;
    ADC10CTL1 |= ADC10SSEL_3    // ADC10CLK source = SMCLK
              |  ADC10DIV_3     // ADC10CLK divider = 4
              |  INCH_0;        // Select input = chnl A0 (default);

    //TODO: Clock needs to have better timing
    // Timer Config
    TACTL   = TASSEL_2 |ID_3 | MC_1;      // TA uses SMCLK/8, in Up mode
    TACCR0  = 60000;                      // ~480 msec @  1/8 MHz
    TACCTL0 = CCIE;		                // enable TA CCR0 IRQ

    // LED Port config
    P1DIR |=  0x03;                       // Set LED pins to output
    P1OUT &= ~0x03;                       // Clear LEDs

    // Wireless Initialization
    P2SEL = 0;                            // P2.6, P2.7 = GDO0, GDO2 (GPIO)
    TI_CC_SPISetup();                     // Init SPI port for cc2500
    TI_CC_PowerupResetCCxxxx();           // Reset cc2500
    writeRFSettings();                    // Put settings to cc2500 config regs

    TI_CC_SPIWriteReg(TI_CCxxx0_CHANNR,  CHANNEL);  // Set Your Own Channel Number
                                              // only AFTER writeRFSettings

    for(delay=0; delay<650; delay++){};   // Empirical: Let cc2500 finish setup

    P1OUT = 0x02;                         // Setup done => Turn on green LED
}

void main ()
{
    WDTCTL  = WDTPW + WDTHOLD;  // Stop Watchdog (Yawn)
    Setup();                    // initialize ADC and cc2250
    while(1) {
        ADC10CTL0 |= ENC | ADC10SC;           // Enable Conversion
        __bis_SR_register(LPM0_bits + GIE);   // Sleep until interupt from ADC
    }

}