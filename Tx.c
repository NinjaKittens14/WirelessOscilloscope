#include "include_ez430.h"
#include "msp430x22x4.h"
#include "stdint.h"
#include "wireless-v2.h"
#include <stdio.h>

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
    uint8_t u8[2];
} adcValue;

uint8_t i = 0;

#define CHANNEL 5

#pragma vector=TIMERA1_VECTOR
__interrupt void TimerISR(void)
//------------------------------------------------------------------------------
// Func:  Trigger on Timer A Rollover, read value of ADC, and send out to cc2250
// Args:  None
// Retn:  None
//------------------------------------------------------------------------------
{
    switch(__even_in_range(TAIV,10))
    {
        case TAIV_TAIFG:
            while(ADC10CTL1 & ADC10BUSY){};   // wait for conversion to be ready
            adcValue.u16 = ADC10MEM;          // read value of ADC
            
            adcValue.u16 = adcValue.u16 << 2; //shift for 12bit DAC

            uint8_t pktLen = 3;
            uint8_t pktData[3] = {0x02, adcValue.u8[0], adcValue.u8[1]};  // set packets
            
            P1OUT ^= 0x01;                    // Toggle LED
            RFSendPacket(pktData, pktLen);    // Activate TX mode & transmit packet
            TI_CC_SPIStrobe(TI_CCxxx0_SIDLE); // Set cc2250 to IDLE mode
                                              // Tx mode re-activates in RFSendPacket
        
            TACTL &= ~TAIFG;                       // Clear TA flag
            __bic_SR_register_on_exit(LPM0_bits);  // Clr previous Low Pwr bits on stack
            break;
        case TAIV_TACCR1:
        case TAIV_TACCR2:
        default:
    }

}
void Setup()
//-----------------------------------------------------------------------------
// Func:  Setup MSP430 Ports & Clocks, reset & config cc2500 chip,
//        Initialize ADC settings
// Args:  none
// Retn:  none
//------------------------------------------------------------------------------
{
    volatile uint16_t delay;

    for(delay=0; delay<650; delay++){};   // Empirical: cc2500 Pwr up settle

    // Crystal Setup
    DCOCTL  = CALDCO_8MHZ;      // DCO = 8 MHz;
    BCSCTL1 = CALBC1_8MHZ;      // DCO = 8 MHz;
    BCSCTL2 |= DIVS_3;          // SMCLK = MCLK/8 = 1MHz
    
    // Config built-in LED
    P1DIR |= 0x01;              // P1.0 = output
    P1OUT &= ~0x01;             // Set LED as Off

    // ADC Setup
    ADC10AE0  |= 0x01;          // Enable chnl A0 = P2.0;
    ADC10CTL0 |= ADC10ON        // Turn ON ADC10
              |  ADC10SHT_1;    // samp-hold tim = 8 cyc;
    ADC10CTL1 |= ADC10SSEL_3    // ADC10CLK source = SMCLK
              |  ADC10DIV_0     // ADC10CLK divider = 1
              |  INCH_0;        // Select input = chnl A0 (default);
    //|  CONSEQ_2;      // might want to do repeat single channel
    // instead of starting before TimerA interrupt

    // Timer Config
    TACTL   = TASSEL_2 | MC_1 | TAIE;      // TA uses SMCLK, in Up mode
    TACCR0  = 64;                          // ~65us @  1 MHz

    // Wireless Initialization
    P2SEL = 0;                            // P2.6, P2.7 = GDO0, GDO2 (GPIO)
    TI_CC_SPISetup();                     // Init SPI port for cc2500
    TI_CC_PowerupResetCCxxxx();           // Reset cc2500
    writeRFSettings();                    // Put settings to cc2500 config regs
	//UCB0CTL1 |= UCSWRST;                  // Disab USCI state mach
    //UCB0BR0 = 0x01;                       // UCLK/1 = SMCLK/1
    //UCB0CTL1 &= ~UCSWRST;                 // Enab USCI state mach

    TI_CC_SPIWriteReg(TI_CCxxx0_CHANNR,  CHANNEL);  // Set Your Own Channel Number
                                                    // only AFTER writeRFSettings

    for(delay=0; delay<650; delay++){};   // Empirical: Let cc2500 finish setup
}

void main ()
{
    WDTCTL  = WDTPW + WDTHOLD;  // Stop Watchdog (Yawn)
    Setup();                    // initialize ADC and cc2250
    while(1) {
        ADC10CTL0 |= ENC | ADC10SC;           // Enable Conversion
        __bis_SR_register(LPM0_bits + GIE);   // Sleep until interupt from Timer
    }

}