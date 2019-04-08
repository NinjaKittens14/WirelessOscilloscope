#include "include_ez430.h"
#include "msp430x22x4.h"
#include "stdint.h"
#include "wireless-v2.h"

/**
 * Tx.c
 * Receive Analog signal and input into ADC to convert to Digital data.
 * Send Digital data to wireless card and transmit to Receiver card.
 * 4/8/19
 */

#pragma vector=ADC10_VECTOR
__interupt void IsrAdc10LedSwitch(void)
//------------------------------------------------------------------------------
// Func:  Read value of ADC and send out to cc2250
// Args:  None
// Retn:  None
//------------------------------------------------------------------------------
{
    newNadc = ADC10MEM;            // read value of ADC


    __bic_SR_register_on_exit(LPM0_bits);  // Clr previous Low Pwr bits on stack
}



void main ()
/**
 * Func: Initialize ADC10 and cc2250 wrireless card
 * Args: None
 * Retn: None
 */
{
    WDTCTL  = WDTPW + WDTHOLD;  // Stop Watchdog (Yawn);

    DCOCTL  = CALDCO_8MHZ;      // DCO = 8 MHz;
    BCSCTL1 = CALBC1_8MHZ;      // DCO = 8 MHz;
    P3DIR  |=  0x02;            // P3.1 = output (to external LED);
    P3OUT  &= ~0x02;            // Init P3.1 = 0 (LED off)

    ADC10AE0  |= 0x01;          // Enable chnl A0 = P2.0;
    ADC10CTL0 |= ADC10ON        // Turn ON ADC10
              |  ADC10IE        // enable ADC IRQ
              |  ADC10SHT_2;    // samp-hold tim = 16 cyc;
    ADC10CTL1 |= ADC10SSEL_3    // ADC10CLK source = SMCLK
              |  ADC10DIV_3     // ADC10CLK divider = 4
              |  INCH_0;        // Select input = chnl A0 (default);

    while(1) {
        ADC10CTL0 |= ENC | ADC10SC;           // Enable Conversion
        __bis_SR_register(LPM0_bits + GIE);   // Sleep until interupt from ADC
    }

}