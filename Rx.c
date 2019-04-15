// ========================================================================= BOF
// FILE FUNC:  receive ADC Data over the cc2500, then send it via SPI to the
//             DAC.
// -----------------------------------------------------------------------------

#include "stdint.h"                // MSP430 data type definitions
#include "msp430x22x4.h"           // Mammoth MSP430-specific macros & defs
#include "wireless-v2.h"           // Mammoth cc2500 setup & function defs

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

  static uint8_t len = 3;          // Packet Len = 3 bytes (data only)
  uint8_t status[2];               // Buffer to store pkt status bytes
  uint8_t crcOk = 0;               // Flag that pkt was received w/ good CRC     
  uint8_t rxPkt[3];
  
  if(TI_CC_GDO0_PxIFG & TI_CC_GDO0_PIN)            // chk GDO0 bit of P2 IFG Reg
  { 
    crcOk = RFReceivePacket(rxPkt, &len, status);  // Get packet from cc2500
  }
  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;             // Reset GDO0 IRQ flag
 
  TI_CC_SPIStrobe(TI_CCxxx0_SIDLE);      // Set cc2500 to idle mode.
  TI_CC_SPIStrobe(TI_CCxxx0_SRX);        // Set cc2500 to RX mode.
                                         // AutoCal @ IDLE to RX Transition

  // <Do stuff with the data received>

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
  BCSCTL2 |= DIVS_3;                    // SMCLK = MCLK/8 = 1MHz

  // Config. GPIO Ports for SPI (NEED TO MAKE SURE THE CORRECT PORTS ARE USED)
  P3SEL  =  0x0A;               // P3.1 & P3.3 = alt modes = SPI SIMO & SCLK 
  P3DIR  =  0x0A;               // P3.1 & P3.3 = output dir

  P3SEL &= ~0x01;               // P3.0  = GPIO for SPI SS
  P3DIR |=  0x01;               // P3.0  = output dir
  P3OUT |=  0x01;               // init. = clear SPI SS

  // Config. USCA0 for SPI Master
  BCSCTL3  |= LFXT1S_2;                 // ACLK Source = VLO (12 KHz)
  UCA0CTL0 |=  UCSYNC | UCMST | UCMSB;  // 3-pin SPI master, Send msb 1st
  UCA0CTL1 |=  UCSSEL_2;                // SCLK Source Select
  UCA0BR1   =  0x00;                    // SCLK Divider MSB
  UCA0BR0   =  0x01;                    // SCLK Divider LSB
  
  // Wireless Initialization
  P2SEL = 0;                            // P2.6, P2.7 = GDO0, GDO2 (GPIO)
  TI_CC_SPISetup();                     // Initialize SPI port
  TI_CC_PowerupResetCCxxxx();           // Reset cc2500
  writeRFSettings();                    // Send RF settings to config regs

  TI_CC_GDO0_PxIES |=  TI_CC_GDO0_PIN;  // IRQ on GDO0 fall. edge (end of pkt)
  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;  // Clear  GDO0 IRQ flag
  TI_CC_GDO0_PxIE  |=  TI_CC_GDO0_PIN;  // Enable GDO0 IRQ 
  TI_CC_SPIStrobe(TI_CCxxx0_SRX);       // Init. cc2500 in RX mode.
  
  TI_CC_SPIWriteReg(TI_CCxxx0_CHANNR, 0);    // Set Your Own Channel Number
                                             // AFTER writeRFSettings (???)

  for(delay=0; delay<650; delay++);     // Empirical: Let cc2500 finish setup
}

// -----------------------------------------------------------------------------
void main()
{ WDTCTL = WDTPW + WDTHOLD;         // halt watchdog

  volatile uint8_t rxPkt[3];        // Buffer to store received pkt payload
  SetupAll();                       // Setup clocks, ports & cc2500 settings

  __bis_SR_register(CPUOFF + GIE);  // sleep til Pkt RX
}

// ========================================================================= EOF