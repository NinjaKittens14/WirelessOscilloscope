//=======================================================================BOF
// This file contains several functions that simplify interfacing the
// MSP430 software to the cc2500 transceiver chip. These funcs handle  
// the details of cc2500 initialization and SPI communication. 
//
// This file is edited down from the original file to be case-specific 
// to the eZ430-RF2500 target card.
// 
// Original written:       K. Quiring                  TI, Inc  2006/07
// Brilliantly adapted:    B. Ghena & A. Maine         MTU/ECE  2012/04
// Revised & edited:       R. Kieckhafer               MTU/ECE  2013/04
// Revised & re-adapted:   P. Ramesh & R. Kieckhafer   MTU/ECE  2014/04  
// Re-revised & edited:    R. Kieckhafer               MTU/ECE  2015/04
//--------------------------------------------------------------------------

#include "include_ez430.h"  // Lots of #defines for cc2500 interfacing

char paTable[] = {0xFE};    // Init CC2500 TX power level = 0dbm
char paTableLen = 1;        // Only write to PATABLE[0] in MSK mode

//--------------------------------------------------------------------------
// void TI_CC_Wait(unsigned int cycles)
//
// DESCRIPTION:
// Delay function, stalls for specified number of clock cycles.
// Actual num of clock cycles delayed is approx "cycles" argument. Specif,
// it's ((cycles-15) % 6) + 15. Not exact, but gives a sense of real-time
// delay. Also, if MCLK ~1MHz, "cycles" is approx num of usec delayed.
//--  
void TI_CC_Wait(unsigned int cycles)
{
  while(cycles>15)           // 15 cycles consumed by overhead
  cycles = cycles - 6;       //  6 cycles consumed each iteration
}

//--------------------------------------------------------------------------
// void TI_CC_SPISetup(void)
//
// DESCRIPTION:
// Configures the assigned interface to function as a SPI port and
// initializes it.
//--  
void TI_CC_SPISetup(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_CSn_PxDIR |= TI_CC_CSn_PIN;                // Clear CSn
  UCB0CTL1 |= UCSWRST;                             // Disab USCI state mach
  UCB0CTL0 |= UCMST+UCCKPH+UCMSB+UCSYNC;           // 3-pin, 8-bit SPI mast
  UCB0CTL1 |= UCSSEL_2;                            // SCLK = SMCLK
  UCB0BR0 = 0x02;                                  // UCLK/2
  UCB0BR1 = 0;
  TI_CC_SPI_USCIB0_PxSEL |= TI_CC_SPI_USCIB0_SIMO  // SPI pins option sel
                         |  TI_CC_SPI_USCIB0_SOMI
                         |  TI_CC_SPI_USCIB0_UCLK;
 
  TI_CC_SPI_USCIB0_PxDIR |= TI_CC_SPI_USCIB0_SIMO  // SPI Tx pins out dir
                         |  TI_CC_SPI_USCIB0_UCLK;

  UCB0CTL1 &= ~UCSWRST;                            // Enab USCI state mach
}

//--------------------------------------------------------------------------
// void TI_CC_SPIWriteReg(char addr, char value)
//
// DESCRIPTION:
// Writes "value" to a single config. register at address "addr".
//--
void TI_CC_SPIWriteReg(char addr, char value)
{
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;           // Assert CSn
  while (!(IFG2 & UCB0TXIFG));                 // Wait for TXBUF ready
  UCB0TXBUF = addr;                            // Send reg address
  while (!(IFG2 & UCB0TXIFG));                 // Wait for TXBUF ready
  UCB0TXBUF = value;                           // Send data
  while (UCB0STAT & UCBUSY);                   // Wait for SPI done
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;            // Clear CSn
}

//--------------------------------------------------------------------------
// void TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
//
// DESCRIPTION:
// Writes values to multiple config. registers, the first register being
// at address "addr". First data byte is at "buffer", & both addr and
// buffer are incremented sequentially (within the CCxxxx & MSP430,
// respectively) until "count" writes have been performed.
//--  
void TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
{
  unsigned int i;
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;           // Assert CSn
  while (!(IFG2 & UCB0TXIFG));                 // Wait for TXBUF ready
  UCB0TXBUF = addr | TI_CCxxx0_WRITE_BURST;    // Send reg address
  for (i = 0; i < count; i++)
  {
    while (!(IFG2 & UCB0TXIFG));               // Wait for TXBUF ready
    UCB0TXBUF = buffer[i];                     // Send data
  }
  while (UCB0STAT & UCBUSY);                   // Wait for SPI done
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;            // Clear CSn
}

//--------------------------------------------------------------------------
// char TI_CC_SPIReadReg(char addr)
//
// DESCRIPTION:
// Reads a single config. register at address "addr" & returns the
// value read.
//--  
char TI_CC_SPIReadReg(char addr)
{
  char x;
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;           // Assert CSn
  while (!(IFG2 & UCB0TXIFG));                 // Wait for TXBUF ready
  UCB0TXBUF = (addr | TI_CCxxx0_READ_SINGLE);  // Send reg address
  while (!(IFG2 & UCB0TXIFG));                 // Wait for TXBUF ready
  UCB0TXBUF = 0;                               // Dummy Wr so we can read
  while (UCB0STAT & UCBUSY);                   // Wait for SPI done
  x = UCB0RXBUF;                               // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;            // Clear CSn
  return x;
}

//--------------------------------------------------------------------------
// void TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
//
// DESCRIPTION:
// Reads multiple config. registers, the first register being at address
// "addr". Values read are deposited sequentially starting at address
// "buffer", until "count" registers have been read.
//--  
void TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
{
  char i;
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;         // Assert CSn
  while (!(IFG2 & UCB0TXIFG));               // Wait for TXBUF ready
  UCB0TXBUF = (addr | TI_CCxxx0_READ_BURST); // Send reg address
  while (UCB0STAT & UCBUSY);                 // Wait for SPI done
  UCB0TXBUF = 0;                             // Dummy Wr to read 1st byte

  // Addr byte is now being TX'ed, with dummy byte to follow immed. after

  IFG2 &= ~UCB0RXIFG;                     // Clear IRQ flag
  while (!(IFG2 & UCB0RXIFG));            // Wait for end of 1st data byte 
                                          // First data byte now in RXBUF
  for (i = 0; i < (count-1); i++)
  {
    UCB0TXBUF = 0;                        //Initiate next data RX, meanwhile.
    buffer[i] = UCB0RXBUF;                // Store data from last data RX
    while (!(IFG2 & UCB0RXIFG));          // Wait for SPI done
  }
  buffer[count-1] = UCB0RXBUF;            // Store last RX byte in buffer
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // Clear CSn
}

//--------------------------------------------------------------------------
// char TI_CC_SPIReadStatus(char addr)
//
// DESCRIPTION:
// Special function for reading status registers. Reads status register
// at register "addr" & returns the value read.
//--  
char TI_CC_SPIReadStatus(char addr)
{
  char status;
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;          // Assert CSn
  while (!(IFG2 & UCB0TXIFG));                // Wait for TXBUF ready
  UCB0TXBUF = (addr | TI_CCxxx0_READ_BURST);  // Send reg address
  while (!(IFG2 & UCB0TXIFG));                // Wait for TXBUF ready
  UCB0TXBUF = 0;                              // Dummy Wr so can read data
  while (UCB0STAT & UCBUSY);                  // Wait for SPI done
  status = UCB0RXBUF;                         // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;           // Clear CSn
  return status;
}

//--------------------------------------------------------------------------
// void TI_CC_SPIStrobe(char strobe)
//
// DESCRIPTION:
// Special function for writing to command strobe registers. Writes
// to the strobe at address "addr".
//--  
void TI_CC_SPIStrobe(char strobe)
{
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;     // Assert CSn
  while (!(IFG2 & UCB0TXIFG));           // Wait for TXBUF ready
  UCB0TXBUF = strobe;                    // Send strobe
                                         // Strobe addr is now being TX'ed
  while (UCB0STAT & UCBUSY);             // Wait for SPI done
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;      // Clear CSn
}

//--------------------------------------------------------------------------
// void TI_CC_PowerupResetCCxxxx(void)
//
// DESCRIPTION:
// Function for reseting the CCxxxx on powerup.
//--  
void TI_CC_PowerupResetCCxxxx(void)
{
  TI_CC_CSn_PxOUT |=  TI_CC_CSn_PIN;     // Clear CSn
  TI_CC_Wait(30);                        // Stall
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;     // Assert CSn
  TI_CC_Wait(30);                        // Stall
  TI_CC_CSn_PxOUT |=  TI_CC_CSn_PIN;     // Clear CSn
  TI_CC_Wait(45);                        // Stall
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;     // Assert CSn
  while (!(IFG2 & UCB0TXIFG));           // Wait for TXBUF ready
  UCB0TXBUF = TI_CCxxx0_SRES;            // Send reset strobe
                                         // Strobe addr is now being TX'ed
  while (UCB0STAT & UCBUSY);             // Wait for SPI done
  TI_CC_CSn_PxOUT |=  TI_CC_CSn_PIN;     // Clear CSn
}

//--------------------------------------------------------------------------
// void writeRFSettings(void)
//
// DESCRIPTION:
// Used to configure the CCxxxx registers.
//
// ARGUMENTS:
// none
//
// Product = CC2500
// Crystal accuracy = 40 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 542 kHz     RMK-2014-04 - iaw SmartRF Studio
// Deviation = 0.000000
// Return state: Return to RX state upon leaving either TX or RX
// Datarate = 250 kbps
// Modulation = (7) MSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 2423.5 MHz       RMK-2014-04 Start of WiFi chnl 1-6 Gap
// Channel spacing = 250 kHz       RMK-2014-04 Enlarged to lower interf.
// Channel number = 04             RMK-2015-04 Center of WiFi chnl Gap
// Autocal = (01) = Cal at TX or RX mode entry
// Optimization = Sensitivity
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX & TX
// CRC operation = (1) CRC calculation in TX & CRC check in RX enabled
// Forward Error Correction = (0) FEC disabled
// Length config. = (1):
// => Variable length packets, 
// => packet length configured by first received byte after sync word.
// Packetlength = 255
// Preamble count = (2) 4 bytes
// Append status = 1
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = (6):
// => Asserts when sync word has been sent / received, 
// => & de-asserts at the end of the packet
// GDO2 signal selection = (11) Serial Clock
//--  
void writeRFSettings(void)
{
  // Write register settings
  TI_CC_SPIWriteReg(TI_CCxxx0_IOCFG2,   0x0B);  // GDO2 out pin config
  TI_CC_SPIWriteReg(TI_CCxxx0_IOCFG0,   0x06);  // GDO0 out pin config
  TI_CC_SPIWriteReg(TI_CCxxx0_PKTLEN,   0xFF);  // Packet length
  TI_CC_SPIWriteReg(TI_CCxxx0_PKTCTRL1, 0x04);  // Packet automation ctrl
  TI_CC_SPIWriteReg(TI_CCxxx0_PKTCTRL0, 0x05);  // Packet automation ctrl
  TI_CC_SPIWriteReg(TI_CCxxx0_ADDR,     0xFF);  // Device address.
  TI_CC_SPIWriteReg(TI_CCxxx0_CHANNR,   0x04);  // Channel number     - RMK
  TI_CC_SPIWriteReg(TI_CCxxx0_FSCTRL1,  0x0A);  // Freq synth. ctrl   - RMK
  TI_CC_SPIWriteReg(TI_CCxxx0_FSCTRL0,  0x00);  // Freq synth. ctrl
  TI_CC_SPIWriteReg(TI_CCxxx0_FREQ2,    0x5D);  // Freq ctrl word hi 
  TI_CC_SPIWriteReg(TI_CCxxx0_FREQ1,    0x36);  // Freq ctrl word mid - RMK
  TI_CC_SPIWriteReg(TI_CCxxx0_FREQ0,    0x27);  // Freq ctrl word low - RMK
  TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG4,  0x2D);  // Modem config.
  TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG3,  0x3B);  // Modem config.
  TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG2,  0x73);  // Modem config.      - RMK
  TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG1,  0x23);  // Modem config.      - RMK
  TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG0,  0x3B);  // Modem config.      - RMK
  TI_CC_SPIWriteReg(TI_CCxxx0_DEVIATN,  0x00);  // Modem dev (FSK only)
  TI_CC_SPIWriteReg(TI_CCxxx0_MCSM1 ,   0x3F);  // MainRad Ctrl stat mach
  TI_CC_SPIWriteReg(TI_CCxxx0_MCSM0 ,   0x18);  // MainRad Ctrl stat mach
  TI_CC_SPIWriteReg(TI_CCxxx0_FOCCFG,   0x1D);  // Freq Offset Comp Config
  TI_CC_SPIWriteReg(TI_CCxxx0_BSCFG,    0x1C);  // Bit synch. config.
  TI_CC_SPIWriteReg(TI_CCxxx0_AGCCTRL2, 0xC7);  // AGC ctrl.
  TI_CC_SPIWriteReg(TI_CCxxx0_AGCCTRL1, 0x00);  // AGC ctrl.
  TI_CC_SPIWriteReg(TI_CCxxx0_AGCCTRL0, 0xB0);  // AGC ctrl.
  TI_CC_SPIWriteReg(TI_CCxxx0_FREND1,   0xB6);  // Front end RX config
  TI_CC_SPIWriteReg(TI_CCxxx0_FREND0,   0x10);  // Front end RX config
  TI_CC_SPIWriteReg(TI_CCxxx0_FSCAL3,   0xEA);  // freq. synthesizer cal
  TI_CC_SPIWriteReg(TI_CCxxx0_FSCAL2,   0x0A);  // freq. synthesizer cal
  TI_CC_SPIWriteReg(TI_CCxxx0_FSCAL1,   0x00);  // freq. synthesizer cal
  TI_CC_SPIWriteReg(TI_CCxxx0_FSCAL0,   0x11);  // freq. synthesizer cal
  TI_CC_SPIWriteReg(TI_CCxxx0_FSTEST,   0x59);  // freq. synthesizer cal
  TI_CC_SPIWriteReg(TI_CCxxx0_TEST2,    0x88);  // Various test settings
  TI_CC_SPIWriteReg(TI_CCxxx0_TEST1,    0x31);  // Various test settings
  TI_CC_SPIWriteReg(TI_CCxxx0_TEST0,    0x0B);  // Various test settings
  TI_CC_SPIWriteBurstReg(TI_CCxxx0_PATABLE, 
                         paTable, paTableLen);  //Init TX power in PATABLE
}

//-------------------------------------------------------------------------
// void RFSendPacket(char *txBuffer, char size)
//
// DESCRIPTION:
// This function transmits a packet with length up to 63 bytes
// 
// To use this func, GD00 must be configured to be asserted when sync word 
// is sent & de-asserted at the end of the packet, which is accomplished 
// by setting IOCFG0 register to 0x06, per the CCxxxx datasheet. 
//
// GDO0 goes high at packet start & returns low when complete. This func 
// polls GDO0 to ensure packet completion before returning.
//
// ARGUMENTS:
// char *txBuffer = Pointer to a buffer containing the data to be Tx'ed
// char size      = length of the txBuffer
//--  
void RFSendPacket(char *txBuffer, char size)
{
  TI_CC_SPIWriteBurstReg(TI_CCxxx0_TXFIFO, txBuffer, size); // Wr Tx data
  TI_CC_SPIStrobe(TI_CCxxx0_STX);          // Change state to TX, 
                                           // => initiating data Tx
  int fail_counter = 0;                    // Var to detect failure to Tx

  // Failure Mode: Sometimes the CC2500 appears to miss the strobe for 
  // TX & it needs to be resent. Experimentally determined that it takes 
  // exactly 24 iterations of this "while" loop to send a normal message. 
  // Any more iterations indicates a failure.

  while(!(TI_CC_GDO0_PxIN & TI_CC_GDO0_PIN)) // Wait for GDO0 to go hi 
  {                                          // => sync has been TX'ed
    fail_counter ++;  
    if (fail_counter > 30)                   // if in loop too long
    {
      TI_CC_SPIStrobe(TI_CCxxx0_STX);        // Resend the TX strobe
      fail_counter = 0;                      // Restart the loop
    }
  }
  while(TI_CC_GDO0_PxIN & TI_CC_GDO0_PIN);   // Wait for GDO0 to go low
                                             // => sync has been TX'ed
  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;       // Flag is set after pkt TX
                                             // Must be clr'ed before exit
}
//-------------------------------------------------------------------------
// char RFReceivePacket(char *rxBuffer, char *length, char *status)
//
// DESCRIPTION:
// Receives a packet of variable length (first byte in packet must be the
// length byte). The packet length should not exceed the RXFIFO size. 

// To use this func, APPEND_STATUS in PKTCTRL1 register must be enabled. 
// Assumes that the func is called after it is known that a packet has
// been received; for example, in response to GDO0 going low when it is
// configured to output packet reception status.
//
// RXBYTES register is first read to ensure there are bytes in the FIFO.
// This is done because GDO signal will go high even if FIFO is flushed
// due to address filtering, CRC filtering, or packet length filtering.
//
// ARGUMENTS:
// char *rxBuffer = Ptr to buffer where incoming data should be stored
// char *length   = Ptr to a variable containing the size of the buffer 
//                  where incoming data should be stored. After this func 
//                  returns, that variable holds the packet length.
// char *status   = Ptr to store the returned status of the recieve. 
//                  Contains RSSI & LQI.
//
// RETURN VALUE:
// char = 0x80: CRC OK
//      = 0x00: CRC NOT OK (or no pkt was put in RXFIFO due to filtering)
//--  
char RFReceivePacket(char *rxBuffer, char *length, char *status)
{
  char pktLen;
  if ((TI_CC_SPIReadStatus(TI_CCxxx0_RXBYTES) & TI_CCxxx0_NUM_RXBYTES))
  {
    pktLen = TI_CC_SPIReadReg(TI_CCxxx0_RXFIFO);  // Read length byte
    if (pktLen <= *length)                        // If pktLen <= rxBuffer
    {                                             // Then save packet
      TI_CC_SPIReadBurstReg(TI_CCxxx0_RXFIFO, rxBuffer, pktLen); // Get data
      *length = pktLen;                                          // Get Len
      TI_CC_SPIReadBurstReg(TI_CCxxx0_RXFIFO, status, 2);        // Get stat
       return (char)(status[TI_CCxxx0_LQI_RX]&TI_CCxxx0_CRC_OK); // Ret OK
    } 
    else
    {
      *length = pktLen;                         // Return the large size
      TI_CC_SPIStrobe(TI_CCxxx0_SFRX);          // Flush RXFIFO
      return 0;                                 // Retn Error flag
    }
  }
  else
    return 0;                                   // Retn Error flag
}

//======================================================================EOF
