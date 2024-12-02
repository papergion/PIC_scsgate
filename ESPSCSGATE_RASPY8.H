/******************************************************************************
 * Header file for SCSGATE                                                    *
 ******************************************************************************/
#ifndef _ESPSCSGATE_H
#define _ESPSCSGATE_H
#define HW 20201220		// per RASPBERRY optical

#warning VERSIONE per "raspberry optical"

/*----------------------------------------------------------------------------
;                                      18F26K80
;                                +--------+  +--------+
;                 mclr   --------|1  re3/mclr   rb7 28|---<----   pgd   rx2 usart usb
;                    jn  --------|2  ra0        rb6 27|--->----   pgc   tx2 usart usb
;                    nc  --------|3  ra1        rb5 26|--------   LED
;                    nc  --------|4  ra2        rb4 25|---->---   nc   
;                    nc  --------|5  ra3        rb3 24|--------   nc
;                  Vcap  --------|6             rb2 23|--------   jn
;                    nc  --------|7  ra5        rb1 22|----<---   C1inB
;                      - --------|8  Vss        rb0 21|--------   nc
;                    nc  --------|9  ra7        Vdd 20|-------- +
;                    nc  --------|10 ra6        Vss 19|-------- -
;                    jn  --------|11 rc0        rc7 18|---<----   uart1 RX   from esp8266 tx
;                    nc  --------|12 rc1        rc6 17|--->----   uart1 TX     to esp8266 rx
;                    jn  --------|13 rc2        rc5 16|--------   jn
;                    nc  --------|14 rc3        rc4 15|--------   OUT
;                                +--------------------+
; -------------------------------------------------------------------------*/

#if (HW == 20201220)
  #define INIT_TRISA  (0b00000001) //
  #define INIT_TRISB  (0b10010110) // 
  #define INIT_TRISC  (0b10100101) //
  #define LED_SYS      LATBbits.LATB5
  #define BAUD_RATE         7      // 115200 baud
  #define INIT_PORTC  (0b01000000) //
  #define SCSTX3       LATCbits.LATC4
  #define USE_UART1    // per colloquio con ESP
//  #define USE_UART2    // per colloquio con USB
//===============================================================================
#endif

  #define INIT_PORTA  (0b00000000) //
  #define INIT_PORTB  (0b01000000) //

#warning HW

// Initialization values for analog  registers
  #define INIT_ADCON0 (0b00000000) //
  #define INIT_ADCON1 (0b00000000) //
  #define INIT_ADCON2 (0b10111010) //

  #define INIT_ANCON0 (0b00000000)
  #define INIT_ANCON1 (0b00000000)
//===============================================================================
//----- mappatura   porte fisiche ------------------------------------
//===============================================================================
  #define INPUT_SCS     CMSTATbits.CMP1OUT
//===============================================================================
  #define FREQUENZA               64000000ul
  #define GetInstructionClock()  (GetSystemClock()/4)
  #define GetPeripheralClock()    GetInstructionClock()
//===============================================================================
#endif // _MAIN_H
