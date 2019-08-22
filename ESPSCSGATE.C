#define TITLE    "ScsGAte"

#define VERSION  "SCS 19.303"		// V3
#define EEPROM_VER	0x8D	// per differenziare scs e knx

// #define ECHO

//   A8 <dest> <source> <type> <cmd> XX A3
//          dest B0-.. pulsanti
// type=12 (comandi) o 15 (query)
//          cmd=00 switch/dim ON
//          cmd=01 switch/dim OFF
//          cmd=03 dimmer  + MORE (?)
//          cmd=04 dimmer  - LESS (?)
//          cmd=08 tapparella ALZA
//          cmd=09 tapparella ABBASSA
//          cmd=0A tapparella STOP
//
//          cmd=80-FE intensita dimmer raggiunta

// 19.30 - raccoglie tabella info dispositivi - la manda a uart su richiesta @Dn (n bin o hex) indirizzo device - risponde con Dnt (n=indirizzo valido   t=tipo (0xff fine tab)
//                                                                                se n = FF ripulisce la tabella
// 18.93 - correzione debug uart2 - comando @0x15 per evitare memo eeprom
// 18.92 - sistemata la prima inizializzazione della eeprom
// 18.91 - rivista la gestione dei filtri - loop su 0xFF successivi
// 18.90 - nel comando @W accetta carattere K ad indicare checksum calcolato, @0xFF per evitare lampeggio led
// 18.90 - setup per comandi brevi in scrittura @Y2 in lettura @Y1  in entrambe @Y3
//         modificato @W, @r, @R  - nuovo cmd write breve senza setup @y.... (l fissa 4, hex)
// 18.69 - timeout di stream gestito a parametro configurabile 
// 18.69 - fix problema di pausa troppo lunga a fine stream ed errata successiva lettura - implica SCSINTRPT 4.9
// 18.68 - nuovo comando @V uguale a @W ma senza controllo di collisione - implica SCSINTRPT 4.8
// 18.67 - nel comando @W accetta carattere K ad indicare checksum calcolato, @0xFF per evitare lampeggio led
// 18.66 - correzione problema buffer overflow
// 18.64 - nuovi comandi lampeggio led breve/lungo (da esp)
// 18.61 - echo da uart1 a uart2 comando @e - programmazione da uart1-uart2
// 18.60 - utilizza uart1 e uart2
// 18.51 - adeguata a nuova versione di SCSINTRP - scrittura di telegrammi > 7 bytes
// 18.50 - adeguata a nuova versione di SCSINTRP - scrittura anti collisione
// 18.4C - fix options,7 che non veniva settato in SCSINTRP
// 18.4A - query device
// 18.48 - byte filtering A e B
// 18.47 - byte filtering & nuova versione knxintrp che gestisce overflow

// 18.44 - riconosce cmd echo 0x9A BC DE F0 12 34 56 78
// 18.43 - comando eeprom reset @E e comando @p  prova rx
// 18.42 - ora il comando @Dxx blocca il netname, per sbloccarlo usare @D00
// 18.41 - nuovi comandi @s  @S - low speed   high speed
// 18.40 - adattamento per hw 141113 - input da RC invece che da zener
//         nuovi comandi @3  @5
// ===================================================================================
// gate intelligente  SCS-UART
// bus out con TRX                bus in con differenziale pic
// ===================================================================================
// autore: Guido Pagani - 2013 -  guidopic@altervista.org
//                                http://guidopic.altervista.org/alter/index.html
// ===================================================================================
// questo software e' rilasciato senza nessun tipo di garanzia - non sono responsabile
// dell'uso che ne farete ne' di eventuali danni che potreste causare con l'utilizzo
// che ricade quindi sotto la vostra intera responsabilita'
// ===================================================================================
// questo software e' liberamente utilizzabile per uso amatoriale e hobbystico
// con il solo vincolo di mantenere in evidenza i dati dell'autore
// ===================================================================================
// ne e' vietato l'uso a scopo commerciale e industriale senza il consenso dell'autore
// ===================================================================================
// ne e' vietata la riproduzione anche parziale senza il consenso dell'autore
// ===================================================================================
#define _ESPSCSGATE_C
// #define _OSC_XTAL     // xtal da 16Mhz

#define CMDTYPE_SET		0x12	// 18.4A
#define CMDTYPE_QUERY	0x15	// 18.4A


#include "p18cxxx.h"
#include "GenericTypeDefs.h"
#include "ESPSCSGATE.h"
#include <stdlib.h>
#include <string.h>
#include "Compiler.h"
#include "Delay.h"
#include "eep.h"
#if defined(USE_UART2)
#warning ATTENZIONE - USA UART2 per DEBUG ---------------<<<<<<<<<<<<<<<<<<<<<
#endif
// ===================================================================================
#if defined(__18F26K80)
#if defined (_OSC_XTAL)
    #pragma config FOSC=HS1
#else
    #pragma config FOSC=INTIO2 // internal oscillator - IO on ra6/7
#endif
    #pragma config RETEN=OFF    // low power reg enabled
    #pragma config INTOSCSEL=LOW //LF-INTOSC in Low-power mode during Sleep
    #pragma config SOSCSEL=DIG
    #pragma config PLLCFG=ON   // PLL ON
    #pragma config IESO=OFF    // oscillator switch disabled
    #pragma config PWRTEN=ON
    #pragma config BOREN=OFF
    #pragma config FCMEN=OFF   // fail safe monitor disabled
    #pragma config WDTEN=SWDTDIS// WATCHDOG ENABLED
    #pragma config WDTPS=1024   // WATCHDOG PRESCALER  4ms x 1024
    #pragma config CANMX=PORTB  // can su portB2-B3
    #pragma config MSSPMSK=MSK7 // can mask a 7 bit
    #pragma config MCLRE=ON
    #pragma config XINST=OFF
#endif
// ===================================================================================
#define EE_CONFIG     0x00   // indirizzo eeprom configurazione
// ===================================================================================

// ========================STATO monitor==============================================
    BYTE baud = 4;
ROM char *baud_str[] = { "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"};
ROM char baud_value[2][8] = {
    { 52, 26, 13,   6,  3,   1,  1,  0 },
    { 20,  9,  4, 129, 64, 159, 20, 137}
};
// ===================================================================================

// ===================================================================================
static BYTE         Ticks;
static BYTE         ledLamps = 200;
static BYTE         SystemTicks;
static BYTE         EEaddress;
static BYTE         uartRc;
static BYTE         pMode;
static BYTE         uart_in_use = 2;
#if (defined(USE_UART1) && defined(USE_UART2))
//          UART2: icsp          UART1: esp8266
// echo=0 : l'output di knxgate viene inviato all'uart che ha fornito l'ultimo input in ordine di tempo
// echo=1 : l'output di knxgate viene inviato ad entrambe le uart indipendentemente dall'input
// echo=2 : l'output di knxgate viene inviato ad entrambe le uart indipendentemente dall'input
//          l'input di uart2    viene inviato solo in output a uart1 ma non a knxgate
// echo=3 : l'output di knxgate viene inviato ad entrambe le uart indipendentemente dall'input
//          l'input di uart1    viene inviato anche in output a uart2
//          l'input di uart2    viene ignorato

static BYTE         uart_echo = 3;    
#else
static BYTE         uart_echo = 0;
#endif
// ===================================================================================
#pragma udata TABDEV
BYTE devc[180];	// 0xFF:empty    01:switch    03:dimmer    08:tapparella
BYTE didx;
BYTE dlen;
// ===================================================================================
#pragma udata
// ===================================================================================

// ===================================================================================
    enum _SM_COMMAND
    {
        SM_WAIT_HEADER,         //   @
        SM_WAIT_COMMAND,        //
        SM_WAIT_MODO,           //  "@M[A|X] : modo ascii | hex"
        SM_WAIT_FILTRO,         //  "@F[0-7] : filtro"
        SM_WAIT_FILTRO_BYTE_A,    //  "@A[i/e][byte][valore]: filtro byte"	// 4.2  gestisce il byte filtering A
        SM_WAIT_FILTRO_BYTE_A_NR, //										   4.2  gestisce il byte filtering A
        SM_WAIT_FILTRO_BYTE_A_VALUE, //										   4.2  gestisce il byte filtering A
        SM_WAIT_FILTRO_BYTE_B,    //  "@B[i/e][byte][valore]: filtro byte"	// 4.4  gestisce il byte filtering B
        SM_WAIT_FILTRO_BYTE_B_NR, //										   4.4  gestisce il byte filtering B
        SM_WAIT_FILTRO_BYTE_B_VALUE, //										   4.4  gestisce il byte filtering B
        SM_WAIT_ABBREV_TYPE,    //  "@Y[0-3]: tipo di abbreviazione (0:no   1:read&log    2:write    3=all
        SM_WAIT_WRITE_LENGTH,   //  "@W[1-F][data] write"
        SM_WAIT_WRITE_DATA,     //  "@W[1-F][data] write"
        SM_WAIT_WRITE_DATA_BREVE,// "@y[data] write"
        SM_WAIT_WRITE_CMD,      //  "@w[value][destin] write"
        SM_WAIT_WRITE_DESTIN,   //  "@w[value][destin] write"
        SM_WAIT_WRITE_QUERY,    //  "@t[destin] write test query"
        SM_WAIT_WRITE_LOOP_I,   //  "@Z[value] write loop"
        SM_WAIT_WRITE_LOOP,     //  "@z[value] write loop"
        SM_WAIT_TABSTART,       //  "@D[start]"
        SM_READ_DEQUEUE,        //  "@r
        SM_WAIT_RESET,          //  "@|| 
//      SM_READ_WAIT,           //  "@R
    }   sm_command = SM_WAIT_HEADER;
// ===================================================================================
    enum _SM_STATO
    {
        SM_HOME,
        SM_READ_WAIT,
        SM_LOG_WAIT,
        SM_ECHO_ON,
        SM_TEST_MODE
    }   sm_stato   = SM_HOME;
// ===================================================================================
BYTE    writeLength;            //    [1-F] data length
BYTE    dataLength;             //    [1-F] data length
BYTE	testCounter;
// ===================================================================================
typedef union _SCS_OPTIONS {
  struct {
			BYTE		eeVersion;              //    versione eeprom
			BYTE		opzioneModo;            //    [A|X] : modo ascii | hex
			BYTE		opzioneFiltro;          //    [0-7] : filtro"
			BYTE		Vref;
			BYTE		EfilterByte_A;	//v 4.2  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
										//v 4.2  HB  0100=exclude ack        1000=exclude equal tlgrm
										//v 4.2  LB: byte nr su cui filtrare (1-F) - comparato con rByteCount (0=filteroff)
			BYTE		EfilterValue_A;	//v 4.2  valore byte di filtro   (0-FF)
			BYTE		EfilterByte_B;	//v 4.4  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
										//v 4.4  LB: byte nr su cui filtrare (1-F) - comparato con rByteCount (0=filteroff)
			BYTE		EfilterValue_B;	//v 4.4  valore byte di filtro   (0-FF)
			BYTE_VAL	abbrevia;		// setup comandi brevi 00=FF:no  0x01:lettura    0x02:scrittura    0x03:tutto
			BYTE		stream_timeout;	// stream timeout (complemento a 255 dei uS/4 ti timeout di stream
        };
  unsigned char data[16];
} SCS_OPTIONS;
// ===================================================================================
SCS_OPTIONS	opt;
BYTE ee_avoid_memo = 0;
BYTE ee_avoid_answer = 0;
// ===================================================================================
typedef union _SCS_MESSAGE {
  struct {
//      control byte    (1 byte) - 0xA8
        BYTE    TopByte;
//      destin.address
        BYTE	DestinationAddress;
//      source address
        BYTE	SourceAddress;
//      transport   protocol data unit (command type)
        BYTE    CommandType;
//      application   protocol data unit (command)
        BYTE    CommandValue;
//      check byte
        BYTE    CheckByte;
//      control byte    (1 byte) - 0xA3
        BYTE    BottomByte;
        };
  unsigned char data[16];
} SCS_MESSAGE;

SCS_MESSAGE	dataByte;           //          data values

BYTE    thisHalf;
BYTE    thisByte;
BYTE    dataTwin;
BYTE    writeValue;
BYTE    writeDest;
BYTE    check;
static BYTE         autocheck;			// 18.90
// ===================================================================================
extern	char rBufferIdxR;
extern	BYTE filterByte_A;	//v 4.2  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
							//v 4.2  HB  0100=exclude ack        1000=exclude equal tlgrm
					        //v 4.2  LB: byte nr su cui filtrare (1-F) - comparato con rByteCount (0=filteroff)
extern BYTE filterValue_A;	//v 4.2  valore byte di filtro   (0-FF)
extern BYTE filterByte_B;	//v 4.4  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
							//v 4.4  LB: byte nr su cui filtrare (1-F) - comparato con rByteCount (0=filteroff)
extern BYTE filterValue_B;	//v 4.4  valore byte di filtro   (0-FF)
extern BYTE stream_timeout; //v 4.7  timeout di stream (il complemento a 255 diviso 4 è il timeout in uS)
char rBufferIdxP;
// ===================================================================================
extern void scsInit(void);
extern void scsInterruptInput(void);
extern void scsInterruptTimer(void);
extern BYTE scsSend(BYTE len);
extern BYTE scsSendWait(BYTE len);
extern BYTE scsSendTest(BYTE chr);
#define SCSBUFMAX 10
#define SCSBUFLEN 16
extern volatile far BYTE scsMessageRx[SCSBUFMAX][SCSBUFLEN];
extern volatile far BYTE scsMessageTx[SCSBUFLEN];
//extern volatile far BYTE rBitCount;  // solo per test
//     volatile     BYTE rInterrupt;  // solo per test
//extern volatile far BYTE rByteCount; // solo per test
extern volatile far BYTE rBufferIdxW;
extern volatile far BYTE_VAL optionW;
 #define      W_COLLISION  bits.b1	//; 0=intercettare le collisioni      1=non intercettarle
 #define      W_ERROR      bits.b2	//; 0=scrittura ok					  1=errore collisione o timeout
 #define      W_INTRP      bits.b3
 #define      W_WAIT       bits.b4
 #define      W_EXIT       bits.b5
 #define      W_EOB        bits.b6
 #define      W_END        bits.b7

extern volatile far BYTE_VAL optionR;
 #define      R_EOB        bits.b0
//#define      T_START      bits.b1
 #define      T_NOW        bits.b1
 #define      R_READING    bits.b2
 #define      R_FILTER_BYTE_DISCARD        bits.b3	; v 4.2  gestisce il byte filtering

extern          far int  wPeriodoDomin; //;; FFFF - periodo dominante  [write timer]
extern          far int  wPeriodoRece;  //;; FFFF - periodo Recessivo [write timer]
// ===================================================================================
#pragma udata DATARS
char RS_Out_Buffer[160];
// ===================================================================================
BYTE btohexa_high(BYTE b);
BYTE btohexa_low(BYTE b);
char getUSBwait(void);
char getUSBnowait(void);
BYTE getUSBvalue(void);
void putcUSBwait(BYTE  data);
void putsUSBwait(char *data);
void putrsUSBwait(const ROM char *data);
void puthexUSBwait(BYTE  data);
void WriteByteEEP(BYTE Data, int Addr);
void Read_eep_array (BYTE *Data, int Address, char Len);
void Write_eep_array(BYTE *Data, int Address, char Len);
void Write_config(void);

static void InitializeBoard(void);
void TickInit(void);
void AppInitialize(void);
void IncrementBufferPtr(void);
void LogScsDisplay(void);
void AnswerMsg(void);
void InputMapping(void);
void TransferFunction(void);
void OutputMapping(void);
void queueWrite(char dataLength);
void DisplayScsTx(BYTE len);
void DisplayScsRxBuffers(void);
void UserProcess(void);

// ===================================================================================
ROM char menuScsPrompt[] =
{
    "\r\nSCSgate " VERSION
    "\r\n@M[A|X] : modo ascii | hex"
    "\r\n@F[0-7] : filtro"
    "\r\n@A[i/e][byte][valore]: filtro byte"	// 4.2  gestisce il byte filtering
    "\r\n@B[i/e][byte][valore]: filtro byte"	// 4.4  gestisce il byte filtering
    "\r\n@q : query version"
    "\r\n@b : buffer clear"
    "\r\n@r : read immed."
    "\r\n@R : read defer."
    "\r\n@c : cancel def."
    "\r\n@l : cont log."
    "\r\n@W[0-F][data] write"
    "\r\n@t[destin] write query"
    "\r\n@w[value][destin] write"
    "\r\n>"
};
// ===================================================================================
ROM char msgPrompt[] =
{
    "\r\n>"
};
ROM char crlf[] =
{
    "\r\n"
};
// ===================================================================================

#pragma code
/******************************************************************************
 * The following functions provide the specific hardware and application      *
 * configuration initialization                                               *
 ******************************************************************************/
static void InitializeBoard(void)
{
    int  j;

    for (j = 0; j < 3; j++)   DelayMs(100);   // startup delay 1 sec

    // Set direction and initial default valuefor GPIO Ports
    LATA  = INIT_PORTA;
    TRISA = INIT_TRISA;
    LATB  = INIT_PORTB;
    TRISB = INIT_TRISB;
    LATC  = INIT_PORTC;
    TRISC = INIT_TRISC;

    ANCON0 = INIT_ANCON0;
    ANCON1 = INIT_ANCON1;
    ADCON0 = INIT_ADCON0;      // Set up analog features of PORTA
    ADCON1 = INIT_ADCON1;
    ADCON2 = INIT_ADCON2;
    INTCON2bits.RBPU = 0;   // portB pullups enabled
    WPUB = 0x20;            // portB RB5 pullup

// set up oscillator values
#if defined(LED_SYS)
    LED_SYS= 1;
#endif
#if defined(LED_SYS1)
    LED_SYS1= 1;
#endif
#if defined(LED_INT)
    LED_INT= 1;
#endif
    for (j = 0; j < 1000; j++);

#if !defined (_OSC_XTAL)
    OSCTUNE = 0;
    OSCCON = 0b01110000;           // 16Mhz x 4 = 64Mhz - obbligatorio 16 or 64
    OSCTUNE = 0;
    OSCTUNEbits.PLLEN = 1;         // x4
    while (OSCCONbits.HFIOFS == 0);  // wait for frequency stable
#endif
    ODCON = 0;                     // open drain out disabled

#if defined(LED_SYS)
    LED_SYS= 0;
#endif
#if defined(LED_SYS1)
    LED_SYS1= 0;
#endif
#if defined(LED_INT)
    LED_INT= 0;
#endif
    ClrWdt();
    for (j = 0; j < 30000; j++);
#if defined(LED_SYS)
    LED_SYS= 1;
#endif
#if defined(LED_SYS1)
    LED_SYS1= 1;
#endif
#if defined(LED_INT)
    LED_INT= 1;
#endif

    // -----------------------------------------------------------------------------------
    // I/O  UART2  from/to USB - configuration is 115200, 8, N, 1.
    // -----------------------------------------------------------------------------------
#if defined(USE_UART2)
    RCSTA2bits.SPEN = 1;
    BAUDCON2 = 0b00001000; // 16 bit baud gen
    //-----------------------
	#if defined(BAUDLOW)
		if (BAUDLOW == 0)
		{
			SPBRGH2  = baud_value[0][BAUD_RATE - 2];
		    SPBRG2   = baud_value[1][BAUD_RATE - 2];
		}
		else
		{
			SPBRGH2  = baud_value[0][BAUD_RATE];
		    SPBRG2   = baud_value[1][BAUD_RATE];
		}
	#else
	//  SPBRGH2  = 0;          //
	//  SPBRG2   =  138;         // 115200 at 64Mhz
		SPBRGH2  = baud_value[0][BAUD_RATE];
	    SPBRG2   = baud_value[1][BAUD_RATE];

	#endif
    //-----------------------
    TXSTA2 = 0b00100110;   // tx enabled. hig speed
    if (uart_echo == 3)
		RCSTA2 = 0b10000000;   // rx disabled.
	else
		RCSTA2 = 0b10010000;   // rx enabled.
    PIR3bits.RC2IF = 0;
#else
    INTCON2bits.RBPU = 0;   // portB pullups enabled
    WPUB = 0xC0;            // portB RB6-7 pullup
#endif

	
	
	
	

    // -----------------------------------------------------------------------------------
    // I/O  UART1  from/to ESP - configuration is 115200, 8, N, 1.
    // -----------------------------------------------------------------------------------
#if defined(USE_UART1)
    RCSTA1bits.SPEN = 1;
    BAUDCON1 = 0b00001000; // 16 bit baud gen
    // -----------------------------------------------------------------------------------
	SPBRGH1  = baud_value[0][BAUD_RATE];
    SPBRG1   = baud_value[1][BAUD_RATE];
    // -----------------------------------------------------------------------------------
    TXSTA1 = 0b00100110;   // tx enabled
    RCSTA1 = 0b10010000;   // rx enabled
    PIR1bits.RC1IF = 0;
#endif
    // -----------------------------------------------------------------------------------

	
	
	
	
	
	
	
	
	
	
	
	
    // -----------------------------------------------------------------------------------
    RCONbits.IPEN = 1;       // interrupt priority enabled
//  INTCON2bits.INTEDG0 = 1; // INT0 RB0 rising edge
    // -----------------------------------------------------------------------------------
    // comparator: c1inB (-) [rb1] input
    //             c1inA (+) [rb0] reference
    // -----------------------------------------------------------------------------------
    CM1CON = 0;
//  CM1CONbits.EVPOL = 0b10; // interrupt on falling edge  (INPUT RISING)
    CM1CONbits.EVPOL = 0b01; // interrupt on rising  edge  (INPUT FALLING)
    CM1CONbits.CON   = 1;    // comparator ON
    CM1CONbits.CREF  = 0;    // non inverting input connected to C1INA (RB0)
    PIR4bits.CMP1IF  = 0;    // clear interrupt
    PIE4bits.CMP1IE  = 0;    // no    interrupt
    IPR4bits.CMP1IP  = 1;    // high-priority interrupt
#if (( HW == 141113 ) || ( HW == 160318))
    CM1CONbits.CREF  = 1;    // non inverting input connected to internal cVref
    CVRCON = 0b10000000;     // Vref gen enabled, internal only, Vdd-Vss
    CVRCONbits.CVR = opt.Vref;      // ref value from 0 to 31 - ogni step 5/31 = 0,15Volt
#endif
}

// ===================================================================================
BYTE SetInternalVref(void)
{
// calcola valore Vref interno sul comparatore - valore limite - 0,27Volt
BYTE VrValue;

    INTCONbits.GIEH     = 0; // high priority interrupt disabled
    INTCONbits.GIEL     = 0; // low  priority interrupt disabled


//  PIE4bits.CMP1IE  = 0;    // no    interrupt
//  CM1CONbits.EVPOL = 0b00; // disable interrupt
//  CM1CONbits.CREF  = 1;    // non inverting input connected to internal cVref <<<<<<<<<<<<<<<<<<<<<<<<<
//  CVRCON = 0b10000000;     // Vref gen enabled, internal only, Vdd-Vss
    VrValue = 0;
    CVRCONbits.CVR = VrValue;// ref value from 0 to 31   - ogni step 5/32 = 0,16Volt a 5V    0,10 a 3.3V
    Delay10us(20);
    while ((CMSTATbits.CMP1OUT == 0) && (VrValue < 31))  // alzo la tensione di riferimento finche' scatta
    {
		ClrWdt();
		VrValue++;
        CVRCONbits.CVR = VrValue;   // ref value from 0 to 31
        Delay10us(50);
    }
    if (VrValue) VrValue--;                      // abbasso la tensione di riferimento di 0,10 Volts // se alimentazione 3.3V
    if (VrValue) VrValue--;                      // abbasso la tensione di riferimento di 0,20 Volts // se alimentazione 3.3V
    if (VrValue) VrValue--;                      // abbasso la tensione di riferimento di 0,30 Volts // se alimentazione 3.3V
    if (VrValue) VrValue--;                      // abbasso la tensione di riferimento di 0,40 Volts // se alimentazione 3.3V : 1,5 x 13 = 3,5V

  #ifdef PULL_3
    if (VrValue) VrValue--;                      // abbasso la tensione di riferimento di 0,50 Volts // se alimentazione 3.3V
    if (VrValue) VrValue--;                      // abbasso la tensione di riferimento di 0,60 Volts // se alimentazione 3.3V
    if (VrValue) VrValue--;                      // abbasso la tensione di riferimento di 0,70 Volts // se alimentazione 3.3V
  #endif

	CVRCONbits.CVR = VrValue;   // ref value from 0 to 31

//  CM1CONbits.EVPOL = 0b01; // interrupt on rising  edge  (INPUT FALLING)
    PIR4bits.CMP1IF  = 0;    // clear interrupt
//  PIE4bits.CMP1IE  = 1;    //

    INTCONbits.GIEH     = 1; // high priority interrupt enabled
    INTCONbits.GIEL     = 1; // low  priority interrupt enabled
	return VrValue;
}
// ================================================================================

// ******************************************************************************/
// * initialize timer Tx                                                        */
// ******************************************************************************/
void TickInit(void)
{
// ------------------------ TIMING -------------------------------------------------
// -------------------------------------------------------------------------------
// TMR0  - setup timer per lettura SCS con interrupt C
// -------------------------------------------------------------------------------
// tmr0 period = 1: (64mhz/4 : 64) =  4uSec
// tmr0 timeout= 256*4uSec =    1024uSec
        T0CON = 0b01000101;        // prescale 64   8 bit timer
        T0CONbits.TMR0ON = 1;      // timer start
        INTCONbits.TMR0IF = 0;     // clear INTERRUPT FLAG
        INTCONbits.TMR0IE = 0;     // disable TMR0 interrupt
        INTCON2bits.TMR0IP  = 0; // TMR0 interrupt low priority
// -------------------------------------------------------------------------------
// TMR1  - led timing
// -------------------------------------------------------------------------------
// tmr1 period = 1: (64mhz/4 : 8 ) = 0.5uSec
// tmr1 timeout= 256*256*0.5uSec =   32mSec
        T1CON = 0b00110010;
        T1CONbits.TMR1ON = 1;    // timer start
        PIR1bits.TMR1IF = 0;     // clear INTERRUPT FLAG
        PIE1bits.TMR1IE = 0;     // disable TMR1 interrupt
// -------------------------------------------------------------------------------
// TMR3  - per SCS write
// -------------------------------------------------------------------------------
// tmr3 period = 1: (64mhz/4 : 8 ) = 0.5uSec
// tmr3 timeout= 256*256*0.5uSec =   32mSec
        T3CON = 0b00110010;
        T3CONbits.TMR3ON = 1;    // timer start
        PIR2bits.TMR3IF  = 0;    // clear INTERRUPT FLAG
        PIE2bits.TMR3IE  = 0;    // disable TMR3 interrupt
        IPR2bits.TMR3IP  = 0;    // TMR3 interrupt low priority
}
// -----------------------------------------------------------------------------------
void eepromInit(void)
{
        opt.opzioneModo   = 'X';        //    [A|X] : modo ascii | hex"
        opt.opzioneFiltro = 0;          //    [0|1|2|3|4] : filtro"
		opt.EfilterByte_A = 0;			// 4.2  gestisce il byte filtering A
		opt.EfilterValue_A = 0;			// 4.2  gestisce il byte filtering A
		opt.EfilterByte_B = 0;			// 4.4  gestisce il byte filtering B
		opt.EfilterValue_B = 0;			// 4.4  gestisce il byte filtering B
		opt.Vref = 20;
		opt.abbrevia.Val = '0';
		opt.stream_timeout = 191;
		opt.eeVersion = EEPROM_VER;
		Write_config();
		DelayMs(10);

		filterByte_A = 0;			// 4.2  gestisce il byte filtering A
		filterValue_A = 0;			// 4.2  gestisce il byte filtering A
		filterByte_B = 0;			// 4.4  gestisce il byte filtering B
		filterValue_B = 0;			// 4.4  gestisce il byte filtering B
//		Reset();
}
//--------------------------------------------------------------------
void InterruptHandlerHigh (void);
//--------------------------------------------------------------------
// High priority interrupt vector
#pragma code InterruptVectorHigh = 0x0008
void InterruptVectorHigh( void )
{
#if defined(LED_INT)
            LED_INT ^= 1;
#endif
#if defined(OUT_INT)
            OUT_INT = 1;
            OUT_INT = 0;
#endif

//  rInterrupt = 0x11;
_asm
  goto scsInterruptInput
_endasm
}
//--------------------------------------------------------------------
void InterruptHandlerLow (void);
//--------------------------------------------------------------------
// Low  priority interrupt vector
#pragma code InterruptVectorLow = 0x0018
void InterruptVectorLow( void )
{
//  rInterrupt++;
_asm
  goto scsInterruptTimer
_endasm
}
//--------------------------------------------------------------------
#pragma code /* return to default code section */
// --------------------------------------------------------------------------------




// -----------------------------------------------------------------------------------
void AppInitialize(void)
{
    SCSTX3 = 0;    // idle=low
//  T2CON = 0b01111010;      // postscale 16   prescaler 16   - timeout 4,09mS

// dominante = (0xFFFF - (SCS_TIME_WRITE / 3) + 1);     105/3  = -35/0,5us=  -70
// recessivo = (0xFFFF - (SCS_TIME_WRITE / 3) * 2 + 1); 105/3*2= -70/0,5uS= -140
    wPeriodoDomin =  -64;  //;; FFFF - periodo dominante  [write timer]
    wPeriodoRece = -146;   //;; FFFF - periodo Recessivo [write timer]
    wPeriodoRece += 4;     //;; 2uS in meno x gestione interrupt

    T0CONbits.TMR0ON = 1;    // timer start
    INTCONbits.TMR0IF = 0;   // clear INTERRUPT FLAG
    INTCONbits.TMR0IE = 0;   // disable TMR0 interrupt
    INTCON2bits.TMR0IP  = 0; // TMR0 interrupt low priority
    T3CONbits.TMR3ON = 1;    // timer start
    PIR2bits.TMR3IF  = 0;    // clear INTERRUPT FLAG
    PIE2bits.TMR3IE  = 0;    // disable TMR3 interrupt
    IPR2bits.TMR3IP  = 0;    // TMR3 interrupt low priority
    PIR4bits.CMP1IF  = 0;    // clear interrupt
    PIE4bits.CMP1IE     = 1; //
    INTCONbits.GIEH     = 1; // high priority interrupt enabled
    INTCONbits.GIEL     = 1; // low  priority interrupt enabled
    rBufferIdxW = 0;
    rBufferIdxR = 0;
}
// ******************************************************************************/
void IncrementBufferPtr(void)
{
    rBufferIdxR++;
    if (rBufferIdxR >= SCSBUFMAX) rBufferIdxR = 0;
}
// ******************************************************************************/
void LogScsDisplay(void)
{
BYTE s, m, n, l;
    if (optionW.W_ERROR)  // test bit 2
    {
        putrsUSBwait("\r\n > Write timeout");
        optionW.W_ERROR = 0; // azzera bit 2
    }
    strcpypgm2ram(RS_Out_Buffer, "\r\nSCS[ ]:  ");
    m = 10;
    l = scsMessageRx[rBufferIdxR][0];
    if (l > 15) l = 15;

	if (opt.abbrevia.bits.b0)	// abbreviazione, evitare PFX, CHK, SFX
	{
		s = 2;		// start
		l -= 3;		// length
	}
	else
		s = 1;

    RS_Out_Buffer[6] = btohexa_low(l);

	n = 0;
	while (n<l)
    {
        RS_Out_Buffer[m++] = btohexa_high(scsMessageRx[rBufferIdxR][s]);
        RS_Out_Buffer[m++] = btohexa_low (scsMessageRx[rBufferIdxR][s]);
        RS_Out_Buffer[m++] = ' ';
		n++;
		s++;
    }
    RS_Out_Buffer[m] = 0;
    putsUSBwait(RS_Out_Buffer);
}
// ===================================================================================


// ******************************************************************************/
// * answer telegram from scs to uart                                           */
// ******************************************************************************/
void AnswerMsg(void)
{
BYTE s,n,l;

	if (opt.opzioneModo == 'A')    // modo ascii
    {
        if (rBufferIdxR == rBufferIdxW)  // buffer vuoto
        {
            putcUSBwait('0');
        }
        else
        {
            l = scsMessageRx[rBufferIdxR][0];
            if (l > 15) l = 15;
			if (opt.abbrevia.bits.b0)	// abbreviazione, evitare PFX, CHK, SFX
			{
				s = 2;		// start
				l -= 3;	    // length
			}
			else
				s = 1;
            putcUSBwait(btohexa_low (l));

			n = 0;
			while (n < l)
//			for (n=s; n<=l; n++)
            {
                putcUSBwait(btohexa_high(scsMessageRx[rBufferIdxR][s]));
                putcUSBwait(btohexa_low (scsMessageRx[rBufferIdxR][s]));
				n++;
				s++;
            }
        }
    }
	else
    if (opt.opzioneModo == 'X')    // esadecimale puro
    {
        if (rBufferIdxR == rBufferIdxW)  // buffer vuoto
        {
            putcUSBwait(0x00);
        }
        else
        {
            l = scsMessageRx[rBufferIdxR][0];	// 7
            if (l > 15) l = 15;
			if (opt.abbrevia.bits.b0)	// abbreviazione, evitare PFX, CHK, SFX
			{
				s = 2;			// start ptr
				l -= 3;	        // length
			}
			else
				s = 1;
            putcUSBwait(l);  // length: 7 or 4

			n = 0;
			while (n < l)
            {
                putcUSBwait(scsMessageRx[rBufferIdxR][s++]);
				n++;
            }
        }
    }
}

// ******************************************************************************/
// * input analyze and mapping                                                  */
// ******************************************************************************/
void InputMapping(void)
{
    BYTE choice0;
    BYTE m, n, x, b;
	int  i;

    choice0 = getUSBnowait();
//  if (choice0 != 0)
    if (uartRc != 0)
    {
        switch (sm_command)
        {
            case SM_WAIT_HEADER:
                 if (choice0 == '@')  sm_command = SM_WAIT_COMMAND;
                 if (choice0 == 0x11) REPROmainB(0x10, 0x12); // firmware update
                 if (choice0 == 0x12) putcUSBwait('k');       // dummy firmware update
                 if ((opt.opzioneModo == 'A')
                 && (choice0 == 'h')) putrsUSBwait(menuScsPrompt);
                 break;

            case SM_WAIT_COMMAND:
                 switch(choice0)
                 {
                     case '@':
                          break;
                     case 0x11:      // firmware update
                          REPROmainB(0x10, 0x12);
                          break;
                     case 0x12:      // dummy firmware update
                          putcUSBwait('k');
                          break;
                     case 0x15:      // per evitare memo in eeprom
						  ee_avoid_memo = 1;
						  ee_avoid_answer = 1;
						  sm_command = SM_WAIT_HEADER;
						  if (ee_avoid_answer == 0)
								putcUSBwait('k');
                          break;
                     case 0x16:      // ripristina memo in eeprom
						  ee_avoid_memo = 0;
						  ee_avoid_answer = 0;
						  sm_command = SM_WAIT_HEADER;
                          putcUSBwait('k');
                          break;
                     case 0xF0:      // lampeggio lunghissimo
						  ledLamps = 200;
						  sm_command = SM_WAIT_HEADER;
                          break;
                     case 0xF1:      // lampeggio standard
						  ledLamps = 30;
						  sm_command = SM_WAIT_HEADER;
                          break;
                     case 0xF2:      // lampeggio frequente
						  ledLamps = 10;
						  sm_command = SM_WAIT_HEADER;
                          break;
                     case 0xFF:      // nessun lampeggio
						  ledLamps = 0;
						  sm_command = SM_WAIT_HEADER;
                          break;
                      case 'E':       // eeprom initialize
					      eepromInit();
						  if (ee_avoid_answer == 0)
								putcUSBwait('k');
						  sm_command = SM_WAIT_HEADER;
                          break;
#if (defined(USE_UART1) && defined(USE_UART2))
                     case 'e':       // echo uart
						  uart_echo++;
						  if (uart_echo > 3) uart_echo = 0;
						  if (ee_avoid_answer == 0)
								putcUSBwait('k');
						  sm_command = SM_WAIT_HEADER;
                          break;
#endif
                     case 'M':       // mode ascii-hex
                          sm_command = SM_WAIT_MODO;
                          break;
                     case 'F':      // msg filter
                          sm_command = SM_WAIT_FILTRO;
                          break;
                     case 'A':      // msg byte filter A
                          sm_command = SM_WAIT_FILTRO_BYTE_A;	//  "@A[i/e][byte][valore]: filtro byte"	// 4.2  gestisce il byte filtering
                          break;
                     case 'B':      // msg byte filter B
                          sm_command = SM_WAIT_FILTRO_BYTE_B;	//  "@B[i/e][byte][valore]: filtro byte"	// 4.4  gestisce il byte filtering
                          break;
                     case 'Y':      // set abbreviation
                          sm_command = SM_WAIT_ABBREV_TYPE;
                          break;
                     case 'y':      // abbreviated write 4 bytes
//						  if (opt.opzioneModo == 'X')
//						  {
							  writeLength = 4;
							  dataLength = 0;
 							  dataByte.data[dataLength++] = 0xA8;
							  check = 0;
							  sm_command = SM_WAIT_WRITE_DATA_BREVE;
//						  }
//						  else
//						  {
//						      putcUSBwait('E');
//	                          sm_command = SM_WAIT_HEADER;
//						  }
                          break;
                     case 'V':      // write stream senza controllo collisioni
						  optionW.W_COLLISION = 1; // controllo collisioni provvisoriamente tolto
                          sm_command = SM_WAIT_WRITE_LENGTH;
                          break;
                     case 'W':      // write stream
                          sm_command = SM_WAIT_WRITE_LENGTH;
                          break;
                     case 'w':      // write single command
                          sm_command = SM_WAIT_WRITE_CMD;
                          break;
                     case 't':      // write single test command
                          sm_command = SM_WAIT_WRITE_QUERY;
                          break;
                     case 'Z':      // write loop test
                          sm_command = SM_WAIT_WRITE_LOOP_I;
                          break;
                     case 'z':      // write loop test
                          sm_command = SM_WAIT_WRITE_LOOP;
                          break;
                     case 'r':      // buffer read immediate
                          sm_command = SM_READ_DEQUEUE;
                          break;
                     case 'R':      // buffer read if available
                          sm_stato = SM_READ_WAIT;
                          sm_command = SM_WAIT_HEADER;
                          break;
                     case 'b':       // buffer clear
                          for (b=0; b<SCSBUFMAX; b++)
                          {
                              for (n=0; n<SCSBUFLEN; n++)
                              {
                                  scsMessageRx[b][n] = 0;
                              }
                          }
                          rBufferIdxW = 0;
                          rBufferIdxR = 0;
                          sm_command = SM_WAIT_HEADER;
						  if (ee_avoid_answer == 0)
								putcUSBwait('k');
                          break;
                     case 'q':       // query version
                          sm_command = SM_WAIT_HEADER;
                          putcUSBwait('k');
                          putrsUSBwait(VERSION);
                          break;
                     case 'Q':       // query version
                          sm_command = SM_WAIT_HEADER;
                          putcUSBwait('k');
                          if  (opt.opzioneModo == 'A')
                          {
                              putrsUSBwait(VERSION);
                          }
                          break;

                     case 'D':       // richiede tabella dispositivi
                          sm_command = SM_WAIT_TABSTART;
                          break;

                     case 'T':       // test mode  @TS
                          if  (opt.opzioneModo == 'A')
                          {
							  m = getUSBwait();
							  if (m != 'S')
								  putcUSBwait('E');
							  else
							  {
								  sm_stato   = SM_TEST_MODE;
								  dataByte.data[0] = 0x9A;
								  dataByte.data[1] = 0xBC;
								  dataByte.data[2] = 0xDE;
								  dataByte.data[3] = 0xF0;
								  dataByte.data[4] = 0x12;
								  dataByte.data[5] = 0x34;
								  dataByte.data[6] = 0x56;
								  dataByte.data[7] = 0x78;
								  queueWrite(8);
								  putrsUSBwait("\r\nTesting: r");
								  putcUSBwait('.');
								  testCounter = 200;
								  rBufferIdxW = 0;
								  rBufferIdxR = 0;
							  }
						  }
						  else
							  putcUSBwait('E');

						  sm_command = SM_WAIT_HEADER;
                          break;
                     case 'S':       // byte stream timeout 
						  stream_timeout = getUSBvalue();
						  opt.stream_timeout = stream_timeout;
						  Write_config();
						  if (ee_avoid_answer == 0)
								putcUSBwait('k');
						  sm_command = SM_WAIT_HEADER;
                          break;
/*
                     case 'S':       // high speed - 115200
                          sm_command = SM_WAIT_HEADER;
						  SPBRGH2  = baud_value[0][BAUD_RATE];
		    			  SPBRG2   = baud_value[1][BAUD_RATE];
                          break;

                     case 's':       // low speed - 38400
                          sm_command = SM_WAIT_HEADER;
						  SPBRGH2  = baud_value[0][BAUD_RATE - 2];
		    			  SPBRG2   = baud_value[1][BAUD_RATE - 2];
                          break;
*/
// ---------------------------- opzioni di test - valide solo in ascii mode-----------
                     case 'h':
                          if (opt.opzioneModo == 'A')
                          {
                              putrsUSBwait(menuScsPrompt);
#if defined(BAUDLOW)
							  if (BAUDLOW == 0)
	                              putrsUSBwait("\r\n- ");
							  else
    	                          putrsUSBwait("\r\n+ ");
#endif
                              putrsUSBwait("\r\nActual Vref: ");
                              itoa((int)opt.Vref,RS_Out_Buffer);
                              putsUSBwait(RS_Out_Buffer);

							  putrsUSBwait("\r\n---> modo...: A");
							  putrsUSBwait("\r\n---> filtro : ");
                              putcUSBwait('0'+opt.opzioneFiltro);

							// 4.2  gestisce il byte filtering -A--------------------------------------------------
							  putrsUSBwait("\r\n---> filtro byte A: ");
							  if ((opt.EfilterByte_A & 0x30) == 0)
								  putrsUSBwait("NO ");
							  else
							  {
								  if (opt.EfilterByte_A & 0x10) putrsUSBwait("include byte nr: ");
								  else
								  if (opt.EfilterByte_A & 0x20) putrsUSBwait("exclude byte nr: ");

								  putcUSBwait('0'+(opt.EfilterByte_A & 0x0F));
								  putrsUSBwait("  - value: ");
								  putcUSBwait(btohexa_high(opt.EfilterValue_A));
								  putcUSBwait(btohexa_low(opt.EfilterValue_A));
							  }
						    // 4.2  gestisce il byte filtering ----------------------------------------------------

							// 4.4  gestisce il byte filtering -B--------------------------------------------------
							  putrsUSBwait("\r\n---> filtro byte B: ");
							  if ((opt.EfilterByte_B & 0x30) == 0)
								  putrsUSBwait("NO ");
							  else
							  {
								  if (opt.EfilterByte_B & 0x10) putrsUSBwait("include byte nr: ");
								  else
								  if (opt.EfilterByte_B & 0x20) putrsUSBwait("exclude byte nr: ");

								  putcUSBwait('0'+(opt.EfilterByte_B & 0x0F));
								  putrsUSBwait("  - value: ");
								  putcUSBwait(btohexa_high(opt.EfilterValue_B));
								  putcUSBwait(btohexa_low(opt.EfilterValue_B));
							  }
							  putrsUSBwait("\r\n---> stream abbreviati: ");
							  putcUSBwait(opt.abbrevia.Val);

							  putrsUSBwait("\r\n---> stream timeout: ");
                              itoa((int)opt.stream_timeout,RS_Out_Buffer);
                              putsUSBwait(RS_Out_Buffer);
							  putrsUSBwait(" ... uS: ");
				//	uS = ((255 - stream_timeout) + 1) * 4
							  i = 256;
							  i -= opt.stream_timeout;
							  i *= 4;
                              itoa(i,RS_Out_Buffer);
                              putsUSBwait(RS_Out_Buffer);

							// 4.2  gestisce il byte filtering ----------------------------------------------------
#if (defined(USE_UART1) && defined(USE_UART2))
// echo=0 : l'output di knxgate viene inviato all'uart che ha fornito l'ultimo input in ordine di tempo
// echo=1 : l'output di knxgate viene inviato ad entrambe le uart indipendentemente dall'input
// echo=2 : l'output di knxgate viene inviato ad entrambe le uart indipendentemente dall'input
//          l'input di uart2    viene inviato solo in output a uart1 ma non a knxgate

							  if (uart_echo == 0) putrsUSBwait("\r\nuart echo OFF");
							  if (uart_echo == 1) putrsUSBwait("\r\nuart echo 1 ON");
							  if (uart_echo == 2) putrsUSBwait("\r\nuart gate USB master");
							  if (uart_echo == 3) putrsUSBwait("\r\nuart gate ESP master");
#endif
                              putrsUSBwait("\r\n");

						  }
                          else
                              putcUSBwait('E');
                          sm_command = SM_WAIT_HEADER;
                          break;

                     case 'l':       // log x test
//                        if  (opt.opzioneModo == 'A')
                          {
							  if (ee_avoid_answer == 0)
									putcUSBwait('k');
                              sm_stato = SM_LOG_WAIT;
                          }
//                        else
//                            putcUSBwait('E');
                          sm_command = SM_WAIT_HEADER;
                          break;
                     case 'c':       // clear stato x test
//                        if  (opt.opzioneModo == 'A')
                          {
                              sm_stato = SM_HOME;
							  if (ee_avoid_answer == 0)
									putcUSBwait('k');
                          }
//                        else
//                            putcUSBwait('E');
                          sm_command = SM_WAIT_HEADER;
                          break;
                     case 'd':       // dump x test
                          if  (opt.opzioneModo == 'A')
                              DisplayScsRxBuffers();
                          else
                              putcUSBwait('E');
                          sm_command = SM_WAIT_HEADER;
                          break;

                     case 'i':       // set internal Vref instead of comparator input
                          if  (opt.opzioneModo == 'A')
                          {
                              opt.Vref = SetInternalVref();
                              putcUSBwait('k');
                              putrsUSBwait("Internal Vref: ");
                              itoa((int)opt.Vref,RS_Out_Buffer);
                              putsUSBwait(RS_Out_Buffer);
                              putrsUSBwait("\r\n");
							  Write_config();
                          }
                          else
                              putcUSBwait('E');
                          sm_command = SM_WAIT_HEADER;
                          break;

#if (( HW == 141113 ) || (HW == 160318))
                     case 'I':       // UPDATE internal Vref
                          if  (opt.opzioneModo == 'A')
                          {
                              x = getUSBwait();
                              if (x == '-')
                              {
                                  opt.Vref--;
								  Write_config();
                              }
                              if (x == '+')
                              {
                                  opt.Vref++;
								  Write_config();
                              }
                              CVRCONbits.CVR = opt.Vref;   // ref value from 0 to 31
                              putrsUSBwait("\r\nInternal Vref: ");
                              itoa((int)opt.Vref,RS_Out_Buffer);
                              putsUSBwait(RS_Out_Buffer);
                              putrsUSBwait("\r\n");
                          }
                          else
                              putcUSBwait('E');
                          sm_command = SM_WAIT_HEADER;
                          break;
#endif

						  //                     case 's':       // sbilancia rx pointer x test
//                          if  (opt.opzioneModo == 'A')
//                          {
//                              IncrementBufferPtr();
//                              putcUSBwait('k');
//                          }
//                          else
//                              putcUSBwait('E');
//                          sm_command = SM_WAIT_HEADER;
//                          break;
// ----------------------- fine opzioni di test --------------------------------------
                     case 'p':       // rx msg fasullo
						  if (rBufferIdxR == 0) rBufferIdxR = SCSBUFMAX;
						  rBufferIdxR--;
						  scsMessageRx[rBufferIdxR][0] = 0x07;
						  scsMessageRx[rBufferIdxR][1] = 0xA8;
						  scsMessageRx[rBufferIdxR][2] = 0x33;
						  scsMessageRx[rBufferIdxR][3] = 0x00;
						  scsMessageRx[rBufferIdxR][4] = 0x02;

						  scsMessageRx[rBufferIdxR][7] = 0xA3;
						  if (pMode == 0)
						  {
							  scsMessageRx[rBufferIdxR][5] = 0x01;
							  scsMessageRx[rBufferIdxR][6] = 0x20;
							  pMode = 1;
        				  }
						  else     
						  {
							  scsMessageRx[rBufferIdxR][5] = 0x00;
							  scsMessageRx[rBufferIdxR][6] = 0x21;
							  pMode = 0;
        				  }
                          sm_command = SM_WAIT_HEADER;
                          break;


                     case '|':       // reset request - first req char
                          sm_command = SM_WAIT_RESET;
                          break;

 // ----------------------- fine opzioni di test --------------------------------------


					 default:
                          putcUSBwait('E');
                          sm_command = SM_WAIT_HEADER;
                          break;
                 }
                 break;

            case SM_WAIT_RESET:  
                 if (choice0 == '|') Reset();
                 putcUSBwait('E');
                 sm_command = SM_WAIT_HEADER;
                 break;

            case SM_WAIT_MODO:    //    "@M[A|X] : modo ascii | hex"
                 if ((choice0 == 'A') || (choice0 == 'X'))
                 {
                     if (opt.opzioneModo != choice0)
					 {
						 opt.opzioneModo = choice0;
						 Write_config();
					 }
					  if (ee_avoid_answer == 0)
							putcUSBwait('k');
                 }
                 else
                     putcUSBwait('E');
                 sm_command = SM_WAIT_HEADER;
                 break;

            case SM_WAIT_FILTRO:  //    "@F[0-7] : filtro"
                 if ((choice0 >= '0') && (choice0 <= '7'))
                 {
                     if (opt.opzioneFiltro != (choice0 - '0'))
					 {
						 opt.opzioneFiltro = choice0 - '0';
	// v 4.3  gestisce opzione di byte filtering per escludere gli ACK
						 if ((opt.opzioneFiltro & 0x02) || ((opt.EfilterByte_A & 0x0F) > 1))
							 opt.EfilterByte_A |= 0x40;      // include messaggi ack
						 else
							 opt.EfilterByte_A &= 0xBF;      // esclude messaggi ack
						 Write_config();
						 filterByte_A = opt.EfilterByte_A;
                            //v 4.2  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
							//v 4.2  HB  0100=exclude ack        1000=exclude equal tlgrm
					        //v 4.2  LB: byte nr su cui filtrare (1-F) - comparato con rByteCount (0=filteroff)
					 }
					  if (ee_avoid_answer == 0)
							putcUSBwait('k');
                 }
                 else
                     putcUSBwait('E');
                 sm_command = SM_WAIT_HEADER;
                 break;

            case SM_WAIT_ABBREV_TYPE:  //    "@Y[0-3] : abbreviazione messaggi
                 if ((choice0 >= '0') && (choice0 <= '3'))
                 {
                     if (opt.abbrevia.Val != choice0)
					 {
						 opt.abbrevia.Val = choice0;
						 Write_config();
					 }
					  if (ee_avoid_answer == 0)
							putcUSBwait('k');
                 }
                 else
                     putcUSBwait('E');
                 sm_command = SM_WAIT_HEADER;
                 break;

            case SM_WAIT_FILTRO_BYTE_A:	//  "@B[i/e][byte][valore]: filtro byte"	// 4.2  gestisce il byte filtering
										// filterByte_A;	//v 4.2  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
										//v 4.2  HB  0100=exclude ack        1000=exclude equal tlgrm
										//v 4.2  LB: byte nr su cui filtrare (1-F) - comparato con rByteCount (0=filteroff)
                 if ((choice0 == 'I') || (choice0 == 'i'))
                 {
					 opt.EfilterByte_A	 = 0x10;
                 }
                 else
                 if ((choice0 == 'E') || (choice0 == 'e'))
                 {
					 opt.EfilterByte_A	 = 0x20;
                 }
                 else
				 {
					 if (opt.EfilterByte_A	 != 0)
					 {
						 opt.EfilterByte_A	 = 0;
						 filterByte_A = 0;
						 Write_config();
					 }
					  if (ee_avoid_answer == 0)
							putcUSBwait('k');
					 sm_command = SM_WAIT_HEADER;
					 break;
				 }
                 sm_command = SM_WAIT_FILTRO_BYTE_A_NR;
				 break;

            case SM_WAIT_FILTRO_BYTE_A_NR:	//  "@B[i/e][byte][valore]: filtro byte"	// 4.2  gestisce il byte filtering
											// filterByte_A;	//v 4.2  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
											//v 4.2  HB  0100=exclude ack        1000=exclude equal tlgrm
											//v 4.2  LB: byte nr su cui filtrare (1-F) - comparato con rByteCount (0=filteroff)
                 if ((choice0 >= '0') && (choice0 <= '9'))
                 {
					 choice0 -= '0';
					 opt.EfilterByte_A	+= choice0;
				 }
                 else
				 {
                     putcUSBwait('E');
					 sm_command = SM_WAIT_HEADER;
					 break;
				 }
                 dataTwin = 0;
                 sm_command = SM_WAIT_FILTRO_BYTE_A_VALUE;
				 break;

            case SM_WAIT_FILTRO_BYTE_A_VALUE:	//  "@B[i/e][byte][valore]: filtro byte"	// 4.2  gestisce il byte filtering
											// filterByte_A;	//v 4.2  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
											//v 4.2  HB  0100=exclude ack        1000=exclude equal tlgrm
											//v 4.2  LB: byte nr su cui filtrare (1-F) - comparato con rByteCount (0=filteroff)
                 if ((choice0 >= '0') && (choice0 <= '9'))
                 {
                     thisHalf = choice0 - '0';
                 }
                 else
                 if ((choice0 >= 'A') && (choice0 <= 'F'))
                 {
                     thisHalf = choice0 - 'A' + 10;
                 }
                 else
                 if ((choice0 >= 'a') && (choice0 <= 'f'))
                 {
                     thisHalf = choice0 - 'a' + 10;
                 }
                 else
                 {
                     thisHalf = 0;
                 }
                 if (dataTwin)
                 {
// v 4.3  gestisce opzione di byte filtering per escludere gli ACK
					 if ((opt.opzioneFiltro & 0x02) || ((opt.EfilterByte_A & 0x0F) > 1))
						 opt.EfilterByte_A |= 0x40;      // esclude messaggi ack

                     thisByte += thisHalf;
                     dataByte.data[dataLength++] = thisByte;
                     opt.EfilterValue_A = thisByte;
					 Write_config();
					 filterByte_A = opt.EfilterByte_A;
					 filterValue_A = opt.EfilterValue_A;
					  if (ee_avoid_answer == 0)
							putcUSBwait('k');
                     sm_command = SM_WAIT_HEADER;
                     dataTwin = 0;
                 }
                 else
                 {
                     thisByte = thisHalf;
                     thisByte<<=4;
                     dataTwin = 1;
                 }
                 break;


            case SM_WAIT_FILTRO_BYTE_B:	//  "@B[i/e][byte][valore]: filtro byte"	// 4.4  gestisce il byte filtering
										// filterByte_B;	//v 4.4  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
										//v 4.4  LB: byte nr su cui filtrare (1-F) - comparato con rByteCount (0=filteroff)
                 if ((choice0 == 'I') || (choice0 == 'i'))
                 {
					 opt.EfilterByte_B	 = 0x10;
                 }
                 else
                 if ((choice0 == 'E') || (choice0 == 'e'))
                 {
					 opt.EfilterByte_B	 = 0x20;
                 }
                 else
				 {
					 if (opt.EfilterByte_B	 != 0)
					 {
						 opt.EfilterByte_B	 = 0;
						 filterByte_B = 0;
						 Write_config();
					 }
					  if (ee_avoid_answer == 0)
							putcUSBwait('k');
					 sm_command = SM_WAIT_HEADER;
					 break;
				 }
                 sm_command = SM_WAIT_FILTRO_BYTE_B_NR;
				 break;

            case SM_WAIT_FILTRO_BYTE_B_NR:	//  "@B[i/e][byte][valore]: filtro byte"	// 4.4  gestisce il byte filtering
											// filterByte_B;	//v 4.4  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
											//v 4.4  LB: byte nr su cui filtrare (1-F) - comparato con rByteCount (0=filteroff)
                 if ((choice0 >= '0') && (choice0 <= '9'))
                 {
					 choice0 -= '0';
					 opt.EfilterByte_B	+= choice0;
				 }
                 else
				 {
                     putcUSBwait('E');
					 sm_command = SM_WAIT_HEADER;
					 break;
				 }
                 dataTwin = 0;
                 sm_command = SM_WAIT_FILTRO_BYTE_B_VALUE;
				 break;

            case SM_WAIT_FILTRO_BYTE_B_VALUE:	//  "@B[i/e][byte][valore]: filtro byte"	// 4.4  gestisce il byte filtering
											// filterByte_B;	//v 4.4  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
											//v 4.4  LB: byte nr su cui filtrare (1-F) - comparato con rByteCount (0=filteroff)
                 if ((choice0 >= '0') && (choice0 <= '9'))
                 {
                     thisHalf = choice0 - '0';
                 }
                 else
                 if ((choice0 >= 'A') && (choice0 <= 'F'))
                 {
                     thisHalf = choice0 - 'A' + 10;
                 }
                 else
                 if ((choice0 >= 'a') && (choice0 <= 'f'))
                 {
                     thisHalf = choice0 - 'a' + 10;
                 }
                 else
                 {
                     thisHalf = 0;
                 }
                 if (dataTwin)
                 {
                     thisByte += thisHalf;
                     dataByte.data[dataLength++] = thisByte;
                     opt.EfilterValue_B = thisByte;
					 Write_config();
					 filterByte_B = opt.EfilterByte_B;
					 filterValue_B = opt.EfilterValue_B;
					  if (ee_avoid_answer == 0)
							putcUSBwait('k');
                     sm_command = SM_WAIT_HEADER;
                     dataTwin = 0;
                 }
                 else
                 {
                     thisByte = thisHalf;
                     thisByte<<=4;
                     dataTwin = 1;
                 }
                 break;


            case SM_WAIT_WRITE_LENGTH:  //    "@W[1-F][data] write"
                 if  (opt.opzioneModo == 'X')
                 {
                     writeLength = choice0;
                 }
                 else
                 if ((choice0 >= '1') && (choice0 <= '9'))
                 {
                     writeLength = choice0 - '0';
                 }
                 else
                 if ((choice0 >= 'A') && (choice0 <= 'F'))
                 {
                     writeLength = choice0 - 'A' + 10;
                 }
                 else
                 if ((choice0 >= 'a') && (choice0 <= 'f'))
                 {
                     writeLength = choice0 - 'a' + 10;
                 }
                 else
                 {
                     putcUSBwait('E');
                     sm_command = SM_WAIT_HEADER;
                     break;
                 }

                 dataLength = 0;
				 if (opt.abbrevia.bits.b1)	// abbreviazione, evitare bytes 0, 5, 6 - ricevere solo 4 bytes
				 {
                     dataByte.data[dataLength++] = 0xA8;
					 writeLength++;
					 check = 0;
				 }
				 dataTwin   = 0;
				 autocheck  = 0;				// 18.90
                 sm_command = SM_WAIT_WRITE_DATA;
                 break;

            case SM_WAIT_WRITE_DATA_BREVE:  //    "@y[data] : write"
				  if (opt.opzioneModo == 'X')
				  {
					 dataByte.data[dataLength++] = choice0;
					 check ^= choice0; 
					 if (dataLength > writeLength)
					 {
						 dataByte.data[dataLength++] = check;
						 dataByte.data[dataLength++] = 0xA3;
						 queueWrite(dataLength);
						  if (ee_avoid_answer == 0)
								putcUSBwait('k');
						 sm_command = SM_WAIT_HEADER;
					 }
				  }
				  else
				  {
					 if ((choice0 >= '0') && (choice0 <= '9'))
					 {
						 thisHalf = choice0 - '0';
					 }
					 else
					 if ((choice0 >= 'A') && (choice0 <= 'F'))
					 {
						 thisHalf = choice0 - 'A' + 10;
					 }
					 else
					 if ((choice0 >= 'a') && (choice0 <= 'f'))
					 {
						 thisHalf = choice0 - 'a' + 10;
					 }
					 else
					 {
						 putcUSBwait('E');
						 sm_command = SM_WAIT_HEADER;
						 break;
					 }
					 if (dataTwin)
					 {
						 thisByte += thisHalf;
						 dataByte.data[dataLength++] = thisByte;
						 check ^= thisByte; 
						 if (dataLength > writeLength)
						 {
							 dataByte.data[dataLength++] = check;
							 dataByte.data[dataLength++] = 0xA3;
							 queueWrite(dataLength);
							  if (ee_avoid_answer == 0)
									putcUSBwait('k');
							 sm_command = SM_WAIT_HEADER;
						 }
						 dataTwin = 0;
					 }
					 else
					 {
						 thisByte = thisHalf;
						 thisByte<<=4;
						 dataTwin = 1;
					 }
				  }
                 break;

            case SM_WAIT_WRITE_DATA:  //    "@W[1-F][data] : write"
                 if (opt.opzioneModo == 'X')
                 {
                     dataByte.data[dataLength++] = choice0;
					 check ^= choice0; 
                     if (dataLength >= writeLength)
                     {
						 if (opt.abbrevia.bits.b1)	// abbreviazione, evitare bytes 0, 5, 6 - ricevere solo 4 bytes
						 {
							 dataByte.data[dataLength++] = check;
							 dataByte.data[dataLength++] = 0xA3;
						 }
						 queueWrite(dataLength);
						  if (ee_avoid_answer == 0)
								putcUSBwait('k');
                         sm_command = SM_WAIT_HEADER;
                     }
                     break;
                 }

                 if ((choice0 >= '0') && (choice0 <= '9'))
                 {
                     thisHalf = choice0 - '0';
                 }
                 else
                 if ((choice0 >= 'A') && (choice0 <= 'F'))
                 {
                     thisHalf = choice0 - 'A' + 10;
                 }
                 else
                 if ((choice0 >= 'a') && (choice0 <= 'f'))
                 {
                     thisHalf = choice0 - 'a' + 10;
                 }
                 else
                 if (choice0 == 'K')						// 18.90
                 {											// 18.90
                     dataByte.data[dataLength++] = autocheck;	// 18.90
                     dataByte.data[dataLength++] = 0xA3;	// 18.90
                     if (dataLength >= writeLength)			// 18.90
                     {										// 18.90
                         queueWrite(dataLength);			// 18.90
						  if (ee_avoid_answer == 0)
								putcUSBwait('k');
                         sm_command = SM_WAIT_HEADER;		// 18.90
                     }										// 18.90
                     dataTwin = 0;							// 18.90
					 break;									// 18.90
				 }											// 18.90
                 else
                 {
                     putcUSBwait('E');
                     sm_command = SM_WAIT_HEADER;
                     break;
                 }
                 if (dataTwin)
                 {
                     thisByte += thisHalf;
                     dataByte.data[dataLength++] = thisByte;
					 check ^= thisByte; 
					 if (dataLength > 0) autocheck ^= thisByte;				// 18.90
                     if (dataLength >= writeLength)
                     {
						 if (opt.abbrevia.bits.b1)	// abbreviazione, evitare bytes 0, 5, 6 - ricevere solo 4 bytes
						 {
							 dataByte.data[dataLength++] = check;
							 dataByte.data[dataLength++] = 0xA3;
						 }
                         queueWrite(dataLength);
						  if (ee_avoid_answer == 0)
								putcUSBwait('k');
                         sm_command = SM_WAIT_HEADER;
                     }
                     dataTwin = 0;
                 }
                 else
                 {
                     thisByte = thisHalf;
                     thisByte<<=4;
                     dataTwin = 1;
                 }
                 break;

            case SM_WAIT_WRITE_CMD:  //  "@w[value][destin] write"
                 if (opt.opzioneModo == 'X')
                 {
                     writeValue = choice0;
                     sm_command = SM_WAIT_WRITE_DESTIN;
                     dataTwin   = 0;
                     break;
                 }
                 if ((choice0 >= '0') && (choice0 <= '9'))
                 {
                     writeValue = choice0 - '0';
                 }
                 else
                 if ((choice0 >= 'A') && (choice0 <= 'F'))
                 {
                     writeValue = choice0 - 'A' + 10;
                 }
                 else
                 if ((choice0 >= 'a') && (choice0 <= 'f'))
                 {
                     writeValue = choice0 - 'a' + 10;
                 }
                 else
                 {
                     putcUSBwait('E');
                     sm_command = SM_WAIT_HEADER;
                     break;
                 }
                 sm_command = SM_WAIT_WRITE_DESTIN;
                 dataTwin   = 0;
                 break;

            case SM_WAIT_WRITE_DESTIN:  //  "@w[value][destin] write"
                 if (opt.opzioneModo == 'X')
                 {
                     writeDest  = choice0;
                 }
                 else
                 {
                     if ((choice0 >= '0') && (choice0 <= '9'))
                     {
                         thisHalf = choice0 - '0';
                     }
                     else
                     if ((choice0 >= 'A') && (choice0 <= 'F'))
                     {
                         thisHalf   = choice0 - 'A' + 10;
                     }
                     else
                     if ((choice0 >= 'a') && (choice0 <= 'f'))
                     {
                         thisHalf   = choice0 - 'a' + 10;
                     }
                     else
                     {
                         putcUSBwait('E');
                         sm_command = SM_WAIT_HEADER;
                         break;
                     }
                     if (dataTwin)
                     {
                         thisByte += thisHalf;
                         writeDest = thisByte;
                     }
                     else
                     {
                         thisByte = thisHalf;
                         thisByte<<=4;
                         dataTwin = 1;
                         break;
                     }
                 }
//        BYTE    TopByte;
//        BYTE	  DestinationAddress;
//        BYTE	  SourceAddress;
//        BYTE    CommandType;
//        BYTE    CommandValue;
//        BYTE    CheckByte;
//        BYTE    BottomByte;
                 dataByte.TopByte = 0xA8;
                 dataByte.DestinationAddress = writeDest;
                 dataByte.SourceAddress = 0x00;

                 dataByte.CommandType = CMDTYPE_SET;	// 18.4A

				 dataByte.CommandValue = writeValue;
                 dataByte.CheckByte = dataByte.data[1]^dataByte.data[2]^dataByte.data[3]^dataByte.data[4];
                 dataByte.BottomByte = 0xA3;
                 queueWrite(7);
				  if (ee_avoid_answer == 0)
						putcUSBwait('k');
                 sm_command = SM_WAIT_HEADER;
                 dataTwin = 0;
                 break;


            case SM_WAIT_TABSTART:  //  "@D[start]"
//BYTE devc[180];	// 0xFF:empty    01:switch    03:dimmer    08:tapparella
//BYTE didx;
                 if (opt.opzioneModo == 'X')
                 {
                     didx  = choice0;
                 }
                 else
                 {
                     if ((choice0 >= '0') && (choice0 <= '9'))
                     {
                         thisHalf = choice0 - '0';
                     }
                     else
                     if ((choice0 >= 'A') && (choice0 <= 'F'))
                     {
                         thisHalf   = choice0 - 'A' + 10;
                     }
                     else
                     if ((choice0 >= 'a') && (choice0 <= 'f'))
                     {
                         thisHalf   = choice0 - 'a' + 10;
                     }
                     else
                     {
                         putcUSBwait('E');
                         sm_command = SM_WAIT_HEADER;
                         break;
                     }
                     if (dataTwin)
                     {
                         thisByte += thisHalf;
                         didx = thisByte;
                     }
                     else
                     {
                         thisByte = thisHalf;
                         thisByte<<=4;
                         dataTwin = 1;
                         break;
                     }
                 }

				 if (didx == 0xFF)
				 {
					didx = 0;
					while (didx < sizeof(devc))
					{
						devc[didx++] = 0xFF;
					}
					didx = 0;
				 }
				 else
				 {
					 while ((devc[didx] == 0xFF) && (didx < 180)) // sizeof(devc))
					 {
						 didx++;
					 }
				 }
                 putcUSBwait('D');
                 if (opt.opzioneModo == 'X')
	                 putcUSBwait(didx);
				 else
	                 puthexUSBwait(didx);
				 if (didx < sizeof(devc))
				 {
                     if (opt.opzioneModo == 'X')
	                     putcUSBwait(devc[didx]);
					 else
	                     puthexUSBwait(devc[didx]);
				 }
				 else
				 {
                     if (opt.opzioneModo == 'X')
	                     putcUSBwait(0xFF);
					 else
	                     puthexUSBwait(0xFF);
				 }

				 sm_command = SM_WAIT_HEADER;
                 dataTwin = 0;
                 break;


            case SM_WAIT_WRITE_QUERY:  //  "@t[destin] write test query"
                 if (opt.opzioneModo == 'X')
                 {
                     writeDest  = choice0;
                 }
                 else
                 {
                     if ((choice0 >= '0') && (choice0 <= '9'))
                     {
                         thisHalf = choice0 - '0';
                     }
                     else
                     if ((choice0 >= 'A') && (choice0 <= 'F'))
                     {
                         thisHalf   = choice0 - 'A' + 10;
                     }
                     else
                     if ((choice0 >= 'a') && (choice0 <= 'f'))
                     {
                         thisHalf   = choice0 - 'a' + 10;
                     }
                     else
                     {
                         putcUSBwait('E');
                         sm_command = SM_WAIT_HEADER;
                         break;
                     }
                     if (dataTwin)
                     {
                         thisByte += thisHalf;
                         writeDest = thisByte;
                     }
                     else
                     {
                         thisByte = thisHalf;
                         thisByte<<=4;
                         dataTwin = 1;
                         break;
                     }
                 }
//        BYTE    TopByte;
//        BYTE	  DestinationAddress;
//        BYTE	  SourceAddress;
//        BYTE    CommandType;
//        BYTE    CommandValue;
//        BYTE    CheckByte;
//        BYTE    BottomByte;
                 dataByte.TopByte = 0xA8;
                 dataByte.DestinationAddress = writeDest;
                 dataByte.SourceAddress = 0x00;
				 
                 dataByte.CommandType = CMDTYPE_QUERY;	// 18.4A

				 dataByte.CommandValue = 0;
                 dataByte.CheckByte = dataByte.data[1]^dataByte.data[2]^dataByte.data[3]^dataByte.data[4];
                 dataByte.BottomByte = 0xA3;
                 queueWrite(7);
				  if (ee_avoid_answer == 0)
						putcUSBwait('k');
                 sm_command = SM_WAIT_HEADER;
                 dataTwin = 0;
                 break;

            case SM_WAIT_WRITE_LOOP_I:  //  "@Z[value] write loop"
                 if (opt.opzioneModo == 'X')
                 {
                     writeDest  = choice0;
                 }
                 else
                 {
                     if ((choice0 >= '0') && (choice0 <= '9'))
                     {
                         thisHalf = choice0 - '0';
                     }
                     else
                     if ((choice0 >= 'A') && (choice0 <= 'F'))
                     {
                         thisHalf   = choice0 - 'A' + 10;
                     }
                     else
                     if ((choice0 >= 'a') && (choice0 <= 'f'))
                     {
                         thisHalf   = choice0 - 'a' + 10;
                     }
                     else
                     {
                         putcUSBwait('E');
                         sm_command = SM_WAIT_HEADER;
                         break;
                     }
                     if (dataTwin)
                     {
                         thisByte += thisHalf;
                         writeDest = thisByte;
                     }
                     else
                     {
                         thisByte = thisHalf;
                         thisByte<<=4;
                         dataTwin = 1;
                         break;
                     }
                 }
                 dataByte.data[1] = writeDest;
                 INTCONbits.GIEH     = 0;
                 INTCONbits.GIEL     = 0;
                 scsSendTest(writeDest);
                 PIR4bits.CMP1IF  = 0;    // clear interrupt
                 PIR2bits.TMR3IF  = 0;    // clear INTERRUPT FLAG
                 INTCONbits.TMR0IF = 0;   // clear INTERRUPT FLAG
                 INTCONbits.GIEH     = 1; // high priority interrupt enabled
                 INTCONbits.GIEL     = 1; // low  priority interrupt enabled
                 putcUSBwait('k');
                 sm_command = SM_WAIT_HEADER;
                 dataTwin = 0;
                 break;

            case SM_WAIT_WRITE_LOOP:  //  "@z[value] write loop"
                 if (opt.opzioneModo == 'X')
                 {
                     writeDest  = choice0;
                 }
                 else
                 {
                     if ((choice0 >= '0') && (choice0 <= '9'))
                     {
                         thisHalf = choice0 - '0';
                     }
                     else
                     if ((choice0 >= 'A') && (choice0 <= 'F'))
                     {
                         thisHalf   = choice0 - 'A' + 10;
                     }
                     else
                     if ((choice0 >= 'a') && (choice0 <= 'f'))
                     {
                         thisHalf   = choice0 - 'a' + 10;
                     }
                     else
                     {
                         putcUSBwait('E');
                         sm_command = SM_WAIT_HEADER;
                         break;
                     }
                     if (dataTwin)
                     {
                         thisByte += thisHalf;
                         writeDest = thisByte;
                     }
                     else
                     {
                         thisByte = thisHalf;
                         thisByte<<=4;
                         dataTwin = 1;
                         break;
                     }
                 }
                 dataByte.data[1] = writeDest;
                 scsSendTest(writeDest);
                 putcUSBwait('k');
                 sm_command = SM_WAIT_HEADER;
                 dataTwin = 0;
                 break;
        }
    }
}

// ******************************************************************************/
// * transfer info from input & status to logic output                          */
// ******************************************************************************/
void TransferFunction(void)
{
    if (PIR1bits.TMR1IF) //   32mSec
    {
        SystemTicks++;
        if (SystemTicks > 31) // 1,024 secondi
        {
            SystemTicks = 0;
        }
        PIR1bits.TMR1IF = 0;     // clear INTERRUPT FLAG
#if defined(LED_INT)
        LED_INT = 0;
#endif
        if ((ledLamps != 0) && (Ticks++ > ledLamps))	// 18.90
        {
            Ticks = 0;
#if defined(LED_SYS)
            LED_SYS = 1;
#endif
#if defined(LED_SYS1)
            LED_SYS1= 1;
#endif
        }
        else
        {
#if defined(LED_SYS)
            LED_SYS = 0;
#endif
#if defined(LED_SYS1)
            LED_SYS1= 0;
#endif
        }
    }
}
// ===================================================================================
BYTE MsgValido(void)
{
BYTE f,l,m;
BYTE valido;

	valido = 1;

	if  ((scsMessageRx[rBufferIdxR][0] == 1) && (scsMessageRx[rBufferIdxR][1] == 0xFF))
		  valido = 0;
	else
	if  ((scsMessageRx[rBufferIdxR][0] == 2) && (scsMessageRx[rBufferIdxR][1] == 0xFF)&& (scsMessageRx[rBufferIdxR][2] == 0xFF))
		  valido = 0;

	if (opt.opzioneFiltro & 0x04)                                  // esclude messaggi di stato
	{
		if  ((scsMessageRx[rBufferIdxR][0] > 2) && (scsMessageRx[rBufferIdxR][3] != 0))
		  valido = 0;
	}
	if (opt.opzioneFiltro & 0x02)       // esclude messaggi ack
	{
		if  ((scsMessageRx[rBufferIdxR][0] == 1) && (scsMessageRx[rBufferIdxR][1] == 0xA5))
		  valido = 0;
	}
	if ((opt.opzioneFiltro & 0x01)       // esclude messaggi doppi
	&&  (rBufferIdxR != rBufferIdxP)
	&&  (rBufferIdxP != 0xFF))
	{
		f = 1;
		l = scsMessageRx[rBufferIdxR][0];
		for (m=0; m<=l;m++)
		{
			if (scsMessageRx[rBufferIdxR][m] != scsMessageRx[rBufferIdxP][m])   f = 0;
		}
		if (f)     valido = 0;
	}
	return valido;
}
// ===================================================================================





// ===================================================================================

// ******************************************************************************/
// * output analyze and mapping                                                 */
// ******************************************************************************/
void OutputMapping(void)
{
BYTE f,l,m;
BYTE increment;

	increment = 0;

    if  (rBufferIdxR != rBufferIdxW)
	{
		if (MsgValido())
		{
			rBufferIdxP = rBufferIdxR;

			if (sm_command ==  SM_READ_DEQUEUE)
			{
				AnswerMsg();
				increment = 1;
				sm_command = SM_WAIT_HEADER;
			}

			if (sm_stato   ==  SM_TEST_MODE)
			{
				if ((scsMessageRx[rBufferIdxR][0] == 8) &&
					(scsMessageRx[rBufferIdxR][1] == dataByte.data[0]) &&
					(scsMessageRx[rBufferIdxR][2] == dataByte.data[1]) &&
					(scsMessageRx[rBufferIdxR][3] == dataByte.data[2]) &&
					(scsMessageRx[rBufferIdxR][4] == dataByte.data[3]) &&
					(scsMessageRx[rBufferIdxR][5] == dataByte.data[4]) &&
					(scsMessageRx[rBufferIdxR][6] == dataByte.data[5]) &&
					(scsMessageRx[rBufferIdxR][7] == dataByte.data[6]) &&
					(scsMessageRx[rBufferIdxR][8] == dataByte.data[7]) )
					putcUSBwait('k');
				else
				{
					putcUSBwait('N');
					if (opt.opzioneModo == 'A')
						LogScsDisplay();
					testCounter = 0;
					putrsUSBwait("\r\nTest FAILED!!!");
					sm_stato = SM_HOME;
				}
				dataByte.data[0]++;
				dataByte.data[1]--;
				dataByte.data[2]++;
				dataByte.data[3]--;
				dataByte.data[4]++;
				dataByte.data[5]--;
				dataByte.data[6]++;
				queueWrite(8);

				putcUSBwait('.');
				testCounter--;
				if (testCounter == 0) 
				{
					sm_stato = SM_HOME;
					putrsUSBwait("\r\nTest O.K.!");
				}
				increment = 1;
			}

//   A8 <dest> <source> <type> <cmd> XX A3
//          dest B0-.. pulsanti
// type=12 (comandi) o 15 (query)
//          cmd=00 switch/dim ON
//          cmd=01 switch/dim OFF
//          cmd=03 dimmer  + MORE (?)
//          cmd=04 dimmer  - LESS (?)
//          cmd=08 tapparella ALZA
//          cmd=09 tapparella ABBASSA
//          cmd=0A tapparella STOP
//
//          cmd= 80-FE intensita dimmer raggiunta

//BYTE devc[180];	// 0xFF:empty    01:switch    03:dimmer    08:tapparella
//BYTE didx;

//			if (sm_stato   ==  SM_READ_WAIT)
			{
				if ((scsMessageRx[rBufferIdxR][0] == 7) &&
					(scsMessageRx[rBufferIdxR][1] == 0xA8) &&
					(scsMessageRx[rBufferIdxR][4] == 0x12) &&
					(scsMessageRx[rBufferIdxR][7] == 0xA3) )
				{
					if (scsMessageRx[rBufferIdxR][2] < 0xB0)
						didx = scsMessageRx[rBufferIdxR][2];
					else
						didx = scsMessageRx[rBufferIdxR][3];

					if ((didx > 0) && (didx < 180)) // (sizeof(devc)))
					{
						switch (scsMessageRx[rBufferIdxR][5])
						{
						case 0:
						case 0x01:
							if (devc[didx] == 0xFF)
								devc[didx] = 1;
							break;
						case 0x03:
						case 0x04:
							devc[didx] = 3;
							break;
						case 0x08:
						case 0x09:
						case 0x0A:
							devc[didx] = 8;
							break;
						default:
							if ((scsMessageRx[rBufferIdxR][5] & 0x0F) == 0x0D)
								devc[didx] = 3;
							break;
						}
					}
				}
			}

			if (sm_stato   ==  SM_READ_WAIT)
			{
				AnswerMsg();
				increment = 1;
				sm_stato = SM_HOME;
			}

			if (sm_stato   ==  SM_LOG_WAIT)
			{
				if (opt.opzioneModo == 'A')
					LogScsDisplay();
				else
					AnswerMsg();

				increment = 1;
			}

			if (sm_stato   ==  SM_ECHO_ON)
			{
				l = scsMessageRx[rBufferIdxR][0];
				f = 1;
				m = 0;
				while (m < l)
				{
        			dataByte.data[m++] = scsMessageRx[rBufferIdxR][f++];
				}
				queueWrite(l);
				increment = 1;
			}
			if (increment)
				IncrementBufferPtr();
		}
		else
			IncrementBufferPtr();
	}
	else
	{
		if (sm_command ==  SM_READ_DEQUEUE)
		{
			AnswerMsg();
			sm_command = SM_WAIT_HEADER;
		}
	}
}

// ==========================get  USB via USART======================================

char getUSBwait(void)
{
BYTE c;
#if defined(USE_UART2)
    if (uart_echo != 3)
	{
		if (uart_in_use == 2)
		{
			if (RCSTA2bits.OERR) {
				RCSTA2bits.CREN = 0;
				RCSTA2bits.CREN = 1;
			}
			while (PIR3bits.RC2IF == 0)  ClrWdt();
			c = RCREG2;
			PIR3bits.RC2IF = 0;
			uartRc = 1;
			return c;
		}
	}
#endif

#if defined(USE_UART1)
	if (uart_in_use == 1)
    {
        if (RCSTA1bits.OERR) {
            RCSTA1bits.CREN = 0;
            RCSTA1bits.CREN = 1;
        }
        while (PIR1bits.RC1IF == 0)  ClrWdt();
        c = RCREG1;
        PIR1bits.RC1IF = 0;
        uartRc = 1;
        return c;
    }
#endif
    uartRc = 0;
    return 0;
}
// ===================================================================================
BYTE getUSBvalue(void)
{
BYTE in, res;
     res = 0;
     in = getUSBwait();
     while ((in >= '0') && (in <= '9'))
     {
         in -= '0';
         res *= 10;
         res += in;
         in = getUSBwait();
     }
     return res;
}

// ==========================read USB via USART======================================
char getUSBnowait(void)
{
BYTE c;
    c = 0;
#if defined(USE_UART2)
    if (uart_echo != 3)
	{
		if (RCSTA2bits.OERR) {
			RCSTA2bits.CREN = 0;
			RCSTA2bits.CREN = 1;
		}
		if (PIR3bits.RC2IF != 0)
		{
			c = RCREG2;
			PIR3bits.RC2IF = 0;
	// echo=0 : l'output di knxgate viene inviato all'uart che ha fornito l'ultimo input in ordine di tempo
	// echo=1 : l'output di knxgate viene inviato ad entrambe le uart indipendentemente dall'input
	// echo=2 : l'output di knxgate viene inviato ad entrambe le uart indipendentemente dall'input
	//          l'input di uart2  (USB)  viene inviato solo in output a uart1 (ESP) ma non a knxgate
	// echo=3 : uart1 (ESP) master,  uart2 echo di tutto il traffico in input e output
		


	#if defined(USE_UART1)
			if (uart_echo == 2)
			{						// ricevuto da USB -> inviato a ESP
				while (TXSTA1bits.TRMT == 0); // wait if the buffer is full 
				TXREG1 = c; // transfer data word to TX reg 
				c = 0;
			}
			else
	#endif
			{
				uartRc = 1;
				uart_in_use = 2;
				return c;
			}
		}
	}
#endif

#if defined(USE_UART1)
    if (RCSTA1bits.OERR) {
        RCSTA1bits.CREN = 0;
        RCSTA1bits.CREN = 1;
    }
    if (PIR1bits.RC1IF != 0)
    {
		PIR1bits.RC1IF = 0;
		c = RCREG1;
		uartRc = 1;
		uart_in_use = 1;

#if defined(USE_UART2)
		if (uart_echo == 3)
		{						// ricevuto da ESP -> inviato a USB
            while (TXSTA2bits.TRMT == 0); // wait if the buffer is full 
            TXREG2 = c; // transfer data word to TX reg 
		}
#endif
		return c;
    }
#endif
    uartRc = 0;
    return c;
}

// ==========================write USB via USART=====================================
void putcUSBwait (BYTE data)
{
// echo=0 : l'output di knxgate viene inviato all'uart che ha fornito l'ultimo input in ordine di tempo
// echo=1 : l'output di knxgate viene inviato ad entrambe le uart indipendentemente dall'input
// echo=2 : l'output di knxgate viene inviato ad entrambe le uart indipendentemente dall'input
//          l'input di uart2    viene inviato solo in output a uart1 ma non a knxgate
// echo=3 : uart1 (ESP) master,  uart2 echo di tutto il traffico in input e output

#if defined(USE_UART2)
	if ((uart_in_use == 2) || (uart_echo))
    {
        {
            while (TXSTA2bits.TRMT == 0); // wait if the buffer is full 
            TXREG2 = data; // transfer data word to TX reg 
        }
    }
#endif

#if defined(USE_UART1)
	if ((uart_in_use == 1) || (uart_echo))
    {
        {
            while (TXSTA1bits.TRMT == 0); // wait if the buffer is full 
            TXREG1 = data; // transfer data word to TX reg 
        }
    }
#endif
}
// ==========================write USB via USART=====================================
void puthexUSBwait (BYTE data)
{
	putcUSBwait(btohexa_high(data));
	putcUSBwait(btohexa_low(data));
}
// ==========================write USB via USART=====================================
void putsUSBwait(char *data)
{
	char * ptr;
#if defined(USE_UART2)
	if ((uart_in_use == 2) || (uart_echo))
    {
		ptr = data;
        while(*ptr != '\0')
        {
            while (TXSTA2bits.TRMT == 0); // wait if the buffer is full 
            TXREG2 = *ptr++; // transfer data word to TX reg 
        }
    }
#endif
#if defined(USE_UART1)
	if ((uart_in_use == 1) || (uart_echo))
    {
		ptr = data;
	    while(*ptr != '\0')
        {
            while (TXSTA1bits.TRMT == 0); // wait if the buffer is full 
            TXREG1 = *ptr++; // transfer data word to TX reg 
        }
    }
#endif
}


// ==========================write USB via USART=====================================
void putrsUSBwait(const ROM char *data)
{
const ROM	char * ptr;
#if defined(USE_UART2)
	if ((uart_in_use == 2) || (uart_echo))
    {
		ptr = data;
        // transmit till NULL character is encountered 
        while(*ptr != '\0')
        {
            while (TXSTA2bits.TRMT == 0); // wait if the buffer is full 
            TXREG2 = *ptr++; // transfer data word to TX reg 
        }
    }
#endif
#if defined(USE_UART1)
	if ((uart_in_use == 1) || (uart_echo))
    {
		ptr = data;
        // transmit till NULL character is encountered 
        while(*ptr != '\0')
        {
            while (TXSTA1bits.TRMT == 0); // wait if the buffer is full 
            TXREG1 = *ptr++; // transfer data word to TX reg 
        }
    }
#endif
}
// ===================================================================================











// ===================================================================================


/******************************************************************************
 ******************************************************************************
 * Main Application code                                                      *
 ******************************************************************************
 ******************************************************************************/
void main(void)
{
 BYTE i;

//	rBitCount   = 0;  // solo per test
//	rByteCount  = 0;  // solo per test
    rBufferIdxW = 0;
    rBufferIdxR = 0;

	filterByte_A = 0;	// 4.2
	filterValue_A = 0;	// 4.2
	filterByte_B = 0;	// 4.4
	filterValue_B = 0;	// 4.4

	dataTwin = 0;
	optionR.Val = 0;
	optionW.Val = 0;
	optionW.W_END = 1;

	opt.Vref = 20;     // 3,3V                ogni step 3.3/32 = 0,10Volt - 2,00V

    DelayMs(100);
    ClrWdt();
	Read_eep_array(&opt, EE_CONFIG, (char) sizeof(opt));

    if (opt.eeVersion != EEPROM_VER)
    {
		eepromInit();
    }

	if (opt.abbrevia.Val == 0xFF)	opt.abbrevia.Val = '0';
	filterByte_A = opt.EfilterByte_A;
	filterValue_A = opt.EfilterValue_A;
	filterByte_B = opt.EfilterByte_B;
	filterValue_B = opt.EfilterValue_B;
	stream_timeout	= opt.stream_timeout;

	ClrWdt();

    InitializeBoard();    // Initialize hardware

    TickInit();                // Initialize timers
    DelayMs(150);

    Ticks = 0;

	scsInit();
	opt.stream_timeout	= stream_timeout;

    AppInitialize();

	didx = 0;
	while (didx < sizeof(devc))
	{
		devc[didx++] = 0xFF;
	}

    while(1)
    {
// ===================================================================================
        ClrWdt();
        InputMapping();      // read input ports
        TransferFunction();  // from inputs to outputs
        OutputMapping();     // setting output ports
// ===================================================================================
    }
}
// ===================================================================================



void queueWrite(char dataLength)
{
BYTE clear;
BYTE n, msgLen;
BYTE retry;
	retry = 0;
	do
	{
		n = 0;
		DelayMs(3);

	// attende che termini eventuale scrittura in corso - ( inutile, si usa scrittura diretta ! )
	  while ((optionW.W_END == 0) && (n < 200)) // optionW.bits.b7 (1) = write_end
		{
			n++;
			DelayMs(1);
			ClrWdt();
		}

		msgLen = dataLength;
		if (msgLen > 15) msgLen = 15;
		for (n=0; n<msgLen; n++)
		{
			scsMessageTx[n] = dataByte.data[n];
		}
		if (sm_stato   ==  SM_LOG_WAIT)
			if (opt.opzioneModo == 'A')
				DisplayScsTx(msgLen);

	// attende che termini eventuale telegramma in arrivo - anti collisione PREVENTIVO
		clear = 4;
		n = 0;
		while ((clear) && (n < 200))
		{
			DelayMs(1);
			if ((optionR.R_READING))// || (INPUT_SCS == 1)) 	// lettura in corso !
			{
				ClrWdt();
				clear = 4;
				n++;
			}
			else
				clear--;
		}

		INTCONbits.GIEH     = 0;
		INTCONbits.GIEL     = 0;
		n = scsSendWait(msgLen);
		retry++;
	} while ((optionW.W_ERROR == 1) && (retry < 50));

	PIR4bits.CMP1IF  = 0;    // clear interrupt
    PIR2bits.TMR3IF  = 0;    // clear INTERRUPT FLAG
    INTCONbits.TMR0IF = 0;   // clear INTERRUPT FLAG
    INTCONbits.GIEH     = 1; // high priority interrupt enabled
    INTCONbits.GIEL     = 1; // low  priority interrupt enabled
    optionW.W_COLLISION = 0; // controllo collisioni ripristinato
	optionR.Val = 0;
    DelayMs(1);
}
// ===================================================================================
void DisplayScsTx(BYTE len)
{
    BYTE dev, cmd, m, n, b;
    strcpypgm2ram(RS_Out_Buffer, "\r\nSnd[  ]:  ");
    RS_Out_Buffer[6] = btohexa_high(len);
    RS_Out_Buffer[7] = btohexa_low(len);
    m = 11;
    for (n=0; n<len; n++)
    {
        RS_Out_Buffer[m++] = btohexa_high(scsMessageTx[n]);
        RS_Out_Buffer[m++] = btohexa_low (scsMessageTx[n]);
        RS_Out_Buffer[m++] = ' ';
    }
    RS_Out_Buffer[m] = 0;
    putsUSBwait(RS_Out_Buffer);
}
// ===================================================================================
void DisplayScsRxBuffers(void)
{
    BYTE m, n, b;
//  WORD_VAL tmp;
    for (b=0; b<SCSBUFMAX; b++)
    {
        strcpypgm2ram(RS_Out_Buffer, "\r\nBuff[ ]:  ");
        RS_Out_Buffer[7] = btohexa_low(b);
        m = 11;
        for (n=0; n<SCSBUFLEN; n++)
        {
            RS_Out_Buffer[m++] = btohexa_high(scsMessageRx[b][n]);
            RS_Out_Buffer[m++] = btohexa_low (scsMessageRx[b][n]);
//          RS_Out_Buffer[m++] = ' ';
        }
        RS_Out_Buffer[m] = 0;
        putsUSBwait(RS_Out_Buffer);
    }
    strcpypgm2ram(RS_Out_Buffer, "\r\nidxW=     ");
    RS_Out_Buffer[8] = btohexa_high(rBufferIdxW);
    RS_Out_Buffer[9] = btohexa_low (rBufferIdxW);
    putsUSBwait(RS_Out_Buffer);
    strcpypgm2ram(RS_Out_Buffer, "\r\nidxR=     \r\n");
    RS_Out_Buffer[8] = btohexa_high(rBufferIdxR);
    RS_Out_Buffer[9] = btohexa_low (rBufferIdxR);
    putsUSBwait(RS_Out_Buffer);
}
// ===================================================================================
void WriteByteEEP(BYTE Data, int Addr)
{
    {
        Write_b_eep (Addr, Data);
        Busy_eep ();
    }
}
// ===================================================================================
void Read_eep_array(BYTE *Data, int Address, char Len)
{
    char  j;
    for (j=0; j<Len; j++)
    {
        *Data++ = Read_b_eep (Address++);
    }
}
// ===================================================================================
void Write_eep_array(BYTE *Data, int Address, char Len)
{
    char  j;
    for (j=0; j<Len; j++)
    {
        Write_b_eep (Address++, *Data++);
        Busy_eep ();
    }
}
// ===================================================================================
void Write_config(void)
{
	if (ee_avoid_memo == 0)
		Write_eep_array(&opt, EE_CONFIG, (char) sizeof(opt));
}
// ===================================================================================
BYTE btohexa_high(BYTE b)
{
        b >>= 4;
        return (b>0x9u) ? b+'A'-10:b+'0';
}
BYTE btohexa_low(BYTE b)
{
        b &= 0x0F;
        return (b>9u) ? b+'A'-10:b+'0';
}
// ===================================================================================
