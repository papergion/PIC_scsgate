#define TITLE    "ScsGAte"

#define VERSION  "SCS 19.37E"
#define EEPROM_VER	0x8D		// per differenziare scs e knx
#define UART1_BUFFER  64        // numero bytes buffer uart1 interrupt
#define UART1_INTERRUPT         // numero bytes buffer uart1 interrupt
//#define UART1_CALL            // numero bytes buffer uart1 interrupt
#define IMMEDIATE_ANSWER        // ricava lo stato anche dal comando (non solo dalla risposta di stato)
#define DEV_NR		175	        // numero elementi tabella devices (0xAF)

#define inUART()   getUART()

void getUART(void);

//		i comandi interni hanno un particolare formato:
//          § data  (§ sostituisce @)
//                        § D <address>    per richiesta censimento dispositivi
//		                  § U <comando>
//		                  § u <indirizzo> <valore>
//		                  § y <comandi>
//		                  § l              short log request
//      risposte di comunicazione interna     
//          0xF1-5
//                        0xf3  D <address> <tipo>
//		                  0xf3  u <indirizzo> <valore>
//		                  0xf5  y <stati> log


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
//          cmd=1D-9D intensita dimmer raggiunta

// 19.37 - aggiornamento manuale setup tapparelle - revisione processi di comunicazione UART
// 19.36 - uart1 rx in interrupt - usa SCS_UART.ASM  invece di SCSINTRP.ASM
// 19.35 - gestione tapparelle a % - nuovo comando diretto @u - configurazione @Ux  0=no   1=raccogli i dati   2=online   5=list 9=ripulisci
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
#define EE_CONFIG		0x00   // indirizzo eeprom configurazione
#define EE_TAPPA_SETUP  0x20   // indirizzo eeprom tabella setup tapparelle
// ===================================================================================
#pragma udata DATARS
char RS_Out_Buffer[120];
// ===================================================================================
#pragma udata TABDEV
BYTE devc[DEV_NR];	// 0xFF:empty    01:switch    03:dimmer    08:tapparella
BYTE didx;
BYTE dlen;
// ===================================================================================
#pragma udata tapdata
#define MAXTAPP 25

#define UP   8
#define DOWN 9
#define STOP 0x0A

typedef union _TAPSETUP    {
  struct {
        BYTE device;		//      [] indirizzo 
        WORD maxposition;	//      [] posizione massima (in unità da 0,1 secondi)
        };
  struct {
        BYTE adevice;		//      [] indirizzo 
        WORD_VAL amaxposition;	//      [] posizione massima (in unità da 0,1 secondi)
        };
} TAP_SETUP;

typedef union _TAPDATA     {
  struct {
        BYTE_VAL direction;		//      [] half.LB  direzione (0=ferma  9=giu  8=su )      bit7 = 0 : da pubblicare       1 : pubblicato
        WORD	position;		//      [] posizione attuale (in unità da 0,1 secondi)
        WORD	request;		//      [] posizione richiesta tapparella (in unità da 0,1 secondi) - nulla = 0xFFFF
        BYTE	timeout;		//      [] timeout attuale (in unita da 0,1 secondi)
        };
  struct {
        BYTE_VAL adirection;	//      [] half.LB  direzione (0=ferma  9=giu  8=su )      bit7 = 0 : da pubblicare       1 : pubblicato
        WORD_VAL aposition;		//      [] posizione attuale (in unità da 0,1 secondi)
        WORD_VAL arequest;		//      [] posizione richiesta tapparella (in unità da 0,1 secondi) - nulla = 0xFFFF
        BYTE_VAL atimeout;		//      [] timeout attuale (in unita da 0,1 secondi)
        };
} TAP_DATA;
// ===================================================================================
TAP_SETUP devicetappa[MAXTAPP];
BYTE      maxtapp = 0;
BYTE      ixTapp;
BYTE      ixPublish = 0;

TAP_DATA  tapparella [MAXTAPP];
// ===================================================================================


#pragma udata
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
static BYTE         TappaMoveTicks;
static BYTE         TappaPublishTicks;
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
// echo=4 : l'output di knxgate viene inviato solo a UART1
//          l'input di uart1    viene inviato anche in output a uart2
//          l'input di uart2    viene ignorato

static BYTE         uart_echo = 4;    
#else
static BYTE         uart_echo = 0;
#endif
static int errorCollision = 0;
static int errorChecksum = 0;
BYTE	uartTrace = 0;
#pragma udata
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
		SM_WAIT_WRITE_TAPP_ADDR,//  "@u<address><pct>   comando tapparelle a %  
		SM_WAIT_WRITE_TAPP_POS, //  "@u<address><pct>   comando tapparelle a %
		SM_WAIT_WRITE_TAPP_SETUP,//  @U<opz>   setup tapparelle a %  
									 //   '0':	close % mode
									 //   '1':  data collection
									 //   '2':  consolida e apre (se sono state ABBASSATE e alzate tapparelle
									 //   '3':  open % mode
									 //   '4':  open without publish position
									 //   '5':  lista ascii
									 //   '6':  dati singola tapparella hex mode 
									 //   '7':  ri-pubblica tutte le posizioni
									 //   '9':  pulisce tutta la tabella

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
        SM_TEST_MODE,
        SM_TEST_W_MODE
    }   sm_stato   = SM_HOME;
// ===================================================================================
    enum _SM_MODO
    {
        SM_MODO_UDP,
        SM_MODO_INTERNO
    }   sm_modo   = SM_MODO_UDP;
// ===================================================================================
BYTE    writeLength;            //    [1-F] data length
BYTE    dataLength;             //    [1-F] data length
BYTE	testCounter;
BYTE    testPhase;
BYTE	test_destin;
BYTE	test_source;
BYTE	test_command;
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
			BYTE	    tapparelle_pct; // 0=no   1=raccogli i dati   2=consolida   3=work   9=ripulisci
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
// ==============================================================================================================================
extern	            char rBufferIdxR;	// pointer msgbuffer: prossimo messaggio da scodare (se diverso da idxW) 
extern volatile far BYTE rBufferIdxW;	// pointer msgbuffer: prossimo messaggio da accodare da parte della routine di interrupt
					char rBufferIdxP;	// pointer msgbuffer: ultimo messaggio scodato
					char rBufferIdxInternal;	// pointer msgbuffer: ultimo messaggio scodato per uso interno
// -------------------------------------------------------------------------------------------------------------------------------
extern	BYTE filterByte_A;	//v 4.2  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
							//v 4.2  HB  0100=exclude ack        1000=exclude equal tlgrm
					        //v 4.2  LB: byte nr su cui filtrare (1-F) - comparato con rByteCount (0=filteroff)
extern BYTE filterValue_A;	//v 4.2  valore byte di filtro   (0-FF)
extern BYTE filterByte_B;	//v 4.4  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
							//v 4.4  LB: byte nr su cui filtrare (1-F) - comparato con rByteCount (0=filteroff)
extern BYTE filterValue_B;	//v 4.4  valore byte di filtro   (0-FF)
extern BYTE stream_timeout; //v 4.7  timeout di stream (il complemento a 255 diviso 4 è il timeout in uS)
// ==============================================================================================================================
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
extern volatile far BYTE_VAL optionW;
 #define      W_COLLISION  bits.b1	//; non usato
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
#ifdef UART1_BUFFER
extern	far   char uartMessage[UART1_BUFFER];	// buffer rx uart1
extern  volatile far   char uPtrW;				// ptr next mem (scrittura nel buffer)
extern  volatile far   char uMax;				// nr char depositati nel buffer
extern  volatile far   char uState;				// uart error
extern  volatile far   char uSpikes;			// spike errors
char    uPtrR = 0;								// ptr scodamento (lettura del buffer)
#endif
int uartFERR = 0;
int uartOERR = 0;
// ===================================================================================
BYTE btohexa_high(BYTE b);
BYTE btohexa_low(BYTE b);
char getUSBwait(void);
BYTE getUSBhex(void);
char getUSBnowait(void);
WORD getUSBvalue(void);
void putcUSBwait(BYTE  data);
void putsUSBwait(char *data);
void putrsUSBwait(const ROM char *data);
void puthexUSBwait(BYTE  data);
void WriteByteEEP(BYTE Data, int Addr);
void Read_eep_array (BYTE *Data, int Address, char Len);
void Write_eep_array(BYTE *Data, int Address, char Len);
void Write_config(BYTE mode);
void Read_tapparelle(void);
void Write_tapparelle(void);

static void InitializeBoard(void);
void TickInit(void);
void AppInitialize(void);
void IncrementBufferPtr(void);
void LogScsDisplay(void);
void AnswerMsg(void);
void AnswerMsgInternal(void);
void InputMapping(void);
void TransferFunction(void);
void OutputMapping(void);
void queueWrite(char dataLength);
void DisplayScsTx(BYTE len);
void DisplayScsRxBuffers(void);
void UserProcess(void);
void TapparellaAction(char ixTapp, char action);
BYTE TapparellaGoto(char ixTapp, char position);
void UartDisplay(void);
BYTE MsgValido(void);
// ===================================================================================
ROM char menuScsPrompt[] =
{
    "\r\nSCSgate " VERSION
/*
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
*/
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
	SPBRGH2  = baud_value[0][BAUD_RATE];
    SPBRG2   = baud_value[1][BAUD_RATE];
    //-----------------------
    TXSTA2 = 0b00100110;   // tx enabled. hig speed
    if ((uart_echo == 3) || (uart_echo == 4))
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
#ifdef UART1_INTERRUPT
    INTCONbits.PEIE = 1;	// Enable peripheral interrupts
    PIE1bits.RC1IE = 1;
    IPR1bits.RC1IP = 0;
#endif
	PIR1bits.RC1IF = 0;
#endif
    // -----------------------------------------------------------------------------------

	
	
	
    // -----------------------------------------------------------------------------------
    RCONbits.IPEN = 1;      // interrupt priority enabled

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

//	putrsUSBwait("\r\nComparator Vi: ");
//	itoa((int)VrValue, RS_Out_Buffer);
//	putsUSBwait(RS_Out_Buffer);

	if ((VrValue > 20) && (VrValue < 28))	// resistenza 1.5k collegata a +3.3V
	{
    if (VrValue) VrValue--;                      // abbasso la tensione di riferimento di 0,50 Volts // se alimentazione 3.3V
    if (VrValue) VrValue--;                      // abbasso la tensione di riferimento di 0,60 Volts // se alimentazione 3.3V
    if (VrValue) VrValue--;                      // abbasso la tensione di riferimento di 0,70 Volts // se alimentazione 3.3V
	}

    if (VrValue) VrValue--;                      // abbasso la tensione di riferimento di 0,10 Volts // se alimentazione 3.3V
    if (VrValue) VrValue--;                      // abbasso la tensione di riferimento di 0,20 Volts // se alimentazione 3.3V
    if (VrValue) VrValue--;                      // abbasso la tensione di riferimento di 0,30 Volts // se alimentazione 3.3V
    if (VrValue) VrValue--;                      // abbasso la tensione di riferimento di 0,40 Volts // se alimentazione 3.3V : 1,5 x 13 = 3,5V

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
	BYTE m;
        opt.opzioneModo   = 'X';        //    [A|X] : modo ascii | hex"
        opt.opzioneFiltro = 0;          //    [0|1|2|3|4] : filtro"
		opt.EfilterByte_A = 0;			// 4.2  gestisce il byte filtering A
		opt.EfilterValue_A = 0;			// 4.2  gestisce il byte filtering A
		opt.EfilterByte_B = 0;			// 4.4  gestisce il byte filtering B
		opt.EfilterValue_B = 0;			// 4.4  gestisce il byte filtering B
		opt.Vref = 17;
		opt.abbrevia.Val = '0';
		opt.stream_timeout = 191;
		opt.eeVersion = EEPROM_VER;
        opt.tapparelle_pct = 0;
		Write_config(0); // scrive davvero
		DelayMs(10);

		m = 0;
		while (m<MAXTAPP)
		{
			devicetappa[m].device = 0;
			devicetappa[m].maxposition = 0;
			tapparella[m].direction.Val = 0;	//      [] direzione (0=ferma  9=giu  8=su )
			tapparella[m].position = 0;		//      [] posizione attuale (in unità da 0,1 secondi)
			tapparella[m].request = 0xFFFF;	//      [] posizione richiesta tapparella (in unità da 0,1 secondi) - nulla = 0xFFFF
			tapparella[m].timeout = 0;		//      [] timeout attuale (in unita da 0,1 secondi)
			m++;
		}
		maxtapp = 0;
		Write_tapparelle();

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
//	putcUSBwait('s'); ////////////////////////////// test
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
// * answer telegram from scs to uart                                           */
// ******************************************************************************/
void AnswerMsgInternal(void)
{
BYTE s,n,len;

    len = scsMessageRx[rBufferIdxR][0];	// 7
    if (len > 15) len = 15;
	s = 2;			// start ptr
	len -= 3;	        // length

	putcUSBwait(0xF0+len+1);	// internal answer, + length + 1
	putcUSBwait('y');	// stamp

	n = 0;
	while (n < len)
    {
        putcUSBwait(scsMessageRx[rBufferIdxR][s++]);
		n++;
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
	WORD w;
    WORD_VAL wv;

    choice0 = getUSBnowait();
//  if (choice0 != 0)
    if (uartRc != 0)            // messaggio da ESP8266 <=============================
    {
#ifdef UART1_BUFFER
		if (uartTrace)
			UartDisplay();
#endif
        switch (sm_command)
        {
            case SM_WAIT_HEADER:
                 if (choice0 == '@')  
				 {
					 sm_command = SM_WAIT_COMMAND;
					 sm_modo = SM_MODO_UDP; 
				 }
				 else
                 if (choice0 == '§')  
				 {
					 sm_command = SM_WAIT_COMMAND;
					 sm_modo = SM_MODO_INTERNO; 
				 }
				 else
                 if (choice0 == 0x11) 
				 {
			 		 INTCONbits.GIEH     = 0;	// non va bene - chiude anche interrupt UART
					 INTCONbits.GIEL     = 0;	// non va bene - chiude anche interrupt UART
					 REPROmainB(0x10, 0x12); // firmware update
				 }
				 else
                 if (choice0 == 0x12) 
					 putcUSBwait('k');       // dummy firmware update
				 else
                 if ((choice0 == 'h') 
                 &&  (opt.opzioneModo == 'A'))
						putrsUSBwait(menuScsPrompt);
/*
				 // --------------------- PROVVISORIO TEST --------------------------------------
				 else
                 if (choice0 == 't')
				 {
				    if (uartTrace == 0)
					{
						uartTrace = 1;
						UartDisplay();
					}
					else
						uartTrace = 0;
				 }
				 */
                 break;

            case SM_WAIT_COMMAND:
                 switch(choice0)
                 {
                     case '@':
                          break;
                     case '§':
                          break;
                     case 0x11:      // firmware update
	 					  INTCONbits.GIEH     = 0;	// non va bene - chiude anche interrupt UART
						  INTCONbits.GIEL     = 0;	// non va bene - chiude anche interrupt UART
                          REPROmainB(0x10, 0x12);
                          break;
                     case 0x12:      // dummy firmware update
                          putcUSBwait('k');
                          break;
                     case 0x15:      // per evitare memo in eeprom e answer
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
						  putcUSBwait(uart_echo + '0');
						  if (uart_echo > 4) uart_echo = 0;
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
						  writeLength = 4;
						  dataLength = 0;
						  dataByte.data[dataLength++] = 0xA8;
						  check = 0;
						  sm_command = SM_WAIT_WRITE_DATA_BREVE;
                          break;
                     case 'u':      // comando tapparelle a %  @u<address><pct>
						  sm_command = SM_WAIT_WRITE_TAPP_ADDR;
	                      dataTwin = 0;
                          break;
                     case 'U':      // setup tapparelle a %  @u<cmd>   0=no   1=raccogli i dati   2=online   9=ripulisci
						  sm_command = SM_WAIT_WRITE_TAPP_SETUP;
                          break;
                     case 'W':      // write stream
                          sm_command = SM_WAIT_WRITE_LENGTH;
                          break;
                     case 'w':      // write single command
                          sm_command = SM_WAIT_WRITE_CMD;
                          break;
                     case 't':      // write query command
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

                     case 'T':       // test mode  @TS   @TK
                          if  (opt.opzioneModo == 'A')
                          {
							  m = getUSBwait();
							  if (m == 'K')
							  {
								// spike test
								  putrsUSBwait("\r\nSPIKE test...");
								  dataByte.data[0] = 0x9A;
								  dataByte.data[1] = 0xBC;
								  dataByte.data[2] = 0xDE;
								  dataByte.data[3] = 0xF0;
								  dataByte.data[4] = 0x12;
								  dataByte.data[5] = 0x34;
								  dataByte.data[6] = 0xBB;
								  dataByte.data[7] = 0xCD;
								  queueWrite(8);
								  putrsUSBwait("\r\nSPIKE test requested");
							  }
							  else
							  if (m == 'S')
							  {
								  sm_stato   = SM_TEST_MODE;
								  dataByte.data[0] = 0x9A;
								  dataByte.data[1] = 0xBC;
								  dataByte.data[2] = 0xDE;
								  dataByte.data[3] = 0xF0;
								  dataByte.data[4] = 0x12;
								  dataByte.data[5] = 0x34;
								  dataByte.data[6] = 0x56;
//								  dataByte.data[6] = dataByte.data[1] ^ dataByte.data[2] ^ dataByte.data[3] ^ dataByte.data[4] ^ dataByte.data[5];
								  dataByte.data[7] = 0x78;
								  rBufferIdxW = 0;
								  rBufferIdxR = 0;
								  queueWrite(8);
								  putrsUSBwait("\r\nTesting: r");
								  putcUSBwait('.');
								  testCounter = 200;
							  }
							  else
							  if (m == 'W')
							  {
								  sm_stato   = SM_TEST_W_MODE;
								  rBufferIdxW = 0;
								  rBufferIdxR = 0;

								 test_destin = 0xFF;
								 test_source = 0x80;
								 test_command = 0x55;

								 dataByte.TopByte = 0xA8;
								 dataByte.DestinationAddress = test_destin;
								 dataByte.SourceAddress = test_source;
								 dataByte.CommandType = CMDTYPE_SET;	// 18.4A
								 dataByte.CommandValue = test_command;
								 dataByte.CheckByte = dataByte.data[1]^dataByte.data[2]^dataByte.data[3]^dataByte.data[4];
								 dataByte.BottomByte = 0xA3;
								 queueWrite(7);

								  putrsUSBwait("\r\nTesting: r");
								  putcUSBwait('.');
								  testCounter = 200;
								  testPhase = 0;
							  }
							  else
								  putcUSBwait('E');
						  }
						  else
							  putcUSBwait('E');

						  sm_command = SM_WAIT_HEADER;
                          break;
                     case 'S':       // byte stream timeout 
						  stream_timeout = (BYTE) getUSBvalue();
						  opt.stream_timeout = stream_timeout;
						  Write_config(ee_avoid_memo);
						  if (ee_avoid_answer == 0)
								putcUSBwait('k');
						  sm_command = SM_WAIT_HEADER;
                          break;

// ---------------------------- opzioni di test - valide solo in ascii mode-----------
                     case 'h':
                          if (opt.opzioneModo == 'A')
                          {
                              putrsUSBwait(menuScsPrompt);
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

							  putrsUSBwait("\r\n---> spikes counter: ");
                              itoa((int)uSpikes,RS_Out_Buffer);
                              putsUSBwait(RS_Out_Buffer);

							// 4.2  gestisce il byte filtering ----------------------------------------------------
#if (defined(USE_UART1) && defined(USE_UART2))

// echo=0 : l'output di knxgate viene inviato all'uart che ha fornito l'ultimo input in ordine di tempo
// echo=1 : l'output di knxgate viene inviato ad entrambe le uart indipendentemente dall'input
// echo=2 : l'output di knxgate viene inviato ad entrambe le uart indipendentemente dall'input
//          l'input di uart2    viene inviato solo in output a uart1 ma non a knxgate
// echo=3 : l'output di knxgate viene inviato ad entrambe le uart indipendentemente dall'input
//          l'input di uart1    viene inviato anche in output a uart2
//          l'input di uart2    viene ignorato
// echo=4 : l'output di knxgate viene inviato solo a UART1
//          l'input di uart1    viene inviato anche in output a uart2
//          l'input di uart2    viene ignorato

							  if (uart_echo == 0) putrsUSBwait("\r\nuart echo OFF");
							  if (uart_echo == 1) putrsUSBwait("\r\nuart echo 1 ON");
							  if (uart_echo == 2) putrsUSBwait("\r\nuart gate USB master");
							  if (uart_echo == 3) putrsUSBwait("\r\nuart gate ESP master");
							  if (uart_echo == 4) putrsUSBwait("\r\nuart gate USB reply ESP");
#endif

							  putrsUSBwait("\r\n---> write error: ");
                              itoa(errorCollision,RS_Out_Buffer);
                              putsUSBwait(RS_Out_Buffer);

							  putrsUSBwait("\r\n---> read  error: ");
                              itoa(errorChecksum,RS_Out_Buffer);
                              putsUSBwait(RS_Out_Buffer);

							  putrsUSBwait("\r\n---> uart   FERR: ");
                              itoa(uartFERR,RS_Out_Buffer);
                              putsUSBwait(RS_Out_Buffer);

							  putrsUSBwait("\r\n---> uart   OERR: ");
                              itoa(uartOERR,RS_Out_Buffer);
                              putsUSBwait(RS_Out_Buffer);
//							  uartFERR = 0;
//							  uartOERR = 0;

							  putrsUSBwait("\r\nGestione tapparelle %: ");
							  switch (opt.tapparelle_pct)
							  {
							  case 0:
								putrsUSBwait("NO");
								break;
							  case 1:
								putrsUSBwait("scanning");
								break;
							  case 3:
								  putrsUSBwait("SI & publ: ");
	                              itoa(maxtapp,RS_Out_Buffer);
		                          putsUSBwait(RS_Out_Buffer);
								break;
							  case 4:
								  putrsUSBwait("SI npub: ");
	                              itoa(maxtapp,RS_Out_Buffer);
		                          putsUSBwait(RS_Out_Buffer);
								break;
							  }
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
							  CVRCONbits.CVR = opt.Vref;
                              putcUSBwait('k');
                              putrsUSBwait("\r\nInternal Vref: ");
                              itoa((int)opt.Vref,RS_Out_Buffer);
                              putsUSBwait(RS_Out_Buffer);
                              putrsUSBwait("\r\n");
							  Write_config(0);
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
								  Write_config(0);
                              }
                              if (x == '+')
                              {
                                  opt.Vref++;
								  Write_config(0);
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
						 Write_config(ee_avoid_memo);
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
						 Write_config(ee_avoid_memo);
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
						 Write_config(ee_avoid_memo);
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
						 Write_config(ee_avoid_memo);
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
					 Write_config(ee_avoid_memo);
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
						 Write_config(ee_avoid_memo);
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
					 Write_config(ee_avoid_memo);
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
				  if ((opt.opzioneModo == 'X') || (sm_modo == SM_MODO_INTERNO))
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

				 //   '0':	close % mode
				 //   '1':  data collection
				 //   '2':  consolida e apre (se sono state ABBASSATE e alzate tapparelle
				 //   '3':  open % mode
				 //   '4':  open without publish position
				 //   '5':  lista ascii
				 //   '6':  dati singola tapparella hex mode 
				 //   '7':  ri-pubblica tutte le posizioni
				 //   '8':  inserisce o modifica tapparella (HEX)
				 //   '9':  pulisce tutta la tabella

			case SM_WAIT_WRITE_TAPP_SETUP:	// setup tapparelle a %  @u<cmd>   0=no   1=raccogli i dati   2=online   9=ripulisci
                 switch (choice0)
                 {
				 case '2':  // consolida e apre (se sono state ABBASSATE e alzate tapparelle
					 m = 0;
					 n = 0;
					 while (m < maxtapp)
					 {
						if ((devicetappa[m].device > 0) && (devicetappa[m].device < DEV_NR))
						{
							if (devicetappa[m].maxposition < tapparella[m].position)
								devicetappa[m].maxposition = tapparella[m].position;
							didx = devicetappa[m].device;
							devc[didx] = 9;
							if (devicetappa[m].maxposition > 0)
								n++;
                            tapparella[m].direction.Val = 0x80;
						}
						m++;
					 }

					 Write_tapparelle();
					 if (n == 0)
						choice0 = '0'; // nessuna tapparella
					 else
						choice0 = '3'; // tapparelle pct trovate

				 case '0':	// close
				 case '1':  // data collection
				 case '3':  // open
				 case '4':  // open without publish position
                     opt.tapparelle_pct = choice0 - '0';
					 Write_config(0); // scrive sempre
					 if (ee_avoid_answer == 0)
							putcUSBwait('k');
					break;

				 case '5':  // lista ascii
     				if (opt.opzioneModo == 'A')
					{
						 putrsUSBwait("\r\n\r\nTapparelle: ");
						 m = 0;
						 while (m < maxtapp)
						 {
							if (devicetappa[m].device)
							{
								putrsUSBwait("\r\n[");
								puthexUSBwait(devicetappa[m].device);
								putrsUSBwait("] max:");
								itoa(devicetappa[m].maxposition,RS_Out_Buffer);
								putsUSBwait(RS_Out_Buffer);	 
								putrsUSBwait(" now:");
								itoa(tapparella[m].position,RS_Out_Buffer);
								putsUSBwait(RS_Out_Buffer);	 
								putrsUSBwait(" dir:");
								puthexUSBwait(tapparella[m].direction.Val);	 
								putrsUSBwait(" req:");
								itoa(tapparella[m].request,RS_Out_Buffer);
								putsUSBwait(RS_Out_Buffer);	 
								putrsUSBwait(" out:");
								itoa(tapparella[m].timeout,RS_Out_Buffer);
								putsUSBwait(RS_Out_Buffer);	 
							}
							DelayMs(10);
							m++;
						 }
						 putrsUSBwait("\r\n");
					}
					break;

				 case '6':  // dati singola tapparella hex
     				if (opt.opzioneModo == 'X')
					{
						n = getUSBwait();
						m = 0;
						b = 0;
						while (m < maxtapp)
						{
							if (devicetappa[m].device == n)
							{
								putcUSBwait('[');

								putcUSBwait(devicetappa[m].amaxposition.byte.HB);
								putcUSBwait(devicetappa[m].amaxposition.byte.LB);

								putcUSBwait(tapparella[m].aposition.byte.HB);
								putcUSBwait(tapparella[m].aposition.byte.LB);

								putcUSBwait(tapparella[m].direction.Val);

								putcUSBwait(tapparella[m].arequest.byte.HB);
								putcUSBwait(tapparella[m].arequest.byte.LB);

								putcUSBwait(tapparella[m].timeout);
								m = maxtapp;
								b = 1;
							}
							m++;
						}
					 // non esiste
						if (b == 0)
						{
							putcUSBwait('[');
							for (m=0; m<8; m++)
								putcUSBwait(0x00);
						}
					}
					else
     				if (opt.opzioneModo == 'A')
					{
						putrsUSBwait("\r\nSetup tapparella ");
						putrsUSBwait("\r\nIndirizzo scs: ");
						n = getUSBhex();
						m = 0;
						b = 0;
						while (m < maxtapp)
						{
							if (devicetappa[m].device == n)
							{
								putrsUSBwait(" max[");
								itoa(devicetappa[m].maxposition,RS_Out_Buffer);
								putsUSBwait(RS_Out_Buffer);	 
								putrsUSBwait("] : ");
								w = getUSBvalue();
								if (w)
								{
									devicetappa[m].maxposition = w;
									Write_tapparelle();
								}
								putrsUSBwait("\r\nok ");
								m = maxtapp;
								b = 1;
							}
							m++;
						}
					 // non esiste
						if (b == 0)
						{
							putrsUSBwait("\r\nNuova - max: ");
							w = getUSBvalue();
							if (w)
							{
								maxtapp++;
								devicetappa[m].device = n;
								devicetappa[m].maxposition = w;
								Write_tapparelle();
							}
							putrsUSBwait("\r\nok ");
						}
					}
					break;

				 case '7':  // ri-pubblica tutte le posizioni
					 m = 0;
					 while (m < maxtapp)
					 {
                        tapparella[m].direction.bits.b7 = 0;
						m++;
					 }
				 	 break;

				 case '8':	// update tab !!!  - '8' <devx> <type> <maxpH> <maxpL>
     				 if (opt.opzioneModo == 'X')
					 {
						n = getUSBwait();   // device id
						m = getUSBwait();   // device type
						wv.byte.HB = getUSBwait();   // max position H (se devtype 9)
						wv.byte.LB = getUSBwait();   // max position L (se devtype 9)

						if ((n > 0) && (n < DEV_NR) && (m > 0) && (m < 32))
						{
						   devc[n] = m;
                           if (m == 9)
						   {
							b = 0;
							while (b < maxtapp)
							{
								if (devicetappa[b].device == n)
								{
									devicetappa[b].maxposition = wv.Val;
									Write_tapparelle();
									break;
								}
								b++;
							}
                            if (b == maxtapp)
							{
								maxtapp++;
								devicetappa[b].device = n;
								devicetappa[b].maxposition = wv.Val;
								Write_tapparelle();
							}
						   } // m == 9
						}
					 }
				 	 break;

				 case '9':	// clear tab !!!
					 m = 0;
					 while (m<MAXTAPP)
					 {
						devicetappa[m].device = 0;
						devicetappa[m].maxposition = 0;
						tapparella[m].direction.Val = 0;//      [] direzione (0=ferma  9=giu  8=su )
						tapparella[m].position = 0;		//      [] posizione attuale (in unità da 0,1 secondi)
						tapparella[m].request = 0xFFFF;	//      [] posizione richiesta tapparella (in unità da 0,1 secondi) - nulla = 0xFFFF
						tapparella[m].timeout = 0;		//      [] timeout attuale (in unita da 0,1 secondi)
						m++;
					 }
					 maxtapp = 0;
					 Write_tapparelle();
 	 				 opt.tapparelle_pct = 0;
					 Write_config(0); // scrive sempre
					 if (ee_avoid_answer == 0)
							putcUSBwait('k');
					 break;

				 default:
                     putcUSBwait('E');
					 break;
				 }
                 sm_command = SM_WAIT_HEADER;
                 break;


			case SM_WAIT_WRITE_TAPP_ADDR:	// comando tapparelle a %  @u<address><pct>
                 if ((opt.opzioneModo == 'X') || (sm_modo == SM_MODO_INTERNO))
                 {
                     dataByte.DestinationAddress = choice0;
                     sm_command = SM_WAIT_WRITE_TAPP_POS;
                     break;
                 }
                 if ((choice0 >= '0') && (choice0 <= '9'))
                     thisHalf = choice0 - '0';
                 else
                 if ((choice0 >= 'A') && (choice0 <= 'F'))
                     thisHalf = choice0 - 'A' + 10;
                 else
                 if ((choice0 >= 'a') && (choice0 <= 'f'))
                     thisHalf = choice0 - 'a' + 10;
                 else
                 {
                     putcUSBwait('E');
                     sm_command = SM_WAIT_HEADER;
                     break;
                 }
                 if (dataTwin)
                 {
                     thisByte += thisHalf;
                     dataByte.DestinationAddress = thisByte;
                     dataTwin = 0;
                     sm_command = SM_WAIT_WRITE_TAPP_POS;
                 }
                 else
                 {
                     thisByte = thisHalf;
                     thisByte<<=4;
                     dataTwin = 1;
                 }
				 break;

			case SM_WAIT_WRITE_TAPP_POS:	// comando tapparelle a %  @u<address><pct>
                 if ((opt.opzioneModo == 'X') || (sm_modo == SM_MODO_INTERNO))
                 {
					 if (0 == TapparellaGoto(dataByte.DestinationAddress, choice0) )
					 {
						 if (ee_avoid_answer == 0)
							putcUSBwait('k');
					 }
					 else
						 if (ee_avoid_answer == 0)
		                    putcUSBwait('E');
                     sm_command = SM_WAIT_HEADER;
                     break;
                 }
                 if ((choice0 >= '0') && (choice0 <= '9'))
                     thisHalf = choice0 - '0';
                 else
                 if ((choice0 >= 'A') && (choice0 <= 'F'))
                     thisHalf = choice0 - 'A' + 10;
                 else
                 if ((choice0 >= 'a') && (choice0 <= 'f'))
                     thisHalf = choice0 - 'a' + 10;
                 else
                 {
                     putcUSBwait('E');
                     sm_command = SM_WAIT_HEADER;
                     break;
                 }
                 if (dataTwin)
                 {
                     thisByte += thisHalf;
					 if (0 == TapparellaGoto(dataByte.DestinationAddress, thisByte))
					 {
						 if (ee_avoid_answer == 0)
							putcUSBwait('k');
					 }
					 else
						 if (ee_avoid_answer == 0)
		                    putcUSBwait('E');
                     dataTwin = 0;
                     sm_command = SM_WAIT_HEADER;
                 }
                 else
                 {
                     thisByte = thisHalf;
                     thisByte<<=4;
                     dataTwin = 1;
                 }
				 break;

            case SM_WAIT_WRITE_DATA:		//    "@W[1-F][data] : write"
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
//BYTE devc[DEV_NR];	// 0xFF:empty    01:switch    03:dimmer    08:tapparella
//BYTE didx;
                 if ((opt.opzioneModo == 'X') || (sm_modo == SM_MODO_INTERNO))
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
					while (didx < DEV_NR)
					{
						devc[didx++] = 0xFF;
					}
// predisporre eventuali tapparelle
					m = 0;
					while (m < maxtapp)
					{
						if ((devicetappa[m].device > 0) && (devicetappa[m].device < DEV_NR))
						{
							didx = devicetappa[m].device;
							devc[didx] = 9;
						}
						m++;
					}
					didx = 0;
				 }
				 else
				 {
					 while ((devc[didx] == 0xFF) && (didx < DEV_NR)) // sizeof(devc))
					 {
						 didx++;
					 }
				 }

				if (sm_modo == SM_MODO_INTERNO)
				{
					putcUSBwait(0xF3);	// internal answer, length 3
	                putcUSBwait('D');
					putcUSBwait(didx);
					if (didx < DEV_NR)
	                     putcUSBwait(devc[didx]);
					else
	                     putcUSBwait(0xFF);
				}
				else
				if (opt.opzioneModo == 'X')
				{
					putcUSBwait('D');
					putcUSBwait(didx);
					if (didx < DEV_NR)
	                     putcUSBwait(devc[didx]);
					else
	                     putcUSBwait(0xFF);
				}
				else
				{
					putcUSBwait('D');
					puthexUSBwait(didx);
					if (didx < DEV_NR)
						puthexUSBwait(devc[didx]);
					 else
						puthexUSBwait(0xFF);
				}

				sm_command = SM_WAIT_HEADER;
                dataTwin = 0;
                break;


            case SM_WAIT_WRITE_QUERY:  //  "@t[destin] write query"
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
// ===================================================================================
void TransferFunction(void)
{
DWORD percent;
BYTE  try;

#ifdef	UART1_CALL
	inUART();
#endif

	if (PIR1bits.TMR1IF) //   32mSec
    {
// -----------------------------------------------------------------------------------------------------------
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
// -----------------------------------------------------------------------------------------------------------


		TappaPublishTicks++;
		if ((opt.tapparelle_pct == 3) && (TappaPublishTicks > 21) && (sm_stato ==  SM_LOG_WAIT))   // ogni n x 32mSec
		{
	  		TappaPublishTicks = 0;

			try=0;
			while (try < maxtapp)
			{
				if (tapparella[ixPublish].direction.bits.b7 == 0)
					try = maxtapp;
				else
				{
					if (++ixPublish >= maxtapp)	ixPublish = 0;
					try++;
				}
			}

			if (tapparella[ixPublish].direction.bits.b7 == 0)
			{
				percent = tapparella[ixPublish].position;
				percent *= 100;
				percent /= devicetappa[ixPublish].maxposition;
				if (sm_modo == SM_MODO_INTERNO)
				{
					putcUSBwait(0xF3);	// internal answer, length 3
					putcUSBwait('u');	// stamp
					putcUSBwait(devicetappa[ixPublish].device);	// device
					putcUSBwait((BYTE)percent);					// percent position
				}
				else
				if (opt.opzioneModo == 'A')
				{
//					putrsUSBwait("\r\nu");	// length
//					puthexUSBwait(devicetappa[ixPublish].device);	// device
//					puthexUSBwait((BYTE)percent);					// percent position
				}
				else
				{
					putcUSBwait('u');	// stamp
					putcUSBwait(devicetappa[ixPublish].device);	// device
					putcUSBwait((BYTE)percent);					// percent position
				}
				tapparella[ixPublish].direction.bits.b7 = 1;
			}
			if (++ixPublish >= maxtapp)	ixPublish = 0;
		}



		TappaMoveTicks++;
		if ((opt.tapparelle_pct) && (TappaMoveTicks > 2))   // ogni 100mSec
		{
	  		TappaMoveTicks = 0;
			ixTapp = 0;
			while (ixTapp < maxtapp)
			{
				if (tapparella[ixTapp].direction.half.LS == DOWN)  // down
				{
					if (tapparella[ixTapp].position > 0)
					{
						tapparella[ixTapp].position--;
						tapparella[ixTapp].direction.bits.b7 = 0;
					}
					else
						tapparella[ixTapp].timeout++;
					if ((tapparella[ixTapp].request != 0xFFFF) && (tapparella[ixTapp].position <= tapparella[ixTapp].request))
					{
						// fermare la tapparella 
						if (opt.tapparelle_pct > 2) TapparellaAction(devicetappa[ixTapp].device, STOP);
						tapparella[ixTapp].request = 0xFFFF;
#ifdef IMMEDIATE_ANSWER        // ricava lo stato anche dal comando (non solo dalla risposta di stato)
						tapparella[ixTapp].direction.half.LS = 0;
						tapparella[ixTapp].timeout = 0;
#endif
					}
				}
				else
				if (tapparella[ixTapp].direction.half.LS == UP)  // up
				{
					if ((tapparella[ixTapp].position < devicetappa[ixTapp].maxposition) || (opt.tapparelle_pct == 1))
					{
						tapparella[ixTapp].position++;
						tapparella[ixTapp].direction.bits.b7 = 0;
					}
					else
						tapparella[ixTapp].timeout++;
					if ((tapparella[ixTapp].request != 0xFFFF) && (tapparella[ixTapp].position >= tapparella[ixTapp].request))
					{
						// fermare la tapparella 
						if (opt.tapparelle_pct > 2) TapparellaAction(devicetappa[ixTapp].device, STOP);
						tapparella[ixTapp].request = 0xFFFF;
#ifdef IMMEDIATE_ANSWER        // ricava lo stato anche dal comando (non solo dalla risposta di stato)
						tapparella[ixTapp].direction.half.LS = 0;
						tapparella[ixTapp].timeout = 0;
#endif
					}
				}
				else
				{
					tapparella[ixTapp].timeout = 0;
					tapparella[ixTapp].request = 0xFFFF;
					tapparella[ixTapp].direction.half.LS = 0;
				}

				if (tapparella[ixTapp].timeout > 50)	// timeout 5 secondi
				{
					tapparella[ixTapp].timeout = 0;
					tapparella[ixTapp].request = 0xFFFF;
					tapparella[ixTapp].direction.half.LS = 0;
					TapparellaAction(devicetappa[ixTapp].device, STOP);  /// ??? sarà meglio ???
				}
				ixTapp++;
			}
		}
// -----------------------------------------------------------------------------------------------------------
    }
}
// ===================================================================================
BYTE MsgValido(void)
{
BYTE f,len,m;
BYTE valido;
BYTE check = 0;

	len = scsMessageRx[rBufferIdxR][0];
	valido = 1;

	if  ((len == 1) && (scsMessageRx[rBufferIdxR][1] == 0xFF))
		  valido = 0;
	else
	if  ((len == 2) && (scsMessageRx[rBufferIdxR][1] == 0xFF)&& (scsMessageRx[rBufferIdxR][2] == 0xFF))
		  valido = 0;
	else
	if (len > 6)
	{
		f = 2;
		while (f < len)   // 07 A8 B8 00 12 01 cc A3 - 2-3-4-5-6
		{
			check ^= scsMessageRx[rBufferIdxR][f];
			f++;
		}
		if	(check ^= 0)
		{
			errorChecksum++;
			valido = 0;
		}
	}

	if (valido)
	{
		if (opt.opzioneFiltro & 0x04)                                  // esclude messaggi di stato
		{
			if  ((len > 2) && (scsMessageRx[rBufferIdxR][3] != 0))
			  valido = 0;
		}
		if (opt.opzioneFiltro & 0x02)       // esclude messaggi ack
		{
			if  ((len == 1) && (scsMessageRx[rBufferIdxR][1] == 0xA5))
			  valido = 0;
		}

		if ((opt.opzioneFiltro & 0x01)       // esclude messaggi doppi
		&&  (rBufferIdxR != rBufferIdxP)
		&&  (rBufferIdxP != 0xFF))
		{
			f = 1;
			for (m=0; m<=len;m++)
			{
				if (scsMessageRx[rBufferIdxR][m] != scsMessageRx[rBufferIdxP][m])   f = 0;
			}
			if (f)     valido = 0;
		}
	}
	return valido;
}
// ===================================================================================
// ===================================================================================





// ===================================================================================

// ******************************************************************************/
// * output analyze and mapping                                                 */
// ******************************************************************************/
void OutputMapping(void)
{
BYTE f,l,m;
BYTE increment;

#ifdef	UART1_CALL
	inUART();
#endif
	increment = 0;

    if  (rBufferIdxR != rBufferIdxW)  // qualcosa da leggere
	{
		if (MsgValido())   // messaggio dal BUS  SCS <=============================
		{
			rBufferIdxP = rBufferIdxR;

			if  (rBufferIdxR != rBufferIdxInternal)  // se non ancora trattato internamente
			{
				rBufferIdxInternal = rBufferIdxR;

// LL a8 55 00 12 08 xx a3
// -----------------------------aggiorna tabella dispositivi-------------------------------
				if ((scsMessageRx[rBufferIdxR][0] == 7) &&
					(scsMessageRx[rBufferIdxR][1] == 0xA8) &&
					(scsMessageRx[rBufferIdxR][4] == 0x12) &&
					(scsMessageRx[rBufferIdxR][7] == 0xA3) &&
					(sm_stato   !=  SM_TEST_MODE) &&
					(sm_stato   !=  SM_TEST_W_MODE))
				{
					if (scsMessageRx[rBufferIdxR][2] < 0xB0)
						didx = scsMessageRx[rBufferIdxR][2];
					else
						didx = scsMessageRx[rBufferIdxR][3];

					if ((didx > 0) && (didx < DEV_NR)) // (sizeof(devc)))
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
// -----------------------------aggiorna anche tabella tapparelle-----------------------------
						case 0x08:					// alza
							if (devc[didx] != 9) devc[didx] = 8;
							if (opt.tapparelle_pct)
							{
								ixTapp = 0;
								while (ixTapp < maxtapp)
								{
									if (devicetappa[ixTapp].device == didx)
									{
										if (tapparella[ixTapp].direction.half.LS == 0)
										{
//											tapparella[ixTapp].direction.half.LS = UP;
											tapparella[ixTapp].direction.Val = UP; // pubblicazione immediata
											tapparella[ixTapp].timeout = 0;
										}
										ixTapp = maxtapp;
									}
									else
										ixTapp++;
								}
							}
							break;

						case 0x09:					// abbassa
							if (devc[didx] != 9) devc[didx] = 8;
							f = 0;
							if (opt.tapparelle_pct)
							{
								ixTapp = 0;
								while (ixTapp < maxtapp)
								{
									if (devicetappa[ixTapp].device == didx)
									{
										if (tapparella[ixTapp].direction.half.LS == 0)
										{
											tapparella[ixTapp].direction.Val = DOWN; // pubblicazione immediata
//											tapparella[ixTapp].direction.half.LS = DOWN;
											tapparella[ixTapp].timeout = 0;
										}
										f = 1;
										ixTapp = maxtapp;
									}
									else
										ixTapp++;
								}
								if (( f == 0 ) && (opt.tapparelle_pct == 1) && (maxtapp < MAXTAPP)) // censimento tapparella?
								{
									devicetappa[maxtapp].device = didx;
									tapparella[maxtapp].direction.half.LS = DOWN;
									tapparella[maxtapp].timeout = 0;
									tapparella[maxtapp].position = 0;
									tapparella[maxtapp].request = 0xFFFF;
									maxtapp++;
								}
							}
							break;

						case 0x0A:                  // stop
							if (devc[didx] != 9) devc[didx] = 8;
							if (opt.tapparelle_pct)
							{
								ixTapp = 0;
								while (ixTapp < maxtapp)
								{
									if (devicetappa[ixTapp].device == didx)
									{
//										if (tapparella[ixTapp].direction.Val != 0)
										{
											tapparella[ixTapp].direction.half.LS = 0;
											tapparella[ixTapp].timeout = 0;
											tapparella[ixTapp].request = 0xFFFF;
										}
										ixTapp = maxtapp;
									}
									else
										ixTapp++;
								}
							}
							break;
// -------------------------------------------------------------------------------------------
						default:
							if ((scsMessageRx[rBufferIdxR][5] & 0x0F) == 0x0D)
								devc[didx] = 3;
							break;
						}
					}
				}
// -------------------------------------------------------------------------------------------


			}	// se non ancora trattato internamente






			if (sm_command ==  SM_READ_DEQUEUE)
			{
				AnswerMsg();
				increment = 1;
				sm_command = SM_WAIT_HEADER;
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

//BYTE devc[DEV_NR];	// 0xFF:empty    01:switch    03:dimmer    08:tapparella
//BYTE didx;



			if (sm_stato   ==  SM_READ_WAIT)
			{
				AnswerMsg();
				increment = 1;		// messaggio scodato
				sm_stato = SM_HOME;
			}

			if (sm_stato   ==  SM_LOG_WAIT)
			{
				if (sm_modo == SM_MODO_INTERNO)
					AnswerMsgInternal();
				else
				if (opt.opzioneModo == 'A')
					LogScsDisplay();
				else
					AnswerMsg();

				increment = 1;		// messaggio scodato
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
				increment = 1;		// messaggio scodato
			}
		}		// (MsgValido())

		else
		{
			increment = 1;		// messaggio scodato
		}

		if (sm_stato   ==  SM_TEST_W_MODE)
		{
			increment = 1;
			if (testPhase == 0)
			{
				if ((scsMessageRx[rBufferIdxR][0] != 1) ||
					(scsMessageRx[rBufferIdxR][1] != 0xA5))
				{
					putcUSBwait('N');
					if (opt.opzioneModo == 'A')
						LogScsDisplay();
					testCounter = 0;
					testPhase = 0;
					putrsUSBwait("\r\nTest FAILED!!!");
					sm_stato = SM_HOME;
				}
				else
				{
//					putcUSBwait('0'+testPhase);
					testPhase++;
				}
			}
			else
			{
				if ((scsMessageRx[rBufferIdxR][0] != 7) ||
					(scsMessageRx[rBufferIdxR][1] != 0xA8) ||
					(scsMessageRx[rBufferIdxR][3] != test_destin) ||
					(scsMessageRx[rBufferIdxR][4] != CMDTYPE_SET) ||
					(scsMessageRx[rBufferIdxR][5] != test_command) ||
//					(scsMessageRx[rBufferIdxR][6] != (scsMessageRx[rBufferIdxR][2] ^ scsMessageRx[rBufferIdxR][3] ^ scsMessageRx[rBufferIdxR][4] ^ scsMessageRx[rBufferIdxR][5])) ||
					(scsMessageRx[rBufferIdxR][7] != 0xA3))
				{
					putcUSBwait('N');
					if (opt.opzioneModo == 'A')
						LogScsDisplay();
					testCounter = 0;
					testPhase = 0;
					putrsUSBwait("\r\nTest FAILED!!!");
					sm_stato = SM_HOME;
				}
				else
				{
//					putcUSBwait('0'+testPhase);
					testPhase++;
				}

				if (testPhase > 3)
				{
					testCounter--;
					testPhase = 0;
					if (testCounter == 0)
					{
						sm_stato = SM_HOME;
						putrsUSBwait("\r\nTest O.K.!");
					}
					else
					{
						putcUSBwait('k');
						test_destin++;
						test_source--;
						test_command++;
						test_command++;

						dataByte.TopByte = 0xA8;
						dataByte.DestinationAddress = test_destin;
						dataByte.SourceAddress = test_source;
						dataByte.CommandType = CMDTYPE_SET;	// 18.4A
						dataByte.CommandValue = test_command;
						dataByte.CheckByte = dataByte.data[1]^dataByte.data[2]^dataByte.data[3]^dataByte.data[4];
						dataByte.BottomByte = 0xA3;
						queueWrite(7);
						putcUSBwait('.');
					}
				}
			}
		}
		else
		if (sm_stato   ==  SM_TEST_MODE)
		{
			f = 0;
            l = 1;
			while (l < scsMessageRx[rBufferIdxR][0])
			{
				if (scsMessageRx[rBufferIdxR][l] != dataByte.data[l-1])
					f = 1;
				l++;
			}
			if (f == 0)
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

			l = dataByte.data[0] & 0x0F;
			if (l < 4) l = 4;
			f = 0;
			while (f < l)  
			{
				dataByte.data[f]++;
				f ++;
				dataByte.data[f]--;
				f ++;
			}
			f = 1;
			m = 0;
			while (f < l-2)  // 8  1-2-3-4-5-c-7
			{
				m ^= dataByte.data[f];
				f ++;
			}
			dataByte.data[l-2] = m; // check byte
  		    queueWrite(l);

			putcUSBwait('.');
			testCounter--;
			if (testCounter == 0) 
			{
				sm_stato = SM_HOME;
				putrsUSBwait("\r\nTest O.K.!");
			}
			increment = 1;
		}  // (sm_stato   ==  SM_TEST_MODE)

		if (increment)
			IncrementBufferPtr();  // scodato

	}
	else
	{				// nessun messaggio da scodare
		if (sm_command ==  SM_READ_DEQUEUE)
		{
			AnswerMsg();
			sm_command = SM_WAIT_HEADER;
		}
	}
}

// ==========================get  USB via USART======================================
#ifdef UART1_BUFFER
// ----------------------------------------------------------------------
void getUART(void)
{
	char c;

    if (RCSTA1bits.OERR) {
        RCSTA1bits.CREN = 0;
        RCSTA1bits.CREN = 1;
		uartOERR++;
    }
	if (PIR1bits.RC1IF != 0)
    {
		PIR1bits.RC1IF = 0;
		uartMessage[uPtrW++] = RCREG1;
		if (uPtrW > 63)  uPtrW = 0;
		if (uMax < 64) uMax++;
	}
}
#endif
// ----------------------------------------------------------------------
char getUSBwait(void)
{
BYTE c;
#if defined(USE_UART2)
    if (uart_echo < 3)
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

#ifdef UART1_BUFFER
        while (uMax == 0)  ClrWdt();
		c = uartMessage[uPtrR++];
        if (uPtrR >= UART1_BUFFER) uPtrR = 0;
		uMax--;
		if (uState)	// uart error !!!
		{
			if (uState & 1) uartFERR++;;
			if (uState & 2) uartOERR++;;
			uState = 0;
		}
#else
        if (RCSTA1bits.OERR) {
            RCSTA1bits.CREN = 0;
            RCSTA1bits.CREN = 1;
			uartOERR++;
        }
        while (PIR1bits.RC1IF == 0)  ClrWdt();
        c = RCREG1;
        PIR1bits.RC1IF = 0;
#endif
        uartRc = 1;
        return c;
    }
#endif
    uartRc = 0;
    return 0;
}

// ===================================================================================
WORD getUSBvalue(void)
{
BYTE in;
WORD res;
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
// ===================================================================================
BYTE getUSBhex(void)
{
BYTE in, res;
     res = 0;
     in = getUSBwait();
	 if ((in >= '0') && (in <= '9'))
		 in -= '0';
	 else
	 if ((in >= 'A') && (in <= 'F'))
		 in -= ('A' - 10);
	 else
	 if ((in >= 'a') && (in <= 'f'))
		 in -= ('a' - 10);
	 else
		 in = 0;
	 res = in;
	 res <<= 4;
     in = getUSBwait();
	 if ((in >= '0') && (in <= '9'))
		 in -= '0';
	 else
	 if ((in >= 'A') && (in <= 'F'))
		 in -= ('A' - 10);
	 else
	 if ((in >= 'a') && (in <= 'f'))
		 in -= ('a' - 10);
	 else
		 in = 0;
     res |= in;
	 return res;
}

// ==========================read USB via USART======================================
char getUSBnowait(void)
{
BYTE c;
    c = 0;
#if defined(USE_UART2)
    if (uart_echo < 3)
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
//          l'input di uart2    viene inviato solo in output a uart1 ma non a knxgate
// echo=3 : l'output di knxgate viene inviato ad entrambe le uart indipendentemente dall'input
//          l'input di uart1    viene inviato anche in output a uart2
//          l'input di uart2    viene ignorato
// echo=4 : l'output di knxgate viene inviato solo a UART1
//          l'input di uart1    viene inviato anche in output a uart2
//          l'input di uart2    viene ignorato

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

#ifdef UART1_BUFFER
	if (uMax > 0)
	{
		c = uartMessage[uPtrR++];
        if (uPtrR >= UART1_BUFFER) uPtrR = 0;
		uMax--;
		if (uState)	// uart error !!!
		{
			if (uState & 1) uartFERR++;;
			if (uState & 2) uartOERR++;;
			uState = 0;
		}
#else
    if (RCSTA1bits.OERR) {
        RCSTA1bits.CREN = 0;
        RCSTA1bits.CREN = 1;
		uartOERR++;
    }
	if (PIR1bits.RC1IF != 0)
    {
		PIR1bits.RC1IF = 0;
		c = RCREG1;
#endif
		uartRc = 1;
		uart_in_use = 1;

#if defined(USE_UART2)
		if (uart_echo >= 3)
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
// echo=3 : l'output di knxgate viene inviato ad entrambe le uart indipendentemente dall'input
//          l'input di uart1    viene inviato anche in output a uart2
//          l'input di uart2    viene ignorato
// echo=4 : l'output di knxgate viene inviato solo a UART1
//          l'input di uart1    viene inviato anche in output a uart2
//          l'input di uart2    viene ignorato


#if defined(USE_UART2)
	if ((uart_in_use == 2) || (uart_echo & 0x03))
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
#ifdef	UART1_CALL
			inUART();
#endif
            while (TXSTA1bits.TRMT == 0); // wait if the buffer is full 
            TXREG1 = data; // transfer data word to TX reg 
        }
    }
#ifdef	UART1_CALL
	inUART();
#endif
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
	if ((uart_in_use == 2) || (uart_echo & 0x03))
    {
		ptr = data;
        while(*ptr != '\0')
        {
#ifdef	UART1_CALL
			inUART();
#endif
            while (TXSTA2bits.TRMT == 0); // wait if the buffer is full 
            TXREG2 = *ptr++; // transfer data word to TX reg 
#ifdef	UART1_CALL
			inUART();
#endif
        }
    }
#endif
#if defined(USE_UART1)
	if ((uart_in_use == 1) || (uart_echo))
    {
		ptr = data;
	    while(*ptr != '\0')
        {
#ifdef	UART1_CALL
			inUART();
#endif
            while (TXSTA1bits.TRMT == 0); // wait if the buffer is full 
            TXREG1 = *ptr++; // transfer data word to TX reg 
#ifdef	UART1_CALL
			inUART();
#endif
        }
    }
#endif
}


// ==========================write USB via USART=====================================
void putrsUSBwait(const ROM char *data)
{
const ROM	char * ptr;
#if defined(USE_UART2)
	if ((uart_in_use == 2) || (uart_echo & 0x03))
    {
		ptr = data;
        // transmit till NULL character is encountered 
        while(*ptr != '\0')
        {
#ifdef	UART1_CALL
			inUART();
#endif
            while (TXSTA2bits.TRMT == 0); // wait if the buffer is full 
            TXREG2 = *ptr++; // transfer data word to TX reg 
#ifdef	UART1_CALL
			inUART();
#endif
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
#ifdef	UART1_CALL
			inUART();
#endif
            while (TXSTA1bits.TRMT == 0); // wait if the buffer is full 
            TXREG1 = *ptr++; // transfer data word to TX reg 
#ifdef	UART1_CALL
			inUART();
#endif
        }
    }
#endif
}
// ===================================================================================

#ifdef UART1_BUFFER
void UartDisplay(void)
{
char i;
	i = 0;
	putrsUSBwait("\r\nbuffer: ");
	while (i < UART1_BUFFER)
	{
		puthexUSBwait(uartMessage[i]);
		putcUSBwait(' ');
		i++;
	}
	putrsUSBwait("\r\nuPtrW ");
	puthexUSBwait(uPtrW);

	putrsUSBwait("\r\nuPtrR ");
	puthexUSBwait(uPtrR);

	if (uMax > 0)
	{
		putrsUSBwait("\r\nuMax ");
		puthexUSBwait(uMax);
	}
}
#endif
// ===================================================================================


/******************************************************************************
 ******************************************************************************
 * Main Application code                                                      *
 ******************************************************************************
 ******************************************************************************/
void main(void)
{
 BYTE i;
 BYTE t;

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

	opt.Vref = 17;     // 3,3V                ogni step 3.3/32 = 0,10Volt - 2,00V

    DelayMs(100);
    ClrWdt();
	Read_eep_array(&opt, EE_CONFIG, (char) sizeof(opt));

    if (opt.eeVersion != EEPROM_VER)
    {
		eepromInit();
    }
	else
		Read_tapparelle();

	if (maxtapp > MAXTAPP) maxtapp = 0;
	if (opt.tapparelle_pct > 9) opt.tapparelle_pct = 0;

	if (opt.abbrevia.Val == 0xFF)	opt.abbrevia.Val = '0';
	filterByte_A = opt.EfilterByte_A;
	filterValue_A = opt.EfilterValue_A;
	filterByte_B = opt.EfilterByte_B;
	filterValue_B = opt.EfilterValue_B;
	stream_timeout	= opt.stream_timeout;

	didx = 0;
	while (didx < DEV_NR)
	{
		devc[didx++] = 0xFF;
	}


	i = 0;
	while (i<MAXTAPP)
	{
		tapparella[i].direction.Val = 0;	//      [] direzione (0=ferma  9=giu  8=su )
		tapparella[i].position = 0;		//      [] posizione attuale (in unità da 0,1 secondi)
		tapparella[i].request = 0xFFFF;	//      [] posizione richiesta tapparella (in unità da 0,1 secondi) - nulla = 0xFFFF
		tapparella[i].timeout = 0;		//      [] timeout attuale (in unita da 0,1 secondi)
		i++;
	}

	i = 0;
	while (i<maxtapp)
	{
		t = devicetappa[i].device;
		if (t < DEV_NR)
			devc[t] = 0x09;
		i++;
	}

	i = 0;
#ifdef UART1_BUFFER
	while (i < UART1_BUFFER)
	{
		uartMessage[i++] = 0;
	}
#endif


	ClrWdt();

    InitializeBoard();    // Initialize hardware

    TickInit();                // Initialize timers
    DelayMs(150);

    Ticks = 0;

	scsInit();
	opt.stream_timeout	= stream_timeout;

    AppInitialize();

    while(1)
    {
// ===================================================================================
#ifdef	UART1_CALL
		inUART();
#endif
	
        ClrWdt();
        InputMapping();      // read input ports
        TransferFunction();  // from inputs to outputs
        OutputMapping();     // setting output ports
// ===================================================================================
    }
}
// ===================================================================================











// ===========================comando @u di posizionamento da esp8266=============================
BYTE TapparellaGoto(char device, char position) // return 0=ok
{
	DWORD required;
	WORD  movement;
	BYTE  action;
    BYTE  newdirection = 0;

	if (opt.tapparelle_pct < 3) 
		return 1;

	if (position > 100) position = 100;	// posizione percentuale richiesta
	required = position;
	ixTapp = 0;
	while (ixTapp < maxtapp)
	{
		if (devicetappa[ixTapp].device == device)
		{
			required *=	devicetappa[ixTapp].maxposition;
			required /=	100;				// posizione assoluta richiesta
			if (tapparella[ixTapp].position > (WORD)required)
			{
				movement = tapparella[ixTapp].position - (WORD)required;
//				if (movement > 9)
					newdirection = DOWN;
//				else
//					return 0; // movimento non necessario
			}
			else
			{
				movement = (WORD)required - tapparella[ixTapp].position;
				if (movement > 0)
					newdirection = UP;
				else
				{
					tapparella[ixTapp].request = 0xFFFF;
					tapparella[ixTapp].direction.Val = 0;
					TapparellaAction(device, STOP);
					return 0;
				}
			}

			if (tapparella[ixTapp].direction.half.LS == 0)  // tapparella ferma
			{
				tapparella[ixTapp].request = (WORD) required;
				tapparella[ixTapp].direction.half.LS = newdirection;
				TapparellaAction(device, newdirection);
				return 0;
			}
			else
			{		// gia in movimento <-----------------------------
				if (tapparella[ixTapp].direction.half.LS == newdirection) // nella giusta direzione
				{
					tapparella[ixTapp].request = (WORD) required;
					return 0;
				}
				else
				{ // gia in movimento  ---- nella direzione opposta
					TapparellaAction(device, STOP);
//					DelayMs(80);
//					tapparella[ixTapp].request = (WORD) required;
//					tapparella[ixTapp].direction.half.LS = newdirection;
//					TapparellaAction(device, newdirection);
					return 0;
				}
			}
		}
		ixTapp++;
	}
	return 1;
}
// ===================================================================================
void TapparellaAction(char ixTapp, char action)
{
     dataByte.TopByte = 0xA8;
     dataByte.DestinationAddress = ixTapp;
     dataByte.SourceAddress = 0x00;
     dataByte.CommandType = CMDTYPE_SET;
	 dataByte.CommandValue = action;
     dataByte.CheckByte = dataByte.data[1]^dataByte.data[2]^dataByte.data[3]^dataByte.data[4];
     dataByte.BottomByte = 0xA3;
     queueWrite(7);
}
// ===================================================================================
void queueWrite(char dataLength)
{
BYTE clear;
BYTE msgLen;
int n;
BYTE retry;
	retry = 0;
	do
	{
		n = 0;
//		DelayMs(3);

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
		while ((clear) && (n < 300))
		{
			DelayMs(1);
			if (optionR.R_READING)// || (INPUT_SCS == 1)) 	// lettura in corso !
			{
				ClrWdt();
				clear = 4;
				n++;
			}
			else
				clear--;
		}

		INTCONbits.GIEH     = 0;	// non va bene - chiude anche interrupt UART
		INTCONbits.GIEL     = 0;	// non va bene - chiude anche interrupt UART
//		PIE4bits.CMP1IE  = 0;    // close interrupt
//		INTCONbits.TMR0IE = 0;   // close INTERRUPT     
//		PIR2bits.TMR3IF  = 0;    // clear INTERRUPT     

		n = scsSendWait(msgLen);
		retry++;
		if (optionW.W_ERROR)  errorCollision++;
	} while ((optionW.W_ERROR == 1) && (retry < 50));

	rBufferIdxP = 0xFF;		// <--- al prossimo giro non controlla messaggi doppi

	PIR4bits.CMP1IF  = 0;    // clear interrupt
    PIR2bits.TMR3IF  = 0;    // clear INTERRUPT FLAG
    INTCONbits.TMR0IF = 0;   // clear INTERRUPT FLAG
	INTCONbits.GIEH     = 1; // high priority interrupt enabled
	INTCONbits.GIEL     = 1; // low  priority interrupt enabled
//	PIE4bits.CMP1IE  = 1;    // open interrupt
//	INTCONbits.TMR0IE = 1;   // open INTERRUPT     

//  optionW.W_COLLISION = 0; // controllo collisioni ripristinato
	optionR.Val = 0;
    DelayMs(52);   // stream da 7 byte = 10+4=14mS  x 3stream = 42mS + ack = 50mS
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
void Write_config(BYTE mode)  // 0= scrivi      1=evita di scrivere
{
	if (mode == 0)
		Write_eep_array(&opt, EE_CONFIG, (char) sizeof(opt));
}
// ===================================================================================
void Read_tapparelle(void)
{
    maxtapp = Read_b_eep (EE_TAPPA_SETUP);
    Busy_eep ();
	Read_eep_array(&devicetappa, EE_TAPPA_SETUP+1, MAXTAPP*3);
}
// ===================================================================================
void Write_tapparelle(void)
{
    Write_b_eep (EE_TAPPA_SETUP, maxtapp);
    Busy_eep ();
	Write_eep_array(&devicetappa, EE_TAPPA_SETUP+1, MAXTAPP*3);
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
