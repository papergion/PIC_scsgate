; ===================================================================================
; autore: Guido Pagani - 2013 -   guidopic@altervista.org
;                                 http://guidopic.altervista.org/alter/index.html
; v 4.1  gestisce il buffer overflow
; v 4.2  gestisce il byte filtering A
; v 4.3  gestisce opzione di byte filtering per escludere gli ACK
; v 4.4  gestisce il byte filtering B
; v 4.5  suddivisione optionR e optionW
; v 4.6  gestione collisioni
; v 4.7  timeout di stream gestito a parametro configurabile 
; v 4.8  gestione collisioni disabilitabile 
; v 4.9  fix pausa di fine stream troppo lunga si perde dei pacchetti 
; v 4.10 rx uart1 in interrupt 
; v 4.11 fix problema spikes rapidi 
; v 4.12 fix errore rx uart interrupt
; ===================================================================================
; questo software e' rilasciato senza nessun tipo di garanzia - non sono responsabile
; dell'uso che ne farete ne' di eventuali danni che potreste causare con l'utilizzo
; che ricade quindi sotto la vostra intera responsabilita'
; ===================================================================================
; questo software e' liberamente utilizzabile per uso amatoriale e hobbystico
; con il solo vincolo di mantenere in evidenza i dati dell'autore
; ===================================================================================
; ne e' vietato l'uso a scopo commerciale e industriale senza il consenso dell'autore
; ===================================================================================
; ne e' vietata la riproduzione anche parziale senza il consenso dell'autore
; ===================================================================================
; ---------------------------------------------------------------------------------
;       File: scsintrp.asm
; ---------------------------------------------------------------------------------
; settings:
;       timer  va preimpostato a 1uS per tick
; #define SCS_TIMER_WRITEH   timer H x WRITE - 16 bit - 1uS
; #define SCS_TIMER_WRITEL   timer L x WRITE - 16 bit - 1uS
; #define SCS_TIMER_WRITE_IF timer write interrupt flag
; #define SCS_TIMER_WRITE_IE timer write interrupt flag enable
; #define SCS_TIME_WRITE     periodo bit in uS  (104 per 9600 baud)
; #define SCS_TIMER_READ     timer x READ - 8 bit - 4uS
; #define SCS_TIMER_READ_IF  timer read interrupt flag
; #define SCS_TIMER_READ_IE  timer read interrupt flag enable
; #define SCS_TIME_READ      periodo bit in uS  (104 per 9600 baud)
; #define SCS_TIMEOUT_BYTE   periodo di timeout in uS  (fine byte)
; #define SCS_TIMEOUT_STREAM periodo di timeout in uS  (fine stream)
; #define SCS_INPUT          porta di input scs
; #define SCS_OUT            porta di output scs
; #define SYNCHOUT           OUTPUT sincronismo di byte  (opzionale)
; #define SYNCHOUT1          OUTPUT sincronismo di stream (opzionale)
; #define SYNCHWOUT          OUTPUT sincronismo di collisione (opzionale)
; #define SCS_INTERRUPT_IF   Flag interrupt avvenuto
; #define SCS_INTERRUPT_IE   Flag interrupt abilitato
  #define TIMING_MEASURE     routines per misurare timig impulsi
;
; ---------------------------------------------------------------------------------

;;#ifndef _P18CXXX_H
#define _P18CXXX_H
#include "P18CXXX.INC"
#include "P18MACRO.INC"

#ifdef NOESP
#include "SCS_AA.INC"
#else
#ifdef ESP8285
#include "SCS_8285.INC"
#else
#ifdef ESP8266
#include "SCS_8266.INC"
#else
#ifdef RASPBERRY_7
#include "SCS_RASPY7.INC"
#else
#ifdef RASPBERRY_8
#include "SCS_RASPY8.INC"
#else
#ifdef USBSERIAL
#include "CANSCSGATE.INC"
#else
 error 997  definire macro ESP8266 o ESP8285 o RASPBERRY_7 o RASPBERRY_8 o USBSERIAL
#endif
#endif
#endif
#endif
#endif
#endif

; ---------------------------------------------------------------------------------
;  #define BUFFER_READ        10
;  #define SCS_TIMER_WRITEH   TMR3H
;  #define SCS_TIMER_WRITEL   TMR3L
;  #define SCS_TIMER_WRITE_IF PIR2,TMR3IF
;  #define SCS_TIMER_WRITE_IE PIE2,TMR3IE
;  #define SCS_TIMER_READ     TMR0L
;  #define SCS_TIMER_READ_IF  INTCON,TMR0IF
;  #define SCS_TIMER_READ_IE  INTCON,TMR0IE
;  #define SCS_TIME_READ      104       ; 104 uS
;  #define SCS_TIME_WRITE     208       ; 104 uS - doppio perche' WR timing 0,5uS
;  #define SCS_TIMEOUT_BYTE   936       ;     uS timeout END OF BYTE (104*9)
;  #define SCS_TIMEOUT_STREAM 200       ;     uS timeout END OF MESSAGE (oltre la fine byte)
;  #define SCS_INPUT          PORTB,RB0_PORTB
;  ;;efine SCS_INPUT          CMSTAT,CMP1OUT
;  #define SCS_OUT            LATC,RC6
;  #define SYNCHOUT           LATA,RA0
;  #define SYNCHOUT1          LATA,RA1
;  #define SCS_INTERRUPT_IF   INTCON,INT0IF
;  #define SCS_INTERRUPT_IE   INTCON,INT0IE
;  ;;efine SCS_INTERRUPT_IF   PIR4bits.CMP1IF
;  ;;efine SCS_INTERRUPT_IE   PIE4bits.CMP1IE
; ---------------------------------------------------------------------------------
        RADIX DEC
; ---------------------------------------------------------------------------------
        global    scsInit
        global    scsInterruptInput
        global    scsInterruptTimer
        global    scsSend       ; send con interrupt
        global    scsSendWait   ; send diretto
        global    scsSendTest
        global    scsMessageRx
        global    scsPulse
		global    inUART 

#ifdef  TIMING_MEASURE
        global    scsTiming
        global    scsTMR0L
        global    scsTMR0H
#endif
        global    scsMessageTx
;;;        global    rBitCount   ;; solo per test
;;;        global    rByteCount ;; solo per test
        global    wPeriodoDomin ;; FFFF - periodo dominante  [write timer]
        global    wPeriodoRece  ;; FFFF - periodo Recessivo [write timer]
        global    rBufferIdxW
        global    optionW
        global    optionR
        global    stream_timeout
; ---------------------------------------------------------------------------------
        global    rBufferIdxR ; v 4.1  gestisce il buffer overflow
        global    filterByte_A  ; v 4.2  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
							  ; v 4.2  HB  0100=exclude ack        1000=exclude equal tlgrm
                              ; v 4.2  LB: byte nr su cui filtrare (1-F) - comparato con rByteCount (0=filteroff)
        global    filterValue_A ; v 4.2  valore byte di filtro   (0-FF)
        global    filterByte_B  ; v 4.4  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
                              ; v 4.4  LB: byte nr su cui filtrare (1-F) - comparato con rByteCount (0=filteroff)
        global    filterValue_B ; v 4.4  valore byte di filtro   (0-FF)
; ---------------------------------------------------------------------------------
		global	  uartMessage;	// buffer rx uart1
		global    uPtrW;		// ptr next mem
		global    uMax;			// nr of chars
		global    uState;		// uart error
		global    uSpikes;		// spike errors
; ---------------------------------------------------------------------------------
scsram       udata
; ---------------------------------------------------------------------------------
; rx fields
scsMessageRx     res 16*BUFFER_READ;   ;; 10 buffer da 16 bytes l'uno (LL+15 chars)
scsMessageTx     res 16;
rByte            res 1;
rBitCount        res 1;
rByteCount       res 1;
rBufferIdxW      res 1;
rBufferIdxR      res 1;
filterByte_A       res 1;	// 4.2
filterValue_A      res 1;	// 4.2
filterByte_B       res 1;	// 4.4
filterValue_B      res 1;	// 4.4
uSpikes          res 1;

wPeriodoDomin    res 2;
wPeriodoRece     res 2;
wByte            res 1;  190
wByteCount       res 1;
wBitCount        res 1;
wMaxLen          res 1;
wPauseL          res 1;
wPauseH          res 1;
ixTransmit       res 1;

optionW          res 1;
 #define      W_COLLISION  1	;   NON PIU USATO
 
 #define      W_ERROR      2	; 0=scrittura ok					1=errore collisione o timeout
 #define      W_INTRP      3
 #define      W_WAIT       4
 #define      W_EXIT       5
 #define      W_EOB        6
 #define      W_END        7

optionR          res 1;
 #define      R_EOB        0
;#define      T_START      1
 #define      T_NOW        1
 #define      R_READING    2
 #define      R_FILTER_BYTE_DISCARD        3	; v 4.2  gestisce il byte filtering

stream_timeout   res 1;
saveTMR          res 1;      200
saveFSR0L        res 1;
saveFSR0H        res 1;
saveFSR1L        res 1;
saveFSR1H        res 1;
saveBSR          res 1;

;--------------------------------------------------------------------------------------
#ifdef  TIMING_MEASURE
scstmr0l     udata
scsTMR0L         res 256
scstmr0h     udata
scsTMR0H         res 256
#endif
;--------------------------------------------------------------------------------------
uartram       udata
#ifdef _BUF_128
uartMessage		 res 128;	// buffer rx uart1
#else
uartMessage		 res 64;	// buffer rx uart1
#endif
uPtrW			 res 1;		// ptr next mem
uMax			 res 1;		// nr of chars
uState           res 1;
 #define      U_FERR        0
 #define      U_OERR        1
usaveFSR1L       res 1;
usaveFSR1H       res 1;
usaveBSR         res 1;
;--------------------------------------------------------------------------------------
SCS_VERSION      EQU 0x02
;--------------------------------------------------------------------------------------
scsMain   code
;--------------------------------------------------------------------------------------
scsInit
    movlb   HIGH(uartMessage)      ;// PER INDIRIZZARE DATA BANK
	clrf    uPtrW
	clrf    uMax
	clrf    uState
	clrf    uSpikes
	
    movlb   HIGH(rByte)            ;// PER INDIRIZZARE DATA BANK
    clrf    rBitCount
    clrf    rByteCount
#ifdef SCS_TIMEOUT_STREAM
	movf    stream_timeout,w
	bnz     scsInitEnd		;; non ancora usato
	
    movlw   low (0xFF - (SCS_TIMEOUT_STREAM / 4) + 1)
    movwf   stream_timeout
scsInitEnd
#endif
	return
;--------------------------------------------------------------------------------------
; scsInterruptInput viene chiamato come routine di interrupt quando l'input passa da 0 a 1
;  input=comparatore passa da 0 a 1 quando il segnale in input si abbassa sotto la soglia
;--------------------------------------------------------------------------------------
scsInterruptInput
;-----------------------------------------------------------------------------
; pre-code - copiato da interrupt C
;
; 0220    CFDA     MOVFF 0xfda, 0xfe4
; 0224    CFE2     MOVFF 0xfe2, 0xfda
; 0228    CFE9     MOVFF 0xfe9, 0xfe4
; 022C    CFEA     MOVFF 0xfea, 0xfe4
; 0230    CFF3     MOVFF 0xff3, 0xfe4
; 0234    CFF4     MOVFF 0xff4, 0xfe4
; 0238    52E6     MOVF 0xfe6, F, ACCESS
;-----------------------------------------------------------------------------
        bcf     SCS_INTERRUPT_IE; // INT0 now disabled
        bcf     SCS_INTERRUPT_IF; //
;-----------------------------------------------------------------------------
;;                 MOVFF FSR2H, PREINC1     ; da fsr2h a  preinc1
;;                 MOVFF FSR1H, FSR2H       ; da fsr1h a  fsr2h
                   MOVFF FSR0L, PREINC1     ; da fsr0l a  preinc1
                   MOVFF FSR0H, PREINC1     ; da fsr0h a  preinc1
                   MOVFF PRODL, PREINC1     ; da PRODL a  preinc1
                   MOVFF PRODH, PREINC1     ; da PRODH a  preinc1
                   MOVF  POSTINC1, F, ACCESS    ; postinc1
;-----------------------------------------------------------------------------
;
;
        movlb   HIGH(rByte)            ;// PER INDIRIZZARE DATA BANK
        movff   SCS_TIMER_READ,saveTMR ; in saveTMR il valore attuale del timer di lettura
;;      movf    POSTDEC1,w             ; dummy - only for decrement
;;      movf    POSTDEC1,w             ; recupera parametro 1 dallo stack software
#ifdef  SCS_LED
        bsf     SCS_LED
#endif
        bsf     optionR,R_READING      ; in stato di lettura !

        movf    rBitCount,f            ; bits gia' ricevuti
        bnz     rRealBits
        
        
;--------------------------------------------------------------------------------------------------------------
rStartBit                              ; zero - e' lo start bit - inizia un nuovo BYTE -
        bcf     SCS_TIMER_READ_IE;     // disable TMR0 interrupt

        movlw   (SCS_TIME_READ/8)+16  ;// 104/8=13+16=29*4=116  256-29=227*4=908uS
;
; nuovo calcolo:   Timeout da parametro
;       0xFF - (SCS_TIMEOUT_BYTE/4)   255 - (936/4) + 1 = 22
;
;;      movlw   low (0xFF - (SCS_TIMEOUT_BYTE / 4) + 1)
;
; imposta il timer di lettura al valore iniziale - quando arrivera' in overflow il byte sarà finito
;
        movwf   SCS_TIMER_READ;   ;//  13 (52uS half scs period for round)
                                  ;// +16 (64uS) for reduced timeout = (1024-8bit-bytegap)/4
                                  ;//      (1024-8*104-104)/4 =(1024 - 936)/4=22
        bcf     SCS_TIMER_READ_IF;     // clear INTERRUPT FLAG
        bsf     SCS_TIMER_READ_IE;     // enable TMR0 interrupt
        movlw   0xFF
        movwf   rByte
        bcf     optionR,R_EOB

        clrf    rBitCount
        incf    rBitCount,f                ; il prossimo sara'  un bit valido
        bra     scsInterruptInputEnd
;--------------------------------------------------------------------------------------------------------------
rRealBits
        incf    rBitCount,f                ; bit valido
        movlw   16              ; sottrae byte-gap
        subwf   saveTMR,f



; int0 viene attivato in salita (fronte di discesa sul bus) che indica la situazione
;       di "dominante" - qui interpretata come bit a 0
;;      movlw   26

        movlw   SCS_TIME_READ/4 ;// timer tick 4 uS
        subwf   saveTMR,f       ;// 104uS per bit   /26   *38
        bnc     rBitNull
        subwf   saveTMR,f       ;// 104uS per bit   /26   *38
        bnc     rBit0
        subwf   saveTMR,f       ;// 104uS per bit   /26   *38
        bnc     rBit1
        subwf   saveTMR,f       ;// 104uS per bit   /26   *38
        bnc     rBit2
        subwf   saveTMR,f       ;// 104uS per bit   /26   *38
        bnc     rBit3
        subwf   saveTMR,f       ;// 104uS per bit   /26   *38
        bnc     rBit4
        subwf   saveTMR,f       ;// 104uS per bit   /26   *38
        bnc     rBit5
        subwf   saveTMR,f       ;// 104uS per bit   /26   *38
        bnc     rBit6
        subwf   saveTMR,f       ;// 104uS per bit   /26   *38
        bnc     rBit7
rBitNull
        bra     scsInterruptInputEnd

rBit0
        bcf     rByte,0
        bra     scsInterruptInputEnd
rBit1
        bcf     rByte,1
        bra     scsInterruptInputEnd
rBit2
        bcf     rByte,2
        bra     scsInterruptInputEnd
rBit3
        bcf     rByte,3
        bra     scsInterruptInputEnd
rBit4
        bcf     rByte,4
        bra     scsInterruptInputEnd
rBit5
        bcf     rByte,5
        bra     scsInterruptInputEnd
rBit6
        bcf     rByte,6
        bra     scsInterruptInputEnd
rBit7
        bcf     rByte,7

scsInterruptInputEnd
#ifdef  SCS_LED
        bcf     SCS_LED
#endif

scsInterruptInputExit
        btfsc   SCS_INTERRUPT_IF; 
		incf    uSpikes,f
;-----------------------------------------------------------------------------
; post-code - copiato da interrupt C
;
; 028C    52E5     MOVF 0xfe5, F, ACCESS
; 028E    CFE5     MOVFF 0xfe5, 0xff4
; 0292    CFE5     MOVFF 0xfe5, 0xff3
; 0296    CFE5     MOVFF 0xfe5, 0xfea
; 029A    CFE5     MOVFF 0xfe5, 0xfe9
; 029E    CFE5     MOVFF 0xfe5, 0xfda
; 02A2    0011     RETFIE 0x1
;-----------------------------------------------------------------------------
                   MOVF  POSTDEC1, F, ACCESS    ; postdec1
                   MOVFF POSTDEC1, PRODH       ; da postdec1 a fsr2h
                   MOVFF POSTDEC1, PRODL       ; da postdec1 a fsr2h
                   MOVFF POSTDEC1, FSR0H       ; da postdec1 a fsr2h
                   MOVFF POSTDEC1, FSR0L       ; da postdec1 a fsr0l
;;                 MOVFF POSTDEC1, FSR2H       ; da postdec1 a fsr0h
;-----------------------------------------------------------------------------
        bcf     SCS_INTERRUPT_IF; //
        bsf     SCS_INTERRUPT_IE; // INT0 now enabled
;-----------------------------------------------------------------------------
                   RETFIE 0x1
;-----------------------------------------------------------------------------
;                  return



;--------------------------------------------------------------------------------------------------------------
;--------------------------------------------------------------------------------------------------------------
;--------------------------------------------------------------------------------------------------------------
rUART
;;;        btfss   PIE1,RC1IE   ; interrupt uart1 enabled? ------> TODO
;;;            bra     scsInterruptTimerEnd                ------> TODO
        btfss   PIR1,RC1IF   ; interrupt uart1?
            bra     scsInterruptTimerEnd

#ifdef  UART_LED
        bsf     UART_LED
#endif
        bcf   PIR1,RC1IF  
        
		movlb   HIGH(uartMessage)            ;// PER INDIRIZZARE DATA BANK

		btfss RCSTA1,OERR    ;  se OERR oppure FERR   segnalare errore di lettura !!!!
		  bra rUARTok
        bcf  RCSTA1,CREN
        nop
        bsf  RCSTA1,CREN
        bsf  uState,U_OERR
rUARTok        
		btfsc RCSTA1,FERR    
          bsf  uState,U_FERR
		
        lfsr    0,uartMessage ;// pointer at start of buffers
        movf    uPtrW,w
        addwf   FSR0L,f     ;// pointer at start of uart buffer
        btfsc   STATUS,C
        incf    FSR0H,f     ;// pointer at start of uart buffer
		movf    RCREG1,w
        movwf   INDF0		;// update buffer contents

		incf    uMax,f
		btfss   uMax,6		;// se arriva a 0x80 torna a 0x7F
		  bra   rUARTend
		decf    uMax,f
        bra     scsInterruptTimerEnd

rUARTend
		incf    uPtrW,f
		btfsc   uPtrW,6		;// se arriva a 0x80 riparte da 0
			clrf    uPtrW

        bra     scsInterruptTimerEnd

;--------------------------------------------------------------------------------------
; routine di LETTURA
; scsInterruptTimer viene chiamato quando sono trascorsi 908 uS dallo start bit (end of byte)
;--------------------------------------------------------------------------------------
; routine di SCRITTURA
; scsInterruptTimer viene chiamato quando sono trascorsi i necessari uS dal bit precedente
;                   e quindi va scritto il bit successivo - o lo start bit del nuovo byte
;--------------------------------------------------------------------------------------
scsInterruptTimer
;-----------------------------------------------------------------------------
; pre-code - copiato da interrupt C
;
; 0220    CFDA     MOVFF 0xfda, 0xfe4
; 0224    CFE2     MOVFF 0xfe2, 0xfda
; 0228    CFE9     MOVFF 0xfe9, 0xfe4
; 022C    CFEA     MOVFF 0xfea, 0xfe4
; 0230    CFF3     MOVFF 0xff3, 0xfe4
; 0234    CFF4     MOVFF 0xff4, 0xfe4
; 0238    52E6     MOVF 0xfe6, F, ACCESS
;-----------------------------------------------------------------------------
        bcf     SCS_INTERRUPT_IE; // INT0 now disabled
;-----------------------------------------------------------------------------
;;                 MOVFF FSR2H, PREINC1     ; da fsr2h a  preinc1
;;                 MOVFF FSR1H, FSR2H       ; da fsr1h a  fsr2h
                   MOVFF FSR0L, PREINC1     ; da fsr0l a  preinc1
                   MOVFF FSR0H, PREINC1     ; da fsr0h a  preinc1
                   MOVFF PRODL, PREINC1     ; da PRODL a  preinc1
                   MOVFF PRODH, PREINC1     ; da PRODH a  preinc1
                   MOVF  POSTINC1, F, ACCESS    ; postinc1
;-----------------------------------------------------------------------------
        movlb   HIGH(rByte)         ;// PER INDIRIZZARE DATA BANK
;-----------------------------------------------------------------------------
tTestRead
        btfss   SCS_TIMER_READ_IE;
          bra   tTestWrite
        btfsc   SCS_TIMER_READ_IF;
          bra   rTimerInterrupt      ; interrupt di lettura ha precedenza
          
tTestWrite
        btfss   SCS_TIMER_WRITE_IE;
;;;       bra   scsInterruptTimerEnd
          bra   rUART

        btfsc   SCS_TIMER_WRITE_IF;
          bra   wTimerInterrupt
;;;     bra     scsInterruptTimerEnd
        bra   rUART

rTimerInterrupt
        bcf     SCS_TIMER_READ_IE;   // TMR0 now disabled
        bcf     SCS_TIMER_READ_IF;   // clear interrupt flag
;
        movf    rBitCount,f    ;  sono arrivati bits validi?
        bz      rTestEob
rEndOfByte                     ;    si, e' un normale fine byte

        bsf     optionR,R_EOB        ; sono trascorsi 910 uS + 100 uS (1 byte + 300uS)
        movlw   14             ;
        cpfsgt  rByteCount
          incf  rByteCount,f   ;// increment byte counter
        lfsr    0,scsMessageRx ;// pointer at start of buffers
        movlw   16
        mulwf   rBufferIdxW    ;// buffer pointer x 16 - result in PRODL
        movf    PRODL,w        ;// result in W
        addwf   FSR0L,f        ;// pointer at start of buffer to write into
        movff   rByteCount,INDF0  ;// update buffer length



; v 4.2  gestisce il byte filtering -A----------------------------------------------------------
		movf    filterByte_A,w
		andlw   0x0F
		   bz   noByteFilterA
		xorwf   rByteCount,w
		  bnz   noByteFilterA
        movf    rByte,w
;		// filterByte_A;	//v 4.2  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
        btfsc   filterByte_A,4
          bra   ByteFilterIncludeA
        btfss   filterByte_A,5
		  bra   noByteFilterA
		  
ByteFilterExcludeA
		xorwf   filterValue_A,w
		  bnz   noByteFilterA
	    bsf     optionR,R_FILTER_BYTE_DISCARD
	    bra     noByteFilterA
		  
ByteFilterIncludeA
		xorwf   filterValue_A,w
		  bz    noByteFilterA
	    bsf     optionR,R_FILTER_BYTE_DISCARD
;	    bra     noByteFilterA
   
noByteFilterA
; v 4.2  gestisce il byte filtering ------------------------------------------------------------


; v 4.4  gestisce il byte filtering -B----------------------------------------------------------
		movf    filterByte_B,w
		andlw   0x0F
		   bz   noByteFilterB
		xorwf   rByteCount,w
		  bnz   noByteFilterB
        movf    rByte,w
;		// filterByte_B;	//v 4.4  HB: 0000=byte filter off    0001=byte filter include    byte 0010=filter exclude
        btfsc   filterByte_B,4
          bra   ByteFilterIncludeB
        btfss   filterByte_B,5
		  bra   noByteFilterB
		  
ByteFilterExcludeB
		xorwf   filterValue_B,w
		  bnz   noByteFilterB
	    bsf     optionR,R_FILTER_BYTE_DISCARD
	    bra     noByteFilterB
		  
ByteFilterIncludeB
		xorwf   filterValue_B,w
		  bz    noByteFilterB
	    bsf     optionR,R_FILTER_BYTE_DISCARD
;	    bra     noByteFilterB
   
noByteFilterB
; v 4.4  gestisce il byte filtering ------------------------------------------------------------
		   
		   
		   
        movf    rByteCount,w
        addwf   FSR0L,f        ;// pointer at start of write buffer byte
        movff   rByte,INDF0    ;// update buffer contents

        clrf    rBitCount            ; a zero il numero di bits validi
        bcf     SCS_TIMER_READ_IE;   ;// TMR0 interrupt off
;;      movlw   206          ;   // msg end = 200uS   - time out: 50*4=200uS senza segnale
;;      movlw   190          ;   // msg end = 200uS   - time out: 65*4=200uS senza segnale
;;                           ;                          oltre il timeout di byte

        movf    stream_timeout,w

; nuovo calcolo:   Timeout da parametro
;       (0xFF - (SCS_TIMEOUT_STREAM/4) + 1       255 - (260/4) + 1 = 191
;
;;      movlw   low (0xFF - (SCS_TIMEOUT_STREAM / 4) + 1)
;

        movwf   SCS_TIMER_READ
        bsf     SCS_TIMER_READ_IE;   ;// TMR0 interrupt on
        bra     rInterruptTimerEnd


rTestEob                             ; sono trascorsi 910 uS + 100 uS (1 byte + 300uS)
        btfss   optionR,R_EOB
          bra   rInterruptTimerEnd


; fine message stream
        bcf     optionR,R_EOB


; v 4.2  gestisce il byte filtering ------------------------------------------------------------
		btfsc  optionR,R_FILTER_BYTE_DISCARD
		  bra   rTestEob1       
; v 4.2  gestisce il byte filtering ------------------------------------------------------------


; v 4.3  gestisce esclusione ACK ----------------------------------------------------------------------------
;	filterByte_A.HB  0100=exclude ack        1000=exclude equal tlgrm
        btfss   filterByte_A,6
          bra   IncrementBufferW
        movf    rByteCount,w
        xorlw   0x01  
          bnz   IncrementBufferW
        movf	rByte,w  
        xorlw   0xA5  
		  bz    rTestEob1       
; v 4.3  gestisce esclusione ACK ----------------------------------------------------------------------------


IncrementBufferW
        incf    rBufferIdxW,f  ;// increment buffer pointer
        movlw   BUFFER_READ    ;// max buffer pointer (0-9) is 9 ; else rotate to zero
        cpfslt  rBufferIdxW
        clrf    rBufferIdxW

        ; v 4.1  gestisce il buffer overflow-------------------------------------------------------------------------
        movf    rBufferIdxR,w
        cpfseq  rBufferIdxW
		  bra   rTestEob1       
        
  ; --------------------------------- E R R O R E ------------------------------------------------------------------
  ;     decfsz  rBufferIdxW,f  ;// decrement buffer pointer
  ;     bra   rTestEob1      ;// SBAGLIATO! deve controllare zero PRIMA di decrementare, oppure testare NEGATIVO
  ; --------------------------------- CORREZIONE  ------------------------------------------------------------------
        decf    rBufferIdxW,f  ;// decrement buffer pointer
		  bnn   rTestEob1
  ; ----------------------------------------------------------------------------------------------------------------		  
		  
        movlw   BUFFER_READ-1    ;// max buffer pointer (0-9) is 9 ; else rotate to zero
        movwf   rBufferIdxW
        ; v 4.1  gestisce il buffer overflow-------------------------------------------------------------------------
        
rTestEob1
        clrf    rBitCount
        clrf    rByteCount
	    bcf     optionR,R_FILTER_BYTE_DISCARD	; v 4.2  gestisce il byte filtering ------------------------------------------------------------
        bcf     optionR,R_READING

rInterruptTimerEnd
        btfss   SCS_TIMER_WRITE_IE;
          bra   scsInterruptTimerEnd
        btfsc   SCS_TIMER_WRITE_IF;
          bra   wTimerInterrupt
          
        bra   rUART    ; <------------------------ TODO
          
scsInterruptTimerEnd
;-----------------------------------------------------------------------------
        btfsc   SCS_INTERRUPT_IF; 
		incf    uSpikes,f
;-----------------------------------------------------------------------------
; post-code - copiato da interrupt C
;
; 028C    52E5     MOVF 0xfe5, F, ACCESS
; 028E    CFE5     MOVFF 0xfe5, 0xff4
; 0292    CFE5     MOVFF 0xfe5, 0xff3
; 0296    CFE5     MOVFF 0xfe5, 0xfea
; 029A    CFE5     MOVFF 0xfe5, 0xfe9
; 029E    CFE5     MOVFF 0xfe5, 0xfda
; 02A2    0011     RETFIE 0x1
;-----------------------------------------------------------------------------
                   MOVF  POSTDEC1, F, ACCESS    ; postdec1
                   MOVFF POSTDEC1, PRODH       ; da postdec1 a prodh
                   MOVFF POSTDEC1, PRODL       ; da postdec1 a prodl
                   MOVFF POSTDEC1, FSR0H       ; da postdec1 a fsr0h
                   MOVFF POSTDEC1, FSR0L       ; da postdec1 a fsr0l
;;                 MOVFF POSTDEC1, FSR2H       ; da postdec1 a fsr0h
;-----------------------------------------------------------------------------
        bcf     SCS_INTERRUPT_IF; //
        bsf     SCS_INTERRUPT_IE; // INT0 now enabled
;-----------------------------------------------------------------------------
                   RETFIE 0x1
;-----------------------------------------------------------------------------
;                  return













;--------------------------------------------------------------------------------------
; scsSendTest  invia ripetutamente il byte passato fino a interruzione manuale rs232 o can
;--------------------------------------------------------------------------------------
scsSendTest ; (carattere)
;-----------------------------------------------------------------------------
        movf    BSR,w                  ;
        movlb   HIGH(wByte)            ;// PER INDIRIZZARE DATA BANK
        movwf   saveBSR
        movff   FSR1H,saveFSR1H        ;// salvataggio FSR1 - dopo il decremento
        movff   FSR1L,saveFSR1L
        movff   FSR0H,saveFSR0H        ;// salvataggio FSR0
        movff   FSR0L,saveFSR0L

        movff   POSTDEC1, scsMessageRx ; dummy - only for decrement
        movff   POSTDEC1, scsMessageRx ; recupera parametro 1 dallo stack software
        bcf     optionW,W_END
        bcf     optionW,W_EOB
        bsf     optionW,W_EXIT
        bsf     optionW,W_WAIT
        bcf     optionW,W_INTRP
wSendTest
        clrwdt
        movlw   1
        movwf   wMaxLen
        movff   scsMessageRx,wByte
        call    wStreamLoop1       ; invia il carattere

;;      btfsc   PIR1,RC1IF
;;        bra   rEndStream             ; fine se arriva un carattere rs232
;;      btfsc  COMSTAT,NOT_FIFOEMPTY   ; fine se arriva un msg can
;;        bra   rEndStream             ; fine se arriva un carattere rs232
        bra     wSendTest
        movlw   0x00
        bra     Restore_Context


;--------------------------------------------------------------------------------------
; scsSendWait  va chiamata nell'istante in cui si vuole inviare un messaggio scs
;              scrittura diretta - dura circa 940uS per ogni byte
;--------------------------------------------------------------------------------------
scsSendWait ; (length)
;-----------------------------------------------------------------------------
        movf    BSR,w                  ;
        movlb   HIGH(wByte)            ;// PER INDIRIZZARE DATA BANK
        movwf   saveBSR
        movff   FSR1H,saveFSR1H        ;// salvataggio FSR1 - dopo il decremento
        movff   FSR1L,saveFSR1L
        movff   FSR0H,saveFSR0H        ;// salvataggio FSR0
        movff   FSR0L,saveFSR0L
        clrf    ixTransmit
        bsf     optionW,W_WAIT
        bcf     optionW,W_EXIT
        bcf     optionW,W_EOB
        bcf     optionW,W_END
        bcf     optionW,W_INTRP
        bcf     optionW,W_ERROR

        movff   POSTDEC1, wMaxLen       ; dummy - only for decrement
        movff   POSTDEC1, wMaxLen      ; recupera parametro 1 dallo stack software
        lfsr    FSR0,scsMessageTx      ; in FSR0 indirizzo tx buffer
        clrf    wByteCount
#ifdef  SYNCHOUT1
        bsf     SYNCHOUT1
#endif
#ifdef  SYNCHWOUT
        btfsc   SCS_INPUT
        bsf     SYNCHWOUT
        btfss   SCS_INPUT
        bcf     SYNCHWOUT
#endif
#ifdef  SYNCHWERR
        bcf     SYNCHWERR
#endif
        bra   wStreamLoop


;--------------------------------------------------------------------------------------
; scsSend    va chiamata nell'istante in cui si vuole inviare un messaggio scs
;            la scrittura dura 35uS e poi prosegue in interrupt
;--------------------------------------------------------------------------------------
scsSend  ; (length)
;-----------------------------------------------------------------------------
        movf    BSR,w                  ;
        movlb   HIGH(wByte)            ;// PER INDIRIZZARE DATA BANK
        movwf   saveBSR
        movff   FSR1H,saveFSR1H        ;// salvataggio FSR1 - dopo il decremento
        movff   FSR1L,saveFSR1L
        movff   FSR0H,saveFSR0H        ;// salvataggio FSR0
        movff   FSR0L,saveFSR0L
        clrf    ixTransmit
        bcf     optionW,W_WAIT  ; <-------
        bcf     optionW,W_EXIT
        bcf     optionW,W_EOB
        bcf     optionW,W_END
        bcf     optionW,W_INTRP
        bcf     optionW,W_ERROR

        movff   POSTDEC1, wMaxLen       ; dummy - only for decrement
        movff   POSTDEC1, wMaxLen      ; recupera parametro 1 dallo stack software
        lfsr    FSR0,scsMessageTx      ; in FSR0 indirizzo tx buffer
        clrf    wByteCount
#ifdef  SYNCHOUT1
        bsf     SYNCHOUT1
#endif
#ifdef  SYNCHWERR
        bcf     SYNCHWERR
#endif

wStreamLoop
        movff   POSTINC0,wByte         ; byte da inviare
        incf    ixTransmit,f
wStreamLoop1
#ifdef  SYNCHOUT
        bsf     SYNCHOUT
#endif
        movlw   0x08
        movwf   wBitCount              ; numero bits da inviare
#ifdef  SYNCHOUT1
        bcf     SYNCHOUT1
#endif

wByteLoop
; ------------- imposta timer3 per impulso dominante
;;      movlw   high(0xFFFF - (SCS_TIME_WRITE / 3) + 1); 1/3 35uS
        movf    wPeriodoDomin+1,w     ;; FFFF - periodo dominante  [write timer]
        movwf   SCS_TIMER_WRITEH
;;      movlw   low (0xFFFF - (SCS_TIME_WRITE / 3) + 1); 1/3 35uS
        movf    wPeriodoDomin,w       ;; FFFF - periodo dominante  [write timer]
        movwf   SCS_TIMER_WRITEL
        bcf     SCS_TIMER_WRITE_IF     ; clr  timer  interrupt flag
        
;;;;    bsf     SCS_TIMER_WRITE_IE     ; set   timer interrupt enable for READ

;; 4.6 anti collisione: se SCS_INPUT vale 1 terminare in errore (gia' dominante prima di inizio bit)
        btfsc   SCS_INPUT
          bra   wCollisionError

#ifdef  SYNCHWOUT
        btfsc   SCS_INPUT
        bsf     SYNCHWOUT
        btfss   SCS_INPUT
        bcf     SYNCHWOUT
#endif
          
wNoCollisionStart
        bsf     SCS_OUT                ; inizio impulso SCS
; ----------------------------------------------ora il BUS e' BASSO - dominante --------------
; --------------------------------------------------------------------------------------------
; --------------------------------------------------------------------------------------------
; intanto che aspetta calcola la pausa di attesa prima del prossimo impulso
; --------------------------------------------------------------------------------------------
; --------------------------------------------------------------------------------------------
                    ; all'impulso successivo mancano i 2/3
;;      movlw   high(0xFFFF - (SCS_TIME_WRITE / 3) * 2 + 1); 2/3 70uS
        movf    wPeriodoRece+1,w      ;; FFFF - periodo Recessivo [write timer]
        movwf   wPauseH
;;      movlw   low (0xFFFF - (SCS_TIME_WRITE / 3) * 2 + 1); 2/3 70uS
        movf    wPeriodoRece,w        ;; FFFF - periodo Recessivo [write timer]
        movwf   wPauseL

        movf    wBitCount,f  ; se ha finito gli 8 bit
        bz      wEndByte     ;    deve solo aggiungere la pausa finale
wCalcPause ; ------------------------------------------------------------------------intanto il BUS e' BASSO - dominante ---------
        rrcf    wByte,f      ; ruota a destra di un bit

        btfss   STATUS,C     ; se il bit successivo e' UNO  non servono impulsi alti
          bra   wBitZero
                             ; e quindi si somma un periodo intero (3/3)
        movlw   low (0xFFFF - SCS_TIME_WRITE + 1);
        addwf   wPauseL,f
        movlw   high(0xFFFF - SCS_TIME_WRITE + 1);
        addwfc  wPauseH,f
        decf    wBitCount,f  ; decrementa bit counter
;;      movf    wBitCount,f  ; se non ha finito gli 8 bit
        bnz     wCalcPause   ;   ritorna a controllare il bit successivo
                             ; fine calcolo pausa successiva

wEndByte                     ; fine byte aggiunge 1 periodo e 1/3 (140uS)
        movlw   low (0xFFFF - (SCS_TIME_WRITE/3)*4 + 1);
        addwf   wPauseL,f
        movlw   high(0xFFFF - (SCS_TIME_WRITE/3)*4 + 1);
        addwfc  wPauseH,f
        bsf     optionW,W_EOB
                             ; fine settimo byte - aggiunge 6 periodi (630uS)
                             ; fix - aggiunge solo 2 periodi
;;      movf    wByteCount,w
;;      xorlw   0x06
	    movf    wMaxLen,w
		xorlw   0x01   
        bnz     wEndCalc
        movlw   low (0xFFFF - SCS_TIME_WRITE*1 + 1);    fix 4.9
        addwf   wPauseL,f
        movlw   high(0xFFFF - SCS_TIME_WRITE*1 + 1);    fix 4.9
        addwfc  wPauseH,f
        bra     wEndCalc

wBitZero
        decf    wBitCount,f  ; decrementa bit counter
        bcf     optionW,W_EOB

wEndCalc                               ; fine calcolo pausa successiva
#ifdef  SYNCHWOUT
        btfsc   SCS_INPUT
        bsf     SYNCHWOUT
        btfss   SCS_INPUT
        bcf     SYNCHWOUT
#endif
; ---------------------------------------------------------intanto il BUS e' sempre BASSO - dominante ---------
; --------------------------------------------------------------------------------------------
; --------------------------------------------------------------------------------------------
; fine calcolo - la pausa successiva deve durare "wPauseH wPauseL" microsecondi
;                se optionW,7  e' ZERO dovra' essere seguita da un impulso
;                altrimenti no (byte finito)
; --------------------------------------------------------------------------------------------
; --------------------------------------------------------------------------------------------

; se il flag IF fosse immediatamente su, abbiamo speso troppo tempo

        btfsc   SCS_TIMER_WRITE_IF     ;
          bsf   optionW,W_ERROR        ; <========================================================

wBitLoop
; // tempo di impulso 35uS
        btfss   SCS_TIMER_WRITE_IF     ; aspetta che siano trascorsi i uS previsti (1/3)
          bra   wBitLoop
        bcf     SCS_OUT                ; fine   impulso SCS
; ---------------------------------------------------------ora il BUS e' ALTO - recessivo ---------
		bcf		SCS_INTERRUPT_IF

#ifdef  SYNCHWOUT
        btfsc   SCS_INPUT
        bsf     SYNCHWOUT
        btfss   SCS_INPUT
        bcf     SYNCHWOUT
#endif

#ifdef  SYNCHOUT
        bcf     SYNCHOUT
#endif
; --------------------------------------------------------------------------------------------
; ora pausa come calcolato
; --------------------------------------------------------------------------------------------

        movff   wPauseH,SCS_TIMER_WRITEH
        movff   wPauseL,SCS_TIMER_WRITEL
        bcf     SCS_TIMER_WRITE_IE
        bcf     SCS_TIMER_WRITE_IF     ; clr  timer  interrupt flag

        btfsc   optionW,W_WAIT; se chiamata con interrupt
          bra   wWaitEnter

;       esce e riprendera' in interrupt a tempo scaduto
        bsf     SCS_TIMER_WRITE_IE     ; set   timer interrupt enable for READ
        btfsc   optionW,W_INTRP
          bra   scsInterruptTimerEnd

        movlw   0x00
        bra     Restore_Context

wWaitEnter
; // tempo di pausa 70-105uS


wUART ; ------------------------------------------------------------------------------------------------------------
;;;;        btfss   PIE1,RC1IE   ; interrupt uart1 enabled?
;;;;			bra wWaitLoop
        btfss   PIR1,RC1IF   ; interrupt uart1?
			bra wWaitLoop

#ifdef  UART_LED
        bsf     UART_LED
#endif
wUARTgo						; <------------ TODO
        bcf   PIR1,RC1IF  
        
		movlb   HIGH(uartMessage)            ;// PER INDIRIZZARE DATA BANK

		btfss RCSTA1,OERR      ;se OERR oppure FERR   segnalare errore di lettura !!!!
		  bra wUARTok
        bcf  RCSTA1,CREN
        nop
        bsf  RCSTA1,CREN
        bsf  uState,U_OERR
wUARTok        
		btfsc RCSTA1,FERR    
          bsf  uState,U_FERR

        lfsr    1,uartMessage ;// pointer at start of buffers
        movf    uPtrW,w
        addwf   FSR1L,f     ;// pointer at start of uart buffer
        btfsc   STATUS,C
        incf    FSR1H,f     ;// pointer at start of uart buffer
		movf    RCREG1,w
        movwf   INDF1		;// update buffer contents

		incf    uMax,f
		btfss   uMax,6		;// se arriva a 0x80 torna a 0x7F
		  bra   wUARTend
		decf    uMax,f
		bra wWaitLoopStart

wUARTend
		incf    uPtrW,f
		btfsc   uPtrW,6		;// se arriva a 0x80 riparte da 0
		  clrf    uPtrW

		; ------------------------------------------------------------------------------------------------------------

wWaitLoopStart
		movlb   HIGH(rByte)            ;// PER INDIRIZZARE DATA BANK
;; 4.6 anti collisione: se SCS_INPUT vale 1 terminare in errore
        btfsc   SCS_INPUT
          bra   wCollisionError		   ; il bus ora è dominante - qualcun altro sta scrivendo <<<<<<<<<<<<<<<<<<<<

wWaitLoop
#ifdef  SYNCHWOUT
        btfsc   SCS_INPUT
        bsf     SYNCHWOUT
        btfss   SCS_INPUT
        bcf     SYNCHWOUT
#endif
        btfsc   PIR1,RC1IF   ; interrupt uart1?  <------------- TODO
          bra   wUARTgo		 ;					  <------------ TODO

        btfss   SCS_TIMER_WRITE_IF     ; aspetta che siano trascorsi i uS previsti
          bra   wWaitLoop

;; 4.6 anti collisione: se SCS_INPUT vale 1 terminare in errore
        btfsc   SCS_INPUT
          bra   wCollisionError		   ; il bus ora è dominante - qualcun altro sta scrivendo

;; 4.6 anti collisione: se SCS_INPUT vale 1 terminare in errore
		btfsc	SCS_INTERRUPT_IF
          bra   wCollisionError		   ; il bus ora è dominante - qualcun altro sta scrivendo

#ifdef  SYNCHWOUT
        btfsc   SCS_INPUT
        bsf     SYNCHWOUT
        btfss   SCS_INPUT
        bcf     SYNCHWOUT
#endif


wNoCollision
                                       ; se ci sono ancora bit ricomincia da capo
        btfss   optionW,W_EOB ; se non ha finito gli 8 bit
          bra   wByteLoop
        btfsc   optionW,W_EXIT; se chiamata come routine
          return
                             ; fine byte
        incf    wByteCount,f
        decfsz  wMaxLen,f
           bra  wStreamLoop

        bsf     optionW,W_END ;; <---------------------------- fix -----------------

        movlw   0x00
        bra     Restore_Context

;; 4.6 anti collisione
wCollisionError
        bsf		optionW,W_ERROR        ; <========================================================
        bsf     optionW,W_END 
        movlw   0xFF
#ifdef  SYNCHWOUT
        btfsc   SCS_INPUT
        bsf     SYNCHWOUT
        btfss   SCS_INPUT
        bcf     SYNCHWOUT
#endif
#ifdef  SYNCHWERR
        bsf     SYNCHWERR
#endif
		bcf		SCS_INTERRUPT_IF
        bra     Restore_Context


wTimerInterrupt  ; interrupt scaduto
        bcf     SCS_TIMER_WRITE_IE
        bcf     SCS_TIMER_WRITE_IF     ; clr  timer  interrupt flag
        bsf     optionW,W_INTRP
        bcf     optionW,W_WAIT

        btfss   optionW,W_EOB          ; se ci sono ancora bit ricomincia da capo
          bra   wByteLoop
                             ; fine byte

;;        bra     scsInterruptTimerEnd  ; TEST - FERMIAMOCI A UN BAIT

        lfsr    FSR0,scsMessageTx      ; in FSR0 indirizzo tx buffer
        movf    ixTransmit,w
        addwf   FSR0L,f
        incf    wByteCount,f
        decfsz  wMaxLen,f
           bra  wStreamLoop

        bsf     optionW,W_END

wInterruptEnd
        btfss   SCS_TIMER_READ_IE;
          bra   scsInterruptTimerEnd
        btfsc   SCS_TIMER_READ_IF;
          bra   rTimerInterrupt      ; interrupt di lettura in corso
        bra     scsInterruptTimerEnd


#ifdef  TIMING_MEASURE
;--------------------------------------------------------------------------------------
scsTiming ;
;-----------------------------------------------------------------------------
        movf    BSR,w                  ;
        movlb   HIGH(wByte)            ;// PER INDIRIZZARE DATA BANK
        movwf   saveBSR
        movff   FSR1H,saveFSR1H        ;// salvataggio FSR1 - dopo il decremento
        movff   FSR1L,saveFSR1L
        movff   FSR0H,saveFSR0H        ;// salvataggio FSR0
        movff   FSR0L,saveFSR0L

        lfsr    0,scsTMR0L
        lfsr    1,scsTMR0H
        clrf    rByte
;       bcf     optionR,T_START
        bcf     optionR,T_NOW
        btfsc   SCS_INPUT
          bra   scsTimWaitL

scsTimWaitH
        clrwdt
;       bcf     optionR,T_START
        bsf     optionR,T_NOW
        btfss   SCS_INPUT
          bra   scsTimWaitH
        bra scsTimStart

scsTimWaitL
        clrwdt
;       bsf     optionR,T_START
        bcf     optionR,T_NOW
        btfsc   SCS_INPUT
          bra   scsTimWaitL

scsTimStart
       bsf      T0CON,TMR0ON       ;; partito !

scsTimContinue
       btfsc    optionR,T_NOW
         bra    scsTwaitL

scsTwaitH
        clrwdt
        bsf     optionR,T_NOW
        btfss   SCS_INPUT
          bra   scsTwaitH
        bra scsTok

scsTwaitL
        clrwdt
        bcf     optionR,T_NOW
        btfsc   SCS_INPUT
          bra   scsTwaitL

scsTok
        movff   TMR0L, POSTINC0
        movff   TMR0H, POSTINC1
        incfsz  rByte
          bra   scsTimContinue

        bcf     T0CON,TMR0ON

        movlw   0x00
        bra     Restore_Context
#endif

;--------------------------------------------------------------------------------------
scsPulse
;--------------------------------------------------------------------------------------
        movf    BSR,w                  ;
        movlb   HIGH(wByte)            ;// PER INDIRIZZARE DATA BANK
        movwf   saveBSR
        movff   FSR1H,saveFSR1H        ;// salvataggio FSR1 - dopo il decremento
        movff   FSR1L,saveFSR1L
        movff   FSR0H,saveFSR0H        ;// salvataggio FSR0
        movff   FSR0L,saveFSR0L

scsPulseMainloop
        movf    wPeriodoDomin+1,w     ;; FFFF - periodo dominante  [write timer]
        movwf   SCS_TIMER_WRITEH
        movf    wPeriodoDomin,w       ;; FFFF - periodo dominante  [write timer]
        movwf   SCS_TIMER_WRITEL
        bcf     SCS_TIMER_WRITE_IF     ; clr  timer  interrupt flag
        bcf     SCS_TIMER_WRITE_IE     ; clr  timer  interrupt flag

        bsf     SCS_OUT                ; inizio impulso SCS
; --------------------------------------------------------------------------------------------
scsPulseXloop
        btfss   SCS_TIMER_WRITE_IF     ; aspetta che siano trascorsi i uS previsti (1/3)
          bra   scsPulseXloop
        bcf     SCS_OUT                ; fine   impulso SCS

        movf    wPeriodoRece+1,w     ;; FFFF - periodo recessivo  [write timer]
        movwf   SCS_TIMER_WRITEH
        movf    wPeriodoRece,w       ;; FFFF - periodo recessivo  [write timer]
        movwf   SCS_TIMER_WRITEL
        bcf     SCS_TIMER_WRITE_IE
        bcf     SCS_TIMER_WRITE_IF     ; clr  timer  interrupt flag


scsPulseZloop
        btfss   SCS_TIMER_WRITE_IF     ; aspetta che siano trascorsi i uS previsti
          bra   scsPulseZloop
        clrwdt
        btfss   PIR3,RC2IF
          bra   scsPulseMainloop
        movlw   0x00
        bra     Restore_Context


Restore_Context
;    movwf    PRODL
;    clrf     PRODL+1
    movff    saveFSR0H,FSR0H
    movff    saveFSR0L,FSR0L
    movff    saveFSR1H,FSR1H
    movff    saveFSR1L,FSR1L
    movff    saveBSR,BSR
    return


;--------------------------------------------------------------------------------------
inUART 
;-----------------------------------------------------------------------------
        movf    BSR,w                  ;
		movlb   HIGH(uartMessage)      ;// PER INDIRIZZARE DATA BANK
        movwf   usaveBSR
        movff   FSR1H,usaveFSR1H        ;// salvataggio FSR1 - dopo il decremento
        movff   FSR1L,usaveFSR1L

#ifdef  UART_LED
        bsf     UART_LED
#endif
		btfss RCSTA1,OERR      ;se OERR oppure FERR   segnalare errore di lettura !!!!
		  bra inUARTok
        bcf  RCSTA1,CREN
        nop
        bsf  RCSTA1,CREN
        bsf  uState,U_OERR
inUARTok        
        btfss   PIR1,RC1IF   ; interrupt uart1?
			bra inUARTexit

        bcf   PIR1,RC1IF  

		btfsc RCSTA1,FERR    
          bsf  uState,U_FERR

        lfsr    1,uartMessage ;// pointer at start of buffers
        movf    uPtrW,w
        addwf   FSR1L,f     ;// pointer at start of uart buffer
        btfsc   STATUS,C
        incf    FSR1H,f     ;// pointer at start of uart buffer
		movf    RCREG1,w
        movwf   INDF1		;// update buffer contents

		incf    uMax,f
		btfss   uMax,6		;// se arriva a 0x80 torna a 0x7F (??)
		  bra   inUARTend
		decf    uMax,f

inUARTend
		incf    uPtrW,f
		btfsc   uPtrW,6		;// se arriva a 0x80 riparte da 0
		  clrf    uPtrW
; ------------------------------------------------------------------------------------------------------------
inUARTexit
		movff    usaveFSR1H,FSR1H
		movff    usaveFSR1L,FSR1L
		movff    usaveBSR,BSR
		return
		
    end
