; ---------------------------------------------------------------------------------
;       File: canrepro_232.ASM
; ---------------------------------------------------------------------------------
; settings:  uart settings devono essere preimpostati
;; ---------------------------------------------------------------------------------
;		RIPROGRAMMAZIONE ZONA PROGRAMMI    0000 - EFFF
; ---------------------------------------------------------------------------------
; aggiornamento firmware pic18f26k80 - input da UART (1 o 2)
; va allocato in pic18f26k80 a 0xf000
; ---------------------------------------------------------------------------------

#ifndef _P18CXXX_H
#define _P18CXXX_H
         INCLUDE <p18CXXX.INC>
#endif

; ---------------------------------------------------------------------------------
 INCLUDE <canrepro.inc>
; ---------------------------------------------------------------------------------

#ifndef _USE_UART1 
#ifndef _USE_UART2 
	#define _USE_UART2  ; default UART2
#endif
#endif

        RADIX DEC

        global    REPROmainB
 IFDEF REPRO_SWAP
        extern    ReproEvoBase
 ENDIF
 IFDEF REPRO_DEBUG
        global    REPRO_TestReceiveMessages
        global    RxMessage

        global    HostAddress
        global    MyAddress
 ENDIF

; ---------------------------------------------------------------------------------
reproram     udata
; ---------------------------------------------------------------------------------
rxRC             res 1;
rxByte           res 1;
RxMessage        res 13;
TxMessage        res 13;
HostAddress      res 1;
MyAddress        res 1;
LastTXBufferUsed res 1;
TXHead           res 1;
FlashLength      res 1;
FlashAddress     res 2;
CheckWord        res 2;
CanCommand       res 1;  //x00: query version  (received from main proc!)
                      ;  //x01: new flash block binary mode - AA: address - L: data length (8-16-24-32-40-48-56-64)
                      ;  //x02: end flash block binary mode - CCKK: check-word (sum of values)
                      ;  //x03: end flash block binary mode - NO WRITE
                      ;  //x40: (virtual)    8 data bytes
                      ;  //x61: swap bank - branch to CANREPRO_232_EVB for flashing F000-FFFF addresses
                      ;  //x80: end repro - reboot
iTimeout         res 2;
FlashData        res 64;
FlashPtr         res 1;
ReproError       res 1;
i                res 1;
temp             res 1;

 IFDEF REPRO_DEBUG
saveFSR0H        res 1;
saveFSR0L        res 1;
saveFSR1H        res 1;
saveFSR1L        res 1;
saveFSR2H        res 1;
saveFSR2L        res 1;
iDebug           res 1;
 ENDIF
;--------------------------------------------------------------------------------------
REPRO_VERSION         EQU  0x1B
CMD_FIRMWARE          EQU  0x07
;--------------------------------------------------------------------------------------
; protocollo fisico
;  receive 0xll SS pp dd dd dd dd dd dd dd dd  [length, source, pdu format, data] ll=0 means no data
;  send    0xll DD pp dd dd dd dd dd dd dd dd  [length, destin, pdu format, data] ll=0 means no data
;--------------------------------------------------------------------------------------
; protocollo logico (dati)
;  receive      dato[0]    altro              len  send data
;      query     0x00                         0x02 0x00  0xVV (version number)
;
;      new block 0x01   AddrL   AddrH  lngt        none
;               //x01: new flash block binary mode - AA: address - L: data length (8-16-24-32-40-48-56-64)
;
;      end block 0x02   chkL    chkH          0x06 0x00       ok
;          write                                   0x01       protected area
;                                                  0xFE       checksum error
;                                                  0xFF       checksum error
;               //x02: end flash block binary mode - CCKK: check-word (sum of values)
;
;      end block 0x03   chkL    chkH          0x06 0x00       ok
;        nowrite
;
;      flsh writ 0x40   8 data bytes               none
;               //x40: flash data (0x40 is virtual code consecutive of 0x02 or 0x03 command)
;
;      bankswap  0x61                              none
;               //x61: bank swap
;
;      flsh end  0x80                              none
;               //x80: end - reboot
;
;--------------------------------------------------------------------------------------
#DEFINE RX_LENGTH             RxMessage
#DEFINE RX_SOURCEADDRESS      RxMessage+1
#DEFINE RX_PDUFORMAT          RxMessage+2
#DEFINE RX_DATA0              RxMessage+3
#DEFINE RX_DATA1              RxMessage+4
#DEFINE RX_DATA2              RxMessage+5
#DEFINE RX_DATA3              RxMessage+6
#DEFINE RX_DATA4              RxMessage+7
#DEFINE RX_DATA5              RxMessage+8
;--------------------------------------------------------------------------------------
#DEFINE TX_LENGTH             TxMessage
#DEFINE TX_PDUSPECIFIC        TxMessage+1
#DEFINE TX_PDUFORMAT          TxMessage+2
#DEFINE TX_DATA0              TxMessage+3
#DEFINE TX_DATA1              TxMessage+4
#DEFINE TX_DATA2              TxMessage+5
#DEFINE TX_DATA3              TxMessage+6
#DEFINE TX_DATA4              TxMessage+7
#DEFINE TX_DATA5              TxMessage+8
;--------------------------------------------------------------------------------------
BOOTb     code
;-----------------------------------------------------------------------------
REPROmainB
        movlb   HIGH(RxMessage)          ;// PER INDIRIZZARE DATA BANK
 IFDEF REPRO_DEBUG
        call    Save_Context
 ENDIF
        clrf    LastTXBufferUsed;
        clrf    INTCON;                   // disable all interrupts
        clrf    CanCommand;
        clrf    FlashLength;
        clrf    iTimeout;
        clrf    ReproError;
        clrf    INTCON;                   // disable all interrupts

; versione 1B - recupera HostAddress e MyAddress dallo stack di chiamata caricati con POSTINC1
;               da call C  REPROmain_232(HostAddress, MyAddress); (prima viene pushato MY e poi HOST)
; FSR1 dovrebbe puntare alla sommità dello stack software
        movff   POSTDEC1, HostAddress   ; dummy - only for decrement
        movff   POSTDEC1, HostAddress
        movff   POSTDEC1, MyAddress		; se entrambe le uart sono abilitate può valere 0x01 per uart1, altro per uart2

; // loop until command END-REPRO

;   while ( CanCommand != 0x80 )
ReproLoop
        clrwdt
;       switch (CanCommand)
        movf    CanCommand,w
        bz      ReproQuery         ; 0x00
        xorlw   0x80
        bz      ReproEnd           ; 0x80
        xorlw   0x80 ^ 0x01
        bz      ReproNewBlock      ; 0x01
        xorlw   0x01 ^ 0x02
        bz      ReproEndWrBlock    ; 0x02
        xorlw   0x02 ^ 0x03
        bz      ReproEndBlock      ; 0x03
        xorlw   0x03 ^ 0x40
        bz      ReproWriteBlock    ; 0x40
        xorlw   0x40 ^ 0x55
        bz      ReproEchoFlash     ; 0x55
        xorlw   0x55 ^ 0x61
        bz      ReproSwapBank      ; 0x61
        bra     ReproLoopEnd

;-----------------------------------------------------------------------------
ReproSwapBank                      ; 0x61
 IFDEF REPRO_SHIFT
        goto     ReproEvoBase
 ENDIF
;-----------------------------------------------------------------------------
ReproQuery                         ; 0x00
        movlw   2
        movwf   TX_LENGTH       ; txmessage.length
        clrf    TX_DATA0        ; txmessage.data[0]
        movlw   REPRO_VERSION;
        movwf   TxMessage+6     ; txmessage.data[1]
        call    REPRO_TransmitMessages
        bra     ReproLoopEnd

;-----------------------------------------------------------------------------
ReproNewBlock                      ; 0x01
;               //x01: new flash block binary mode - AA: address - L: data length (8-16-24-32-40-48-56-64)
        movff   RX_DATA1,FlashAddress        ; RxMessage.data[1]
        movff   RX_DATA2,FlashAddress+1      ; RxMessage.data[2]
        movff   RX_DATA3,FlashLength         ; RxMessage.data[3]
        clrf    FlashPtr
        clrf    CheckWord
        clrf    CheckWord+1
        lfsr    FSR0, FlashData        ; in fsr0 indirizzo flash buffer
        bra     ReproLoopEnd

;-----------------------------------------------------------------------------
ReproEndWrBlock                    ; 0x02
;;;;;   bcf     LATC,5          ;  LEDON
;               //x02: end flash block binary mode - CCKK: check-word (sum of values)

        clrf    TX_DATA1
        movff   RX_DATA1,TX_DATA2            ; CheckL previsto
        movff   RX_DATA2,TX_DATA3            ; CheckH previsto
        movff   CheckWord,TX_DATA4           ; CheckL calcolato
        movff   CheckWord+1,TX_DATA5         ; CheckH calcolato

        movf    RX_DATA1,w                   ; RxMessage-data[1]
        xorwf   CheckWord,w
        bnz     ReproNoCheck
        movf    RX_DATA2,w                   ; RxMessage-data[2]
        xorwf   CheckWord+1,w
        bnz     ReproNoCheck

        movf    FlashAddress+1,w             ; Flash Address HB

#ifdef __18F25K80
        andlw   0x78
        xorlw   0x70
#else
        andlw   0xF0        ; // proteggere da F0 a FF compresi
        xorlw   0xF0        ; // = F0: protetto      diverso: scrivibile
#endif

        bz      ReproProtect

        call    REPROwritePage
        bra     ReproEndReply

ReproNoCheck
;;;;;   bsf     LATC,5          ;  LEDOFF
;;      movff   RX_DATA0,TX_DATA0            ; TxMessage-data[0]

        movlw   0xFF
        infsnz  ReproError,f
        movlw   0xFE

        movwf   TX_DATA1                     ; TxMessage-data[1]
        bra     ReproEndReply

ReproProtect
        movlw   0x01
        movwf   TX_DATA1                     ; TxMessage-data[1]
ReproEndReply

;-----------------------------------------------------------------------------
ReproEndBlock						; 0x03
        movlw   6
        movwf   TX_LENGTH                    ; txmessage.length
        movff   RX_DATA0,TX_DATA0            ; txmessage.data[0]
        call    REPRO_TransmitMessages

        bra     ReproLoopEnd
;-----------------------------------------------------------------------------
ReproEchoFlash						; 0x55
        call    REPRO_TransmitFlash
        bra     ReproLoopEnd
;-----------------------------------------------------------------------------
ReproWriteBlock                     ; 0x40

;               //x40: flash data
        lfsr    FSR1, RX_DATA0         ; in fsr1 indirizzo Rx data
ReproWriteLoop
        movf    POSTINC1,w                  ; rxmessage.data[x]
        addwf   CheckWord,f
        bnc     ReproWriteLoop1
        incf    CheckWord+1,f
ReproWriteLoop1
        movwf   POSTINC0
        decf    FlashLength,f
        decf    RX_LENGTH,f      ; rxmessage.length
        bnz     ReproWriteLoop

;-----------------------------------------------------------------------------
ReproLoopEnd
;  // wait for next command
        clrwdt
        call    REPRO_ReceiveMessages
        xorlw   0
         bz     ReproLoopEnd1  ; retlw 0 : ok


        bra     ReproLoopEnd   ; per ora evitiamo il timeout

 ; * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

        incf    iTimeout,f
        bnz     ReproLoopEnd
        incf    iTimeout+1,f
        bnz     ReproLoopEnd

        clrf    FlashLength     ;     // timeout 4 mSec  x 15 = 60mSec
        movlw   0x03
        movwf   RX_DATA0        ; rxmessage.data[0]
        movlw   0xFF
        movwf   TX_DATA1        ; txmessage.data[1]
        movwf   TX_DATA2        ; txmessage.data[2]
        bra     ReproNoCheck

ReproLoopEnd1
        clrf    iTimeout
        clrf    iTimeout+1
        movf    FlashLength,f
        bz      ReproLoopNewCmd
        movlw   0x40
        movwf   CanCommand
        bra     ReproLoop
ReproLoopNewCmd
        movff   RX_DATA0,CanCommand         ; Rxmessage.data[0]
        bra     ReproLoop

ReproEnd
        reset

; ---------------------------------------------------------------------------------


; ---------------------------------------------------------------------------------
REPROwritePage
    btfsc  EECON1,WR                  ;// wait for any eeprom write to finish
      bra  REPROwritePage

;   // set table pointer to desired page address....
    clrf   TBLPTRU                       ; // set up table pointer register (upper byte)
    movff  FlashAddress+1,TBLPTRH        ; // set up table pointer register (high byte)
    movff  FlashAddress,  TBLPTRL        ; // set up table pointer register (low byte)

;   // erase page....
    bsf    EECON1,EEPGD ;                  // point to flash code space
    bcf    EECON1,CFGS ;                   // do not access config space
    bsf    EECON1,WREN ;                   // enable write to memory
    bsf    EECON1,FREE ;                   // enable page erase operation
    movlw  0x55  ;                         // write entry sequence step 1
    movwf  EECON2;                         // write entry sequence step 1
    movlw  0xAA  ;                         // write entry sequence step 2
    movwf  EECON2;                         // write entry sequence step 2
    bsf    EECON1,WR ;                     // erase page of code memory (CPU stalls until complete)
    nop  ;                                 // NOP instruction immediately after CPU stall

REPROwritePageW1
    btfsc  EECON1,WR                  ;// wait for any eeprom write to finish
      bra  REPROwritePageW1

;   // set up starting addresses (FSR0 & TBLPTR)....
    lfsr    FSR0, FlashData              ; in fsr0 indirizzo flash buffer
    TBLRD*-                              ; // dummy table read decrements TBLPTR
    movlw   32
    movwf   i

REPROwritePage1
;   // fill holding registers from data array (for 1st half-page write)....
        movf    POSTINC0,w               ; rxmessage.data[x]
        movwf   TABLAT                   ; // move byte from data array to TABLAT register
        TBLWT+*                          ; // write data from TABLAT into holding register
        decfsz  i,f
         bra    REPROwritePage1

;// write 1st half-page to flash....
    bsf    EECON1,EEPGD ;                  // point to flash code space
    bcf    EECON1,CFGS ;                   // do not access config space
    bsf    EECON1,WREN ;                   // enable write to memory
    movlw  0x55  ;                         // write entry sequence step 1
    movwf  EECON2;                         // write entry sequence step 1
    movlw  0xAA  ;                         // write entry sequence step 2
    movwf  EECON2;                         // write entry sequence step 2
    bsf    EECON1,WR ;                     // erase page of code memory (CPU stalls until complete)
    nop  ;                                 // NOP instruction immediately after CPU stall

REPROwritePageW2
    btfsc  EECON1,WR                  ;// wait for any eeprom write to finish
      bra  REPROwritePageW2

    movlw   32
    movwf   i

REPROwritePage2
;   // fill holding registers from data array (for 2st half-page write)....
        movf    POSTINC0,w                  ; rxmessage.data[x]
        movwf   TABLAT     ;                 // move byte from data array to TABLAT register
        TBLWT+*            ;                 // write data from TABLAT into holding register
        decfsz  i,f
         bra    REPROwritePage2

;   // write 2nd half-page to flash....
    bsf    EECON1,EEPGD ;                  // point to flash code space
    bcf    EECON1,CFGS ;                   // do not access config space
    bsf    EECON1,WREN ;                   // enable write to memory
    movlw  0x55  ;                         // write entry sequence step 1
    movwf  EECON2;                         // write entry sequence step 1
    movlw  0xAA  ;                         // write entry sequence step 2
    movwf  EECON2;                         // write entry sequence step 2
    bsf    EECON1,WR ;                     // erase page of code memory (CPU stalls until complete)
    nop  ;                                 // NOP instruction immediately after CPU stall

REPROwritePageW3
    btfsc  EECON1,WR                  ;// wait for any eeprom write to finish
      bra  REPROwritePageW3

    bcf    EECON1,WREN ;                   // disable write to memory
    return
; ---------------------------------------------------------------------------------
REPRO_ReceiveByte

#ifdef _USE_UART1
#ifdef _USE_UART2
							; se entrambe le uart sono abilitate può valere 0x01 per uart1, altro per uart2
    movlb   HIGH(MyAddress)          ;// PER INDIRIZZARE DATA BANK
	movlw	0x01
	cpfseq  MyAddress				 ; 0x01 = uart1
	   bra	REPRO_ReceiveByte2 		
#endif
#endif

#ifdef _USE_UART1
REPRO_ReceiveByte1 
    movlb   HIGH(RCSTA1)          ;// PER INDIRIZZARE DATA BANK
    btfss RCSTA1,OERR
      bra  Repro_ReceiveByte_1
    bcf RCSTA1,CREN
    bsf RCSTA1,CREN
Repro_ReceiveByte_1
    btfss PIR1,RC1IF
      bra  Repro_ReceiveByte_no
    bcf   PIR1,RC1IF
    movf  RCREG1,w

    movlb   HIGH(RxMessage)       ;// PER INDIRIZZARE DATA BANK
    movwf rxByte
    retlw  0x01         ; carattere letto
#endif

#ifdef _USE_UART2
REPRO_ReceiveByte2 
    movlb   HIGH(RCSTA2)          ;// PER INDIRIZZARE DATA BANK
    btfss RCSTA2,OERR
      bra  Repro_ReceiveByte_2
    bcf RCSTA2,CREN
    bsf RCSTA2,CREN
Repro_ReceiveByte_2
    btfss PIR3,RC2IF
      bra  Repro_ReceiveByte_no
    bcf   PIR3,RC2IF
    movf  RCREG2,w

    movlb   HIGH(RxMessage)       ;// PER INDIRIZZARE DATA BANK
    movwf rxByte
    retlw  0x01         ; carattere letto
#endif

    
Repro_ReceiveByte_no
    movlb  HIGH(RxMessage)       ;// PER INDIRIZZARE DATA BANK
    clrf   rxByte
    retlw  0x00     ; niente da leggere
; ---------------------------------------------------------------------------------
REPRO_ReceiveMessages
    call   REPRO_ReceiveByte
    xorlw  0
    bz     Repro_Receive_null
; ricevuto length
    bsf     LATA,3
    lfsr    FSR2,RxMessage        ; in fsr2 indirizzo Tx data
    movf    rxByte,w
    movwf   i
    incf    i                     ; incrementa lunghezza messaggio (+1)  include LL
    bra     REPRO_ReceiveMessages2

REPRO_ReceiveMessages1
    call    REPRO_ReceiveByte
    xorlw  0
    bz      REPRO_ReceiveMessages1
    movf    rxByte,w
REPRO_ReceiveMessages2
    movwf   POSTINC2
    decfsz  i,f
     bra    REPRO_ReceiveMessages1

    decf    RX_LENGTH,f      ; da lunghezza msg a lunghezza dati (-2)
    decf    RX_LENGTH,f      ; da lunghezza msg a lunghezza dati (-2)

    movf    RX_PDUFORMAT,w              ; PDUFormat
    xorlw   CMD_FIRMWARE
    bnz     Repro_Receive_false

;    movf    RX_PDUSPECIFIC,w            ; PDUSpecific (destination)
;    xorwf   MyAddress,w
;    bnz     Repro_Receive_false
Repro_Receive_true
    bcf     LATA,3
    retlw 0x00

Repro_Receive_false
    retlw 0x01

Repro_Receive_null
    retlw 0xFF
; ---------------------------------------------------------------------------------




; ---------------------------------------------------------------------------------
REPRO_TransmitByte


#ifdef _USE_UART1
#ifdef _USE_UART2
							; se entrambe le uart sono abilitate può valere 0x01 per uart1, altro per uart2
    movlb   HIGH(MyAddress)          ;// PER INDIRIZZARE DATA BANK

;	// movlw	0x01
;	// cpfseq	MyAddress				 ; 0x01 = uart1

    btfss   MyAddress,0	; test bit 0, skip if 1
	   bra	REPRO_TransmitByte2 
	   		
#endif
#endif

#ifdef _USE_UART1
REPRO_TransmitByte1
    movlb   HIGH(TXSTA1)          ;// PER INDIRIZZARE DATA BANK
    btfss   TXSTA1,TRMT
      bra   REPRO_TransmitByte
    movwf   TXREG1
    movlb   HIGH(TxMessage)          ;// PER INDIRIZZARE DATA BANK
    return
#endif

#ifdef _USE_UART2
REPRO_TransmitByte2
    movlb   HIGH(TXSTA2)          ;// PER INDIRIZZARE DATA BANK
    btfss   TXSTA2,TRMT
      bra   REPRO_TransmitByte
    movwf   TXREG2
    movlb   HIGH(TxMessage)          ;// PER INDIRIZZARE DATA BANK
    return
#endif

; ---------------------------------------------------------------------------------
REPRO_TransmitMessages

    movff   HostAddress,TX_PDUSPECIFIC ; destination
    movlw   CMD_FIRMWARE
    movwf   TX_PDUFORMAT            ; pduformat
;   movff   MyAddress,TX_SOURCEADDRESS ; sourceaddress
    incf    TX_LENGTH,f      ; da lunghezza dati a lunghezza msg (+2)
    incf    TX_LENGTH,f      ; da lunghezza dati a lunghezza msg (+2)
    movf    TX_LENGTH,w
    movwf   i
    incf    i,f
    lfsr    FSR2,TxMessage        ; in fsr2 indirizzo Tx data
Repro_TransmitMessages1
    movf    POSTINC2,w
    call    REPRO_TransmitByte
    decfsz  i
       bra  Repro_TransmitMessages1

    return
; ---------------------------------------------------------------------------------
REPRO_TransmitFlash
    movlb   HIGH(FlashData)       ;// PER INDIRIZZARE DATA BANK
	movlw   .64
    movwf   TX_LENGTH
	movwf	i
    call    REPRO_TransmitByte
    lfsr    FSR2,FlashData
Repro_TransmitFlash1
    movf    POSTINC2,w
    call    REPRO_TransmitByte
    decfsz  i
       bra  Repro_TransmitFlash1

    return
; ---------------------------------------------------------------------------------




 IFDEF REPRO_DEBUG
Save_Context
    movff    FSR0H,saveFSR0H
    movff    FSR0L,saveFSR0L
    movff    FSR1H,saveFSR1H
    movff    FSR1L,saveFSR1L
    movff    FSR2H,saveFSR2H
    movff    FSR2L,saveFSR2L
    return

Restore_Context
    movff    saveFSR0H,FSR0H
    movff    saveFSR0L,FSR0L
    movff    saveFSR1H,FSR1H
    movff    saveFSR1L,FSR1L
    movff    saveFSR2H,FSR2H
    movff    saveFSR2L,FSR2L
    return
; ---------------------------------------------------------------------------------
REPRO_TestReceiveMessages
    movlb   HIGH(RxMessage)          ;// PER INDIRIZZARE DATA BANK
    call    Save_Context
    call    REPRO_ReceiveMessages
    call    Restore_Context
    return

 ENDIF

 IFDEF L_OUT_SERIAL

;**************************************************************************************
; Standard serial transmit at 38400 BPS thru PGD  - bitlen 26uS
; 38400  is 52  cycles at 8 MHz
; 38400  is 208 cycles at 32MHz
;**************************************************************************************
serial_send                                             ; 38.4  mode
                movwf           Ser_OUT
                movlw           .9                      ; set up loop ctr
                movwf           BitCnt
                bra             Ser_clear               ; start with START bit

Ser_lp          rrcf            Ser_OUT,F               ; 49
                btfsc           STATUS,C                ; 50
                  bra           Ser_set                 ; 51..52
Ser_clear       bcf             OUT_SERIAL              ; 0
                bra             Ser_next                ; 1..2
Ser_set         bsf             OUT_SERIAL              ; 0
                bra             Ser_next                ; 1..2
Ser_next                                                ; 3
 IF FREQUENZA == .8000
                movlw           .12                     ; 3   (52-16 : 3= 12
 ENDIF
 IF FREQUENZA == .32000
                movlw           .63                     ; 3   (208-16 : 3= 64
 ENDIF
                call            Delay ; *3+5            ; 4..43
                nop                                     ; 44
                nop                                     ; 45
                decfsz          BitCnt,F                ; 46
                  bra           Ser_lp                  ; 47..48
                nop                                     ; 48
                nop                                     ; 49
                nop                                     ; 50
                nop                                     ; 51
                bsf             OUT_SERIAL              ; 0
                nop
 IF FREQUENZA == .8000
                movlw           .20                     ; 1 stopBIT
 ENDIF
 IF FREQUENZA == .32000
                movlw           .100                    ; 1 stopBIT
 ENDIF
                call            Delay ; *3+5            ; 2..26
                return
;**************************************************************************************
; Delay for 3n+5 (including call/return)
Delay           movwf           TMP
Delaylp         decfsz          TMP,F
                  bra           Delaylp
                return
;**************************************************************************************

 ENDIF

        end

