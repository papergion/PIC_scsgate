; ---------------------------------------------------------------------------------
#define BUFFER_READ        10
#define SCS_TIMER_WRITEH   TMR3H
#define SCS_TIMER_WRITEL   TMR3L
#define SCS_TIMER_WRITE_IF PIR2,TMR3IF
#define SCS_TIMER_WRITE_IE PIE2,TMR3IE
#define SCS_TIMER_READ     TMR0L
#define SCS_TIMER_READ_IF  INTCON,TMR0IF
#define SCS_TIMER_READ_IE  INTCON,TMR0IE
#define SCS_TIME_READ      104       ; 104 uS
#define SCS_TIME_WRITE     208       ; 104 uS - doppio perche' WR timing 0,5uS
#define SCS_TIMEOUT_BYTE   936       ;     uS timeout END OF BYTE (104*9)
#define SCS_TIMEOUT_STREAM 260       ;     uS timeout END OF MESSAGE (oltre la fine byte) FF - 260/4 + 1 = 191
#define SCS_INPUT          CMSTAT,CMP1OUT
#define SCS_OUT            LATC,RC4
#define SCS_LED            LATA,RA5
#define SCS_INTERRUPT_IF   PIR4,CMP1IF
#define SCS_INTERRUPT_IE   PIE4,CMP1IE
; ---------------------------------------------------------------------------------
