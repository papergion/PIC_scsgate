/*********************************************************************
 *
 *                  Generic Type Definitions
 *
 *********************************************************************
 * FileName:        GenericTypeDefs.h
 * Dependencies:        None
 * Processor:       PIC18, PIC24F, PIC24H, dsPIC30F, dsPIC33F, PIC32
 * Complier:        Microchip C18, C30, C32
 *                                      HI-TECH PICC-18
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the "Company") is intended and supplied to you, the Company's
 * customer, for use solely and exclusively with products manufactured
 * by the Company.
 *
 * The software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *
 * Author                                       Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Nilesh Rajbharti                     07/12/04        Rel 0.9
 * Nilesh Rajbharti                     11/24/04        Rel 0.9.1
 * Rawin Rojvanit                       09/17/05        Rel 0.9.2
 * D Flowers & H Schlunder      08/10/06        Much better now (1.0)
 * D Flowers & H Schlunder      09/11/06        Add base signed types (1.1)
 * D Flo, H Sch, et. al         02/28/07        Add QWORD, LONGLONG, QWORD_VAL (1.2)
 * Bud Caldwell             02/06/08    Added def's for PIC32
 ********************************************************************/

#ifndef __GENERIC_TYPE_DEFS_H_
#define __GENERIC_TYPE_DEFS_H_

typedef enum _BOOL { FALSE = 0, TRUE } BOOL;    // Undefined size

#ifndef NULL
#define NULL    0  //((void *)0)
#endif

#define PUBLIC                                  // Function attributes
#define PROTECTED
#define PRIVATE   static

typedef unsigned char           BYTE;                           // 8-bit unsigned
//pedef unsigned short int      WORD;                           // 16-bit unsigned
typedef unsigned int            WORD;                           // 16-bit unsigned
typedef unsigned long           DWORD;                          // 32-bit unsigned
typedef unsigned long long      QWORD;                          // 64-bit unsigned
typedef signed char             CHAR;                           // 8-bit signed
typedef signed short int        SHORT;                          // 16-bit signed
typedef signed long             LONG;                           // 32-bit signed
typedef signed long long        LONGLONG;                       // 64-bit signed

#if defined(__18CXX)                                   // my addon
typedef unsigned short long  LWORD;   // 24-bit        // my addon
//      #error "LWORD unsigned short long"
#endif                                                 // my addon
                                                       // my addon
#if defined(__dsPIC33F__) || defined (__PIC24H__)      // my addon
typedef unsigned       long  LWORD;   // 32-bit        // my addon
//      #error "LWORD unsigned long"
#endif                                                 // my addon

/* Alternate definitions */
typedef void                VOID;

typedef char                CHAR8;
typedef unsigned char       UCHAR8;

/* Processor & Compiler independent, size specific definitions */
// To Do:  We need to verify the sizes on each compiler.  These
//         may be compiler specific, we should either move them
//         to "compiler.h" or #ifdef them for compiler type.
typedef signed int          INT;
typedef signed char         INT8;
typedef signed short int    INT16;
typedef signed long int     INT32;
typedef signed long long    INT64;

typedef unsigned int        UINT;
typedef unsigned char       UINT8;
typedef unsigned short int  UINT16;
typedef unsigned long int   UINT32;  // other name for 32-bit integer
typedef unsigned long long  UINT64;

typedef union _BYTE_VAL
{
    BYTE Val;
    struct                                             // my addon
    {                                                  // my addon
        unsigned char LS:4;                            // my addon
        unsigned char MS:4;                            // my addon
    } half;                                            // my addon
    struct
    {
        unsigned char b0:1;
        unsigned char b1:1;
        unsigned char b2:1;
        unsigned char b3:1;
        unsigned char b4:1;
        unsigned char b5:1;
        unsigned char b6:1;
        unsigned char b7:1;
    } bits;
} BYTE_VAL, BYTE_BITS;

typedef union _WORD_VAL
{
    SHORT sVal;
    WORD  Val;
    BYTE v[2];
    struct
    {
        BYTE LB;
        BYTE HB;
    } byte;
    struct
    {
        unsigned char f1:4;
        unsigned char f2:4;
        unsigned char f3:4;
        unsigned char f4:4;
    } semibyte;
    struct
    {
        unsigned char b0:1;
        unsigned char b1:1;
        unsigned char b2:1;
        unsigned char b3:1;
        unsigned char b4:1;
        unsigned char b5:1;
        unsigned char b6:1;
        unsigned char b7:1;
        unsigned char b8:1;
        unsigned char b9:1;
        unsigned char b10:1;
        unsigned char b11:1;
        unsigned char b12:1;
        unsigned char b13:1;
        unsigned char b14:1;
        unsigned char b15:1;
    } bits;
} WORD_VAL, WORD_BITS;
typedef union _SHORT_VAL
{
    int  Val;
    BYTE v[2];
    struct
    {
        BYTE LB;
        BYTE HB;
    } byte;
    struct
    {
        unsigned char f1:4;
        unsigned char f2:4;
        unsigned char f3:4;
        unsigned char f4:4;
    } semibyte;
    struct
    {
        unsigned char b0:1;
        unsigned char b1:1;
        unsigned char b2:1;
        unsigned char b3:1;
        unsigned char b4:1;
        unsigned char b5:1;
        unsigned char b6:1;
        unsigned char b7:1;
        unsigned char b8:1;
        unsigned char b9:1;
        unsigned char b10:1;
        unsigned char b11:1;
        unsigned char b12:1;
        unsigned char b13:1;
        unsigned char b14:1;
        unsigned char b15:1;
    } bits;
} SHORT_VAL;

typedef union _DWORD_VAL
{
    LONG  sVal;
    DWORD Val;
    WORD w[2];
    int  i[2];
    BYTE v[4];
    struct
    {
        WORD LW;
        WORD HW;
    } word;
    struct
    {
        BYTE LB;
        BYTE HB;
        BYTE UB;
        BYTE MB;
    } byte;
    struct
    {
        WORD_VAL low;
        WORD_VAL high;
    }wordUnion;
    struct
    {
        unsigned char b0:1;
        unsigned char b1:1;
        unsigned char b2:1;
        unsigned char b3:1;
        unsigned char b4:1;
        unsigned char b5:1;
        unsigned char b6:1;
        unsigned char b7:1;
        unsigned char b8:1;
        unsigned char b9:1;
        unsigned char b10:1;
        unsigned char b11:1;
        unsigned char b12:1;
        unsigned char b13:1;
        unsigned char b14:1;
        unsigned char b15:1;
        unsigned char b16:1;
        unsigned char b17:1;
        unsigned char b18:1;
        unsigned char b19:1;
        unsigned char b20:1;
        unsigned char b21:1;
        unsigned char b22:1;
        unsigned char b23:1;
        unsigned char b24:1;
        unsigned char b25:1;
        unsigned char b26:1;
        unsigned char b27:1;
        unsigned char b28:1;
        unsigned char b29:1;
        unsigned char b30:1;
        unsigned char b31:1;
    } bits;
} DWORD_VAL;

#define LSB(a)          ((a).v[0])
#define MSB(a)          ((a).v[1])

#define LOWER_LSB(a)    ((a).v[0])
#define LOWER_MSB(a)    ((a).v[1])
#define UPPER_LSB(a)    ((a).v[2])
#define UPPER_MSB(a)    ((a).v[3])

typedef union _QWORD_VAL
{
        QWORD Val;
        DWORD d[2];
        WORD w[4];
    BYTE v[8];
    struct
    {
        DWORD LD;
        DWORD HD;
    } dword;
    struct
    {
        WORD LW;
        WORD HW;
        WORD UW;
        WORD MW;
    } word;
    struct
    {
        unsigned char b0:1;
        unsigned char b1:1;
        unsigned char b2:1;
        unsigned char b3:1;
        unsigned char b4:1;
        unsigned char b5:1;
        unsigned char b6:1;
        unsigned char b7:1;
        unsigned char b8:1;
        unsigned char b9:1;
        unsigned char b10:1;
        unsigned char b11:1;
        unsigned char b12:1;
        unsigned char b13:1;
        unsigned char b14:1;
        unsigned char b15:1;
        unsigned char b16:1;
        unsigned char b17:1;
        unsigned char b18:1;
        unsigned char b19:1;
        unsigned char b20:1;
        unsigned char b21:1;
        unsigned char b22:1;
        unsigned char b23:1;
        unsigned char b24:1;
        unsigned char b25:1;
        unsigned char b26:1;
        unsigned char b27:1;
        unsigned char b28:1;
        unsigned char b29:1;
        unsigned char b30:1;
        unsigned char b31:1;
        unsigned char b32:1;
        unsigned char b33:1;
        unsigned char b34:1;
        unsigned char b35:1;
        unsigned char b36:1;
        unsigned char b37:1;
        unsigned char b38:1;
        unsigned char b39:1;
        unsigned char b40:1;
        unsigned char b41:1;
        unsigned char b42:1;
        unsigned char b43:1;
        unsigned char b44:1;
        unsigned char b45:1;
        unsigned char b46:1;
        unsigned char b47:1;
        unsigned char b48:1;
        unsigned char b49:1;
        unsigned char b50:1;
        unsigned char b51:1;
        unsigned char b52:1;
        unsigned char b53:1;
        unsigned char b54:1;
        unsigned char b55:1;
        unsigned char b56:1;
        unsigned char b57:1;
        unsigned char b58:1;
        unsigned char b59:1;
        unsigned char b60:1;
        unsigned char b61:1;
        unsigned char b62:1;
        unsigned char b63:1;
    } bits;
} QWORD_VAL;

#if defined(__18CXX)               // my addon
typedef union _LWORD_VAL           // my addon
{                                  // my addon
    LWORD Val;                     // my addon
    BYTE v[3];                     // my addon
    WORD w;                        // my addon
    struct                         // my addon
    {                              // my addon
        BYTE LB;                   // my addon
        BYTE HB;                   // my addon
        BYTE UB;                   // my addon
    } byte;                        // my addon
    struct                         // my addon
    {                              // my addon
        unsigned char b0:1;        // my addon
        unsigned char b1:1;        // my addon
        unsigned char b2:1;        // my addon
        unsigned char b3:1;        // my addon
        unsigned char b4:1;        // my addon
        unsigned char b5:1;        // my addon
        unsigned char b6:1;        // my addon
        unsigned char b7:1;        // my addon
        unsigned char b8:1;        // my addon
        unsigned char b9:1;        // my addon
        unsigned char b10:1;       // my addon
        unsigned char b11:1;       // my addon
        unsigned char b12:1;       // my addon
        unsigned char b13:1;       // my addon
        unsigned char b14:1;       // my addon
        unsigned char b15:1;       // my addon
        unsigned char b16:1;       // my addon
        unsigned char b17:1;       // my addon
        unsigned char b18:1;       // my addon
        unsigned char b19:1;       // my addon
        unsigned char b20:1;       // my addon
        unsigned char b21:1;       // my addon
        unsigned char b22:1;       // my addon
        unsigned char b23:1;       // my addon
    } bits;                        // my addon
} LWORD_VAL;                       // my addon
#endif                             // my addon


#if defined(__dsPIC33F__) || defined(__PIC24H__)
typedef union _LWORD_VAL                          // my addon
{                                  // my addon
    LWORD Val;                     // my addon
    BYTE v[4];                     // my addon
    WORD w;                        // my addon
    struct                         // my addon
    {                              // my addon
        BYTE LB;                   // my addon
        BYTE HB;                   // my addon
        BYTE UB;                   // my addon
        BYTE WB;                   // my addon
    } byte;                        // my addon
    struct                         // my addon
    {                              // my addon
        unsigned char b0:1;        // my addon
        unsigned char b1:1;        // my addon
        unsigned char b2:1;        // my addon
        unsigned char b3:1;        // my addon
        unsigned char b4:1;        // my addon
        unsigned char b5:1;        // my addon
        unsigned char b6:1;        // my addon
        unsigned char b7:1;        // my addon
        unsigned char b8:1;        // my addon
        unsigned char b9:1;        // my addon
        unsigned char b10:1;       // my addon
        unsigned char b11:1;       // my addon
        unsigned char b12:1;       // my addon
        unsigned char b13:1;       // my addon
        unsigned char b14:1;       // my addon
        unsigned char b15:1;       // my addon
        unsigned char b16:1;       // my addon
        unsigned char b17:1;       // my addon
        unsigned char b18:1;       // my addon
        unsigned char b19:1;       // my addon
        unsigned char b20:1;       // my addon
        unsigned char b21:1;       // my addon
        unsigned char b22:1;       // my addon
        unsigned char b23:1;       // my addon
        unsigned char b24:1;       // my addon
        unsigned char b25:1;       // my addon
        unsigned char b26:1;       // my addon
        unsigned char b27:1;       // my addon
        unsigned char b28:1;       // my addon
        unsigned char b29:1;       // my addon
        unsigned char b30:1;       // my addon
        unsigned char b31:1;       // my addon
    } bits;                        // my addon
} LWORD_VAL;                       // my addon
#endif                             // my addon


#endif //__GENERIC_TYPE_DEFS_H_
