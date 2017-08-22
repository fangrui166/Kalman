#ifndef __INTEGER_H
#define __INTEGER_H

/* These types must be 16-bit, 32-bit or larger integer */
typedef int		INT;
typedef unsigned int	UINT;

#define u8    uint8_t
#define u16   uint16_t
#define u32   uint32_t

/* Boolean type */
#include <stdbool.h>
#define BOOL INT
#ifndef bool
#define bool BOOL
#endif

#if 0
/* These types must be 8-bit integer */
typedef char	        CHAR;
typedef unsigned char	UCHAR;
typedef unsigned char	BYTE;
typedef unsigned char   *PBYTE;
#define int8_t CHAR
#define uint8_t UCHAR

/* These types must be 16-bit integer */
typedef short		SHORT;
typedef unsigned short	USHORT;
typedef unsigned short	WORD;
typedef unsigned short	WCHAR;
#define int16_t SHORT
#define uint16_t USHORT

/* These types must be 32-bit integer */
typedef long		LONG;
typedef unsigned long	ULONG;
typedef unsigned long	DWORD;
typedef unsigned long   *PDWORD;
#define int32_t LONG
#define uint32_t ULONG
#define time_t LONG

/* Misc */
typedef void            *HANDLE;
#endif

#define TRUE     (1)
#define FALSE    (0)

#ifndef false
#define false FALSE
#define true TRUE
#endif

#endif /* __INTEGER_H */

