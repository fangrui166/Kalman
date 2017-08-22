#ifndef __GLOBAL_H__
#define __GLOBAL_H__
#include <stdint.h>



typedef union {
  uint8_t Byte;
  struct {
    uint8_t bit0        :1;
    uint8_t bit1        :1;
    uint8_t bit2        :1;
    uint8_t bit3        :1;
    uint8_t bit4        :1;
    uint8_t bit5        :1;
    uint8_t bit6        :1;
    uint8_t bit7        :1;
  } Bits;
} DBit;


extern volatile DBit            _BitParament0;
#define sys10msFlag             _BitParament0.Bits.bit0     /* 10ms flag      */
#define sys100msFlag            _BitParament0.Bits.bit1     /* 100ms flag     */
#define sys1minFlag            _BitParament0.Bits.bit2     /* 1min flag     */


extern volatile uint32_t        tickcount;        /* Timer tick count         */
extern volatile uint8_t         sysTickfor1ms;    /* 10millisecond timer      */
extern volatile uint8_t         sysTickfor10ms;   /* 10millisecond timer      */
extern volatile uint8_t         sysTickfor100ms;  /* 100millisecond timer     */
extern volatile uint32_t         sysTickfor1min;  /* 1min timer 	*/
#endif
