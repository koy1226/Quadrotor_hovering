#ifndef __RF24_CONFIG_H__
#define __RF24_CONFIG_H__
 
/*** USER DEFINES:  ***/
//#define FAILURE_HANDLING
//#define SERIAL_DEBUG
//#define MINIMAL
/**********************/
#define rf24_max(a,b) (a>b?a:b)
#define rf24_min(a,b) (a<b?a:b)
 
 
#include <mbed.h>
 
// RF modules support 10 Mhz SPI bus speed
const uint32_t RF_SPI_SPEED = 10000000;
 
#define HIGH 1
#define LOW 0
 
//#include <stdint.h>
//#include <stdio.h>
//#include <string.h>
 
#define _BV(x) (1<<(x))
#define _SPI SPI
 
#ifdef SERIAL_DEBUG
#define IF_SERIAL_DEBUG(x) ({x;})
#else
#define IF_SERIAL_DEBUG(x)
#endif
 
//#define printf_P printf
#define printf_P
//#define _BV(bit) (1<<(bit))
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
 
typedef uint16_t prog_uint16_t;
#define PSTR(x) (x)
  #define sprintf(...) os_sprintf( __VA_ARGS__ )
  //#define printf_P printf
  #define printf_P
  #define strlen_P strlen  
  #define PROGMEM
  #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
  #define pgm_read_word(p) (*(p))
  #define PRIPSTR "%s"
 
#endif // __RF24_CONFIG_H__

