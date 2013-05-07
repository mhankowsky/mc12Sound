/*
Serial Library
*/
#include <hidef.h>      /* common defines and macros */
#include <MC9S12C128.h>     /* derivative information */
#pragma LINK_INFO DERIVATIVE "mc9s12c128"


/* Initialize the PLL clk and modify oscillator clk speed */
void clockSetup(void);
unsigned char CRC(char * string, 
  unsigned char length, 
  unsigned char polynomial) ;
//void doSerialComm( void );
//void serialSetup(char baudIndex);
void serialSetup();
void transmit(char * message);