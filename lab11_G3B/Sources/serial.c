/*
  Lab 06 code skeleton
  file:  lab_6_skeleton.c
  author:  Justin Ray
  Revision History:
    9-21-2009 - Updated to include modclock.c/h and lab renumbering.
  
  This file contains a starting point for lab 6.
  
  make the following connections on the board for this lab:
  connect SW1_1 to PAD04
  connect SW1_2 to PAD05
  connect SW1_3 to PAD06
  connect SW1_4 to PAD07
  
  connect PB1 to PAD00
  
  The PAD4:7 bits represent the index for the baud rate lookup
  The PAD0 bit represent the push button (for activating communication).
  

*/
#include "serial.h"
#include <hidef.h>         /* common defines and macros */
#include <mc9s12c128.h>     /* derivative information */
#pragma LINK_INFO DERIVATIVE "mc9s12c128"

//user includes
#include "lcd_lib.h"
#include "modclock.h"
#include <stdio.h>

//defines
#define RESPONSE_MAX_LEN 16
#define QUERY_MAX_LEN 16
#define POLY 0xEA

//function prototypes


//globals

char groupString[] = "3B";
char msgBuf[17];
int baudRate = 0;
volatile TDRE = 0;
volatile TC   = 0;


void serialSetup() {
  SCIBDH = 0x00;
  SCIBDL = 0x34;
  //baudRate = 9600;
  //configure SCICR1 and SCICR2 for 8 bit, one stop bit, no parity
  
  SCICR1 = 0x00;
  SCICR2 = 0x0C;
}

void transmit( char * message ) {

  //Declare variables as needed
  char query[QUERY_MAX_LEN+1];  //string to hold query value;
  char i;        //counter
  char crcComp;  //computed CRC value
  char * string; // pointer to hold return values and remove warnings
  int temp = 0;
  
  /***********TRANSMISSION PREP PORTION**************/
  
  //compute CRC of groupString
  crcComp = CRC(message, 2, POLY);
  /****************TRANSMISSION PORTION *************/
  //Construct the query by concatentating groupString:crcComp:NULL
  //  Note:  NULL denoted a null byte, not the string "NULL" 
  
  string = strcpy(query, message);
  temp = strlen(query);
  query[temp] = crcComp;
  query[temp+1] = '\0';

  // Transmit each byte of the query, including the NULL terminator
  //       hint:  After sending each byte, you should monitor the status
  //              register to see when the transmitter is ready for the next
  //              value.
  
  
  for(i = 0; i < strlen(query) + 1; i++){
   for(;;) {
      TDRE = (0x01 & (SCISR1 >> 7));
      if(TDRE){
        SCIDRL = query[i];
        break;
      }
    }
    
  }    
}

/*
  CRC -
  this function computes the CRC of the first <length> bytes at the location <string> using
  <polynomial> for a polynomial.  The return value is the computed CRC value (FCS).
*/
unsigned char CRC(char * string, 
                  unsigned char length, 
                  unsigned char polynomial) 
{
  unsigned char crc;
  unsigned char i,j;
 
  crc = 0;
  for (i=0;i<length;i++) {
    crc ^= string[i];
    for (j=0;j<8;j++) {
      if (0x80 & crc) {
        crc = (crc << 1) ^ polynomial;
      } else {
        crc = crc << 1;
      }
    }
  }
  return crc;
  
}
