#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */



void main(void) {
  /* put your own code here */
  


	EnableInterrupts;


  for(;;) {
    _FEED_COP(); /* feeds the dog */
  } /* loop forever */
  /* please make sure that you never leave main */
}
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
unsigned char CRC(char * string, 
                  unsigned char length, 
                  unsigned char polynomial) ;
void doSerialComm( void );
void serialSetup(char baudIndex);

//globals

char groupString[] = "3B";
char msgBuf[17];
int baudRate = 0;
volatile TDRE = 0;
volatile TC   = 0;

void main(void) {

  //char oldPTAD;
  
  EnableInterrupts;

  //set up I/O
  //TODO:  set up port A as output
  DDRA = 0xFF;
  
  
  //TODO:  set up port AD is input
  
  DDRAD = 0x00;
  PERAD = 0x00;
  ATDDIEN = 0xFF;

  //call clock setup to set module bus clock to 8 MHz
  clockSetup();

  //call serial setup to configure control registers  
  serialSetup(5);//0x0F & (PTAD >> 4));

  //call to set up lcd for output
  lcdSetup();

  //call to initiate serial communciation
  doSerialComm();
  
  for(;;) {
  } /* wait forever */
  /* please make sure that you never leave this function */
}


/*
    This function initilizes the SCI control registers.  Baud rate
    is determined by looking up the baudIndex parameter in a switch statement.
*/
void serialSetup(char baudIndex) {
  //volatile int SCICR1;
  //volatile int SCICR2;
  
  //set up serial comm

  //Use a switch statement to select the baud rate based on the baudIndex.  
  //       The case values should correspond to the table below:
  //case 0 - 300 baud
  //case 1 - 600 baud
  //case 2 - 1200 baud
  //case 3 - 2400 baud
  //case 4 - 4800 baud
  //case 5 - 9600 baud
  //case 6 - 14400 baud
  //case 7 - 19200 baud
  //case 8 - 38400 baud
  //case 9 - 56000 baud
  //case 10 - 115200 baud
  //default - 2400 buad
  
  switch(baudIndex){
    case 0:
      SCIBDH = 0x16;
      SCIBDL = 0x67;
      baudRate = 300;
      break;
    case 1:
      SCIBDH = 0x8;
      SCIBDL = 0x33;
      baudRate = 600;
      break;
    case 2:
      SCIBDH = 0x4;
      SCIBDL = 0x16;
      baudRate = 1200;
      break;  
    case 3:
      SCIBDH = 0x2;
      SCIBDL = 0x08;
      baudRate = 2400;
      break;  
    case 4:
      SCIBDH = 0x1;
      SCIBDL = 0x04;
      baudRate = 4800;
      break;
    case 5:
      SCIBDH = 0x00;
      SCIBDL = 0x52;
      baudRate = 9600;
      break;
    case 6:
      SCIBDH = 0x00;
      SCIBDL = 0x34;
      baudRate = 14400;
      break;  
    case 7:
      SCIBDH = 0x00;
      SCIBDL = 0x26;
      baudRate = 19200;
      break;  
    case 8:
      SCIBDH = 0x00;
      SCIBDL = 0x13;
      baudRate = 38400;
      break;
    case 9:
      SCIBDH = 0x00;
      SCIBDL = 0x08;
      baudRate = 56000;
      break; 
    case 10:
      SCIBDH = 0x00;
      SCIBDL = 0x04;
      baudRate = 11520;
      break;
    default:
      SCIBDH = 0x2;
      SCIBDL = 0x08;
      baudRate = 2400;
      break; 
  }
         
  //configure SCICR1 and SCICR2 for 8 bit, one stop bit, no parity
  
  SCICR1 = 0x00;
  //SCICR2 = 0x0C;
}

void doSerialComm( void ) {

  //Declare variables as needed
  char query[QUERY_MAX_LEN+1];  //string to hold query value;
  char response[RESPONSE_MAX_LEN+1]; //string to hold response value
  char i;        //counter
  //char crcRx;    //received CRC value
  char crcComp;  //computed CRC value
  int errFlag;   //to hold return values
  char recGroup[3]; //string to create response group
  char * string; // pointer to hold return values and remove warnings
  int temp = 0;
  
  /***********TRANSMISSION PREP PORTION**************/
  
  //compute CRC of groupString
  crcComp = CRC(groupString,2,POLY);
  
  //display group string, the CRC, and the current baud rate on the LCD:
  //hint, use sprintf with "%s 0x%02X BR=%d" as your control string
   errFlag = sprintf(msgBuf, "%s 0x%02X BR=%d", groupString, crcComp, baudRate); 
   lcdSimpleWrite(msgBuf);

  //wait for button press
  //wait for the user to press PB1 to initiate transmission
  //for(;;) {
  //  if(~(PTAD) & 0x1) break;
  //}


  /****************TRANSMISSION PORTION *************/
  //Construct the query by concatentating groupString:crcComp:NULL
  //  Note:  NULL denoted a null byte, not the string "NULL" 
  
  string = strcpy(query, groupString);
  temp = strlen(query);
  query[temp] = crcComp;
  query[temp+1] = '\0';

  // Transmit each byte of the query, including the NULL terminator
  //       hint:  After sending each byte, you should monitor the status
  //              register to see when the transmitter is ready for the next
  //              value.
  
  SCICR2 = 0x08;
  for(i = 0; i < strlen(query); i++){
   SCIDRL = query[i];
   for(;;) {
      TDRE = (0x01 & (SCISR1 >> 7));
      //TC   = (0x01 & (SCISR1 >> 6));
      if(TDRE){
        break;
      }
    }
    
  }    
  
  
 
  /*************RECEPTION PORTION*****************/

  //TODO:  Receive bytes and copy them into 'response' string until
  //       the null character is received or the RESPONSE_MAX_LEN
  //       is reached. 
  //Hint:  You must monitor the status register to see when a value
  //       has been received into the data register.

  for(i=0; i<16; i++) {
    for(;;){
      if(0x01 & (SCISR1>>5)){
        response[i] = SCIDRL;
        break;
      }
    }
    if(SCISR1==0x00){
      SCIDRL = 0x00;
      break;
    } else SCIDRL = 0x00;
  }
        


  //TODO:  CRC check
  //       Extract the CRC from the received data and replace the CRC with a NULL byte.
  //       Compute the CRC of the received message.
  //       Compare the received and computed CRC values and display an error if they don't match  
  
  if(response[3] != crcComp){
    errFlag = -1;
  }
    
  //TODO Format the response and the crc and display the values on the LCD
  //Hint:  Use sprintf with the control string "%s 0x%x".
  
  string = strcpy(recGroup, response);
  errFlag = sprintf(msgBuf, "%s 0x%x", recGroup, baudRate); 
  lcdSimpleWrite(msgBuf);
  
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