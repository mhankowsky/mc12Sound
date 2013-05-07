
// ECE348 Spring 2013
// Michael Hankowsky
// Greg nazario
// G3B

/* Setup is as follows:
 * Analog input are:
   * Port AD 0 = microphone
   * Port AD 1 = potentiometer
 * Port A0 is clock for latch
 * LED Output is Port B
   * Port B 0-8 are LEDs
   * 
   *
   *
 *
 *
 *
 */


// Workload parameters

// Task period (in msec)
#define PERIOD0    50   // if you make this too fast the scheduler overhead gets out of hand (8 msec per run)
#define PERIOD1    100
#define PERIOD2    150
#define PERIOD3    200 

// Preemptive Utilization ~= 8/50 + 8/250 + 117/500 + 227/1000 + 450/5000 + 1100/10000 = .853 + other tasks + B

//*****************
//  Includes
//*****************

#include <hidef.h>      /* common defines and macros */
#include <mc9s12c128.h>     /* derivative information */
#pragma LINK_INFO DERIVATIVE "mc9s12c128"

#include <stdio.h>
#include "modclock.h"
#include "lcd_lib.h"
#include "serial.h"

//*****************
//  Typedefs
//*****************
typedef char bool;
#define TRUE  1
#define FALSE 0
#define ON    1
#define OFF   0 

typedef   signed char   int8;
typedef unsigned char  uint8;
typedef   signed int   int16;
typedef unsigned int  uint16;
typedef   signed long  int32;
typedef unsigned long uint32;

typedef void (*Ptr2Function)(void);

//***************************
//*  Function prototypes
//***************************
void SetLED(uint8 position, bool flag);  // CAUTION -- enables interrupts as side effect
uint32 TimeNow(void);                    // CAUTION -- enables interrupts as a side effect
uint32 TimeNowNoInterrupts(void);

void InitPorts(void);
void LaunchRequest(uint8 Task);         //Launches request for a Task  
void InitPCB(void);

void TaskScheduler(void);
void TaskCheck(void);    
void TaskSetLED(void);
void TaskSetLCD(void);

void main(void);

void WasteMsec(uint16 WaitTime);
void SetupTimer(void);
void interrupt 16 TimerHandler(void); 

uint8 AddrHi(Ptr2Function Ptr);
uint8 AddrLo(Ptr2Function Ptr);
void TaskTerminate(void);
void CreateLaunchStack(uint8 Task); 

//*****************************
//   Variables
//*****************************


char LCDvalue[50];
uint8 mic_in = 0;
uint8 pot_in = 0; 
uint32 alarmTime = 0;

#define setup 0
#define running 1
#define alarm 2
uint8 state;


//*****************************
//   Switch and LED interface
//*****************************

// Define switch and LED bit positions
#define  SW3_1  0x01
#define  SW3_2  0x02
#define  SW3_3  0x04
#define  SW3_4  0x08
#define  LED1   0x10
#define  LED2   0x20
#define  LED3   0x40
#define  LED4   0x80

// ----------

// Initialize Switches and LEDs attached to port B
//
void InitPorts(void) {
  // Set up port A as output
  DDRA = 0xFF;

  // Set up port B as output
  DDRB = 0xFF;  //All bits are outputs
  
  
  /*
  Configure Port AD as follows:
    AD0, AD1 as analog inputs
    AD2-7 as digital inputs
  Configure A/D converter as follows:
    Enable interrupt on conversion completion
    Conversion sequence covers channels 0 and 1
    Freeze mode behavior:  freeze after current conversion
    ATDclock frequency = 666,666 Hz
    ATD conversion is 8 bits
    ATD phase 2 sample time is 2 ATD clock periods
    Multichannel, continuous conversion, non-FIFO 
    Store result in unsigned, left justified format
  */
  
  // Analog input AD0 microphone AD1 potentiometer
  DDRAD = 0x00; //All input
  ATDCTL2 = ATDCTL2_ADPU_MASK | ATDCTL2_ASCIE_MASK;
  ATDCTL3 = ATDCTL3_FRZ1_MASK;
  
  ATDCTL4 = ATDCTL4_SRES8_MASK | ATDCTL4_PRS0_MASK | ATDCTL4_PRS2_MASK;
  ATDCTL5 = ATDCTL5_SCAN_MASK | ATDCTL5_MULT_MASK;
  ATDDIEN = 0xFC;
  
  
  ATDCTL3 = ATDCTL3 | ATDCTL3_S2C_MASK;
  
  
  //Configure the Watchdog Timer
  
  COPCTL = 0x05; // timer is 8MHz/2^22
  
  /* 
  configure the timer for:
      clock rate = 0.5 MHz
      reset timer on successful timer 7 output compare
      stop in freeze and wait modes   
  configure Timer 7 for the following:
      output compare with a default compare value of 0xF000
      enable interrupt on successful compare
  */
  /*TSCR1 = TSCR1_TEN_MASK | TSCR1_TSWAI_MASK | TSCR1_TSFRZ_MASK;
  TSCR2 = TSCR2_TCRE_MASK | TSCR2_PR2_MASK;
  TIOS = TIOS_IOS7_MASK;
  OC7M = OC7M_OC7M7_MASK;
  TC7 = 0xF000;
  
  TIE = 0xFF;
  */

  
}

// ----------

// This routine sets the LED outputs given an LED bit position and an On/Off flag
// It sets the LED and all others of a lower value
void SetLEDS(uint8 ledValue) { 
  uint8 LEDValue = 0xFF;    // inverted output values (1 = LED off); tracks PORTB contents
  int i;
  
  //clock down for latches
  PORTA = 0;
  
  //DisableInterrupts;    // Need atomic update to LEDValue 

  if(ledValue > 8) return;

  for(i=0; i<ledValue; i++){
      // turn LED on by zeroing appropriate bit
      LEDValue = LEDValue & ( (0x1<<i) ^ 0xFF);
  }
  
  //EnableInterrupts;    // End Atomic - LEDValue now set and flag can not be changed so we can be sure the value is correct  

  //output new value to LEDs, Clock up
  PORTB = LEDValue;
  sleepMS(5);
  PORTA = 0xFF;
}


//*****************************
//   Tasking Support
//*****************************

#define NTASKS  4     // total number of tasks in this workload, not counting main loop

uint8 CurrentTask = NTASKS;    // current task executing is initialized to be main loop

// Process control block for tracking task status
//
typedef struct PCBstruct { 
  Ptr2Function TaskPtr;  // points to start of task code
  bool   LaunchRequest;  // true when task is being requested by a switch
  bool   ReadyToLaunch;  // true when task is eligible to launched (i.e., called at initial entry point)
  bool   Running;        // true when task is actually running (post-launch)
  uint8 * SPsave;
  uint16 Period;         // period in msec
  uint32 NextTime;       // next start time in msec
} PCBstruct;

PCBstruct PCB[NTASKS+1];   // allocate one extra entry for main loop SP save

// Reserve a separate stack area for each task
//
#define STACKSIZE  64          
uint8 STACK[NTASKS][STACKSIZE]; 

// ----------

// Initialize PCB entries before starting tasker
//
void InitPCB(void) 
{
  // Initialize PCB entries for the tasks  (Note that PCB[NTASKS] doesn't need initialization)
  //   SPsave doesn't need to be initialized; that's done when setting up stack upon task launch
  
  uint32 TempTime; // store time to avoid time changing between uses

  // Set up task pointers and initial run status
  PCB[0].TaskPtr = &TaskScheduler;  PCB[0].LaunchRequest = TRUE;  // always run scheduler
  PCB[1].TaskPtr = &TaskCheck;      PCB[1].LaunchRequest = TRUE;  // always run switch checker
  PCB[2].TaskPtr = &TaskSetLED;     PCB[2].LaunchRequest = FALSE;  // other tasks off by default until switches are set
  PCB[3].TaskPtr = &TaskSetLCD;     PCB[3].LaunchRequest = FALSE; 
  
  // Set up task periods
  //    Period is in msec -- what period actually does varies depending on scheduling technique
  PCB[0].Period = PERIOD0; 
  PCB[1].Period = PERIOD1; 
  PCB[2].Period = PERIOD2; 
  PCB[3].Period = PERIOD3; 
  
  // Set task running status flags
  PCB[0].ReadyToLaunch = TRUE;     PCB[0].Running = FALSE;   // Always launch scheduler
  PCB[1].ReadyToLaunch = FALSE;    PCB[1].Running = FALSE;   
  PCB[2].ReadyToLaunch = FALSE;    PCB[2].Running = FALSE;   
  PCB[3].ReadyToLaunch = FALSE;    PCB[3].Running = FALSE;   
  
  // Set all NextTime values with same current time to initialize task timing
  TempTime = 0;    
  PCB[0].NextTime = 0;
  PCB[1].NextTime = 0;
  PCB[2].NextTime = 0;
  PCB[3].NextTime = 0;
}
    
// --------------------

// Helper functions to extract low and high byte of function addresses to save on return stack
uint8 AddrHi(Ptr2Function Ptr) { 
  return ( ((uint16)(Ptr)>>8) & 0xFF); 
}

uint8 AddrLo(Ptr2Function Ptr) { 
  return ( (uint8)(Ptr) & 0xFF ); 
}

// Call when task terminates after running
void TaskTerminate(void) {
  PCB[CurrentTask].Running = FALSE;
  for(;;){} // Loop forever
}

// Create a task launch image onto the task's stack
void CreateLaunchStack(uint8 Task) {
  static uint8 StackIdx; 
  
  // Set up fresh stack image for executing the newly launched task
  // Current stack already has RTI information for restarting old task; don't worry about it
  StackIdx = STACKSIZE;
  STACK[Task][--StackIdx] = AddrLo(&TaskTerminate);   // return to termination routine when completed
  STACK[Task][--StackIdx] = AddrHi(&TaskTerminate);  

  STACK[Task][--StackIdx] = AddrLo(PCB[Task].TaskPtr); // Taskptr
  STACK[Task][--StackIdx] = AddrHi(PCB[Task].TaskPtr);
  
  STACK[Task][--StackIdx] = AddrLo(0x00);  //Reg Y Lo
  STACK[Task][--StackIdx] = AddrHi(0x00);  //Reg Y Hi
  
  STACK[Task][--StackIdx] = AddrLo(0x00);  //Reg X
  STACK[Task][--StackIdx] = AddrHi(0x00);  //Reg X
  
  STACK[Task][--StackIdx] = AddrLo(0x00);  //Reg D
  STACK[Task][--StackIdx] = AddrHi(0x00);  //Reg D
  
  STACK[Task][--StackIdx] = AddrLo(0x00);  //CCR
  // -----------------------------------------------------------
     
  // Tag task for launch by putting info into the task's PCB entry
  //
  PCB[Task].SPsave = &STACK[Task][StackIdx];  // points to newly created stack data
  PCB[Task].ReadyToLaunch = FALSE;            // we just launched it; don't relaunch right away
  PCB[Task].Running = TRUE;                   // setting this TRUE tells tasker it is fair game to run

  // That's it; the task will start running at its initial entry point as soon as the tasker sees it
}



//*****************************
//   Scheduler
//*****************************
                                      
// Scheduler task
// Executing this task examines all the ReadyToRun tasks and launches those whose period has elapsed
//
void TaskScheduler(void) 
{
 
 uint8 ThisTask;   

 uint32 TempTime;   
 DisableInterrupts;
 TempTime = TimeNowNoInterrupts();

   // We don't protect access to the PCB since this is the highest priority task that should be messing with it
   //    except for the tasker.  The tasker will ignore the PCB until the Running flag is set, so do that last   
   // For all tasks, launch task if time has come
   for (ThisTask = 1; ThisTask < NTASKS;  ThisTask++) 
   { 
     if( PCB[ThisTask].LaunchRequest && !PCB[ThisTask].Running &&                   
        !PCB[ThisTask].ReadyToLaunch &&  PCB[ThisTask].NextTime <= TempTime     
       ) 
     { 
       PCB[ThisTask].ReadyToLaunch = TRUE; 
       PCB[ThisTask].NextTime += PCB[ThisTask].Period;

       CreateLaunchStack(ThisTask);   // sets Running flag
     }
     
    //Kick the Watchdog Timer!
    ARMCOP = 0x55;
    ARMCOP = 0xAA; 
   }
   EnableInterrupts;
}

//********************************************
//   Check switches to see which tasks to run
//********************************************


// Set LaunchRequest accordingly
//
void LaunchRequest(uint8 Task) 
{ 
  // Only if new request
  if(PCB[Task].LaunchRequest == FALSE)   
  { 
    PCB[Task].LaunchRequest = TRUE;      
    // Assert a request for the task
    
    if (TimeNow() >= PCB[Task].NextTime + PCB[Task].Period)  
    {  
       PCB[Task].NextTime = TimeNow();   
    }
  }
}

// ----------

// Task to read switches and set requests for task launch based on those values
//
void TaskCheck(void) 
{ 
  // For each task, make ready to run according to corresponding switch
  //   Note that each task always runs to completion (subject to preemption) once restarted
  //   But each task isn't re-launched unless switch stays ON
  LaunchRequest(2);
  LaunchRequest(3);
}
 
//********************************************
//   Workload tasks
//********************************************

void TaskSetLCD(void) {
  int err;
  DisableInterrupts;
  err = sprintf(LCDvalue, "THR:0x%x", pot_in);
  lcdWriteLine(1, LCDvalue);
  
  err = sprintf(LCDvalue, "Cur:0x%x", mic_in);
  lcdWriteLine(2, LCDvalue);
  EnableInterrupts; 
}

void TaskSetLED(void) {
 static uint8 alarm_leds = 8;
 uint8 saved_mic;
 uint8 leds_on;
 uint8 threshold;
 bool  alarm_flag = FALSE;
 
 //Grab the values from A/D -- Make sure they do not change in middle
 DisableInterrupts;
 saved_mic = mic_in;
 threshold = pot_in; 
 
 if (state == alarm) {
   alarm_flag = TRUE;
 }
 EnableInterrupts;
 
 
 if (alarm_flag) {
   alarm_leds = (alarm_leds == 0) ? 8 : 0;
   leds_on = alarm_leds;
   transmit("ALARM");
 } else {
   // To deal with negative and positive values  
   if (saved_mic < 0x60) {
     saved_mic = (0x60 - saved_mic);
   } else {
     saved_mic = (saved_mic - 0x60);
   }
   
   leds_on = (saved_mic >> 3);
   leds_on = (leds_on > 8) ? 8 : leds_on;
   
           /*
   
   if (threshold < 0x80)
    threshold = 0xFF - threshold * 2;
   else
    threshold = 2 * (threshold - 0x80);
   
   threshold = threshold/8;     
   
   // Prevent dividing by 0
   if (threshold == 0)
    threshold = 1;
   
   leds_on = leds_on%threshold;   */
 }
 
 SetLEDS(leds_on); 
}



//***************************************************************************
//        Main Function, which is the scheduler for non-preemptive tasks
//***************************************************************************

void main(void) {
  state = setup;
  // Perform setup tasks
  InitPorts();     //Set up all ports
  clockSetup();       // run module at 8 MHz
  SetupTimer();       // init time of day ISR and variables
  InitPCB();
  lcdSetup();           //Set up LCD for writing 
  serialSetup();
  CurrentTask = NTASKS; // current task is main loop
  state = running;
  EnableInterrupts;     // this starts the tasker

  //executed forever
  for(;;) {
  }

}
//*****************************************************
//     Timer Task & ISR;   includes Preemptive Tasker
//*****************************************************

uint32 TimeInMsec;   // 32 bit integer msec of current time since system boot
                     //    CAUTION -- this code doesn't handle rollover!
uint32 CurrentTime;  // 8.24 fixed point clock ticks; integer portion is in msec

// ----------

// Return current time value
//
uint32 TimeNow(void) 
{ 
  uint32 TempTime;
  DisableInterrupts;
  TempTime = TimeInMsec;
  EnableInterrupts;
  
  return(TempTime);  
}

// Return current time value
//
uint32 TimeNowNoInterrupts(void) 
{ 
  uint32 TempTime;
  TempTime = TimeInMsec;
  
  return(TempTime);  
}


// Timer setup
// Call once at start of program to initialize counter/timer and time variables
//
void SetupTimer( void ) {

 // Initialize timer variables
 TimeInMsec = 0;
 CurrentTime = 0;

 // set TN = 1   Timer Enable   TSCR1 bit 7
 TSCR1 |= 0x80;
 
 // set  PR[2:0]  Timer prescale in bottom 3 bits of TSCR2
 TSCR2 = (TSCR2 & 0x78) | 0x80 | 0x00;   // enable TOF interrupt; set 0x07  bus clock / 1
 // Timer interrupt enabled at this point

}


// ----------
 
// Timer ISR
//
//Increment the timer counter and update current time in msec when it increments
//
#define TIMEINCR 0x083126E9   // 137438953  => just over 8 msec per TCNT rollover   8 MHz  divider of 1

void interrupt 16 TimerHandler(void) 
{ 
   static uint8 * SPtemp;  // makes it easier to in-line assembly to save SP value
   static uint8 i;  // temp var, but make it static to avoid stack complications

   TFLG2 = 0x80;  // Clear TOF; acknowledge interrupt
  
   CurrentTime += TIMEINCR;  // Increment time using 8.24 fixed point  (i.e., integer scaled to 2**-24 secs)
   TimeInMsec += (CurrentTime >> 24) & 0xFF;  // add integer portion to msec time count
   CurrentTime &= 0x00FFFFFF;                 // strip off integer portion, since that went into TimeInMsec

   if (state == alarm) {
     if (TimeInMsec > (alarmTime + 2000)) { 
       state = running;
     }
   }
   


   // This is the tasker in preemptive mode; switch to highest priority running task
   
   // Save current SP
   { asm    STS  SPtemp;  }
   PCB[CurrentTask].SPsave = SPtemp;

   // Set SP to main loop stack during scheduling to avoid crash when
   //         scheduling Task 0
   SPtemp = PCB[NTASKS].SPsave;
   { asm    LDS  SPtemp; }

   
   // Check to see if it is time to run the scheduler   (who schedules the scheduler?)
   //    we could check all the tasks this way, but we want to keep this ISR as fast as possible, so don't do that
   if(    !PCB[0].Running                   // it's not already running
       &&  PCB[0].NextTime <= TimeInMsec    // and it's been a period since last launch
       ) 
     { 
       PCB[0].NextTime += PCB[0].Period;
       if(PCB[0].NextTime <= TimeInMsec)
       { 
        PCB[0].NextTime = TimeInMsec + PCB[0].Period;  // Catch up if behind
       }

       // Create a launch image on the scheduler's stack; we're going to go to it at the next ISR below
       CreateLaunchStack(0);            // Note: double-launching is OK; if we missed a period no point running it twice

     } 

  for (i = NTASKS; i>0; i--)  // iterate enough times to check all tasks
  { 
    CurrentTask++;   // advance to next task in round robin order
    if (CurrentTask >= NTASKS) { CurrentTask = 0; }   // Wrap around 
    if (PCB[CurrentTask].Running) { break; } // found next active task
  }  // end EITHER when we've found a running task or have checked all tasks

  if (!PCB[CurrentTask].Running) 
  { 
    CurrentTask = NTASKS;  // if no tasks are running, default to main loop
  }  

   
  // -------------------------------------------------------------------

  // Have found the highest priority running task OR have defaulted to main loop task
  //    (this is the only place we use the main loop PCB entry, but it saves having special code for that case
  SPtemp = PCB[CurrentTask].SPsave;
   
  // Restore SP for this newly selected task (or same task that was just interrupted, depending on situation)
  { asm    LDS  SPtemp; }
}

/*
ATDInterrupt:
*/
void interrupt 22 ATDInterrupt( void ) {

  mic_in = ATDDR0H;
  pot_in = ATDDR1H;
  
  
  if (mic_in > pot_in) {
    state = alarm; 
    alarmTime = TimeNowNoInterrupts();
  }
   
  ATDSTAT0 = ATDSTAT0_SCF_MASK | ATDSTAT0;
}

/*
timer7Interrupt
*/
void interrupt 15 timer7Interrupt(void) { 
  

 /* chase_counter = (chase_counter+1)%8;
  
  if ((PTAD & 0x4)) {
    PTT = (PTT& ~0x1C) | (chase_counter<<2);
  }
  
  
   */ 
   TFLG1 = TFLG1_C7F_MASK;
}

// -----------
