// os.c
// Runs on LM4F120/TM4C123/MSP432
// A priority/blocking real-time operating system 
// Lab 4 starter file.
// Daniel Valvano
// March 25, 2016
// Hint: Copy solutions from Lab 3 into Lab 4
#include <stdint.h>
#include "os.h"
#include "../inc/CortexM.h"
#include "../inc/BSP.h"
#include "../inc/tm4c123gh6pm.h"

// function definitions in osasm.s
void StartOS(void);

#define NUMTHREADS  8        // maximum number of threads
#define NUMPERIODIC 2        // maximum number of periodic threads
#define STACKSIZE   100      // number of 32-bit words in stack per thread

struct tcb{
  int32_t *sp;		// pointer to stack (valid for threads not running
  struct tcb *next;	// linked-list pointer
  int32_t *blocked;	// nonzero if blocked on this semaphore
  uint32_t sleep;	// nonzero if this thread is sleeping
  uint32_t priority;	// lower value is higher priority
};

typedef struct tcb tcbType;
tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];
void static runperiodicevents(void);

// ******** OS_Init ************
// Initialize operating system, disable interrupts
// Initialize OS controlled I/O: periodic interrupt, bus clock as fast as possible
// Initialize OS global variables
// Inputs:  none
// Outputs: none
void OS_Init(void){
  DisableInterrupts();
  BSP_Clock_InitFastest();// set processor clock to fastest speed
// perform any initializations needed, 
// set up periodic timer to run runperiodicevents to implement sleeping
  BSP_PeriodicTask_Init(&runperiodicevents, 1000, 6);
}

void SetInitialStack(int i){
  Stacks[i][STACKSIZE-1] = 0x01000000;   // Thumb bit 
  Stacks[i][STACKSIZE-3] = 0x14141414;   // R14 
  Stacks[i][STACKSIZE-4] = 0x12121212;   // R12 
  Stacks[i][STACKSIZE-5] = 0x03030303;   // R3 
  Stacks[i][STACKSIZE-6] = 0x02020202;   // R2 
  Stacks[i][STACKSIZE-7] = 0x01010101;   // R1 
  Stacks[i][STACKSIZE-8] = 0x00000000;   // R0 
  Stacks[i][STACKSIZE-9] = 0x11111111;   // R11 
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10 
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9 
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8 
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7 
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6 
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5 
  Stacks[i][STACKSIZE-16] = 0x04040404;  // R4 
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer 
}

//******** OS_AddThreads ***************
// Add eight main threads to the scheduler
// Inputs: function pointers to eight void/void main threads
//         priorites for each main thread (0 highest)
// Outputs: 1 if successful, 0 if this thread can not be added
// This function will only be called once, after OS_Init and before OS_Launch
int OS_AddThreads(void(*thread0)(void), uint32_t p0,
                  void(*thread1)(void), uint32_t p1,
                  void(*thread2)(void), uint32_t p2,
                  void(*thread3)(void), uint32_t p3,
                  void(*thread4)(void), uint32_t p4,
                  void(*thread5)(void), uint32_t p5,
                  void(*thread6)(void), uint32_t p6,
                  void(*thread7)(void), uint32_t p7){
// **similar to Lab 3. initialize priority field****
  int32_t sr;

  sr = StartCritical();
  int i, j;
  void(*threads[NUMTHREADS])(void) = {
    thread0, thread1, thread2, thread3, thread4, thread5, thread6, thread7
  };
  uint32_t priorities[NUMTHREADS] = {
    p0 ,p1, p2, p3, p4, p5, p6, p7
  };

  // initialize TCB circular list (same as RTOS project)
  for (i = 0; i < NUMTHREADS; i++) {
    tcbs[i].next = &tcbs[(i+1)%NUMTHREADS];
  }
  // initialize TCB
  for (j = 0; j < NUMTHREADS; j++) {
    SetInitialStack(j);         // initialize stacks
    Stacks[j][STACKSIZE-2] = (int32_t) threads[j]; // initialize PC
    tcbs[j].blocked = 0;        // not blocked
    tcbs[j].sleep = 0;          // not sleeping
    tcbs[j].priority = priorities[j];	// initalize priority
  }

  RunPt = &tcbs[0];      // thread 0 is first to run
  EndCritical(sr);

  return 1;               // successful
}

void static decrement_sleep_timer(void) {
  int i;
  for (i = 0; i < NUMTHREADS; i++) {
    if (tcbs[i].sleep)
      tcbs[i].sleep--;
  }
}

void static runperiodicevents(void){
  decrement_sleep_timer(); 
// In Lab 4, handle periodic events in RealTimeEvents
  
}

//******** OS_Launch ***************
// Start the scheduler, enable interrupts
// Inputs: number of clock cycles for each time slice
// Outputs: none (does not return)
// Errors: theTimeSlice must be less than 16,777,216
void OS_Launch(uint32_t theTimeSlice){
  STCTRL = 0;                  // disable SysTick during setup
  STCURRENT = 0;               // any write to current clears it
  SYSPRI3 =(SYSPRI3&0x00FFFFFF)|0xE0000000; // priority 7
  STRELOAD = theTimeSlice - 1; // reload value
  STCTRL = 0x00000007;         // enable, core clock and interrupt arm
  StartOS();                   // start on the first task
}
// runs every ms
void Scheduler(void){      // every time slice
// look at all threads in TCB list choose
// highest priority thread not blocked and not sleeping 
// If there are multiple highest priority (not blocked, not sleeping) run these round robin
  uint32_t max;
  int higherPriority, notBlocked, notSleeping;
  tcbType *threadPt, *highestPt;

  max = 0xFFFFFFFF;
  threadPt = RunPt;
  do {
    threadPt = threadPt->next;
    higherPriority = (threadPt->priority < max);
    notBlocked = (threadPt->blocked == 0);
    notSleeping = (threadPt->sleep == 0);
    if (higherPriority && notBlocked && notSleeping) {
      highestPt = threadPt;
      max = threadPt->priority;
    }
  } while (threadPt != RunPt);
  
  RunPt = highestPt;
}

//******** OS_Suspend ***************
// Called by main thread to cooperatively suspend operation
// Inputs: none
// Outputs: none
// Will be run again depending on sleep/block status
void OS_Suspend(void){
  STCURRENT = 0;        // any write to current clears it
  INTCTRL = 0x04000000; // trigger SysTick
// next thread gets a full time slice
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
  RunPt->sleep = sleepTime;
  OS_Suspend();
}

// ******** OS_InitSemaphore ************
// Initialize counting semaphore
// Inputs:  pointer to a semaphore
//          initial value of semaphore
// Outputs: none
void OS_InitSemaphore(int32_t *semaPt, int32_t value){
  *semaPt = value;
}

// ******** OS_Wait ************
// Decrement semaphore and block if less than zero
// Lab2 spinlock (does not suspend while spinning)
// Lab3 block if less than zero
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Wait(int32_t *semaPt){
  uint32_t sr;

  sr = StartCritical();
  (*semaPt)--;
  if (*semaPt < 0) {    // block current thread
    RunPt->blocked = semaPt;
    EndCritical(sr);
    OS_Suspend();
    return;             // OS_Suspend() has triggered SysTick 
  }
  EndCritical(sr);
}

// ******** OS_Signal ************
// Increment semaphore
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Signal(int32_t *semaPt){
  uint32_t sr;
  tcbType *waitPt;

  sr = StartCritical();
  (*semaPt)++;
  if (*semaPt <= 0) {   // unblock a waiting thread
    waitPt = RunPt->next;
    while (waitPt->blocked != semaPt) {
      waitPt = waitPt->next;
    }
    waitPt->blocked = 0;
  }
  EndCritical(sr);
}

#define FSIZE 10    // can be any size
#define FIFOSUCCESS 0
#define FIFOFULL    -1
uint32_t PutI;      // index of where to put next
uint32_t GetI;      // index of where to get next
uint32_t Fifo[FSIZE];
int32_t CurrentSize;// 0 means FIFO empty, FSIZE means full
uint32_t LostData;  // number of lost pieces of data

// ******** OS_FIFO_Init ************
// Initialize FIFO.  The "put" and "get" indices initially
// are equal, which means that the FIFO is empty.  Also
// initialize semaphores to track properties of the FIFO
// such as size and busy status for Put and Get operations,
// which is important if there are multiple data producers
// or multiple data consumers.
// Inputs:  none
// Outputs: none
void OS_FIFO_Init(void){
  OS_InitSemaphore(&CurrentSize, 0);
  PutI = 0;
  GetI = 0;
  LostData = 0;
}

// ******** OS_FIFO_Put ************
// Put an entry in the FIFO.  Consider using a unique
// semaphore to wait on busy status if more than one thread
// is putting data into the FIFO and there is a chance that
// this function may interrupt itself.
// Inputs:  data to be stored
// Outputs: 0 if successful, -1 if the FIFO is full
int OS_FIFO_Put(uint32_t data){
  int status;

  if (CurrentSize == FSIZE) {
    LostData++;
    status = FIFOFULL;
  } else {
    Fifo[PutI] = data;
    PutI = (PutI + 1) % FSIZE;
    OS_Signal(&CurrentSize);
    status = FIFOSUCCESS;
  }

  return status;
}

// ******** OS_FIFO_Get ************
// Get an entry from the FIFO.  Consider using a unique
// semaphore to wait on busy status if more than one thread
// is getting data from the FIFO and there is a chance that
// this function may interrupt itself.
// Inputs:  none
// Outputs: data retrieved
uint32_t OS_FIFO_Get(void){uint32_t data;
  OS_Wait(&CurrentSize);    // block if empty
  data = Fifo[GetI];
  GetI = (GetI + 1) % FSIZE;

  return data;
}
// *****periodic events****************
int32_t *PeriodicSemaphore0;
uint32_t Period0; // time between signals
int32_t *PeriodicSemaphore1;
uint32_t Period1; // time between signals
void RealTimeEvents(void){int flag=0;
  static int32_t realCount = -10; // let all the threads execute once
  // Note to students: we had to let the system run for a time so all user threads ran at least one
  // before signalling the periodic tasks
  realCount++;
  if(realCount >= 0){
    if((realCount%Period0)==0){
      OS_Signal(PeriodicSemaphore0);
      flag = 1;
		}
    if((realCount%Period1)==0){
      OS_Signal(PeriodicSemaphore1);
      flag=1;
		}
    if(flag){
      OS_Suspend();
    }
  }
}
// ******** OS_PeriodTrigger0_Init ************
// Initialize periodic timer interrupt to signal 
// Inputs:  semaphore to signal
//          period in ms
// priority level at 0 (highest
// Outputs: none
void OS_PeriodTrigger0_Init(int32_t *semaPt, uint32_t period){
	PeriodicSemaphore0 = semaPt;
	Period0 = period;
	BSP_PeriodicTask_InitC(&RealTimeEvents,1000,0);
}
// ******** OS_PeriodTrigger1_Init ************
// Initialize periodic timer interrupt to signal 
// Inputs:  semaphore to signal
//          period in ms
// priority level at 0 (highest
// Outputs: none
void OS_PeriodTrigger1_Init(int32_t *semaPt, uint32_t period){
	PeriodicSemaphore1 = semaPt;
	Period1 = period;
	BSP_PeriodicTask_InitC(&RealTimeEvents,1000,0);
}

//****edge-triggered event************
int32_t *edgeSemaphore;
static const uint32_t BTN1_PIN = 0x00000040;
// ******** OS_EdgeTrigger_Init ************
// Initialize button1, PD6, to signal on a falling edge interrupt
// Inputs:  semaphore to signal
//          priority
// Outputs: none
void OS_EdgeTrigger_Init(int32_t *semaPt, uint8_t priority){
  edgeSemaphore = semaPt;
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;	// 1) activate clock for Port D
  // allow time for clock to stabilize
  while ((SYSCTL_RCGCGPIO_R&SYSCTL_RCGCGPIO_R3) == 0);
  GPIO_PORTD_CR_R |= BTN1_PIN;		// 2) no need to unlock PD6, allow configuration of PD6
  GPIO_PORTD_AMSEL_R &= ~BTN1_PIN;	// 3) disable analog on PD6
  GPIO_PORTD_PCTL_R &= ~0x0F000000;	// 4) configure PD6 as GPIO
  GPIO_PORTD_DIR_R &= ~BTN1_PIN;	// 5) make PD6 input
  GPIO_PORTD_AFSEL_R &= ~BTN1_PIN;	// 6) disable alt funct on PD6
  GPIO_PORTD_PUR_R &= ~BTN1_PIN;	// disable pull-up on PD6
  GPIO_PORTD_DEN_R |= BTN1_PIN;		// 7) enable digital I/O on PD6
  GPIO_PORTD_IS_R &= ~BTN1_PIN;		// (d) PD6 is edge-sensitive
  GPIO_PORTD_IBE_R &= ~BTN1_PIN;	//     PD6 is not both edges
  GPIO_PORTD_IEV_R &= ~BTN1_PIN;	//     PD6 is falling edge event
  GPIO_PORTD_ICR_R |= BTN1_PIN;		// (e) clear PD6 flag
  GPIO_PORTD_IM_R |= BTN1_PIN;		// (f) arm interrupt on PD6
  // set priority on Port D edge trigger bits 31-29
  NVIC_PRI0_R = (NVIC_PRI0_R&0x1FFFFFFF)|(priority << 29);
  NVIC_EN0_R |= 0x00000008;	// enable is bit 3 in NVIC_EN0_R
}

// ******** OS_EdgeTrigger_Restart ************
// restart button1 to signal on a falling edge interrupt
// rearm interrupt
// Inputs:  none
// Outputs: none
void OS_EdgeTrigger_Restart(void){
  GPIO_PORTD_IM_R |= BTN1_PIN;		// rearm interrupt 3 in NVIC
  GPIO_PORTD_ICR_R |= BTN1_PIN;		// clear flag6
}
void GPIOPortD_Handler(void){
  if (GPIO_PORTD_RIS_R & BTN1_PIN) {
    GPIO_PORTD_ICR_R |= BTN1_PIN;	// step 1 acknowledge by clearing flag
    OS_Signal(edgeSemaphore);		// step 2 signal semaphore (no need to run scheduler)
    GPIO_PORTD_IM_R &= ~BTN1_PIN;	// step 3 disarm interrupt to prevent bouncing to create multiple signals
  }
}


