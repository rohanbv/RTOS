// RTOS Framework - Fall 2020
// J Losh

// Student Name: Rohan B Venkatesh
// TO DO: Add your name on this line.  Do not include your ID number in the file.

// Add xx_ prefix to all files in your project
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// xx_other files
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDS on these pins:
// Blue:   PF2 (on-board)
// Red:    PE1
// Orange: PE2
// Yellow: PE3
// Green:  PE4
// PBs on these pins
// PB0:    PA2
// PB1:    PA3
// PB2:    PA4
// PB3:    PA5
// PB4:    PA6
// PB5:    PA7
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "wait.h"

//#define DEBUG
//#define DEBUG_TASK_SWITCH

#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) // off-board red LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) // off-board green LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) // off-board yellow LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) // off-board orange LED

#define PUSH_BUTTON_0   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) //off-board pushbutton 0
#define PUSH_BUTTON_1   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) //off-board pushbutton 1
#define PUSH_BUTTON_2   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) //off-board pushbutton 2
#define PUSH_BUTTON_3   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4))) //off-board pushbutton 3
#define PUSH_BUTTON_4   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4))) //off-board pushbutton 4
#define PUSH_BUTTON_5   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4))) //off-board pushbutton 5

#define BLUE_LED_MASK   (1<<2)
#define RED_LED_MASK    (1<<1)
#define GREEN_LED_MASK  (1<<4)
#define ORANGE_LED_MASK (1<<2)
#define YELLOW_LED_MASK (1<<3)

#define PUSH_BUTTON_0_MASK  (1<<2)
#define PUSH_BUTTON_1_MASK  (1<<3)
#define PUSH_BUTTON_2_MASK  (1<<4)
#define PUSH_BUTTON_3_MASK  (1<<5)
#define PUSH_BUTTON_4_MASK  (1<<6)
#define PUSH_BUTTON_5_MASK  (1<<7)

#define SYSTICK_TIM_CLK 40000000
#define EXEC_RETURN     0xFFFFFFFD
#define INITIAL_xPSR    0x01000000

#define MAX_PRIORITY_LEVEL 16
#define CREATED_SEMAPHORES  4
#define TASKS_CREATED 10

enum SERVICE
{
            YIELD = 10,
            SLEEP = 20,
            WAIT  = 30,
            POST  = 40,
            REBOOT = 50,
            RUN   = 51,
            KILL  = 52,
            PS  = 53,
            IPCS = 54,
            PIDOF = 55,
            RESTART = 56,
            SETPRIORITY = 57,
            SCHED = 60,
            PI    = 61,
            PREEMPT = 62
};

enum SCHEDULING
{
            PRIORITY = 1,
            RR = 2
};

enum SWITCH
{
            ON = 1,
            OFF = 2
};

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
typedef struct _semaphore
{
    uint16_t count;
    uint16_t queueSize;
    char     name[16];
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphore;

semaphore semaphores[MAX_SEMAPHORES];

uint8_t semaphoreCount = 0;

#define keyPressed  0
#define keyReleased  1
#define flashReq  2
#define resource  3


enum SCHEDULING schedulingMode = PRIORITY;
enum SWITCH pInherit = OFF;
enum SWITCH preemptSched = ON;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

uint32_t  perfBuffer = 0;
uint32_t Ti[2][MAX_TASKS];
uint32_t sysTickCount = 1000;

//Used GCC attributes for ease of portability and readability and did static allocation from heap
//reffered from Stack overflow and GCC documentation
uint32_t stack[MAX_TASKS][512] __attribute__((aligned (2048))) __attribute__((location(0x20001000)));
// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // location of original stack pointer
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // 0=highest to 15=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];                  // One Delta time for buffer,one continous recording

//Memory Table in Shared Memory Space for Kernal and Task For Getters
struct _piCpuUsage
{
    uint32_t intUsage;
    uint32_t fracUsage;
} cpuUsage[MAX_TASKS]__attribute__((location(0x20007700)));
struct _tcb psTcb[MAX_TASKS]__attribute__((location(0x20007200)));
semaphore ipcsSemaphores[MAX_SEMAPHORES]__attribute__((location(0x20007500)));

//-----------------------------------------------------------------------------
// Functions Prototypes
//-----------------------------------------------------------------------------

//Assembly  Functions
extern uint32_t getMSP(void);
extern uint32_t getPSP(void);
extern void setPSP(uint32_t);
extern void setMSP(uint32_t);
extern void changeSPtoPSP(void);
extern uint8_t getSVCNumber(void);
extern void change_Privilege(void);

//Added Functions Prototypes
void startSysTick(uint32_t tick_hz);
void run(char*);
void sched(bool prio);
void pi(bool pi);
void preempt(bool prempt);
void initMpu(void);
void callIpcs(semaphore* shellSemaphore,uint8_t i);

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
}

// REQUIRED: Implement prioritization to 16 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    uint8_t priTask;
    uint8_t i;
    uint8_t priorityLevel = 0;
    if(schedulingMode == PRIORITY)
    {
        while (!ok)
        {
            //Fetch Current Task
            priTask = taskCurrent;
            //If priority level
            if(priorityLevel >= MAX_PRIORITY_LEVEL)
                priorityLevel = 0;
            //loop MAX_TASKS times to check for lowest priority READY task
            for(i = 1;i <= MAX_TASKS;i++)
            {
                //increment on every loop
                priTask = priTask + 1;
                if(priTask >= MAX_TASKS)
                    priTask = 0;
                 if(tcb[priTask].priority == priorityLevel)
                 {
                     ok = (tcb[priTask].state == STATE_READY || tcb[priTask].state == STATE_UNRUN);
                     if(ok)
                     {
                         return priTask;
                     }
                 }
            }
            priorityLevel++;
        }
    }
    else
    {
        while (!ok)
        {
            task++;
            if (task >= MAX_TASKS)
                task = 0;
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }
        return task;
    }
    return -1;
 }



bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0,j = 0;
    bool found = false;
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            j = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            tcb[i].spInit = &stack[i][512];
            tcb[i].sp = tcb[i].spInit;
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            while(name[j] != '\0')
            {
                psTcb[i].name[j] = name[j];
                tcb[i].name[j] = name[j];
                j++;
            }
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    return ok;
}

void restartThread(_fn fn)
{
    __asm(" SVC #56");
}

void destroyThread(_fn fn)
{
    __asm(" SVC #52");
}

void setThreadPriority(_fn fn, uint8_t priority)
{
    __asm(" SVC #57");
}

bool createSemaphore(uint8_t count,uint8_t s,const char name[])
{
    uint8_t j = 0;
    bool ok = (semaphoreCount < MAX_SEMAPHORES);
    if (ok)
    {
        while(name[j] != '\0')
        {
            semaphores[s].name[j] = name[j];
            j++;
        }
        semaphores[s].count = count;
    }
    return ok;
}

void startRtos()
{
    taskCurrent = rtosScheduler();
    tcb[taskCurrent].state = STATE_READY;
    setPSP((uint32_t)tcb[taskCurrent].spInit);
    changeSPtoPSP();
    _fn currentProc = (_fn)tcb[taskCurrent].pid;
    tcb[taskCurrent].state = STATE_READY;
    //start sysTick at 1Khz rate
    startSysTick(1000);
    initMpu();
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    change_Privilege();
    currentProc();
}

void yield()
{
    __asm(" SVC #10 ");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    __asm(" SVC #20 ");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(int8_t semaphore)
{
    __asm(" SVC #30");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int8_t semaphore)
{
    __asm(" SVC #40 ");
}

void systickIsr()
{
    int i = 0,j = 0;
    //Support for sleep command
    for(i = 0; i < MAX_TASKS; i++)
    {
        if(tcb[i].state == STATE_DELAYED)
        {
            if(tcb[i].ticks == 0)
            {
                tcb[i].state = STATE_READY;
            }
            tcb[i].ticks--;
        }
    }

    //Support for Copying Process times to Swap data Buffer
    sysTickCount--;
    if(sysTickCount == 0)
    {
        perfBuffer ^= 1;
        sysTickCount = 1000;
        for(j = 0;j < MAX_TASKS;j++)
        {
            Ti[perfBuffer][j] = 0;
        }
    }

    if(preemptSched == ON)
    {
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    }
}

void pendSvIsr()
{
       int T;
       //Store/PUSH the state of the process and update and maintain Process SP in TCB
       __asm volatile(" MRS R0,PSP ");
       __asm volatile(" STMDB R0!,{R4-R11} ");
       __asm volatile(" MSR PSP,R0");
       tcb[taskCurrent].sp = (uint32_t *)getPSP();
       //Store the timer difference into tasks Delta Time
       TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
       T = (0xFFFFFFFF - TIMER1_TAV_R);
       Ti[perfBuffer][taskCurrent] += T;
       //Check for Next Task
       taskCurrent = rtosScheduler();

       TIMER1_TAV_R = 0xFFFFFFFF;
       TIMER1_CTL_R |= TIMER_CTL_TAEN;

       if(tcb[taskCurrent].state == STATE_READY)
       {
           //Load/POP State of the Next Process to be scheduled from TCB
           setPSP((uint32_t)tcb[taskCurrent].sp);
           __asm volatile(" MRS R0,PSP ");
           __asm volatile(" LDMIA R0!,{R4-R11}");
           __asm volatile(" MSR PSP,R0 ");
       }
       else
       {
           //If Task has never been run,initialize a dummy Stack frame to be popped from.
           uint8_t reg=0;
           uint32_t *pPSP = (uint32_t*)tcb[taskCurrent].sp;
           tcb[taskCurrent].state = STATE_READY;
           pPSP--;
           *pPSP = INITIAL_xPSR;
           pPSP--;
           *pPSP = (uint32_t)tcb[taskCurrent].pid;
           pPSP--;
           *pPSP = EXEC_RETURN;
           for(reg = 0; reg < 5 ; reg++)
           {
               pPSP--;
               *pPSP = 0;
           }
           tcb[taskCurrent].sp = (uint32_t*)pPSP;
           setPSP((uint32_t)tcb[taskCurrent].sp);
           setPSP((uint32_t)pPSP);
       }
}

void svCallIsr()
{
   //Check service that triggered SVCIsr
   uint8_t svcNumber = getSVCNumber();
   uint32_t *pPSP = (uint32_t*)getPSP();
   uint8_t i = 0;
   uint16_t j = 0,k = 0;
   semaphore *tasksSemaphore;
   bool found;
   char* reqRunTask;
   char str[20];
   _fn killPid,restartFn;
   //Ps related variable
   uint64_t taskTime,presentTick;
   uint32_t TotalCpu = 0;
   uint32_t R0 = *pPSP;
   pPSP++;
   uint32_t R1 = *pPSP;
   #ifdef DEBUG
   uint32_t R1 = *pPSP;
   pPSP++;
   uint32_t R2 = *pPSP;
   pPSP++;
   uint32_t R3 = *pPSP;
   pPSP++;
   uint32_t R12 = *pPSP;
   pPSP++;
#endif
   //Service Particular ISR
   switch(svcNumber)
   {

   case(YIELD) :
       NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
   break;

   case(SLEEP) :
       tcb[taskCurrent].ticks = R0;
       tcb[taskCurrent].state = STATE_DELAYED;
       NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
   break;

   case(WAIT) :
        if(semaphores[R0].count > 0)
        {
            semaphores[R0].count--;
            tcb[taskCurrent].semaphore = &semaphores[R0];
        }
        else
        {
            tcb[taskCurrent].state = STATE_BLOCKED;
            tcb[taskCurrent].semaphore = &semaphores[R0];
            semaphores[R0].processQueue[semaphores[R0].queueSize] = taskCurrent;
            semaphores[R0].queueSize++;

            if(pInherit == ON)
            {
                if(tcb[taskCurrent].state == STATE_BLOCKED)
                {
                    for(i = 0;i < MAX_TASKS;i++)
                    {
                        if(tcb[i].semaphore == tcb[taskCurrent].semaphore)
                            if(tcb[taskCurrent].priority < tcb[i].priority)
                                tcb[i].priority = tcb[taskCurrent].currentPriority;
                    }
                }
            }
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        }
   break;

   case(POST) :
           semaphores[R0].count++;
           if(semaphores[R0].queueSize > 0)
           {
               tcb[semaphores[R0].processQueue[0]].state = STATE_READY;
               semaphores[R0].count--;
               for(i = 0;i < semaphores[R0].queueSize;i++)
               {
                   semaphores[R0].processQueue[i] = semaphores[R0].processQueue[i+1];
               }
               semaphores[R0].queueSize--;
           }
           if(pInherit == ON)
           {
               //restore back old priority when resource has been posted
               tcb[taskCurrent].priority = tcb[taskCurrent].currentPriority;
           }
           if(tcb[semaphores[R0].processQueue[0]].priority > tcb[taskCurrent].priority)
           {
               NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
           }
   break;

   case(REBOOT) :
        NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ ;
   break;

   case(RUN) :
        reqRunTask = (char *)R0;
        for(i = 0;i < MAX_TASKS;i++)
        {
            if(stringCompare(reqRunTask, tcb[i].name))
            {
                tcb[i].state = STATE_UNRUN;
                tcb[i].sp = tcb[i].spInit;
                tcb[i].currentPriority = 8;
                tcb[i].priority = 8;
                break;
            }
        }
   break;

   case(RESTART) :
        restartFn = (_fn)R0;
        for(i = 0;i < MAX_TASKS;i++)
        {
            if((_fn)tcb[i].pid == restartFn)
            {
                tcb[i].state = STATE_UNRUN;
                tcb[i].sp = tcb[i].spInit;
                break;
            }
        }
   break;

   case(KILL) :
                killPid = (_fn)R0;
                for(i = 0;i < MAX_TASKS;i++)
                {
                    if((_fn)tcb[i].pid == killPid)
                    {
                        if(!stringCompare(tcb[i].name,"IDLE"))
                        {
                            tcb[i].state = STATE_INVALID;
                            taskCount--;
                            if(tcb[i].semaphore != 0)
                            {
                                tasksSemaphore = tcb[i].semaphore;
                                for(j = 0;j < tasksSemaphore -> queueSize; j++)
                                {
                                    if(tasksSemaphore -> processQueue[j] == i)
                                    {
                                        tasksSemaphore -> processQueue[j] = 0;
                                        for(k = 0;k < tasksSemaphore -> queueSize;k++)
                                        {
                                            tasksSemaphore -> processQueue[k] = tasksSemaphore -> processQueue[k+1];
                                        }
                                        tasksSemaphore -> queueSize--;
                                    }
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
   break;

   case(SETPRIORITY) :
       for(i = 0;i < taskCount;i++)
       {
           if((_fn)tcb[i].pid == (_fn)R0);
           {
               tcb[i].priority = R1;
               break;
           }
       }
   break;

   case(SCHED) :
           if(R0)
               schedulingMode = PRIORITY;
           else
               schedulingMode = RR;
   break;

   case(PREEMPT) :
           if(R0)
               preemptSched = ON;
           else
               preemptSched = OFF;
   break;

   case(PI) :
           if(R0)
               pInherit = ON;
           else
               pInherit = OFF;
   break;

   case(IPCS) :
              j = 0;
              for(i = 0;i < CREATED_SEMAPHORES;i++)
              {
                  while(semaphores[i].name[k] != '\0')
                  {
                      ipcsSemaphores[i].name[k] = semaphores[i].name[k];
                      k++;
                  }
                  ipcsSemaphores[i].count = semaphores[i].count;
                  ipcsSemaphores[i].queueSize = semaphores[i].queueSize;
              }
   break;

   case(PS) :
           for(j = 0;j < taskCount; j++)
           {
               TotalCpu += Ti[!perfBuffer][j];
           }
           for(i = 0;i < taskCount; i++)
           {
               presentTick = Ti[!perfBuffer][i];
               presentTick = presentTick * 10000;
               taskTime = (presentTick)/TotalCpu;
               cpuUsage[i].intUsage = taskTime/100;
               cpuUsage[i].fracUsage = taskTime%100;
           }
               for(i = 0;i < taskCount;i++)
               {

                   psTcb[i].state = tcb[i].state;
                   psTcb[i].pid = tcb[i].pid;
               }


   break;

   case(PIDOF) :
             reqRunTask = (char *)R0;
             for(i = 0;i < MAX_TASKS;i++)
             {
                 if(stringCompare(reqRunTask, tcb[i].name))
                 {
                     putsUart0("Pid of ");
                     putsUart0(tcb[i].name);
                     putsUart0(": ");
                     putsUart0(itoa((uint32_t)tcb[i].pid, str, 16));
                     putsUart0("\r\n");
                     found = true;
                     break;
                 }
             }
             if(!found)
             {
                 putsUart0("Please Enter a Valid Task Name\r\n");
             }
   break;

   }
}

void mpuFaultIsr()
{
    char Str[33];
    uint32_t* pBaseStackFrame = (uint32_t*)getPSP();
    putsUart0("\r\nMPU Fault in Process: ");
    putsUart0(itoa(taskCurrent,Str,10));
    putsUart0("\r\nMSP: ");
    putsUart0(itoa(getMSP(),Str,16));
    putsUart0("\r\nPSP: ");
    putsUart0(itoa(getPSP(),Str,16));
    putsUart0("\r\nmFLAGS: ");
    putsUart0(itoa(NVIC_FAULT_STAT_R | 0x11,Str,16));
    putsUart0("\r\nOffending Address: ");
    putsUart0(itoa(pBaseStackFrame[5],Str,16));
    putsUart0("\r\nOffending Data Location: ");
    putsUart0(itoa(NVIC_MM_ADDR_R,Str,16));
    putsUart0("\r\nProcess Stack Dump");
    putsUart0("\r\nR0: ");
    putsUart0(itoa(pBaseStackFrame[0],Str,16));
    putsUart0("\r\nR1: ");
    putsUart0(itoa(pBaseStackFrame[1],Str,16));
    putsUart0("\r\nR2: ");
    putsUart0(itoa(pBaseStackFrame[2],Str,16));
    putsUart0("\r\nR3: ");
    putsUart0(itoa(pBaseStackFrame[3],Str,16));
    putsUart0("\r\nR12: ");
    putsUart0(itoa(pBaseStackFrame[4],Str,16));
    putsUart0("\r\nLR: ");
    putsUart0(itoa(pBaseStackFrame[5],Str,16));
    putsUart0("\r\nPC: ");
    putsUart0(itoa(pBaseStackFrame[6],Str,16));
    putsUart0("\r\nxPSR: ");
    putsUart0(itoa(pBaseStackFrame[7],Str,16));
    while(1);
}

void hardFaultIsr()
{
    char Str[33];
    putsUart0("\r\nHard Fault in Process: ");
    putsUart0(itoa(taskCurrent,Str,10));
    putsUart0("\r\nMSP: ");
    putsUart0(itoa(getMSP(),Str,16));
    putsUart0("\r\nPSP: ");
    putsUart0(itoa(getPSP(),Str,16));
    putsUart0("\r\nHard Fault Flags: ");
    putsUart0(itoa(NVIC_HFAULT_STAT_R,Str,16));
    while(1);
}

void busFaultIsr()
{
    char Str[33];
    putsUart0("\r\nBus Fault in Process: ");
    putsUart0(itoa(taskCurrent,Str,10));
    putsUart0("\r\n");
    while(1);
}

void usageFaultIsr()
{
    char Str[33];
    putsUart0("\r\nUsage Fault in Process: ");
    putsUart0(itoa(taskCurrent,Str,10));
    while(1);
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clock to GPIO portF, portE  and portA peripherals
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOA;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    // Configure pushbutton pins
    GPIO_PORTA_LOCK_R = 0x4C4F434B;     //Unlock Critical GPIO Pins registers
    GPIO_PORTA_CR_R = 0b00111111;       //Allow them to be Written
    GPIO_PORTA_DEN_R |= PUSH_BUTTON_0_MASK | PUSH_BUTTON_1_MASK | PUSH_BUTTON_2_MASK | PUSH_BUTTON_3_MASK | PUSH_BUTTON_4_MASK | PUSH_BUTTON_5_MASK;
    GPIO_PORTA_PUR_R |= PUSH_BUTTON_0_MASK | PUSH_BUTTON_1_MASK | PUSH_BUTTON_2_MASK | PUSH_BUTTON_3_MASK | PUSH_BUTTON_4_MASK | PUSH_BUTTON_5_MASK;
     GPIO_PORTA_CR_R = 0b00000000;       //Lock critical GPIO Pins registers

    //Configure LED pins for OffBoard LEDs
    GPIO_PORTE_DIR_R |= RED_LED_MASK | GREEN_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;
    GPIO_PORTE_DR2R_R |= RED_LED_MASK | GREEN_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;
    GPIO_PORTE_DEN_R |= RED_LED_MASK | GREEN_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;

    //Configure LED pin for OnBoard LED
    GPIO_PORTF_DIR_R |= BLUE_LED_MASK;
    GPIO_PORTF_DR2R_R |= BLUE_LED_MASK;
    GPIO_PORTF_DEN_R |= BLUE_LED_MASK;

    //Configure Timer to keep track of the process timing
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
    TIMER1_TAPMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;
    TIMER1_TAILR_R = TIMER_TAILR_M;
}

uint8_t readPbs()
{
    uint8_t i=0;
    if(PUSH_BUTTON_0 == 0)
    {
        i += 1;
    }
    if(PUSH_BUTTON_1 == 0)
    {
        i += 2;
    }
    if(PUSH_BUTTON_2 == 0)
    {
        i += 4;
    }
    if(PUSH_BUTTON_3 == 0)
    {
        i += 8;
    }
    if(PUSH_BUTTON_4 == 0)
    {
        i += 16;
    }
    if(PUSH_BUTTON_5 == 0)
    {
        i += 32;
    }
    return i;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

void startSysTick(uint32_t tick_hz)
{
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_CLK_SRC;
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;
    NVIC_ST_RELOAD_R &= ~(0x00FFFFFF);
    NVIC_ST_RELOAD_R |= count_value;
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
}

void run(char* arg2)
{
    __asm(" SVC #51");
}


void sched(bool prio)
{
    __asm(" SVC #60");
}

void pi(bool pi)
{
    __asm(" SVC #61");
}

void preempt(bool prempt)
{
    __asm(" SVC #62");
}

void pidOf(char* arg2)
{
    __asm(" SVC #55");
}


void initMpu(void)
{
    //MPU Aperture for Entire Memory with Execute Never and Read and Write for Priv/Unpriv Software
    NVIC_MPU_NUMBER_R = 0;
    NVIC_MPU_BASE_R = 0x00000000;
    NVIC_MPU_ATTR_R = 0;
    NVIC_MPU_ATTR_R |=  (1<<28) | (0b011<<24) | (0b111<16) | (0b11111<<1) |NVIC_MPU_ATTR_ENABLE;

    //MPU Aperture for Flash Memory with execute read and write for priv/unpriv software
    NVIC_MPU_NUMBER_R = 1;
    NVIC_MPU_BASE_R = 0x00000000;
    NVIC_MPU_ATTR_R = 0;
    NVIC_MPU_ATTR_R |=  (0b011<<24) | (0b010<16) | (0b10010<<1) | NVIC_MPU_ATTR_ENABLE;

    //Disabled rule for lower 16K subregion
    NVIC_MPU_NUMBER_R = 4;
    NVIC_MPU_BASE_R = 0x20001000;
    NVIC_MPU_ATTR_R = 0;
    NVIC_MPU_ATTR_R |=  (0b011<<24) | (0b110<16)  | (0b01101<<1) | NVIC_MPU_ATTR_ENABLE;

    //Even tough above rules overlap because of the higher priority of Below rule,below rule will  be enforced
    //Disabled rule for Upper 16K Subregion
    NVIC_MPU_NUMBER_R = 5;
    NVIC_MPU_BASE_R = 0x20004000;
    NVIC_MPU_ATTR_R = 0;
    NVIC_MPU_ATTR_R |=  (0b011<<24) | (0b110<16)  | (0b01101<<1) | NVIC_MPU_ATTR_ENABLE;


}
// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
#ifdef DEBUG
        __asm(" MOV R0,#0");
        __asm(" MOV R1,#1");
        __asm(" MOV R2,#2");
        __asm(" MOV R3,#3");
        __asm(" MOV R12,#12");
        __asm(" MOV R4,#4");
        __asm(" MOV R5,#5");
        __asm(" MOV R6,#6");
        __asm(" MOV R7,#7");
        __asm(" MOV R8,#8");
        __asm(" MOV R9,#9");
        __asm(" MOV R10,#10");
        __asm(" MOV R11,#11");
#endif
        yield();
    }
}

#ifdef DEBUG_TASK_SWITCH
void idle2()
{
    while(true)
    {
        YELLOW_LED = 1;
        waitMicrosecond(1000);
        YELLOW_LED = 0;
        __asm(" MOV R0,#0");
        __asm(" MOV R1,#2");
        __asm(" MOV R2,#3");
        __asm(" MOV R3,#4");
        __asm(" MOV R12,#13");
        __asm(" MOV R4,#5");
        __asm(" MOV R5,#6");
        __asm(" MOV R6,#7");
        __asm(" MOV R7,#8");
        __asm(" MOV R8,#9");
        __asm(" MOV R9,#10");
        __asm(" MOV R10,#11");
        __asm(" MOV R11,#12");
        yield();
    }
}
#endif

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void errant()
{
    uint32_t* p = (uint32_t*)0x20000000;
    while(true)
    {
        while (readPbs() == 32)
        {
            *p = 0;
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

// REQUIRED: add processing for the shell commands through the UART here
void shell()
{
    USER_DATA info;
    while (true)
    {

        //Terminal processing happens here
        if (kbhitUart0())
        {
           getsUart0(&info);        //get uart data into a USER INFO structure
           yield();
           putsUart0(info.buffer);  //display the info onto the terminal
           putsUart0("\n\r");
           parseFields(&info);      //parse information in the buffer and store it in the structure

           //is Command RESET
           if(isCommand(&info,"REBOOT",0))               //Is command argument REBOOT
               {
                   __asm(" SVC #50");
               }
           else if(isCommand(&info,"PS",0))             //Is command argument PS
               {
                   char str[15];
                   uint8_t i;
                   __asm(" SVC #53");
                   putsUart0("Name\t\tPid\t\tState\t\tCpu Usage\r\n");
                   for(i = 0;i < TASKS_CREATED;i++)
                   {
                       putsUart0(psTcb[i].name);
                       putsUart0("\t");
                       //MAGIC TO SET ALIGNMENT
                       if(i == 0||i ==2||i==5||i==8||i==9)
                       putsUart0("\t");
                       putsUart0(itoa((uint32_t)psTcb[i].pid, str, 16));
                       putsUart0("\t");
                       if(psTcb[i].state == STATE_READY)
                           putsUart0("STATE READY\t");
                       else if(psTcb[i].state == STATE_UNRUN)
                           putsUart0("STATE_UNRUN\t");
                       else if(psTcb[i].state == STATE_BLOCKED)
                           putsUart0("STATE_BLOCKED\t");
                       else if(psTcb[i].state == STATE_DELAYED)
                           putsUart0("STATE_DELAYED\t");
                       else if(psTcb[i].state == STATE_INVALID)
                           putsUart0("STATE_INVALID\t");
                       putsUart0(itoa(cpuUsage[i].intUsage, str, 10));
                       putsUart0(".");
                       putsUart0(itoa(cpuUsage[i].fracUsage, str, 10));
                       putsUart0("\r\n");
                   }
               }
           else if(isCommand(&info,"IPCS",0))           //Is command argument IPCS
               {
                   uint8_t i;
                   char str[15];
                   __asm(" SVC #54");
                   putsUart0("Semaphore Name  QueueCount  Count\r\n");
                   for(i = 0; i < CREATED_SEMAPHORES; i++)
                   {
                       if(i == keyPressed)
                           putsUart0("Key Pressed");
                       else if(i == keyReleased)
                           putsUart0("Key Released");
                       else if(i == flashReq)
                           putsUart0("Flash Req");
                       else if(i == resource)
                           putsUart0("Resource");
                       putsUart0("\t\t");
                       putsUart0(itoa(ipcsSemaphores[i].count, str, 10));
                       putsUart0("\t");
                       putsUart0(itoa(ipcsSemaphores[i].queueSize, str, 10));
                       putsUart0("\r\n");
                   }
               }
           else if(isCommand(&info,"KILL",1))           //Is command argument Kill
               {
                   _fn toBeKilledPid =(_fn)getFieldInt(&info, 2);
                   destroyThread(toBeKilledPid);
               }
           else if(isCommand(&info,"PI",1))             //Is command argument PI
               {
               if(stringCompare(getFieldString(&info,2),"ON") == true)
                   pi(true);
               else if(stringCompare(getFieldString(&info,2),"OFF") == true)
                   pi(false);
               else
                   putsUart0("Please enter valid Input Argument\r\n");
               }
           else if(isCommand(&info,"PREEMPT",1))        //Is command argument PREEMPT
               {
               if(stringCompare(getFieldString(&info,2),"ON") == true)
                   preempt(true);
               else if(stringCompare(getFieldString(&info,2),"OFF") == true)
                   preempt(false);
               else
                   putsUart0("Please enter valid Input Argument\r\n");
               }
           else if(isCommand(&info,"SCHED",1))          //Is command argument SCHED
               {
                   if(stringCompare(getFieldString(&info,2),"PRIO") == true)
                       sched(true);
                   else if(stringCompare(getFieldString(&info,2),"RR") == true)
                       sched(false);
                   else
                       putsUart0("Please enter valid Input Argument\r\n");
               }
           else if(isCommand(&info,"PIDOF",1))          //Is command argument PIDOF
               {
                   pidOf(getFieldString(&info, 2));
               }
           else if(isCommand(&info,"RUN",1))            //Is command argument RUN
               {
                   run(getFieldString(&info,2));
               }
           else
               putsUart0("Please Enter a Valid Input Command\r\n");
        }
        yield();
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    initUart0();
    initRtos();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    createSemaphore(1,keyPressed,"KeyPressed");
    createSemaphore(0,keyReleased,"KeyReleased");
    createSemaphore(5,flashReq,"FlashReq");
    createSemaphore(1,resource,"Resource");


#ifdef DEBUG_TASK_SWITCH
    ok = createThread(idle2, "Idle2", 15, 1024);
#endif

    // Add required idle process at lowest priority
    // Add other processes
    ok =  createThread(idle, "IDLE", 15, 1024);
    ok &= createThread(flash4Hz, "FLASH4HZ", 8, 1024);
    ok &= createThread(oneshot, "ONESHOT", 4, 1024);
    ok &= createThread(important, "IMPORTANT", 0, 1024);
    ok &= createThread(lengthyFn, "LENGTHYFN", 12, 1024);
    ok &= createThread(shell, "SHELL", 12, 1024);
    ok &= createThread(readKeys, "READKEYS", 12, 1024);
    ok &= createThread(debounce, "DEBOUNCE", 12, 1024);
    ok &= createThread(uncooperative, "UNCOOP", 12, 1024);
    ok &= createThread(errant, "ERRANT", 12, 1024);


    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
