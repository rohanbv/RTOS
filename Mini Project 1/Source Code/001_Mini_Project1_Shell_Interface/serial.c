//-----------------------------------------------------------------------------
// README
//-----------------------------------------------------------------------------

//  For testing of application,please connect to COM port of ICDI debugger as it
//  enumerates the com port at baud rate: 115200
//  Commands supported for Now:
//      Reboot
//      PS
//      IPCS
//      kill PID
//      pi ON|OFF
//      preempt ON|OFF
//      sched PRIO|RR
//      pidof proc_name
//      run proc_name

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
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

// Bitband aliases
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

// PortF masks
#define GREEN_LED_MASK 8
#define RED_LED_MASK 2

//All the functions prototypes
void initHw();
void Reset(void);
void shell(void);
void KILL(int pid);
void PS(void);
void IPCS(void);
void RUN(void);
void pidof(char name[]);
void sched(bool prio_On);
void preempt(bool on);
void pi(bool on);

//Global
char* pid_Number;
char* Str;

uint32_t stack[12][512]  __attribute__((aligned (2048)));;
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure LED pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | RED_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK | RED_LED_MASK;  // enable LEDs
}

//Blocking Shell function
void shell(void)
{
    //variable
    USER_DATA info;

    while(true)
    {
        //Terminal processing happens here
        if (kbhitUart0())
        {
           getsUart0(&info);        //get uart data into a USER INFO structure
           putsUart0(info.buffer);  //display the info onto the terminal
           putsUart0("\n\r");
           parseFields(&info);      //parse information in the buffer and store it in the structure

           //is Command RESET
           if(isCommand(&info,"REBOOT",0))               //Is command argument REBOOT
               {
                   Reset();                             //call RESET routine
               }
           else if(isCommand(&info,"PS",0))             //Is command argument PS
               {
                   PS();                                //call PS routine
               }
           else if(isCommand(&info,"IPCS",0))           //Is command argument IPCS
               {
                   IPCS();                              //call IPCS routine
               }
           else if(isCommand(&info,"KILL",1))           //Is command argument Kill
               {
                   int Number = getFieldInt(&info,2);
                   pid_Number = getFieldString(&info,2);
                   KILL(Number);                        //call Kill routine
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
                  pidof(getFieldString(&info,2));       //call pidof routine
               }
           else if(isCommand(&info,"RUN",1))            //Is command argument RUN
               {
                  RUN();                                //call run routine
               }
           else
               putsUart0("Please Enter a Valid Input Command\r\n");
        }
    }
}

void Reset(void)
{
    putsUart0("Is a Valid Command for RESET,performing RESET\r\n");
    waitMicrosecond(10000);
    //Reset core and all on-chip peripherals
    NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ ;
}

void PS(void)
{
    putsUart0("PS called\r\n");
}

void IPCS(void)
{
    putsUart0("IPCS called\r\n");
}

void KILL(int pid)
{
    putsUart0("PID:");
    putsUart0(pid_Number);
    putsUart0(" Killed\r\n");
}

void RUN(void)
{
    putsUart0("Toggling the RED led\r\n");
    RED_LED ^= 1;
}

void pidof(char name[])
{
   putsUart0(name);
   putsUart0(" launched\r\n");
}

void sched(bool prio_On)
{
    if(prio_On)
        putsUart0("sched prio\r\n");
    else
        putsUart0("sched rr\r\n");
}

void pi(bool on)
{
    if(on)
        putsUart0("pi ON\r\n");
    else
        putsUart0("pi OFF\r\n");
}

void preempt(bool on)
{
    if(on)
        putsUart0("preempt ON\r\n");
    else
        putsUart0("preempt OFF\r\n");
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
	// Initialize hardware
	initHw();
	initUart0();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);
    GREEN_LED = 1;
    waitMicrosecond(500000);
    GREEN_LED = 0;
    shell();
}
