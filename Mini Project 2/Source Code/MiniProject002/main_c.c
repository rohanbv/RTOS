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

//Debug Macros
#define MPU_TESTING

// Defining Stack Spaces
#define SRAM_START          0x20000000U
#define SRAM_SIZE           (32 * 1024)
#define SRAM_END            ( (SRAM_START) + (SRAM_SIZE) )
#define STACK_START         SRAM_END

#define STACK_MSP_START     STACK_START
#define STACK_MSP_END       (STACK_START - 512)
#define STACK_PSP_START     STACK_MSP_END

//Defining System Fault Enables
#define ENABLE_USAGE_FAULT   1<<18
#define ENABLE_BUS_FAULT     1<<17
#define ENABLE_MEM_FAULT     1<<16
#define SET_MEMARV           1<<7
#define USAGE_FAULT_PEND     1<<12
#define MEM_FAULT_PEND       1<<13

// Bitband aliases
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

// PortF masks
#define GREEN_LED_MASK 8
#define RED_LED_MASK 2


//All the functions prototypes
void initHw();
char* decToHexaString(int n);
int div(int a,int b);


extern uint32_t getMSP(void);
extern uint32_t getPSP(void);
extern void setPSP(uint32_t);
extern void setMSP(uint32_t);
extern uint32_t getR0(void);
extern uint32_t getR1(void);
extern uint32_t getR2(void);
extern uint32_t getR3(void);
extern uint32_t getR12(void);
extern uint32_t getPC(void);
extern uint32_t getLR(void);
extern uint32_t getxPSR(void);
extern void changeSPtoPSP(void);
//This changes privilege from unprivileged to privileged execution only in Handler Mode
//But It can Change Privilege to Unprivileged in User/handler Mode
extern void change_Privilege(void);
void thread(void);

uint32_t pid;
char Str[10];

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
void HardFaultISR(void)
{
    uint32_t p,m;
    p = getPSP();
    m = getMSP();
    putsUart0("\r\nHard Fault in Process: ");
    putsUart0(itoa(pid,Str,10));
    putsUart0("\r\nMSP: ");
    putsUart0(itoa(m,Str,16));
    putsUart0("\r\nPSP: ");
    putsUart0(itoa(p,Str,16));
    putsUart0("\r\nHard Fault Flags: ");
    putsUart0(itoa(NVIC_HFAULT_STAT_R,Str,16));
    while(1);
}
void PendSVISR(void)
{
    uint32_t *pUnprivVariable = (uint32_t*) 0x20007C98;
      *pUnprivVariable = 32;
    putsUart0("\r\nPendSV in Process: ");
    putsUart0(itoa(pid,Str,10));
    if(NVIC_FAULT_STAT_R || 0b11 == 1)
    {
    NVIC_FAULT_STAT_R &= ~NVIC_FAULT_STAT_IERR | ~NVIC_FAULT_STAT_DERR ;
    putsUart0("\r\nCalled from MPU ");
    }
    while(1);
}

void MPUFaultISR(void)
{

    uint32_t* pBaseStackFrame = (uint32_t*)getPSP();
    putsUart0("\r\nMPU Fault in Process: ");
    putsUart0(itoa(pid,Str,10));
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
    //clearing the MPU fault Pending bit
    NVIC_SYS_HND_CTRL_R&=~(1<<13);//MPU Fault PENDING BIT
    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
}

void BusFaultISR(void)
{
    putsUart0("\r\nBus Fault in Process: ");
    putsUart0(itoa(pid,Str,10));
    putsUart0("\r\n");
    while(1);
}

void UsageFaultISR(void)
{

    uint32_t* pBaseStackFrame = (uint32_t*)getPSP();
    putsUart0("\r\nUsage Fault in Process: ");
    while(1);
}

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

int div(int a,int b)
{
    return a/b;
}

void thread(void)
{

    uint32_t *pUnprivVariable = (uint32_t*) 0x20007C98;
    *pUnprivVariable = 32;
    uint32_t *punaccessVariable = (uint32_t*)0x20000C08;
    *punaccessVariable= 20;
    while(1);
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    setMSP(STACK_MSP_START);
    setPSP(STACK_PSP_START);
    changeSPtoPSP();
    NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_BUS | NVIC_SYS_HND_CTRL_USAGE | NVIC_SYS_HND_CTRL_MEM;
    //Active write MEMADDR register on memory access violation
    NVIC_FAULT_STAT_R |= NVIC_FAULT_STAT_MMARV;
    //NVIC_SYS_PRI3_R |= (0b010<<21);
    //NVIC_SYS_PRI1_R |= (0b111<<5);

    // Initialize hardware
    initHw();
    initUart0();
    setUart0BaudRate(115200, 40e6);
    //Activate all the system exceptions

    putsUart0("\r\n\r\nOperational Stack : PSP");


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
#ifdef MPU_TESTING

    //Multiple MPU regions to cover SRAM
    NVIC_MPU_NUMBER_R = 3;
    NVIC_MPU_BASE_R = 0x20000000;
    NVIC_MPU_ATTR_R = 0;
    NVIC_MPU_ATTR_R |=  (0b001<<24) | (0b110<16)  | (0b01101<<1) | NVIC_MPU_ATTR_ENABLE;

    NVIC_MPU_NUMBER_R = 4;
    NVIC_MPU_BASE_R = 0x20002000;
    NVIC_MPU_ATTR_R = 0;
    NVIC_MPU_ATTR_R |=  (0b001<<24) | (0b110<16) | (0b01101<<1) | NVIC_MPU_ATTR_ENABLE;

    NVIC_MPU_NUMBER_R = 5;
    NVIC_MPU_BASE_R = 0x20004000;
    NVIC_MPU_ATTR_R = 0;
    NVIC_MPU_ATTR_R |=  (0b001<<24) | (0b110<16) | (0b01101<<1) | NVIC_MPU_ATTR_ENABLE;

    NVIC_MPU_NUMBER_R = 6;
    NVIC_MPU_BASE_R = 0x20007C00;
    NVIC_MPU_ATTR_R = 0;
    NVIC_MPU_ATTR_R |=  (0b011<<24) | (0b110<16)  | (0b01010<<1) | NVIC_MPU_ATTR_ENABLE;

    NVIC_MPU_NUMBER_R = 7;
    NVIC_MPU_BASE_R = 0x20006000;
    NVIC_MPU_ATTR_R = 0;
    NVIC_MPU_ATTR_R |=  (0b001<<24) | (0b110<16)  | (0b01101<<1) | (1<<15) | NVIC_MPU_ATTR_ENABLE;

    NVIC_MPU_CTRL_R |= NVIC_MPU_CTRL_ENABLE | NVIC_MPU_CTRL_PRIVDEFEN;
    putsUart0("\r\nReject Range: 0x2000000 - 0x20007C00");
#endif

    //change_Privilege(); //Change Privilege of Execution to Unprivileged Mode

    GREEN_LED ^= 1;
    GREEN_LED ^= 1;

    putsUart0("\r\nChanging Privilege");
    change_Privilege();
#ifdef MPU_TESTING
    thread();
#endif



    while(1);

}
