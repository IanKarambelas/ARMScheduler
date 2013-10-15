#include <stdio.h>
#include <setjmp.h>
#include <stdlib.h>
#include "inc/lm3s6965.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "rit128x96x4.h"
#include "scheduler.h"

#define STACK_SIZE 4096   // Amount of stack space for each thread

typedef struct {
	int active;       // non-zero means thread is allowed to run
	char *stack;      // pointer to TOP of stack (highest memory location)
	jmp_buf state;    // saved state for longjmp()
  unsigned savedregs[40];
} threadStruct_t;

// thread_t is a pointer to function with no parameters and
// no return value...i.e., a user-space thread.
typedef void (*thread_t)(void);

// These are the external user-space threads. In this program, we create
// the threads statically by placing their function addresses in
// threadTable[]. A more realistic kernel will allow dynamic creation
// and termination of threads.
extern void thread1(void);
extern void thread2(void);
extern void thread3(void);

static thread_t threadTable[] = {
	thread1,
	thread2,
	thread3
};

#define NUM_THREADS (sizeof(threadTable)/sizeof(threadTable[0]))

// These static global variables are used in scheduler(), in
// the yield() function, and in threadStarter()
//static jmp_buf scheduler_buf;   // saves the state of the scheduler
static threadStruct_t threads[NUM_THREADS]; // the thread table
unsigned currThread;    // The currently active thread

// This function is called from within user thread context. It executes
// a jump back to the scheduler. When the scheduler returns here, it acts
// like a standard function return back to the caller of yield().
void yield(void)
{

  asm volatile ("SVC #10");

}

// This is the starting point for all threads. It runs in user thread
// context using the thread-specific stack. The address of this function
// is saved by createThread() in the LR field of the jump buffer so that
// the first time the scheduler() does a longjmp() to the thread, we
// start here.
void threadStarter(void)
{
  
	// Call the entry point for this thread. The next line returns
	// only when the thread exits.
	(*(threadTable[currThread]))();

	// Do thread-specific cleanup tasks. Currently, this just means marking
	// the thread as inactive. Do NOT free the stack here because we're
	// still using it! Remember, this function runs in user thread context.
	threads[currThread].active = 0;

	// This yield returns to the scheduler and never returns back since
	// the scheduler identifies the thread as inactive.
	yield();
}

// This function is implemented in assembly language. It sets up the
// initial jump-buffer (as would setjmp()) but with our own values
// for the stack (passed to createThread()) and LR (always set to
// threadStarter() for each thread).
extern void createThread(unsigned buf[], char *stack);


//store the state of the thread by storing the registers
void sstore(unsigned *savedregs)
{

  asm volatile (
	  //store all the callee-preserved registers into the array
	  "mrs 	  r12, psp\n"
	  "isb\n"
	  "stmia 	r0!, { r4-r10, r11, r12 }\n"
	  "bx 	  lr"
  );

}

//restore the state of the thread by restoring the registers
void srestore(unsigned *savedregs)
{
  
  asm volatile (
    //restore all the callee-preserved registers from the array
    "ldmia  r0!, { r4-r10, r11, r12 }\n"
    "msr    psp, r12\n"
    "isb\n"
    "bx     lr"
	);
  
}

// This is the "main loop" of the program.
void scheduler(void)
{

  //iprintf("Current Thread: %d\r\n", currThread);

  //save the state of the current thread (but not the first time through)
	if (currThread >= 0)
    sstore(threads[currThread].savedregs);
  
  //increment to the next thread
  currThread++;

  //if the current thread exceeds the number of threads, go back to the first
  if (currThread >= NUM_THREADS)
    currThread = 0;

  //if the thread is active, restore it
  if (threads[currThread].active)
    srestore(threads[currThread].savedregs);

  //otherwise, branch out
  //else
  //  asm volatile("bx lr");
  
  //sets the EXC_RETURN value to return to thread (unprivileged) mode 
  //using the PSP
  asm volatile("ldr pc, =0xFFFFFFFD");

}

void main(void)
{
	unsigned i;

  currThread = -1;
  
	// Set the clocking to run directly from the crystal.
	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
								 SYSCTL_XTAL_8MHZ);

	// Initialize the OLED display and write status.
	RIT128x96x4Init(1000000);
	RIT128x96x4StringDraw("Project 3",       20,  0, 15);

	// Enable the peripherals used by this example.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// Set GPIO A0 and A1 as UART pins.
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 115,200, 8-N-1 operation.
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
											(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
											 UART_CONFIG_PAR_NONE));

  // Enable the GPIO port that is used for the on-board LED.
  SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;

  // Enable the GPIO pin for the LED (PF0).  Set the direction as output, and
  // enable the GPIO pin for digital function.
  GPIO_PORTF_DIR_R = 0x01;
  GPIO_PORTF_DEN_R = 0x01;

	//scheduler runs as the SysTick and SVC interrupt handler,
	//which requires that interrupts be enabled
	IntMasterEnable();

	// Create all the threads and allocate a stack for each one
	for (i=0; i < NUM_THREADS; i++) {
		// Mark thread as runnable
		threads[i].active = 1;

		// Allocate stack
		threads[i].stack = (char *)malloc(STACK_SIZE) + STACK_SIZE;

		if (threads[i].stack == 0) {
			iprintf("Out of memory\r\n");
			exit(1);
		}

		// After createThread() executes, we can execute a longjmp()
		// to threads[i].state and the thread will begin execution
		// at threadStarter() with its own stack.
		createThread(threads[i].savedregs, threads[i].stack);

	}

  
  //SysTick Timer configuration
  //use the internal clock
  NVIC_ST_CTRL_R = 0x00000004;

  //1ms second timer
  NVIC_ST_RELOAD_R = 16000;
  
  //clear the current value
  NVIC_ST_CURRENT_R = 0;
  
  //start the SysTick timer
  NVIC_ST_CTRL_R = 0x00000007;


	// Start running coroutines
	yield();

	// If scheduler() returns, all coroutines are inactive and we return
	// from main() hence exit() should be called implicitly (according to
	// ANSI C). However, TI's startup_gcc.c code (ResetISR) does not
	// call exit() so we do it manually.
	exit(0);
}

/*
 * Compile with:
 * ${CC} -o crsched.elf -I${STELLARISWARE} -L${STELLARISWARE}/driverlib/gcc 
 *     -Tlinkscript.x -Wl,-Map,crsched.map -Wl,--entry,ResetISR 
 *     crsched.c create.S threads.c startup_gcc.c syscalls.c rit128x96x4.c 
 *     -ldriver
 */
// vim: expandtab ts=2 sw=2 cindent
