#include <stdio.h>
#include "inc/lm3s6965.h"
#include "rit128x96x4.h"
#include "scheduler.h"

void thread1(void)
{
	
	while(1) 
	{
		iprintf("\tPrinting to the terminal window.\r\n");

		yield();
	}

	yield();

}

void thread2(void)
{

	volatile unsigned long ulLoop;
 
	while(1)
	{
		// Turn on the LED.
		GPIO_PORTF_DATA_R |= 0x01;

		// Delay for a bit.
		for(ulLoop = 0; ulLoop < 200000; ulLoop++){}

		// Turn off the LED.
		GPIO_PORTF_DATA_R &= ~(0x01);

		// Delay for a bit.
		for(ulLoop = 0; ulLoop < 200000; ulLoop++){}

		yield();		

	}

	yield();

}

void thread3(void)
{

	volatile unsigned long ulLoop;

	while(1)
	{

		//display some text
		RIT128x96x4StringDraw("Project 3", 20,  0, 15);

		// Delay for a bit.
		for(ulLoop = 0; ulLoop < 200000; ulLoop++){}

		//clear it
		RIT128x96x4StringDraw("         ", 20,  0, 15);

		// Delay for a bit.
		for(ulLoop = 0; ulLoop < 200000; ulLoop++){}
		
		yield();
	}

	yield();

}
