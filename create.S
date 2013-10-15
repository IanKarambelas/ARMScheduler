/*
 * Implement the thread creation task:
 *
 *   - initialize the jump buffer with appropriate values for
 *     R13 (stack) and R14 (first address to jump to)
 *   - all other registers are irrelevant upon thread creation
 *
 *   In the jump buffer, the R13 slot is set to the second parameter of this
 *   function (the top-of-stack address, passed in R1). The R14 slot is set to
 *   the address of the threadStarter() function.
 *
 *   The C prototype for this function call is:
 *       createThread(threads[i].state, threads[i].stack)
 *   thus:
 *       R0 <-- state (a setjmp()-style jump buffer)
 *       R1 <-- stack (address of top-of-stack)
 */
    .syntax unified
    .text
    .align 2
    .thumb
    .thumb_func
    .type createThread,function
    .global createThread
createThread:

	@move the pointer down 9 spaces
	sub		r12, r1, #36

	@store all of the array values
	stmia 	r0!, { r4-r12 }

	@the initial XPSR value
	movw	r3, 0x0000
	movt	r3, 0x0100

	@stores the XPSR value in the stack
	str		r3, [r1, #-8]

	@store the address to return to in the stack
	ldr     r2, .L0
	str		r2, [r1, #-12]

	@store initial values of zero into the stack
	mov		r3, #0
	str		r3, [r1, #-16]
	str		r3, [r1, #-20]
	str		r3, [r1, #-24]
	str		r3, [r1, #-28]
	str		r3, [r1, #-32]
	str		r3, [r1, #-36]
	
	bx		lr
    
.L0:
    .word   threadStarter
