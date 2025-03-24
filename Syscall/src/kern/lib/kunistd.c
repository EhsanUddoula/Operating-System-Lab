/*
 * Copyright (c) 2022 
 * Computer Science and Engineering, University of Dhaka
 * Credit: CSE Batch 25 (starter) and Prof. Mosaddek Tushar
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#include <kunistd.h>
#include <stdint.h>
#include <cm4.h>
#include <sys_usart.h>
/* Add your functions here */

volatile uint32_t QUEUE_SIZE_P = 3;
volatile uint32_t CURR_TASK_P = 0;
volatile uint16_t TASK_ID = 1000;

volatile TCB_TypeDef READY_QUEUE[MAX_QUEUE_SIZE_P];



/**************** *********************/

uint32_t *psp_stack_addresses[2];
uint32_t size = 2;
uint32_t CURR_TASK = 0;


void __attribute__((noreturn)) sleep_state(void)
{
    // set_pending(0);

    __set_pending(0);

    while (1)
    {
        __WFI(); // Wait For Interrupt (Stops CPU Until An Interrupt Occurs)
    }
}

#ifndef RR
#define RR
void __attribute__((naked)) PendSV_Handler(void)
{
    // Clear all pending interrupts
    SCB->ICSR |= (1 << 27);

    // kprintf("PendSV_Handler\n");

    // save current context
    if (READY_QUEUE[CURR_TASK_P].status == RUNNING)
    {
        READY_QUEUE[CURR_TASK_P].status = READY;
        asm volatile(
            "mrs r0, psp\n"
            "isb 0xf\n"
            "stmdb r0!, {r4-r11}\n");

        asm volatile("mov %0, r0\n"
                     : "=r"(READY_QUEUE[CURR_TASK_P].psp)
                     :);
    }
    __DSB();
    __ISB();
    /*---------------------------------------------*/

    // CURR_TASK_P = (CURR_TASK_P + 1) % QUEUE_SIZE_P;

    uint32_t chosen_task = MAX_QUEUE_SIZE_P;
    uint32_t count = 0;

    for (int i = (CURR_TASK_P + 1) % QUEUE_SIZE_P;; i = (i + 1) % QUEUE_SIZE_P)
    {

        if (READY_QUEUE[i].status == READY)
        {
            chosen_task = i;
            break;
        }

        count++;

        if (count >= MAX_QUEUE_SIZE_P)
        {
            break;
        }
    }

    if (chosen_task == 5)
    { // finished

        // while(1);
        // uint32_t new_psp_stack[1024];
        // uint32_t* new_psp = (uint32_t*) new_psp_stack + 1024;
        // *(--new_psp) = 0x01000000;      // xPSR
        // *(--new_psp) = (uint32_t)sleep_state; // PC
        // *(--new_psp) = 0xFFFFFFFD;      // LR

        // for (uint32_t i = 0; i < 13; i++)
        // {
        //     *(--new_psp) = 0;
        // }

        __set_pending(0);
        // __asm volatile(
        //     "mov r0, %0\n"
        //     "msr psp, r0\n"
        //     "isb 0xf\n"
        //     :
        //     : "r" (new_psp)
        // );

        __DSB();
        __ISB();

        // __asm volatile("bx lr\n");
        // sleep_state();

        asm volatile("bl sleep_state");
    }
    else
    {
        CURR_TASK_P = chosen_task;
    }

    __DSB();
    __ISB();

    asm volatile(
        "mov r0, %0"
        :
        : "r"((uint32_t)READY_QUEUE[CURR_TASK_P].psp));

    READY_QUEUE[CURR_TASK_P].status = RUNNING;

    asm volatile(
        "ldmia r0!,{r4-r11}\n"
        "msr psp, r0\n"
        "isb 0xf\n"
        "bx lr\n");
}
#endif

