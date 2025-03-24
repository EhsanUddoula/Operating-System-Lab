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

#ifndef __KERN_UNISTD_H
#define __KERN_UNISTD_H
/* Constants for read/write/etc: special file handles */
#include <kstring.h>
#include <cm4.h>
#define STDIN_FILENO  0      /* Standard input */
#define STDOUT_FILENO 1      /* Standard output */
#define STDERR_FILENO 2      /* Standard error */
#define O_RDONLY 0
#define O_WRONLY 1
#define O_APPEND 2
#define RUNNING 0
#define WAITING 1
#define READY 2
#define KILLED 3
#define TERMINATED 4
extern volatile uint16_t  TASK_ID;
const static uint32_t DIGITAL_SIGNATURE = 0x00000001; 

#define MAX_QUEUE_SIZE_P  5


extern volatile uint32_t QUEUE_SIZE_P ;
extern volatile uint32_t CURR_TASK_P ;

extern volatile TCB_TypeDef READY_QUEUE[MAX_QUEUE_SIZE_P];
void __sys_open(char *,uint8_t,uint32_t *);
void __sys_close(uint32_t *);
void __sys_reboot(void);

void __sys_read(uint8_t,char **,uint32_t);
void __sys_write(uint8_t fd,char *data);

void __sys_start_task(uint32_t);
void __sys_getpid(unsigned int *val,uint16_t value);

void __sys_get_time(uint32_t *time);

extern dev_table device_list[64];
extern uint32_t device_count;

void __init_dev_table(void);
#endif /* KERN_UNISTD_H */

