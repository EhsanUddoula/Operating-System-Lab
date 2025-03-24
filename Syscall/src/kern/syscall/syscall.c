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

#include <syscall.h>
#include <syscall_def.h>
#include <errno.h>
#include <stdint.h>
#include <kstdio.h>
#include <sys_usart.h>
#include <cm4.h>
#include <types.h>
#include <kstring.h>
#include <system_config.h>
#include <UsartRingBuffer.h>
#include <stdarg.h>

extern uint32_t device_count;
dev_table device_list[64];
uint32_t device_count = 0;
volatile TCB_TypeDef READY_QUEUE[10];

#define MAX_ARGS 10  // Maximum number of arguments
#define MAX_ARG_LEN 64  // Maximum length of each argument

typedef struct {
    char *argv[MAX_ARGS];  // Array of argument strings
    int argc;              // Number of arguments
} ExecArgs;

#define MAX_TASKS 10  // Maximum number of tasks
#define TIME_SLICE 10 // Time slice for each task (in ms)

typedef enum {
    TASK_READY,
    TASK_RUNNING,
    TASK_WAITING,
    TASK_KILLED
} TaskStatus;

TCB_TypeDef *task_list[MAX_TASKS];  // List of tasks
uint8_t current_task_index = 0;     // Index of the currently running task
uint8_t total_tasks = 0; 

void scheduler_add_task(TCB_TypeDef *task) {
    if (total_tasks < MAX_TASKS) {
        task_list[total_tasks] = task;
        task->status = TASK_READY;
        total_tasks++;
    } else {
        kprintf("Scheduler: Task list full!\n");
    }
}

void *load_executable(const char *filename) {
    // Placeholder: Implement the logic to load the executable file
    // For now, return a dummy entry point
    kprintf("Loading executable: %s\n", filename);
    return (void *)0x08000000;  // Dummy entry point
}

void setup_stack(ExecArgs *exec_args) {
    // Placeholder: Implement the logic to set up the stack
    kprintf("Setting up stack with %d arguments\n", exec_args->argc);
    for (int i = 0; i < exec_args->argc; i++) {
        kprintf("Argument %d: %s\n", i, exec_args->argv[i]);
    }
}

void jump_to_entry_point(void *entry_point) {
    // Placeholder: Implement the logic to jump to the entry point
    kprintf("Jumping to entry point: %p\n", entry_point);
    __asm volatile("BX %0" : : "r"(entry_point));
}

int sys_execv(const char *filename, char *const argv[]) {
    // Step 1: Load the executable file into memory
    // (This is a placeholder; you need to implement the actual file loading logic)
    void *entry_point = load_executable(filename);
    if (entry_point == NULL) {
        kprintf("sys_execv: Failed to load executable '%s'\n", filename);
        return -1;  // Return -1 on failure
    }

    // Step 2: Set up the stack with the program's arguments
    ExecArgs exec_args;
    exec_args.argc = 0;

    // Copy arguments into the ExecArgs structure
    for (int i = 0; argv[i] != NULL && i < MAX_ARGS; i++) {
        exec_args.argv[i] = argv[i];
        exec_args.argc++;
    }
    exec_args.argv[exec_args.argc] = NULL;  // Null-terminate the argument list

    // Step 3: Prepare the stack for the new program
    // (This is a placeholder; you need to implement the stack setup logic)
    setup_stack(&exec_args);

    // Step 4: Jump to the entry point of the new program
    // (This is a placeholder; you need to implement the jump logic)
    jump_to_entry_point(entry_point);

    // If we reach here, something went wrong
    return -1;
}

void __init_dev_table(void){
    //initialize device_list.tref = 0
    for (int i = 0; i < 64; i++){
        device_list[i].t_ref = 0;
    }

    //init device_list[device_count]
    strcopy(device_list[device_count].name, "USART2");
    device_list[device_count].t_ref += 1;
    device_list[device_count].t_access = O_RDONLY;
    device_list[device_count].op_addr = USART2;
    device_count++;

    //init device_list[device_count] (usart2)
    strcopy(device_list[device_count].name, "USART2");
    device_list[device_count].t_ref += 1;
    device_list[device_count].t_access = O_WRONLY;
    device_list[device_count].op_addr = USART2;
    device_count++;

    //init device_list[device_count] (stderr)
    strcopy(device_list[device_count].name, "STDERR");
    device_list[device_count].t_ref += 1;
    device_list[device_count].t_access = O_WRONLY;
    device_list[device_count].op_addr = 0;
    device_count++;

}
void __sys_open(char *name,uint8_t t_access, uint32_t *op_addr){
    strcopy(device_list[device_count].name,name);
    device_list[device_count].t_ref++;
    device_list[device_count].t_access = t_access;
    device_list[device_count].op_addr = op_addr;
    device_count++;
}

void __sys_close(uint32_t *op_addr){
    uint32_t delete_index = -1;
    for (int i = 0; i < device_count; i++){
        if (device_list[i].op_addr == op_addr){
            device_list[i].t_ref--;
            if (device_list[i].t_ref == 0){
                delete_index = i;
            }
        }
    }
    if (delete_index != -1){
        for (int i = delete_index; i < device_count - 1; i++){
            device_list[i] = device_list[i+1];
        }
        device_count--;
    }
}

void __sys_reboot(void){
	SCB->AIRCR = 0x05FA0004;
}

void __sys_getpid(unsigned int *val,uint16_t value){
	*val = value;
	return ;
}

void __sys_get_time(uint32_t *time){
    *time = __getTime();
}

void task1(void) {
    while (1) {
        kprintf("Task 1 is running\n");
        kyield();  // Yield the CPU
    }
}

void task2(void) {
    while (1) {
        kprintf("Task 2 is running\n");
        kyield();  // Yield the CPU
    }
}

static MemoryBlock *heap_start = NULL;

void init_heap(void) {
    // Initialize the first block to cover the entire heap
    heap_start = (MemoryBlock *)&_sheap;
    heap_start->size = (uint32_t)&_eheap - (uint32_t)&_sheap;
    heap_start->is_free = 1;
    heap_start->next = NULL;

    kprintf("Heap initialized. Start address: %x, Size: %d bytes\n", heap_start, heap_start->size);
}

void *sys_malloc(uint32_t size) {
    // Ensure proper alignment (e.g., 8-byte alignment)
    size = (size + 7) & ~7;

    // Add the size of the header to the requested size
    uint32_t total_size = size + sizeof(MemoryBlock);

    // Traverse the linked list to find a free block
    MemoryBlock *current = heap_start;
    MemoryBlock *previous = NULL;

    while (current != NULL) {
        if (current->is_free && current->size >= total_size) {
            // Found a suitable block
            //kprintf("Found free block: %x, Size: %d\n", current, current->size);

            // Split the block if the remaining space is large enough
            if (current->size > total_size + sizeof(MemoryBlock)) {
                MemoryBlock *new_block = (MemoryBlock *)((uint8_t *)current + total_size);
                new_block->size = current->size - total_size;
                new_block->is_free = 1;
                new_block->next = current->next;

                current->size = total_size;
                current->next = new_block;
            }

            // Mark the block as allocated
            current->is_free = 0;

            // Return a pointer to the data area (after the header)
            void *allocated = (void *)((uint8_t *)current + sizeof(MemoryBlock));
            //kprintf("Allocated %d bytes at %x\n", size, allocated);
            return allocated;
        }

        // Move to the next block
        previous = current;
        current = current->next;
    }

    // No suitable block found
    kprintf("sys_malloc: Out of memory!\n");
    return NULL;
}

void sys_free(void *ptr) {
    if (ptr == NULL) {
        return;
    }

    // Get the block header from the data pointer
    MemoryBlock *block = (MemoryBlock *)((uint8_t *)ptr - sizeof(MemoryBlock));
    block->is_free = 1;

    kprintf("Freed block: %x, Size: %d\n", block, block->size);

    // Merge with the next block if it is free
    MemoryBlock *next_block = block->next;
    if (next_block != NULL && next_block->is_free) {
        block->size += next_block->size;
        block->next = next_block->next;
        kprintf("Merged with next block: %x\n", next_block);
    }

    // Merge with the previous block if it is free
    MemoryBlock *current = heap_start;
    MemoryBlock *previous = NULL;

    while (current != NULL) {
        if (current->next == block && current->is_free) {
            current->size += block->size;
            current->next = block->next;
            kprintf("Merged with previous block: %x\n", block);
            break;
        }
        previous = current;
        current = current->next;
    }
}

int sys_fork(void) {
    // Step 1: Allocate a new stack for the child process
    uint8_t *child_stack = (uint8_t *)sys_malloc(STACK_SIZE);
    if (child_stack == NULL) {
        kprintf("sys_fork: Failed to allocate stack for child process\n");
        return -1;  // Return -1 on failure
    }

    // Step 2: Copy the parent's stack into the child's stack
    uint8_t *parent_stack = (uint8_t *)current_task->psp;
    for (uint32_t i = 0; i < STACK_SIZE; i++) {
        child_stack[i] = parent_stack[i];
    }

    // Step 3: Create a new TCB for the child process
    TCB_TypeDef *child_tcb = (TCB_TypeDef *)sys_malloc(sizeof(TCB_TypeDef));
    if (child_tcb == NULL) {
        kprintf("sys_fork: Failed to allocate TCB for child process\n");
        sys_free(child_stack);  // Free the child stack on failure
        return -1;  // Return -1 on failure
    }

    // Initialize the child TCB
    child_tcb->magic_number = MAGIC_NUMBER;
    child_tcb->task_id = next_task_id++;
    child_tcb->psp = (void *)(child_stack + STACK_SIZE);  // PSP points to the top of the stack
    child_tcb->status = 0;  // Set status to "ready"
    child_tcb->execution_time = 0;
    child_tcb->waiting_time = 0;
    child_tcb->digital_signature = 0x00000001;

    // Step 5: Return the child's PID to the parent and 0 to the child
    if (current_task->task_id != 0) {
        // Parent process: Return the child's PID
        return child_tcb->task_id;
    } else {
        // Child process: Return 0
        return 0;
    }
}

void create_tasks(void) {
    TCB_TypeDef *task1_tcb = (TCB_TypeDef *)sys_malloc(sizeof(TCB_TypeDef));
    task1_tcb->task_id = 1000;
    task1_tcb->psp = (void *)(sys_malloc(1024) + 1024);  // Allocate stack for task1
    task1_tcb->status = TASK_READY;

    TCB_TypeDef *task2_tcb = (TCB_TypeDef *)sys_malloc(sizeof(TCB_TypeDef));
    task2_tcb->task_id = 1001;
    task2_tcb->psp = (void *)(sys_malloc(1024) + 1024);  // Allocate stack for task2
    task2_tcb->status = TASK_READY;

    scheduler_add_task(task1_tcb);
    scheduler_add_task(task2_tcb);
}


void syscall(unsigned int *svc_args)
{
/* The SVC_Handler calls this function to evaluate and execute the actual function */
/* Take care of return value or code */
    uint8_t callno;
    callno = ((uint8_t *)svc_args[6])[-2];
    uint32_t *op_addr;
	switch(callno)
	{
		/* Write your code to call actual function (kunistd.h/c or times.h/c and handle the return value(s) */
		case SYS_open:
			kprintf("Will call __sys_open\n");
			char *device_name = (char *)svc_args[0];
			uint8_t t_access = (uint8_t)svc_args[1];
			*op_addr = (uint32_t *)svc_args[2];
			kprintf("wot :0\n");
			__sys_open(device_name,t_access,op_addr);
			break;		
		case SYS_close:
			kprintf("Will call __sys_close\n");
			*op_addr = (uint32_t *)svc_args[0];
			__sys_close(op_addr);
			break;
        case SYS_read:
            char *format = (char *)svc_args[0];
            va_list *args_ptr = (va_list*)svc_args[1];  // Cast to va_list pointer
            va_list args;
            va_copy(args, *args_ptr);  // Copy contents properly
            kscanf(format, args);
            va_end(args);
            break;
        case SYS_write:
            
            uint8_t fd_w = svc_args[0];
            char* buffer_w = (char *)svc_args[1];
            uint32_t size_w = svc_args[3];
    
            __ISB();
            // kprintf("fd: %d\n", fd_w);
            // kprintf("buffer: %s\n", (char *)buffer_w);
            // kprintf("size: %d\n", size_w);
    
            Uart_sendstring(buffer_w,&huart2);
            // _USART_WRITE(USART2, (uint8_t*)buffer);
            
            break;
		case SYS_yield:
			SCB->ICSR |= (1 << 28);
			break;				
		case SYS__exit:
			TCB_TypeDef* task = svc_args[16];
			task->status = KILLED;
			break;
		case SYS_getpid:
			uint32_t pid = svc_args[10];	
			task = svc_args[16];
			__sys_getpid((unsigned int *)pid,task->task_id);
			break;
		case SYS_reboot:
			kprintf("Will call __sys_reboot\n");
			__sys_reboot();
			break;	
		case SYS___time:
			uint32_t time = svc_args[0];
			__sys_get_time((uint32_t *)time);
			break;	
		case SYS_malloc:
            void *allocated = sys_malloc(svc_args[0]);
            // Return the allocated address in svc_args[0]
            svc_args[0] = (uint32_t)allocated;
            break;
        case SYS_free:
            sys_free((void *)svc_args[0]);
            break;
		case SYS_fork:
            int child_pid = sys_fork();
            svc_args[0] = (uint32_t)child_pid;  
            break;
        case SYS_execv:
            const char *filename = (const char *)svc_args[0];
            char *const *argv = (char *const *)svc_args[1];
            int result = sys_execv(filename, argv);
            svc_args[0] = (uint32_t)result;
            break;
		/* return error code see error.h and errmsg.h ENOSYS sys_errlist[ENOSYS]*/	
		default: ;
	}
/* Handle SVC return here */
    return;
}

