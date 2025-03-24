#include<userlib.h>
#include<stdint.h>
#include<kstdio.h>
#include<syscall_def.h>
#include<stdarg.h>

int kopen(const char *device_name, uint8_t t_access, uint32_t *op_addr) {
    int result;
    __asm volatile(
        "mov r0, %1\n"  // Move device_name to r0
        "mov r1, %2\n"  // Move t_access to r1
        "mov r2, %3\n"  // Move op_addr to r2
        "svc %4\n"      // Trigger SVC with syscall number
        "mov %0, r0\n"  // Move result to result
        : "=r"(result)  // Output
        : "r"(device_name), "r"(t_access), "r"(op_addr), "i"(SYS_open)  // Inputs
        : "r0", "r1", "r2", "memory"
    );
    return result;
}

int kclose(uint32_t *op_addr) {
    int result;
    __asm volatile(
        "mov r0, %1\n"  // Move op_addr to r0
        "svc %2\n"      // Trigger SVC with syscall number
        "mov %0, r0\n"  // Move result to result
        : "=r"(result)  // Output
        : "r"(op_addr), "i"(SYS_close)  // Inputs
        : "r0", "memory"
    );
    return result;
}

int kread(const char *format, ...) {
    va_list args;
    va_start(args, format);
    int result;
    __asm volatile(
        "mov r0, %1\n"  // Move format to r0
        "mov r1, %2\n"  // Move args to r1
        "svc %3\n"      // Trigger SVC with syscall number
        "mov %0, r0\n"  // Move result to result
        : "=r"(result)  // Output
        : "r"(format), "r"(&args), "i"(SYS_read)  // Inputs
        : "r0", "r1", "memory"
    );
    va_end(args);
    return result;
}

int kwrite(uint8_t fd, const char *buffer, uint32_t size) {
    int result;
    __asm volatile(
        "mov r0, %1\n"  // Move fd to r0
        "mov r1, %2\n"  // Move buffer to r1
        "mov r2, %3\n"  // Move size to r2
        "svc %4\n"      // Trigger SVC with syscall number
        "mov %0, r0\n"  // Move result to result
        : "=r"(result)  // Output
        : "r"(fd), "r"(buffer), "r"(size), "i"(SYS_write)  // Inputs
        : "r0", "r1", "r2", "memory"
    );
    return result;
}

void kyield(void) {
    __asm volatile(
        "svc %0\n"  // Trigger SVC with syscall number
        :
        : "i"(SYS_yield)  // Input
        : "memory"
    );
}

void kexit(void) {
    __asm volatile(
        "svc %0\n"  // Trigger SVC with syscall number
        :
        : "i"(SYS__exit)  // Input
        : "memory"
    );
}

int kgetpid(void) {
    int pid;
    __asm volatile(
        "svc %1\n"      // Trigger SVC with syscall number
        "mov %0, r0\n"  // Move result to pid
        : "=r"(pid)     // Output
        : "i"(SYS_getpid)  // Input
        : "r0", "memory"
    );
    return pid;
}

uint32_t kgettime(void) {
    uint32_t time;
    __asm volatile(
        "svc %1\n"      // Trigger SVC with syscall number
        "mov %0, r0\n"  // Move result to time
        : "=r"(time)    // Output
        : "i"(SYS___time)  // Input
        : "r0", "memory"
    );
    return time;
}


void* kmalloc(uint32_t size) {
    void *ptr;
    __asm volatile(
        "mov r0, %0\n"
        "PUSH {r4-r11, ip, lr}\n"
        "svc %1\n"
        "POP {r4-r11, ip, lr}\n"
        :
        : "r" (size), "i" (SYS_malloc)
    );

    __asm volatile(
        "mov %0, r0\n"
        : "=r"(ptr)
    );

    return ptr;
}

void kfree(void *ptr) {
    __asm volatile(
        "mov r0, %0\n"  // Move pointer to r0
        "svc %1\n"      // Trigger SVC with syscall number
        :
        : "r"(ptr), "i"(SYS_free)  // Inputs
        : "r0", "memory"
    );
}

int kfork(void) {
    int child_pid;
    __asm volatile(
        "svc %1\n"      // Trigger SVC with syscall number
        "mov %0, r0\n"  // Move result (child PID) to child_pid
        : "=r"(child_pid)  // Output
        : "i"(SYS_fork)    // Input
        : "r0", "memory"
    );
    return child_pid;
}

int kexecv(const char *filename, char *const argv[]) {
    int result;
    __asm volatile(
        "mov r0, %1\n"  // Move filename to r0
        "mov r1, %2\n"  // Move argv to r1
        "svc %3\n"      // Trigger SVC with syscall number
        "mov %0, r0\n"  // Move result to result
        : "=r"(result)  // Output
        : "r"(filename), "r"(argv), "i"(SYS_execv)  // Inputs
        : "r0", "r1", "memory"
    );
    return result;
}


void userProgram(){
    kprintf("*** In User Program ***\n");

    /*Open a Device*/
    // uint32_t op_addr;
    // int fd = kopen("/dev/uart", 0, &op_addr);
    // if (fd < 0) {
    //     kprintf("Failed to open device\n");
    // } else {
    //     kprintf("Device opened with fd: %d\n", fd);
    // }

    // /*Write to a device*/
    // const char *message = "Hello, World!\n";
    // kwrite(fd, message, strlen(message));

    /*Read from a device*/
    // char buffer[64];
    // kread("%s", buffer);
    // kprintf("Read from device: %s\n", buffer);

    /*Close the device*/
    // kclose(&op_addr);

    /*Yield the CPU*/
    // kyield();

    /*Get the current process ID*/
    // int pid = kgetpid();
    // kprintf("Current PID: %d\n", pid);

    /*Get the current time*/
    // uint32_t time = kgettime();
    // kprintf("Current time: %d\n", time);

    /*Reboot the system*/
    // kprintf("Rebooting the system...\n");
    // kreboot();

    /*Allocating Heap*/
    uint32_t *ptr1 = (uint32_t *)kmalloc(64);
    if (ptr1 == NULL) {
        kprintf("Memory allocation failed for ptr1!\n");
    } else {
        kprintf("Allocated address for ptr1: %x\n", ptr1);
    }
    *ptr1=10;
    kprintf("Value in heap is %d at address %x\n",*ptr1,ptr1);
    uint32_t *ptr2 = (uint32_t *)kmalloc(12);
    if (ptr2 == NULL) {
        kprintf("Memory allocation failed for ptr2!\n");
    } else {
        kprintf("Allocated address for ptr2: %x\n", ptr2);
    }
    
    /*Deallocating Heap*/
    kprintf("Freeing ptr1\n");
    kfree(ptr1);

    /*fork*/
    kprintf("Forking process\n");

    int child_pid = kfork();
    if (child_pid == 0) {
        kprintf("Child process: PID = %d\n", child_pid);
    } else if (child_pid > 0) {
        kprintf("Parent process: Child PID = %d\n", child_pid);
    } else {
        kprintf("Fork failed!\n");
    }
    int child_pid1 = kfork();
    if (child_pid1 == 0) {
        kprintf("Child process: PID = %d\n", child_pid1);
    } else if (child_pid1 > 0) {
        kprintf("Parent process: Child PID = %d\n", child_pid1);
    } else {
        kprintf("Fork failed!\n");
    }


    // /*Execv*/
    // // const char *filename = "my_program";
    // // char *const argv[] = {"arg1", "arg2", NULL};

    // // int result = kexecv(filename, argv);
    // // if (result == -1) {
    // //     kprintf("Execv failed!\n");
    // // }
}