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
 
#include "bootloader.h"
#include "cm4.h"
const uint32_t STACK_START = (uint32_t)SRAM_END;
uint32_t NVIC_VECTOR[] __attribute__((section (".isr_vector")))={
	STACK_START,
	(uint32_t) &Reset_Handler,
	(uint32_t) &NMI_Handler,
	(uint32_t) &HardFault_Handler,
	(uint32_t) &MemManage_Handler,
	(uint32_t) &BusFault_Handler,
	(uint32_t) &UsageFault_Handler,
	0,
	0,
	0,
	0,
	(uint32_t) &SVCall_Handler,
	(uint32_t) &DebugMonitor_Handler,
	0,
	(uint32_t) &PendSV_Handler,
	(uint32_t) &SysTick_Handler,
	(uint32_t) &WWDG_Handler,
	(uint32_t) &PVD_Handler,
	(uint32_t) &TAMP_STAMP_Handler,
	(uint32_t) &RTC_WKUP_Handler,
	(uint32_t) &FLASH_Handler,
	(uint32_t) &RCC_Handler,
	(uint32_t) &EXTI0_Handler,
	(uint32_t) &EXTI1_Handler,
	(uint32_t) &EXTI2_Handler,
	(uint32_t) &EXTI3_Handler,
	(uint32_t) &EXTI4_Handler,
	(uint32_t) &DMA1_Stream0_Handler,
	(uint32_t) &DMA1_Stream1_Handler,
	(uint32_t) &DMA1_Stream2_Handler,
	(uint32_t) &DMA1_Stream3_Handler,
	(uint32_t) &DMA1_Stream4_Handler,
	(uint32_t) &DMA1_Stream5_Handler,
	(uint32_t) &DMA1_Stream6_Handler,
	(uint32_t) &ADC_Handler,
	(uint32_t) &CAN1_TX_Handler,
	(uint32_t) &CAN1_RX0_Handler,
	(uint32_t) &CAN1_RX1_Handler,
	(uint32_t) &CAN1_SCE_Handler,
	(uint32_t) &EXTI9_5_Handler,
	(uint32_t) &TIM1_BRK_TIM9_Handler,
	(uint32_t) &TIM1_UP_TIM10_Handler,
	(uint32_t) &TIM1_TRG_COM_TIM11_Handler,
	(uint32_t) &TIM1_CC_Handler,
	(uint32_t) &TIM2_Handler,
	(uint32_t) &TIM3_Handler,
	(uint32_t) &TIM4_Handler,
	(uint32_t) &I2C1_EV_Handler,
	(uint32_t) &I2C1_ER_Handler,
	(uint32_t) &I2C2_EV_Handler,
	(uint32_t) &I2C2_ER_Handler,
	(uint32_t) &SPI1_Handler,
	(uint32_t) &SPI2_Handler,
	(uint32_t) &USART1_Handler,
	(uint32_t) &USART2_Handler,
	(uint32_t) &USART3_Handler,
	(uint32_t) &EXTI15_10_Handler,
	(uint32_t) &RTC_Alarm_Handler,
	(uint32_t) &OTG_FS_WKUP_Handler,
	(uint32_t) &TIM8_BRK_TIM12_Handler,
	(uint32_t) &TIM8_UP_TIM13_Handler,
	(uint32_t) &TIM8_TRG_COM_TIM14_Handler,
	(uint32_t) &TIM8_CC_Handler,
	(uint32_t) &DMA1_Stream7_Handler,
	(uint32_t) &FMC_Handler,
	(uint32_t) &SDIO_Handler,
	(uint32_t) &TIM5_Handler,
	(uint32_t) &SPI3_Handler,
	(uint32_t) &UART4_Handler,
	(uint32_t) &UART5_Handler,
	(uint32_t) &TIM6_DAC_Handler,
	(uint32_t) &TIM7_Handler,
	(uint32_t) &DMA2_Stream0_Handler,
	(uint32_t) &DMA2_Stream1_Handler,
	(uint32_t) &DMA2_Stream2_Handler,
	(uint32_t) &DMA2_Stream3_Handler,
	(uint32_t) &DMA2_Stream4_Handler,
	0,
	0,
	(uint32_t) &CAN2_TX_Handler,
	(uint32_t) &CAN2_RX0_Handler,
	(uint32_t) &CAN2_RX1_Handler,
	(uint32_t) &CAN2_SCE_Handler,
	(uint32_t) &OTG_FS_Handler,
	(uint32_t) &DMA2_Stream5_Handler,
	(uint32_t) &DMA2_Stream6_Handler,
	(uint32_t) &DMA2_Stream7_Handler,
	(uint32_t) &USART6_Handler,
	(uint32_t) &I2C3_EV_Handler,
	(uint32_t) &I2C3_ER_Handler,
	(uint32_t) &OTG_HS_EP1_OUT_Handler,
	(uint32_t) &OTG_HS_EP1_IN_Handler,
	(uint32_t) &OTG_HS_WKUP_Handler,
	(uint32_t) &OTG_HS_Handler,
	(uint32_t) &DCMI_Handler,
	0,
	0,
	(uint32_t) &FPU_Handler,
	0,
	0,
	(uint32_t) &SPI4_Handler,
	0,
	0,
	(uint32_t) &SAI1_Handler,
	0,
	0,
	0,
	(uint32_t) &SAI2_Handler,
	(uint32_t) &QuadSPI_Handler,
	(uint32_t) &HDMI_CEC_Handler,
	(uint32_t) &SPDIF_Rx_Handler,
	(uint32_t) &FMPI2C1_Handler,
	(uint32_t) &FMPI2C1_ERR_Handler
};

void Reset_Handler(void){
	uint32_t size = (uint32_t)&_edata - (uint32_t)&_sdata;
	uint8_t *pDst = (uint8_t*)&_sdata;
	uint8_t *pSrc = (uint8_t*)&_la_data;
	for(uint32_t i=0;i<size;i++){
		*pDst++ = *pSrc++;
	}
	size = (uint32_t)&_ebss - (uint32_t)&_sbss;
	pDst = (uint8_t*)&_sbss;
	for(uint32_t i=0;i<size;i++){
		*pDst++ = 0;
	}
	_text_size = (uint32_t)&_etext - (uint32_t)&_stext;
	_data_size = (uint32_t)&_edata - (uint32_t)&_sdata;
	_bss_size = (uint32_t)&_ebss - (uint32_t)&_sbss;
	uart_config();
	__ISB();

	// Flash_Unlock();
	// Flash_EraseSector(5,5);
	// uint32_t address=OS_START_ADDRESS;
	// uint8_t data[]={0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,0x9,0xA,0xB,0xC,0xD,0xE,0xF,
	// 				0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,0x9,0xA,0xB,0xC,0xD,0xE,0xF};
	// for(uint32_t i=0;i<32;i+=1){
	// 	//uint16_t data16 = (data[i] << 8) | data[i + 1];
    
	// 	// Write the 8-bit data to flash
	// 	Flash_Write(address, data[i]);

	// 	// Move to the next 16-bit aligned address
	// 	address += 1;
	// }
	// flash_lock();
	// set_OS_version(0,0);
	

	if(bootloader_check_for_update()){
		os_update();
		uint8_t response_buffer[4] = {0};
		UART_GetString(USART2, 3, response_buffer);
		uint32_t major = (response_buffer[0] - '0');
		uint32_t minor = (response_buffer[2] - '0');
		set_OS_version(major,minor);
		bootloader_jump_to_os();
	}else{
		bootloader_jump_to_os();
	}

}

uint8_t bootloader_check_for_update(void){
	uint32_t major = get_version_major();
	uint32_t minor = get_version_minor();
	char verson[22];
	format_version_string(verson,major,minor);
	UART_SendString(USART2,verson);
	uint8_t response_buffer[4];  // Adjusted for 3 characters
    UART_GetString(USART2, 3, response_buffer);  // Receive exactly 3 characters
    
    // Convert the response to an integer
    uint32_t response_code = (response_buffer[0] - '0') * 100 +
                        (response_buffer[1] - '0') * 10 +
                        (response_buffer[2] - '0');
    
    if (response_code == UPDATE_AVAILABLE) {
        return 1;  // Update available
    }

    return 0;  // No update
}


void os_update(void) {
    UART_SendString(USART2, FILE_SIZE_REQ);

    // Receive the file size from the server as a 32-bit integer
    uint8_t size_buffer[4]={0};
    UART_GetString(USART2, 4, size_buffer);  // Receiving file size as a 4-byte integer

    uint32_t file_size = (size_buffer[0] << 24) |
                         (size_buffer[1] << 16) |
                         (size_buffer[2] << 8) |
                         (size_buffer[3]);

	UART_SendString(USART2, START_REQ);

	Flash_Unlock(); // Unlock Flash for writing
    Flash_EraseSector(2,4); // Erase the sector where OS will be stored

	uint32_t address = OS_START_ADDRESS;
    uint32_t received_bytes = 0;
	uint8_t packet_buffer[PACKET_SIZE];

	// Enable CRC clock
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;

    while (received_bytes < file_size) {
        uint32_t bytes_to_read = (file_size - received_bytes) < (CHUNK_SIZE + 4) ? (file_size - received_bytes + 4) : PACKET_SIZE;
        
        UART_GetString(USART2, bytes_to_read, packet_buffer);

		uint32_t chunk_size = bytes_to_read - 4; // Exclude 4 CRC bytes

		// Extract the CRC (last 4 bytes) from the packet
		uint32_t received_chunk_crc = (packet_buffer[chunk_size]     << 24) |
									(packet_buffer[chunk_size + 1] << 16) |
									(packet_buffer[chunk_size + 2] << 8)  |
									(packet_buffer[chunk_size + 3]);


		// uint32_t crc= crc32_manual(chunk_buffer, bytes_to_read);

		// Read the computed CRC
		uint32_t computed_chunk_crc = crc32_hardware(packet_buffer, chunk_size);


		// Verify the CRC for the current chunk
        if (computed_chunk_crc != received_chunk_crc) {
            UART_SendString(USART2, ERROR_CODE); // Notify the server about the CRC error
			char crc_string[12]; // 12 characters for hex + 1 for null terminator
			uint32_to_string(computed_chunk_crc, crc_string);
			uint8_t c[5]={0};
			UART_GetString(USART2,5,c);
			UART_SendString(USART2, crc_string); //For debug, sending computed crc
            continue; 
        }

        received_bytes += chunk_size;

		//Write the chunk to Flash, 1 bytes at a time
        for (uint32_t i = 0; i < chunk_size; i += 1) {
			Flash_Write(address, packet_buffer[i]);  
			address+=1;
		}
        UART_SendString(USART2, ACKNOWLEDGEMENT);
    }

	flash_lock();
	// Disable CRC clock after use
	RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;
}

uint32_t crc32_hardware(uint8_t *packet_buffer, uint32_t chunk_size){
	// Reset CRC->CR
	CRC->CR |= CRC_CR_RESET;

	// Feed the data into the CRC unit
	for (uint32_t i = 0; i < chunk_size; i++) {
		CRC->DR = packet_buffer[i];
	}

	return CRC->DR;
}

uint32_t crc32_manual(uint8_t *data, uint32_t length) {
    uint32_t crc = 0xFFFFFFFF;
	uint32_t POLYNOMIAL = 0x04C11DB7;

    for (size_t i = 0; i < length; i++) {
        crc ^= (data[i] << 24); // XOR the byte shifted into the high byte of the CRC

        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80000000) {
                crc = ((crc << 1) ^ POLYNOMIAL); // Apply polynomial if MSB is 1
            } else {
                crc = crc << 1; // Shift left if MSB is 0
            }
            crc &= 0xFFFFFFFF; // Ensure CRC remains 32-bit
        }
    }

    return crc;
}

void uint32_to_string(uint32_t value, char* buffer) {
    int i = 0;
    
    // Handle the special case where the value is 0
    if (value == 0) {
        buffer[i++] = '0';
        buffer[i] = '\0';
        return;
    }

    // Process the digits of the number in reverse order
    while (value > 0) {
        buffer[i++] = (value % 10) + '0';  // Get the last digit and convert to character
        value /= 10;                       // Remove the last digit
    }

    buffer[i] = '\0';  // Null-terminate the string

    // Reverse the string to get the correct order
    for (int j = 0; j < i / 2; j++) {
        char temp = buffer[j];
        buffer[j] = buffer[i - 1 - j];
        buffer[i - 1 - j] = temp;
    }
}

void format_version_string(char *buffer, uint32_t major, uint32_t minor) {
    char major_str[12];  // To hold the major version as a string
    char minor_str[12];  // To hold the minor version as a string

    // Convert major and minor to strings
    uint32_to_string(major, major_str);
    uint32_to_string(minor, minor_str);

    // Construct the version string
    char *ptr = buffer;
    int i = 0;

    // Copy the major version string to the buffer
    while (major_str[i] != '\0') {
        *ptr++ = major_str[i++];
    }

    *ptr++ = '.';  // Add the dot separator

    // Reset index and copy the minor version string to the buffer
    i = 0;
    while (minor_str[i] != '\0') {
        *ptr++ = minor_str[i++];
    }

    *ptr = '\0';  // Null-terminate the string
}


void Flash_Unlock(void) {
    if ((FLASH->CR & FLASH_CR_LOCK) != 0) { // Check if Flash is locked
        FLASH->KEYR =  0x45670123U;          // Write first key
        FLASH->KEYR = 0xCDEF89ABU;           // Write second key
    }
}

void flash_lock(void) {
    FLASH->CR |= FLASH_CR_LOCK;  // Set the lock bit
}

void Flash_EraseSector(uint8_t startSector, uint8_t endSector) {

	for (uint8_t sectorNumber = startSector; sectorNumber <= endSector; sectorNumber++){
    // Step 1: Check that no Flash memory operation is ongoing
		while (FLASH->SR & FLASH_SR_BSY);  // Wait if Flash is busy

		// Step 2: Set the SER bit and select the sector to erase
		FLASH->CR &= ~FLASH_CR_SNB;              // Clear the sector number bits
		FLASH->CR |= FLASH_CR_SER | (sectorNumber << FLASH_CR_SNB_Pos); // Set SER and sector number

		// Step 3: Set the STRT bit to start the erase operation
		FLASH->CR |= FLASH_CR_STRT;

		// Step 4: Wait for the erase operation to complete
		while (FLASH->SR & FLASH_SR_BSY);  // Wait until the BSY bit is cleared

		// Clear the SER bit and SNB bits for safety (optional but good practice)
		FLASH->CR &= ~FLASH_CR_SER;
		FLASH->CR &= ~FLASH_CR_SNB;
	}
}


void Flash_Write(uint32_t address, uint8_t data) {
    // Step 1: Wait if Flash is busy
    while (FLASH->SR & FLASH_SR_BSY);

    // Step 2: Clear all error flags
    FLASH->SR |= FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR;

    // Step 3: Set programming size to half-word (16 bits)
    FLASH->CR &= ~FLASH_CR_PSIZE;               // Clear PSIZE bits
    //FLASH->CR |= FLASH_CR_PSIZE_0;              // Set PSIZE to 16 bits

    // Step 4: Enable programming mode
    FLASH->CR |= FLASH_CR_PG;

    // Step 5: Perform the write operation
   	//(* (volatile uint32_t*) address) = data;
	volatile uint8_t *flash_address = (volatile uint8_t *)address;
    *flash_address = data;

    // Step 6: Wait until programming is complete
    while (FLASH->SR & FLASH_SR_BSY);

    // Step 7: Clear the PG bit to exit programming mode
    FLASH->CR &= ~FLASH_CR_PG;
}

void Flash_write4bytes(uint32_t address, uint32_t data){
    // Step 1: Wait if Flash is busy
    while (FLASH->SR & FLASH_SR_BSY);

    // Step 2: Clear all error flags
    FLASH->SR |= FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR;

    // Step 3: Set programming size to word (32 bits)
    FLASH->CR &= ~FLASH_CR_PSIZE;               // Clear PSIZE bits
    FLASH->CR |= FLASH_CR_PSIZE_1;              // Set PSIZE to 32 bits (binary: 10)

    // Step 4: Enable programming mode
    FLASH->CR |= FLASH_CR_PG;

    // Step 5: Perform the write operation
    *(volatile uint32_t*)address = data;

    // Step 6: Wait until programming is complete
    while (FLASH->SR & FLASH_SR_BSY);

    // Step 7: Check for errors
    if (FLASH->SR & (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR)) {
        // Handle error (optional)
        FLASH->SR |= FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR;  // Clear error flags
    }

    // Step 8: Clear the PG bit to exit programming mode
    FLASH->CR &= ~FLASH_CR_PG;
}


void bootloader_jump_to_os(void){
    uint32_t os_start_address= OS_START_ADDRESS;
    // Relocate the vector table to the application's start address
    SCB->VTOR = os_start_address;
    uint32_t os_reset_handler = *(volatile uint32_t*)(os_start_address + 4U);


    // Jump to application reset handler
    void (*os_entry)(void) = (void(*)(void))os_reset_handler;
    os_entry();
}

void uart_config(void){
	__init_sys_clock();
	__ISB();

	//1. Enable UART clock and GPIO clock
	RCC->APB1ENR |= (1<<17); //enable UART 2
	RCC->AHB1ENR |= (1<<0); //enable GPIOA clock
	
		
	//2. Configure UART pin for Alternate function
	GPIOA->MODER |= (2<<4); //bits [5:4] -> 1:0 -->Alternate function for pin PA2
	GPIOA->MODER |= (2<<6); //bits [7:6] -> 1:0 -->Alternate function for PA3
	
	GPIOA->OSPEEDR |= (3<<4) | (3<<6); //bits [5:4] -> 1:1 -> high speed PA2; bits [7:6] -> 1:1 -> high speed PA3 
	
	GPIOA->AFR[0] |= (7<<8);//Bytes (11:10:09:08) = 0:1:1:1 --> AF7 Alternate function for USART2 at pin PA2
	GPIOA->AFR[0] |= (7<<12); //Bytes (15:14:13:12) = 0:1:1:1 --> AF7 Alternate function for USART2 at pin PA3
	
	//3. Enable UART on USART_CR1 rgister
	USART2->CR1 = 0x00; //clear USART
	USART2->CR1 |= (1<<13);  // UE-bit enable USART
	
	//4. Program M bit in USART CR1 to define the word length
	USART2->CR1 &= ~(1U<<12); // set M bit  = 0 for 8-bit word length
	
	//5. Select the baud rate using the USART_BRR register.
	USART2->BRR |= (7<<0) | (24<<4); //115200
	
	//  6. Enable transmission TE and recieption bits in USART_CR1 register
	USART2->CR1 |= (1<<2); // enable RE for receiver 
	USART2->CR1 |= (1<<3); //enable TE for transmitter
}

void UART_SendChar(USART_TypeDef *usart,uint8_t c){
	usart->DR = c;
	while(!(usart->SR & (1<<7)));
}

void UART_SendString(USART_TypeDef *usart,const char *s){
	while (*s) {UART_SendChar(usart,*s);s++;}
}

uint8_t UART_GetChar(USART_TypeDef *usart){
	uint8_t tmp;
	while(!(usart->SR & (1<<5)));
	tmp=(uint8_t)usart->DR;
	return tmp;
}

void UART_GetString(USART_TypeDef *uart,uint16_t size,uint8_t* buff){
	uint16_t i=0;
	while(size--)
	{
		uint8_t x=UART_GetChar(uart);
		buff[i]=x;
		i++;
	}
	buff[i]='\0';
		
}

uint32_t get_version_major(void){
	return *(volatile uint32_t*) OS_VERSION_ADDRESS;
}

uint32_t get_version_minor(void){
	return *(volatile uint32_t*) (OS_VERSION_ADDRESS + 4U);
}

void set_OS_version(uint32_t major, uint32_t minor){
	Flash_Unlock();
	Flash_EraseSector(5,5);
	// Write the major version (32 bits) at OS_VERSION_ADDRESS
    Flash_write4bytes(OS_VERSION_ADDRESS, major);
	Flash_write4bytes(OS_VERSION_ADDRESS + 4U, minor);

	flash_lock();
}

void Default_Handler(void){
	while(1);
}
//2. implement the fault handlers
void HardFault_Handler(void)
{
//	printf("Exception : Hardfault\n");
	while(1);
}


void MemManage_Handler(void)
{
//	printf("Exception : MemManage\n");
	while(1);
}

void BusFault_Handler(void)
{
//	printf("Exception : BusFault\n");
	while(1);
}

void SVCall_Handler(void){
/* Write code for SVC handler */
/* the handler function evntually call syscall function with a call number */


}


