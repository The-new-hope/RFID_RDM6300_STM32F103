#include "header.h"

//Function unlock permittion to erase or write
void flash_unlock(void) {
	FLASH->KEYR = KEY1;
	FLASH->KEYR = KEY2;
}


//Function lock permittion to erase or write
void flash_lock() {
	FLASH->CR |= FLASH_CR_LOCK;
}

// Function return true when you can erase or write data.
uint8_t flash_ready(void) {
return !(FLASH->SR & FLASH_SR_BSY);
}

//Function erase all pages/ Be carefull
void flash_erase_all_pages(void) {
	FLASH->CR |= FLASH_CR_MER; //Set bit erase all pages
	FLASH->CR |= FLASH_CR_STRT; //Start erase
while(!flash_ready()) // Wait when will be ready
	FLASH->CR &= FLASH_CR_MER;
}
 
//Function erase one page. You should use one address of address range this page
void flash_erase_page(uint32_t address) {
	FLASH->CR|= FLASH_CR_PER; //Set bit erase one page
	FLASH->AR = address; // Set she address
	FLASH->CR|= FLASH_CR_STRT; // Start erase
while(!flash_ready()) //Wait when will be ready
	FLASH->CR&= ~FLASH_CR_PER; //Clear bit erase one page
}

void flash_write(uint32_t address,uint32_t data) {
	FLASH->CR |= FLASH_CR_PG; 		//Set permitiion for write
	while(!flash_ready()) 				//Wait when will be ready
	*(__IO uint16_t*)address = (uint16_t)data; //Write 2 byte LSB
	while(!flash_ready())					//Wait when will be ready
	address+=2;
	data>>=16;
	*(__IO uint16_t*)address = (uint16_t)data; //Write 2 byte MSB
	while(!flash_ready())
	FLASH->CR &= ~(FLASH_CR_PG); //Clear permitiion for write
}

uint32_t flash_read(uint32_t address) {
	return (*(__IO uint32_t*) address);
}


	
