#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include <inttypes.h>
#define KEY1 ((uint32_t)0x45670123)
#define KEY2 ((uint32_t)0xCDEF89AB)

void flash_unlock(void);
void flash_write(uint32_t address,uint32_t data);
uint32_t flash_read(uint32_t address);
void flash_erase_page(uint32_t address);
void flash_erase_all_pages(void);
uint8_t flash_ready(void);
void flash_lock(void);
