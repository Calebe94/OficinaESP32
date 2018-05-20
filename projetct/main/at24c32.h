#ifndef _AT24C32_H_
#define _AT24C32_H_

#include <stdint.h>
#include <driver/i2c.h>

#define AT2432_ADDRESS 		0x50
#define MAX_OFFSET 		 	4096

void at24c_write_byte(uint16_t offset, uint8_t data);

uint8_t at24c_read_byte(uint16_t offset);

void clear_at24c(void);

#endif
