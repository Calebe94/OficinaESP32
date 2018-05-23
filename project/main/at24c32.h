#ifndef _AT24C32_H_
#define _AT24C32_H_

void at24c_write_byte(uint16_t offset, uint8_t data);

void at24c_read_byte(uint16_t offset, uint8_t *data);

void clear_at24c(void);

#endif
