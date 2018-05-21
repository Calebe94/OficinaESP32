#ifndef _AT24C32_H_
#define _AT24C32_H_

#include <stdint.h>
#include <driver/i2c.h>

#define AT24C32 		0x50					/*!< AT24C32 device address*/
#define ACK_CHECK_EN   	0x1                     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  	0x0                     /*!< I2C master will not check ack from slave */
#define ACK_VAL        	0x0                     /*!< I2C ack value */
#define NACK_VAL       	0x1                     /*!< I2C nack value */
#define MAX_OFFSET		4096

void at24c_write_byte(uint16_t offset, uint8_t data);

void at24c_read_byte(uint16_t offset, uint8_t *data);

void clear_at24c(void);

#endif
