#include "at24c32.h"
#include <freertos/FreeRTOS.h>

void at24c_write_byte(uint16_t offset, uint8_t data){
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (AT2432_ADDRESS<<1)|I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, (offset&0x0F00)>>8, 1);
	i2c_master_write_byte(cmd, (offset&0xFF), 1);
	i2c_master_write_byte(cmd, (data&0xFF), 0);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	// i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	// ESP_ERROR_CHECK(i2c_master_start(cmd));
	// ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (AT2432_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	// ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (offset&0x0F00)>>8, 1));
	// ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (offset&0xFF), 1));
	// ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (data&0xFF), 1));
	// ESP_ERROR_CHECK(i2c_master_stop(cmd));
	// i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	// i2c_cmd_link_delete(cmd);
	// vTaskDelay(50/portTICK_PERIOD_MS);
}

uint8_t at24c_read_byte(uint16_t offset){
	uint8_t data;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (AT2432_ADDRESS<<1)|I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, (offset&0x0F00)>>8, 1);
	i2c_master_write_byte(cmd, (offset&0xFF), 1);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (AT2432_ADDRESS<<1)|I2C_MASTER_READ, 1);
	i2c_master_read_byte(cmd, &data, 0);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	return data;
}

void clear_at24c(void){
	for(register uint16_t offset = 0; offset < MAX_OFFSET; offset++){
		at24c_write_byte(offset, 0x00);
		if(at24c_read_byte(offset) == 0){
			printf("Address: %d cleared!\n", offset);
		}
	}
}