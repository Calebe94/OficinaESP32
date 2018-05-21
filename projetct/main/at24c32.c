#include "at24c32.h"
#include <freertos/FreeRTOS.h>

void at24c_write_byte(uint16_t offset, uint8_t data){
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (AT24C32 << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (offset&0x0F00)>>8, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (offset&0xFF), ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (data&0xFF), ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	vTaskDelay(5/portTICK_PERIOD_MS);
}

void at24c_read_byte(uint16_t offset, uint8_t *data){
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (AT24C32 << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (offset&0x0F00)>>8, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, offset&0xFF, ACK_CHECK_EN));
	
	ESP_ERROR_CHECK(i2c_master_start(cmd));

	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (AT24C32 << 1) | I2C_MASTER_READ, ACK_CHECK_EN));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data, NACK_VAL));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	vTaskDelay(5/portTICK_PERIOD_MS);
}

void clear_at24c(void){
	uint8_t aux_data = 0;
	for(register uint16_t offset = 0; offset < MAX_OFFSET; offset++){
		at24c_read_byte(offset, &aux_data);
		if(aux_data != 0x00){
			at24c_write_byte(offset, 0x00);
			at24c_read_byte(offset, &aux_data);
			if(aux_data == 0x00)	printf("Address: %d cleared!\n", offset);
			else printf("Não Limpou");
		}else break;
	}
}