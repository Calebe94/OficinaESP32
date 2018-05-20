#include "ds1307.h"
#include <freertos/FreeRTOS.h>

time_t get_time(void){
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (DS1307_ADDRESS<<1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, 0x0, 1);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (DS1307_ADDRESS<<1) | I2C_MASTER_READ,  1);

	uint8_t data[7];
	i2c_master_read(cmd, data, 7, 0);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	return (time_t)array_to_time(data);
}

void set_time(time_t time){
	struct tm tm;
	gmtime_r(&time, &tm);

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (DS1307_ADDRESS<<1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, 0x00, 1);
	i2c_master_write_byte(cmd, int_to_BCD(tm.tm_sec)&0x7F, 1);
	i2c_master_write_byte(cmd, int_to_BCD(tm.tm_min), 1);
	i2c_master_write_byte(cmd, int_to_BCD(tm.tm_hour), 1);
	i2c_master_write_byte(cmd, int_to_BCD(tm.tm_wday+1), 1);
	i2c_master_write_byte(cmd, int_to_BCD(tm.tm_mday), 1);
	i2c_master_write_byte(cmd, int_to_BCD(tm.tm_mon+1), 1);
	i2c_master_write_byte(cmd, int_to_BCD(tm.tm_year-100), 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}

time_t array_to_time(uint8_t data[]){
	struct tm time = {
		.tm_sec  = BCD_to_int(data[0]),
		.tm_min  = BCD_to_int(data[1]),
		.tm_hour = BCD_to_int(data[2]),
		.tm_mday = BCD_to_int(data[4]),
		.tm_mon  = BCD_to_int(data[5]) - 1,
		.tm_year = BCD_to_int(data[6]) + 100
	};
	return (time_t)mktime(&time);
}

uint8_t int_to_BCD(uint8_t integer){
	return ((integer/10)<<4) | (integer%10);
}

uint8_t BCD_to_int(uint8_t bcd){
	return ((bcd>>4)*10)+(bcd&0xF);
}

void setar_hora(uint8_t week_day, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint8_t year){
  struct tm time_to_set = {
    .tm_sec = second,
    .tm_min = minute,
    .tm_hour = hour,
    .tm_wday = week_day,
    .tm_mday = day,
    .tm_mon = month,
    .tm_year = year - 1900
  };
  set_time(mktime(&time_to_set));
}