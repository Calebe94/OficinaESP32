#ifndef _DS1307_H_
#define _DS1307_H_

#include <stdint.h>
#include <time.h>
#include <driver/i2c.h>

#define SDA_PIN 		21
#define SCL_PIN 		22
#define DS1307_ADDRESS 	0x68

time_t get_time(void);

void set_time(time_t time);

time_t array_to_time(uint8_t data[]);

uint8_t int_to_BCD(uint8_t integer);

uint8_t BCD_to_int(uint8_t bcd);

void setar_hora(uint8_t week_day, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint8_t year);

#endif