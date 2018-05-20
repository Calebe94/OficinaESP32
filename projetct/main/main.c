#include <stdio.h>
#include <time.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ds1307.h"
#include "at24c32.h"

#define TIME_OFFSET MAX_OFFSET/sizeof(time_t)

void i2c0_init(void){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void save_to_eeprom(time_t time){
    uint8_t byte_1 = 0, byte_2 = 0, byte_3 = 0, byte_4 = 0;
    byte_1 = (uint8_t)((time&0xFF000000)>>24);
	byte_2 = (uint8_t)((time&0x00FF0000)>>16);
	byte_3 = (uint8_t)((time&0x0000FF00)>>8);
	byte_4 = (uint8_t)(time&0x000000FF);

    for( uint16_t offset = 0; offset < MAX_OFFSET; offset++){
        if(at24c_read_byte(offset) == 0x00){
			at24c_write_byte(offset++, (byte_1&0xFF));
			at24c_write_byte(offset++, (byte_2&0xFF));
			at24c_write_byte(offset++, (byte_3&0xFF));
			at24c_write_byte(offset,   (byte_4&0xFF));
            break;
        }
    }
}

void get_from_eeprom(){
    time_t aux_time = 0;
    uint8_t byte_1 = 0, byte_2 = 0, byte_3 = 0, byte_4 = 0;
    for( uint16_t offset = 0; offset < MAX_OFFSET; offset++){
        if(at24c_read_byte(offset) != 0x00){
            byte_1 = at24c_read_byte((offset++));
            byte_2 = at24c_read_byte((offset++));
            byte_3 = at24c_read_byte((offset++));
            byte_4 = at24c_read_byte((offset));
            aux_time = (byte_1<<24) | (byte_2<<16) | (byte_3<<8) | byte_4;
            printf("%s\n", ctime(&aux_time));
        }else break;
    }
}

void app_main(void){
    gpio_pad_select_gpio(GPIO_NUM_32);
    gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT);
    gpio_pad_select_gpio(GPIO_NUM_33);
    gpio_set_direction(GPIO_NUM_33, GPIO_MODE_INPUT);
    i2c0_init();
    // clear_at24c();
    while (1) {
        time_t time = 0;
        if(gpio_get_level(GPIO_NUM_32)){
            while(gpio_get_level(GPIO_NUM_32));
            time = get_time();
            printf("%s\n", ctime(&time));
            save_to_eeprom(time);
        } 
        if(gpio_get_level(GPIO_NUM_33)){
            while(gpio_get_level(GPIO_NUM_33));
            get_from_eeprom();
            // printf("GPIO33\n");
        }
        vTaskDelay(10);
    }
}

