/*
 * bmp280_add.h
 *
 *  Created on: Nov 28, 2019
 *      Author: piotr
 */

int8_t BMP280_init(void);
int8_t BMP280_read(void);
void delay_ms(uint32_t period_ms);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(char * api_name, int8_t rslt);


