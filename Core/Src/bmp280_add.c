/*
 * bmp280_add.c
 *
 *  Created on: Nov 28, 2019
 *      Author: piotr
 */

#include "stdio.h"
#include "i2c.h"
#include "usart.h"
#include "bmp280.h"
#include "bmp280_add.h"

struct bmp280_dev bmp;

int8_t BMP280_init(void) {
	int8_t rslt;
    struct bmp280_config conf;

    /* Map the delay function pointer with the function responsible for implementing the delay */
    bmp.delay_ms = delay_ms;

    /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
    bmp.dev_id = (BMP280_I2C_ADDR_PRIM<<1);

    /* Select the interface mode as I2C */
    bmp.intf = BMP280_I2C_INTF;

    /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
    bmp.read = i2c_reg_read;
    bmp.write = i2c_reg_write;

    /* To enable SPI interface: comment the above 4 lines and uncomment the below 4 lines */

    /*
     * bmp.dev_id = 0;
     * bmp.read = spi_reg_read;
     * bmp.write = spi_reg_write;
     * bmp.intf = BMP280_SPI_INTF;
     */
    rslt = bmp280_init(&bmp);
    print_rslt(" bmp280_init status", rslt);

    /* Always read the current settings before writing, especially when
     * all the configuration is not modified
     */
    rslt = bmp280_get_config(&conf, &bmp);
    print_rslt(" bmp280_get_config status", rslt);

    /* configuring the temperature oversampling, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    conf.filter = BMP280_FILTER_COEFF_2;

    /* Pressure oversampling set at 4x */
    conf.os_pres = BMP280_OS_4X;

    /* Temperature oversampling set at 4x */
    conf.os_temp = BMP280_OS_4X;

    /* Setting the output data rate as 1HZ(1000ms) */
    conf.odr = BMP280_ODR_1000_MS;
    rslt = bmp280_set_config(&conf, &bmp);
    print_rslt(" bmp280_set_config status", rslt);

    /* Always set the power mode after setting the configuration */
    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
    print_rslt(" bmp280_set_power_mode status", rslt);

    return rslt;

}

int8_t BMP280_read(void) {
	 int8_t rslt;
     struct bmp280_uncomp_data ucomp_data;
     uint32_t pres32, pres64, temp32;
     double pres, temp;

	 /* Pobranie danych z czujnika */
     rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);

     /* Konwersja danych na liczby 32-bitowe */
     /* rslt = bmp280_get_comp_pres_32bit(&pres32, ucomp_data.uncomp_press, &bmp);
      * rslt = bmp280_get_comp_temp_32bit(&temp32, ucomp_data.uncomp_temp, &bmp);
      */

     /* Konwersja na liczby zmiennoprzecinkowe */
     rslt = bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
     rslt = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);

     /* Wyswietlenie wartosci */
     printf("Temperatura: %f C Cisnienie: %f Pa \r\n", temp, pres);

     return rslt;
}

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as "bmg250_soft_reset",
 *      "bmg250_set_foc", "bmg250_perform_self_test"  and so on.
 *
 *  @param[in] period_ms  : the required wait time in milliseconds.
 *  @return void.
 *
 */
void delay_ms(uint32_t period_ms)
{
    /* Implement the delay routine according to the target machine */
	HAL_Delay(period_ms);
}

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Implement the I2C write routine according to the target machine. */
	HAL_I2C_Mem_Write(&hi2c1, i2c_addr, reg_addr, 1, reg_data, length, 100);
    return 0;
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Implement the I2C read routine according to the target machine. */
	HAL_I2C_Mem_Read(&hi2c1, i2c_addr, reg_addr, 1, reg_data, length, 100);
    return 0;
}

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] cs           : Chip select to enable the sensor.
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose data has to be written.
 *  @param[in] length       : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void print_rslt(char * api_name, int8_t rslt)
{
    if (rslt != BMP280_OK)
    {
    	printf("%s\t", api_name);
        if (rslt == BMP280_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer error\r\n", rslt);
        }
        else if (rslt == BMP280_E_COMM_FAIL)
        {
            printf("Error [%d] : Bus communication failed\r\n", rslt);
        }
        else if (rslt == BMP280_E_IMPLAUS_TEMP)
        {
            printf("Error [%d] : Invalid Temperature\r\n", rslt);
        }
        else if (rslt == BMP280_E_DEV_NOT_FOUND)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}
