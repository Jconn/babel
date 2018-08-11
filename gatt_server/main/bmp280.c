#include <stdlib.h>
#include <stdio.h>
#include "driver/adc.h"
#include <string.h>

#include "driver/i2c.h"
#include "sample_capture.h"
#include "eeprom_config.h"
#include "sensor_drive_constructor.h"
#include "bmp280.h"
/*
 - master:
 *    GPIO14 is assigned as the data signal of i2c master port
 *    GPIO15 is assigned as the clock signal of i2c master port
 *
*/

//last bit is defined by SDO pin
#define BMP280_I2C_ADDR (0x77)

#define PRESS_MSB_ADDR (0xF7)
#define CTL_MEASURE (0xF4)

#define BMP_280_CALIBRATION_ADDR (0x88)


#define BMP_CALIBRATION_LENGTH (24)

static uint16_t deserialize_u16(uint8_t *buffer);
static int16_t deserialize_i16(uint8_t *buffer);

static int32_t calibrate_temp(uint32_t raw_temp, int32_t *t_fine);
static uint32_t calibrate_pressure(uint32_t raw_press, int32_t t_fine);

static void initBMP280(void);
static int getPressAndTemp(char*Buffer);

static esp_err_t bmp280_read(uint8_t reg_addr, int length, uint8_t* outBuffer);

static esp_err_t bmp280_write(uint8_t reg_addr, uint8_t data);

char* BMP_280_TAG = "bmp280";

uint8_t bmp_calibration_values[BMP_CALIBRATION_LENGTH];


void activate_bmp280(tSampleCapture *active_sensor)
{
    active_sensor->getSample = getPressAndTemp;
    active_sensor->initModule = initBMP280; 
    init_sensor();
}

static uint16_t deserialize_u16(uint8_t *buffer)
{
    return buffer[1] << 8 | buffer[0];
}

static int16_t deserialize_i16(uint8_t *buffer)
{
    return (int16_t) deserialize_u16(buffer);
}

static void initBMP280(void)
{
    i2c_sensor_init();
    //
    //top 3 bits are for pressure
    //next 3 bits are for temperature
    //next 2 bits are for mode
    //current settings:
    //-pressure oversampled to 20 bits of resolution
    //-temp oversampled to 20 bits of resolution
    //-normal mode, which is repeated sampling
    //
    bmp280_write(CTL_MEASURE, (0x5 << 5) | (0x5 << 2) | 0x3 );
    bmp280_read(BMP_280_CALIBRATION_ADDR,
            BMP_CALIBRATION_LENGTH, bmp_calibration_values);
}



static uint32_t calibrate_pressure(uint32_t raw_press, int32_t t_fine)
{

    uint16_t P1 = deserialize_u16(bmp_calibration_values+6);
    int16_t P2 = deserialize_i16(bmp_calibration_values+8);
    int16_t P3 = deserialize_i16(bmp_calibration_values+10);
    int16_t P4 = deserialize_i16(bmp_calibration_values+12);
    int16_t P5 = deserialize_i16(bmp_calibration_values+14);
    int16_t P6 = deserialize_i16(bmp_calibration_values+16);
    int16_t P7 = deserialize_i16(bmp_calibration_values+18);
    int16_t P8 = deserialize_i16(bmp_calibration_values+20);
    int16_t P9 = deserialize_i16(bmp_calibration_values+22);
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)P6;
    var2 = var2 + ((var1*(int64_t)P5)<<17);
    var2 = var2 + (((int64_t)P4)<<35);
    var1 = ((var1 * var1 * (int64_t)P3)>>8) + ((var1 * (int64_t)P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)P1)>>33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576-raw_press;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)P7)<<4);
    return (uint32_t)p;
}

static int32_t calibrate_temp(uint32_t raw_temp, int32_t *t_fine)
{
    int32_t T1 = deserialize_u16(bmp_calibration_values);
    int32_t T2 = deserialize_i16(bmp_calibration_values+2);
    int32_t T3 = deserialize_i16(bmp_calibration_values+4);
    int32_t var1, var2, T;
    var1 = ((((raw_temp>>3) - (T1<<1))) * (T2)) >> 11;

    var2 = (((((raw_temp>>4) - (T1)) * ((raw_temp>>4) - (T1))) >> 12)*(T3)) >> 14;
    *t_fine = var1 + var2;
    T = (*t_fine * 5 + 128) >> 8;
    return T;
}

static int getPressAndTemp(char*Buffer)
{
    uint8_t raw_results[6];
    bmp280_read(PRESS_MSB_ADDR,6, raw_results);

    uint32_t raw_press = raw_results[0] << 12 |
                        raw_results[1] << 4 |
                        (0xF & (raw_results[2] >> 4) );

    uint32_t raw_temp = raw_results[3] << 12 |
                        raw_results[4] << 4 |
                        (0xF & (raw_results[5] >> 4) );

    ESP_LOGI(BMP_280_TAG, "read: %d,%d", raw_press,raw_temp); 
    int32_t t_fine;
    int32_t calTemp = calibrate_temp(raw_temp, &t_fine);
    uint32_t calPressure = calibrate_pressure(raw_press, t_fine);
    
    int len = sprintf(Buffer,
                      "%.4fPa,%.3fC",calPressure/(256.0 * 101325),
                     calTemp/100.0);
    if(len > 20)
        len = 20;
    return len;
}

static esp_err_t bmp280_write(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //start condition
    i2c_master_start(cmd);

    //addr write
    i2c_master_write_byte(cmd,
                    (BMP280_I2C_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    //setting reg addr
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    
    //write to reg addr
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);

    //final stop condition
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(
            BMP_280_PORT, 
            cmd,
            1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
static esp_err_t bmp280_read(uint8_t reg_addr, int length, uint8_t* outBuffer)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //start condition
    i2c_master_start(cmd);

    //addr write
    i2c_master_write_byte(cmd,
                    (BMP280_I2C_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    //setting reg addr
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    
    //repeated start condition for next txn 
    i2c_master_start(cmd);
    
    //addr write for the read txn 
    i2c_master_write_byte(cmd,
                    (BMP280_I2C_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);

    //read bytes
    i2c_master_read(cmd, outBuffer, length-1, ACK_VAL);
    i2c_master_read_byte(cmd, &(outBuffer[length-1]) , NACK_VAL);
//
    //final stop condition
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(
            BMP_280_PORT, 
            cmd,
            1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

