#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "eeprom_config.h"
#include "sample_capture.h"
#include "mcp9700_temp.h"
#include "df_robot_soil.h"

#define EEPROM_I2C_ADDR (0x54)

//
//defines the location in eeprom of the sensor profile we're dealing with
//
#define PROFILE_ADDR 0
#define PROFILE_LEN 4


/*
 - master:
 *    GPIO18 is assigned as the data signal of i2c master port
 *    GPIO19 is assigned as the clock signal of i2c master port
 *
*/
#define DATA_LENGTH                        512              /*!<Data buffer length for test buffer*/
#define RW_TEST_LENGTH                     129              /*!<Data length for r/w test, any value from 0-DATA_LENGTH*/
#define DELAY_TIME_BETWEEN_ITEMS_MS        1234             /*!< delay time between different test items */

#define I2C_EXAMPLE_SLAVE_NUM              I2C_NUM_0        /*!<I2C port number for slave dev */
#define I2C_EXAMPLE_SLAVE_TX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave tx buffer size */
#define I2C_EXAMPLE_SLAVE_RX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave rx buffer size */

#define I2C_EXAMPLE_MASTER_SCL_IO          19               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          18               /*!< gpio number for I2C master data  */
#define EEPROM_PORT             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

#define BH1750_SENSOR_ADDR                 0x23             /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START                   0x23             /*!< Command to set measure mode */
#define ESP_SLAVE_ADDR                     0x28             /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT                          0 /*!< I2C master write */
#define READ_BIT                           1  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */


//
//from the datasheet  - pages are 16 bytes long
//                      the first 8 bytes set up the page write
//                      I think reads/writes roll over when hitting the end 
//                      of the page
//



static void i2c_example_master_init(void);
static void eeprom_poll(void* arg);
static void deserialize_u32(uint8_t*in , uint32_t *out);
static void serialize_u32(uint8_t*out , uint32_t in);
static int32_t eeprom_profile = -1;

int32_t get_cached_profile(void)
{
    return eeprom_profile;
}

static void activate_profile(int32_t profile)
{
    switch(profile)
    {
        case SENS_MCP9700:
            ESP_LOGI("eeprom", "activating mcp9700"); 
            activate_mpc9700(get_active_sensor());
            break;
        case SENS_SOIL_ANALOG:
            activate_df_robot_soil(get_active_sensor() );
            ESP_LOGI("eeprom", "activating soil sensor"); 
            break;
        case SENS_MAG_FIELD_ANALOG:
            ESP_LOGI("eeprom", "activating mag sensor"); 
            break;
        default:
            ESP_LOGI("eeprom", "unknown profile"); 
            break;
    }
}

esp_err_t eeprom_read(uint8_t page_addr, int length, uint8_t* outBuffer)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //start condition
    i2c_master_start(cmd);

    //addr write
    i2c_master_write_byte(cmd,
                    (EEPROM_I2C_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    //setting page addr
    i2c_master_write_byte(cmd, page_addr, ACK_CHECK_EN);
    
    //repeated start condition for next txn 
    i2c_master_start(cmd);
    
    //addr write for the read txn 
    i2c_master_write_byte(cmd,
                    (EEPROM_I2C_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);

    //the eeprom datasheet specs that ACK is required on 
    //all bytes, not just the first n-1 bytes
    //read bytes
    i2c_master_read(cmd, outBuffer, length, ACK_VAL);
//    i2c_master_read_byte(cmd, &(outBuffer[length-1]), NACK_VAL);
//
    //final stop condition
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(
            EEPROM_PORT, 
            cmd,
            1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t eeprom_write(uint8_t page_addr, int length, uint8_t* inBuffer)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
            (EEPROM_I2C_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);

    i2c_master_write_byte(cmd, page_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, inBuffer, length, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(
            EEPROM_PORT,
            cmd,
            1000 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);
    return ret;
}

int set_profile(uint32_t profile_val)
{
    uint8_t raw_buf[PROFILE_LEN];
    serialize_u32(raw_buf, profile_val);
    return eeprom_write(
            PROFILE_ADDR,
            PROFILE_LEN,
            raw_buf);
}

int32_t get_eeprom_profile(void)
{
    uint8_t profile[10];
    if(eeprom_read(PROFILE_ADDR,
                PROFILE_LEN,
                profile) == ESP_FAIL)
    {
        ESP_LOGI("eeprom", "i2c read failed"); 
        
        return -1;
    }
    
    ESP_LOGI("eeprom", "i2c read success"); 
    uint32_t out;
    deserialize_u32(profile,&out); 
    return out;
}

void eeprom_init(void* arg)
{
    xTaskCreate(eeprom_poll, 
                 "i2c_test_task_0",
                 1024 * 2,
                 (void* ) 0, 
                 10,
                 NULL);
}

static void deserialize_u32(uint8_t*in , uint32_t *out)
{
    *out = (in[0])    |
        (in[1] << 8)  |
        (in[2] << 16) | 
        (in[3] << 24); 
}

static void serialize_u32(uint8_t*out , uint32_t in)
{
    out[0] = in & 0xFF;
    out[1] = (in >> 8 ) & 0xFF;
    out[2] = (in >> 16) & 0xFF;
    out[3] = (in >> 24) & 0xFF;
}

static void i2c_example_master_init(void)
{
    int i2c_master_port = EEPROM_PORT;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}

static void eeprom_poll(void* arg)
{
    i2c_example_master_init();
    activate_adc();
    while (1) {

        int32_t current_profile = get_eeprom_profile();
        if(current_profile != eeprom_profile)
        {
            activate_profile(current_profile); 
        }
        eeprom_profile = current_profile;

        ESP_LOGI("eeprom", "current profile: %d\n", eeprom_profile); 
        vTaskDelay(1000/ portTICK_RATE_MS);
    }
}





