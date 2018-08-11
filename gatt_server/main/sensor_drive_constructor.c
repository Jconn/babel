//c standard library includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//
//esp-idf includes
//
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "driver/i2c.h"


//
//application includes
//
#include "sample_capture.h"
#include "sensor_drive_constructor.h"
#include "eeprom_config.h"
#include "bmp280.h"


#define ID_PAGE_INDEX 0
#define CAL_PAGE_INDEX 1
#define INIT_PAGE_INDEX 17
#define SAMPLE_CAPTURE_PAGE_INDEX 33


#define I2C_EXAMPLE_MASTER_SCL_IO          26               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          27               /*!< gpio number for I2C master data  */


#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */





static char* tag = "sensor";
static void construct_analog_reader(void);
static void construct_i2c_reader(void);
static int default_read(uint8_t *data_buf);
static int get_analog(uint8_t *data_buf);
static int get_i2c(uint8_t *data_buf);
static void perform_i2c_sensor_init_action(void);


static void deserialize_u32(uint8_t*in , uint32_t *out);
static void serialize_u32(uint8_t*out , uint32_t in);

static int (*s_sensor_read_fn)(uint8_t* data_buf) = get_analog;

static tIicManager s_manager;

static uint8_t s_serial_data[500];
static int s_serial_length = 0;
void activate_profile(void)
{
    uint8_t data[EEPROM_PAGE_LENGTH];
    eeprom_read(ID_PAGE_INDEX, EEPROM_PAGE_LENGTH, data);
    uint32_t family, drive_type, id; 
    deserialize_u32(data, &family);
    deserialize_u32(data, &id);
    deserialize_u32(data, &drive_type);
    switch(drive_type)
    {
        case DRIVE_TYPE_ANALOG:
            ESP_LOGI(tag, "activating analog sensor"); 
            //easy
            construct_analog_reader();
            break;
        case DRIVE_TYPE_I2C:
            ESP_LOGI(tag, "activating i2c sensor"); 
            construct_i2c_reader();
            break;
        default:
            ESP_LOGI(tag, "unknown drive type"); 
            break;
    }
}

int get_sensor(uint8_t* data_buf)
{
    ESP_LOGI(tag, "getting sensor value"); 
    return s_sensor_read_fn(data_buf);
}

void i2c_sensor_init(void)
{
    int i2c_master_port = BMP_280_PORT;
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

static void construct_analog_reader(void)
{
    initADC();
    s_sensor_read_fn = get_analog;
}

void process_byte(tI2cActions *current_command, 
                    tI2cActions *prev_command,
                    tIicManager *manager
                    )
{
    bool interpreting_raw_data = *prev_command == WRITE_ACK ||  
                    *prev_command == WRITE_NACK;
    if(!interpreting_raw_data) 
    {
        switch(*current_command)
        {
            case START:
                i2c_master_start(manager->cmd);
                ESP_LOGI(tag, "start"); 
                break;
            case WRITE_ACK:
                //next byte is ignored and processed as the write data
                ESP_LOGI(tag, "write ack"); 
                break;

            case WRITE_NACK:
                //next byte is ignored and processed as the write data
                ESP_LOGI(tag, "write nack"); 
                break;
            case READ_ACK:
                i2c_master_read_byte(manager->cmd,
                        &(manager->rxBuffer[manager->rxBufLength]),
                        ACK_VAL);
                ESP_LOGI(tag, "read ack"); 
                manager->rxBufLength += 1;
                break;
            case READ_NACK:
                i2c_master_read_byte(manager->cmd,
                        &(manager->rxBuffer[manager->rxBufLength]),
                        NACK_VAL);
                ESP_LOGI(tag, "read nack"); 
                manager->rxBufLength += 1;
                break;
            case STOP:
                ESP_LOGI(tag, "stop"); 
                i2c_master_stop(manager->cmd);
                break;
            default:

                ESP_LOGI(tag, "interpeting unknown command(%d)", *current_command); 
                break;
        }
    }
    else
    {
        uint8_t raw_data = *current_command;
        ESP_LOGI(tag, "raw data: %2X", raw_data); 
        switch(*prev_command)
        {
            case WRITE_ACK:
                i2c_master_write_byte(manager->cmd, raw_data, ACK_CHECK_EN);
                break;

            case WRITE_NACK:
                i2c_master_write_byte(manager->cmd, raw_data, ACK_CHECK_DIS);
                break;
            default:
                //assert
                ESP_LOGI(tag, "interpreting command as raw byte in non-write stage(%d)", *prev_command); 
                break;
        }
    }
    
    *prev_command = *current_command; 

}

static void perform_i2c_sensor_init_action(void)
{
    uint8_t data[16];
    eeprom_read(INIT_PAGE_INDEX, EEPROM_PAGE_LENGTH, data);
    int sample_len = data[0];
    ESP_LOGI(tag, "building init of len %d", sample_len); 
    if(sample_len == 0)
    {
        return;
    }
    tI2cActions prev_command = 0;
    tI2cActions current_command;


    tIicManager init_manager;
    init_manager.rxBufLength = 0;
    init_manager.cmd = NULL;
    init_manager.cmd = i2c_cmd_link_create();
    for(int i = 1; i < sample_len + 1; ++i)
    {
        if(i%16 == 0)
        {
            int read_length = (sample_len - i) > EEPROM_PAGE_LENGTH ? 
                EEPROM_PAGE_LENGTH : sample_len - i;
            if(read_length > 0)
            {
                eeprom_read(SAMPLE_CAPTURE_PAGE_INDEX,read_length, data);
            }
        }

        current_command = data[i%16]; 

        process_byte(&current_command, 
                &prev_command,
                &init_manager 
                    );
    }

    esp_err_t ret = i2c_master_cmd_begin(
            BMP_280_PORT, 
            init_manager.cmd,
            1000 / portTICK_RATE_MS);


    if(ret == ESP_OK)
    {
        ESP_LOGI(tag,"i2c init success, length %d", s_manager.rxBufLength);
    }
    else
    {
        ESP_LOGI(tag,"i2c init fail");
        return;
    }
}

static void construct_i2c_reader(void)
{
    uint8_t data[16];
    eeprom_read(SAMPLE_CAPTURE_PAGE_INDEX, EEPROM_PAGE_LENGTH, data);
    int sample_len = data[0];


    i2c_sensor_init();
    s_manager.cmd = i2c_cmd_link_create();
    s_manager.rxBufLength = 0;

    //initialize so it isn't interpreted as a write or 
    //write ack
    //
    ESP_LOGI(tag, "building capture of len %d", sample_len); 
    s_serial_length = sample_len;
    for(int i = 1; i < sample_len + 1; ++i)
    {
        if(i%16 == 0)
        {
            int read_length = (sample_len - i) > EEPROM_PAGE_LENGTH ? 
                EEPROM_PAGE_LENGTH : sample_len - i;
            if(read_length > 0)
            {
                eeprom_read(SAMPLE_CAPTURE_PAGE_INDEX,read_length, data);
            }
        }

        s_serial_data[i-1] = data[i%16];


    }

    //this is the hard one - have to see if there is any init stuff to do
    //then store what the capture one is
    s_sensor_read_fn = get_i2c;
    perform_i2c_sensor_init_action();
}

static int default_read(uint8_t *data_buf)
{

    return 0;
}

static int get_analog(uint8_t *data_buf)
{
    uint32_t voltage = getADCmV();
    memcpy(data_buf, &voltage, sizeof(voltage)); 
    return sizeof(voltage);
}

static int get_i2c(uint8_t *data_buf)
{
    if(s_manager.cmd != NULL)
    {
        i2c_cmd_link_delete(s_manager.cmd);
    }
    s_manager.cmd = i2c_cmd_link_create();
    s_manager.rxBufLength = 0;

    tI2cActions prev_command = 0;
    tI2cActions current_command;
    for(int i = 0; i < s_serial_length; ++i)
    {

        current_command = s_serial_data[i];
        process_byte(&current_command, 
                &prev_command,
                &s_manager 
                    );
    }
    esp_err_t ret = i2c_master_cmd_begin(
            BMP_280_PORT, 
            s_manager.cmd,
            1000 / portTICK_RATE_MS);
        if(ret == ESP_OK)
        {
            ESP_LOGI(tag,"i2c success, length %d", s_manager.rxBufLength);
            memcpy(data_buf, s_manager.rxBuffer, s_manager.rxBufLength);
            return s_manager.rxBufLength;
        }
        else
        {
            ESP_LOGI(tag,"i2c fail");
            return 0;
        }
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

