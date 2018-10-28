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
#include "serial_driver.h"

#include "babel.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_common.h"

#define ID_PAGE_INDEX 0
#define CAL_PAGE_INDEX 1
#define INIT_PAGE_INDEX 17
#define SAMPLE_CAPTURE_PAGE_INDEX 33


#define I2C_EXAMPLE_MASTER_SCL_IO          26               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          27               /*!< gpio number for I2C master data  */


#define I2C_SDA_ENABLE_IO 33
#define I2C_SCL_ENABLE_IO 32

#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */





static char* tag = "sensor";
static void construct_analog_reader(void);
static void construct_i2c_reader(void);
static int get_analog(uint8_t *data_buf);
static int get_i2c(uint8_t *data_buf);
static void perform_i2c_sensor_init_action(void);


static int deserialize_u32(uint8_t*in , uint32_t *out);

static int (*s_sensor_read_fn)(uint8_t* data_buf) = get_analog;

static tIicManager s_manager;

static uint32_t s_serial_data[256];
static int s_serial_length = 0;


static void adc_pin_configure(void);
static void i2c_pin_configure(void);


static void i2c_pin_configure(void)
{
    /* Set the GPIO as a push/pull output */
    gpio_set_level(I2C_SCL_ENABLE_IO, 0);
    gpio_set_level(I2C_SDA_ENABLE_IO, 0);
    i2c_sensor_init();
}

static void adc_pin_configure(void)
{

    gpio_pad_select_gpio(I2C_EXAMPLE_MASTER_SDA_IO); 
    gpio_pad_select_gpio(I2C_EXAMPLE_MASTER_SCL_IO);

    gpio_set_direction(I2C_EXAMPLE_MASTER_SCL_IO, GPIO_MODE_INPUT);
    gpio_set_direction(I2C_EXAMPLE_MASTER_SDA_IO, GPIO_MODE_INPUT);

    gpio_set_level(I2C_SCL_ENABLE_IO, 1);
    gpio_set_level(I2C_SDA_ENABLE_IO, 1);
}

void activate_profile(void)
{
    static bool init_profile = false;
    if(!init_profile)
    {
        gpio_pad_select_gpio(I2C_SCL_ENABLE_IO); 
        gpio_pad_select_gpio(I2C_SDA_ENABLE_IO);
        gpio_set_direction(I2C_SCL_ENABLE_IO, GPIO_MODE_OUTPUT);
        gpio_set_direction(I2C_SDA_ENABLE_IO, GPIO_MODE_OUTPUT);
        init_profile = true;
    }
    uint8_t data[EEPROM_PAGE_LENGTH];
    eeprom_read(ID_PAGE_INDEX, EEPROM_PAGE_LENGTH, data);
    uint32_t family, drive_type, id; 
    int offset = 0;
    offset += deserialize_u32(&(data[offset]), &family);
    offset += deserialize_u32(&(data[offset]), &id);
    offset +=  deserialize_u32(&(data[offset]), &drive_type);
    switch(drive_type)
    {
        case DRIVE_TYPE_I2C:
            i2c_pin_configure();
            ESP_LOGI(tag, "activating i2c sensor"); 
            construct_i2c_reader();
            break;

        default:
            ESP_LOGW(tag, "unknown drive type"); 
        case DRIVE_TYPE_ANALOG:
            adc_pin_configure();
            ESP_LOGI(tag, "activating analog sensor"); 
            //easy
            construct_analog_reader();
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
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE,
                       false, //busy flag -added by micropy
                       0);
}

static void construct_analog_reader(void)
{
    initADC();
    s_sensor_read_fn = get_analog;
}


//this is no longer just a byte
void process_byte(uint32_t *current_command, 
                    uint32_t *prev_command,
                    tIicManager *manager
                    )
{
    switch(*current_command)
    {
        case SerialMessage_serialAction_START:
            i2c_master_start(manager->cmd);
            ESP_LOGI(tag, "start"); 
            break;
        case SerialMessage_serialAction_WRITE_ACK:
            //next byte is ignored and processed as the write data
            ESP_LOGI(tag, " write ack- 0 byte"); 
            i2c_master_write_byte(manager->cmd, 0, ACK_CHECK_EN);
            break;

        case SerialMessage_serialAction_WRITE_NACK:
            //next byte is ignored and processed as the write data
            i2c_master_write_byte(manager->cmd, 0, ACK_CHECK_DIS);
            ESP_LOGI(tag, "write nack - 0 byte"); 
            break;
        case SerialMessage_serialAction_READ_ACK:
            i2c_master_read_byte(manager->cmd,
                    &(manager->rxBuffer[manager->rxBufLength]),
                    ACK_VAL);
            ESP_LOGI(tag, "read ack"); 
            manager->rxBufLength += 1;
            break;
        case SerialMessage_serialAction_READ_NACK:
            i2c_master_read_byte(manager->cmd,
                    &(manager->rxBuffer[manager->rxBufLength]),
                    NACK_VAL);
            ESP_LOGI(tag, "read nack"); 
            manager->rxBufLength += 1;
            break;
        case SerialMessage_serialAction_STOP:
            ESP_LOGI(tag, "stop"); 
            i2c_master_stop(manager->cmd);
            break;
        default:
            {
                uint32_t packed_value = (*current_command) & 0xF;
                uint32_t unpacked_value = (*current_command) >> 4;
                switch(packed_value)
                {

                    case SerialMessage_serialAction_WRITE_ACK: 
                        ESP_LOGI(tag, "raw write ACK: %2X", unpacked_value); 
                        i2c_master_write_byte(manager->cmd, unpacked_value, ACK_CHECK_EN);
                        break;
                    case SerialMessage_serialAction_WRITE_NACK:
                        ESP_LOGI(tag, "raw write NACK: %2X", unpacked_value); 
                        i2c_master_write_byte(manager->cmd, unpacked_value, ACK_CHECK_DIS);
                        break;
                    case SerialMessage_serialAction_READ_ACK:
                        ESP_LOGI(tag, "read ack %d consecutive", unpacked_value); 
                        for(int i = 0; i < unpacked_value; ++i)
                        {
                            i2c_master_read_byte(manager->cmd,
                                    &(manager->rxBuffer[manager->rxBufLength]),
                                    ACK_VAL);
                            manager->rxBufLength += 1;
                        }
                        break;
                    case SerialMessage_serialAction_READ_NACK: 
                        ESP_LOGI(tag, "read nack %d consecutive", unpacked_value); 
                        for(int i = 0; i < unpacked_value; ++i)
                        {

                            i2c_master_read_byte(manager->cmd,
                                    &(manager->rxBuffer[manager->rxBufLength]),
                                    NACK_VAL);
                            manager->rxBufLength += 1;
                        }
                        break;
                    default:
                        ESP_LOGI(tag, "interpeting unknown command(%d)", *current_command); 
                        break;
                }
            }
            break;
    }
    
    *prev_command = *current_command; 

}

static void perform_i2c_sensor_init_action(void)
{
    uint8_t data[256];
    eeprom_read(INIT_PAGE_INDEX, EEPROM_PAGE_LENGTH, data);
    int sample_len = data[0];
    ESP_LOGI(tag, "building init of len %d", sample_len); 
    if(sample_len == 0)
    {
        return;
    }

    //cycle through and read the rest of the pages
    for(int i = EEPROM_PAGE_LENGTH; i < sample_len;  i+=16)
    {
        int read_length = (sample_len - i) > EEPROM_PAGE_LENGTH ? 
            EEPROM_PAGE_LENGTH : sample_len - i;
        if(read_length > 0)
        {
            eeprom_read(SAMPLE_CAPTURE_PAGE_INDEX,read_length, &(data[i]));
        }
    }

    SerialMessage msg = SerialMessage_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(&(data[1]),
            sample_len);
    bool status = pb_decode(&stream, SerialMessage_fields, &msg);
    if(!status)
    {
        ESP_LOGI(tag,"serial decode failed");
        return;
    }

    tIicManager init_manager;
    init_manager.rxBufLength = 0;
    init_manager.cmd = NULL;
    init_manager.cmd = i2c_cmd_link_create();

    uint32_t current_value,prev_value = 0;
    for(int i = 0; i< msg.action_count; ++i)
    {
        current_value = msg.action[i]; 
        process_byte(&current_value, &prev_value, &init_manager);

        if(current_value == SerialMessage_serialAction_STOP)
        {

            esp_err_t ret = i2c_master_cmd_begin(
                    BMP_280_PORT, 
                    init_manager.cmd,
                    1000 / portTICK_RATE_MS);
            if(ret == ESP_OK)
            {
                ESP_LOGI(tag,"i2c init success, length %d", init_manager.rxBufLength);
                //memcpy(data_buf, init_manager.rxBuffer, init_manager.rxBufLength);
                return;
            }
            else
            {
                ESP_LOGI(tag,"i2c init fail");
                return;
            }

            if( i != msg.action_count)
            {
                //there must be more to do still
                init_manager.cmd = i2c_cmd_link_create();
                init_manager.rxBufLength = 0;
            }
        }
    }
}

static void construct_i2c_reader(void)
{
    uint8_t data[256];
    eeprom_read(SAMPLE_CAPTURE_PAGE_INDEX, EEPROM_PAGE_LENGTH, data);
    int sample_len = data[0];
    s_serial_length = 0;

    s_manager.cmd = i2c_cmd_link_create();
    s_manager.rxBufLength = 0;

    //initialize so it isn't interpreted as a write or 
    //write ack
    //
    ESP_LOGI(tag, "building capture of len %d", sample_len); 
    for(int i = EEPROM_PAGE_LENGTH; i < sample_len + 1; i += EEPROM_PAGE_LENGTH)
    {
        int read_length = (sample_len - i) > EEPROM_PAGE_LENGTH ? 
            EEPROM_PAGE_LENGTH : sample_len - i;
        if(read_length > 0)
        {
            eeprom_read(SAMPLE_CAPTURE_PAGE_INDEX,read_length, &(data[i]));
        }
    }

    SerialMessage msg = SerialMessage_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(&(data[1]),
            sample_len);
    bool status = pb_decode(&stream, SerialMessage_fields, &msg);
    if(status == false)
    {
        ESP_LOGI(tag, "i2c construct message parse failed"); 
        return;
    }
    uint32_t current_value,prev_value = 0;
    for(int i = 0; i< msg.action_count; ++i)
    {
        s_serial_data[i] = msg.action[i];
        s_serial_length++;
        current_value = msg.action[i]; 
        process_byte(&current_value, &prev_value, &s_manager);
    }
    //this is the hard one - have to see if there is any init stuff to do
    //then store what the capture one is
    s_sensor_read_fn = get_i2c;
    perform_i2c_sensor_init_action();
}

int default_read(uint8_t *data_buf)
{

    return 0;
}

static int get_analog(uint8_t *data_buf)
{
    uint32_t voltage = getADCmV();
    memcpy(data_buf, &voltage, sizeof(voltage)); 
    ESP_LOGI(tag,"voltage is %d", voltage);
    return sizeof(voltage);
}

static int get_i2c(uint8_t *data_buf)
{
    ESP_LOGI(tag,"constructing i2c message, length %d", s_serial_length);
    if(s_manager.cmd != NULL)
    {
        i2c_cmd_link_delete(s_manager.cmd);
    }
    s_manager.cmd = i2c_cmd_link_create();
    s_manager.rxBufLength = 0;

    uint32_t prev_command = 0;
    uint32_t current_command;
    int data_buf_index = 0;
    for(int i = 0; i < s_serial_length; ++i)
    {
        current_command = s_serial_data[i];
        process_byte(&current_command, 
                &prev_command,
                &s_manager 
                    );

        if(current_command == SerialMessage_serialAction_STOP)
        {

            esp_err_t ret = i2c_master_cmd_begin(
                    BMP_280_PORT, 
                    s_manager.cmd,
                    1000 / portTICK_RATE_MS);
            if(ret == ESP_OK)
            {
                ESP_LOGI(tag,"i2c success, length %d", s_manager.rxBufLength);
                memcpy(&(data_buf[data_buf_index]), s_manager.rxBuffer, s_manager.rxBufLength);
                data_buf_index += s_manager.rxBufLength;
            }
            else
            {
                ESP_LOGI(tag,"i2c fail");
                return 0;
            }

            if( i != s_serial_length)
            {
                //there must be more to do still
                s_manager.cmd = i2c_cmd_link_create();
                s_manager.rxBufLength = 0;
            }
        }
    }
    return data_buf_index;
}

static int deserialize_u32(uint8_t*in , uint32_t *out)
{
    *out = (in[0])    |
        (in[1] << 8)  |
        (in[2] << 16) | 
        (in[3] << 24); 
    return 4;
}

//unused
void serialize_u32(uint8_t*out , uint32_t in)
{
    out[0] = in & 0xFF;
    out[1] = (in >> 8 ) & 0xFF;
    out[2] = (in >> 16) & 0xFF;
    out[3] = (in >> 24) & 0xFF;
}

