

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "eeprom_config.h"
#include "sample_capture.h"
#include "mcp9700_temp.h"
#include "df_robot_soil.h"
#include "bmp280.h"
#include "sensor_drive_constructor.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "gatts_demo.h"


#define EEPROM_I2C_ADDR (0x50)

//
//defines the location in eeprom of the sensor profile we're dealing with
//
#define PROFILE_ADDR 0 
#define PROFILE_LEN 4


#define GATTS_SERVICE_UUID_TEST_B   0x00EE
#define GATTS_CHAR_UUID_TEST_B      0xEE01
#define GATTS_DESCR_UUID_TEST_B     0x2222
#define GATTS_NUM_HANDLE_TEST_B     4
/*
 - master:
 *    GPIO18 is assigned as the data signal of i2c master port
 *    GPIO19 is assigned as the clock signal of i2c master port
 *
*/


#define I2C_EXAMPLE_MASTER_SCL_IO          19               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          18               /*!< gpio number for I2C master data  */
#define EEPROM_PORT             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */



//
//from the datasheet  - pages are 16 bytes long
//                      the first 8 bytes set up the page write
//                      I think reads/writes roll over when hitting the end 
//                      of the page
//

char* EEPROM_TAG = "eeprom";

static prepare_type_env_t b_prepare_write_env;

static void i2c_example_master_init(void);
static void eeprom_poll(void* arg);
static void deserialize_u32(uint8_t*in , uint32_t *out);
static void serialize_u32(uint8_t*out , uint32_t in);
static int32_t eeprom_profile = -1;
static int32_t s_eeprom_read_page = 0;
esp_gatt_char_prop_t b_property = 0;

int32_t get_cached_profile(void)
{
    return eeprom_profile;
}
/*
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
        case SENS_BMP_280:
            ESP_LOGI("eeprom", "activating bmp 280"); 
            activate_bmp280(get_active_sensor());
            break;
        default:
            ESP_LOGI("eeprom", "unknown profile"); 
            break;
    }
}
*/
esp_err_t eeprom_read(uint8_t page_addr, int length, uint8_t* outBuffer)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //start condition
    i2c_master_start(cmd);


    uint8_t i2c_addr = EEPROM_I2C_ADDR | (page_addr/16);

    //addr write
    i2c_master_write_byte(cmd,
                    (i2c_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    //setting page addr
    i2c_master_write_byte(cmd, (page_addr % 16) * 16, ACK_CHECK_EN);
    
    //repeated start condition for next txn 
    i2c_master_start(cmd);
    
    //addr write for the read txn 
    i2c_master_write_byte(cmd,
                    (i2c_addr << 1 ) | READ_BIT, ACK_CHECK_EN);

    //the eeprom datasheet specs that ACK is required on 
    //all bytes, not just the first n-1 bytes
    //read bytes
    i2c_master_read(cmd, outBuffer, length-1, ACK_VAL);
    i2c_master_read_byte(cmd, &(outBuffer[length-1]), NACK_VAL);
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
    ESP_LOGI(EEPROM_TAG, "eWrite: page %d, len %d, value: ", page_addr, length); 
    
    //
    // there are 8 sections of 256 bytes 
    // so only 16 pages per section
    // the api to this eeprom abstracts this away
    //
    uint8_t i2c_addr = EEPROM_I2C_ADDR | (page_addr/16);
    esp_log_buffer_hex(EEPROM_TAG, inBuffer, length);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
            (i2c_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);

    //the entire block is the word
    i2c_master_write_byte(cmd, 16 * (page_addr%16), ACK_CHECK_EN);
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
            activate_profile(); 
        }
        eeprom_profile = current_profile;

        ESP_LOGI("eeprom", "sensor family: %d\n", eeprom_profile); 
        vTaskDelay(1000/ portTICK_RATE_MS);
    }
}


void eeprom_bt_profile(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param, struct gatts_profile_inst *profile)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(EEPROM_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        profile->service_id.is_primary = true;
        profile->service_id.id.inst_id = 0x00;
        profile->service_id.id.uuid.len = ESP_UUID_LEN_16;
        profile->service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_B;

        esp_ble_gatts_create_service(gatts_if, &profile->service_id, GATTS_NUM_HANDLE_TEST_B);
        break;
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(EEPROM_TAG, "eeprom page read evt(%d), conn_id %d, trans_id %d, handle %d\n",s_eeprom_read_page, param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        //return the page that was read, and the whole eeprom page
        rsp.attr_value.value[0] = s_eeprom_read_page;
        eeprom_read(s_eeprom_read_page, 16, rsp.attr_value.value + 1);
        rsp.attr_value.len = 17;
        //int32_t profile = get_cached_profile();
        //rsp.attr_value.len = 4;
        //serialize_u32(rsp.attr_value.value, profile);
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(EEPROM_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            ESP_LOGI(EEPROM_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex(EEPROM_TAG, param->write.value, param->write.len);

            uint16_t descr_value= param->write.value[1]<<8 | param->write.value[0];
            
            if (profile->descr_handle == param->write.handle && param->write.len == 2){
                if (descr_value == 0x0001){
                    if (b_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(EEPROM_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i%0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, profile->char_handle,
                                                sizeof(notify_data), notify_data, false);
                    }
                }else if (descr_value == 0x0002){
                    if (b_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(EEPROM_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i%0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, profile->char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(EEPROM_TAG, "notify/indicate disable ");
                }else{
                    ESP_LOGE(EEPROM_TAG, "unknown value");
                }

            }
            else
            {
                if(param->write.len !=0)
                {

                    int command = param->write.value[0];   
                    int page = param->write.value[1];
                    switch(command)
                    {
                        //write 
                        case 1:
                        {
                            int len = param->write.value[2];
                            if(len > 16)
                            {
                                ESP_LOGI(EEPROM_TAG, "len too big: %d",len);
                                break;
                            }
                            if(param->write.len - 3 != len)
                            {
                                ESP_LOGI(EEPROM_TAG, "len mismatch, %d expected %d", param->write.len, len+2);
                                break;
                            }
                            ESP_LOGI(EEPROM_TAG, "eeprom page %d write len %d",page, len);
                            //write the eeprom page 
                            eeprom_write(page, len, &(param->write.value[3])); 
                        }
                            break;
                        //read
                        case 2:
                            //set up for a read - if this characteristic is read
                            //then trigger an eeprom read to the page
                            s_eeprom_read_page = page;
                            break;
                        default:
                            ESP_LOGI(EEPROM_TAG, "unknown command: %d",command);
                            break;
                    }
                }
            }
            
        }
        example_write_event_env(gatts_if, &b_prepare_write_env, param);
        break;
                              }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(EEPROM_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&b_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(EEPROM_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(EEPROM_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        profile->service_handle = param->create.service_handle;
        profile->char_uuid.len = ESP_UUID_LEN_16;
        profile->char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_B;

        esp_ble_gatts_start_service(profile->service_handle);
        b_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret =esp_ble_gatts_add_char( profile->service_handle, &profile->char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        b_property,
                                                        NULL, NULL);
        if (add_char_ret){
            ESP_LOGE(EEPROM_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(EEPROM_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

        profile->char_handle = param->add_char.attr_handle;
        profile->descr_uuid.len = ESP_UUID_LEN_16;
        profile->descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_ble_gatts_add_char_descr(profile->service_handle, &profile->descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                     NULL, NULL);
        break;
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        profile->descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(EEPROM_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(EEPROM_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(EEPROM_TAG, "CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        profile->conn_id = param->connect.conn_id;
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(EEPROM_TAG, "ESP_GATTS_CONF_EVT status %d", param->conf.status);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(EEPROM_TAG, param->conf.value, param->conf.len);
        }
    break;
    case ESP_GATTS_DISCONNECT_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}



