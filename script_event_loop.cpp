

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

extern "C" {
#include "babel.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_common.h"
#include "ads1118.h"
#include "uart_programmer.h"
#include "display_manager.h"
#include "sample_capture.h"
#include "gatts_demo.h"
}
#include "script_event_loop.hpp"
#include "script_controller.hpp"
#define EEPROM_I2C_ADDR (0x50)

//
//defines the location in eeprom of the sensor profile we're dealing with
//
#define PROFILE_ADDR 0 
#define PROFILE_LEN 4
#define CRC_LEN 2 
#define METADATA_LEN (CRC_LEN + PROFILE_LEN)

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


//
//from the datasheet  - pages are 16 bytes long
//                      the first 8 bytes set up the page write
//                      I think reads/writes roll over when hitting the end 
//                      of the page
//

/* static variables */

typedef struct _scriptMetadata {
    uint32_t length;
    uint16_t crc;
} tScriptMetadata;

const char* babel_file_name = "babel_program.py";
const char* EEPROM_TAG = "eeprom";
static prepare_type_env_t b_prepare_write_env;

esp_gatt_char_prop_t b_property = 0;

static QueueHandle_t eeprom_program_queue;

ScriptController script_controller; 



/* static function */

static void eeprom_poll(void* arg);

bool script_ready(void)
{
    return script_controller.get_script_file() != NULL;
}

float ScriptControllerCalValue(char* key)
{
    return script_controller.get_cal_data(key);
}

void script_controller_event_loop_init(void* arg)
{
    ESP_LOGI("eeprom", "starting eeprom task"); 
    xTaskCreate(eeprom_poll, 
                 "i2c_test_task_0",
                 1024 * 5,
                 (void* ) 0, 
                 10,
                 NULL);
}


static void i2c_example_master_init(void)
{
    i2c_port_t i2c_master_port = EEPROM_PORT;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = static_cast<gpio_num_t>(I2C_EXAMPLE_MASTER_SDA_IO);
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = static_cast<gpio_num_t>(I2C_EXAMPLE_MASTER_SCL_IO);
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 
                       false, //micropy arg - looks like slave feature
                       0);
}


QueueHandle_t get_programmer_queue(void)
{
    return eeprom_program_queue; 
}

static void eeprom_poll(void* arg)
{
    set_display();
    i2c_example_master_init();
    init_adsdevice();
    //i2c_example_master_init();
    activate_adc();
    /* setup display */

    init_bt();
    ble_gatts_init();

    eeprom_consumer consumer;

    bool cached_valid = false;
    script_controller.init_fs();
    //script_controller.verify_eeprom_valid();
    //this will verify the eeprom and verify the script in the fs
    if(script_controller.verify_fs_script()) {
        cached_valid = true;
    }
    eeprom_program_queue = xQueueCreate(
            3, //number of elements the queue can hold
            sizeof(programTransfer) //the size of a queue element
            );
    while (1) {
         
        programTransfer msg;
        if(xQueueReceive(eeprom_program_queue,
                    (void * )&msg,
                    (portTickType)0)) {

            switch(msg.action) {
                case programTransfer_pTransferAction_BEGIN_FILE_MESSAGE:
                    ESP_LOGI(EEPROM_TAG, "received msg begin, len %d", msg.action_type.length);
                    consumer = script_controller.get_script_consumer();
                    break;

                case programTransfer_pTransferAction_BEGIN_CAL_MESSAGE:
                    consumer = script_controller.get_cal_consumer();
                    break;

                case  programTransfer_pTransferAction_BLOCK_TRANSFER:
                    ESP_LOGI(EEPROM_TAG, "received msg block %d", msg.action_type.payload.size);
                    
                    consumer.write_data(msg.action_type.payload.bytes,
                                    msg.action_type.payload.size);
                    break;

                case programTransfer_pTransferAction_END_MSG:
                    ESP_LOGI(EEPROM_TAG, "received msg end, crc %d", msg.action_type.crc);
                    script_controller.validate_script(consumer, msg.action_type.crc);
                    break;

                default:
                    ESP_LOGE(EEPROM_TAG, "unknown pmessage type: %d",msg.action);
                    break;
            }
        }

        if(script_controller.script_valid())
        {
            if(!cached_valid)
                script_controller.commit_script();
            cached_valid = true;

            //double-check that eeprom is valid
            vTaskDelay(50 / portTICK_PERIOD_MS);
            bool eeprom_valid = script_controller.confirm_eeprom();
            if(!eeprom_valid)
                cached_valid = false;
        }
        else
        {
            script_controller.try_validate();
            cached_valid = false;
            if(script_controller.verify_fs_script()) {
                cached_valid = true;
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
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
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        //return the page that was read, and the whole eeprom page
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
        {
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




