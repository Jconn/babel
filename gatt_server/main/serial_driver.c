//
//this subystem gives a ble characteristic to the application layer
//in order to perform raw serial actions (write/read)
//

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


char *serial_tag = "serial_driver";
#define GATTS_SERVICE_UUID_TEST_C   0x00DD
#define GATTS_CHAR_UUID_TEST_C      0xDD01
#define GATTS_DESCR_UUID_TEST_C     0x4444
#define GATTS_NUM_HANDLE_TEST_C     4


esp_gatt_char_prop_t c_property = 0;

static prepare_type_env_t c_prepare_write_env;
static tIicManager s_i2c_manager;
typedef enum serial_action 
{
    begin_command = 0,
    command_chunk = 1
} tSerialCommand;

void serial_driver_profile(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param, struct gatts_profile_inst *profile)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(serial_tag, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        profile->service_id.is_primary = true;
        profile->service_id.id.inst_id = 0x00;
        profile->service_id.id.uuid.len = ESP_UUID_LEN_16;
        profile->service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_C;

        esp_ble_gatts_create_service(gatts_if, &profile->service_id, GATTS_NUM_HANDLE_TEST_C);
        break;
    case ESP_GATTS_READ_EVT: {

        ESP_LOGI(serial_tag,"performing i2c read");

        esp_err_t ret = i2c_master_cmd_begin(
                BMP_280_PORT, 
                s_i2c_manager.cmd,
                1000 / portTICK_RATE_MS);

        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        //return the page that was read, and the whole eeprom page
        if(ret == ESP_OK)
        {
            ESP_LOGI(serial_tag,"i2c success, length %d", s_i2c_manager.rxBufLength);
            rsp.attr_value.value[0] = s_i2c_manager.rxBufLength;
            esp_log_buffer_hex(serial_tag, s_i2c_manager.rxBuffer, s_i2c_manager.rxBufLength);

            memcpy(rsp.attr_value.value + 1,
                    s_i2c_manager.rxBuffer, 
                    s_i2c_manager.rxBufLength);

            rsp.attr_value.len = s_i2c_manager.rxBufLength + 1;
        }
        else
        {
            ESP_LOGI(serial_tag,"i2c read fail");
            //pass error code back up 
            rsp.attr_value.len = 1;
            rsp.attr_value.value[0] = -1; 
        }
        esp_ble_gatts_send_response(gatts_if,
                            param->read.conn_id,
                            param->read.trans_id,
                            ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
         
        if(param->write.len !=0)
        {
            int command = param->write.value[0];   
            int length = param->write.value[1];
            static tI2cActions current_command = 0;
            static tI2cActions prev_command = 0;
            switch(command)
            {
                case begin_command:
                    ESP_LOGI(serial_tag,"received begin command");
                    prev_command = 0;
                    if(s_i2c_manager.cmd != NULL)
                    {
                        i2c_cmd_link_delete(s_i2c_manager.cmd);
                    }

                    memset(s_i2c_manager.rxBuffer, 0, sizeof(s_i2c_manager.rxBuffer) );
                    s_i2c_manager.rxBufLength = 0;
                    s_i2c_manager.cmd = i2c_cmd_link_create();
                break;
                case command_chunk:
                {
                    ESP_LOGI(serial_tag,"received command chunk of len %d", length);
                    for(int i = 0; i < length; ++i)
                    {
                        current_command = param->write.value[i+2]; 

                        process_byte(&current_command,
                                &prev_command,
                                &s_i2c_manager);
                        prev_command = current_command;
                    }

                }
                break;
            }
        }

        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(serial_tag,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&c_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(serial_tag, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(serial_tag, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        profile->service_handle = param->create.service_handle;
        profile->char_uuid.len = ESP_UUID_LEN_16;
        profile->char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_C;

        esp_ble_gatts_start_service(profile->service_handle);
        c_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret =esp_ble_gatts_add_char( profile->service_handle, &profile->char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        c_property,
                                                        NULL, NULL);
        if (add_char_ret){
            ESP_LOGE(serial_tag, "add char failed, error code =%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(serial_tag, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
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
        ESP_LOGI(serial_tag, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(serial_tag, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(serial_tag, "CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        profile->conn_id = param->connect.conn_id;
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(serial_tag, "ESP_GATTS_CONF_EVT status %d", param->conf.status);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(serial_tag, param->conf.value, param->conf.len);
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

