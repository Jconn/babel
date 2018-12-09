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
#include "sensor_drive_constructor.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "gatts_demo.h"

#include "babel.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_common.h"

char *serial_tag = "serial_driver";
#define GATTS_SERVICE_UUID_TEST_C   0x00DD
#define GATTS_CHAR_UUID_TEST_C      0xDD01
#define GATTS_DESCR_UUID_TEST_C     0x4444
#define GATTS_NUM_HANDLE_TEST_C     4


esp_gatt_char_prop_t c_property = 0;

static tBabelMsgHandler s_ble_manager;
static prepare_type_env_t c_prepare_write_env;
static tIicManager s_i2c_manager;
typedef enum serial_action 
{
    begin_command = 0,
    command_chunk = 1
} tSerialCommand;


static bool process_serial_msg(tBabelMsgHandler *manager)
{

    ESP_LOGI(serial_tag, "processing serial message of len %d", manager->current_msg_length);
    esp_log_buffer_hex(serial_tag, manager->msg_bytes, manager->current_msg_length);
            
    SerialMessage msg = SerialMessage_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(manager->msg_bytes,
            manager->current_msg_length);
    bool status = pb_decode(&stream, SerialMessage_fields, &msg);
    if(!status)
    {
        ESP_LOGI(serial_tag, "decode failed");
        return false;
    }
    ESP_LOGI(serial_tag, " parsed message with %d actions", msg.action_count);
    if(msg.action_count == 0)
    {
        return false;
    }
    uint32_t current_value,prev_value = 0;
    if(s_i2c_manager.cmd != NULL)
    {
        i2c_cmd_link_delete(s_i2c_manager.cmd);
    }

    memset(s_i2c_manager.rxBuffer, 0, sizeof(s_i2c_manager.rxBuffer) );
    s_i2c_manager.rxBufLength = 0;
    s_i2c_manager.cmd = i2c_cmd_link_create();
    for(int i = 0; i< msg.action_count; ++i)
    {
        current_value = msg.action[i]; 
        process_byte(&current_value, &prev_value, &s_i2c_manager);
    }
    /*
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
    */
    return true;
}


