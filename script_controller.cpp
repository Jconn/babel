#include "script_controller.hpp"
#include "babel_utils.hpp"
#include "esp_log.h"

#include "esp_spiffs.h"
#include <vector>
#include <iostream>
#include <memory>
#include <map>
#include <stdio.h>
#include <string.h>

void ScriptController::init_fs(void) {

}


void ScriptController::commit_script(void) {

    FILE *fd;
    ESP_LOGI(m_Tag, "commiting script to file  %s", m_ScriptFileName );
    fd = fopen(m_ScriptFileName, "wb");

    if (!fd) {
        ESP_LOGE(m_Tag, "couldnt open program file %s", m_ScriptFileName );
        return;
    }
    else
    {
        const size_t i_step = 16;
        uint8_t temp_buffer[i_step+1];
        const size_t script_len = m_scriptEeprom.size();

        for (size_t i = 0; i < script_len; i+=i_step){
            size_t len = i_step;
            if((script_len - i) < i_step) len = script_len - i;
            if(!m_scriptEeprom.read_data(temp_buffer, i, len))
            {
                fclose(fd);
                m_ScriptInFs = false;
                return;
            }
            temp_buffer[len] = 0;
            fprintf(fd, "%s", temp_buffer);
        }
        fclose(fd);
    }

    m_ScriptInFs = true;
}

void ScriptController::new_script(size_t len) {
    m_scriptEeprom.begin_new_file();
}

bool ScriptController::verify_eeprom_valid(void) {

    return m_scriptEeprom.validate_self();
}

bool ScriptController::verify_fs_script(void) {
    //if eeprom valid, then continue to trying to validate
    //the file system
    if(!verify_eeprom_valid())
        return false;
    
    FILE *fd;
    ESP_LOGI(m_Tag, "valid eeprom, validating file %s", m_ScriptFileName );
    fd = fopen(m_ScriptFileName, "r");

    if (!fd) {
        ESP_LOGE(m_Tag, "couldnt open program file %s", m_ScriptFileName );
    }
    else
    {
        crc_advancer crc;
        size_t file_len = 0;
        while(1) {
            char c = fgetc(fd);
            if( feof(fd) ) {
                break ;
            }
            file_len++;
            crc.add_byte(c);
        }
        if(crc.get_crc() == m_scriptEeprom.get_cached_crc()){
            ESP_LOGI(m_Tag, "valid script file in fs");
            return true;
        }
        else {
            ESP_LOGE(m_Tag, "failed to validate file of len %d, expecting %d", file_len, m_scriptEeprom.get_cached_length());
        }
    }
    return false;
}


bool ScriptController::confirm_eeprom(void) {
    return m_scriptEeprom.confirm_valid();
}
