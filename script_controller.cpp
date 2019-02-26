#include "script_controller.hpp"
#include "babel_utils.hpp"
#include "eeprom_consumer.hpp"
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
        if(!m_scriptEeprom.populate_metadata())
        {
            ESP_LOGI(m_Tag, "failed to populate metadata");
        }
        const size_t script_len = m_scriptEeprom.get_cached_length();

        for (size_t i = 0; i < script_len; i+=i_step){
            size_t len = i_step;
            if((script_len - i) < i_step) len = script_len - i;
            if(!m_scriptEeprom.read_data(temp_buffer, len, i))
            {
                fclose(fd);
                ESP_LOGE(m_Tag, "failed to read from eeprom aborting script write");
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

bool ScriptController::verify_eeprom_valid(void) {

    return m_scriptEeprom.validate_self();
}

bool ScriptController::commit_section(const eeprom_consumer& consumer, uint16_t new_crc)
{
    eeprom_utils* utils = consumer.m_utils;
    if(!utils->commit_changes(consumer.data_consumed(),
            new_crc))
    {
        return false;
    }
    return utils->validate_self();
}

bool ScriptController::try_validate(void)
{
    if(!m_scriptEeprom.validate_self()) return false;
    if(!m_calDataEeprom.validate_self()) return false;
    return true;
}

bool ScriptController::verify_fs_script(void) {
    //if eeprom valid, then continue to trying to validate
    //the file system
    m_ScriptInFs = false;
    if(!verify_eeprom_valid())
        return false;
    if(!m_calDataEeprom.validate_self()) 
        ESP_LOGW(m_Tag, "valid eeprom, invalid cal file");
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
            m_ScriptInFs = true;
            return true;
        }
        else {
            ESP_LOGE(m_Tag, "failed to validate file of len %d, expecting %d", file_len, m_scriptEeprom.get_cached_length());
        }
    }
    return false;
}

eeprom_consumer ScriptController::start_cal_programmer(size_t total_length)
{
    m_calDataEeprom.predict_begin_page(total_length);
    eeprom_consumer new_consumer(&m_calDataEeprom);
    return new_consumer;
}

eeprom_consumer ScriptController::get_script_consumer(void)
{
    eeprom_consumer new_consumer(&m_scriptEeprom);
    return new_consumer;
}

eeprom_consumer ScriptController::get_cal_consumer(void)
{
    eeprom_consumer new_consumer(&m_calDataEeprom);
    return new_consumer;
}

bool ScriptController::confirm_eeprom(void) {
    return m_scriptEeprom.confirm_valid();
}


float ScriptController::get_cal_data(char* key)
{
    return m_calDataEeprom.get_cal_data(key);
}

