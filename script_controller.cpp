#include "script_controller.hpp"
#include "babel_utils.hpp"
#include "esp_log.h"

#include "esp_spiffs.h"

void ScriptController::init_fs(void) {

    /*
    ESP_LOGI(m_Tag, "Initializing SPIFFS");
    
    esp_vfs_spiffs_conf_t conf = {
      .base_path = m_basePath,
      .partition_label = "internalfs",
      .max_files = 2,
      .format_if_mount_failed = false 
    };
    
    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(m_Tag, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(m_Tag, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(m_Tag, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
    */
    m_FsInit = true;
}

void ScriptController::commit_script(void) {

    FILE *fd;
    ESP_LOGI(m_Tag, "commiting script to file  %s", m_ScriptFileName );
    fd = fopen(m_ScriptFileName, "wb");

    if (!fd) {
        ESP_LOGE(m_Tag, "couldnt open program file %s", m_ScriptFileName );
    }
    else
    {
        const size_t i_step = 16;
        uint8_t temp_buffer[i_step+1];
        const size_t script_len = m_Eeprom.size();

        for (size_t i = 0; i < script_len; i+=i_step){
            size_t len = i_step;
            if((script_len - i) < i_step) len = script_len - i;
            m_Eeprom.read_data(temp_buffer, i, len);   
            temp_buffer[len] = 0;
            fprintf(fd, "%s", temp_buffer);
        }
        fclose(fd);
    }

    m_ScriptCommited = true;
}

void ScriptController::new_script(size_t len) {
    m_ScriptLength = len; 
    m_Eeprom.begin_new_file();
    m_ScriptValid = false;
    m_ScriptCommited = false;
}

void ScriptController::write_script(const std::vector<uint8_t> &script) {
    m_Eeprom.append_data(script);
}

void ScriptController::append_data(const std::vector <uint8_t> &data) {
    m_Eeprom.append_data(data);
}

void ScriptController::append_data(const uint8_t *payload, size_t len) {
    m_Eeprom.append_data(payload, len);
}

bool ScriptController::verify_eeprom_valid(void) {
    m_Eeprom.populate_metadata();
    return validate_script( m_Eeprom.get_cached_crc() );
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
        if(crc.get_crc() == m_ScriptCrc){
            ESP_LOGI(m_Tag, "valid script file in fs");
            m_ScriptCommited = true;
            return true;
        }
        else {
            ESP_LOGE(m_Tag, "failed to validate file of len %d, expecting %d", file_len, m_ScriptLength);
        }
    }
    return false;
}


bool ScriptController::validate_script(uint16_t new_crc) { 
    uint32_t script_len = m_Eeprom.size(); 
    if(script_len > (m_Eeprom.max_size() ) )
    {
        ESP_LOGE(m_Tag, "script_len too large:  %d", script_len );
        return false;
    }
    const size_t i_step = 16;
    uint8_t temp_buffer[i_step];
    crc_advancer crc;

    for (size_t i = 0; i < script_len; i+=i_step){
        size_t len = i_step;
        if((script_len - i) < i_step) len = script_len - i;
        if(!m_Eeprom.read_data(temp_buffer, i, len))
            ESP_LOGE(m_Tag, "read data failed");
        crc.add_buffer(temp_buffer, len);
    }

    //check if the dumb thing is actually valid
    if (new_crc != crc.get_crc() ) {
        ESP_LOGE(m_Tag, "crc not matched expected: %d, calculated:%d", new_crc, crc.get_crc());
        return false;
    }


    m_ScriptLength = script_len;
    m_ScriptCrc = crc.get_crc();

    ESP_LOGI(m_Tag, "validated script of len %d with crc %d",
            m_ScriptLength, m_ScriptCrc);
    m_Eeprom.write_file_metadata(m_ScriptLength, m_ScriptCrc);
    m_ScriptValid = true;
    return true;
}

