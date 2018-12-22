#include "script_controller.hpp"
#include "babel_utils.hpp"
void ScriptController::write_script(const std::vector<uint8_t> &script) {
    m_Eeprom.begin_new_file();
    m_Eeprom.write_file_metadata(m_ScriptLength, m_ScriptCrc);
    m_Eeprom.append_data(script);
}

void ScriptController::append_data(const std::vector <uint8_t> &data) {
    m_Eeprom.append_data(script);
}


bool ScriptController::validate_script(uint16_t new_crc, uint32_t script_len) { 
    const size_t i_step = 16;
    uint8_t temp_buffer[i_step];
    uint16_t crc = 0;

    for (size_t i = 0; i < script_len; i+=i_step){
        size_t len = i_step;
        if((script_len - i) < i_step) len = script_len - i;
        m_Eeprom.read_data(temp_buffer, i, len);   
        crc = compute_crc16_buffer(temp_buffer, len, crc);
    }

    //check if the dumb thing is actually valid
    if (new_crc != crc) {
        return false;
    }

    m_ScriptLength = script_len;
    m_ScriptCrc = new_crc;
    return true;
}

