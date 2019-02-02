#ifndef _EEPROM_CAL_CONTROLLER_H_
#define _EEPROM_CAL_CONTROLLER_H_

#include "eeprom_controller.hpp"
#include <string>
#include <map>
#include <memory>


class eeprom_cal_controller: public eeprom_utils {
    public:
        struct cal_metadata {
            uint32_t length;
            uint16_t crc;
            uint32_t start_page;
        };
        eeprom_cal_controller(size_t metadata_page);
        virtual bool populate_metadata(void);
        bool get_cal_metadata(cal_metadata& data);
        uint16_t get_cached_crc(void) { return m_metadata.crc;} 
        uint16_t get_cached_length(void) { return m_metadata.length; }
        bool write_metadata(const cal_metadata& new_metadata);
        bool validate_self(void);
        float get_cal_data(char* key);
        bool populate_cal_data(void);
    private:
        std::map<std::string,float> m_CalData;
        cal_metadata m_metadata;
        constexpr static const char* m_Tag = "eeCalController";
};
#endif //_EEPROM_CAL_CONTROLLER_H_
