
#ifndef _EEPROM_FILE_CONTROLLER_H_
#define _EEPROM_FILE_CONTROLLER_H_

#include "eeprom_controller.hpp"



class eeprom_file_controller: public eeprom_utils {
    public:
        struct file_metadata {
            uint16_t crc;
            uint32_t length;
        };
        eeprom_file_controller(size_t metadata_page, size_t init_page);
        eeprom_file_controller(size_t metadata_page);
        virtual bool populate_metadata(void);
        bool get_metadata(file_metadata& data);
        uint16_t get_cached_crc(void) { return m_metadata.crc;} 
        uint16_t get_cached_length(void) { return m_metadata.length; }
        bool write_metadata(const file_metadata &new_metadata);
        virtual bool validate_self(void);
        virtual bool commit_changes(uint32_t new_len, uint16_t new_crc); 
        bool confirm_valid(void);
    private:
        file_metadata m_metadata;
        constexpr static const char* m_Tag = "eeCalController";
};
#endif //_EEPROM_FILE_CONTROLLER_H_
