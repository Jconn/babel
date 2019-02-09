#ifndef _EEPROM_CONTROLLER_H_
#define _EEPROM_CONTROLLER_H_

#ifdef __cplusplus
#include <vector>
#include "stddef.h"

class eeprom_utils {
    public:
        eeprom_utils(size_t metadata_page);
        eeprom_utils(size_t metadata_page, size_t init_page);
        void begin_new_file(void) {m_offset = 0;}
        size_t size(void) { return m_offset;}
        bool append_data(const std::vector <uint8_t> &data);
        bool append_data(const uint8_t *payload, size_t len);
        bool write_data(const uint8_t *payload, size_t len, size_t offset);
        static bool write_page(const uint8_t *data, uint16_t page, uint16_t length, uint16_t offset = 0);
        static bool read_page(uint8_t *outData, uint16_t page, uint16_t length, uint16_t offset = 0);
        bool read_data(uint8_t *outData, uint16_t offset, uint16_t length);
        virtual bool populate_metadata(void) = 0;
        virtual bool validate_self(void) = 0;
        virtual bool commit_changes(uint32_t new_len, uint16_t new_crc) = 0;
        static size_t max_size(void) { return page_size * 0xFF;}
        
        bool validated() { return m_validated; }
        void set_validated(bool new_status){ m_validated = new_status; }
        bool confirm_eeprom();
        static constexpr size_t get_page_size() {return  page_size;}
        static constexpr size_t get_last_page() { return total_pages - 1; }
    private:
        uint16_t page_thresholded_length(uint16_t raw_length) { 
            if (raw_length > page_size) return page_size;
            return raw_length;                                                               
        }
    protected:
        size_t m_metadataPage;
        size_t m_scriptPageBegin;

        bool validate(size_t script_len, uint16_t new_crc); 
        void set_begin_page(size_t page) { m_scriptPageBegin = page;}
    private:
        bool m_validated;
        size_t m_offset = 0;
        constexpr static const char* m_Tag = "eeController";
        constexpr static size_t page_size = 16;
        constexpr static size_t total_pages = 512;
};

class eeprom_consumer {
    public:
    eeprom_consumer() {}
    eeprom_consumer(eeprom_utils* my_utils)
        :m_utils(my_utils)
    {

    }

    void init_consumer(eeprom_utils* eeprom){m_utils = eeprom;}

    bool read_data(eeprom_utils* eeprom, uint8_t* data, size_t length)
    {
        if(!m_utils->read_data(data, m_offset, length))
        {
            return false;
        }
        m_offset += length;
        return true;
    }

    bool write_data(uint8_t* data, size_t length)
    {
        if(!m_utils->write_data(data, m_offset, length))
        {
            return false;
        }
        m_offset += length;
        return true;
    }

    size_t data_consumed(void) const { return m_offset;}
    public:
        size_t m_offset = 0;
        eeprom_utils* m_utils = nullptr;
};


#endif //#ifdef __cplusplus

#endif //_EEPROM_CONTROLLER_H_
