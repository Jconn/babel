#ifndef _EEPROM_CONTROLLER_H_
#define _EEPROM_CONTROLLER_H_

#ifdef __cplusplus
#include <vector>
class eeprom_utils {
    public:
        void begin_new_file(void) {m_offset = 0;}
        size_t size(void) { return m_offset;}
        bool append_data(const std::vector <uint8_t> &data);
        bool append_data(const uint8_t *payload, size_t len);
        bool write_data(const uint8_t *data, uint16_t page, uint16_t length, uint16_t offset = 0);
        bool read_page(uint8_t *outData, uint16_t page, uint16_t length, uint16_t offset = 0);
        bool read_data(uint8_t *outData, uint16_t offset, uint16_t length);
        bool write_file_metadata(uint32_t profile_val, uint16_t crc);
        bool populate_metadata(void);
        uint16_t get_cached_crc(void) { return m_crc; }
        static size_t max_size(void) { return page_size * 0xFF;}
    private:
        uint16_t page_thresholded_length(uint16_t raw_length) { 
            if (raw_length > page_size) return page_size;
            return raw_length;                                                               
        }
        static const size_t page_size = 16;
        static const size_t mc_metaDataPage = 0;
        static const size_t mc_metaDataLen = 6;
        size_t m_offset = 0;
        uint16_t m_crc = 0;
        constexpr static const char* m_Tag = "eeController";
};
#endif //#ifdef __cplusplus

#endif //_EEPROM_CONTROLLER_H_
