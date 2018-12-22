#ifndef _EEPROM_CONTROLLER_H_
#define _EEPROM_CONTROLLER_H_
#include <vector>
class eeprom_utils {
    public:
        void begin_new_file(void) {m_offset = 0;}
        bool append_data(const std::vector <uint8_t> &data);
        bool write_data(uint8_t *data, uint16_t page, uint16_t length, uint16_t offset = 0);
        bool read_page(uint8_t *outData, uint16_t page, uint16_t length, uint16_t offset = 0);
        bool read_data(uint8_t *outData, uint16_t offset, uint16_t length);
        bool write_file_metadata(uint32_t profile_val, uint16_t crc);
    private:
        static const size_t page_size = 16;
        static const size_t mc_metaDataPage = 0;
        static const size_t mc_metaDataLen = 6;
        size_t m_offset = 0;
};


#endif //_EEPROM_CONTROLLER_H_
