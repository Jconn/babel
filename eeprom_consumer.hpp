#ifndef _EEPROM_CONSUMER_H_ 
#define _EEPROM_CONSUMER_H_


#include "esp_log.h"
#include "script_controller.hpp"

class eeprom_consumer {
    public:
        eeprom_consumer() { m_offset = 0;}
        eeprom_consumer(eeprom_utils* my_utils)
            : m_offset(0), m_expected_length(0), m_utils(my_utils)
        {
        }

        void init_consumer(eeprom_utils* eeprom){m_utils = eeprom;}

        bool read_data(eeprom_utils* eeprom, uint8_t* data, size_t length)
        {
            if(m_utils == nullptr) {

                ESP_LOGE(m_Tag, "consumer points to invalid eeprom"); 
                return false;
            }

            if(!m_utils->read_data(data, m_offset, length))
            {
                return false;
            }
            m_offset += length;
            return true;
        }

        bool write_data(uint8_t* data, size_t length)
        {
            if(m_utils == nullptr) {

                ESP_LOGE(m_Tag, "consumer points to invalid eeprom"); 
                return false;
            }

            if(!m_utils->write_data(data, length, m_offset))
            {
                return false;
            }
            m_offset += length;
            return true;
        }

        size_t data_consumed(void) const { return m_offset;}
    public:
        constexpr static const char* m_Tag = "eeConsumer";
        size_t m_offset = 0;
        size_t m_expected_length = 0;
        eeprom_utils* m_utils = nullptr;
};

#endif //_EEPROM_CONSUMER_H_
