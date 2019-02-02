#include "eeprom_cal_controller.hpp"
#include "eeprom_controller.hpp"
#include "babel_utils.hpp"
#include "esp_log.h"
#include <memory>
#include <cstring>

bool eeprom_cal_controller::populate_metadata(void)
{
    uint8_t raw_buf[page_size];
    if(!read_page(raw_buf,
            m_metadataPage,
            page_size))
    {
        ESP_LOGE(m_Tag, "cal metadata pop failed "); 
        return false;
    }
    uint32_t offset = 0;

    offset += babel::deserialize_u32(&(raw_buf[offset]), &m_metadata.length);
    offset += babel::deserialize_u16(&(raw_buf[offset]), &m_metadata.crc);
    offset += babel::deserialize_u32(&(raw_buf[offset]), &m_metadata.start_page);

    set_begin_page(m_metadata.start_page);

    return true;
}


bool eeprom_cal_controller::get_cal_metadata(cal_metadata &data)
{
    if(!populate_metadata()){
        return false;
    }
    data = m_metadata; 

    return true;
}

bool eeprom_cal_controller::validate_self() {
    if(!populate_metadata())
    {
        return false;
    }
    return validate(m_metadata.length, m_metadata.crc);
}

bool eeprom_cal_controller::write_metadata(const cal_metadata& new_metadata)
{
    uint8_t raw_buf[page_size];
    uint32_t offset = 0;
    offset += babel::serialize_u32(&(raw_buf[offset]), new_metadata.length);
    offset += babel::serialize_u16(&(raw_buf[offset]), new_metadata.crc);
    offset += babel::serialize_u32(&(raw_buf[offset]), new_metadata.start_page);
    if(!write_data(
                raw_buf,
                m_metadataPage,
                offset) ) 
    {
        return false;
    }
    m_metadata = new_metadata;
    return true;
}


bool eeprom_cal_controller::populate_cal_data(void) {
    if(!validated()) {
        ESP_LOGE(m_Tag, "trying to populate cal data without validation");
        return false;
    }
    //max key string length of 48 chars 
    uint8_t raw_buf[64];
    size_t offset = 0;
    const size_t i_step = 16;
    //need algorithm for string to float
    for (size_t i = 0; i < m_metadata.length; i+=i_step){
        //curtail length
        size_t len = i_step;
        if((m_metadata.length - i) < i_step) {
            len = m_metadata.length - i;
        }
        //read latest page
        if(!read_page(&(raw_buf[offset]),
                    m_metadata.start_page + (i / eeprom_utils::page_size),
                    len))
        {
            ESP_LOGE(m_Tag, "cal read data failed");
            return false;
        }
        //look for string
        for(size_t j = offset; j < offset + len; ++j)
        {
            //null character ends cal data str
            if(raw_buf[j] == 0)
            {
                // if no data in this set, pick it up
                // in the next read
                if( j > (offset + len - 4))
                {
                    break;
                }
                else
                {
                    //
                    //TODO: make sure key memory is being copied off
                    //make sure value is being implicitly converted right
                    //
                    uint8_t* key = &(raw_buf[0]); 
                    std::string temp((char*)key); 
                    std::unique_ptr<std::string> safe_key = std::make_unique<std::string>(temp);
                    float value = key[j];
                    //m_CalData.add(std::pair<std::string,float>(safe_key, value) ); 
                    m_CalData[*safe_key] = value;
                    int unused_index = j + 4;

                    memmove(raw_buf, 
                            &(raw_buf[unused_index]),
                            offset+len - unused_index);
                    len = offset + len - unused_index; 
                    offset = 0;
                    j = 0;
                }
            }
        }
        offset += len;
    }
    return true;
}



float eeprom_cal_controller::get_cal_data(char* key)
{

  auto it = m_CalData.find(key);
  if (it != m_CalData.end())
  {
      return it->second; 
  }
  return 0.0;
}
