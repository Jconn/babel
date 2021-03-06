#include "eeprom_file_controller.hpp"
#include "eeprom_controller.hpp"
#include "babel_utils.hpp"
#include "esp_log.h"

eeprom_file_controller::eeprom_file_controller(size_t metadata_page)
    : eeprom_utils(metadata_page) 
{
}

eeprom_file_controller::eeprom_file_controller(size_t metadata_page, 
        size_t init_page)
    : eeprom_utils(metadata_page, init_page) 
{

}


bool eeprom_file_controller::validate_self() {
    if(!populate_metadata())
    {
        set_validated(false);
        return false;
    }
    return validate(m_metadata.length, m_metadata.crc);
}

bool eeprom_file_controller::populate_metadata(void)
{
    uint8_t raw_buf[get_page_size()];
    if(!read_page(raw_buf,
            m_metadataPage,
            get_page_size()))
    {
        ESP_LOGE(m_Tag, "cal metadata pop failed "); 
        return false;
    }
    uint32_t offset = 0;

    offset += babel::deserialize_u32(&(raw_buf[offset]), &m_metadata.length);
    offset += babel::deserialize_u16(&(raw_buf[offset]), &m_metadata.crc);

    return true;

}

bool eeprom_file_controller::commit_changes(uint32_t new_len, uint16_t new_crc)
{
    ESP_LOGE(m_Tag, "commiting new section with len %d, crc %d", new_len, new_crc); 
    file_metadata new_metadata;
    new_metadata.crc = new_crc;
    new_metadata.length = new_len;
    return write_metadata(new_metadata);
}
bool eeprom_file_controller::write_metadata(const file_metadata &new_metadata)
{
    uint8_t raw_buf[get_page_size()];
    uint32_t offset = 0;
    offset += babel::serialize_u32(&(raw_buf[offset]), new_metadata.length);
    offset += babel::serialize_u16(&(raw_buf[offset]), new_metadata.crc);
    if(!write_page(
                raw_buf,
                m_metadataPage,
                offset) ) 
    {
        return false;
    }
    m_metadata = new_metadata;
    return true;
}


bool eeprom_file_controller::confirm_valid(void)
{
    const file_metadata old_data = m_metadata; 
    if(!validated())
    {
        goto error;
    }
    if(!populate_metadata()) goto error;
    if(old_data.crc != m_metadata.crc) goto error;
    set_validated(true);
    return true;
error:
    set_validated(false);
    return false;
}

