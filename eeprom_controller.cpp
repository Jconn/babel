#include "driver/i2c.h"
#include "eeprom_controller.hpp"
#include "babel_utils.hpp"
#include "esp_log.h"
#include "display_manager.h"
#include <stdio.h>
#include <string.h>
#define EEPROM_I2C_ADDR (0x50)

#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */

eeprom_utils::eeprom_utils(size_t metadata_page) 
    : m_metadataPage(metadata_page) 
{

}

eeprom_utils::eeprom_utils(size_t metadata_page, size_t init_page) 
    :m_metadataPage(metadata_page), m_scriptPageBegin(init_page) 
{

}

bool eeprom_utils::write_data(const uint8_t *payload, size_t len, size_t offset)
{
    uint16_t page_offset = (offset) % page_size; 
    uint16_t end_byte = (offset + page_size) + len;
    //adding the first page to this logic
    uint16_t page_begin = (offset /page_size) + m_scriptPageBegin;
    uint16_t page_end = end_byte/page_size;
    if ( (end_byte % page_size) > 0)
        page_end +=1;

    size_t data_offset = 0;
    for(size_t i = page_begin; i < page_end; ++i)
    {

        uint16_t page_len_write = page_thresholded_length(end_byte - i*page_size);
        page_len_write -= page_offset;
        if (!write_page(&(payload[data_offset]), 
                            i,
                            page_len_write,
                            page_offset)) 
        {
            ESP_LOGE(m_Tag, "eWrite: page %d, len %d, offset %d", i, 
                    len, page_offset); 
            return false;
        }
        data_offset += page_len_write;
        page_offset = 0;

    }
    offset += len;
    ESP_LOGI(m_Tag, "appended data in pages %d through %d", page_begin, page_end);
    return true;

}

bool eeprom_utils::append_data(const std::vector <uint8_t> &data)
{
    return append_data(data.data(), data.size());
}


bool eeprom_utils::write_page(const uint8_t *data, uint16_t page, uint16_t length, uint16_t offset)
{
    ESP_LOGI(m_Tag, "eWrite: page %d, len %d, offset %d", page, length, offset); 
    if( (length + offset) > page_size)
    {
        ESP_LOGE(m_Tag, "eWrite: page %d, len %d, offset %d", page, length, offset); 
    }
    uint8_t temp_buffer[page_size];
    if(offset != 0)
    {
        read_page(temp_buffer, page, page_size);
    }
    memcpy(&temp_buffer[offset], data, length);
         
    //
    // there are 8 sections of 256 bytes 
    // so only 16 pages per section
    // the api to this eeprom abstracts this away
    //
    uint8_t i2c_addr = EEPROM_I2C_ADDR | (page/16);
    esp_log_buffer_hex(m_Tag, data, length);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
            (i2c_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);

    //the entire block is the word
    i2c_master_write_byte(cmd, 16 * (page%16), ACK_CHECK_EN);
    i2c_master_write(cmd, temp_buffer, length + offset, ACK_CHECK_EN);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(
            EEPROM_PORT,
            cmd,
            1000 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}


bool eeprom_utils::read_data(uint8_t *outData, uint16_t offset, uint16_t length)
{
    //have to add the metadata size
    offset += page_size + m_scriptPageBegin * page_size; 
    
    uint16_t page_offset = offset % page_size; 
    uint16_t end_byte = offset + length;
    uint16_t page_begin = offset/page_size;
    uint16_t page_end = end_byte/page_size ;
    if ( (end_byte % page_size) > 0)
        page_end +=1;

    uint16_t data_offset = 0;

    for(uint16_t i = page_begin; i < page_end; ++i) 
    {
        uint16_t page_len_read = page_thresholded_length(end_byte - i*page_size);
        page_len_read -= page_offset;
        if (!read_page(&(outData[data_offset]), i, page_len_read, page_offset)) 
            return false;
        data_offset += page_len_read;
        page_offset = 0;
    }
    return true;
}

bool eeprom_utils::validate(size_t script_len, uint16_t new_crc) { 

    const size_t i_step = 16;
    uint8_t temp_buffer[i_step];
    crc_advancer crc;


    if(script_len == 0) {
        return true;
    }

    if(script_len > max_size() )
    {
        ESP_LOGE(m_Tag, "script_len too large:  %d", script_len );
        return false;
    }


    for (size_t i = 0; i < script_len; i+=i_step){
        size_t len = i_step;
        if((script_len - i) < i_step) len = script_len - i;
        if(!read_data(temp_buffer, i, len))
            ESP_LOGE(m_Tag, "read data failed");
        crc.add_buffer(temp_buffer, len);
    }

    //check if the dumb thing is actually valid
    if (new_crc != crc.get_crc() ) {
        ESP_LOGE(m_Tag, "crc not matched expected: %d, calculated:%d", new_crc, crc.get_crc());
        return false;
    }

    ESP_LOGI(m_Tag, "validated script of len %d with crc %d",
            script_len, crc.get_crc());
    return true;
}

bool eeprom_utils::read_page(uint8_t *outData, uint16_t page, uint16_t length, uint16_t offset)
{
    ESP_LOGI(m_Tag, "eRead: page %d, len %d, offset %d", page, length, offset); 
    if (offset + length > page_size)
    {
        ESP_LOGE(m_Tag, "read assert: page %d, len %d, offset %d ", 
                        page, 
                        length, 
                        offset); 

        return false;
    }

    uint8_t temp_buffer[page_size];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //start condition
    i2c_master_start(cmd);


    uint8_t i2c_addr = EEPROM_I2C_ADDR | (page/16);

    //addr write
    i2c_master_write_byte(cmd,
                    (i2c_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    //setting page addr
    i2c_master_write_byte(cmd, (page % 16) * 16, ACK_CHECK_EN);
    
    //repeated start condition for next txn 
    i2c_master_start(cmd);
    
    //addr write for the read txn 
    i2c_master_write_byte(cmd,
                    (i2c_addr << 1 ) | READ_BIT, ACK_CHECK_EN);

    //the eeprom datasheet specs that ACK is required on 
    //all bytes, not just the first n-1 bytes
    //read bytes
    i2c_master_read(cmd, temp_buffer, page_size-1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &(temp_buffer[page_size-1]), I2C_MASTER_LAST_NACK);

    //final stop condition
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(
            EEPROM_PORT, 
            cmd,
            1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    if(ret == ESP_OK)
    {
        memcpy(outData, &(temp_buffer[offset]), length);
        return true;
    }
    return false;
}
 


