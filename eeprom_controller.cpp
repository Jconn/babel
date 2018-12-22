#include "driver/i2c.h"
#include "eeprom_controller.hpp"
#include "babel_utils.h"
#include "esp_log.h"
#include "eeprom_config.h"
#include "display_manager.h"
#define EEPROM_I2C_ADDR (0x50)

const char* EEPROM_TAG = "eeController";
bool eeprom_utils::append_data(const std::vector <uint8_t> &data)
{
    for(size_t i = 0; i < data.size(); i += page_size - (m_offset % 16) )
    {
        int write_len = (data.size() - i) < page_size ?
            (data.size() - i) : page_size;
        if (write_len > (page_size - (m_offset % 16)) )
        {
            write_len -= ( m_offset % 16);
        }

        write_data(&(data.data()[i]), m_offset / page_size, 
                        write_len,
                        m_offset % page_size);
        m_offset += write_len;
    }
    return true;
}


bool eeprom_utils::write_data(uint8_t *data, uint16_t page, uint16_t length, uint16_t offset)
{
    ESP_LOGI(EEPROM_TAG, "eWrite: page %d, len %d, value: ", page, length); 
    
    //
    // there are 8 sections of 256 bytes 
    // so only 16 pages per section
    // the api to this eeprom abstracts this away
    //
    uint8_t i2c_addr = EEPROM_I2C_ADDR | (page/16);
    esp_log_buffer_hex(EEPROM_TAG, data, length);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
            (i2c_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);

    //the entire block is the word
    i2c_master_write_byte(cmd, 16 * (page%16), ACK_CHECK_EN);
    i2c_master_write(cmd, data, length, ACK_CHECK_EN);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(
            EEPROM_PORT,
            cmd,
            1000 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);
    return ret > 0;
}


bool eeprom_utils::read_page(uint8_t *outData, uint16_t page, uint16_t length, uint16_t offset)
{
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
    i2c_master_read(cmd, outData, length-1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &(outData[length-1]), I2C_MASTER_LAST_NACK);
//
    //final stop condition
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(
            EEPROM_PORT, 
            cmd,
            1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret > 0;
}
 

bool eeprom_utils::write_file_metadata(uint32_t profile_val, uint16_t crc)
{
    uint8_t raw_buf[page_size];
    uint32_t offset = 0;
    offset += serialize_u32(&(raw_buf[offset]), profile_val);
    offset += serialize_u16(&(raw_buf[offset]), crc);
    return write_data(
            raw_buf,
            mc_metaDataPage,
            mc_metaDataLen);
}
