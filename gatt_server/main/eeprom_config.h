#ifndef EEPROM_CONFIG_H_
#define EEPROM_CONFIG_H_ 
typedef enum {
    SENS_MCP9700 = 0x1,
    SENS_SOIL_ANALOG = 0x2,
    SENS_MAG_FIELD_ANALOG = 0x3
} tSensorProfiles;


void eeprom_init(void* arg);
esp_err_t eeprom_read(uint8_t page_addr, int length, uint8_t* outBuffer);
esp_err_t eeprom_write(uint8_t page_addr, int length, uint8_t* inBuffer);

int set_profile(uint32_t profile_val);

int getADCConversion(char* Buffer);
#endif //EEPROM_CONFIG_H_

