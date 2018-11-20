#ifndef EEPROM_CONFIG_H_
#define EEPROM_CONFIG_H_ 

#include "sdkconfig.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define WRITE_BIT                          0 /*!< I2C master write */
#define READ_BIT                           1  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */
#define EEPROM_PAGE_LENGTH 16
typedef enum {
    SENS_MCP9700 = 0x1,
    SENS_SOIL_ANALOG = 0x2,
    SENS_MAG_FIELD_ANALOG = 0x3,
    SENS_BMP_280 = 0x4
} tSensorProfiles;

typedef struct _msgManager {
    bool processing_msg;
    uint32_t total_msg_length;
    uint32_t current_msg_length;
    uint8_t msg_bytes[256];
} tBabelMsgHandler;

void eeprom_init(void* arg);
esp_err_t eeprom_read(uint8_t page_addr, int length, uint8_t* outBuffer);
esp_err_t eeprom_write(uint8_t page_addr, int length, uint8_t* inBuffer);

int set_profile(uint32_t profile_val);

int getADCConversion(char* Buffer);



bool manage_new_bytes( tBabelMsgHandler *manager,
                        uint8_t *new_data,
                        uint32_t new_data_len,
                        bool(*msg_handler)(tBabelMsgHandler*) );

char* get_script(void);

char* get_script_name(void);

bool store_script(uint8_t* program);

QueueHandle_t get_programmer_queue(void);

#endif //EEPROM_CONFIG_H_

