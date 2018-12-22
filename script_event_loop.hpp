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

void eeprom_init(void* arg);

QueueHandle_t get_programmer_queue(void);


#endif //EEPROM_CONFIG_H_

