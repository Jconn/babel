#ifndef EEPROM_CONFIG_H_
#define EEPROM_CONFIG_H_ 

#include "sdkconfig.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C"
{
#endif
    void script_controller_event_loop_init(void* arg);

    QueueHandle_t get_programmer_queue(void);

    void script_controller_event_loop_init(void* arg);

    bool script_ready(void);

#ifdef __cplusplus
}
#endif

#endif //EEPROM_CONFIG_H_

