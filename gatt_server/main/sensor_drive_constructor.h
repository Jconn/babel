#ifndef ACTIVATE_PROFILE_H
#define ACTIVATE_PROFILE_H

#include <stdio.h>
#include <stdlib.h>
#include "driver/i2c.h"

typedef enum i2c_actions {
    START = 1, 
    WRITE_ACK = 2, 
    WRITE_NACK = 3,
    READ_ACK = 4, 
    READ_NACK = 5, 
    STOP = 6
} tI2cActions; 

typedef enum {
    DRIVE_TYPE_ANALOG = 0x1,
    DRIVE_TYPE_I2C = 0x2
} tDriveType;

typedef struct i2c_capture {
    i2c_cmd_handle_t cmd; 
    uint8_t rxBuffer[32];
    int rxBufLength;
} tIicManager;


void activate_profile(void);
int get_sensor(uint8_t* data_buf);
void i2c_sensor_init(void);
void process_byte(tI2cActions *current_command, 
                    tI2cActions *prev_command,
                    tIicManager *manager
                    );
#endif
