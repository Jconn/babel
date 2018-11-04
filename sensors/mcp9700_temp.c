
#include "sample_capture.h"
#include <stdio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "eeprom_config.h"


static int getTemp(char* Buffer);


void activate_mpc9700(tSampleCapture *active_sensor)
{
    active_sensor->getSample = getTemp;
    active_sensor->initModule = initADC;
    init_sensor();
}


static int getTemp(char* Buffer)
{ 
    uint32_t voltage = getADCmV();
    uint32_t v_c0 = 500;
    uint32_t tempCo = 10;

    float temp = ((float)voltage - v_c0)/(float)tempCo;
    int len =  sprintf(Buffer,"result:%fC",temp); 
    return len;
}


