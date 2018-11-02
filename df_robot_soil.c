#include "sample_capture.h"
#include <stdio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "eeprom_config.h"
#include "sample_capture.h"
/*
When VDD_A is 3.3V:

0dB attenuaton (ADC_ATTEN_DB_0) gives full-scale voltage 1.1V
2.5dB attenuation (ADC_ATTEN_DB_2_5) gives full-scale voltage 1.5V
6dB attenuation (ADC_ATTEN_DB_6) gives full-scale voltage 2.2V
11dB attenuation (ADC_ATTEN_DB_11) gives full-scale voltage 3.9V (see note below)
*/
#define ADC_MAX_MV (3300)


void activate_df_robot_soil(tSampleCapture *active_sensor)
{
    active_sensor->getSample = getADCConversion;
    active_sensor->initModule = initADC;
    init_sensor();
}

/*
int getSoilMoisture(char* Buffer)
{ 
    uint32_t voltage = getADCmV();
    int len = 0;
    if(voltage > (7*ADC_MAX_MV/10) )
    {
        len =  sprintf(Buffer,"flooded"); 
    }
    else if(voltage > (3 * ADC_MAX_MV/10) )
    {
        len =  sprintf(Buffer,"humid"); 
    }
    else
    {
        len =  sprintf(Buffer,"dry"); 
    }

    return len;
}

*/
