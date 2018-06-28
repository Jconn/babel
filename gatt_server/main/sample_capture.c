#include "sample_capture.h"
#include <stdio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() 
                                    //to obtain a better estimate
//defines how the sample capture is going to work

//the default stuff for the adc
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

static esp_adc_cal_characteristics_t *adc_chars;

static tSampleCapture active_sensor;

static int getADCConversion(char* Buffer);
static void initADC(void);
static void check_efuse();

static void print_char_val_type(esp_adc_cal_value_t val_type);

void activate_adc(void)
{
    active_sensor.getSample = getADCConversion;
    active_sensor.initModule = initADC;
    init_sensor();
}


void init_sensor(void)
{
    active_sensor.initModule();
}

int getProcessedSample(char* input_data)
{
    return active_sensor.getSample(input_data); 
}


static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void initADC(void)
{
    //configure adc
    check_efuse();
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, atten);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

}


static int getADCConversion(char* Buffer)
{ 
    uint32_t adc_reading  = adc1_get_raw((adc1_channel_t)channel);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    int len =  sprintf(Buffer,"result:%dmV",voltage); 
    return len;
}


static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}
