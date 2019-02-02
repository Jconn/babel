#ifndef __ADS_1118_H_
#define __ADS_1118_H_
#include "stdint.h"

#define PIN_NUM_MISO 12 
#define PIN_NUM_MOSI 13 
#define PIN_NUM_CLK  14 
#define PIN_NUM_ADC_CS   25 

//[2:1] in config register - magic number for data 
//                          sanitation
#define NOP_VALID_VALUE (0x01)
#define RESERVED_FIELD_VALUE (0x1)

//this is a bitmap in the config register 
typedef enum ads1118_MuxState {
    Diff0to1 = 0x0, //default
    Diff0to3 = 0x1,
    Diff1to3 = 0x2,
    Diff2to3 = 0x3,
    Abs0 = 0x4,
    Abs1 = 0x5,
    Abs2 = 0x6,
    Abs3 = 0x7
} tMuxState;

typedef enum ads1118_ProgrammableGain{
    Gain6p144 = 0x0, 
    Gain4p096 = 0x1,
    Gain2p048 = 0x2, //default
    Gain1p024 = 0x3,
    Gain0p512 = 0x4,
    Gain0p256 = 0x5
} tGainState;

typedef enum ads1118_dataRate {
    SPS_8 = 0x0, 
    SPS_16 = 0x1,
    SPS_32= 0x2, 
    SPS_64 = 0x3,
    SPS_128 = 0x4, //default
    SPS_250 = 0x5,
    SPS_475 = 0x6,
    SPS_860 = 0x7
} tDataRate;

typedef enum ads1118_OperatingMode{
    continuous = 0,
    one_shot = 1 //default
} tOperatingMode;

typedef enum ads1118_PullupStatus{
    pullup_disabled = 0,
    pullup_enabled = 1 //default
} tPullupStatus;

typedef enum ads1118_SensorMode{
    adc_sensor = 0, //default
    temp_sensor = 1 
} tSensorMode;


typedef struct ext_ads1118{
    bool idle;
    tMuxState mux_state;
    tOperatingMode mode;
    tDataRate sample_frequency;
    tPullupStatus pullup_status;
    tGainState gain_state;
    tSensorMode sensor_mode;
} ads1118;


typedef ads1118 ext_adc;

bool sync_config(ext_adc* adc);

float get_measurement(ext_adc* adc);

bool init_adsdevice(void);

void default_one_shot_config(ext_adc *adc);

float get_dynamic_range(ext_adc *adc);
#endif // __ADS_1118_H_


