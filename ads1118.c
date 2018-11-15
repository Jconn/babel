#include "driver/spi_master.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>



#include "ads1118.h"
  
#define PIN_NUM_MISO 12 
#define PIN_NUM_MOSI 13 
#define PIN_NUM_CLK  14 
#define PIN_NUM_CS   25 

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 16

static spi_device_handle_t spi;

#define DEFAULT_ADS_CONFIG { \
    .idle = true, \
    .mux_state = Abs0, \
    .mode = one_shot,\
    .sample_frequency = SPS_32, \
    .pullup_status  = pullup_enabled, \
    .gain_state = Gain4p096, \
    .sensor_mode = adc_sensor, \
}

float get_dynamic_range(ext_adc *adc)
{
    switch(adc->gain_state)
    {
        case Gain6p144:
            return 6.144;
            break;
        case Gain4p096:
            return 4.096;
            break;
        case Gain2p048:
            return 2.048;
            break;
        case Gain1p024:
            return 1.024;
            break;
        case Gain0p512:
            return 0.512;
            break;
        case Gain0p256:
            return 0.256;
            break;
        default:
            return 0;
    }
}

void default_one_shot_config(ext_adc *adc)
{
    ext_adc default_adc = DEFAULT_ADS_CONFIG;
    memcpy(adc, &default_adc, sizeof(default_adc));
}
uint16_t get_config(ext_adc* adc)
{
    uint16_t config = 0;
    config |= ( (adc->mux_state & 0x7) << 12);
    config |= ( (adc->gain_state & 0x7) << 9);
    config |= ( (adc->mode & 0x1) << 8);
    config |= ( (adc->sample_frequency & 0x7) << 5 );
    config |= ( (adc->sensor_mode & 0x1) << 4 );
    config |= ( (adc->pullup_status & 0x1) << 3 );
    config |= (NOP_VALID_VALUE << 1 );
    config |= ( RESERVED_FIELD_VALUE << 0);
    return config;
}

bool sync_config(ext_adc* adc)
{
    uint8_t data[2];
    uint16_t config = get_config(adc);

    //serialize
    data[0] = config >> 8;
    data[1] = config & 0xFF;
    int len = 2;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    return spi_device_transmit(spi, &t); //blocking
}


float get_measurement(ext_adc* adc)
{
    //write to spi,
    //
    //read spi until the measurement is done
    uint8_t data[4];
    uint16_t config = get_config(adc);

    config |= 1 << 15; //start conversion
    //serialize
    data[0] = config >> 8;
    data[1] = config & 0xFF;
    int len = 2;
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    ret = spi_device_transmit(spi, &t); //blocking

    //read until our data comes back as transaction not happening
    memset(data, 0, sizeof(data));
    len = 4;
    uint8_t response[4];
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.rx_buffer = response;
    ret = spi_device_transmit(spi, &t); //blocking
    assert(ret==ESP_OK);            //Should have had no issues.

    ESP_LOGI("ads1118", "got ads data:");
    esp_log_buffer_hex("ads1118", response, len);

    int16_t raw_result = (response[0] << 8 | response[1]);
    float result = (float) raw_result/0x7FFF;
    result *= get_dynamic_range(adc) * 1000; 
    return result;
}

bool init_adsdevice(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=PARALLEL_LINES*320*2+8
    };
    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));

    devcfg.cs_ena_pretrans = 2;
    devcfg.cs_ena_posttrans = 2;
    devcfg.clock_speed_hz=100000;           //Clock out at 1 MHz
    devcfg.mode=1;                                //SPI mode 1 
    devcfg.spics_io_num=PIN_NUM_CS;               //CS pin
    devcfg.queue_size=3;                          //We want to be able to queue 3 transactions at a time
    
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    return ret;

}
