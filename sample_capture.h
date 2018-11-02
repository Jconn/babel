#ifndef _SAMPLE_CAPTURE_H_
#define _SAMPLE_CAPTURE_H_

#include <stdint.h>

//
//each sampling subsytem wants to grab a sample
//and stuff it into the char buffer, that the bluetooth handler
//will agnostically send OTA
//
typedef struct {
    int (*getSample)(char* Buffer);
    void (*initModule)(void);
} tSampleCapture;

void activate_adc(void);

void init_sensor(void);

int getProcessedSample(char* input_data);

void initADC(void);

tSampleCapture* get_active_sensor(void);

uint32_t getADCmV(void);

int getADCConversion(char* Buffer);
#endif //_SAMPLE_CAPTURE_H_
