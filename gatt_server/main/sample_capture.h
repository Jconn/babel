#ifndef _SAMPLE_CAPTURE_H_
#define _SAMPLE_CAPTURE_H_

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

#endif //_SAMPLE_CAPTURE_H_
