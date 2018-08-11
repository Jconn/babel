#ifndef BMP_280_TEMP_H_
#define BMP_280_TEMP_H_ 

void activate_bmp280(tSampleCapture *active_sensor);

#define BMP_280_PORT             I2C_NUM_0        /*!< I2C port number for master dev */

#endif //BMP_280_TEMP_H_
