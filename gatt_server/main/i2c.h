

#ifndef I2C_H_
#define I2C_H_ 
bool write_data(uint8_t* buffer, int length, int address);
bool read_data(uint8_t* buffer, int length, int address);
#endif //#define I2C_H_ 
