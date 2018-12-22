#ifndef BABEL_UTILS_H_
#define BABEL_UTILS_H_
#include <stdio.h>
#include <stdint.h>

uint16_t compute_crc16_buffer(uint8_t* buffer, size_t len, uint16_t crc);

int deserialize_u32(uint8_t*in , uint32_t *out);

int serialize_u32(uint8_t*out , uint32_t in);

int serialize_u16(uint8_t*out , uint16_t in);

int deserialize_u16(uint8_t *in , uint16_t *out);

class crc_advancer {
public:
    uint16_t get_crc(void) { return m_crc; }
private:
    uint16_t m_crc = 0;
    void add_byte(const uint8_t byte, uint16_t orig_crc); 
    void add_buffer(uint8_t* buffer, size_t len);  

};
#endif //BABEL_UTILS_H_
