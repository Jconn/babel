#ifndef BABEL_UTILS_H_
#define BABEL_UTILS_H_
#include <stdio.h>
#include <stdint.h>

uint16_t compute_crc16_buffer(uint8_t* buffer, size_t len);

int deserialize_u32(uint8_t*in , uint32_t *out);

int serialize_u32(uint8_t*out , uint32_t in);

int serialize_u16(uint8_t*out , uint16_t in);

int deserialize_u16(uint8_t *in , uint16_t *out);

#endif //BABEL_UTILS_H_
