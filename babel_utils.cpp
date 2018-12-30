#include "babel_utils.hpp"


int babel::deserialize_u32(uint8_t*in , uint32_t *out)
{
    *out = (in[0])    |
        (in[1] << 8)  |
        (in[2] << 16) | 
        (in[3] << 24); 
    return 4;
}

int babel::serialize_u32(uint8_t*out , uint32_t in)
{
    out[0] = in & 0xFF;
    out[1] = (in >> 8 ) & 0xFF;
    out[2] = (in >> 16) & 0xFF;
    out[3] = (in >> 24) & 0xFF;
    return 4;
}


int babel::deserialize_u16(uint8_t *in , uint16_t *out)
{
    *out = (in[0])    |
        (in[1] << 8); 

    return 2;
}

int babel::serialize_u16(uint8_t*out , uint16_t in)
{
    out[0] = in & 0xFF;
    out[1] = (in >> 8 ) & 0xFF;
    return 2;
}

uint16_t crc_advancer::compute_crc16_buffer(uint8_t* buffer, size_t len, uint16_t crc)
{
    for(size_t i = 0; i < len; ++i)
    {
        crc = compute_crc16(buffer[i], crc);
    }
    return crc;
}

uint16_t crc_advancer::compute_crc16(const uint8_t byte, const uint16_t orig_crc)
{
    //random generator i got from sunshine
    const uint16_t generator = 0x1021;	/* divisor is 16bit */
    uint16_t crc = orig_crc; /* CRC value is 16bit */

    crc ^= (((uint16_t)byte) << 8); /* move byte into MSB of 16bit CRC */

    for (size_t i = 0; i < 8; i++)
    {
        if ( (crc & 0x8000) != 0) /* test for MSB = bit 15 */
        {
            crc = ( (0xFFFF & ( crc << 1)) ^ generator);
        }
        else
        {
            crc <<= 1;
        }
    }

    return crc;
}


void crc_advancer::add_buffer(uint8_t* buffer, size_t len)
{
    m_crc = compute_crc16_buffer(buffer, len, m_crc);
}

void crc_advancer::add_byte(const uint8_t byte, uint16_t orig_crc)
{
    m_crc = compute_crc16(byte, orig_crc);
}
