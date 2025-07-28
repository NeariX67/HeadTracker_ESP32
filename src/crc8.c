#include <stdint.h>
#include <stdio.h>
#include "crc8.h"

// CRC-8 parameters
#define CRC8_POLYNOMIAL 0x07
#define CRC8_INITIAL 0x00

// CRC8 lookup table
static uint8_t crc8_table[256];

// Generate CRC8 lookup table
void crc8_generate_table()
{
    for (int i = 0; i < 256; i++)
    {
        uint8_t crc = i;
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            }
            else
            {
                crc <<= 1;
            }
        }
        crc8_table[i] = crc;
    }
}

// Calculate CRC8
uint8_t crc8_calculate(uint8_t *data, size_t length)
{
    uint8_t crc = CRC8_INITIAL;
    for (size_t i = 0; i < length; i++)
    {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}
