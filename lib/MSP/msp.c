#include "msp.h"

#include "esp_log.h"
#include "esp_timer.h"

/* ==========================================
MSP V2 Message Structure:
Offset: Usage:         In CRC:  Comment:
======= ======         =======  ========
0       $                       Framing magic start char
1       X                       'X' in place of v1 'M'
2       type                    '<' / '>' / '!' Message Type (TODO find out what ! type is)
3       flag           +        uint8, flag, usage to be defined (set to zero)
4       function       +        uint16 (little endian). 0 - 255 is the same function as V1 for backwards compatibility
6       payload size   +        uint16 (little endian) payload size in bytes
8       payload        +        n (up to 65535 bytes) payload
n+8     checksum                uint8, (n= payload size), crc8_dvb_s2 checksum
========================================== */

static const char *TAG = "MSP";

// CRC8 lookup table for DVB-S2 polynomial (0xD5)
static const uint8_t crc8_dvb_s2_table[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

// ESP-IDF equivalent of Arduino's millis() function
static uint32_t millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// CRC helper function.
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t data)
{
    return crc8_dvb_s2_table[crc ^ data];
}

// mspPacket_t helper functions
void mspPacket_reset(mspPacket_t *packet)
{
    packet->type = MSP_PACKET_UNKNOWN;
    packet->flags = 0;
    packet->function = 0;
    packet->payloadSize = 0;
    packet->payloadReadIterator = 0;
    packet->readError = false;
}

void mspPacket_addByte(mspPacket_t *packet, uint8_t b)
{
    packet->payload[packet->payloadSize++] = b;
}

void mspPacket_makeResponse(mspPacket_t *packet)
{
    packet->type = MSP_PACKET_RESPONSE;
}

void mspPacket_makeCommand(mspPacket_t *packet)
{
    packet->type = MSP_PACKET_COMMAND;
}

uint8_t mspPacket_readByte(mspPacket_t *packet)
{
    if (packet->payloadReadIterator >= packet->payloadSize)
    {
        // We are trying to read beyond the length of the payload
        packet->readError = true;
        return 0;
    }

    return packet->payload[packet->payloadReadIterator++];
}

// MSP main functions
void msp_init(msp_t *msp)
{
    msp->inputState = MSP_IDLE;
    msp->offset = 0;
    msp->crc = 0;
    mspPacket_reset(&msp->packet);
}

bool msp_processReceivedByte(msp_t *msp, uint8_t c)
{
    switch (msp->inputState)
    {

    case MSP_IDLE:
        // Wait for framing char
        if (c == '$')
        {
            msp->inputState = MSP_HEADER_START;
        }
        break;

    case MSP_HEADER_START:
        // Waiting for 'X' (MSPv2 native)
        switch (c)
        {
        case 'X':
            msp->inputState = MSP_HEADER_X;
            break;
        default:
            msp->inputState = MSP_IDLE;
            break;
        }
        break;

    case MSP_HEADER_X:
        // Wait for the packet type (cmd or req)
        msp->inputState = MSP_HEADER_V2_NATIVE;

        // Start of a new packet
        // reset the packet, offset iterator, and CRC
        mspPacket_reset(&msp->packet);
        msp->offset = 0;
        msp->crc = 0;

        switch (c)
        {
        case '<':
            msp->packet.type = MSP_PACKET_COMMAND;
            break;
        case '>':
            msp->packet.type = MSP_PACKET_RESPONSE;
            break;
        default:
            msp->packet.type = MSP_PACKET_UNKNOWN;
            msp->inputState = MSP_IDLE;
            break;
        }
        break;

    case MSP_HEADER_V2_NATIVE:
        // Read bytes until we have a full header
        msp->inputBuffer[msp->offset++] = c;
        msp->crc = crc8_dvb_s2(msp->crc, c);

        // If we've received the correct amount of bytes for a full header
        if (msp->offset == sizeof(mspHeaderV2_t))
        {
            // Copy header values into packet
            mspHeaderV2_t *header = (mspHeaderV2_t *)&msp->inputBuffer[0];
            msp->packet.payloadSize = header->payloadSize;
            msp->packet.function = header->function;
            msp->packet.flags = header->flags;
            // reset the offset iterator for re-use in payload below
            msp->offset = 0;
            if (msp->packet.payloadSize == 0)
                msp->inputState = MSP_CHECKSUM_V2_NATIVE;
            else
                msp->inputState = MSP_PAYLOAD_V2_NATIVE;
        }
        break;

    case MSP_PAYLOAD_V2_NATIVE:
        // Read bytes until we reach payloadSize
        msp->packet.payload[msp->offset++] = c;
        msp->crc = crc8_dvb_s2(msp->crc, c);

        // If we've received the correct amount of bytes for payload
        if (msp->offset == msp->packet.payloadSize)
        {
            // Then we're up to the CRC
            msp->inputState = MSP_CHECKSUM_V2_NATIVE;
        }
        break;

    case MSP_CHECKSUM_V2_NATIVE:
        // Assert that the checksums match
        if (msp->crc == c)
        {
            msp->inputState = MSP_COMMAND_RECEIVED;
        }
        else
        {
            ESP_LOGI(TAG, "CRC failure on MSP packet - Got %d expected %d", c, msp->crc);
            msp->inputState = MSP_IDLE;
        }
        break;

    default:
        msp->inputState = MSP_IDLE;
        break;
    }

    // If we've successfully parsed a complete packet
    // return true so the calling function knows that
    // a new packet is ready.
    if (msp->inputState == MSP_COMMAND_RECEIVED)
    {
        return true;
    }
    return false;
}

mspPacket_t *msp_getReceivedPacket(msp_t *msp)
{
    return &msp->packet;
}

void msp_markPacketReceived(msp_t *msp)
{
    // Set input state to idle, ready to receive the next packet
    // The current packet data will be discarded internally
    msp->inputState = MSP_IDLE;
}

bool msp_sendPacket(mspPacket_t *packet, msp_port_t *port)
{
    // Sanity check the packet before sending
    if (packet->type != MSP_PACKET_COMMAND && packet->type != MSP_PACKET_RESPONSE)
    {
        // Unsupported packet type (note: ignoring '!' until we know what it is)
        return false;
    }

    if (packet->type == MSP_PACKET_RESPONSE && packet->payloadSize == 0)
    {
        // Response packet with no payload
        return false;
    }

    // Write out the framing chars
    uint8_t frameChars[2] = {'$', 'X'};
    port->write(frameChars, 2);

    // Write out the packet type
    uint8_t packetType;
    if (packet->type == MSP_PACKET_COMMAND)
    {
        packetType = '<';
    }
    else if (packet->type == MSP_PACKET_RESPONSE)
    {
        packetType = '>';
    }
    port->write(&packetType, 1);

    // Subsequent bytes are contained in the crc
    uint8_t crc = 0;

    // Pack header struct into buffer
    uint8_t headerBuffer[5];
    mspHeaderV2_t *header = (mspHeaderV2_t *)&headerBuffer[0];
    header->flags = packet->flags;
    header->function = packet->function;
    header->payloadSize = packet->payloadSize;

    // Write out the header buffer, adding each byte to the crc
    for (uint8_t i = 0; i < sizeof(mspHeaderV2_t); ++i)
    {
        port->write(&headerBuffer[i], 1);
        crc = crc8_dvb_s2(crc, headerBuffer[i]);
    }

    // Write out the payload, adding each byte to the crc
    for (uint16_t i = 0; i < packet->payloadSize; ++i)
    {
        port->write(&packet->payload[i], 1);
        crc = crc8_dvb_s2(crc, packet->payload[i]);
    }

    // Write out the crc
    port->write(&crc, 1);

    return true;
}

uint8_t msp_convertToByteArray(mspPacket_t *packet, uint8_t *byteArray)
{
    uint8_t bufferPos = 0;
    // Sanity check the packet before converting
    if (packet->type != MSP_PACKET_COMMAND && packet->type != MSP_PACKET_RESPONSE)
    {
        // Unsupported packet type (note: ignoring '!' until we know what it is)
        return 0;
    }

    if (packet->type == MSP_PACKET_RESPONSE && packet->payloadSize == 0)
    {
        // Response packet with no payload
        return 0;
    }

    // Write out the framing chars
    byteArray[bufferPos++] = '$';
    byteArray[bufferPos++] = 'X';

    ESP_LOGI(TAG, "bufferPos 1: %d", bufferPos);

    // Write out the packet type
    if (packet->type == MSP_PACKET_COMMAND)
    {
        byteArray[bufferPos++] = '<';
    }
    else if (packet->type == MSP_PACKET_RESPONSE)
    {
        byteArray[bufferPos++] = '>';
    }

    ESP_LOGI(TAG, "bufferPos 2: %d", bufferPos);
    // Subsequent bytes are contained in the crc
    uint8_t crc = 0;

    // Pack header struct into buffer
    uint8_t headerBuffer[5];
    mspHeaderV2_t *header = (mspHeaderV2_t *)&headerBuffer[0];
    header->flags = packet->flags;
    header->function = packet->function;
    header->payloadSize = packet->payloadSize;

    // Write out the header buffer, adding each byte to the crc
    for (uint8_t i = 0; i < sizeof(mspHeaderV2_t); ++i)
    {
        byteArray[bufferPos++] = headerBuffer[i];
        crc = crc8_dvb_s2(crc, headerBuffer[i]);
    }
    ESP_LOGI(TAG, "bufferPos 3: %d", bufferPos);

    // Write out the payload, adding each byte to the crc
    for (uint16_t i = 0; i < packet->payloadSize; ++i)
    {
        byteArray[bufferPos++] = packet->payload[i];
        crc = crc8_dvb_s2(crc, packet->payload[i]);
    }
    ESP_LOGI(TAG, "bufferPos 4: %d", bufferPos);

    // Write out the crc
    byteArray[bufferPos++] = crc;
    ESP_LOGI(TAG, "bufferPos 5: %d", bufferPos);

    return bufferPos;
}

uint8_t msp_getTotalPacketSize(mspPacket_t *packet)
{
    uint8_t totalSize = 0;

    // framing chars
    totalSize += sizeof('$');
    totalSize += sizeof('X');

    // packet type
    if (packet->type == MSP_PACKET_COMMAND)
    {
        totalSize += sizeof('<');
    }
    else if (packet->type == MSP_PACKET_RESPONSE)
    {
        totalSize += sizeof('>');
    }

    // header
    totalSize += sizeof(mspHeaderV2_t);

    // payload
    totalSize += packet->payloadSize;

    // crc
    totalSize++;

    return totalSize;
}

bool msp_awaitPacket(msp_t *msp, mspPacket_t *packet, msp_port_t *port, uint32_t timeoutMillis)
{
    uint32_t requestTime = millis();

    msp_sendPacket(packet, port);

    // wait up to <timeoutMillis> milliseconds for a response, then bail out
    while (millis() - requestTime < timeoutMillis)
    {
        while (port->available())
        {
            uint8_t data;
            if (port->read(&data, 1) > 0)
            {
                if (msp_processReceivedByte(msp, data))
                {
                    return true;
                }
            }
        }
        // Small delay to prevent busy waiting
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    ESP_LOGI(TAG, "msp_awaitPacket Exceeded timeout while waiting for packet");
    return false;
}