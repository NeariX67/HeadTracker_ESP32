#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // Forward declaration for ESP-IDF compatible interface
    typedef struct
    {
        int (*write)(const uint8_t *data, size_t len);
        int (*read)(uint8_t *data, size_t len);
        int (*available)(void);
    } msp_port_t;

// TODO: MSP_PORT_INBUF_SIZE should be changed to
// dynamically allocate array length based on the payload size
// Hardcoding payload size to 64 bytes for now, to allow enough space
// for custom OSD text.
#define MSP_PORT_INBUF_SIZE 64

#define CHECK_PACKET_PARSING() \
    if (packet->readError)     \
    {                          \
        return;                \
    }

    typedef enum
    {
        MSP_IDLE,
        MSP_HEADER_START,
        MSP_HEADER_X,

        MSP_HEADER_V2_NATIVE,
        MSP_PAYLOAD_V2_NATIVE,
        MSP_CHECKSUM_V2_NATIVE,

        MSP_COMMAND_RECEIVED
    } mspState_e;

    typedef enum
    {
        MSP_PACKET_UNKNOWN,
        MSP_PACKET_COMMAND,
        MSP_PACKET_RESPONSE
    } mspPacketType_e;

    typedef struct __attribute__((packed))
    {
        uint8_t flags;
        uint16_t function;
        uint16_t payloadSize;
    } mspHeaderV2_t;

    typedef struct
    {
        mspPacketType_e type;
        uint8_t flags;
        uint16_t function;
        uint16_t payloadSize;
        uint8_t payload[MSP_PORT_INBUF_SIZE];
        uint16_t payloadReadIterator;
        bool readError;
    } mspPacket_t;

    // C-style functions for mspPacket_t operations
    void mspPacket_reset(mspPacket_t *packet);
    void mspPacket_addByte(mspPacket_t *packet, uint8_t b);
    void mspPacket_makeResponse(mspPacket_t *packet);
    void mspPacket_makeCommand(mspPacket_t *packet);
    uint8_t mspPacket_readByte(mspPacket_t *packet);

    /////////////////////////////////////////////////

    // MSP structure to replace C++ class
    typedef struct
    {
        mspState_e inputState;
        uint16_t offset;
        uint8_t inputBuffer[MSP_PORT_INBUF_SIZE];
        mspPacket_t packet;
        uint8_t crc;
    } msp_t;

    // C-style function declarations
    void msp_init(msp_t *msp);
    bool msp_processReceivedByte(msp_t *msp, uint8_t c);
    mspPacket_t *msp_getReceivedPacket(msp_t *msp);
    void msp_markPacketReceived(msp_t *msp);
    bool msp_sendPacket(mspPacket_t *packet, msp_port_t *port);
    uint8_t msp_convertToByteArray(mspPacket_t *packet, uint8_t *byteArray);
    uint8_t msp_getTotalPacketSize(mspPacket_t *packet);
    bool msp_awaitPacket(msp_t *msp, mspPacket_t *packet, msp_port_t *port, uint32_t timeoutMillis);

#ifdef __cplusplus
}
#endif