#ifdef FRAMEWORK_ARDUINO
#include <ESP8266WiFi.h>
#include "app_espnow.h"
#include <espnow.h>
#include <Ticker.h>
#include "receiver.h"
#include "ppm.h"
#include "trackersettings.h"
extern "C"
{
#include "crc8.h"
}

#define ESPNOW_QUEUE_SIZE 1
#define ESPNOW_CHANNEL 1 // range 0 to 14
#define ESPNOW_ENABLE_LONG_RANGE false

#define FRAMING_CHAR '$'
static const char *BIND_MSG_TX = "TXTXTX"; // size of payload is 6
#define ESP_NOW_ETH_ALEN 6

static bool is_binding_mode = false;
static uint16_t chanl_data[6];

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t source_mac[ESP_NOW_ETH_ALEN] = {};

static espnow_frame_t recv_cb_data;  // Received data from espnow.
static bool is_recv_cb_data = false; // Flag to indicate if received data is the latest.

static uint8_t msp_recv_buffer[MSP_RECV_BUFFER_SIZE];  // Raw received data for MSP parsing
static int recv_raw_len = 0;

static Ticker LED_blink_timer;

// MSP parsing state for Arduino
static msp_state_e msp_state = MSP_IDLE;
static msp_packet_t msp_packet;
static uint8_t msp_input_buffer[sizeof(msp_header_v2_t)];
static uint8_t msp_offset = 0;
static uint8_t msp_crc = 0;

// CRC8 DVB-S2 calculation for MSP
static uint8_t crc8_dvb_s2_byte(uint8_t crc, uint8_t a)
{
    crc ^= a;
    for (int i = 0; i < 8; ++i) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

// Process a single byte for MSP parsing (Arduino version)
static bool msp_process_byte(uint8_t c)
{
    switch (msp_state) {
        case MSP_IDLE:
            if (c == '$') {
                msp_state = MSP_HEADER_START;
            }
            break;

        case MSP_HEADER_START:
            if (c == 'X') {
                msp_state = MSP_HEADER_X;
            } else {
                msp_state = MSP_IDLE;
            }
            break;

        case MSP_HEADER_X:
            msp_state = MSP_HEADER_V2_NATIVE;
            memset(&msp_packet, 0, sizeof(msp_packet_t));
            msp_offset = 0;
            msp_crc = 0;

            if (c == '<') {
                msp_packet.type = MSP_PACKET_COMMAND;
            } else if (c == '>') {
                msp_packet.type = MSP_PACKET_RESPONSE;
            } else {
                msp_packet.type = MSP_PACKET_UNKNOWN;
                msp_state = MSP_IDLE;
            }
            break;

        case MSP_HEADER_V2_NATIVE:
            msp_input_buffer[msp_offset++] = c;
            msp_crc = crc8_dvb_s2_byte(msp_crc, c);

            if (msp_offset == sizeof(msp_header_v2_t)) {
                msp_header_v2_t* header = (msp_header_v2_t*)&msp_input_buffer[0];
                msp_packet.payloadSize = header->payloadSize;
                msp_packet.function = header->function;
                msp_packet.flags = header->flags;
                msp_offset = 0;
                
                if (msp_packet.payloadSize == 0) {
                    msp_state = MSP_CHECKSUM_V2_NATIVE;
                } else {
                    msp_state = MSP_PAYLOAD_V2_NATIVE;
                }
            }
            break;

        case MSP_PAYLOAD_V2_NATIVE:
            if (msp_offset < MSP_PORT_INBUF_SIZE) {
                msp_packet.payload[msp_offset++] = c;
                msp_crc = crc8_dvb_s2_byte(msp_crc, c);
                
                if (msp_offset == msp_packet.payloadSize) {
                    msp_state = MSP_CHECKSUM_V2_NATIVE;
                }
            } else {
                // Payload overflow - abort parsing
                Serial.println("MSP payload overflow, resetting parser");
                msp_state = MSP_IDLE;
            }
            break;

        case MSP_CHECKSUM_V2_NATIVE:
            if (msp_crc == c) {
                msp_state = MSP_COMMAND_RECEIVED;
            } else {
                char msg[64];
                sprintf(msg, "MSP CRC failure - Got 0x%02X expected 0x%02X", c, msp_crc);
                Serial.println(msg);
                msp_state = MSP_IDLE;
            }
            break;
        
        default:
            msp_state = MSP_IDLE;
            break;
    }

    return (msp_state == MSP_COMMAND_RECEIVED);
}

// Parse MSP data from buffer (Arduino version)
static bool msp_parse_buffer(const uint8_t *data, int len, msp_packet_t *packet)
{
    if (len < 0) {
        return false;  // Invalid length
    }
    
    msp_state = MSP_IDLE;
    for (size_t byte_index = 0; byte_index < (size_t)len; byte_index++) {
        if (msp_process_byte(data[byte_index])) {
            // Copy the parsed packet
            memcpy(packet, &msp_packet, sizeof(msp_packet_t));
            msp_state = MSP_IDLE; // Reset for next packet
            return true;
        }
    }
    return false;
}

bool isBinding(void)
{
    return is_binding_mode;
}

static void espnow_send_cb(u8 *mac_addr, u8 status)
{
}

static void espnow_recv_cb(u8 *mac_addr, u8 *data, u8 len)
{
    if (is_recv_cb_data)
    {
        Serial.println("Last received data not processed yet.");
        return;
    }

    // Only copy mac address in binding mode.
    if (isBinding())
    {
        memcpy(source_mac, mac_addr, ESP_NOW_ETH_ALEN);
    }

    // Store raw data for both legacy and MSP parsing
    if (len <= MSP_RECV_BUFFER_SIZE) {
        memcpy(msp_recv_buffer, data, len);
        recv_raw_len = len;
        
        // Also try to copy to legacy format if it matches
        if (len == sizeof(espnow_frame_t)) {
            memcpy(&recv_cb_data, data, sizeof(espnow_frame_t));
        }
        
        is_recv_cb_data = true;
    } else {
        Serial.println("ESPNOW receive data too long.");
    }
}

// Calculate the crc of the espnow frame.
static inline uint8_t espnow_crc(espnow_frame_t *pvFrame)
{
    // initial number: 0
    // calculate all data in frame except crc byte itself.
    return crc8_calculate((uint8_t *)pvFrame, sizeof(espnow_frame_t) - 1);
}

// Unpair all peers.
static void espnow_unpairAll(void)
{
    uint8_t *peer;
    peer = esp_now_fetch_peer(true); // fetch the first peer.
    while (peer)
    {
        esp_now_del_peer(peer);
        peer = esp_now_fetch_peer(false); // fetch the rest of peers.
    }
}

static void espnow_bind_task()
{
    espnow_frame_t frame;
    bool success_flag = false;

    // Blink led to indicate binding mode.
    LED_blink_timer.attach_ms(200, []()
                              { digitalWrite(GPIO_LED_STATUS, !digitalRead(GPIO_LED_STATUS)); });

    // Add broadcast peer information to peer list if in binding mode.
    esp_now_add_peer(broadcast_mac, ESP_NOW_ROLE_COMBO, ESPNOW_CHANNEL, NULL, 0);

    // esp_now_del_peer(broadcast_mac);

    // Fill binding message.
    frame.framing_char = FRAMING_CHAR;
    frame.function = ESPNOW_FUNCTION_BIND;
    memcpy(frame.payload, BIND_MSG_TX, sizeof(frame.payload));
    frame.crc_8 = espnow_crc(&frame);

    Serial.println("Start binding.");

    for (;;)
    {
        // Send binding message.
        esp_now_send(broadcast_mac, (uint8_t *)&frame, sizeof(espnow_frame_t));
        // Waiting for the other device's binding message.
        delay(100);
        // Do not bind again after success.
        if (is_recv_cb_data && !success_flag)
        {
            bool legacy_bind_detected = false;
            bool msp_bind_detected = false;
            msp_packet_t msp_pkt;
            
            // Try to parse as MSP packet (ELRS Backpack)
            if (msp_parse_buffer(msp_recv_buffer, recv_raw_len, &msp_pkt))
            {
                if (msp_pkt.function == MSP_ELRS_BIND && msp_pkt.payloadSize == 6)
                {
                    Serial.println("Received MSP_ELRS_BIND packet");
                    msp_bind_detected = true;
                }
            }
            
            // Try legacy binding protocol
            if (!msp_bind_detected && recv_raw_len == sizeof(espnow_frame_t))
            {
                // Check crc for legacy protocol
                if (espnow_crc(&recv_cb_data) == recv_cb_data.crc_8)
                {
                    // if the binding message matches, it's legacy bind
                    if (!memcmp(&recv_cb_data, &frame, sizeof(espnow_frame_t)))
                    {
                        legacy_bind_detected = true;
                        Serial.println("Received legacy binding packet");
                    }
                }
            }
            
            if (msp_bind_detected || legacy_bind_detected)
            {
                // Unpair all unicast peers first, make sure only one unicast exist at the same time.
                espnow_unpairAll();
                
                // Add peer to the list.
                uint8_t peer_addr[ESP_NOW_ETH_ALEN];
                
                if (msp_bind_detected)
                {
                    // For ELRS Backpack, the MAC address comes from the MSP payload
                    memcpy(peer_addr, msp_pkt.payload, ESP_NOW_ETH_ALEN);
                    char mac_str[24];
                    sprintf(mac_str, "ELRS Bind MAC from payload: %02X:%02X:%02X:%02X:%02X:%02X",
                            peer_addr[0], peer_addr[1], peer_addr[2], 
                            peer_addr[3], peer_addr[4], peer_addr[5]);
                    Serial.println(mac_str);
                }
                else
                {
                    // For legacy binding, MAC address comes from the sender
                    memcpy(peer_addr, source_mac, ESP_NOW_ETH_ALEN);
                }
                
                esp_now_add_peer(peer_addr, ESP_NOW_ROLE_COMBO, ESPNOW_CHANNEL, NULL, 0);
                success_flag = true;
                Serial.println("Bind success.");
                // save peer info into nvs.
            }
            else
            {
                Serial.println("Binding message validation failed.");
            }
            
            // Always reset flag to allow processing next packet
            is_recv_cb_data = false;
        }
        if (success_flag)
        {
            is_binding_mode = false;
            Serial.println("End binding, start sending channal data.");
            // Stop blinking led.
            LED_blink_timer.detach();
            digitalWrite(GPIO_LED_STATUS, 0);
            break;
        }
    }
}

void set_binding_mode(bool true_or_false)
{
    is_binding_mode = true_or_false;
}

void rx_espnow_init(void)
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Init ESP-NOW
    if (esp_now_init() != 0)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    else
    {
        Serial.println("esp_now init success");
    }

    // Set ESP-NOW Role
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

    // Once ESPNow is successfully Init, we will register for Send CB
    esp_now_register_send_cb(espnow_send_cb);
    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(espnow_recv_cb);
}

void rx_espnow_loop(void)
{
    if (isBinding())
    {
        espnow_bind_task();
    }
    else
    {
        // Prase channel data and send to ppm.
        if (is_recv_cb_data)
        {
            // Check crc.
            if (espnow_crc(&recv_cb_data) != recv_cb_data.crc_8)
            {
                Serial.println("Binding message crc incorrect.");
            }
            else
            {
                if (recv_cb_data.function == ESPNOW_FUNCTION_GET_DATA)
                {
                    // Prase channel data.
                    uint16_t chanl_roll = (recv_cb_data.payload[1] << 8) | recv_cb_data.payload[0];
                    uint16_t chanl_till = (recv_cb_data.payload[3] << 8) | recv_cb_data.payload[2];
                    uint16_t chanl_pan = (recv_cb_data.payload[5] << 8) | recv_cb_data.payload[4];
                    // Send channel data to ppm.
                    PpmOut_setChannel(getRollChl(), chanl_roll);
                    PpmOut_setChannel(getTiltChl(), chanl_till);
                    PpmOut_setChannel(getPanChl(), chanl_pan);
                    Serial.printf("%d,%d,%d\n", PpmOut_getChannel(getTiltChl()), PpmOut_getChannel(getRollChl()), PpmOut_getChannel(getPanChl()));
                }
            }
            is_recv_cb_data = false;
        }
    }
}

#endif