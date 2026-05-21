#if !defined(FRAMEWORK_ARDUINO) && defined(USE_ELRS)
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "app_espnow.h"
#include "esp_crc.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "msp.h"
#include "msptypes.h"
#include "nvs_flash.h"

#include "trackersettings.h"
#include "defines.h"
#include "buzzer.h"
#include "crc8.h"
#include "led.h"

char *bytes_to_hex(const uint8_t *data, size_t len);
void processMspPacket(mspPacket_t *packet, const esp_now_recv_info_t *recv_info);
void printMem(int i);
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

#define ESPNOW_QUEUE_SIZE 1
#define ESPNOW_CHANNEL 0 // range 0 to 14

static const char *TAG = "elrs";

static TaskHandle_t Handle_elrs_task;
static QueueHandle_t espnow_re_queue;
static bool is_binding_mode = false;
static bool is_headtracking_enabled = true;
static bool binding_flag = false;
static bool is_send_failed = false;
static bool is_espnow_connected = false;
static uint16_t chanl_data[6];
esp_now_peer_info_t peerInfo;

static uint8_t bind_phrase[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t local_mac[ESP_NOW_ETH_ALEN] = {};

void set_binding_flag(bool flag)
{
    // ESP_LOGI(TAG, "set_binding_flag: %d", flag);
    binding_flag = flag;
}

bool isBinding(void)
{
    // ESP_LOGI(TAG, "isBinding: %d", is_binding_mode);
    return is_binding_mode;
}

bool isconnected()
{
    // ESP_LOGI(TAG, "isconnected: %d", is_espnow_connected);
    return is_espnow_connected || !is_headtracking_enabled;
}

void toggle_headtracking()
{
    is_headtracking_enabled = !is_headtracking_enabled;
}

static void wifi_init(void)
{
    ESP_LOGI(TAG, "wifi_init");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_LOGI(TAG, "BIND_PHRASE: " MACSTR "", MAC2STR(bind_phrase));
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, bind_phrase));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_start());
    // ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_ABOVE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80)); // 20dbm
                                                    // esp_wifi_disconnect();
#if ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
}

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    // ESP_LOGI(TAG, "espnow_send_cb: %d", status);
    if (status)
    {
        is_send_failed = true;
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    // ESP_LOGI(TAG, "espnow_recv_cb");

    char *data_hex = bytes_to_hex(data, len);
    // ESP_LOGI(TAG, "<<< %s", data_hex);
    free(data_hex);

    msp_t msp;
    msp_init(&msp);
    for (int i = 0; i < len; i++)
    {
        if (msp_processReceivedByte(&msp, data[i]))
        {
            mspPacket_t *packet = msp_getReceivedPacket(&msp);
            processMspPacket(packet, recv_info);
            msp_markPacketReceived(&msp);
            mspPacket_reset(packet);
        }
    }
}

void espnow_data_prepare(uint16_t chanl_roll, uint16_t chanl_tilt, uint16_t chanl_pan)
{
    // ESP_LOGI(TAG, "espnow_data_prepare");
    chanl_data[0] = map(chanl_roll, DEF_MIN_PWM, DEF_MAX_PWM, 192, 1792);
    chanl_data[1] = map(chanl_tilt, DEF_MIN_PWM, DEF_MAX_PWM, 192, 1792);
    chanl_data[2] = map(chanl_pan, DEF_MIN_PWM, DEF_MAX_PWM, 192, 1792);
}

#define ESPNOW_NVS_NAMESPACE "elrs"
#define ESPNOW_NVS_PEER_KEY "peer_mac"

static esp_err_t restore_bind_phrase_from_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_now_peer_info_t peer_info = {0};
    size_t peer_size = sizeof(peer_info);
    esp_err_t err = nvs_open(ESPNOW_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "No NVS namespace for bind phrase: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_get_blob(nvs_handle, ESPNOW_NVS_PEER_KEY, &peer_info, &peer_size);
    nvs_close(nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "No saved bind phrase in NVS: %s", esp_err_to_name(err));
        return err;
    }

    if (peer_size < ESP_NOW_ETH_ALEN)
    {
        ESP_LOGW(TAG, "Saved peer info is invalid (size=%u)", (unsigned)peer_size);
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(bind_phrase, peer_info.peer_addr, ESP_NOW_ETH_ALEN);
    ESP_LOGI(TAG, "Loaded bind phrase from NVS: " MACSTR "", MAC2STR(bind_phrase));
    return ESP_OK;
}

esp_err_t esp_now_save_peer(esp_now_peer_info_t *peer)
{
    ESP_LOGI(TAG, "esp_now_save_peer");
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(ESPNOW_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS");
        return err;
    }

    err = nvs_set_blob(nvs_handle, ESPNOW_NVS_PEER_KEY, peer, sizeof(esp_now_peer_info_t));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write peer info into NVS");
        nvs_close(nvs_handle);
        return err;
    }

    // Commit
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit peer info into NVS");
        nvs_close(nvs_handle);
        return err;
    }

    // Close
    nvs_close(nvs_handle);
    return ESP_OK;
}

esp_err_t esp_now_restore_peer(uint8_t peer_addr_out[ESP_NOW_ETH_ALEN])
{
    ESP_LOGI(TAG, "esp_now_restore_peer");
    nvs_handle_t nvs_handle;
    esp_now_peer_info_t peer_info = {0};
    size_t peer_size = sizeof(peer_info);
    esp_err_t ret = nvs_open(ESPNOW_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS");
        return ret;
    }

    // read peer from nvs.
    ret = nvs_get_blob(nvs_handle, ESPNOW_NVS_PEER_KEY, &peer_info, &peer_size);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read peer from NVS");
        nvs_close(nvs_handle);
        return ret;
    }

    if (peer_size < ESP_NOW_ETH_ALEN)
    {
        ESP_LOGE(TAG, "Invalid peer info size in NVS: %u", (unsigned)peer_size);
        nvs_close(nvs_handle);
        return ESP_ERR_INVALID_SIZE;
    }

    // recover peer information
    if (!esp_now_is_peer_exist(peer_info.peer_addr))
    {
        // add peer if not in ram yet.
        ret = esp_now_add_peer(&peer_info);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to add peer");
            nvs_close(nvs_handle);
            return ret;
        }

        ESP_LOGI(TAG, "Restored peer: " MACSTR "", MAC2STR(peer_info.peer_addr));
    }

    if (peer_addr_out != NULL)
    {
        memcpy(peer_addr_out, peer_info.peer_addr, ESP_NOW_ETH_ALEN);
    }

    nvs_close(nvs_handle);
    return ESP_OK;
}

#ifdef HEADTRACKER
static void espnow_send_task()
{

    // ESP_LOGI(TAG, "espnow_send_task");

    uint8_t peer_addr[ESP_NOW_ETH_ALEN] = {0};
    mspPacket_t frame;
    TickType_t xLastWakeTime;

    uint8_t packetSize = 15;
    uint8_t data[packetSize];
    uint8_t resultSize = 0;

    // TODO

    xLastWakeTime = xTaskGetTickCount();

    if (esp_now_restore_peer(peer_addr) != ESP_OK)
    {
        ESP_LOGE(TAG, "ESPNOW peer not found.");
        Handle_elrs_task = NULL;
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Peer Mac: " MACSTR "", MAC2STR(peer_addr));

    // ELRS Backpack: https://github.com/ExpressLRS/Backpack/blob/b5b7675da124fffb4b4399f4d83a6dfb79527f1c/src/module_base.cpp#L65
    for (;;)
    {
        // ESP_LOGI(TAG, "espnow_send_task tick");
        if (is_binding_mode)
        {
            ESP_LOGI(TAG, "In binding mode, skipping data send.");
            xTaskDelayUntil(&xLastWakeTime, ESPNOW_SEND_PERIOD * 20);
            continue;
        }
        if (!is_headtracking_enabled)
        {
            xTaskDelayUntil(&xLastWakeTime, ESPNOW_SEND_PERIOD * 20);
            led_set_status(disabled);
            continue;
        }

        mspPacket_reset(&frame);
        mspPacket_makeCommand(&frame);
        frame.function = MSP_ELRS_BACKPACK_SET_PTR;
        // MSP Channel order: Pan, Roll, Tilt
        // chanl_data order: Roll, Tilt, Pan

        // Pan
        mspPacket_addByte(&frame, chanl_data[2] & 0xFF);
        mspPacket_addByte(&frame, (chanl_data[2] >> 8) & 0xFF);
        // Roll
        mspPacket_addByte(&frame, chanl_data[0] & 0xFF);
        mspPacket_addByte(&frame, (chanl_data[0] >> 8) & 0xFF);
        // Tilt
        mspPacket_addByte(&frame, chanl_data[1] & 0xFF);
        mspPacket_addByte(&frame, (chanl_data[1] >> 8) & 0xFF);

        resultSize = msp_convertToByteArray(&frame, data);
        if (!resultSize)
        {
            ESP_LOGW(TAG, "msp_convertToByteArray failed");
            xTaskDelayUntil(&xLastWakeTime, ESPNOW_SEND_PERIOD * 20);
            continue;
        }

        // char *data_hex = bytes_to_hex(data, resultSize);
        // ESP_LOGI(TAG, ">>> %s", data_hex);
        // free(data_hex);

        // ESP_LOGI(TAG, "Roll: %d, Tilt: %d, Pan: %d", chanl_data[0], chanl_data[1], chanl_data[2]);

        esp_err_t err = esp_now_send(peer_addr, data, packetSize);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error sending the data: %s", esp_err_to_name(err));
            is_send_failed = true;
        }
        if (is_send_failed)
        {
            // If send failed, delay 20 times of the period to reduce power consumption.
            // ESP_LOGE(TAG, "Send failed.");
            if (is_espnow_connected)
            {
                ESP_LOGI(TAG, "ESP-NOW disconnected.");
            }
            is_send_failed = false;
            is_espnow_connected = false;
            led_set_status(disconnected);
            xTaskDelayUntil(&xLastWakeTime, ESPNOW_SEND_PERIOD * 20);
        }
        else
        {
            if (!is_espnow_connected)
            {
                ESP_LOGI(TAG, "ESP-NOW connected.");
            }
            is_espnow_connected = true;
            led_set_status(connected);
            xTaskDelayUntil(&xLastWakeTime, ESPNOW_SEND_PERIOD);
        }
    }
}
#endif

void set_binding_mode(bool true_or_false)
{
    ESP_LOGI(TAG, "set_binding_mode");
    if (true_or_false)
    {
        is_espnow_connected = false;
        led_set_status(binding);
    }

    is_binding_mode = true_or_false;
}

esp_err_t espnow_init(void)
{
    ESP_LOGI(TAG, "espnow_init");
    espnow_re_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_recv_cb_t));
    if (espnow_re_queue == NULL)
    {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    memcpy(peerInfo.peer_addr, bind_phrase, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
    // esp_now_del_peer(bind_phrase);

#if defined HEADTRACKER
    xTaskCreate(espnow_send_task, "espnow_send_task", ESPNOW_THREAD_STACK_SIZE_SET, NULL, ESPNOW_THREAD_PRIORITY_SET, &Handle_elrs_task);
#elif defined RX_SE
    xTaskCreate(espnow_rx_task, "espnow_rx_task", ESPNOW_THREAD_STACK_SIZE_SET, NULL, ESPNOW_THREAD_PRIORITY_SET, &Handle_elrs_task);
#endif
    return ESP_OK;
}

#ifdef HEADTRACKER
void ht_espnow_init(void)
{
    ESP_LOGI(TAG, "ht_espnow_init");
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGE(TAG, "nvs_flash_init failed.");
        // ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    restore_bind_phrase_from_nvs();

    // MAC address can only be set with unicast, so first byte must be even, not odd
    bind_phrase[0] &= ~0x01;

    esp_read_mac(local_mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "Local Mac: " MACSTR "", MAC2STR(local_mac));

    wifi_init();
    espnow_init();
}

void ht_espnow_deinit(void)
{
    ESP_LOGI(TAG, "ht_espnow_deinit");
    esp_now_unregister_recv_cb();
    esp_now_unregister_send_cb();

    if (Handle_elrs_task != NULL)
    {
        vTaskDelete(Handle_elrs_task);
        Handle_elrs_task = NULL;
    }

    if (espnow_re_queue != NULL)
    {
        vQueueDelete(espnow_re_queue);
    }

    // 4. deinit espnow
    esp_now_deinit();

    // 5. deinit wifi
    esp_wifi_stop();
    esp_wifi_deinit();
}
#endif

char *bytes_to_hex(const uint8_t *data, size_t len)
{
    static const char hex_digits[] = "0123456789ABCDEF";
    size_t out_len = len * 2 + 1;
    char *out = malloc(out_len);
    if (!out)
        return NULL;

    for (size_t i = 0; i < len; i++)
    {
        out[i * 2] = hex_digits[(data[i] >> 4) & 0x0F];
        out[i * 2 + 1] = hex_digits[data[i] & 0x0F];
    }

    out[len * 2] = '\0'; // Null-terminate
    return out;
}

void processMspPacket(mspPacket_t *packet, const esp_now_recv_info_t *recv_info)
{
    switch (packet->function)
    {
    case MSP_ELRS_BACKPACK_CRSF_TLM:
        // IGNORE, we dont need to do anything with the telemetry data coming from the backpack for now
        break;
    case MSP_ELRS_BACKPACK_SET_HEAD_TRACKING:
        ESP_LOGI(TAG, "Received MSP_ELRS_BACKPACK_SET_HEAD_TRACKING command");
        ESP_LOGI(TAG, "Payload size: %d", packet->payloadSize);
        is_headtracking_enabled = packet->payload[0] != 0;
        break;
    case MSP_ELRS_BIND:
        ESP_LOGI(TAG, "Received MSP_ELRS_BIND command");
        if (is_binding_mode)
        {
            bind_phrase[0] = recv_info->src_addr[0];
            bind_phrase[1] = recv_info->src_addr[1];
            bind_phrase[2] = recv_info->src_addr[2];
            bind_phrase[3] = recv_info->src_addr[3];
            bind_phrase[4] = recv_info->src_addr[4];
            bind_phrase[5] = recv_info->src_addr[5];
            ESP_LOGI(TAG, "Set bind phrase to: " MACSTR "", MAC2STR(bind_phrase));
            ESP_LOGI(TAG, "Binding successful.");
            set_binding_flag(true);
            set_binding_mode(false);

            memset(&peerInfo, 0, sizeof(peerInfo));
            memcpy(peerInfo.peer_addr, recv_info->src_addr, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;

            esp_now_save_peer(&peerInfo);
            esp_restart();
        }
        else
        {
            ESP_LOGW(TAG, "Not in binding mode, ignoring bind command.");
        }
        break;
    default:
        ESP_LOGW(TAG, "Received unsupported packet function: %d", packet->function);
        break;
    }
}

void printMem(int i)
{
    size_t free_heap = esp_get_free_heap_size();
    size_t min_free = esp_get_minimum_free_heap_size();

    ESP_LOGI(TAG, "Memory usage (%d): free=%u bytes, min_free=%u bytes",
             i, (unsigned)free_heap, (unsigned)min_free);
}

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif