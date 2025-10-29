#if !defined(FRAMEWORK_ARDUINO) && defined(USE_ELRS)
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "app_espnow.h"

#include "trackersettings.h"
#include "defines.h"
#include "buzzer.h"
#include "crc8.h"
#include "led.h"

#define ESPNOW_QUEUE_SIZE 1
#define ESPNOW_CHANNEL 1 // range 0 to 14

static const char *TAG = "elrs";

static TaskHandle_t Handle_elrs_task;
static QueueHandle_t espnow_re_queue;
static bool is_binding_mode = false;
static bool binding_flag = false;
static bool is_send_failed = false;
static bool is_espnow_connected = false;
static uint16_t chanl_data[6];

static uint8_t bind_phrase[] = {0xB4, 0xDE, 0x4E, 0x14, 0x8B, 0x6F};

static uint8_t local_mac[ESP_NOW_ETH_ALEN] = {};

void set_binding_flag(bool flag)
{
    ESP_LOGI(TAG, "set_binding_flag");
    binding_flag = flag;
}

bool isBinding(void)
{
    // ESP_LOGI(TAG, "isBinding");
    return is_binding_mode;
}

bool isconnected()
{
    // ESP_LOGI(TAG, "isconnected");
    return is_espnow_connected;
}

static void wifi_init(void)
{
    ESP_LOGI(TAG, "wifi_init");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80)); // 20dbm

#if ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
}

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    ESP_LOGI(TAG, "espnow_send_cb");
    if (status)
    {
        is_send_failed = true;
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    ESP_LOGI(TAG, "espnow_recv_cb");
    // TODO
}

void espnow_data_prepare(uint16_t chanl_roll, uint16_t chanl_till, uint16_t chanl_pan)
{
    // ESP_LOGI(TAG, "espnow_data_prepare");
    chanl_data[0] = chanl_roll;
    chanl_data[1] = chanl_till;
    chanl_data[2] = chanl_pan;
}

#define ESPNOW_NVS_NAMESPACE "elrs"
#define ESPNOW_NVS_PEER_KEY "peer_mac"

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
        return err;
    }

    // Commit
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "Successfully write peer info into NVS");
        return err;
    }

    // Close
    nvs_close(nvs_handle);
    return ESP_OK;
}

uint8_t *esp_now_restore_peer(void)
{
    ESP_LOGI(TAG, "esp_now_restore_peer");
    uint8_t *addr_ret;
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(ESPNOW_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS");
        return NULL;
    }

    esp_now_peer_info_t *peer_info;
    size_t peer_size = sizeof(esp_now_peer_info_t);
    peer_info = malloc(sizeof(esp_now_peer_info_t));
    // read peer from nvs.
    ret = nvs_get_blob(nvs_handle, ESPNOW_NVS_PEER_KEY, peer_info, &peer_size);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read peer from NVS");
    }
    else
    {
        // recover peer information
        if (!esp_now_is_peer_exist(peer_info->peer_addr))
        {
            // add peer if not in ram yet.
            ret = esp_now_add_peer(peer_info);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to add peer");
            }
            else
            {
                ESP_LOGI(TAG, "Restored peer: " MACSTR "", MAC2STR(peer_info->peer_addr));
            }
        }
    }

    // copy the mac address to be return.
    addr_ret = malloc(ESP_NOW_ETH_ALEN);
    if (addr_ret != NULL)
    {
        memcpy(addr_ret, peer_info->peer_addr, ESP_NOW_ETH_ALEN);
    }

    free(peer_info);
    nvs_close(nvs_handle);

    return addr_ret;
}

#ifdef HEADTRACKER
static void espnow_send_task()
{
    ESP_LOGI(TAG, "espnow_send_task");

    // TODO

    TickType_t xLastWakeTime;

    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        xTaskDelayUntil(&xLastWakeTime, ESPNOW_SEND_PERIOD * 20);
    }
}
#endif

static void espnow_bind_task()
{
    ESP_LOGI(TAG, "espnow_bind_task");
    // TODO
}

void set_binding_mode(bool true_or_false)
{
    ESP_LOGI(TAG, "set_binding_mode");
    // Only create task once if already in binding mode.
    if (true_or_false && !is_binding_mode)
    {
        is_binding_mode = true_or_false;
        is_espnow_connected = false;
        xTaskCreate(espnow_bind_task, "espnow_bind_task", ESPNOW_THREAD_STACK_SIZE_SET, NULL, ESPNOW_THREAD_PRIORITY_SET, NULL);
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

#endif