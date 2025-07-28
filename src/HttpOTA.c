#include <string.h>

#include "ota.h"
#include "trackersettings.h"

#define WIFI_SSID "HeadTracker_OTA"
#define WIFI_PASS "123456789"
#define OTA_URL "ota.local"

static const char *TAG = "OTA";
static bool OTA_Mode_flag = false;

httpd_handle_t HttpOTA_httpd = NULL;

/* Maximum size of a single file */
#define MAX_FILE_SIZE (1024 * 1024) // 1024 KB
#define MAX_FILE_SIZE_STR "1024KB"
/* Temporary buffer size */
#define SCRATCH_BUFSIZE 1024
/* SHA-256 length */
#define HASH_LEN 32

bool get_OTA_Mode(void)
{
    return OTA_Mode_flag;
}

void set_OTA_Mode(bool true_or_false)
{
    OTA_Mode_flag = true_or_false;
}

// Create WiFi hotspot
void wifi_init_ap(void)
{
    esp_err_t ret;
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to create event loop 0x%x", ret);
        return;
    }

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASS,
            .max_connection = 1,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi AP Started: %s", WIFI_SSID);
}

static void print_sha256(const uint8_t *image_hash, const char *label)
{
    // char hash_print[HASH_LEN * 2 + 1];
    // hash_print[HASH_LEN * 2] = 0;
    // for (int i = 0; i < HASH_LEN; ++i) {
    //     sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    // }
    // ESP_LOGI(TAG, "%s: %s", label, hash_print);
}
// Set a GPIO high to determine if startup is successful, GPIO2 internal pull-down
#define CONFIG_EXAMPLE_GPIO_DIAGNOSTIC 2
static bool diagnostic(void)
{
    return true;
}
// Verify current firmware
void firmware_Sha256()
{
    uint8_t sha_256[HASH_LEN] = {0};
    esp_partition_t partition;

    // Get SHA256 digest of partition table
    partition.address = ESP_PARTITION_TABLE_OFFSET;
    partition.size = ESP_PARTITION_TABLE_MAX_LEN;
    partition.type = ESP_PARTITION_TYPE_DATA;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "Partition table SHA-256: ");

    // Get SHA256 digest for bootloader
    partition.address = ESP_BOOTLOADER_OFFSET;
    partition.size = ESP_PARTITION_TABLE_OFFSET;
    partition.type = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "Bootloader SHA-256: ");

    // Get SHA256 digest for running partition
    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    print_sha256(sha_256, "Current firmware SHA-256: ");

    const esp_partition_t *running_partition = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running_partition, &ota_state) == ESP_OK)
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
        { // This firmware first startup
            // Run diagnostic function...
            bool diagnostic_is_ok = diagnostic();
            if (diagnostic_is_ok)
            {
                ESP_LOGI(TAG, "Firmware diagnostic completed successfully! Continuing ...");
                esp_ota_mark_app_valid_cancel_rollback();
            }
            else
            {
                ESP_LOGE(TAG, "Firmware diagnostic failed! Starting rollback to previous version ...");
                esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
    }

    // const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    // if (configured != running) {
    //     ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
    //              configured->address, running->address);
    //     ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    // }
    // ESP_LOGI(TAG, "Running partition type %s subtype %#x (offset 0x%08x)", running->type?"DATA":"APP", running->subtype, running->address);

    esp_app_desc_t running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
    {
        ESP_LOGI(TAG, "Current firmware version: %s", running_app_info.version);
        ESP_LOGI(TAG, "Compile time  %s,%s", running_app_info.date, running_app_info.time);
    }
}

/* Handler for uploading files to server */
uint8_t Upload_Timeout_num;
static esp_err_t upload_post_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"); // Cross-origin transfer protocol
    esp_err_t err;
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = NULL;
    char SendStr[100];
    Upload_Timeout_num = 0;
    /* File cannot be larger than the limit */
    if (req->content_len > MAX_FILE_SIZE)
    {
        ESP_LOGE(TAG, "File too big : %d bytes", req->content_len);
        /* Respond with 400 Bad Request */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "File size must be less than" MAX_FILE_SIZE_STR "!");
        /* Return failure to close underlying connection else the incoming file content will jam the socket */
        return ESP_FAIL;
    }
    /* Content length of the request gives the size of the file being uploaded */
    int remaining = req->content_len;
    int received, L_remaining = remaining;
    bool image_header_was_checked = false; // Firmware header check flag
    char *OTA_buf = malloc(sizeof(char) * SCRATCH_BUFSIZE);
    while (remaining > 0)
    {
        /* Receive file part into buffer */
        if ((received = httpd_req_recv(req, OTA_buf, MIN(remaining, SCRATCH_BUFSIZE))) <= 0)
        {
            if (received == HTTPD_SOCK_ERR_TIMEOUT)
            {
                Upload_Timeout_num++;
                ESP_LOGE(TAG, "Receive overtime %d", Upload_Timeout_num);
                /* Retry if timeout occurs */
                if (Upload_Timeout_num >= 3)
                {
                    Upload_Timeout_num = 0;
                    ESP_LOGE(TAG, "Too many overtime!");
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "File receiving timeout!");
                    return ESP_FAIL;
                }
                continue;
            }
            /* In case of unrecoverable error, close and delete the unfinished file */
            free(OTA_buf);
            ESP_LOGE(TAG, "File receive filed");
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Unable to receive file!");
            if (update_handle)
                esp_ota_end(update_handle); // If OTA has begun, stop OTA
            return ESP_FAIL;
        }
        /* Firmware header verification */
        // Firmware header received
        if (image_header_was_checked == false)
        {
            esp_app_desc_t new_app_info; // Store new firmware header
            if (received > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
            {

                esp_app_desc_t running_app_info;
                const esp_partition_t *running = esp_ota_get_running_partition();
                if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
                {
                    ESP_LOGI(TAG, "Current firmware version: %s", running_app_info.version);
                    ESP_LOGI(TAG, "Comeplie time %s,%s", running_app_info.date, running_app_info.time);
                }
                // Check new firmware version by download
                memcpy(&new_app_info, &OTA_buf[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
#ifdef HEADTRACKER
                if (strstr(new_app_info.version, "HT_") == NULL) // Version error
#elif defined RECEIVER
                if (strstr(new_app_info.version, "RX_") == NULL) // Version error
#else
                if (strstr(new_app_info.version, "TEST_") == NULL) // Version error
#endif
                {
                    ESP_LOGE(TAG, "Firmware header error!");
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Firmware header error!");
                    return ESP_FAIL;
                }
                ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);
                ESP_LOGI(TAG, "New firmware complie time: %s, %s", new_app_info.date, new_app_info.time);

                // Return next OTA app partition which should be written with new firmware
                // esp_ota_get_next_update_partition automatically selects next available OTA partition
                update_partition = esp_ota_get_next_update_partition(NULL);
                if (update_partition == NULL)
                {
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA Partition error!");
                    return ESP_FAIL;
                }
                sprintf(SendStr, "To: OTA%d Ver: %s Time: %s, %s",
                        update_partition->subtype - ESP_PARTITION_SUBTYPE_APP_OTA_MIN,
                        new_app_info.version, new_app_info.date, new_app_info.time);

                // Begin OTA, OTA_SIZE_UNKNOWN will erase entire partition
                err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
                if (err != ESP_OK)
                {
                    char str[25];
                    sprintf(str, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                    ESP_LOGE(TAG, "%s", str);
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, str);
                    return ESP_FAIL;
                }
                ESP_LOGI(TAG, "esp_ota_begin succeeded");

                image_header_was_checked = true; // Firmware header verification complete, version comparison can be added manually
            }
            else
            {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "received package is not fit len!");
                return ESP_FAIL;
            }
        }
        /* Write firmware chunks to OTA partition */
        err = esp_ota_write(update_handle, (const void *)OTA_buf, received);
        if (err != ESP_OK)
        {
            char str[25];
            sprintf(str, "esp_ota_write failed (%s)", esp_err_to_name(err));
            ESP_LOGE(TAG, "%s", str);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, str);
            return ESP_FAIL;
        }

        /* Keep track of remaining size of the file left to be uploaded */
        remaining -= received;
    }
    free(OTA_buf);
    ESP_LOGI(TAG, "File receive complete: %dByte", L_remaining);

    err = esp_ota_end(update_handle);
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED)
        {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        }
        char str[25];
        sprintf(str, "esp_ota_end failed (%s)", esp_err_to_name(err));
        ESP_LOGE(TAG, "%s", str);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, str);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Firmware validation succeeded");

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        char str[50];
        sprintf(str, "esp_ota_set_boot_partition failed (%s)", esp_err_to_name(err));
        ESP_LOGE(TAG, "%s", str);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, str);
        return ESP_FAIL;
    }
    // httpd_resp_sendstr(req, "OTA successfully");
    httpd_resp_sendstr(req, SendStr);
    vTaskDelay(500 / portTICK_PERIOD_MS); // Delay to wait for message transmission
    ESP_LOGI(TAG, "Ready to reboot.");
    esp_restart();
    return ESP_OK;
}

// Configurator page
static esp_err_t Configurator_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"); // Cross-origin transfer protocol

    extern const unsigned char Configurator_html_gz_start[] asm("_binary_Configurator_html_gz_start");
    extern const unsigned char Configurator_html_gz_end[] asm("_binary_Configurator_html_gz_end");
    size_t Configurator_html_gz_len = Configurator_html_gz_end - Configurator_html_gz_start;

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    return httpd_resp_send(req, (const char *)Configurator_html_gz_start, Configurator_html_gz_len);
}

static esp_err_t getParams_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"); // Cross-origin transfer protocol

    cJSON *json_str = nvs_to_json();
    if (json_str == NULL)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to get NVS data");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    char *json_str_buf = cJSON_PrintUnformatted(json_str);
    esp_err_t ret = httpd_resp_send(req, json_str_buf, strlen(json_str_buf));
    cJSON_Delete(json_str);
    free(json_str_buf);
    return ret;
}

esp_err_t saveparams_handler(httpd_req_t *req)
{
    // 1. Read POST data
    int total_len = req->content_len;
    char *buf = malloc(total_len + 1);
    if (!buf)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No mem");
        return ESP_FAIL;
    }
    int received = 0, ret;
    while (received < total_len)
    {
        ret = httpd_req_recv(req, buf + received, total_len - received);
        if (ret <= 0)
        {
            free(buf);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Recv error");
            return ESP_FAIL;
        }
        received += ret;
    }
    buf[total_len] = 0;

    // 2. Parse JSON
    cJSON *json = cJSON_Parse(buf);
    free(buf);
    if (!json)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    // 3. Save to NVS
    json_to_nvs(json);
    cJSON_Delete(json);

    // 4. Return result
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true}");

    return ESP_OK;
}

esp_err_t resetToDefaults_handler(httpd_req_t *req)
{
    // Call function to restore default values
    trkset_restore_defaults();

    // Return JSON success response
    httpd_resp_set_type(req, "application/json");
    const char *response = "{\"success\":true}";
    return httpd_resp_send(req, response, strlen(response));
}

// OTA page
static esp_err_t HttpOTA_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"); // Cross-origin transfer protocol

    extern const unsigned char HttpOTA_html_gz_start[] asm("_binary_HttpOTA_html_gz_start");
    extern const unsigned char HttpOTA_html_gz_end[] asm("_binary_HttpOTA_html_gz_end");
    size_t HttpOTA_html_gz_len = HttpOTA_html_gz_end - HttpOTA_html_gz_start;

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    return httpd_resp_send(req, (const char *)HttpOTA_html_gz_start, HttpOTA_html_gz_len);
}

// Current firmware information
static esp_err_t Now_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"); // Cross-origin transfer protocol

    static char json_response[1024];

    esp_app_desc_t running_app_info;
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_get_partition_description(running, &running_app_info);

    char *p = json_response;
    *p++ = '{';
    p += sprintf(p, "\"OTAsubtype\":%d,", running->subtype - ESP_PARTITION_SUBTYPE_APP_OTA_MIN); // OTA partition
    p += sprintf(p, "\"address\":%lu,", running->address);                                       // Address
    p += sprintf(p, "\"version\":\"%s\",", running_app_info.version);                            // Version number
    p += sprintf(p, "\"date\":\"%s\",", running_app_info.date);                                  // Date
    p += sprintf(p, "\"time\":\"%s\"", running_app_info.time);                                   // Time
    *p++ = '}';
    *p++ = 0;

    httpd_resp_set_type(req, "application/json");                      // Set HTTP response type
    return httpd_resp_send(req, json_response, strlen(json_response)); // Send a complete HTTP response. Content is in json_response
}

void HttpOTA_server_init()
{
    wifi_init_ap(); // Create WiFi hotspot
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 1;
    config.backlog_conn = 1;
    config.lru_purge_enable = true;
    config.max_uri_handlers = 15;
    config.stack_size = 8192;
    // /* Use URI wildcard matching function to allow the same handler to respond to multiple different target URIs that match the wildcard scheme. */
    // config.uri_match_fn = httpd_uri_match_wildcard;

    config.server_port = 80;
    config.ctrl_port = 32779;

    ESP_LOGI(TAG, "Starting OTA server on port: '%d'", config.server_port);
    if (httpd_start(&HttpOTA_httpd, &config) == ESP_OK)
    {
        httpd_uri_t Configurator_uri = {// Configurator page
                                        .uri = "/",
                                        .method = HTTP_GET,
                                        .handler = Configurator_handler,
                                        .user_ctx = NULL};
        httpd_register_uri_handler(HttpOTA_httpd, &Configurator_uri);

        httpd_uri_t getParams_uri = {// parameter setting page
                                     .uri = "/api/getParams",
                                     .method = HTTP_GET,
                                     .handler = getParams_handler,
                                     .user_ctx = NULL};
        httpd_register_uri_handler(HttpOTA_httpd, &getParams_uri);

        httpd_uri_t saveparams_uri = {
            .uri = "/api/saveParams",
            .method = HTTP_POST,
            .handler = saveparams_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(HttpOTA_httpd, &saveparams_uri);

        httpd_uri_t resetToDefaults_uri = {
            .uri = "/api/resetToDefaults",
            .method = HTTP_POST,
            .handler = resetToDefaults_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(HttpOTA_httpd, &resetToDefaults_uri);

        httpd_uri_t HttpOTA_uri = {// OTA page
                                   .uri = "/OTA",
                                   .method = HTTP_GET,
                                   .handler = HttpOTA_handler,
                                   .user_ctx = NULL};
        httpd_register_uri_handler(HttpOTA_httpd, &HttpOTA_uri);

        httpd_uri_t Now_uri = {// Current firmware information
                               .uri = "/Now",
                               .method = HTTP_GET,
                               .handler = Now_handler,
                               .user_ctx = NULL};
        httpd_register_uri_handler(HttpOTA_httpd, &Now_uri);

        /* URI handler for uploading files to server */
        httpd_uri_t file_upload = {
            .uri = "/upload",
            .method = HTTP_POST,
            .handler = upload_post_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(HttpOTA_httpd, &file_upload);
    }
}
