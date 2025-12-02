#include "wifi_setup.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_http_server.h"
#include "sdkconfig.h" // Required for CONFIG_ macros

#define TAG "WIFI_SETUP"
#define AP_SSID "LEDVANCE_Setup"

static EventGroupHandle_t s_setup_event_group;
#define SETUP_WIFI_CONNECTED_BIT BIT0
#define SETUP_FAIL_BIT           BIT2

static int s_retry_num = 0;
#define MAX_RETRY 3

static void setup_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_setup_event_group, SETUP_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        xEventGroupSetBits(s_setup_event_group, SETUP_WIFI_CONNECTED_BIT);
    }
}

// --- Helper Functions ---
static char from_hex(char ch) {
    return isdigit(ch) ? ch - '0' : tolower(ch) - 'a' + 10;
}

static esp_err_t get_post_field(char *buf, const char *field, char *dest, int dest_len) {
    char *start = strstr(buf, field);
    if (!start) return ESP_FAIL;
    start += strlen(field);
    char *end = strchr(start, '&');
    if (!end) end = buf + strlen(buf);
    int len = end - start;
    int w = 0; 
    for (int r = 0; r < len && w < dest_len - 1; r++) {
        if (start[r] == '+') {
            dest[w++] = ' ';
        } else if (start[r] == '%' && r + 2 < len) {
            char ch = (from_hex(start[r+1]) << 4) | from_hex(start[r+2]);
            dest[w++] = ch;
            r += 2;
        } else {
            dest[w++] = start[r];
        }
    }
    dest[w] = '\0';
    return ESP_OK;
}

// --- Connection Test ---
static esp_err_t test_wifi_logic(const char* ssid, const char* pass) {
    ESP_LOGI(TAG, "Testing Wi-Fi: %s", ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    s_retry_num = 0;
    xEventGroupClearBits(s_setup_event_group, 0xFF);
    esp_wifi_connect();

    EventBits_t bits = xEventGroupWaitBits(s_setup_event_group,
            SETUP_WIFI_CONNECTED_BIT | SETUP_FAIL_BIT,
            pdFALSE, pdFALSE, pdMS_TO_TICKS(10000));

    if (bits & SETUP_FAIL_BIT) return ESP_FAIL;
    if (!(bits & SETUP_WIFI_CONNECTED_BIT)) return ESP_ERR_TIMEOUT;
    return ESP_OK;
}

// --- HTTP Handlers ---
static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char *html_form = 
        "<html><head><title>Wi-Fi Setup</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
        "<style>body{font-family:sans-serif;padding:20px;max-width:500px;margin:0 auto;}"
        "input{width:100%;padding:10px;margin-bottom:10px;box-sizing:border-box;}"
        ".btn{width:100%;padding:10px;background:#4CAF50;color:white;border:none;cursor:pointer;}</style>"
        "</head><body><h1>Wi-Fi Setup</h1>"
        "<form action='/save' method='post'>"
        "<label>SSID:</label><input type='text' name='ssid' required>"
        "<label>Password:</label><input type='password' name='pass'>"
        "<input type='submit' value='Connect' class='btn'>"
        "</form></body></html>";
    httpd_resp_send(req, html_form, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t save_post_handler(httpd_req_t *req)
{
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf));
    if (ret <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    char ssid[32] = {0};
    char pass[64] = {0};
    get_post_field(buf, "ssid=", ssid, sizeof(ssid));
    get_post_field(buf, "pass=", pass, sizeof(pass));

    if (test_wifi_logic(ssid, pass) == ESP_OK) {
        wifi_config_t wifi_config = {0};
        strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
        strncpy((char*)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
        esp_wifi_set_storage(WIFI_STORAGE_FLASH);
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
        httpd_resp_sendstr(req, "<h1>Connected!</h1><p>Restarting...</p>");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    } else {
        httpd_resp_sendstr(req, "<h1>Failed</h1><p>Could not connect. Check credentials.</p><a href='/'>Back</a>");
    }
    return ESP_OK;
}

static void start_softap_mode(void)
{
    esp_wifi_stop();
    esp_wifi_set_mode(WIFI_MODE_AP);
    wifi_config_t wifi_ap_config = {
        .ap = {.ssid = AP_SSID, .ssid_len = strlen(AP_SSID), .channel = 1, .max_connection = 4, .authmode = WIFI_AUTH_OPEN},
    };
    esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config);
    esp_wifi_start();
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_register_uri_handler(server, &root);
        httpd_uri_t save = { .uri = "/save", .method = HTTP_POST, .handler = save_post_handler };
        httpd_register_uri_handler(server, &save);
    }
}

// --- INIT WITH BACKWARD COMPATIBILITY ---

esp_err_t wifi_setup_init(void)
{
    s_setup_event_group = xEventGroupCreate();
    if (esp_netif_init() != ESP_OK) {}
    if (esp_event_loop_create_default() != ESP_OK) {}
    if (!esp_netif_get_handle_from_ifkey("WIFI_STA_DEF")) esp_netif_create_default_wifi_sta();
    if (!esp_netif_get_handle_from_ifkey("WIFI_AP_DEF")) esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    esp_event_handler_instance_t i_any, i_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &setup_event_handler, NULL, &i_any);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &setup_event_handler, NULL, &i_ip);

    wifi_config_t wifi_cfg;
    bool has_config = false;

    // 1. Check if NVS has credentials
    if (esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg) == ESP_OK) {
        if (strlen((const char*)wifi_cfg.sta.ssid) > 0) {
            has_config = true;
            ESP_LOGI(TAG, "Found saved credentials in NVS: %s", wifi_cfg.sta.ssid);
        }
    }

    // 2. Backward Compatibility: Check SDKConfig if NVS is empty
    #ifdef CONFIG_ESP_WIFI_SSID
    if (!has_config) {
        const char *sdk_ssid = CONFIG_ESP_WIFI_SSID;
        const char *sdk_pass = CONFIG_ESP_WIFI_PASSWORD;
        // Only use if not the default placeholder
        if (sdk_ssid != NULL && strlen(sdk_ssid) > 0 && strcmp(sdk_ssid, "myssid") != 0) {
            ESP_LOGI(TAG, "NVS empty. Using SDKConfig credentials: %s", sdk_ssid);
            
            memset(&wifi_cfg, 0, sizeof(wifi_cfg));
            strncpy((char*)wifi_cfg.sta.ssid, sdk_ssid, sizeof(wifi_cfg.sta.ssid));
            if (sdk_pass) {
                strncpy((char*)wifi_cfg.sta.password, sdk_pass, sizeof(wifi_cfg.sta.password));
            }
            
            // This will also save it to NVS if successful connection happens, 
            // ensuring future boots are faster.
            esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
            has_config = true;
        }
    }
    #endif

    // 3. Try to Connect
    if (has_config) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
        
        // Wait for connection
        EventBits_t bits = xEventGroupWaitBits(s_setup_event_group, SETUP_WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(10000));
        
        if (bits & SETUP_WIFI_CONNECTED_BIT) {
            // Connection Success! Clean up setup handlers
            esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, i_any);
            esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, i_ip);
            return ESP_OK;
        } else {
            ESP_LOGW(TAG, "Failed to connect using saved/SDK credentials. Starting AP.");
        }
    }

    // 4. Fallback -> Start SoftAP
    start_softap_mode();
    return ESP_FAIL;
}