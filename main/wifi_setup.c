#include "wifi_setup.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_http_server.h"

#define TAG "WIFI_SETUP"
#define AP_SSID "LEDVANCE_Setup"  // Name of the setup network

// Event group to handle connection states
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;
#define MAX_RETRY 3

static void event_handler(void* arg, esp_event_base_t event_base,
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
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// --- HTTP Server Handlers for Provisioning ---

// GET / handler: Serves the HTML form
static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char *html_form = 
        "<html><head><title>Wi-Fi Setup</title></head>"
        "<body><h1>Setup Wi-Fi</h1>"
        "<form action='/save' method='post'>"
        "<label>SSID:</label><br><input type='text' name='ssid'><br>"
        "<label>Password:</label><br><input type='password' name='pass'><br><br>"
        "<input type='submit' value='Save & Restart'>"
        "</form></body></html>";
    httpd_resp_send(req, html_form, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Helper to parse POST body
static esp_err_t get_post_field(char *buf, const char *field, char *dest, int dest_len) {
    char *start = strstr(buf, field);
    if (!start) return ESP_FAIL;
    start += strlen(field);
    char *end = strchr(start, '&');
    if (!end) end = buf + strlen(buf); // Handle last field
    int len = end - start;
    if (len >= dest_len) len = dest_len - 1;
    strncpy(dest, start, len);
    dest[len] = 0;
    
    // Simple URL decode (replace + with space)
    for(int i=0; i<len; i++) if(dest[i] == '+') dest[i] = ' ';
    return ESP_OK;
}

// POST /save handler: Saves creds and restarts
static esp_err_t save_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret = httpd_req_recv(req, buf, sizeof(buf));
    if (ret <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    char ssid[32] = {0};
    char pass[64] = {0};

    get_post_field(buf, "ssid=", ssid, sizeof(ssid));
    get_post_field(buf, "pass=", pass, sizeof(pass));

    ESP_LOGI(TAG, "Received SSID: %s", ssid);

    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));

    // Save to NVS
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

    httpd_resp_sendstr(req, "Saved. Restarting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

static void start_provisioning_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t save_uri = { .uri = "/save", .method = HTTP_POST, .handler = save_post_handler };
        httpd_register_uri_handler(server, &save_uri);
        
        ESP_LOGI(TAG, "Provisioning Web Server Started");
    }
}

static void start_softap_mode(void)
{
    esp_wifi_stop();
    esp_wifi_set_mode(WIFI_MODE_AP);
    
    wifi_config_t wifi_ap_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .channel = 1,
            .password = "", // Open network
            .max_connection = 4,
            .authmode = WIFI_AUTH_OPEN
        },
    };
    
    esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config);
    esp_wifi_start();
    
    ESP_LOGW(TAG, "Starting SoftAP: %s. Connect and go to 192.168.4.1", AP_SSID);
    start_provisioning_server();
}

esp_err_t wifi_setup_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    // Try to connect using saved credentials
    wifi_config_t wifi_cfg;
    if (esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg) == ESP_OK) {
        if (strlen((const char*)wifi_cfg.sta.ssid) > 0) {
            ESP_LOGI(TAG, "Found saved credentials for SSID: %s", wifi_cfg.sta.ssid);
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
            ESP_ERROR_CHECK(esp_wifi_start());

            // Wait for result
            EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                    WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                    pdFALSE,
                    pdFALSE,
                    portMAX_DELAY);

            if (bits & WIFI_CONNECTED_BIT) {
                return ESP_OK; // Success!
            }
        }
    }

    // If we get here: No credentials OR Connection failed
    start_softap_mode();
    return ESP_FAIL; // Indicates main app should not continue
}