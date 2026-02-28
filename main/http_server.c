#include "http_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "cJSON.h"
#include "main.h"
#include "lamp_nvs.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "nvs.h"

#define TAG "HTTP_SERVER"

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
        if (start[r] == '+') dest[w++] = ' ';
        else if (start[r] == '%' && r + 2 < len) {
            dest[w++] = (from_hex(start[r+1]) << 4) | from_hex(start[r+2]);
            r += 2;
        } else dest[w++] = start[r];
    }
    dest[w] = '\0';
    return ESP_OK;
}

// --- MQTT Test Logic ---
static EventGroupHandle_t s_mqtt_test_group;
#define MQTT_TEST_CONNECTED_BIT BIT0

static void mqtt_test_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    if (event_id == MQTT_EVENT_CONNECTED) {
        xEventGroupSetBits(s_mqtt_test_group, MQTT_TEST_CONNECTED_BIT);
    }
}

static esp_err_t perform_mqtt_test(const char *url, const char *user, const char *pass) {
    s_mqtt_test_group = xEventGroupCreate();
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = url,
        .credentials.username = user,
        .credentials.authentication.password = pass,
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    if (!client) return ESP_FAIL;

    esp_mqtt_client_register_event(client, MQTT_EVENT_CONNECTED, mqtt_test_handler, NULL);
    esp_mqtt_client_start(client);

    EventBits_t bits = xEventGroupWaitBits(s_mqtt_test_group, MQTT_TEST_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(5000));
    
    esp_mqtt_client_stop(client);
    esp_mqtt_client_destroy(client);
    vEventGroupDelete(s_mqtt_test_group);

    return (bits & MQTT_TEST_CONNECTED_BIT) ? ESP_OK : ESP_FAIL;
}

// --- Handlers ---

// CONFIG PAGE
static esp_err_t get_config_handler(httpd_req_t *req) {
    char url[128] = {0}, user[64] = {0}, pass[64] = {0};
    nvs_handle_t h;
    if (nvs_open("mqtt_config", NVS_READONLY, &h) == ESP_OK) {
        size_t len = sizeof(url); nvs_get_str(h, "broker_url", url, &len);
        len = sizeof(user); nvs_get_str(h, "username", user, &len);
        len = sizeof(pass); nvs_get_str(h, "password", pass, &len);
        nvs_close(h);
    }

    // Use a single large buffer to prevent "empty chunk" errors
    char page_buf[2048]; 
    
    snprintf(page_buf, sizeof(page_buf),
        "<html><head><title>Configuration</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
        "<style>body{font-family:sans-serif;padding:20px;max-width:600px;margin:0 auto;}"
        "input{width:100%%;padding:8px;margin-bottom:10px;box-sizing:border-box;}"
        ".btn{padding:10px;width:100%%;cursor:pointer;margin-bottom:10px;}"
        ".save{background:#4CAF50;color:white;border:none;}"
        ".test{background:#2196F3;color:white;border:none;}"
        "</style>"
        "<script>"
        "function test() {"
        "  var b = document.getElementById('testBtn'); b.innerText='Testing...'; b.disabled=true;"
        "  var data = new URLSearchParams(new FormData(document.getElementById('cfg')));"
        "  fetch('/test_mqtt', { method: 'POST', body: data }).then(r=>r.text()).then(t=>{"
        "    alert(t); b.innerText='Test Connection'; b.disabled=false;"
        "  });"
        "}"
        "</script>"
        "</head><body><h1>System Configuration</h1>"
        "<form id='cfg' action='/save_config' method='post'>"
        "<h3>MQTT Settings</h3>"
        "<label>Broker URL:</label><input type='text' name='url' value='%s' placeholder='mqtt://192.168.1.10:1883' required>"
        "<label>Username:</label><input type='text' name='user' value='%s'>"
        "<label>Password:</label><input type='password' name='pass' value='%s'>"
        "<button type='button' id='testBtn' class='btn test' onclick='test()'>Test Connection</button>"
        "<input type='submit' value='Save & Restart' class='btn save'>"
        "</form><a href='/'>Back to Overview</a></body></html>",
        url, user, pass
    );

    httpd_resp_send(req, page_buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// TEST MQTT POST
static esp_err_t test_mqtt_post_handler(httpd_req_t *req) {
    char buf[512];
    int ret = httpd_req_recv(req, buf, sizeof(buf));
    if (ret <= 0) return ESP_FAIL;
    buf[ret] = '\0';
    char url[128]={0}, user[64]={0}, pass[64]={0};
    get_post_field(buf, "url=", url, sizeof(url));
    get_post_field(buf, "user=", user, sizeof(user));
    get_post_field(buf, "pass=", pass, sizeof(pass));

    if (perform_mqtt_test(url, user, pass) == ESP_OK) httpd_resp_sendstr(req, "Connection Successful!");
    else httpd_resp_sendstr(req, "Connection Failed!");
    return ESP_OK;
}

// SAVE CONFIG POST
static esp_err_t save_config_post_handler(httpd_req_t *req) {
    char buf[512];
    int ret = httpd_req_recv(req, buf, sizeof(buf));
    if (ret <= 0) return ESP_FAIL;
    buf[ret] = '\0';
    char url[128]={0}, user[64]={0}, pass[64]={0};
    get_post_field(buf, "url=", url, sizeof(url));
    get_post_field(buf, "user=", user, sizeof(user));
    get_post_field(buf, "pass=", pass, sizeof(pass));

    nvs_handle_t h;
    if (nvs_open("mqtt_config", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, "broker_url", url);
        nvs_set_str(h, "username", user);
        nvs_set_str(h, "password", pass);
        nvs_commit(h);
        nvs_close(h);
    }
    httpd_resp_sendstr(req, "Saved. Restarting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

// --- Lamp Handlers ---
static esp_err_t get_lamps_overview_handler(httpd_req_t *req) {
    httpd_resp_sendstr_chunk(req,
        "<html><head><title>Gateway</title>"
        "<style>body{font-family:sans-serif;padding:20px;max-width:800px;margin:0 auto;}"
        "table{width:100%;border-collapse:collapse;margin-top:20px;}"
        "th,td{border:1px solid #ddd;padding:8px;text-align:left;}"
        "th{background:#4CAF50;color:white;}"
        ".btn{padding:5px 10px;text-decoration:none;border-radius:4px;color:white;display:inline-block;}"
        ".del{background:#f44336;} .edit{background:#2196F3;} .cfg{background:#FF9800;margin-bottom:10px;}"
        "</style></head><body>"
        "<h1>Lamp Overview</h1>"
        "<a href='/config' class='btn cfg'>System Configuration</a>"
        "<table><tr><th>Name</th><th>Address</th><th>Type</th><th>Scale</th><th>Actions</th></tr>");

    int count;
    const LampInfo* lamps = get_all_lamps(&count);
    for (int i = 0; i < count; i++) {
        char row[512];
        snprintf(row, sizeof(row), "<tr><td>%s</td><td>%s</td><td>%s</td><td>%d</td><td>"
            "<form action='/remove_lamp' method='post' style='display:inline;'><input type='hidden' name='lamp_name' value='%s'><input type='submit' value='Remove' class='btn del'></form> "
            "<form action='/edit_lamp' method='get' style='display:inline;'><input type='hidden' name='lamp_name' value='%s'><input type='submit' value='Edit' class='btn edit'></form>"
            "</td></tr>",
            lamps[i].name, lamps[i].address, lamps[i].supports_color?"Color":"White", lamps[i].brightness_scaling, lamps[i].name, lamps[i].name);
        httpd_resp_sendstr_chunk(req, row);
    }
    httpd_resp_sendstr_chunk(req, "</table><h2>Add Lamp</h2>"
        "<form action='/add_lamp' method='post'>"
        "Name: <input type='text' name='lamp_name' required> "
        "Addr: <input type='text' name='lamp_address' required> "
        "Scale: <input type='number' name='lamp_scaling' value='100' style='width:60px'> "
        "Color: <input type='checkbox' name='lamp_color' value='1'> "
        "<input type='submit' value='Add' class='btn edit'></form></body></html>");
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t add_lamp_post_handler(httpd_req_t *req) {
    char buf[512]; httpd_req_recv(req, buf, sizeof(buf));
    LampInfo l = {0}; char col[4]={0}, scl[16]={0};
    get_post_field(buf, "lamp_name=", l.name, sizeof(l.name));
    get_post_field(buf, "lamp_address=", l.address, sizeof(l.address));
    get_post_field(buf, "lamp_color=", col, sizeof(col)); l.supports_color = (col[0]=='1');
    get_post_field(buf, "lamp_scaling=", scl, sizeof(scl)); l.brightness_scaling = atoi(scl)?atoi(scl):100;
    add_lamp_info(&l);
    httpd_resp_set_status(req, "303 See Other"); httpd_resp_set_hdr(req, "Location", "/"); httpd_resp_send(req,NULL,0);
    refresh_mqtt_subscriptions(); publish_ha_discovery_messages();
    return ESP_OK;
}
static esp_err_t remove_lamp_post_handler(httpd_req_t *req) {
    char buf[128]; httpd_req_recv(req, buf, sizeof(buf));
    char name[32]; get_post_field(buf, "lamp_name=", name, sizeof(name));
    remove_lamp_info_by_name(name);
    httpd_resp_set_status(req, "303 See Other"); httpd_resp_set_hdr(req, "Location", "/"); httpd_resp_send(req,NULL,0);
    refresh_mqtt_subscriptions(); publish_ha_discovery_messages();
    return ESP_OK;
}
static esp_err_t edit_lamp_get_handler(httpd_req_t *req) {
    char q[128], name[32]; httpd_req_get_url_query_str(req, q, sizeof(q)); httpd_query_key_value(q, "lamp_name", name, sizeof(name));
    LampInfo l; find_lamp_by_name(name, &l);
    char buf[1024];
    snprintf(buf, sizeof(buf), "<html><body><h1>Edit %s</h1><form action='/update_lamp' method='post'>"
        "<input type='hidden' name='original_name' value='%s'>"
        "Name: <input type='text' name='lamp_name' value='%s'><br>"
        "Addr: <input type='text' name='lamp_address' value='%s'><br>"
        "Scale: <input type='number' name='lamp_scaling' value='%d'><br>"
        "Color: <input type='checkbox' name='lamp_color' value='1' %s><br>"
        "<input type='submit' value='Update'></form></body></html>",
        l.name, l.name, l.name, l.address, l.brightness_scaling, l.supports_color?"checked":"");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
static esp_err_t update_lamp_post_handler(httpd_req_t *req) {
    char buf[512]; httpd_req_recv(req, buf, sizeof(buf));
    char orig[32]; LampInfo l={0}; char col[4]={0}, scl[16]={0};
    get_post_field(buf, "original_name=", orig, sizeof(orig));
    get_post_field(buf, "lamp_name=", l.name, sizeof(l.name));
    get_post_field(buf, "lamp_address=", l.address, sizeof(l.address));
    get_post_field(buf, "lamp_color=", col, sizeof(col)); l.supports_color = (col[0]=='1');
    get_post_field(buf, "lamp_scaling=", scl, sizeof(scl)); l.brightness_scaling = atoi(scl)?atoi(scl):100;
    update_lamp_info(orig, &l);
    httpd_resp_set_status(req, "303 See Other"); httpd_resp_set_hdr(req, "Location", "/"); httpd_resp_send(req,NULL,0);
    refresh_mqtt_subscriptions(); publish_ha_discovery_messages();
    return ESP_OK;
}

httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = get_lamps_overview_handler };
        httpd_register_uri_handler(server, &root);
        httpd_uri_t add = { .uri = "/add_lamp", .method = HTTP_POST, .handler = add_lamp_post_handler };
        httpd_register_uri_handler(server, &add);
        httpd_uri_t rem = { .uri = "/remove_lamp", .method = HTTP_POST, .handler = remove_lamp_post_handler };
        httpd_register_uri_handler(server, &rem);
        httpd_uri_t edit = { .uri = "/edit_lamp", .method = HTTP_GET, .handler = edit_lamp_get_handler };
        httpd_register_uri_handler(server, &edit);
        httpd_uri_t upd = { .uri = "/update_lamp", .method = HTTP_POST, .handler = update_lamp_post_handler };
        httpd_register_uri_handler(server, &upd);

        // NEW CONFIG ROUTES
        httpd_uri_t cfg = { .uri = "/config", .method = HTTP_GET, .handler = get_config_handler };
        httpd_register_uri_handler(server, &cfg);
        httpd_uri_t test = { .uri = "/test_mqtt", .method = HTTP_POST, .handler = test_mqtt_post_handler };
        httpd_register_uri_handler(server, &test);
        httpd_uri_t save = { .uri = "/save_config", .method = HTTP_POST, .handler = save_config_post_handler };
        httpd_register_uri_handler(server, &save);
    }
    return server;
}