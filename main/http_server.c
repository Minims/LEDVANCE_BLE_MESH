#include "http_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include "cJSON.h"
#include "main.h"
#include "lamp_nvs.h"
#include <ctype.h>

// Forward declarations for BLE Mesh functions from main.c
extern void ble_mesh_send_ctl_get(uint16_t addr);
extern void ble_mesh_send_lightness_get(uint16_t addr);

static bool is_valid_hex_address(const char *addr)
{
    // Expect format 0xAAAA
    if (addr == NULL || strlen(addr) < 3 || addr[0] != '0' || (addr[1] != 'x' && addr[1] != 'X'))
    {
        return false;
    }
    for (int i = 2; i < strlen(addr); i++)
    {
        if (!isxdigit((unsigned char)addr[i]))
            return false;
    }
    return true;
}

#define TAG "HTTP_SERVER"

// Helper function to parse URL-encoded form data
static esp_err_t get_post_field(char *buf, const char *field, char *dest, int dest_len)
{
    const char *start = strstr(buf, field);
    if (!start)
    {
        return ESP_FAIL;
    }
    start += strlen(field); // Move pointer past "field="
    char *end = strchr(start, '&');
    size_t len;
    if (end)
    {
        len = end - start;
    }
    else
    {
        len = strlen(start);
    }

    if (len >= dest_len)
    {
        return ESP_FAIL; // Destination buffer too small
    }

    memcpy(dest, start, len);
    dest[len] = '\0';
    // Basic URL decoding for spaces ('+' -> ' ') and other chars might be needed for robustness
    char *p = dest;
    while ((p = strchr(p, '+')) != NULL)
    {
        *p = ' ';
    }
    return ESP_OK;
}

/* An HTTP POST handler for adding a new lamp */
static esp_err_t add_lamp_post_handler(httpd_req_t *req)
{
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0)
    {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    LampInfo new_lamp;
    char scaling_buf[16];
    char color_buf[4];
    char ctl_buf[4];
    char ack_buf[4];

    // Initialize the struct to zero to avoid garbage values
    memset(&new_lamp, 0, sizeof(LampInfo));

    if (get_post_field(buf, "lamp_name=", new_lamp.name, sizeof(new_lamp.name)) != ESP_OK ||
        get_post_field(buf, "lamp_address=", new_lamp.address, sizeof(new_lamp.address)) != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid form data");
        return ESP_FAIL;
    }
    // Parse Color Support (Checkbox present = true, otherwise false)
    if (get_post_field(buf, "lamp_color=", color_buf, sizeof(color_buf)) == ESP_OK)
    {
        new_lamp.supports_color = true;
    }
    else
    {
        new_lamp.supports_color = false;
    }
    // Parse CTL Support (Checkbox present = true, otherwise false)
    if (get_post_field(buf, "lamp_ctl=", ctl_buf, sizeof(ctl_buf)) == ESP_OK)
    {
        new_lamp.supports_ctl = true;
    }
    else
    {
        new_lamp.supports_ctl = false;
    }
    // Parse ACK mode (Checkbox present = true, otherwise false)
    if (get_post_field(buf, "lamp_ack=", ack_buf, sizeof(ack_buf)) == ESP_OK)
    {
        new_lamp.use_ack = true;
    }
    else
    {
        new_lamp.use_ack = false;
    }
    // Parse Brightness Scaling
    if (get_post_field(buf, "lamp_scaling=", scaling_buf, sizeof(scaling_buf)) == ESP_OK)
    {
        new_lamp.brightness_scaling = atoi(scaling_buf);
    }
    else
    {
        new_lamp.brightness_scaling = 100; // Default
    }

    // Parse Min Temperature (mirek: 800-20000)
    char min_temp_buf[16];
    if (get_post_field(buf, "lamp_min_temp=", min_temp_buf, sizeof(min_temp_buf)) == ESP_OK)
    {
        new_lamp.min_mirek = (uint16_t)atoi(min_temp_buf);
    }
    else
    {
        new_lamp.min_mirek = 800; // Default minimum mirek
    }

    // Parse Max Temperature (mirek: 800-20000)
    char max_temp_buf[16];
    if (get_post_field(buf, "lamp_max_temp=", max_temp_buf, sizeof(max_temp_buf)) == ESP_OK)
    {
        new_lamp.max_mirek = (uint16_t)atoi(max_temp_buf);
    }
    else
    {
        new_lamp.max_mirek = 20000; // Default maximum mirek
    }

    // validate proper address format
    if (!is_valid_hex_address(new_lamp.address))
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Address Format. Use 0x0000 hex format.");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Adding lamp: Name='%s', Addr='%s', Color=%d, CTL=%d, ACK=%d, Scale=%d, MinMirek=%d, MaxMirek=%d",
             new_lamp.name, new_lamp.address, new_lamp.supports_color, new_lamp.supports_ctl, new_lamp.use_ack, new_lamp.brightness_scaling, new_lamp.min_mirek, new_lamp.max_mirek);

    esp_err_t err = add_lamp_info(&new_lamp);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Lamp added successfully.");
        refresh_mqtt_subscriptions();
        publish_ha_discovery_messages();
    }
    else
    {
        ESP_LOGE(TAG, "Failed to add lamp: %s", esp_err_to_name(err));
    }

    // Redirect back to the main page
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* An HTTP POST handler for removing a lamp */
static esp_err_t remove_lamp_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0)
    {
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    char lamp_name[MAX_LAMP_NAME_LEN];
    if (get_post_field(buf, "lamp_name=", lamp_name, sizeof(lamp_name)) != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing lamp_name");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Removing lamp: Name='%s'", lamp_name);

    esp_err_t err = remove_lamp_info_by_name(lamp_name);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Lamp removed successfully.");
        refresh_mqtt_subscriptions();
        publish_ha_discovery_messages();
    }
    else
    {
        ESP_LOGE(TAG, "Failed to remove lamp: %s", esp_err_to_name(err));
    }

    // Redirect back to the main page
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* An HTTP POST handler for updating a lamp */
static esp_err_t update_lamp_post_handler(httpd_req_t *req)
{
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0)
    {
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    char original_name[MAX_LAMP_NAME_LEN];
    LampInfo updated_lamp;
    char scaling_buf[16];
    char color_buf[4];
    char ctl_buf[4];
    char ack_buf[4];

    // Initialize the struct to zero to avoid garbage values
    memset(&updated_lamp, 0, sizeof(LampInfo));

    if (get_post_field(buf, "original_name=", original_name, sizeof(original_name)) != ESP_OK ||
        get_post_field(buf, "lamp_name=", updated_lamp.name, sizeof(updated_lamp.name)) != ESP_OK ||
        get_post_field(buf, "lamp_address=", updated_lamp.address, sizeof(updated_lamp.address)) != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid form data");
        return ESP_FAIL;
    }
    // Parse Color Support
    if (get_post_field(buf, "lamp_color=", color_buf, sizeof(color_buf)) == ESP_OK)
    {
        updated_lamp.supports_color = true;
    }
    else
    {
        updated_lamp.supports_color = false;
    }
    // Parse CTL Support
    if (get_post_field(buf, "lamp_ctl=", ctl_buf, sizeof(ctl_buf)) == ESP_OK)
    {
        updated_lamp.supports_ctl = true;
    }
    else
    {
        updated_lamp.supports_ctl = false;
    }
    // Parse ACK mode
    if (get_post_field(buf, "lamp_ack=", ack_buf, sizeof(ack_buf)) == ESP_OK)
    {
        updated_lamp.use_ack = true;
    }
    else
    {
        updated_lamp.use_ack = false;
    }
    // Parse Brightness Scaling
    if (get_post_field(buf, "lamp_scaling=", scaling_buf, sizeof(scaling_buf)) == ESP_OK)
    {
        updated_lamp.brightness_scaling = atoi(scaling_buf);
    }
    else
    {
        updated_lamp.brightness_scaling = 100; // Default
    }

    // Parse Min Temperature (mirek: 800-20000)
    char min_temp_buf[16];
    if (get_post_field(buf, "lamp_min_temp=", min_temp_buf, sizeof(min_temp_buf)) == ESP_OK)
    {
        updated_lamp.min_mirek = (uint16_t)atoi(min_temp_buf);
    }
    else
    {
        updated_lamp.min_mirek = 800; // Default minimum mirek
    }

    // Parse Max Temperature (mirek: 800-20000)
    char max_temp_buf[16];
    if (get_post_field(buf, "lamp_max_temp=", max_temp_buf, sizeof(max_temp_buf)) == ESP_OK)
    {
        updated_lamp.max_mirek = (uint16_t)atoi(max_temp_buf);
    }
    else
    {
        updated_lamp.max_mirek = 20000; // Default maximum mirek
    }

    // validate proper address format
    if (!is_valid_hex_address(updated_lamp.address))
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Address Format. Use 0x0000 hex format.");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Updating lamp '%s' to Name='%s', Addr='%s', Color=%d, CTL=%d, ACK=%d, Scale=%d, MinMirek=%d, MaxMirek=%d",
             original_name, updated_lamp.name, updated_lamp.address, updated_lamp.supports_color, updated_lamp.supports_ctl, updated_lamp.use_ack, updated_lamp.brightness_scaling, updated_lamp.min_mirek, updated_lamp.max_mirek);

    esp_err_t err = update_lamp_info(original_name, &updated_lamp);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Lamp updated successfully.");
        refresh_mqtt_subscriptions();
        publish_ha_discovery_messages();
    }
    else
    {
        ESP_LOGE(TAG, "Failed to update lamp: %s", esp_err_to_name(err));
    }

    // Redirect back to the main page
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* An HTTP GET handler for the main lamp overview page */
static esp_err_t get_lamps_overview_handler(httpd_req_t *req)
{
    httpd_resp_sendstr_chunk(req,
                             "<html><head><title>BLE Mesh Gateway</title>"
                             "<style>"
                             "body { font-family: Arial, sans-serif; margin: 20px; background-color: #f4f4f9; }"
                             "h1, h2 { color: #333; }"
                             "table { width: 100%; border-collapse: collapse; margin-top: 20px; }"
                             "th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }"
                             "th { background-color: #4CAF50; color: white; }"
                             "tr:nth-child(even) { background-color: #f2f2f2; }"
                             "form { display: inline-block; margin: 0; }"
                             "input[type=submit] { background-color: #4CAF50; color: white; padding: 5px 10px; border: none; border-radius: 4px; cursor: pointer; }"
                             "input[type=submit].remove { background-color: #f44336; }"
                             "input[type=text] { padding: 5px; border-radius: 4px; border: 1px solid #ccc; }"
                             ".container { background-color: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }"
                             "</style></head><body><div class='container'>"
                             "<h1>Lamp Overview</h1><table>"
                             "<tr><th>Name</th><th>Address</th><th>Type</th><th>CTL</th><th>ACK Mode</th><th>Scale</th><th>Actions</th></tr>");

    int lamp_count;
    const LampInfo *lamps = get_all_lamps(&lamp_count);

    for (int i = 0; i < lamp_count; i++)
    {
        char row_buf[800];
        snprintf(row_buf, sizeof(row_buf),
                 "<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%d</td><td>"
                 "<form action='/remove_lamp' method='post'>"
                 "<input type='hidden' name='lamp_name' value='%s'>"
                 "<input type='submit' value='Remove' class='remove'>"
                 "</form> "
                 "<form action='/edit_lamp' method='get'>"
                 "<input type='hidden' name='lamp_name' value='%s'>"
                 "<input type='submit' value='Edit'>"
                 "</form> "
                 "<a href='/query_lamp?name=%s'><input type='button' value='Query'></a>"
                 "</td></tr>",
                 lamps[i].name, lamps[i].address,
                 lamps[i].supports_color ? "Color (HS)" : "White",
                 lamps[i].supports_ctl ? "Yes" : "No",
                 lamps[i].use_ack ? "ACK" : "UNACK",
                 lamps[i].brightness_scaling,
                 lamps[i].name, lamps[i].name, lamps[i].name);
        httpd_resp_sendstr_chunk(req, row_buf);
    }

    httpd_resp_sendstr_chunk(req, "</table>");

    httpd_resp_sendstr_chunk(req,
                             "<h2>Add New Lamp</h2>"
                             "<form action='/add_lamp' method='post'>"
                             "<label for='lamp_name'>Name: </label>"
                             "<input type='text' id='lamp_name' name='lamp_name' required> "
                             "<label for='lamp_address'>Address: </label>"
                             "<input type='text' id='lamp_address' name='lamp_address' placeholder='e.g., 0x0019' required><br><br>"
                             "<label for='lamp_color'>Supports Color (HS): </label>"
                             "<input type='checkbox' id='lamp_color' name='lamp_color' value='1'> "
                             "<label for='lamp_ctl'>Supports Temperature (CTL): </label>"
                             "<input type='checkbox' id='lamp_ctl' name='lamp_ctl' value='1'><br><br>"
                             "<label for='lamp_ack'>Use ACK Mode: </label>"
                             "<input type='checkbox' id='lamp_ack' name='lamp_ack' value='1'> "
                             "<small>(default: UNACK)</small><br><br>"
                             "<label for='lamp_scaling'>Brightness Scale (e.g., 100): </label>"
                             "<input type='number' id='lamp_scaling' name='lamp_scaling' value='100' required><br><br>"
                             "<label for='lamp_min_temp'>Min Temperature Mirek (800-20000): </label>"
                             "<input type='number' id='lamp_min_temp' name='lamp_min_temp' value='800' min='800' max='20000'> "
                             "<label for='lamp_max_temp'>Max Temperature Mirek (800-20000): </label>"
                             "<input type='number' id='lamp_max_temp' name='lamp_max_temp' value='20000' min='800' max='20000'><br><br>"
                             "<input type='submit' value='Add Lamp'>"
                             "</form>"
                             "<h2>System</h2>"
                             "<form action='/restart' method='post'><input type='submit' value='Restart Device'></form>"
                             "</div></body></html>");

    // Final chunk to end the response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* An HTTP GET handler to serve the edit lamp page */
static esp_err_t edit_lamp_get_handler(httpd_req_t *req)
{
    char query_buf[128];
    char lamp_name[MAX_LAMP_NAME_LEN];
    LampInfo lamp_to_edit;

    if (httpd_req_get_url_query_str(req, query_buf, sizeof(query_buf)) != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query string");
        return ESP_FAIL;
    }
    if (httpd_query_key_value(query_buf, "lamp_name", lamp_name, sizeof(lamp_name)) != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing lamp_name parameter");
        return ESP_FAIL;
    }

    if (find_lamp_by_name(lamp_name, &lamp_to_edit) != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Lamp not found");
        return ESP_FAIL;
    }

    // Send the response in chunks to avoid large stack buffers and compiler warnings
    httpd_resp_sendstr_chunk(req,
                             "<html><head><title>Edit Lamp</title>"
                             "<style>"
                             "body { font-family: Arial, sans-serif; margin: 20px; background-color: #f4f4f9; }"
                             "h1 { color: #333; }"
                             ".container { background-color: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }"
                             "input[type=text], input[type=submit] { padding: 8px; margin-top: 5px; border-radius: 4px; border: 1px solid #ccc; }"
                             "input[type=submit] { background-color: #4CAF50; color: white; cursor: pointer; }"
                             "</style></head><body><div class='container'>");

    char chunk_buf[1400]; // Increased buffer size for expanded form with temp_inverted field

    // Send header with lamp name
    snprintf(chunk_buf, sizeof(chunk_buf), "<h1>Edit Lamp: %s</h1>", lamp_to_edit.name);
    httpd_resp_sendstr_chunk(req, chunk_buf);

    // Send form part 1
    snprintf(chunk_buf, sizeof(chunk_buf),
             "<form action='/update_lamp' method='post'>"
             "<input type='hidden' name='original_name' value='%s'>"
             "<label for='lamp_name'>New Name:</label><br>"
             "<input type='text' id='lamp_name' name='lamp_name' value='%s' required><br><br>",
             lamp_to_edit.name, lamp_to_edit.name);
    httpd_resp_sendstr_chunk(req, chunk_buf);

    // Send form part 2
    snprintf(chunk_buf, sizeof(chunk_buf),
             "<label for='lamp_address'>New Address:</label><br>"
             "<input type='text' id='lamp_address' name='lamp_address' value='%s' required><br><br>"
             "<label for='lamp_color'>Supports Color (HS): </label>"
             "<input type='checkbox' id='lamp_color' name='lamp_color' value='1' %s> "
             "<label for='lamp_ctl'>Supports Temperature (CTL): </label>"
             "<input type='checkbox' id='lamp_ctl' name='lamp_ctl' value='1' %s><br><br>"
             "<label for='lamp_ack'>Use ACK Mode: </label>"
             "<input type='checkbox' id='lamp_ack' name='lamp_ack' value='1' %s> "
             "<small>(default: UNACK)</small><br><br>"
             "<label for='lamp_scaling'>Brightness Scale:</label><br>"
             "<input type='number' id='lamp_scaling' name='lamp_scaling' value='%d' required><br><br>"
             "<label for='lamp_min_temp'>Min Temperature Mirek (800-20000):</label><br>"
             "<input type='number' id='lamp_min_temp' name='lamp_min_temp' value='%d' min='800' max='20000'> "
             "<label for='lamp_max_temp'>Max Temperature Mirek (800-20000):</label><br>"
             "<input type='number' id='lamp_max_temp' name='lamp_max_temp' value='%d' min='800' max='20000'><br><br>"
             "<input type='submit' value='Update Lamp'>"
             "</form><br><a href='/'>Back to Overview</a>"
             "</div></body></html>",
             lamp_to_edit.address,
             lamp_to_edit.supports_color ? "checked" : "",
             lamp_to_edit.supports_ctl ? "checked" : "",
             lamp_to_edit.use_ack ? "checked" : "",
             lamp_to_edit.brightness_scaling,
             lamp_to_edit.min_mirek,
             lamp_to_edit.max_mirek);
    httpd_resp_sendstr_chunk(req, chunk_buf);

    // Send final chunk to close connection
    httpd_resp_send_chunk(req, NULL, 0);

    return ESP_OK;
}

/* An HTTP GET handler to query lamp status via BLE Mesh */
static esp_err_t query_lamp_get_handler(httpd_req_t *req)
{
    char query_buf[128];
    char lamp_name[MAX_LAMP_NAME_LEN];
    LampInfo lamp_info;

    if (httpd_req_get_url_query_str(req, query_buf, sizeof(query_buf)) != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query string");
        return ESP_FAIL;
    }
    if (httpd_query_key_value(query_buf, "name", lamp_name, sizeof(lamp_name)) != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing 'name' parameter");
        return ESP_FAIL;
    }

    if (find_lamp_by_name(lamp_name, &lamp_info) != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Lamp not found");
        return ESP_FAIL;
    }

    uint16_t addr = (uint16_t)strtol(lamp_info.address, NULL, 0);
    ESP_LOGI(TAG, "HTTP: Querying lamp '%s' (0x%04X) capabilities...", lamp_name, addr);

    // Always query Lightness Range first
    ble_mesh_send_lightness_get(addr);

    // Then query Temperature Range if CTL supported
    if (lamp_info.supports_ctl)
    {
        ble_mesh_send_ctl_get(addr);
    }

    httpd_resp_sendstr(req, "Query sent. Check logs for lamp response in 10-15 seconds.");
    return ESP_OK;
}

/* An HTTP POST handler to restart the device */
static esp_err_t restart_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Restarting device via HTTP request.");
    httpd_resp_sendstr(req, "Restarting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard; // Allow wildcard matching

    ESP_LOGI(TAG, "Starting httpd server");
    if (httpd_start(&server, &config) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start httpd server");
        return NULL;
    }

    // URI Handlers
    const httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = get_lamps_overview_handler,
    };
    httpd_register_uri_handler(server, &root);

    const httpd_uri_t add_lamp = {
        .uri = "/add_lamp",
        .method = HTTP_POST,
        .handler = add_lamp_post_handler,
    };
    httpd_register_uri_handler(server, &add_lamp);

    const httpd_uri_t remove_lamp = {
        .uri = "/remove_lamp",
        .method = HTTP_POST,
        .handler = remove_lamp_post_handler,
    };
    httpd_register_uri_handler(server, &remove_lamp);

    const httpd_uri_t edit_lamp = {
        .uri = "/edit_lamp",
        .method = HTTP_GET,
        .handler = edit_lamp_get_handler,
    };
    httpd_register_uri_handler(server, &edit_lamp);

    const httpd_uri_t update_lamp = {
        .uri = "/update_lamp",
        .method = HTTP_POST,
        .handler = update_lamp_post_handler,
    };
    httpd_register_uri_handler(server, &update_lamp);

    const httpd_uri_t query_lamp = {
        .uri = "/query_lamp",
        .method = HTTP_GET,
        .handler = query_lamp_get_handler,
    };
    httpd_register_uri_handler(server, &query_lamp);

    const httpd_uri_t restart = {
        .uri = "/restart",
        .method = HTTP_POST,
        .handler = restart_post_handler,
    };
    httpd_register_uri_handler(server, &restart);

    return server;
}
