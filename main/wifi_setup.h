#ifndef WIFI_SETUP_H
#define WIFI_SETUP_H

#include "esp_err.h"

/**
 * @brief Tries to connect to stored Wi-Fi. 
 * If it fails or no credentials exist, it starts an Access Point (SoftAP)
 * with a web server to let the user enter credentials.
 * * @return ESP_OK if connected to Wi-Fi successfully.
 * @return ESP_FAIL if connection failed and AP mode was started (Application should stop and wait).
 */
esp_err_t wifi_setup_init(void);

#endif