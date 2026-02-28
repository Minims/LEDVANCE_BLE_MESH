// Define LOG_LOCAL_LEVEL BEFORE including esp_log.h
// This is required for ESP_LOGx macros to work in C files
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include "ble_mesh_bridge.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>

#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_lighting_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"

static const char *TAG = "ble_mesh_bridge";
#define CID_ESP 0x02E5
#define NVS_MESH_INFO_KEY "mesh_info_clean"

// Custom logging macros that work with ESPHome's logging system
// ESP-IDF's ESP_LOGx macros don't appear in ESPHome's log output from C files
#define LOG_I(tag, fmt, ...) printf("[I][%s]: " fmt "\n", tag, ##__VA_ARGS__)
#define LOG_W(tag, fmt, ...) printf("[W][%s]: " fmt "\n", tag, ##__VA_ARGS__)
#define LOG_E(tag, fmt, ...) printf("[E][%s]: " fmt "\n", tag, ##__VA_ARGS__)

// --- State ---
typedef struct {
  uint16_t net_idx;
  uint16_t app_idx;
  uint8_t tid;
} __attribute__((packed)) mesh_app_state_t;

static mesh_app_state_t s_app_state = {
    .net_idx = ESP_BLE_MESH_KEY_UNUSED,
    .app_idx = ESP_BLE_MESH_KEY_UNUSED,
    .tid = 0,
};

static uint8_t s_dev_uuid[16] = {0};

// --- Model Definitions ---
static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
    .default_ttl = 7,
    .net_transmit = ESP_BLE_MESH_TRANSMIT(4, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(4, 20),
};

static esp_ble_mesh_client_t onoff_client;
static esp_ble_mesh_client_t level_client;
static esp_ble_mesh_client_t light_client;
static esp_ble_mesh_client_t hsl_client;

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_cli_pub, 2 + 2, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(level_cli_pub, 2 + 2, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(light_cli_pub, 2 + 2, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(hsl_cli_pub, 2 + 2, ROLE_NODE);

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(&onoff_cli_pub, &onoff_client),
    ESP_BLE_MESH_MODEL_GEN_LEVEL_CLI(&level_cli_pub, &level_client),
    ESP_BLE_MESH_MODEL_LIGHT_LIGHTNESS_CLI(&light_cli_pub, &light_client),
    ESP_BLE_MESH_MODEL_LIGHT_HSL_CLI(&hsl_cli_pub, &hsl_client),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
    .uuid = s_dev_uuid,
    .output_size = 0,
    .output_actions = 0,
};

// --- NVS Helper ---
static void mesh_info_store(void) {
  nvs_handle_t handle;
  if (nvs_open("ble_mesh", NVS_READWRITE, &handle) == ESP_OK) {
    nvs_set_blob(handle, NVS_MESH_INFO_KEY, &s_app_state, sizeof(s_app_state));
    nvs_commit(handle);
    nvs_close(handle);
  }
}

static void mesh_info_restore(void) {
  nvs_handle_t handle;
  if (nvs_open("ble_mesh", NVS_READONLY, &handle) == ESP_OK) {
    size_t len = sizeof(s_app_state);
    if (nvs_get_blob(handle, NVS_MESH_INFO_KEY, &s_app_state, &len) == ESP_OK) {
      ESP_LOGI(TAG, "Restored Mesh State: NetKey=0x%04X, AppKey=0x%04X",
               s_app_state.net_idx, s_app_state.app_idx);
    }
    nvs_close(handle);
  }
}

// --- Callbacks ---
static void prov_callback(esp_ble_mesh_prov_cb_event_t event,
                          esp_ble_mesh_prov_cb_param_t *param) {
  switch (event) {
  case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
    ESP_LOGI(TAG, "Registered Composition");
    break;
  case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
    ESP_LOGI(TAG, "Provisioning Complete. NetKey Index: 0x%04X",
             param->node_prov_complete.net_idx);
    s_app_state.net_idx = param->node_prov_complete.net_idx;
    mesh_info_store();
    break;
  default:
    break;
  }
}

static void config_server_callback(esp_ble_mesh_cfg_server_cb_event_t event,
                                   esp_ble_mesh_cfg_server_cb_param_t *param) {
  if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
    if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND) {
      ESP_LOGI(TAG, "Model Bound to AppKey 0x%04X",
               param->value.state_change.mod_app_bind.app_idx);
      s_app_state.app_idx = param->value.state_change.mod_app_bind.app_idx;
      mesh_info_store();
    }
  }
}

static void
generic_client_callback(esp_ble_mesh_generic_client_cb_event_t event,
                        esp_ble_mesh_generic_client_cb_param_t *param) {
  // Log function placeholder
}

static void light_client_callback(esp_ble_mesh_light_client_cb_event_t event,
                                  esp_ble_mesh_light_client_cb_param_t *param) {
  // Log function placeholder
}

// --- Public Accessors ---

bool ble_mesh_bridge_is_ready_to_init(void) {
  return esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED;
}

void ble_mesh_bridge_init(void) {
  // Note: Bluetooth controller init is handled by ESPHome's
  // esp32_ble_tracker/esp_bt registration

  LOG_I(TAG, "Initializing BLE Mesh Bridge...");

  // Check BT Controller Status
  esp_bt_controller_status_t status = esp_bt_controller_get_status();
  LOG_I(TAG, "BT Controller Status: %d (Enabled=%d)", status,
        ESP_BT_CONTROLLER_STATUS_ENABLED);

  // Set UUID
  esp_efuse_mac_get_default(s_dev_uuid);
  LOG_I(TAG,
        "Device UUID: "
        "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%"
        "02X:%02X:%02X",
        s_dev_uuid[0], s_dev_uuid[1], s_dev_uuid[2], s_dev_uuid[3],
        s_dev_uuid[4], s_dev_uuid[5], s_dev_uuid[6], s_dev_uuid[7],
        s_dev_uuid[8], s_dev_uuid[9], s_dev_uuid[10], s_dev_uuid[11],
        s_dev_uuid[12], s_dev_uuid[13], s_dev_uuid[14], s_dev_uuid[15]);

  mesh_info_restore();

  esp_ble_mesh_register_prov_callback(prov_callback);
  esp_ble_mesh_register_config_server_callback(config_server_callback);
  esp_ble_mesh_register_generic_client_callback(generic_client_callback);
  esp_ble_mesh_register_light_client_callback(light_client_callback);

  esp_err_t err = esp_ble_mesh_init(&provision, &composition);
  if (err == ESP_ERR_INVALID_STATE) {
    LOG_W(TAG,
          "Mesh stack already initialized! Attempting deinit and retry...");
    esp_ble_mesh_deinit(NULL);
    // Small delay?
    err = esp_ble_mesh_init(&provision, &composition);
  }

  if (err != ESP_OK) {
    LOG_E(TAG, "Failed to initialize mesh stack (err %d)", err);
    return;
  }

  err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV |
                                      ESP_BLE_MESH_PROV_GATT);
  if (err == ESP_ERR_INVALID_STATE) {
    LOG_W(TAG, "Node prov already enabled? Continuing...");
  } else if (err != ESP_OK) {
    LOG_E(TAG, "Failed to enable mesh node (err %d)", err);
    return;
  }

  LOG_I(TAG, "BLE Mesh Node initialized (Bridge)");
}

void ble_mesh_bridge_send_onoff(uint16_t addr, bool state) {
  if (s_app_state.app_idx == ESP_BLE_MESH_KEY_UNUSED) {
    LOG_W(TAG, "AppKey not bound, cannot send OnOff.");
    return;
  }

  esp_ble_mesh_generic_client_set_state_t set = {0};
  esp_ble_mesh_client_common_param_t common = {0};

  common.opcode = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK;
  common.model = onoff_client.model;
  common.ctx.net_idx = s_app_state.net_idx;
  common.ctx.app_idx = s_app_state.app_idx;
  common.ctx.addr = addr;
  common.ctx.send_ttl = 7;
  common.msg_timeout = 0;

  set.onoff_set.op_en = false;
  set.onoff_set.onoff = state ? 1 : 0;
  set.onoff_set.tid = s_app_state.tid++;

  esp_err_t err = esp_ble_mesh_generic_client_set_state(&common, &set);
  if (err) {
    LOG_E(TAG, "Send OnOff Set failed: %d", err);
  }
}

void ble_mesh_bridge_send_level(uint16_t addr, uint16_t level) {
  if (s_app_state.app_idx == ESP_BLE_MESH_KEY_UNUSED)
    return;

  esp_ble_mesh_light_client_set_state_t set = {0};
  esp_ble_mesh_client_common_param_t common = {0};

  common.opcode = ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET_UNACK;
  common.model = light_client.model;
  common.ctx.net_idx = s_app_state.net_idx;
  common.ctx.app_idx = s_app_state.app_idx;
  common.ctx.addr = addr;
  common.ctx.send_ttl = 7;

  set.lightness_set.op_en = false;
  set.lightness_set.lightness = level;
  set.lightness_set.tid = s_app_state.tid++;

  esp_ble_mesh_light_client_set_state(&common, &set);
}

void ble_mesh_bridge_send_hsl(uint16_t addr, uint16_t lightness, uint16_t hue,
                              uint16_t saturation) {
  if (s_app_state.app_idx == ESP_BLE_MESH_KEY_UNUSED)
    return;

  esp_ble_mesh_light_client_set_state_t set = {0};
  esp_ble_mesh_client_common_param_t common = {0};

  common.opcode = ESP_BLE_MESH_MODEL_OP_LIGHT_HSL_SET_UNACK;
  common.model = hsl_client.model;
  common.ctx.net_idx = s_app_state.net_idx;
  common.ctx.app_idx = s_app_state.app_idx;
  common.ctx.addr = addr;
  common.ctx.send_ttl = 7;

  set.hsl_set.op_en = false;
  set.hsl_set.hsl_lightness = lightness;
  set.hsl_set.hsl_hue = hue;
  set.hsl_set.hsl_saturation = saturation;
  set.hsl_set.tid = s_app_state.tid++;

  esp_ble_mesh_light_client_set_state(&common, &set);
}
