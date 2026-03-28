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
#include <inttypes.h>
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
static esp_ble_mesh_client_t ctl_client;

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_cli_pub, 2 + 2, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(level_cli_pub, 2 + 2, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(light_cli_pub, 2 + 2, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(hsl_cli_pub, 2 + 2, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(ctl_cli_pub, 2 + 2, ROLE_NODE);

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(&onoff_cli_pub, &onoff_client),
    ESP_BLE_MESH_MODEL_GEN_LEVEL_CLI(&level_cli_pub, &level_client),
    ESP_BLE_MESH_MODEL_LIGHT_LIGHTNESS_CLI(&light_cli_pub, &light_client),
    ESP_BLE_MESH_MODEL_LIGHT_HSL_CLI(&hsl_cli_pub, &hsl_client),
    ESP_BLE_MESH_MODEL_LIGHT_CTL_CLI(&ctl_cli_pub, &ctl_client),
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

static bool mesh_state_ready_(void) {
  return s_app_state.net_idx != ESP_BLE_MESH_KEY_UNUSED &&
         s_app_state.app_idx != ESP_BLE_MESH_KEY_UNUSED;
}

static bool mesh_state_ready_or_warn_(const char *operation) {
  if (mesh_state_ready_()) {
    return true;
  }

  LOG_W(TAG, "Mesh keys not ready, cannot %s.", operation);
  return false;
}

static void fill_common_ctx_(esp_ble_mesh_client_common_param_t *common,
                             uint16_t addr, uint32_t opcode,
                             esp_ble_mesh_model_t *model, bool ack) {
  memset(common, 0, sizeof(*common));
  common->opcode = opcode;
  common->model = model;
  common->ctx.net_idx = s_app_state.net_idx;
  common->ctx.app_idx = s_app_state.app_idx;
  common->ctx.addr = addr;
  common->ctx.send_ttl = 7;
  if (ack) {
    common->msg_timeout = 0;
  }
}

static void bind_local_client_models_(uint16_t elem_addr, uint16_t app_idx) {
  struct bind_entry_t {
    uint16_t model_id;
    const char *name;
  };

  static const struct bind_entry_t entries[] = {
      {ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI, "GEN_ONOFF_CLI"},
      {ESP_BLE_MESH_MODEL_ID_GEN_LEVEL_CLI, "GEN_LEVEL_CLI"},
      {ESP_BLE_MESH_MODEL_ID_LIGHT_LIGHTNESS_CLI, "LIGHT_LIGHTNESS_CLI"},
      {ESP_BLE_MESH_MODEL_ID_LIGHT_HSL_CLI, "LIGHT_HSL_CLI"},
      {ESP_BLE_MESH_MODEL_ID_LIGHT_CTL_CLI, "LIGHT_CTL_CLI"},
  };

  for (size_t i = 0; i < ARRAY_SIZE(entries); i++) {
    esp_err_t err = esp_ble_mesh_node_bind_app_key_to_local_model(
        elem_addr, 0xFFFF, entries[i].model_id, app_idx);
    if (err != ESP_OK) {
      LOG_W(TAG, "Failed to bind %s to AppKey 0x%04X (err %d)",
            entries[i].name, app_idx, err);
    } else {
      LOG_I(TAG, "Bound %s to AppKey 0x%04X", entries[i].name, app_idx);
    }
  }
}

// --- NVS Helper ---
static void mesh_info_store(void) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open("ble_mesh", NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    LOG_W(TAG, "Failed to open NVS for write (err %d)", err);
    return;
  }

  err = nvs_set_blob(handle, NVS_MESH_INFO_KEY, &s_app_state, sizeof(s_app_state));
  if (err != ESP_OK) {
    LOG_W(TAG, "Failed to store mesh info in NVS (err %d)", err);
  } else {
    err = nvs_commit(handle);
    if (err != ESP_OK) {
      LOG_W(TAG, "Failed to commit mesh info to NVS (err %d)", err);
    }
  }
  nvs_close(handle);
}

static void mesh_info_restore(void) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open("ble_mesh", NVS_READONLY, &handle);
  if (err != ESP_OK) {
    LOG_W(TAG, "Failed to open NVS for read (err %d)", err);
    return;
  }

  size_t len = sizeof(s_app_state);
  err = nvs_get_blob(handle, NVS_MESH_INFO_KEY, &s_app_state, &len);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Restored Mesh State: NetKey=0x%04X, AppKey=0x%04X",
             s_app_state.net_idx, s_app_state.app_idx);
  } else {
    LOG_W(TAG, "No persisted mesh info restored (err %d)", err);
  }
  nvs_close(handle);
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

      uint16_t elem_addr = esp_ble_mesh_get_primary_element_address();
      if (elem_addr != ESP_BLE_MESH_ADDR_UNASSIGNED) {
        bind_local_client_models_(elem_addr, s_app_state.app_idx);
      }
    }
  }
}

static void
generic_client_callback(esp_ble_mesh_generic_client_cb_event_t event,
                        esp_ble_mesh_generic_client_cb_param_t *param) {
  if (param == NULL) {
    LOG_W(TAG, "Generic client callback with null param (event %d)", event);
    return;
  }

  if (param->error_code != ESP_OK) {
    LOG_W(TAG, "Generic client event %d failed with err %d", event,
          param->error_code);
    return;
  }

  if (param->params == NULL) {
    LOG_W(TAG, "Generic client callback missing params (event %d)", event);
    return;
  }

  switch (event) {
  case ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT:
  case ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT:
  case ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT:
    LOG_I(TAG, "[GENERIC EVT] event=%d addr=0x%04X recv_op=0x%04" PRIX32,
          event, param->params->ctx.addr, param->params->ctx.recv_op);
    break;
  case ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT:
    LOG_W(TAG, "Generic client timeout addr=0x%04X opcode=0x%04" PRIX32,
          param->params->ctx.addr, param->params->opcode);
    break;
  default:
    break;
  }
}

static void light_client_callback(esp_ble_mesh_light_client_cb_event_t event,
                                  esp_ble_mesh_light_client_cb_param_t *param) {
  if (param == NULL) {
    LOG_W(TAG, "Light client callback with null param (event %d)", event);
    return;
  }

  if (param->error_code != ESP_OK) {
    LOG_W(TAG, "Light client event %d failed with err %d", event,
          param->error_code);
    return;
  }

  if (param->params == NULL) {
    LOG_W(TAG, "Light client callback missing params (event %d)", event);
    return;
  }

  switch (event) {
  case ESP_BLE_MESH_LIGHT_CLIENT_GET_STATE_EVT:
  case ESP_BLE_MESH_LIGHT_CLIENT_SET_STATE_EVT:
  case ESP_BLE_MESH_LIGHT_CLIENT_PUBLISH_EVT:
    if (param->params->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_STATUS) {
      LOG_I(TAG, "[REPONSE LAMPE CTL TEMP] Addr: 0x%04X | Luminosite: %d | Temperature: %d",
            param->params->ctx.addr,
            param->status_cb.ctl_status.present_ctl_lightness,
            param->status_cb.ctl_status.present_ctl_temperature);
    } else if (param->params->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_STATUS) {
      LOG_I(TAG, "[REPONSE LAMPE LIGHTNESS] Addr: 0x%04X | Luminosite: %d",
            param->params->ctx.addr,
            param->status_cb.lightness_status.present_lightness);
    } else if (param->params->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_RANGE_STATUS) {
      LOG_I(TAG, "[REPONSE LAMPE RANGE LUMINOSITE] Addr: 0x%04X | Min: %d | Max: %d",
            param->params->ctx.addr,
            param->status_cb.lightness_range_status.range_min,
            param->status_cb.lightness_range_status.range_max);
    } else if (param->params->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_TEMPERATURE_RANGE_STATUS) {
      LOG_I(TAG, "[REPONSE LAMPE RANGE TEMPERATURE] Addr: 0x%04X | Min: %d | Max: %d",
            param->params->ctx.addr,
            param->status_cb.ctl_temperature_range_status.range_min,
            param->status_cb.ctl_temperature_range_status.range_max);
    } else if (param->params->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_LIGHT_HSL_STATUS) {
      LOG_I(TAG, "[REPONSE LAMPE HSL COULEUR] Addr: 0x%04X | Lum: %d | Hue: %d | Sat: %d",
            param->params->ctx.addr,
            param->status_cb.hsl_status.hsl_lightness,
            param->status_cb.hsl_status.hsl_hue,
            param->status_cb.hsl_status.hsl_saturation);
    } else {
      LOG_I(TAG, "[LIGHT EVT] event=%d addr=0x%04X recv_op=0x%04" PRIX32,
            event, param->params->ctx.addr, param->params->ctx.recv_op);
    }
    break;
  case ESP_BLE_MESH_LIGHT_CLIENT_TIMEOUT_EVT:
    LOG_W(TAG, "Light client timeout addr=0x%04X opcode=0x%04" PRIX32,
          param->params->ctx.addr, param->params->opcode);
    break;
  default:
    break;
  }
}

// --- Public Accessors ---

bool ble_mesh_bridge_is_ready_to_init(void) {
  return esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED;
}

bool ble_mesh_bridge_is_ready(void) { return mesh_state_ready_(); }

bool ble_mesh_bridge_init(void) {
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

  esp_err_t err = esp_ble_mesh_register_prov_callback(prov_callback);
  if (err != ESP_OK) {
    LOG_E(TAG, "Failed to register provisioning callback (err %d)", err);
    return false;
  }

  err = esp_ble_mesh_register_config_server_callback(config_server_callback);
  if (err != ESP_OK) {
    LOG_E(TAG, "Failed to register config server callback (err %d)", err);
    return false;
  }

  err = esp_ble_mesh_register_generic_client_callback(generic_client_callback);
  if (err != ESP_OK) {
    LOG_E(TAG, "Failed to register generic client callback (err %d)", err);
    return false;
  }

  err = esp_ble_mesh_register_light_client_callback(light_client_callback);
  if (err != ESP_OK) {
    LOG_E(TAG, "Failed to register light client callback (err %d)", err);
    return false;
  }

  err = esp_ble_mesh_init(&provision, &composition);
  if (err == ESP_ERR_INVALID_STATE) {
    LOG_W(TAG,
          "Mesh stack already initialized! Attempting deinit and retry...");
    esp_ble_mesh_deinit(NULL);
    // Small delay?
    err = esp_ble_mesh_init(&provision, &composition);
  }

  if (err != ESP_OK) {
    LOG_E(TAG, "Failed to initialize mesh stack (err %d)", err);
    return false;
  }

  err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV |
                                      ESP_BLE_MESH_PROV_GATT);
  if (err == ESP_ERR_INVALID_STATE) {
    LOG_W(TAG, "Node prov already enabled? Continuing...");
  } else if (err != ESP_OK) {
    LOG_E(TAG, "Failed to enable mesh node (err %d)", err);
    return false;
  }

  if (mesh_state_ready_()) {
    uint16_t elem_addr = esp_ble_mesh_get_primary_element_address();
    if (elem_addr != ESP_BLE_MESH_ADDR_UNASSIGNED) {
      bind_local_client_models_(elem_addr, s_app_state.app_idx);
      LOG_I(TAG, "Auto-bound client models to AppKey 0x%04X", s_app_state.app_idx);
    }
  } else {
    LOG_W(TAG, "Mesh initialized but NetKey/AppKey is not ready yet.");
  }

  LOG_I(TAG, "BLE Mesh Node initialized (Bridge)");
  return true;
}

void ble_mesh_bridge_send_onoff(uint16_t addr, bool state, bool ack) {
  if (!mesh_state_ready_or_warn_("send OnOff")) {
    return;
  }

  esp_ble_mesh_generic_client_set_state_t set = {0};
  esp_ble_mesh_client_common_param_t common;
  fill_common_ctx_(&common, addr,
                   ack ? ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET
                       : ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK,
                   onoff_client.model, ack);

  set.onoff_set.op_en = false;
  set.onoff_set.onoff = state ? 1 : 0;
  set.onoff_set.tid = s_app_state.tid++;

  esp_err_t err = esp_ble_mesh_generic_client_set_state(&common, &set);
  if (err) {
    LOG_E(TAG, "Send OnOff Set failed: %d", err);
  }
}

void ble_mesh_bridge_send_level(uint16_t addr, uint16_t level) {
  if (!mesh_state_ready_or_warn_("send Lightness")) {
    return;
  }

  esp_ble_mesh_light_client_set_state_t set = {0};
  esp_ble_mesh_client_common_param_t common;
  fill_common_ctx_(&common, addr, ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET_UNACK,
                   light_client.model, false);

  set.lightness_set.op_en = false;
  set.lightness_set.lightness = level;
  set.lightness_set.tid = s_app_state.tid++;

  esp_err_t err = esp_ble_mesh_light_client_set_state(&common, &set);
  if (err != ESP_OK) {
    LOG_E(TAG, "Send Lightness Set failed: %d", err);
  }
}

void ble_mesh_bridge_get_ranges(uint16_t addr) {
  if (!mesh_state_ready_or_warn_("request ranges")) {
    return;
  }

  esp_ble_mesh_client_common_param_t common;
  esp_ble_mesh_light_client_get_state_t get = {0};

  // Request the supported lightness range.
  fill_common_ctx_(&common, addr, ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_RANGE_GET,
                   light_client.model, true);
  esp_err_t err = esp_ble_mesh_light_client_get_state(&common, &get);
  if (err != ESP_OK) {
    LOG_E(TAG, "Request Lightness Range failed: %d", err);
  }

  // Request the supported CTL temperature range.
  fill_common_ctx_(&common, addr,
                   ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_TEMPERATURE_RANGE_GET,
                   ctl_client.model, true);
  err = esp_ble_mesh_light_client_get_state(&common, &get);
  if (err != ESP_OK) {
    LOG_E(TAG, "Request CTL Temperature Range failed: %d", err);
  }
}

void ble_mesh_bridge_send_ctl(uint16_t addr, uint16_t lightness, uint16_t temperature, bool ack) {
  if (!mesh_state_ready_or_warn_("send CTL")) {
    return;
  }

  esp_ble_mesh_light_client_set_state_t set = {0};
  esp_ble_mesh_client_common_param_t common;
  fill_common_ctx_(&common, addr,
                   ack ? ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_SET
                       : ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_SET_UNACK,
                   ctl_client.model, ack);

  set.ctl_set.op_en = false;
  set.ctl_set.ctl_lightness = lightness;
  set.ctl_set.ctl_temperature = temperature;
  set.ctl_set.ctl_delta_uv = 0;
  set.ctl_set.tid = s_app_state.tid++;

  esp_err_t err = esp_ble_mesh_light_client_set_state(&common, &set);
  if (err != ESP_OK) {
    LOG_E(TAG, "Send CTL Set failed: %d", err);
  }
}

void ble_mesh_bridge_send_hsl(uint16_t addr, uint16_t lightness, uint16_t hue,
                              uint16_t saturation) {
  if (!mesh_state_ready_or_warn_("send HSL")) {
    return;
  }

  esp_ble_mesh_light_client_set_state_t set = {0};
  esp_ble_mesh_client_common_param_t common;
  fill_common_ctx_(&common, addr, ESP_BLE_MESH_MODEL_OP_LIGHT_HSL_SET_UNACK,
                   hsl_client.model, false);

  set.hsl_set.op_en = false;
  set.hsl_set.hsl_lightness = lightness;
  set.hsl_set.hsl_hue = hue;
  set.hsl_set.hsl_saturation = saturation;
  set.hsl_set.tid = s_app_state.tid++;

  esp_err_t err = esp_ble_mesh_light_client_set_state(&common, &set);
  if (err != ESP_OK) {
    LOG_E(TAG, "Send HSL Set failed: %d", err);
  }
}
