#pragma once

#include "ble_mesh_bridge.h"
#include "esphome.h"

#if !defined(CONFIG_BLE_MESH)
#error "CONFIG_BLE_MESH not defined! Check sdkconfig."
#endif

namespace esphome {
namespace ble_mesh_gateway {

static const char *TAG = "ble_mesh_gateway";

class BleMeshGateway : public Component {
public:
  float get_setup_priority() const override {
    return setup_priority::AFTER_BLUETOOTH;
  }

  // --- ESPHome Component Overrides ---

  void loop() override {
    if (!this->init_done_) {
      if (ble_mesh_bridge_is_ready_to_init()) {
        ESP_LOGI(TAG, "BT Controller Ready. Initializing Mesh Bridge...");
        ble_mesh_bridge_init();
        this->init_done_ = true;
      } else {
        // Optional: Log waiting every few seconds if needed, but keeping it
        // silent is fine. Or log once.
        static bool warned = false;
        if (!warned) {
          ESP_LOGW(TAG, "Waiting for BT Controller to be enabled...");
          warned = true;
        }
      }
    }
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "BLE Mesh Gateway (Bridged)");
  }

  // --- Public API for YAML Lambdas ---

  void control_light(uint16_t addr, float state, uint32_t &last_send,
                     uint16_t max_level = 50) {
    if (!this->init_done_) {
      ESP_LOGW(TAG, "Mesh not ready, skipping control_light");
      return;
    }

    uint32_t now = millis();
    // Rate Limit: Only send level every 100ms prevents buffer flooding
    // Immediate update if turning OFF (0) or MAX (1.0) for responsiveness
    if (now - last_send > 100 || state == 0 || state == 1.0) {
      uint16_t level = 0;
      if (state > 0) {
        // SCALING: Map 0-1.0 to 0-max_level
        level = (uint16_t)(state * max_level);
      }

      ble_mesh_bridge_send_level(addr, level);

      // Send explicit OnOff command only when turning off completely
      if (state == 0) {
        ble_mesh_bridge_send_onoff(addr, false);
      }
      last_send = now;
    }
  }

  void control_light_hsl(uint16_t addr, float state, float hue,
                         float saturation, uint32_t &last_send,
                         uint16_t max_level = 50) {
    if (!this->init_done_) {
      ESP_LOGW(TAG, "Mesh not ready, skipping control_light_hsl");
      return;
    }

    uint32_t now = millis();
    if (now - last_send > 100 || state == 0 || state == 1.0) {
      uint16_t lightness = 0;
      if (state > 0) {
        lightness = (uint16_t)(state * max_level);
      }

      // HSL Mapping:
      // Hue: 0-360 -> 0-65535 (standard Mesh)
      // Sat: 0-1.0 -> 0-65535
      // Light: 0-1.0 -> 0-65535 (independent of the level scaling?)
      // Actually, Lighting HSL model has Lightness field.
      // Usually "Level" controls the "Light Lightness Actual" state, but HSL
      // message has its own lightness field.
      // For Ledvance, likely the L field in HSL message works.

      uint16_t h_u16 = (uint16_t)(hue / 360.0f * 65535);
      uint16_t s_u16 = (uint16_t)(saturation * 65535);
      uint16_t l_u16 = (uint16_t)(state * 65535); // Full range lightness

      // Note: Some lamps ignore L in HSL Set and use Level/Lightness Set
      // separately. We'll send HSL Set which includes L.
      ble_mesh_bridge_send_hsl(addr, l_u16, h_u16, s_u16);

      if (state == 0) {
        ble_mesh_bridge_send_onoff(addr, false);
      }
      last_send = now;
    }
  }

protected:
  bool init_done_ = false;
};

} // namespace ble_mesh_gateway
} // namespace esphome
