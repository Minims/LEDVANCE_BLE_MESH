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
  static constexpr float CTL_MIN_MIREDS = 153.0f;
  static constexpr float CTL_MAX_MIREDS = 370.0f;
  static constexpr float CTL_MAX_VALUE = 255.0f;
  static constexpr float FLOAT_EPSILON = 0.0001f;

  float get_setup_priority() const override {
    return setup_priority::AFTER_BLUETOOTH;
  }

  // --- ESPHome Component Overrides ---

  void loop() override {
    if (!this->init_done_) {
      if (ble_mesh_bridge_is_ready_to_init()) {
        ESP_LOGI(TAG, "BT Controller Ready. Initializing Mesh Bridge...");
        this->init_done_ = ble_mesh_bridge_init();
        if (!this->init_done_) {
          ESP_LOGW(TAG, "Mesh bridge init failed, will retry...");
        }
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
    ESP_LOGCONFIG(TAG, "  Initialized: %s", YESNO(this->init_done_));
    ESP_LOGCONFIG(TAG, "  Mesh Ready: %s", YESNO(this->is_ready()));
  }

  // --- Public API for YAML Lambdas ---

  bool is_ready() const { return this->init_done_ && ble_mesh_bridge_is_ready(); }

  float clamp_unit_interval(float value) const {
    if (value <= 0.0f)
      return 0.0f;
    if (value >= 1.0f)
      return 1.0f;
    return value;
  }

  float clamp_hue_degrees(float hue) const {
    if (hue < 0.0f)
      return 0.0f;
    if (hue > 360.0f)
      return 360.0f;
    return hue;
  }

  uint16_t ctl_lightness_from_brightness(float brightness) const {
    brightness = this->clamp_unit_interval(brightness);
    if (brightness <= 0.0f)
      return 0;

    uint16_t lightness = (uint16_t) (brightness * CTL_MAX_VALUE);
    if (lightness == 0)
      lightness = 1;
    return lightness;
  }

  uint16_t ctl_temperature_from_mireds(float mireds) const {
    if (mireds < CTL_MIN_MIREDS)
      mireds = CTL_MIN_MIREDS;
    if (mireds > CTL_MAX_MIREDS)
      mireds = CTL_MAX_MIREDS;

    return (uint16_t) (((mireds - CTL_MIN_MIREDS) /
                         (CTL_MAX_MIREDS - CTL_MIN_MIREDS)) * CTL_MAX_VALUE);
  }

  void control_light(uint16_t addr, float state, uint32_t &last_send,
                     uint16_t max_level = 50) {
    if (!this->is_ready()) {
      ESP_LOGW(TAG, "Mesh not ready, skipping control_light");
      return;
    }

    state = this->clamp_unit_interval(state);
    uint32_t now = millis();
    // Rate Limit: Only send level every 100ms prevents buffer flooding
    // Immediate update if turning OFF (0) or MAX (1.0) for responsiveness
    if (now - last_send > 100 || state <= FLOAT_EPSILON ||
        state >= 1.0f - FLOAT_EPSILON) {
      uint16_t level = 0;
      if (state > 0) {
        // SCALING: Map 0-1.0 to 0-max_level
        level = (uint16_t)(state * max_level);
      }

      ble_mesh_bridge_send_level(addr, level);

      // Send explicit OnOff command only when turning off completely
      if (state == 0) {
        ble_mesh_bridge_send_onoff(addr, false, false); // Original was unack
      }
      last_send = now;
    }
  }

  void control_light_hsl(uint16_t addr, float state, float hue,
                         float saturation, uint32_t &last_send,
                         uint16_t max_level = 50) {
    if (!this->is_ready()) {
      ESP_LOGW(TAG, "Mesh not ready, skipping control_light_hsl");
      return;
    }

    state = this->clamp_unit_interval(state);
    saturation = this->clamp_unit_interval(saturation);
    hue = this->clamp_hue_degrees(hue);

    uint32_t now = millis();
    if (now - last_send > 100 || state <= FLOAT_EPSILON ||
        state >= 1.0f - FLOAT_EPSILON) {
      uint16_t l_u16 = 0;
      if (state > 0) {
        // Preserve max_level behavior in HSL mode by scaling the same
        // 0..255-style ceiling into the 16-bit lightness field.
        l_u16 = (uint16_t)(state * max_level * (65535.0f / 255.0f));
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

      // Note: Some lamps ignore L in HSL Set and use Level/Lightness Set
      // separately. We'll send HSL Set which includes L.
      ble_mesh_bridge_send_hsl(addr, l_u16, h_u16, s_u16);

      if (state <= FLOAT_EPSILON) {
        ble_mesh_bridge_send_onoff(addr, false, false); // Original was unack
      }
      last_send = now;
    }
  }

  void control_light_ctl(uint16_t addr, uint16_t lightness, uint16_t temperature,
                         uint32_t &last_send, bool ack = true) {
    if (!this->is_ready()) {
      ESP_LOGW(TAG, "Mesh not ready, skipping control_light_ctl");
      return;
    }

    uint32_t now = millis();
    // Use a longer cooldown for ACK traffic to reduce "Busy" errors.
    uint32_t cooldown = ack ? 250 : 100;

    if (now - last_send > cooldown || lightness == 0) {
      if (lightness > 0) {
        ESP_LOGI(TAG,
                 "[ENVOI ESPHOME CTL] Addr: 0x%04X | Luminosite: %u | "
                 "Temperature: %u",
                 addr, lightness, temperature);
      }
      if (lightness == 0) {
        ble_mesh_bridge_send_onoff(addr, false, ack);
      } else {
        ble_mesh_bridge_send_ctl(addr, lightness, temperature, ack);
      }
      last_send = now;
    }
  }

  void request_ranges(uint16_t addr) {
    if (!this->is_ready()) {
      ESP_LOGW(TAG, "Mesh not ready, skipping request_ranges");
      return;
    }
    ble_mesh_bridge_get_ranges(addr);
    ESP_LOGI(TAG,
             "=> Requêtes GET (Limites internes) envoyées à l'adresse 0x%04X",
             addr);
  }

protected:
  bool init_done_ = false;
};

} // namespace ble_mesh_gateway
} // namespace esphome
