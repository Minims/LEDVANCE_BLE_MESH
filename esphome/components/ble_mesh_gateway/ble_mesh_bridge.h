#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the BLE Mesh Stack (and NVS, Bluetooth)
void ble_mesh_bridge_init(void);

// Check if controller is ready
bool ble_mesh_bridge_is_ready_to_init(void);

// Send commands (use_ack: true = SET with response, false = SET_UNACK)
void ble_mesh_bridge_send_onoff(uint16_t addr, bool state, bool use_ack);
void ble_mesh_bridge_send_level(uint16_t addr, uint16_t level, bool use_ack);
void ble_mesh_bridge_send_hsl(uint16_t addr, uint16_t lightness, uint16_t hue,
                              uint16_t saturation, bool use_ack);
void ble_mesh_bridge_send_ctl(uint16_t addr, uint16_t lightness,
                              uint16_t temperature, int16_t delta_uv,
                              bool use_ack);

// Range queries
void ble_mesh_bridge_send_lightness_range_get(uint16_t addr);
void ble_mesh_bridge_send_ctl_temperature_range_get(uint16_t addr);

// Callback for range status responses
typedef void (*ble_mesh_lightness_range_cb_t)(uint16_t addr, uint16_t range_min,
                                              uint16_t range_max);
void ble_mesh_bridge_set_lightness_range_callback(ble_mesh_lightness_range_cb_t cb);

typedef void (*ble_mesh_ctl_temp_range_cb_t)(uint16_t addr, uint16_t range_min,
                                             uint16_t range_max);
void ble_mesh_bridge_set_ctl_temp_range_callback(ble_mesh_ctl_temp_range_cb_t cb);

#ifdef __cplusplus
}
#endif
