#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the BLE Mesh Stack (and NVS, Bluetooth)
bool ble_mesh_bridge_init(void);

// Check if controller is ready
bool ble_mesh_bridge_is_ready_to_init(void);
bool ble_mesh_bridge_is_ready(void);

// Send commands
void ble_mesh_bridge_send_onoff(uint16_t addr, bool state, bool ack);
void ble_mesh_bridge_send_level(uint16_t addr, uint16_t level);
void ble_mesh_bridge_send_hsl(uint16_t addr, uint16_t lightness, uint16_t hue,
                              uint16_t saturation);
void ble_mesh_bridge_send_ctl(uint16_t addr, uint16_t lightness, uint16_t temperature,
                              bool ack);
void ble_mesh_bridge_get_ranges(uint16_t addr);

#ifdef __cplusplus
}
#endif
