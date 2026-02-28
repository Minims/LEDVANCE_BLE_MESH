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

// Send commands
void ble_mesh_bridge_send_onoff(uint16_t addr, bool state);
void ble_mesh_bridge_send_level(uint16_t addr, uint16_t level);
void ble_mesh_bridge_send_generic_level(uint16_t addr, int16_t level);
void ble_mesh_bridge_send_hsl(uint16_t addr, uint16_t lightness, uint16_t hue,
                              uint16_t saturation);

#ifdef __cplusplus
}
#endif
