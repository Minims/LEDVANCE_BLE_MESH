# LEDVANCE ESP BLE Mesh Gateway

Control **LEDVANCE Bluetooth Mesh bulbs** from [Home Assistant](https://home-assistant.io) using an ESP32 as a gateway.

Supports: **On/Off**, **Brightness**, **Color Temperature (CTL)**, **HSL Color**. ACK and NO ACK modes configurable per lamp.

---

## 📦 Two Variants

| Variant | Description | Best For |
|---------|-------------|----------|
| **Standalone (ESP-IDF)** | Native ESP-IDF with web UI, MQTT integration | Full-featured self-contained gateway |
| **ESPHome Component** | Integrates directly with ESPHome/Home Assistant | Home Assistant setups with existing ESPHome devices |

---

## 🚀 Quick Start: Web Flasher (Standalone)

The easiest way to install the **standalone** version:

1. Open: **https://ultraworg.github.io/LEDVANCE_BLE_MESH/**
2. Select your ESP32 variant and click **Install**
3. Follow on-screen instructions

See [Standalone Setup](#standalone-setup) below for configuration.

---

## 🏠 ESPHome Component

### Installation

Add to your ESPHome configuration:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/Ultraworg/LEDVANCE_BLE_MESH
      ref: main
    components: [ble_mesh_gateway]
    refresh: 0s

ble_mesh_gateway:
  id: mesh_gateway

esp32:
  board: esp32dev  # or esp32-c6-devkitc-1 for C6
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_BLE_MESH: "y"
      CONFIG_BLE_MESH_NODE: "y"
      CONFIG_BLE_MESH_PB_GATT: "y"
      CONFIG_BLE_MESH_PB_ADV: "y"
      CONFIG_BLE_MESH_GENERIC_CLIENT: "y"
      CONFIG_BLE_MESH_LIGHTING_CLIENT: "y"
      CONFIG_BLE_MESH_GENERIC_ONOFF_CLI: "y"
      CONFIG_BLE_MESH_GENERIC_LEVEL_CLI: "y"
      CONFIG_BLE_MESH_LIGHT_LIGHTNESS_CLI: "y"
      CONFIG_BLE_MESH_LIGHT_CTL_CLI: "y"
      CONFIG_BT_ENABLED: "y"
      CONFIG_BLE_MESH_SETTINGS: "y"
```

### Adding Lamps

#### Color Temperature Lamp (brightness + color temperature)

For LEDVANCE tunable white lamps, use two template outputs (brightness + color temperature) and the `color_temperature` light platform:

```yaml
globals:
  - id: lamp_brightness
    type: float
    initial_value: '0.0'
  - id: lamp_color_temp
    type: float
    initial_value: '0.5'

output:
  # Brightness output — sends Lightness SET + CTL to maintain temperature
  - platform: template
    id: mesh_output_brightness
    type: float
    write_action:
      - lambda: |
          id(lamp_brightness) = state;
          if (state > 0.001f) {
            uint16_t lightness = (uint16_t)(state * 65535.0f);
            float ct = id(lamp_color_temp);
            uint16_t temp_k = 800 + (uint16_t)(ct * 19200.0f);
            ble_mesh_bridge_send_level(0x0005, lightness, false);
            ble_mesh_bridge_send_ctl(0x0005, lightness, temp_k, 0, false);
          } else {
            ble_mesh_bridge_send_onoff(0x0005, false, false);
          }

  # Color temperature output — sends CTL SET
  - platform: template
    id: mesh_output_color_temp
    type: float
    write_action:
      - lambda: |
          id(lamp_color_temp) = state;
          float bright = id(lamp_brightness);
          if (bright > 0.001f) {
            uint16_t lightness = (uint16_t)(bright * 65535.0f);
            uint16_t temp_k = 800 + (uint16_t)(state * 19200.0f);
            ble_mesh_bridge_send_ctl(0x0005, lightness, temp_k, 0, false);
          }

light:
  - platform: color_temperature
    name: "My Lamp"
    color_temperature: mesh_output_color_temp
    brightness: mesh_output_brightness
    cold_white_color_temperature: 6500 K
    warm_white_color_temperature: 2000 K
    gamma_correct: 1.0
    default_transition_length: 0s
```

> **Note on ACK mode**: For ACK-only lamps (e.g., Julie), change `false` to `true` in `send_level()`, `send_ctl()`, and `send_onoff()` calls.

#### Brightness-Only Lamp (monochromatic)

For lamps without color temperature control:

```yaml
output:
  - platform: template
    id: mesh_output_lamp1
    type: float
    write_action:
      - lambda: |-
          static uint32_t last = 0;
          id(mesh_gateway).control_light(0x0020, state, last, 65535);

light:
  - platform: monochromatic
    name: "Living Room Lamp"
    output: mesh_output_lamp1
    gamma_correct: 1.0
    default_transition_length: 0s
```

#### Service-Based Control (no recompilation needed)

Add services to your ESPHome config for dynamic control from Home Assistant:

```yaml
api:
  services:
    # Raw OnOff: state: 0=OFF 1=ON, use_ack: 0=UNACK 1=ACK
    - service: set_mesh_onoff
      variables:
        address: int
        state: int
        use_ack: int
      then:
        - lambda: |-
            ble_mesh_bridge_send_onoff((uint16_t)address, state != 0, use_ack != 0);

    # Raw Lightness SET: level: 0-65535
    - service: set_mesh_lightness
      variables:
        address: int
        level: int
        use_ack: int
      then:
        - lambda: |-
            ble_mesh_bridge_send_level((uint16_t)address, (uint16_t)level, use_ack != 0);

    # Raw CTL SET: lightness: 0-65535, temperature: 800-20000
    - service: set_mesh_ctl
      variables:
        address: int
        lightness: int
        temperature: int
        use_ack: int
      then:
        - lambda: |-
            ble_mesh_bridge_send_ctl((uint16_t)address, (uint16_t)lightness,
                                    (uint16_t)temperature, 0, use_ack != 0);
```

### Bridge API Reference

| Function | Description |
|----------|-------------|
| `ble_mesh_bridge_send_onoff(addr, state, ack)` | ON/OFF control |
| `ble_mesh_bridge_send_level(addr, level, ack)` | Lightness SET (0-65535) |
| `ble_mesh_bridge_send_ctl(addr, lightness, temp, delta_uv, ack)` | CTL SET (lightness + temperature) |
| `ble_mesh_bridge_send_hsl(addr, lightness, hue, sat, ack)` | HSL SET (color) |
| `ble_mesh_bridge_send_lightness_range_get(addr)` | Query lightness range |
| `ble_mesh_bridge_send_ctl_temperature_range_get(addr)` | Query CTL temperature range |

### LEDVANCE-Specific Notes

| Parameter | Range | Notes |
|-----------|-------|-------|
| Lightness | 1 – 65535 | Full 16-bit range, change visible ~5000-10000 |
| CTL Temperature | 800 – 20000 | Internal mesh units (not real Kelvin) |
| ACK mode | per lamp | Some lamps (e.g., Julie) require ACK, others work with NO ACK |
| RANGE_GET | ❌ | LEDVANCE lamps do not respond to range queries |

### Example Configs

See [`prod/esp32-c6.yaml`](prod/esp32-c6.yaml) for a complete production config with two lamps (Julie + Lilou).

---

## ⚙️ Standalone Setup

### Initial Configuration

1. **Wi-Fi Setup**: Connect to **`LEDVANCE_Setup`** hotspot, configure at `http://192.168.4.1`
2. **MQTT Setup**: Navigate to device IP, click **System Configuration**, enter MQTT broker details

### Pre-built Binaries

1. Go to **Actions** tab → download `firmware-<chip>.zip`
2. Flash with: `esptool.py -p PORT write_flash 0x0 merged-binary.bin`

### Build from Source

```bash
idf.py set-target esp32c3
idf.py build
idf.py flash monitor
```

---

## 📝 Provisioning Lamps

Before using either variant, provision your lamps with the **nRF Mesh App**:

1. **Reset lamp** (toggle power 5x)
2. **Provision** with nRF Mesh app
3. **Bind Application Key** to these models on the lamp:
   - Generic OnOff Server
   - Generic Level Server
   - Light Lightness Server
   - Light CTL Server (for color temperature)
   - Light HSL Server (for color)
4. **Note the Unicast Address** (e.g., `0x0005`)
5. **Provision ESP gateway** and bind same App Key to:
   - Generic OnOff Client
   - Generic Level Client
   - Light Lightness Client
   - Light CTL Client (auto-binds if on same element)

---

## 🔧 Supported Hardware

- ESP32 (standard)
- ESP32-C3
- ESP32-C6
- ESP32-S3

---

## 📄 License

MIT License - See [LICENSE](LICENSE) for details.

---

## 🙏 Credits

Based on [ESP-IDF BLE Mesh examples](https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/esp_ble_mesh).
