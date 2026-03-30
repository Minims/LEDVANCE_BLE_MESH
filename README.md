# LEDVANCE ESP BLE Mesh Gateway

Control **LEDVANCE Bluetooth Mesh bulbs** from [Home Assistant](https://home-assistant.io) using an ESP32 as a gateway.

Supports: **On/Off**, **Brightness**, **HSL Color**. ACK and NO ACK modes configurable per lamp.

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
      CONFIG_BT_ENABLED: "y"
      CONFIG_BLE_MESH_SETTINGS: "y"
```

### Adding Lamps

#### Option A: Inline YAML (requires ESP recompilation for new lamps)

Create template outputs for each lamp using its unicast address:

```yaml
output:
  - platform: template
    id: mesh_output_lamp1
    type: float
    write_action:
      - lambda: |-
          static uint32_t last = 0;
          id(mesh_gateway).control_light(0x0020, state, last, 50);

light:
  - platform: monochromatic
    name: "Living Room Lamp"
    output: mesh_output_lamp1
    gamma_correct: 1.0
    default_transition_length: 0s
```

#### Option B: Service-Based (recommended - no ESP recompilation)

Add services to your ESPHome config once:

```yaml
api:
  services:
    # Control brightness (monochromatic lamp)
    # address: unicast address (decimal), brightness: 0-100, max_level: e.g. 50
    # use_ack: optional, pass true as 5th arg for ACK-only lamps
    - service: set_mesh_light
      variables:
        address: int
        brightness: int
        max_level: int
      then:
        - lambda: |-
            static std::map<uint16_t, uint32_t> last_sends;
            float bright_f = brightness / 100.0f;
            id(mesh_gateway).control_light(address, bright_f, last_sends[address], max_level);

    # Control HSL color lamp
    - service: set_mesh_light_hsl
      variables:
        address: int
        brightness: int
        hue: int
        saturation: int
        max_level: int
      then:
        - lambda: |-
            static std::map<uint16_t, uint32_t> last_sends;
            float bright_f = brightness / 100.0f;
            float hue_f = (float)hue;
            float sat_f = saturation / 100.0f;
            id(mesh_gateway).control_light_hsl(address, bright_f, hue_f, sat_f, last_sends[address], max_level);

    # Raw OnOff command (useful for ACK-only lamps)
    # state: 0=OFF, 1=ON | use_ack: 0=NO ACK (default), 1=ACK
    - service: set_mesh_onoff
      variables:
        address: int
        state: int
        use_ack: int
      then:
        - lambda: |-
            ble_mesh_bridge_send_onoff((uint16_t)address, state != 0, use_ack != 0);
```

Then define lamps in Home Assistant's `configuration.yaml` - see [`esphome/homeassistant_example.yaml`](esphome/homeassistant_example.yaml).

**Parameters for `control_light` / `control_light_hsl`:**
- `address` — Lamp's unicast address (decimal, e.g., 20 = 0x0014)
- `brightness` — 0-100 (percent), converted to 0.0-1.0 internally
- `hue` — 0-360 (for HSL)
- `saturation` — 0-100 (for HSL)
- `max_level` — Max lightness value sent to lamp. Start with `50` for LEDVANCE, adjust based on observed range
- `use_ack` *(optional, default false)* — Set `true` for ACK-only lamps (e.g., some LEDVANCE bulbs that ignore UNACK)

### HSL Color Control

For color-changing lamps, use `control_light_hsl`:

```yaml
- lambda: |-
    static uint32_t last = 0;
    id(mesh_gateway).control_light_hsl(0x0020, brightness, hue, saturation, last, 50);
```

### Example Configs

See [`esphome/gateway.yaml`](esphome/gateway.yaml) (ESP32) and [`esphome/gateway-c6.yaml`](esphome/gateway-c6.yaml) (ESP32-C6).

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
3. **Bind Application Key** to these models:
   - Generic OnOff Server
   - Generic Level Server  
   - Light Lightness Server
   - Light HSL Server (for color)
4. **Note the Unicast Address** (e.g., `0x0002`)
5. **Provision ESP gateway** and bind same App Key to:
   - Generic OnOff Client
   - Generic Level Client
   - Light Lightness Client

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
