LEDVANCE ESP BLE Mesh Client Model
========================

This code can be used to program an ESP board to act as an intermediate between Home Assistant and LEDVANCE Bluetooth bulbs.
It supports On/Off, HSL color and setting the brightness level.

It is based on the On-Off client example from the [ESP idf examples](https://github.com/espressif/esp-idf/tree/a5b261f/examples/bluetooth/esp_ble_mesh/onoff_models/onoff_server).

## 🚀 Easy Installation (Web Flasher)

The easiest way to install is using the web-based installer. No software required.

1.  **Open the Web Flasher:**
    Go to: `https://ultraworg.github.io/LEDVANCE_BLE_MESH/`

2.  **Connect your Device:**
    Plug your ESP32, ESP32-C3, ESP32-C6, or ESP32-S3 into your computer via USB.

3.  **Flash Firmware:**
    - Select your chip type from the dropdown menu.
    - Click **"Install"**.
    - Select the correct USB port and follow the on-screen instructions.
    - Holding the boot button on the ESP might be required before hitting install

---

## ⚙️ Initial Configuration

After flashing (via Web or Manual method), the device needs to be connected to your network and MQTT broker.

### 1. Wi-Fi Setup
On the first boot (or if the configured Wi-Fi is unavailable), the device will enter **Setup Mode**.

1.  Look for a Wi-Fi Hotspot named **`LEDVANCE_Setup`** on your phone or computer and connect to it.
2.  A configuration page should open automatically. If not, visit `http://192.168.4.1` in your browser.
3.  Enter your home **SSID** and **Password**.
4.  Click **Connect**. The device will restart and connect to your local network.

### 2. MQTT Setup
Once the device is connected to your Wi-Fi:

1.  Find the device's IP address (check your router's client list or the serial monitor).
2.  Open your browser and navigate to `http://<DEVICE_IP>/`.
3.  Click the **System Configuration** button at the top of the page.
4.  Enter your **MQTT Broker URL** (e.g., `mqtt://192.168.1.50:1883`), **Username**, and **Password**.
5.  (Optional) Click **Test Connection** to verify your settings.
6.  Click **Save & Restart**.

---

## 🛠️ Advanced: Manual Flashing & Development

If you prefer command-line tools or want to modify the code, you can use the methods below.

### Option A: Flash Pre-built Binaries with esptool
You can download the latest compiled firmware from GitHub Actions and flash it manually using `esptool.py`.

1.  Go to the **Actions** tab in this repository.
2.  Click on the latest successful workflow run.
3.  Scroll down to **Artifacts** and download the `firmware-<chip>` zip file for your device (e.g., `firmware-esp32c3`).
4.  Extract the zip file to find `merged-binary.bin`.
5.  Flash it to offset `0x0`:

    ```bash
    # Install esptool if you haven't already
    pip install esptool

    # Flash (Replace PORT with your serial port, e.g., /dev/ttyUSB0 or COM3)
    esptool.py -p PORT -b 460800 write_flash 0x0 merged-binary.bin
    ```

### Option B: Build from Source (ESP-IDF)
To modify the code and build locally:

1.  **Install ESP-IDF:** Follow the [official Espressif installation guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).
2.  **Clone the Repo:**
    ```bash
    git clone [https://github.com/](https://github.com/)<YOUR-USERNAME>/<REPO-NAME>.git
    cd <REPO-NAME>
    ```
3.  **Build and Flash:**
    ```bash
    # Set your target chip (esp32, esp32c3, esp32s3, esp32c6)
    idf.py set-target esp32c3

    # Build the firmware
    idf.py build

    # Flash and monitor serial output
    idf.py flash monitor
    ```

---

## 📝 Usage Instructions

### 1. Provision the Lamps
Before using the gateway, you must provision your Ledvance lamps using the [nRF Mesh App](https://www.nordicsemi.com/Products/Development-tools/nrf-mesh/getstarted).

1.  **Reset Lamps:** Ensure lamps are in pairing mode (usually by toggling power 5 times).
2.  **Provision:** Use the **nRF Mesh App** (Android/iOS) to provision each lamp.
3.  **Assign App Keys:**
    - In the App, go to the lamp's "Elements".
    - Bind the same **Application Key** to these models:
        - Generic OnOff Server
        - Generic Level Server
        - Light Lightness Server
        - Light HSL Server
4.  **Record Address:** Note down the **Unicast Address** (e.g., `0x0002`) for each lamp.

### 2. Provision the ESP Gateway
1.  **Provision:** Use the nRF Mesh App to provision the ESP device itself.
2.  **Bind Keys:** Bind the **same Application Key** (used for lamps) to:
        - Generic OnOff Client
        - Generic Level Client
        - Light Lightness Client
        - Light HSL Client

### 3. Add Lamps to Gateway
1.  Open the ESP's web interface (`http://<DEVICE_IP>/`).
2.  Enter the **Name** and **Unicast Address** (from Step 1) for each lamp.
3.  Click "Add Lamp".
4.  **Restart:** Click "Restart Device" or cycle power to apply the new subscriptions.

---

## 🏠 Home Assistant Integration

The gateway automatically publishes discovery messages to Home Assistant.
Once the entity appears in Home Assistant, it should work.
