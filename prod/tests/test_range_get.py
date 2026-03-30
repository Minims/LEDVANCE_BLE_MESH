#!/usr/bin/env python3
"""Query lightness range from a mesh lamp via ESPHome service API.
Subscribes to logs to capture the response.

Usage: python test_range_get.py <lamp_addr>
  lamp_addr: 4 for Julie, 5 for Lilou
"""
import asyncio
import sys
from aioesphomeapi import APIClient

DEVICE_IP = "192.168.1.212"
API_PORT = 6053
NOISE_PSK = "KbisAPXx5i9m3gzPsa4h3qX4mVUKPbds47kQYfdSuKE="

async def main():
    addr = int(sys.argv[1]) if len(sys.argv) > 1 else 5

    client = APIClient(DEVICE_IP, API_PORT, password=None, noise_psk=NOISE_PSK)
    print(f"Connecting to {DEVICE_IP}...")
    await client.connect(login=True)

    info = await client.device_info()
    print(f"Connected to: {info.name}")

    # Subscribe to logs to capture the range response
    def on_log(msg):
        text = msg.message if isinstance(msg.message, str) else msg.message.decode("utf-8", errors="replace")
        if "ble_mesh" in text.lower() or "range" in text.lower() or "lightness" in text.lower():
            print(f"  [ESP LOG] {text}")

    client.subscribe_logs(on_log, log_level=7)  # 7 = DEBUG

    entities, services = await client.list_entities_services()
    range_svc = None
    for svc in services:
        if "lightness_range" in svc.name.lower():
            range_svc = svc
            break

    if not range_svc:
        print("ERROR: get_lightness_range service not found!")
        await client.disconnect()
        return

    print(f">>> Sending Lightness Range GET to 0x{addr:04X}...")
    await client.execute_service(range_svc, {"address": addr})

    # Wait for response
    print("Waiting 5s for response...")
    await asyncio.sleep(5)

    await client.disconnect()
    print("Done.")

if __name__ == "__main__":
    asyncio.run(main())
