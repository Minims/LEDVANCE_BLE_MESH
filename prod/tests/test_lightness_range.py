#!/usr/bin/env python3
"""Test raw lightness values on a mesh lamp to find working range.

Usage: python test_lightness_range.py <lamp_addr> <level> [ack|noack]
       python test_lightness_range.py <lamp_addr> sweep [ack|noack]
  
  lamp_addr: 4 for Julie, 5 for Lilou
  level: raw lightness value (0-65535) or 'sweep' to test all key values
  
Examples:
  python test_lightness_range.py 5 sweep noack    # Sweep Lilou NO ACK
  python test_lightness_range.py 5 1000 ack       # Set Lilou to 1000 ACK
"""
import asyncio
import sys
from aioesphomeapi import APIClient

DEVICE_IP = "192.168.1.212"
API_PORT = 6053
NOISE_PSK = "KbisAPXx5i9m3gzPsa4h3qX4mVUKPbds47kQYfdSuKE="

async def main():
    addr = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    mode = sys.argv[2] if len(sys.argv) > 2 else "sweep"
    ack_mode = sys.argv[3] if len(sys.argv) > 3 else "noack"
    use_ack = 1 if ack_mode.lower() == "ack" else 0

    client = APIClient(DEVICE_IP, API_PORT, password=None, noise_psk=NOISE_PSK)
    print(f"Connecting to {DEVICE_IP}...")
    await client.connect(login=True)

    info = await client.device_info()
    print(f"Connected to: {info.name}")

    # Subscribe to logs
    def on_log(msg):
        text = msg.message if isinstance(msg.message, str) else msg.message.decode("utf-8", errors="replace")
        if "ble_mesh" in text.lower() or "lightness" in text.lower():
            print(f"  [ESP] {text}")

    client.subscribe_logs(on_log, log_level=7)

    entities, services = await client.list_entities_services()
    lightness_svc = None
    onoff_svc = None
    for svc in services:
        if "set_mesh_lightness" in svc.name.lower():
            lightness_svc = svc
        if "set_mesh_onoff" in svc.name.lower():
            onoff_svc = svc

    if not lightness_svc:
        print("ERROR: set_mesh_lightness service not found!")
        await client.disconnect()
        return

    ack_label = "ACK" if use_ack else "NO ACK"

    if mode == "sweep":
        # First turn on the lamp
        if onoff_svc:
            print(f"\n>>> Turning ON 0x{addr:04X} first...")
            await client.execute_service(onoff_svc, {"address": addr, "state": 1, "use_ack": use_ack})
            await asyncio.sleep(2)

        levels = [1, 10, 50, 100, 255, 500, 1000, 5000, 10000, 32768, 65535]
        print(f"\n=== Sweeping lightness on 0x{addr:04X} ({ack_label}) ===")
        print(f"Values: {levels}")
        print("Watch the lamp and note when brightness changes!\n")

        for level in levels:
            print(f">>> Level {level} ({ack_label})...")
            await client.execute_service(lightness_svc, {"address": addr, "level": level, "use_ack": use_ack})
            await asyncio.sleep(3)

        # Turn off
        print(f"\n>>> Turning OFF 0x{addr:04X}...")
        await client.execute_service(lightness_svc, {"address": addr, "level": 0, "use_ack": use_ack})
    else:
        level = int(mode)
        print(f"\n>>> Setting lightness {level} on 0x{addr:04X} ({ack_label})...")
        await client.execute_service(lightness_svc, {"address": addr, "level": level, "use_ack": use_ack})

    await asyncio.sleep(2)
    await client.disconnect()
    print("Done.")

if __name__ == "__main__":
    asyncio.run(main())
