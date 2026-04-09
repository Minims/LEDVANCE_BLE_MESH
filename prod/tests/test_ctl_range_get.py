#!/usr/bin/env python3
"""Test CTL Temperature Range GET on mesh lamps.

Usage: python test_ctl_range_get.py [addr]
  addr: 4 for Julie, 5 for Lilou (default: both)
"""
import asyncio
import sys
from aioesphomeapi import APIClient

DEVICE_IP = "192.168.1.212"
API_PORT = 6053
NOISE_PSK = "KbisAPXx5i9m3gzPsa4h3qX4mVUKPbds47kQYfdSuKE="

async def main():
    addrs = [int(sys.argv[1])] if len(sys.argv) > 1 else [4, 5]

    client = APIClient(DEVICE_IP, API_PORT, password=None, noise_psk=NOISE_PSK)
    print(f"Connecting to {DEVICE_IP}...")
    await client.connect(login=True)

    info = await client.device_info()
    print(f"Connected to: {info.name}")

    def on_log(msg):
        text = msg.message if isinstance(msg.message, str) else msg.message.decode("utf-8", errors="replace")
        if "ble_mesh" in text.lower() or "ctl" in text.lower() or "range" in text.lower() or "timeout" in text.lower():
            print(f"  [ESP] {text}")

    client.subscribe_logs(on_log, log_level=7)

    entities, services = await client.list_entities_services()
    range_svc = None
    for svc in services:
        if "get_ctl_temperature_range" in svc.name.lower():
            range_svc = svc

    if not range_svc:
        print("ERROR: get_ctl_temperature_range service not found!")
        print(f"Available: {[s.name for s in services]}")
        await client.disconnect()
        return

    for addr in addrs:
        name = {4: "Julie", 5: "Lilou"}.get(addr, f"0x{addr:04X}")
        print(f"\n>>> CTL Temperature Range GET on {name} (0x{addr:04X})...")
        await client.execute_service(range_svc, {"address": addr})
        # Wait for response or timeout
        await asyncio.sleep(5)

    await client.disconnect()
    print("\nDone.")

if __name__ == "__main__":
    asyncio.run(main())
