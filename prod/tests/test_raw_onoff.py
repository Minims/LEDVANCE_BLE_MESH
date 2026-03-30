#!/usr/bin/env python3
"""Test raw OnOff command via ESPHome service API.

Usage: python test_raw_onoff.py <lamp_addr> <on|off> [ack|noack]
  lamp_addr: 4 for Julie, 5 for Lilou
  Example: python test_raw_onoff.py 4 on ack
"""
import asyncio
import sys
from aioesphomeapi import APIClient

DEVICE_IP = "192.168.1.212"
API_PORT = 6053
NOISE_PSK = "KbisAPXx5i9m3gzPsa4h3qX4mVUKPbds47kQYfdSuKE="

async def main():
    addr = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    action = sys.argv[2] if len(sys.argv) > 2 else "on"
    ack_mode = sys.argv[3] if len(sys.argv) > 3 else "ack"

    state = 1 if action.lower() == "on" else 0
    use_ack = 1 if ack_mode.lower() == "ack" else 0

    client = APIClient(DEVICE_IP, API_PORT, password=None, noise_psk=NOISE_PSK)
    print(f"Connecting to {DEVICE_IP}...")
    await client.connect(login=True)

    info = await client.device_info()
    print(f"Connected to: {info.name}")

    # Find the set_mesh_onoff service
    entities, services = await client.list_entities_services()
    onoff_svc = None
    for svc in services:
        if "onoff" in svc.name.lower():
            onoff_svc = svc
            break

    if not onoff_svc:
        print("ERROR: set_mesh_onoff service not found!")
        print(f"Available services: {[s.name for s in services]}")
        await client.disconnect()
        return

    print(f"Service found: {onoff_svc.name} (key={onoff_svc.key})")
    print(f">>> Sending OnOff {'ON' if state else 'OFF'} to 0x{addr:04X} ({'ACK' if use_ack else 'NO ACK'})...")

    await client.execute_service(onoff_svc, {"address": addr, "state": state, "use_ack": use_ack})

    await asyncio.sleep(2)
    await client.disconnect()
    print("Done.")

if __name__ == "__main__":
    asyncio.run(main())
