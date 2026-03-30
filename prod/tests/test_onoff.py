#!/usr/bin/env python3
"""Test ON/OFF for Julie (0x0004) and Lilou (0x0005) via ESPHome API."""
import asyncio
import sys
from aioesphomeapi import APIClient

DEVICE_IP = "192.168.1.212"
API_PORT = 6053
NOISE_PSK = "KbisAPXx5i9m3gzPsa4h3qX4mVUKPbds47kQYfdSuKE="

async def main():
    client = APIClient(DEVICE_IP, API_PORT, password=None, noise_psk=NOISE_PSK)
    print(f"Connecting to {DEVICE_IP}...")
    await client.connect(login=True)
    
    info = await client.device_info()
    print(f"Connected to: {info.name} (v{info.esphome_version})")
    
    # List entities to find light keys
    entities, services = await client.list_entities_services()
    lights = [e for e in entities if hasattr(e, 'key') and hasattr(e, 'name') and 'light' in type(e).__name__.lower()]
    
    print(f"\n=== Found {len(lights)} lights ===")
    for l in lights:
        print(f"  - {l.name} (key={l.key})")
    
    target = sys.argv[1] if len(sys.argv) > 1 else "julie"
    action = sys.argv[2] if len(sys.argv) > 2 else "on"
    brightness_pct = int(sys.argv[3]) if len(sys.argv) > 3 else None
    
    light = None
    for l in lights:
        if target.lower() in l.name.lower():
            light = l
            break
    
    if not light:
        print(f"Light '{target}' not found!")
        await client.disconnect()
        return
    
    print(f"\n>>> Target: {light.name} (key={light.key})")
    
    if action.lower() == "on":
        kwargs = {"key": light.key, "state": True}
        if brightness_pct is not None:
            kwargs["brightness"] = brightness_pct / 100.0
            print(f">>> Turning ON {light.name} at {brightness_pct}%...")
        else:
            print(f">>> Turning ON {light.name}...")
        client.light_command(**kwargs)
    elif action.lower() == "off":
        print(f">>> Turning OFF {light.name}...")
        client.light_command(key=light.key, state=False)
    else:
        print(f"Unknown action: {action}")
    
    await asyncio.sleep(1)
    await client.disconnect()
    print("Done.")

if __name__ == "__main__":
    asyncio.run(main())
