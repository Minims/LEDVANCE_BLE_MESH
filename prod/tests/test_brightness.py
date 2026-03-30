#!/usr/bin/env python3
"""Test brightness range for mesh lights via ESPHome API.

Tests various brightness levels to find the working range.
Usage: python test_brightness.py <lamp> <level_pct>
       python test_brightness.py <lamp> sweep   # sweeps 10,25,50,75,100%
"""
import asyncio
import sys
import time
from aioesphomeapi import APIClient

DEVICE_IP = "192.168.1.212"
API_PORT = 6053
NOISE_PSK = "KbisAPXx5i9m3gzPsa4h3qX4mVUKPbds47kQYfdSuKE="
LOG_DIR = "prod/logs"

async def main():
    target = sys.argv[1] if len(sys.argv) > 1 else "julie"
    mode = sys.argv[2] if len(sys.argv) > 2 else "sweep"

    client = APIClient(DEVICE_IP, API_PORT, password=None, noise_psk=NOISE_PSK)
    print(f"Connecting to {DEVICE_IP}...")
    await client.connect(login=True)
    
    info = await client.device_info()
    print(f"Connected to: {info.name}")
    
    entities, services = await client.list_entities_services()
    lights = [e for e in entities if hasattr(e, 'key') and hasattr(e, 'name') and 'light' in type(e).__name__.lower()]
    
    light = None
    for l in lights:
        if target.lower() in l.name.lower():
            light = l
            break
    
    if not light:
        print(f"Light '{target}' not found!")
        await client.disconnect()
        return
    
    print(f"Target: {light.name} (key={light.key})")
    
    if mode == "sweep":
        levels = [5, 10, 25, 50, 75, 100]
        print(f"\n=== Sweeping brightness levels: {levels}% ===")
        for pct in levels:
            brightness = pct / 100.0
            print(f"\n>>> Setting {light.name} to {pct}% (brightness={brightness:.2f})...")
            client.light_command(key=light.key, state=True, brightness=brightness)
            await asyncio.sleep(3)
        
        print(f"\n>>> Turning OFF {light.name}...")
        client.light_command(key=light.key, state=False)
    else:
        pct = int(mode)
        brightness = pct / 100.0
        print(f"\n>>> Setting {light.name} to {pct}% (brightness={brightness:.2f})...")
        client.light_command(key=light.key, state=True, brightness=brightness)
    
    await asyncio.sleep(1)
    await client.disconnect()
    print("Done.")

if __name__ == "__main__":
    asyncio.run(main())
