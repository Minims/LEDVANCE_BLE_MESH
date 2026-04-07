#!/usr/bin/env python3
"""Test CTL (Color Temperature + Lightness) on a mesh lamp.

Usage: python test_ctl.py <lamp_addr> <lightness> <temperature> [ack|noack]
       python test_ctl.py <lamp_addr> sweep [ack|noack]

  lamp_addr: 4 for Julie, 5 for Lilou
  lightness: 0-65535
  temperature: 800-20000 (Kelvin)

Examples:
  python test_ctl.py 5 sweep noack           # Sweep temperatures on Lilou
  python test_ctl.py 5 32768 4000 noack      # Set specific values
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

    if mode == "sweep":
        ack_mode = sys.argv[3] if len(sys.argv) > 3 else "noack"
    else:
        lightness = int(mode)
        temperature = int(sys.argv[3]) if len(sys.argv) > 3 else 4000
        ack_mode = sys.argv[4] if len(sys.argv) > 4 else "noack"

    use_ack = 1 if ack_mode.lower() == "ack" else 0

    client = APIClient(DEVICE_IP, API_PORT, password=None, noise_psk=NOISE_PSK)
    print(f"Connecting to {DEVICE_IP}...")
    await client.connect(login=True)

    info = await client.device_info()
    print(f"Connected to: {info.name}")

    def on_log(msg):
        text = msg.message if isinstance(msg.message, str) else msg.message.decode("utf-8", errors="replace")
        if "ble_mesh" in text.lower() or "ctl" in text.lower() or "light" in text.lower():
            print(f"  [ESP] {text}")

    client.subscribe_logs(on_log, log_level=7)

    entities, services = await client.list_entities_services()
    ctl_svc = None
    onoff_svc = None
    for svc in services:
        if "set_mesh_ctl" in svc.name.lower():
            ctl_svc = svc
        if "set_mesh_onoff" in svc.name.lower():
            onoff_svc = svc

    if not ctl_svc:
        print("ERROR: set_mesh_ctl service not found!")
        print(f"Available: {[s.name for s in services]}")
        await client.disconnect()
        return

    ack_label = "ACK" if use_ack else "NO ACK"

    if mode == "sweep":
        # Turn on first
        if onoff_svc:
            print(f"\n>>> Turning ON 0x{addr:04X}...")
            await client.execute_service(onoff_svc, {"address": addr, "state": 1, "use_ack": use_ack})
            await asyncio.sleep(2)

        # Sweep temperature at fixed lightness (50%)
        L = 32768
        temps = [800, 2700, 3000, 4000, 5000, 6500, 10000, 15000, 20000]
        print(f"\n=== CTL Sweep on 0x{addr:04X} ({ack_label}) ===")
        print(f"Lightness={L}, Temperatures: {temps}")
        print("Watch for color temperature changes!\n")

        for t in temps:
            print(f">>> CTL L={L}, T={t}K ({ack_label})...")
            await client.execute_service(ctl_svc, {"address": addr, "lightness": L, "temperature": t, "use_ack": use_ack})
            await asyncio.sleep(3)

        print(f"\n>>> Turning OFF...")
        if onoff_svc:
            await client.execute_service(onoff_svc, {"address": addr, "state": 0, "use_ack": use_ack})
    else:
        print(f"\n>>> CTL L={lightness}, T={temperature}K on 0x{addr:04X} ({ack_label})...")
        await client.execute_service(ctl_svc, {"address": addr, "lightness": lightness, "temperature": temperature, "use_ack": use_ack})

    await asyncio.sleep(2)
    await client.disconnect()
    print("Done.")

if __name__ == "__main__":
    asyncio.run(main())
