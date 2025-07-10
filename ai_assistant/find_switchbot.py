import asyncio
from switchbot import GetSwitchbotDevices

async def main():
    scanner = GetSwitchbotDevices()  # optionally: GetSwitchbotDevices(interface=0)
    devices = await scanner.discover()

    if not devices:
        print("❌ No SwitchBot devices found.")
    else:
        for addr, adv in devices.items():
            print(f"✅ Found device: {addr} - model: {adv.data.get('model')} - rssi: {adv.data.get('rssi')}")


asyncio.run(main())
