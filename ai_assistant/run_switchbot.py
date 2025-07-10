import asyncio
from bleak import BleakClient

# Replace with your bot's MAC
MAC = "C7:6A:01:46:3E:2C"

# Press command
PRESS_COMMAND = bytearray([0x57, 0x01, 0x00])
CHAR_UUID = "cba20002-224d-11e6-9fb8-0002a5d5c51b"  # SwitchBot BLE write characteristic

async def press_bot():
    try:
        async with BleakClient(MAC) as client:
            await client.write_gatt_char(CHAR_UUID, PRESS_COMMAND)
            print("✅ SwitchBot pressed via Bluetooth!")
    except Exception as e:
        print(f"❌ Failed to press bot: {e}")

asyncio.run(press_bot())



