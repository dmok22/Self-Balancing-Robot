import asyncio
import pygame
from bleak import BleakClient, BleakScanner

# BLE Setup
DEVICE_NAME = "CJJ"
CHARACTERISTIC_UUID = "00000001-5EC4-4083-81CD-A10B8D5CF6EC"

# Find Arduino BLE device by name ("CJJ")
async def find_device():
    print("🔍 Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    for device in devices:
        if device.name and DEVICE_NAME in device.name:
            print(f"✅ Found {device.name} at {device.address}")
            return device.address
    print("❌ Device not found.")
    return None

# Send BLE command
async def send_command(client, command):
    await client.write_gatt_char(CHARACTERISTIC_UUID, command.encode())
    print(f"📡 Sent: {command}")

# Initialize Xbox controller (via Bluetooth)
def init_joystick():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("❌ No joystick detected.")
        return None

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"🎮 Controller connected: {joystick.get_name()}")
    return joystick

# Main joystick loop
async def joystick_loop(client):
    joystick = init_joystick()
    if not joystick:
        return

    try:
        while True:
            pygame.event.pump()

            x_axis = joystick.get_axis(0)  # Left stick X (turn)
            y_axis = -joystick.get_axis(1)  # Left stick Y (forward/backward)

            # Deadzone filter
            deadzone = 0.05
            x_axis = 0 if abs(x_axis) < deadzone else x_axis
            y_axis = 0 if abs(y_axis) < deadzone else y_axis

            # Format command: "x=0.12,y=0.43"
            command = f"x={x_axis:.2f},y={y_axis:.2f}"
            await send_command(client, command)

            await asyncio.sleep(0.05)  # 20Hz
    except KeyboardInterrupt:
        print("👋 Controller loop stopped.")
    finally:
        pygame.quit()

# Main BLE + control
async def main():
    address = await find_device()
    if not address:
        return

    async with BleakClient(address) as client:
        print("🔗 Connected to CJJ!")
        await client.start_notify(CHARACTERISTIC_UUID, lambda s, d: print(f"🔹 {d.decode()}"))
        await joystick_loop(client)

if __name__ == "__main__":
    asyncio.run(main())
