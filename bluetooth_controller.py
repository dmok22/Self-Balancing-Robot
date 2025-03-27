import asyncio
import pygame
import pyxinput  # Windows-only: Xbox controller interface
from bleak import BleakClient, BleakScanner

# BLE Setup
DEVICE_NAME = "CJJ"
CHARACTERISTIC_UUID = "00000001-5EC4-4083-81CD-A10B8D5CF6EC"

# Connect to first Xbox controller via XInput
def get_xinput_controller():
    try:
        return pyxinput.vController()
    except Exception as e:
        print("❌ Could not initialize XInput controller:", e)
        return None

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
    xinput = get_xinput_controller()
    if not joystick or not xinput:
        return

    try:
        while True:
            pygame.event.pump()

            # Quit if "Back" button is pressed (button 6)
            if joystick.get_button(6):
                print("🛑 Quit button pressed.")
                break

            # Forward/back = left stick Y (axis 1)
            # Left/right = right stick X (axis 2)
            forward = -joystick.get_axis(1)
            turn = joystick.get_axis(2)

            # Deadzone filtering
            deadzone = 0.05
            forward = 0 if abs(forward) < deadzone else forward
            turn = 0 if abs(turn) < deadzone else turn

            # Trigger vibration at extreme turn
            if abs(turn) > 0.95:
                xinput.set_vibration(0.5, 0.5)  # (left_motor, right_motor)
            else:
                xinput.set_vibration(0, 0)

            # Send command to Arduino
            command = f"x={turn:.2f},y={forward:.2f}"
            await send_command(client, command)

            await asyncio.sleep(0.05)  # 20Hz
    except KeyboardInterrupt:
        print("👋 Stopping via keyboard interrupt.")
    finally:
        xinput.set_vibration(0, 0)  # Ensure vibration stops
        pygame.quit()
        print("🧹 Cleaned up and exited.")

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
