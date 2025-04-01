import asyncio
import XInput
from bleak import BleakClient, BleakScanner

# BLE Setup
DEVICE_NAME = "CJJ"
CHARACTERISTIC_UUID = "00000001-5EC4-4083-81CD-A10B8D5CF6EC"

# Scale analog input to -100..100
def scale(value):
    scaled = max(min(int((value / 32767) * 100), 100), -100)
    return 0 if abs(scaled) < 5 else scaled  # Deadzone

# Find BLE device
async def find_device():
    print("🔍 Scanning for BLE devices...")
    devices = await BleakScanner.discover(timeout=5.0)
    for device in devices:
        print(f"🔎 Found: {device.name} @ {device.address}")
        if device.name and DEVICE_NAME in device.name:
            print(f"✅ Matched: {device.name} at {device.address}")
            return device.address
    print("❌ Device not found.")
    return None

# Send command to BLE
async def send_command(client, command):
    try:
        await client.write_gatt_char(CHARACTERISTIC_UUID, command.encode())
    except Exception as e:
        print(f"❌ Failed to send: {e}")

# Controller input loop
async def joystick_loop(client):
    print("🎮 Xbox Controller Active. Press START to exit.")
    last_command = ""

    try:
        while True:
            state = XInput.get_state(0)

            # ✅ Forward = +100, Reverse = -100
            y = scale(state.Gamepad.sThumbLY)   # Left stick Y
            x = scale(state.Gamepad.sThumbRX)   # Right stick X

            command = f"x={x},y={y}"
            if command != last_command:
                await send_command(client, command)
                print(command)
                last_command = command

            # Vibration based on Y-axis intensity
            intensity = abs(y) / 100
            XInput.set_vibration(0, intensity, intensity)

            # Exit on START (hex 0x0010)
            if state.Gamepad.wButtons & 0x0010:
                print("🛑 START pressed — exiting.")
                break

            await asyncio.sleep(0.005)
    except KeyboardInterrupt:
        print("👋 Keyboard exit.")
    finally:
        XInput.set_vibration(0, 0, 0)
        print("🧹 Controller loop ended.")

# Main BLE + controller logic
async def main():
    await asyncio.sleep(1)
    address = await find_device()
    if not address:
        return

    try:
        async with BleakClient(address, timeout=20.0) as client:
            print("🔗 Connected to CJJ!")
            await client.start_notify(CHARACTERISTIC_UUID, lambda s, d: None)
            await joystick_loop(client)
    except Exception as e:
        print(f"❌ BLE connection error: {e}")

if __name__ == "__main__":
    asyncio.run(main())
