import asyncio
import platform
from bleak import BleakClient, BleakScanner

# BLE UUIDs
SERVICE_UUID = "00000000-5EC4-4083-81CD-A10B8D5CF6EC"
CHARACTERISTIC_UUID = "00000001-5EC4-4083-81CD-A10B8D5CF6EC"
DEVICE_NAME = "CJJ"

# PID values
pid_values = {"kp": 0.0, "ki": 0.0, "kd": 0.0}
selected_param = None  # Active tuning param (kp/ki/kd)
KeyboardInterrupt
def notification_handler(sender, data):
    """Handles BLE notifications."""
    response = data.decode("utf-8")
    print(f"🔹 Received: {response}")

async def find_device():
    """Scan for NanoBLE device."""
    print("🔍 Scanning for devices...")
    devices = await BleakScanner.discover()
    for device in devices:
        if device.name and DEVICE_NAME in device.name:
            print(f"✅ Found: {device.name} [{device.address}]")
            return device.address
    print("❌ Device not found. Make sure it's powered on and advertising.")
    return None

async def send_command(client, command):
    """Send a command over BLE."""
    await client.write_gatt_char(CHARACTERISTIC_UUID, command.encode())
    print(f"📡 Sent: {command}")

async def pid_terminal(client):
    """Interactive terminal for PID tuning."""
    global selected_param
    print("\n🎛️ Type a command:\n"
          "- 'kp', 'ki', or 'kd' to select a parameter\n"
          "- '=' to increase selected value by 0.1\n"
          "- '-' to decrease selected value by 0.1\n"
          "- Type a number (e.g. 2.3) to assign that value\n"
          "- 's' to show current PID values\n"
          "- 'q' to quit\n")

    while True:
        cmd = input(">>> ").strip().lower()

        if cmd == "q":
            print("👋 Exiting.")
            break

        elif cmd in ["kp", "ki", "kd"]:
            selected_param = cmd
            print(f"✅ Selected {selected_param.upper()} for tuning.")

        elif cmd == "=" and selected_param:
            pid_values[selected_param] += 0.1
            await send_command(client, f"{selected_param}={pid_values[selected_param]:.1f}")

        elif cmd == "-" and selected_param:
            pid_values[selected_param] -= 0.1
            await send_command(client, f"{selected_param}={pid_values[selected_param]:.1f}")

        elif cmd == "s":
            await send_command(client, "s")

        elif selected_param:
            try:
                value = float(cmd)
                pid_values[selected_param] = value
                await send_command(client, f"{selected_param}={pid_values[selected_param]:.1f}")
            except ValueError:
                print("❌ Invalid input. Type a number, '=', '-', or valid command.")
        else:
            print("❓ Unknown command. Select 'kp', 'ki', or 'kd' first.")
            

async def main():
    """Main BLE connection + control loop."""
    device_address = await find_device()
    if not device_address:
        return

    async with BleakClient(device_address) as client:
        print("🔗 Connected to NanoBLE!")
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
        await send_command(client, "s")  # Get initial PID values
        await pid_terminal(client)

if __name__ == "__main__":
    asyncio.run(main())
KeyboardInterrupt