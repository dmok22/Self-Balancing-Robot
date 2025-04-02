import asyncio
import pygame
from bleak import BleakClient, BleakScanner

# BLE UUIDs
SERVICE_UUID = "00000000-5EC4-4083-81CD-A10B8D5CF6EC"
CHARACTERISTIC_UUID = "00000001-5EC4-4083-81CD-A10B8D5CF6EC"
DEVICE_NAME = "CJJ"

# PID and ST values
tuneable_values = {"kp": 0.0, "ki": 0.0, "kd": 0.0, "md": 0.0, "sp": 0.0}
selected_param = None

def notification_handler(sender, data):
    response = data.decode("utf-8")
    print(f"🔹 Received: {response}")

async def find_device():
    print("🔍 Scanning for devices...")
    devices = await BleakScanner.discover()
    for device in devices:
        if device.name and DEVICE_NAME in device.name:
            print(f"✅ Found: {device.name} [{device.address}]")
            return device.address
    print("❌ Device not found.")
    return None

async def send_command(client, command):
    try:
        await client.write_gatt_char(CHARACTERISTIC_UUID, command.encode())
        print(f"📡 Sent: {command}")
    except Exception as e:
        print(f"❌ Failed to send '{command}': {e}")

async def pid_terminal(client):
    global selected_param
    print("\n🎛️ Type a command:\n"
          "- 'kp', 'ki', 'kd', 'md', or 'sp' to select a parameter\n"
          "- '=' to increase selected value by 0.1\n"
          "- '-' to decrease selected value by 0.1\n"
          "- number (e.g. 2.3) to assign that value\n"
          "- 's' to show current values\n"
          "- 'q' to quit\n")

    while True:
        cmd = input(">>> ").strip().lower()

        if cmd == "q":
            print("👋 Exiting terminal.")
            break

        elif cmd in tuneable_values:
            selected_param = cmd
            print(f"✅ Selected {selected_param.upper()} for tuning.")

        elif cmd == "=" and selected_param:
            tuneable_values[selected_param] += 0.1
            await send_command(client, f"{selected_param}={tuneable_values[selected_param]:.3f}")

        elif cmd == "-" and selected_param:
            tuneable_values[selected_param] -= 0.1
            await send_command(client, f"{selected_param}={tuneable_values[selected_param]:.3f}")

        elif cmd == "s":
            await send_command(client, "s")

        elif selected_param:
            try:
                value = float(cmd)
                tuneable_values[selected_param] = value
                await send_command(client, f"{selected_param}={value:.1f}")
            except ValueError:
                print("❌ Invalid input.")
        else:
            print("❓ Unknown command. Select 'kp', 'ki', 'kd', 'md', or 'sp' first.")

async def read_joystick(client):
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("❌ No joystick found.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"🎮 Using controller: {joystick.get_name()}")

    while True:
        pygame.event.pump()
        lx = joystick.get_axis(0)  # Left stick X
        ly = -joystick.get_axis(1)  # Invert Y for natural up = forward

        # Deadzone filter
        if abs(lx) < 0.1: lx = 0.0
        if abs(ly) < 0.1: ly = 0.0

        # Scale to -100..100
        lx_scaled = int(lx * 100)
        ly_scaled = int(ly * 100)

        # Send over BLE
        await send_command(client, f"lx={lx_scaled}")
        await send_command(client, f"ly={ly_scaled}")

        await asyncio.sleep(0.1)  # ~10 updates/sec

async def main():
    device_address = await find_device()
    if not device_address:
        return

    async with BleakClient(device_address) as client:
        print("🔗 Connected to NanoBLE!")
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
        await send_command(client, "s")  # Request initial values

        # Run terminal and joystick reader in parallel
        await asyncio.gather(
            pid_terminal(client),
            read_joystick(client)
        )

if __name__ == "__main__":
    asyncio.run(main())