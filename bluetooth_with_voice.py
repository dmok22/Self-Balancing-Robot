import asyncio
import os
import XInput
import speech_recognition as sr
from google.cloud import speech
from bleak import BleakClient, BleakScanner
import threading

# Set up Google Cloud credentials
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = (
    "C:/Users/berty/Downloads/elec391/speech-to-text-key.json"  # Location of the speech-to-text-key.json file
)

# BLE Setup
DEVICE_NAME = "CJJ"
CHARACTERISTIC_UUID = "00000001-5EC4-4083-81CD-A10B8D5CF6EC"

# Voice command mapping
voice_command_mapping = {
    "forward": "y=50",
    "backward": "y=-50",
    "left": "x=-50",
    "right": "x=50",
    "stop": "x=0,y=0",
}


# Scale analog input to -100..100
def scale(value):
    scaled = max(min(int((value / 32767) * 100), 100), -100)
    return 0 if abs(scaled) < 5 else scaled  # Deadzone


# Notification handler for BLE responses
def notification_handler(sender, data):
    response = data.decode("utf-8")
    print(f"🔹 Received: {response}")


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
        print(f"📡 Sent: {command}")
        return True
    except Exception as e:
        print(f"❌ Failed to send: {e}")
        return False


# Voice recognition function
async def voice_recognition_loop(client, voice_active):
    recognizer = sr.Recognizer()

    print("🎤 Voice recognition activated")

    while voice_active.is_set():
        with sr.Microphone() as source:
            print("🎤 Listening for commands...")
            recognizer.adjust_for_ambient_noise(source)
            try:
                audio = recognizer.listen(source, timeout=5, phrase_time_limit=5)
                print("🔊 Processing audio...")

                # Use Google Cloud Speech-to-Text
                speech_client = speech.SpeechClient()
                audio_data = sr.AudioData(
                    audio.get_wav_data(), source.SAMPLE_RATE, source.SAMPLE_WIDTH
                )
                response = speech_client.recognize(
                    config=speech.RecognitionConfig(
                        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
                        sample_rate_hertz=source.SAMPLE_RATE,
                        language_code="en-US",
                    ),
                    audio=speech.RecognitionAudio(content=audio_data.get_raw_data()),
                )

                if not response.results:
                    print("❓ No speech detected")
                    continue

                recognized_text = response.results[0].alternatives[0].transcript.lower()
                print(f"🗣️ Recognized: '{recognized_text}'")

                # Check for commands in the recognized text
                for voice_cmd, ble_cmd in voice_command_mapping.items():
                    if voice_cmd in recognized_text:
                        print(f"✅ Executing command: {voice_cmd}")
                        await send_command(client, ble_cmd)

                        # For stop command, ensure everything is stopped
                        if voice_cmd == "stop":
                            await asyncio.sleep(0.1)
                            await send_command(client, "x=0,y=0")
                        break
                else:
                    print("❌ Command not recognized")

            except sr.WaitTimeoutError:
                print("⏱️ Listening timeout, restarting...")
            except sr.UnknownValueError:
                print("❓ Could not understand audio")
            except sr.RequestError as e:
                print(f"🔥 Speech recognition error: {e}")
            except Exception as e:
                print(f"⚠️ Error: {e}")

        # Small delay before listening again
        await asyncio.sleep(0.5)

    print("🎤 Voice recognition deactivated")


# Controller input loop
async def joystick_loop(client):
    print("🎮 Xbox Controller Active. Press START to exit.")
    last_command = ""
    mode_toggle = 0
    x_button_last = 0

    left_signal = 0
    right_signal = 0
    lb_last = 0
    rb_last = 0

    headlight = 0
    y_button_last = 0

    # Create and set the voice recognition flag
    voice_active = threading.Event()
    voice_active.clear()  # Start with voice recognition off
    voice_task = None

    try:
        while True:
            state = XInput.get_state(0)

            # Joystick scaling
            y = scale(state.Gamepad.sThumbLY)
            x = scale(state.Gamepad.sThumbRX)

            # Send x/y only if changed
            command = f"x={x},y={y}"
            if command != last_command:
                await send_command(client, command)
                print(command)
                last_command = command

            # Vibrate based on y intensity
            intensity = abs(y) / 100
            XInput.set_vibration(0, intensity, intensity)

            # Toggle mode on X button (0x0400)
            x_button_now = state.Gamepad.wButtons & 0x400
            if x_button_now and not x_button_last:
                mode_toggle = 1 - mode_toggle
                await send_command(client, f"mode={mode_toggle}")
                print(f"🔁 Mode toggled: mode={mode_toggle}")
            x_button_last = x_button_now

            # Toggle voice recognition on BACK button (0x0020)
            back_button_now = state.Gamepad.wButtons & 0x0020
            if back_button_now and not back_button_last:
                if voice_active.is_set():
                    voice_active.clear()
                    print("🔇 Voice recognition disabled")
                    if voice_task:
                        voice_task.cancel()
                else:
                    voice_active.set()
                    print("🎙️ Voice recognition enabled")
                    voice_task = asyncio.create_task(
                        voice_recognition_loop(client, voice_active)
                    )
            back_button_last = back_button_now

            # Toggle left signal on LB (0x0100)
            lb_now = state.Gamepad.wButtons & 0x0100
            if lb_now and not lb_last:
                left_signal = 1 - left_signal
                await send_command(client, f"left_signal={left_signal}")
                print(f"🟨 Left signal toggled: {left_signal}")
            lb_last = lb_now

            # Toggle right signal on RB (0x0200)
            rb_now = state.Gamepad.wButtons & 0x0200
            if rb_now and not rb_last:
                right_signal = 1 - right_signal
                await send_command(client, f"right_signal={right_signal}")
                print(f"🟧 Right signal toggled: {right_signal}")
            rb_last = rb_now

            # Toggle headlight on Y button (0x0800)
            y_button_now = state.Gamepad.wButtons & 0x0800
            if y_button_now and not y_button_last:
                headlight = 1 - headlight
                await send_command(client, f"headlight={headlight}")
                print(f"💡 Headlight toggled: {headlight}")
            y_button_last = y_button_now

            # Exit on START (0x0010)
            if state.Gamepad.wButtons & 0x0010:
                print("🛑 START pressed — exiting.")
                voice_active.clear()
                if voice_task:
                    voice_task.cancel()
                break

            await asyncio.sleep(0.005)

    except KeyboardInterrupt:
        print("👋 Keyboard exit.")
    finally:
        XInput.set_vibration(0, 0, 0)
        voice_active.clear()
        if voice_task:
            voice_task.cancel()
        print("🧹 Controller loop ended.")


# Standalone voice control mode
async def standalone_voice_mode(client):
    print("🎤 Standalone Voice Control Mode")
    print("Press Ctrl+C to exit")

    voice_active = threading.Event()
    voice_active.set()

    try:
        await voice_recognition_loop(client, voice_active)
    except KeyboardInterrupt:
        print("👋 Keyboard exit.")
    finally:
        voice_active.clear()
        print("🧹 Voice control ended.")


# BLE + controller main
async def main():
    await asyncio.sleep(1)
    address = await find_device()
    if not address:
        return

    try:
        async with BleakClient(address, timeout=20.0) as client:
            print("🔗 Connected to CJJ!")
            await client.start_notify(CHARACTERISTIC_UUID, notification_handler)

            # Check if controller is available
            try:
                XInput.get_state(0)
                print("🎮 Xbox controller detected")
                await joystick_loop(client)
            except XInput.XInputNotConnected:
                print("🎮 No Xbox controller detected, using voice control only")
                await standalone_voice_mode(client)

    except Exception as e:
        print(f"❌ BLE connection error: {e}")


if __name__ == "__main__":
    asyncio.run(main())
