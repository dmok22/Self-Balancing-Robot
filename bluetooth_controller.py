import asyncio
import XInput
from bleak import BleakClient, BleakScanner
import os
import speech_recognition as sr
from google.cloud import speech
import secrets
import string
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import tkinter as tk
from tkinter import simpledialog
from tkinter import messagebox

# BLE Setup
DEVICE_NAME = "CJJ"
CHARACTERISTIC_UUID = "00000001-5EC4-4083-81CD-A10B8D5CF6EC"

# Google Cloud credentials
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = r"C:\Users\Jeffrey He\Desktop\UBC\ELEC391\balancing_code\speech-to-text-key.json"

# Voice command setup
recognizer = sr.Recognizer()
valid_commands = ["forward", "stop", "left", "right"]


def scale(value):
    scaled = max(min(int((value / 32767) * 100), 100), -100)
    return 0 if abs(scaled) < 5 else scaled  # Deadzone


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


async def send_command(client, command):
    try:
        await client.write_gatt_char(CHARACTERISTIC_UUID, command.encode())
    except Exception as e:
        print(f"❌ Failed to send: {e}")


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
    a_button_last = 0

    b_button_last = 0

    prev_lt = -1
    prev_rt = -1

    voice_recognition_active = False

    try:
        while True:
            state = XInput.get_state(0)

            # Joystick movement
            y = scale(state.Gamepad.sThumbLY)
            x = scale(state.Gamepad.sThumbRX)
            command = f"x={x},y={y}"
            if command != last_command:
                await send_command(client, command)
                print(command)
                last_command = command

            # Vibration feedback
            intensity = abs(y) / 100
            XInput.set_vibration(0, intensity, intensity)

            # Triggers
            lt_val = state.Gamepad.bLeftTrigger
            rt_val = state.Gamepad.bRightTrigger
            if lt_val != prev_lt:
                await send_command(client, f"lt={lt_val}")
                prev_lt = lt_val
            if rt_val != prev_rt:
                await send_command(client, f"rt={rt_val}")
                prev_rt = rt_val

            # X button toggles mode
            x_button_now = state.Gamepad.wButtons & 0x4000
            if x_button_now and not x_button_last:
                mode_toggle = 1 - mode_toggle
                await send_command(client, f"mode={mode_toggle}")
                print(f"🔁 Mode toggled: mode={mode_toggle}")
            x_button_last = x_button_now

            # LB and RB toggle signals
            lb_now = state.Gamepad.wButtons & 0x0100
            if lb_now and not lb_last:
                left_signal = 1 - left_signal
                await send_command(client, f"left_signal={left_signal}")
                print(f"🟨 Left signal toggled: {left_signal}")
            lb_last = lb_now

            rb_now = state.Gamepad.wButtons & 0x0200
            if rb_now and not rb_last:
                right_signal = 1 - right_signal
                await send_command(client, f"right_signal={right_signal}")
                print(f"🟧 Right signal toggled: {right_signal}")
            rb_last = rb_now

            # Y button toggles headlight
            y_button_now = state.Gamepad.wButtons & 0x8000
            if y_button_now and not y_button_last:
                headlight = 1 - headlight
                await send_command(client, f"headlight={headlight}")
                print(f"💡 Headlight toggled: {headlight}")
            y_button_last = y_button_now

            # A button triggers voice recognition
            a_button_now = state.Gamepad.wButtons & 0x1000
            if a_button_now and not a_button_last:
                if voice_recognition_active:
                    print("🎙️ Voice recognition deactivated.")
                    voice_recognition_active = False
                else:
                    print("🎙️ Voice recognition activated.")
                    voice_recognition_active = True
                    await recognize_speech_and_send(client)
            a_button_last = a_button_now

            # ✅ B button triggers one-time sonar scan
            b_button_now = state.Gamepad.wButtons & 0x2000
            if b_button_now and not b_button_last:
                await send_command(client, "scan=1")
                print("📡 One-time sonar sweep triggered")
            b_button_last = b_button_now

            # START exits
            if state.Gamepad.wButtons & 0x0010:
                print("🛑 START pressed — exiting.")
                break

            await asyncio.sleep(0.005)

    except KeyboardInterrupt:
        print("👋 Keyboard exit.")
    finally:
        XInput.set_vibration(0, 0, 0)
        print("🧹 Controller loop ended.")


async def recognize_speech_and_send(client):
    with sr.Microphone() as source:
        print("Listening for commands...")
        recognizer.adjust_for_ambient_noise(source)

        while True:
            audio = recognizer.listen(source)

            try:
                client_speech = speech.SpeechClient()
                audio_data = sr.AudioData(audio.get_wav_data(), source.SAMPLE_RATE, source.SAMPLE_WIDTH)
                response = client_speech.recognize(
                    config=speech.RecognitionConfig(
                        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
                        sample_rate_hertz=source.SAMPLE_RATE,
                        language_code="en-US",
                    ),
                    audio=speech.RecognitionAudio(content=audio_data.get_raw_data()),
                )

                for result in response.results:
                    recognized_text = result.alternatives[0].transcript.lower()
                    print(f"🎤 Recognized: {recognized_text}")

                    for word in recognized_text.split():
                        if word in valid_commands:
                            print(f"✅ Executing: {word}")
                            await send_command(client, word)
                            return
                    else:
                        print("❌ Command not recognized.")

            except sr.UnknownValueError:
                print("🤷 Could not understand audio.")
            except sr.RequestError as e:
                print(f"❌ Google STT error: {e}")


def get_user_input(prompt):
    root = tk.Tk()
    root.withdraw()
    return simpledialog.askstring("Input", prompt)


def generate_random_number_password(length=6):
    return "".join(secrets.choice(string.digits) for _ in range(length))


def send_email(subject, message, to_email):
    from_email = "20040418jeff@gmail.com"
    password = "bxmu rzjz ppah gjsd"
    msg = MIMEMultipart()
    msg["From"] = from_email
    msg["To"] = to_email
    msg["Subject"] = subject
    msg.attach(MIMEText(message, "plain"))
    with smtplib.SMTP("smtp.gmail.com", 587) as server:
        server.starttls()
        server.login(from_email, password)
        server.sendmail(from_email, to_email, msg.as_string())


def verify_password():
    password = generate_random_number_password()
    email = get_user_input("Enter your email to receive a one-time password:")
    send_email("Oven Controller Password", f"Your password is: {password}", email)
    for i in range(3, 0, -1):
        user_input = get_user_input(f"Enter the password sent to your email (Attempts left: {i}):")
        if user_input == password:
            messagebox.showinfo("Success", "Password correct!")
            return True
        else:
            messagebox.showerror("Error", "Incorrect password.")
    messagebox.showerror("Error", "All attempts used. Please restart.")
    return False


# BLE connection + controller loop
async def main():
    # if not verify_password(): return
    await asyncio.sleep(1)
    address = await find_device()
    if not address:
        return

    def handle_notify(sender, data):
        msg = data.decode("utf-8")
        if msg.startswith("sonar_"):
            print(f"📡 {msg}")

    try:
        async with BleakClient(address, timeout=20.0) as client:
            print("🔗 Connected to CJJ!")
            await client.start_notify(CHARACTERISTIC_UUID, handle_notify)
            await joystick_loop(client)
    except Exception as e:
        print(f"❌ BLE connection error: {e}")


if __name__ == "__main__":
    asyncio.run(main())
