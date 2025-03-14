import os
import speech_recognition as sr
from google.cloud import speech
import serial

# Set up Google Cloud credentials
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = (
    "C:/Users/berty/Downloads/elec391/speech-to-text-key.json"
)

# Set up the serial connection to the Bluetooth module
bluetooth_port = "COM3"  # Replace with your Bluetooth serial port
baud_rate = 9600  # Common baud rate for Bluetooth modules

# Initialize the serial connection
ser = serial.Serial(bluetooth_port, baud_rate)

# Initialize recognizer
recognizer = sr.Recognizer()

# Define a list of valid commands
valid_commands = ["forward", "stop", "left", "right"]


def recognize_speech():
    with sr.Microphone() as source:
        print("Listening for commands...")
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

        try:
            # Use Google Cloud Speech-to-Text
            client = speech.SpeechClient()
            audio_data = sr.AudioData(
                audio.get_wav_data(), source.SAMPLE_RATE, source.SAMPLE_WIDTH
            )
            response = client.recognize(
                config=speech.RecognitionConfig(
                    encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
                    sample_rate_hertz=source.SAMPLE_RATE,
                    language_code="en-US",
                ),
                audio=speech.RecognitionAudio(content=audio_data.get_raw_data()),
            )

            for result in response.results:
                recognized_text = result.alternatives[0].transcript.lower()
                # print(f"Recognized command: {recognized_text}")

                # Split the recognized text into words
                words = recognized_text.split()

                for word in words:
                    # Check if the recognized text is a valid command
                    if word in valid_commands:
                        print(f"Executing command: {word}")
                        if recognized_text == "forward":
                            ser.write(b"F")
                        elif recognized_text == "stop":
                            ser.write(b"S")
                        elif recognized_text == "left":
                            ser.write(b"L")
                        elif recognized_text == "right":
                            ser.write(b"R")
                        # Here you can add the logic to execute the command
                    else:
                        print("Command not recognized.")

        except sr.UnknownValueError:
            print("Could not understand the audio.")
        except sr.RequestError as e:
            print(f"Google Speech Recognition error; {e}")
        finally:
            # Close the serial connection
            ser.close()


if __name__ == "__main__":
    recognize_speech()
