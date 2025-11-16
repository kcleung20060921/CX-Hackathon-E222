import asyncio
import speech_recognition as sr
from bleak import BleakScanner, BleakClient
import threading
import time

SERVICE_UUID = "12345678-1234-1234-1234-123456789012"
CHARACTERISTIC_UUID = "abcdef12-3456-7890-abcd-ef1234567890"
COMMAND_UUID = "87654321-4321-4321-4321-210987654321"


class VoiceReceiver:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.client = None
        self.loop = None
        self.is_connected = False

        # Speech detection parameters
        self.recognizer.energy_threshold = 4000
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.pause_threshold = 0.8

    async def send_command_to_esp32(self, command):
        """Send command to ESP32"""
        if self.client and self.is_connected:
            try:
                await self.client.write_gatt_char(COMMAND_UUID, command.encode())
                print(f"📤 Sent command to ESP32: {command}")
                return True
            except Exception as e:
                print(f"❌ Failed to send command: {e}")
                return False
        return False

    def listen_for_voice(self):
        """Listen to microphone and recognize speech"""
        try:
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=1)
                print("🎤 Listening for voice commands...\n")

                while self.is_connected:
                    try:
                        print("🎙️  Say a command ('training' or 'repair')...")
                        audio = self.recognizer.listen(source, timeout=15, phrase_time_limit=10)

                        print("🔊 Processing audio...")
                        text = self.recognizer.recognize_sphinx(audio)
                        print(f"✅ You said: '{text}'")
                        self.handle_command(text)

                    except sr.UnknownValueError:
                        print("❓ Could not understand - please speak clearly")
                    except sr.RequestError as e:
                        print(f"❌ Error: {e}")
                    except sr.WaitTimeoutError:
                        print("⏱️ No speech detected, listening again...")

        except Exception as e:
            print(f"❌ Microphone error: {e}")

    def handle_command(self, text):
        """Handle voice commands"""
        text_lower = text.lower()
        print(f"🤖 Processing command: '{text}'")

        if "training" in text_lower:
            print("🎯 Detected 'training' - Sending Mode 1 command...")
            if self.loop:
                asyncio.run_coroutine_threadsafe(self.send_command_to_esp32("mode1"), self.loop)
        elif "repair" in text_lower:
            print("🔧 Detected 'repair' - Sending Mode 2 command...")
            if self.loop:
                asyncio.run_coroutine_threadsafe(self.send_command_to_esp32("mode2"), self.loop)
        else:
            print(f"❓ Command not recognized: '{text}'")
            print("   Supported commands: 'training', 'repair'")

    async def run(self):
        print("💻 ESP32 Voice Command Control")
        print("=" * 50)
        print("🔍 Scanning for ESP32-Thermal-Audio...")

        # Save event loop reference
        self.loop = asyncio.get_running_loop()

        devices = await BleakScanner.discover(timeout=10.0)
        target_device = None

        for device in devices:
            if device.name and "ESP32-Thermal-Audio" in device.name:
                target_device = device
                print(f"✅ Found: {device.name}")
                break

        if not target_device:
            print("❌ ESP32-Thermal-Audio not found")
            print("\nAvailable devices:")
            for device in devices:
                print(f"  - {device.name}")
            return

        try:
            print("🔗 Connecting to ESP32...")
            async with BleakClient(target_device.address) as client:
                self.client = client
                self.is_connected = True
                print("✅ Connected to ESP32!\n")

                print("=" * 50)
                print("🎤 VOICE COMMAND SYSTEM ACTIVE")
                print("=" * 50)
                print("💡 Supported commands:")
                print("   - Say 'training' → Mode 1 (Thermal Monitoring)")
                print("   - Say 'repair'   → Mode 2")
                print("🛑 Press Ctrl+C to stop")
                print("=" * 50 + "\n")

                # Start listening in background thread
                listen_thread = threading.Thread(target=self.listen_for_voice, daemon=True)
                listen_thread.start()

                # Main loop - keep connection alive
                while self.is_connected:
                    await asyncio.sleep(1)

        except KeyboardInterrupt:
            print("\n🛑 Stopped by user")
        except Exception as e:
            print(f"❌ Connection error: {e}")
        finally:
            self.is_connected = False
            self.client = None
            print("👋 Disconnected")


if __name__ == "__main__":
    receiver = VoiceReceiver()
    try:
        asyncio.run(receiver.run())
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")