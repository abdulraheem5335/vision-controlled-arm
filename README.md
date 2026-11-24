# Vision-Controlled Robotic Arm

Real-time teleoperation system for a 3-DOF robotic arm using MediaPipe hand tracking on CPU and low-latency UDP binary streaming over WiFi to an ESP32.

---

## Demo Video

[Watch Demo Video](YOUR_VIDEO_LINK_HERE)

---

## System Overview

```
[Laptop Webcam]
       │
       ▼ (30 FPS)
[MediaPipe HandLandmarker] (Runs on CPU, no GPU required)
       │
       ▼ (Palm X/Y + Pinch Distance)
[EMA Filter + Deadband Gate (±2°)] (Filters hand tremor and prevents servo hum)
       │
       ▼ (5-byte Binary Packet: [0xAA, Base, UpDown, Clipper, Checksum])
[UDP Transmitter]
       │
       ▼ (Local WiFi ~1-3 ms latency, ESP32 AP: "VisionArm")
[ESP32 DevKit V1]
       │
       ▼ (Slew-rate limiter at 50 Hz PWM)
[3x Servos: MG996R Base, MG90S Arm, MG90S Clipper]
```

---

## Gesture to Servo Mapping

| Joint | Servo | Pin | Gesture | Action | Safe Limits |
| :--- | :--- | :--- | :--- | :--- | :--- |
| Base | MG996R | D13 | Hand Left / Right | Mirrored palm X position | 20° to 160° |
| Up / Down | MG90S | D12 | Hand Up / Down | Palm Y position | 30° to 150° |
| Clipper | MG90S | D14 | Thumb + Index Pinch | Euclidean distance between fingertips | 10° (closed) to 100° (open) |

---

## Hardware and Wiring

| Component | Signal Pin | Power Connection | Ground |
| :--- | :--- | :--- | :--- |
| Base Servo (MG996R) | ESP32 D13 | External 5V 2A Rail | Common GND |
| Arm Servo (MG90S) | ESP32 D12 | External 5V 2A Rail | Common GND |
| Clipper Servo (MG90S) | ESP32 D14 | External 5V 2A Rail | Common GND |
| ESP32 DevKit V1 | - | USB or 5V VIN | Common GND |

> **Power Notice:** Do not power servos from the ESP32 onboard regulator pins. Servos draw high stall currents and can cause brownout resets. Power the servos through a dedicated external 5V power supply and connect all grounds together.

---

## Project Structure

```
vision-controlled-arm/
├── firmware/
│   └── firmware.ino          # Non-blocking ESP32 UDP firmware with slew-rate limiting
├── vision/
│   ├── hand_tracker.py       # MediaPipe Tasks pipeline with EMA smoothing and gesture math
│   ├── transmitter.py        # Non-blocking UDP binary transmitter (5-byte packets)
│   ├── run.py                # Main vision controller with resizable OpenCV window
│   ├── mock_esp32.py         # Local UDP receiver mock for testing without hardware
│   └── hand_landmarker.task  # Pre-trained MediaPipe Tasks model asset
├── requirements.txt          # Python dependencies
└── README.md
```

---

## Quickstart

### 1. Python Environment Setup

```bash
git clone https://github.com/abdulraheem5335/vision-controlled-arm.git
cd vision-controlled-arm

python3 -m venv venv
source venv/bin/activate

pip install -r requirements.txt
```

If you need to re-download the model asset manually:
```bash
wget -q -O vision/hand_landmarker.task https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
```

### 2. Flash ESP32 Firmware

1. Open `firmware/firmware.ino` in Arduino IDE.
2. Select Board: **ESP32 Dev Module**.
3. Install the **ESP32Servo** library (`Sketch -> Include Library -> Manage Libraries`).
4. Connect the ESP32 via USB and click **Upload**.
5. Open Serial Monitor at **115200 baud** to confirm initialization:
   ```text
   === Vision Arm UDP Firmware ===
   AP SSID: VisionArm
   AP IP:   192.168.4.1
   UDP port: 4210
   Waiting for packets...
   ```

### 3. Connect and Run

1. Connect your laptop to the WiFi network created by the ESP32:
   - **SSID:** `VisionArm`
   - **Password:** `vision2026`
2. Start the vision controller:
   ```bash
   cd vision
   python3 run.py --ip 192.168.4.1
   ```
3. A resizable OpenCV display will open. Move your hand to control the arm. Press `q` to exit.

---

## Testing Without Hardware

You can test the entire software stack without physical servos connected:

### Vision Only (Camera HUD)
```bash
python3 vision/run.py
```

### Dry Run (Terminal Packet Inspector)
```bash
python3 vision/run.py --dry-run
```

### Localhost UDP Loopback
Run the mock receiver in one terminal and send packets from another:
```bash
# Terminal 1: Mock ESP32 listener
python3 vision/mock_esp32.py

# Terminal 2: Vision transmitter directed to localhost
python3 vision/run.py --ip 127.0.0.1
```

---

## Communication Protocol

### 5-Byte Binary Datagram

```
Byte 0: Start Byte    (0xAA)
Byte 1: Base Angle    (0 - 180)
Byte 2: UpDown Angle  (0 - 180)
Byte 3: Clipper Angle (0 - 180)
Byte 4: Checksum      (Base ^ UpDown ^ Clipper)
```

- **Transport:** UDP over 802.11 WiFi
- **Design:** Fire-and-forget datagrams eliminate TCP head-of-line blocking so stale frames never buffer.
- **Latency Budget:** Under 55 ms total end-to-end response time.
