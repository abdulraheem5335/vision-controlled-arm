/*
 * Vision-Controlled Robotic Arm — ESP32 UDP Firmware
 *
 * Receives absolute servo angles over UDP as compact binary packets.
 * Applies slew-rate limiting so servos move smoothly without gear damage.
 *
 * Packet format (5 bytes):
 *   [0xAA] [base 0-180] [updown 0-180] [clipper 0-180] [XOR checksum]
 *   Checksum = base ^ updown ^ clipper
 *
 * Network:
 *   ESP32 creates WiFi AP  →  "VisionArm" / "vision2026"
 *   Listens on UDP port 4210
 *   AP IP is always 192.168.4.1
 *
 * Pins:
 *   D13 → Base    (MG996R)
 *   D12 → Up/Down (MG90S)
 *   D14 → Clipper  (MG90S)
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// ---- Network config ----
const char* AP_SSID     = "VisionArm";
const char* AP_PASSWORD = "vision2026";
const uint16_t UDP_PORT = 4210;

// ---- Servo pins ----
const int PIN_BASE    = 13;
const int PIN_UPDOWN  = 12;
const int PIN_CLIPPER = 14;

// ---- Mechanical limits (protect the arm) ----
const int BASE_MIN = 20,  BASE_MAX = 160;
const int ARM_MIN  = 30,  ARM_MAX  = 150;
const int GRIP_MIN = 10,  GRIP_MAX = 100;

// ---- Slew rate: max degrees per tick ----
// At 50 Hz servo update (20 ms tick), 5°/tick = 250°/sec max speed.
// Keeps movements smooth and prevents current spikes.
const int MAX_SLEW = 5;

// ---- Protocol ----
const uint8_t START_BYTE  = 0xAA;
const int     PACKET_SIZE = 5;

// ---- Objects ----
Servo servoBase, servoUpDown, servoClipper;
WiFiUDP udp;

// ---- State ----
int currentBase    = 90;
int currentUpDown  = 90;
int currentClipper = 10;   // start closed (safe default)

int targetBase     = 90;
int targetUpDown   = 90;
int targetClipper  = 10;

unsigned long lastTickMs = 0;
const unsigned long TICK_MS = 20;  // 50 Hz servo update rate

unsigned long lastPacketMs = 0;    // watchdog: when did we last hear from laptop?
const unsigned long TIMEOUT_MS = 2000;  // if no packet for 2s, hold position

bool servosAttached = false;

// ---- Helpers ----
int clampAngle(int angle, int lo, int hi) {
  return max(lo, min(hi, angle));
}

int slewStep(int current, int target) {
  int diff = target - current;
  if (diff > MAX_SLEW)       return current + MAX_SLEW;
  else if (diff < -MAX_SLEW) return current - MAX_SLEW;
  else                       return target;
}

void attachServos() {
  if (servosAttached) return;

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  servoBase.setPeriodHertz(50);
  servoUpDown.setPeriodHertz(50);
  servoClipper.setPeriodHertz(50);

  servoBase.attach(PIN_BASE, 500, 2400);
  servoUpDown.attach(PIN_UPDOWN, 500, 2400);
  servoClipper.attach(PIN_CLIPPER, 500, 2400);

  // Write safe starting positions
  servoBase.write(currentBase);
  servoUpDown.write(currentUpDown);
  servoClipper.write(currentClipper);

  servosAttached = true;
  Serial.println("Servos attached.");
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Vision Arm UDP Firmware ===");

  // Create WiFi Access Point
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("AP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("AP IP:   ");
  Serial.println(WiFi.softAPIP());

  // Start UDP listener
  udp.begin(UDP_PORT);
  Serial.print("UDP port: ");
  Serial.println(UDP_PORT);
  Serial.println("Waiting for packets...");
}

// ---- Main loop ----
void loop() {
  // --- Read incoming UDP packets ---
  int packetSize = udp.parsePacket();
  if (packetSize == PACKET_SIZE) {
    uint8_t buf[PACKET_SIZE];
    udp.read(buf, PACKET_SIZE);

    if (buf[0] == START_BYTE) {
      uint8_t base    = buf[1];
      uint8_t updown  = buf[2];
      uint8_t clipper = buf[3];
      uint8_t chk     = buf[4];

      // Verify checksum
      if (chk == (base ^ updown ^ clipper)) {
        // Attach servos on first valid packet
        if (!servosAttached) attachServos();

        // Apply mechanical limits and set targets
        targetBase    = clampAngle(base,    BASE_MIN, BASE_MAX);
        targetUpDown  = clampAngle(updown,  ARM_MIN,  ARM_MAX);
        targetClipper = clampAngle(clipper, GRIP_MIN, GRIP_MAX);

        lastPacketMs = millis();
      } else {
        Serial.println("BAD CHECKSUM");
      }
    }
  }

  // --- Slew-rate limited servo update at 50 Hz ---
  unsigned long now = millis();
  if (now - lastTickMs >= TICK_MS) {
    lastTickMs = now;

    if (servosAttached) {
      currentBase    = slewStep(currentBase,    targetBase);
      currentUpDown  = slewStep(currentUpDown,  targetUpDown);
      currentClipper = slewStep(currentClipper, targetClipper);

      servoBase.write(currentBase);
      servoUpDown.write(currentUpDown);
      servoClipper.write(currentClipper);
    }
  }

  // --- Debug: print position every 500ms ---
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 500) {
    lastPrint = now;
    if (servosAttached) {
      Serial.printf("cur: B=%3d U=%3d C=%3d  |  tgt: B=%3d U=%3d C=%3d\n",
        currentBase, currentUpDown, currentClipper,
        targetBase, targetUpDown, targetClipper);
    }

    // Connection watchdog
    if (servosAttached && (now - lastPacketMs > TIMEOUT_MS)) {
      Serial.println("No packets — holding position");
    }
  }
}
