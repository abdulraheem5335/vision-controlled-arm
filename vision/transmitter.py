"""
UDP transmitter for the Vision-Controlled Arm.

Sends absolute servo angles to the ESP32 as compact binary packets.

Packet format (5 bytes):
  [0xAA] [base] [updown] [clipper] [XOR checksum]

Modes:
  - Live:    sends real UDP datagrams to ESP32_IP:4210
  - Dry-run: prints what would be sent (no network needed)
"""
import socket
import struct
import time
from dataclasses import dataclass
from typing import Optional


START_BYTE = 0xAA
DEFAULT_IP = "192.168.4.1"   # ESP32 AP mode default
DEFAULT_PORT = 4210


class Transmitter:
    """Sends servo angles to the ESP32 over UDP.

    Args:
        esp32_ip:   IP address of the ESP32. None = dry-run mode.
        port:       UDP port (must match firmware).
        min_interval_ms: Minimum time between sends (rate limit).
                         At 30 FPS video, 33 ms is a good default.
    """

    def __init__(
        self,
        esp32_ip: Optional[str] = None,
        port: int = DEFAULT_PORT,
        min_interval_ms: float = 33.0,
    ):
        self.dry_run = esp32_ip is None
        self._addr = (esp32_ip or "0.0.0.0", port)
        self._min_interval = min_interval_ms / 1000.0
        self._last_send_time = 0.0
        self._sock: Optional[socket.socket] = None

        if not self.dry_run:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Non-blocking: fire-and-forget (we don't wait for ACK on UDP)
            self._sock.setblocking(False)

    # -------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------
    def send(self, base: int, updown: int, clipper: int) -> bool:
        """Send absolute servo angles to the ESP32.

        Args:
            base:    0–180 (will be clamped)
            updown:  0–180 (will be clamped)
            clipper: 0–180 (will be clamped)

        Returns:
            True if a packet was actually sent (or printed in dry-run),
            False if rate-limited (too soon since last send).
        """
        now = time.monotonic()
        if now - self._last_send_time < self._min_interval:
            return False

        b = self._clamp(base)
        u = self._clamp(updown)
        c = self._clamp(clipper)
        checksum = b ^ u ^ c

        packet = struct.pack("5B", START_BYTE, b, u, c, checksum)

        if self.dry_run:
            print(
                f"[DRY-RUN] Base={b:3d}  UpDown={u:3d}  "
                f"Clipper={c:3d}  pkt={packet.hex()}"
            )
        else:
            try:
                self._sock.sendto(packet, self._addr)
            except BlockingIOError:
                pass  # non-blocking, OK to drop

        self._last_send_time = now
        return True

    def close(self):
        """Release the UDP socket."""
        if self._sock:
            self._sock.close()
            self._sock = None

    # -------------------------------------------------------------------
    # Private
    # -------------------------------------------------------------------
    @staticmethod
    def _clamp(val: int) -> int:
        return max(0, min(180, int(round(val))))
