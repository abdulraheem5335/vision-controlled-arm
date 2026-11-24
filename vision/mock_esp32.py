"""
Mock ESP32 UDP receiver for testing.

Run this on your laptop to simulate the ESP32 receiving servo angle packets.
It listens on 0.0.0.0:4210 and prints every valid packet.

Usage:
  Terminal 1:  python mock_esp32.py
  Terminal 2:  python run.py --dry-run   (or --ip 127.0.0.1 for live UDP test)
"""
import socket
import struct
import sys

PORT = 4210
START_BYTE = 0xAA
PACKET_SIZE = 5


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT))
    print(f"Mock ESP32 listening on UDP port {PORT}")
    print("Waiting for packets... (Ctrl+C to quit)\n")

    count = 0
    try:
        while True:
            data, addr = sock.recvfrom(64)

            if len(data) != PACKET_SIZE:
                print(f"[WARN] Bad packet size ({len(data)} bytes) from {addr}")
                continue

            start, base, updown, clipper, chk = struct.unpack("5B", data)

            if start != START_BYTE:
                print(f"[WARN] Bad start byte: 0x{start:02X}")
                continue

            expected_chk = base ^ updown ^ clipper
            if chk != expected_chk:
                print(f"[WARN] Checksum mismatch: got 0x{chk:02X}, expected 0x{expected_chk:02X}")
                continue

            count += 1
            print(
                f"[{count:5d}] Base={base:3d}  UpDown={updown:3d}  "
                f"Clipper={clipper:3d}  from {addr[0]}"
            )

    except KeyboardInterrupt:
        print(f"\nReceived {count} valid packets total.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
