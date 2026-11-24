"""
Run the hand tracker on webcam and send servo angles to the ESP32.

Modes:
  python run.py                    → Vision only (no transmission)
  python run.py --dry-run          → Vision + print packets (no network)
  python run.py --ip 192.168.4.1   → Vision + live UDP to ESP32
  python run.py --ip 127.0.0.1    → Vision + UDP to mock_esp32.py on localhost

Controls:
  Move hand left/right  → Base servo
  Move hand up/down     → Up/Down servo
  Pinch thumb + index   → Clipper servo (closed ↔ open)
  Press 'q'             → Quit
"""
import argparse
import cv2
from hand_tracker import HandTracker
from transmitter import Transmitter


def parse_args():
    p = argparse.ArgumentParser(description="Vision-Controlled Arm")
    group = p.add_mutually_exclusive_group()
    group.add_argument(
        "--dry-run", action="store_true",
        help="Print servo packets to terminal (no network)",
    )
    group.add_argument(
        "--ip", type=str, default=None,
        help="ESP32 IP address for live UDP transmission (e.g. 192.168.4.1)",
    )
    p.add_argument(
        "--port", type=int, default=4210,
        help="UDP port (default: 4210)",
    )
    p.add_argument(
        "--camera", type=int, default=0,
        help="Camera index (default: 0)",
    )
    return p.parse_args()


def main():
    args = parse_args()

    # --- Set up transmitter ---
    tx = None
    if args.dry_run:
        tx = Transmitter(esp32_ip=None, port=args.port)
        print("Mode: DRY-RUN (printing packets)")
    elif args.ip:
        tx = Transmitter(esp32_ip=args.ip, port=args.port)
        print(f"Mode: LIVE UDP → {args.ip}:{args.port}")
    else:
        print("Mode: VISION ONLY (no transmission)")

    # --- Set up camera ---
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("ERROR: Cannot open webcam")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # --- Set up tracker ---
    tracker = HandTracker()
    print("Hand tracker running. Press 'q' to quit.")
    print("  Move hand left/right → Base")
    print("  Move hand up/down    → Up/Down")
    print("  Pinch thumb + index  → Clipper")
    print()

    window_name = "Hand Tracking"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 960)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            state, annotated = tracker.process(frame)

            if state.detected:
                # Transmit if we have a transmitter
                if tx:
                    tx.send(state.base_angle, state.updown_angle, state.clipper_angle)

                if not args.dry_run:
                    print(
                        f"\rBase:{state.base_angle:3d}  "
                        f"Up/Down:{state.updown_angle:3d}  "
                        f"Clipper:{state.clipper_angle:3d}  "
                        f"Pinch:{state.pinch:.2f}",
                        end="",
                    )
            else:
                if not args.dry_run:
                    print("\rNo hand detected          ", end="")

            cv2.imshow(window_name, annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        tracker.release()
        if tx:
            tx.close()
        cv2.destroyAllWindows()
        print()


if __name__ == "__main__":
    main()
