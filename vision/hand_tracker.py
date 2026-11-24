"""
Hand tracking module using MediaPipe Tasks API.

Gesture-to-servo mapping for a 3-DOF robotic arm:
  Base servo   (MG996R, D13) ← Palm X position  (left/right rotation)
  Up/Down servo (MG90S, D12) ← Palm Y position  (tilt up/down)
  Clipper servo (MG90S, D14) ← Pinch distance   (thumb tip ↔ index tip)

Includes EMA smoothing and deadband gating to prevent servo jitter.
"""
import os
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import numpy as np
import math
from dataclasses import dataclass, field
from typing import Optional, Tuple


# ---------------------------------------------------------------------------
# MediaPipe Hand Landmark indices (21 total)
# ---------------------------------------------------------------------------
WRIST = 0
THUMB_TIP = 4
INDEX_TIP = 8
MIDDLE_MCP = 9


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------
@dataclass
class HandState:
    """Tracking result for a single frame."""
    detected: bool
    # Normalized hand metrics (0..1 range)
    palm_x: float = 0.5       # screen X (left=0, right=1)
    palm_y: float = 0.5       # screen Y (top=0, bottom=1)
    pinch: float = 1.0        # thumb-index distance, 0=closed 1=fully open
    # Smoothed servo angles (ready to transmit)
    base_angle: int = 90      # 0..180
    updown_angle: int = 90    # 0..180
    clipper_angle: int = 90   # 0..180


# ---------------------------------------------------------------------------
# EMA (Exponential Moving Average) filter
# ---------------------------------------------------------------------------
class EMAFilter:
    """Single-channel exponential moving average.

    alpha closer to 1.0 → less smoothing, faster response.
    alpha closer to 0.0 → heavier smoothing, more lag.
    """
    def __init__(self, alpha: float = 0.35):
        self.alpha = alpha
        self._value: Optional[float] = None

    def update(self, raw: float) -> float:
        if self._value is None:
            self._value = raw
        else:
            self._value = self.alpha * raw + (1 - self.alpha) * self._value
        return self._value

    def reset(self):
        self._value = None

    @property
    def value(self) -> float:
        return self._value if self._value is not None else 0.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


def _landmark_distance(lm_a, lm_b) -> float:
    """Euclidean distance between two NormalizedLandmarks in 2D."""
    return math.hypot(lm_a.x - lm_b.x, lm_a.y - lm_b.y)


def _map_range(value: float, in_lo: float, in_hi: float,
               out_lo: float, out_hi: float) -> float:
    """Linear interpolation from one range to another."""
    value = _clamp(value, in_lo, in_hi)
    return out_lo + (value - in_lo) / (in_hi - in_lo) * (out_hi - out_lo)


# ---------------------------------------------------------------------------
# Core tracker
# ---------------------------------------------------------------------------
class HandTracker:
    # Deadband: suppress servo updates smaller than this many degrees
    DEADBAND_DEG = 2

    # Pinch distance thresholds (normalized screen coords).
    # These are tuned for a typical 640×480 webcam at ~60cm distance.
    # ~0.04 = fingers touching, ~0.25 = hand wide open.
    PINCH_CLOSED = 0.05
    PINCH_OPEN = 0.22

    def __init__(
        self,
        model_path: Optional[str] = None,
        max_hands: int = 1,
        detection_conf: float = 0.7,
        tracking_conf: float = 0.5,
        smoothing_alpha: float = 0.35,
    ):
        # --- MediaPipe Tasks setup ---
        if model_path is None:
            model_path = os.path.join(
                os.path.dirname(__file__), "hand_landmarker.task"
            )

        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=max_hands,
            min_hand_detection_confidence=detection_conf,
            min_tracking_confidence=tracking_conf,
            running_mode=vision.RunningMode.IMAGE,
        )
        self.detector = vision.HandLandmarker.create_from_options(options)

        # --- Smoothing filters (one per servo channel) ---
        self._ema_base = EMAFilter(smoothing_alpha)
        self._ema_updown = EMAFilter(smoothing_alpha)
        self._ema_clipper = EMAFilter(smoothing_alpha)

        # --- Deadband: last angles actually sent ---
        self._sent_base = 90
        self._sent_updown = 90
        self._sent_clipper = 90

    # -----------------------------------------------------------------------
    # Public API
    # -----------------------------------------------------------------------
    def process(self, frame_bgr: np.ndarray) -> Tuple[HandState, np.ndarray]:
        """Process one BGR frame.

        Returns:
            state:     HandState with smoothed servo angles.
            annotated: BGR frame with landmarks and HUD drawn on it.
        """
        frame = cv2.flip(frame_bgr, 1)       # mirror for natural control
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        results = self.detector.detect(mp_image)

        if not results.hand_landmarks:
            self._reset_filters()
            self._draw_no_hand(frame)
            return HandState(detected=False), frame

        lm = results.hand_landmarks[0]

        # --- Draw skeleton ---
        vision.drawing_utils.draw_landmarks(
            frame, lm, vision.HandLandmarksConnections.HAND_CONNECTIONS
        )

        # --- Extract raw hand metrics ---
        palm_x, palm_y = self._palm_center(lm)
        pinch_raw = self._pinch_distance(lm)

        # --- Map to raw servo angles ---
        raw_base = _map_range(palm_x, 0.0, 1.0, 180.0, 0.0)   # mirrored
        raw_updown = _map_range(palm_y, 0.0, 1.0, 180.0, 0.0)  # top=up
        raw_clipper = _map_range(
            pinch_raw, self.PINCH_CLOSED, self.PINCH_OPEN, 0.0, 180.0
        )  # pinched=closed(0), open=open(180)

        # --- Smooth with EMA ---
        smooth_base = self._ema_base.update(raw_base)
        smooth_updown = self._ema_updown.update(raw_updown)
        smooth_clipper = self._ema_clipper.update(raw_clipper)

        # --- Deadband gate ---
        base_out = self._deadband(smooth_base, self._sent_base)
        updown_out = self._deadband(smooth_updown, self._sent_updown)
        clipper_out = self._deadband(smooth_clipper, self._sent_clipper)

        self._sent_base = base_out
        self._sent_updown = updown_out
        self._sent_clipper = clipper_out

        state = HandState(
            detected=True,
            palm_x=palm_x,
            palm_y=palm_y,
            pinch=pinch_raw,
            base_angle=base_out,
            updown_angle=updown_out,
            clipper_angle=clipper_out,
        )

        # --- Draw HUD overlay ---
        self._draw_hud(frame, lm, state)

        return state, frame

    def release(self):
        """Release MediaPipe resources."""
        self.detector.close()

    # -----------------------------------------------------------------------
    # Private helpers
    # -----------------------------------------------------------------------
    @staticmethod
    def _palm_center(lm) -> Tuple[float, float]:
        """Average of wrist and middle-finger MCP gives a stable palm center."""
        wx, wy = lm[WRIST].x, lm[WRIST].y
        mx, my = lm[MIDDLE_MCP].x, lm[MIDDLE_MCP].y
        return (wx + mx) / 2.0, (wy + my) / 2.0

    @staticmethod
    def _pinch_distance(lm) -> float:
        """Normalized Euclidean distance between thumb tip and index tip."""
        return _landmark_distance(lm[THUMB_TIP], lm[INDEX_TIP])

    def _deadband(self, smooth: float, last_sent: int) -> int:
        """Only update the angle if it changed more than DEADBAND_DEG."""
        rounded = int(_clamp(round(smooth), 0, 180))
        if abs(rounded - last_sent) >= self.DEADBAND_DEG:
            return rounded
        return last_sent

    def _reset_filters(self):
        """Reset EMA filters when hand is lost (avoid stale state on re-detect)."""
        self._ema_base.reset()
        self._ema_updown.reset()
        self._ema_clipper.reset()

    # -----------------------------------------------------------------------
    # Drawing
    # -----------------------------------------------------------------------
    @staticmethod
    def _draw_no_hand(frame: np.ndarray):
        cv2.putText(
            frame, "No hand detected",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
        )

    def _draw_hud(self, frame: np.ndarray, lm, state: HandState):
        h_px, w_px = frame.shape[:2]

        # Palm center dot (green)
        cx = int(state.palm_x * w_px)
        cy = int(state.palm_y * h_px)
        cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)

        # Pinch line between thumb and index (cyan when open, red when closed)
        thumb = (int(lm[THUMB_TIP].x * w_px), int(lm[THUMB_TIP].y * h_px))
        index = (int(lm[INDEX_TIP].x * w_px), int(lm[INDEX_TIP].y * h_px))
        pinch_color = (0, 255, 255) if state.clipper_angle > 90 else (0, 0, 255)
        cv2.line(frame, thumb, index, pinch_color, 3)

        # Text HUD
        lines = [
            f"Base:    {state.base_angle:3d}",
            f"Up/Down: {state.updown_angle:3d}",
            f"Clipper: {state.clipper_angle:3d}",
            f"Pinch:   {state.pinch:.2f}",
        ]
        y0 = 28
        for i, text in enumerate(lines):
            cv2.putText(
                frame, text,
                (10, y0 + i * 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )


    # -----------------------------------------------------------------------
    # Private helpers
    # -----------------------------------------------------------------------
    @staticmethod
    def _palm_center(lm) -> Tuple[float, float]:
        """Average of wrist and middle-finger MCP gives a stable palm center."""
        wx, wy = lm[WRIST].x, lm[WRIST].y
        mx, my = lm[MIDDLE_MCP].x, lm[MIDDLE_MCP].y
        return (wx + mx) / 2.0, (wy + my) / 2.0

    @staticmethod
    def _pinch_distance(lm) -> float:
        """Normalized Euclidean distance between thumb tip and index tip."""
        return _landmark_distance(lm[THUMB_TIP], lm[INDEX_TIP])

    def _deadband(self, smooth: float, last_sent: int) -> int:
        """Only update the angle if it changed more than DEADBAND_DEG."""
        rounded = int(_clamp(round(smooth), 0, 180))
        if abs(rounded - last_sent) >= self.DEADBAND_DEG:
            return rounded
        return last_sent

    def _reset_filters(self):
        """Reset EMA filters when hand is lost (avoid stale state on re-detect)."""
        self._ema_base.reset()
        self._ema_updown.reset()
        self._ema_clipper.reset()

    # -----------------------------------------------------------------------
    # Drawing
    # -----------------------------------------------------------------------
    @staticmethod
    def _draw_no_hand(frame: np.ndarray):
        cv2.putText(
            frame, "No hand detected",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
        )

    def _draw_hud(self, frame: np.ndarray, lm, state: HandState):
        h_px, w_px = frame.shape[:2]

        # Palm center dot (green)
        cx = int(state.palm_x * w_px)
        cy = int(state.palm_y * h_px)
        cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)

        # Pinch line between thumb and index (cyan when open, red when closed)
        thumb = (int(lm[THUMB_TIP].x * w_px), int(lm[THUMB_TIP].y * h_px))
        index = (int(lm[INDEX_TIP].x * w_px), int(lm[INDEX_TIP].y * h_px))
        pinch_color = (0, 255, 255) if state.clipper_angle > 90 else (0, 0, 255)
        cv2.line(frame, thumb, index, pinch_color, 3)

        # Text HUD
        lines = [
            f"Base:    {state.base_angle:3d}",
            f"Up/Down: {state.updown_angle:3d}",
            f"Clipper: {state.clipper_angle:3d}",
            f"Pinch:   {state.pinch:.2f}",
        ]
        y0 = 28
        for i, text in enumerate(lines):
            cv2.putText(
                frame, text,
                (10, y0 + i * 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )