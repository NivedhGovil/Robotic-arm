"""
Hand-only ST3215 control.
ID 3 = wrist toward / away from camera (palm size).
ID 4 = half of ID 3.
Directions reversed vs last build.
"""

import math
import os
import time
import urllib.request

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM5"
SERVO_IDS = [1, 2, 3, 4, 5, 6]
CAMERA_INDEX = 0
CAM_W, CAM_H = 640, 480
RATE_HZ = 40
SAFE_MARGIN = 50
MAX_STEP = {1: 140, 2: 110, 3: 140, 4: 90, 5: 140, 6: 100}
SPEED = 1600
ACCEL = 45
REST_SETTLE_S = 1.2

SCALE = {1: 900, 2: 700, 3: 2200, 4: 1100, 5: 800, 6: 500}
DEADZONE = {1: 0.02, 2: 0.025, 3: 0.004, 4: 0.004, 5: 0.02, 6: 0.025}

# Reversed again from the previous file
INVERT = {
    "up": True,
    "lr": True,
    "fingers": True,
    "wrist_depth": True,
    "twist": True,
    "reach": True,
}

HERE = os.path.dirname(os.path.abspath(__file__))
HAND_FILE = os.path.join(HERE, "hand_landmarker.task")
HAND_URL = (
    "https://storage.googleapis.com/mediapipe-models/"
    "hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
)

LIMITS = {
    1: (752, 3443),
    2: (804, 3209),
    3: (770, 3097),
    4: (843, 2911),
    5: (0, 4095),
    6: (700, 2500),
}
REST = {
    1: 2048,
    2: 804,
    3: 3054,
    4: 1926,
    5: 3385,
    6: 2301,
}
LABELS = {
    1: "BASE 1",
    2: "FING 2",
    3: "WRIST 3",
    4: "HALF 4",
    5: "TWIST 5",
    6: "REACH 6",
}

WRIST = 0
INDEX_MCP, INDEX_TIP = 5, 8
MIDDLE_MCP, MIDDLE_TIP = 9, 12
RING_MCP, RING_TIP = 13, 16
PINKY_MCP, PINKY_TIP = 17, 20


class OneEuro:
    def __init__(self, min_cutoff=3.2, beta=0.05, dcutoff=1.5):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.dcutoff = dcutoff
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    @staticmethod
    def _alpha(dt, cutoff):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / max(dt, 1e-6))

    def __call__(self, x, t):
        x = float(x)
        if self.x_prev is None:
            self.x_prev, self.t_prev = x, t
            return x
        dt = max(t - self.t_prev, 1e-3)
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(dt, self.dcutoff)
        edx = a_d * dx + (1.0 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * abs(edx)
        a = self._alpha(dt, cutoff)
        x_hat = a * x + (1.0 - a) * self.x_prev
        self.x_prev, self.dx_prev, self.t_prev = x_hat, edx, t
        return x_hat


def walls(limits, margin):
    out = {}
    for sid, (lo, hi) in limits.items():
        a, b = lo + margin, hi - margin
        if a >= b:
            mid = (lo + hi) // 2
            a, b = mid - 1, mid + 1
        out[sid] = (a, b)
    return out


SAFE = walls(LIMITS, SAFE_MARGIN)
FLOOR = {sid: SAFE[sid][0] for sid in SERVO_IDS}
CEIL = {sid: SAFE[sid][1] for sid in SERVO_IDS}


def download_model():
    if os.path.isfile(HAND_FILE) and os.path.getsize(HAND_FILE) > 10000:
        return
    print("Downloading hand_landmarker.task ...")
    urllib.request.urlretrieve(HAND_URL, HAND_FILE)


def open_camera(index):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    return cap


def torque_on(servo):
    try:
        servo.sram.torque_enable()
    except Exception:
        try:
            servo.sram.write_torque_switch(1)
        except Exception:
            pass


def torque_off(servo):
    try:
        servo.sram.write_torque_switch(0)
        return
    except Exception:
        pass
    try:
        servo.sram.torque_disable()
    except Exception:
        pass


def set_profile(servo, speed, accel):
    try:
        if hasattr(servo.sram, "write_acceleration"):
            servo.sram.write_acceleration(accel)
        if hasattr(servo.sram, "write_running_speed"):
            servo.sram.write_running_speed(speed)
    except Exception:
        pass
    try:
        if hasattr(servo.sram, "write_register"):
            servo.sram.write_register(41, accel, 1)
            servo.sram.write_register(46, speed, 2)
    except Exception:
        pass


def dispatch(controller, servos, targets):
    live = {sid: int(pos) for sid, pos in targets.items() if sid in servos}
    if not live:
        return
    if hasattr(controller, "sync_write_target_location"):
        try:
            controller.sync_write_target_location(live)
            return
        except Exception:
            pass
    for sid, pos in live.items():
        try:
            servos[sid].sram.write_target_location(pos)
        except Exception:
            pass


def clip(sid, pos):
    return max(FLOOR[sid], min(CEIL[sid], int(round(pos))))


def clamp_pose(pose):
    return {sid: clip(sid, pos) for sid, pos in pose.items()}


def limit_step(prev, nxt):
    if prev is None:
        return clamp_pose(nxt)
    out = {}
    for sid, pos in nxt.items():
        old = prev.get(sid, pos)
        step = MAX_STEP.get(sid, 80)
        out[sid] = clip(sid, max(old - step, min(old + step, pos)))
    return out


def dist(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def hand_ok(lm):
    if not (0.10 < lm[WRIST].x < 0.90 and 0.10 < lm[WRIST].y < 0.90):
        return False
    return sum(1 for p in lm if 0.03 < p.x < 0.97 and 0.03 < p.y < 0.97) >= 16


def palm_size(lm):
    """Gets bigger when the hand comes toward the camera."""
    return dist(lm[INDEX_MCP], lm[PINKY_MCP])


def features(lm):
    palm_y = (lm[INDEX_MCP].y + lm[MIDDLE_MCP].y + lm[RING_MCP].y + lm[PINKY_MCP].y) / 4.0
    tips_y = (lm[INDEX_TIP].y + lm[MIDDLE_TIP].y + lm[RING_TIP].y + lm[PINKY_TIP].y) / 4.0
    width = max(palm_size(lm), 0.04)
    reach = (dist(lm[INDEX_TIP], lm[WRIST]) + dist(lm[MIDDLE_TIP], lm[WRIST])) / (2.0 * width)
    twist = math.atan2(
        lm[PINKY_MCP].y - lm[INDEX_MCP].y,
        lm[PINKY_MCP].x - lm[INDEX_MCP].x,
    )
    return {
        "x": lm[WRIST].x,
        "y": lm[WRIST].y,
        "fingers": palm_y - tips_y,
        "wrist": width,
        "twist": twist,
        "reach": reach,
    }


def dead(delta, z):
    if abs(delta) < z:
        return 0.0
    return math.copysign(abs(delta) - z, delta)


def sgn(name, value):
    return -value if INVERT.get(name) else value


def features_to_targets(feat, zero):
    dx = sgn("lr", dead(feat["x"] - zero["x"], DEADZONE[1]))
    dy = sgn("up", dead(zero["y"] - feat["y"], DEADZONE[1]))
    df = sgn("fingers", dead(feat["fingers"] - zero["fingers"], DEADZONE[2]))
    dw = sgn("wrist_depth", dead(feat["wrist"] - zero["wrist"], DEADZONE[3]))
    dt = sgn("twist", dead(feat["twist"] - zero["twist"], DEADZONE[5]))
    dr = sgn("reach", dead(feat["reach"] - zero["reach"], DEADZONE[6]))

    # Palm 0.04 → 0.25 is a normal push toward the camera.
    delta3 = SCALE[3] * dw * 12.0

    return {
        1: REST[1] + SCALE[1] * (0.55 * dy + 0.45 * dx) * 2.2,
        2: REST[2] + SCALE[2] * df * 6.0,
        3: REST[3] + delta3,
        4: REST[4] + 0.5 * delta3,
        5: REST[5] + SCALE[5] * dt / math.pi,
        6: REST[6] + SCALE[6] * dr,
    }


def draw_hand(frame, lm):
    h, w = frame.shape[:2]
    chains = (
        (0, 1, 2, 3, 4), (0, 5, 6, 7, 8), (0, 9, 10, 11, 12),
        (0, 13, 14, 15, 16), (0, 17, 18, 19, 20), (5, 9, 13, 17),
    )

    def xy(i):
        return int(lm[i].x * w), int(lm[i].y * h)

    for chain in chains:
        for a, b in zip(chain, chain[1:]):
            cv2.line(frame, xy(a), xy(b), (0, 210, 0), 2)
    cv2.circle(frame, xy(WRIST), 7, (0, 80, 255), -1)
    for i in range(21):
        cv2.circle(frame, xy(i), 3, (0, 255, 255), -1)


def draw_bars(frame, pose, calibrated, wrist_val=None):
    h, w = frame.shape[:2]
    x0, bw, bh, gap = w - 230, 210, 22, 12
    y = 78
    overlay = frame.copy()
    cv2.rectangle(overlay, (x0 - 12, 36), (w - 6, y + 6 * (bh + gap) + 24), (18, 18, 18), -1)
    cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
    tag = "CALIBRATED" if calibrated else "press C"
    cv2.putText(frame, tag, (x0, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (80, 220, 80) if calibrated else (80, 180, 255), 1)
    for sid in SERVO_IDS:
        lo, hi = FLOOR[sid], CEIL[sid]
        pos = pose.get(sid, REST[sid])
        u = 0.0 if hi == lo else (pos - lo) / float(hi - lo)
        u = max(0.0, min(1.0, u))
        cv2.rectangle(frame, (x0, y), (x0 + bw, y + bh), (70, 70, 70), 1)
        fill = int(bw * u)
        color = (30, 70, 230) if u < 0.08 or u > 0.92 else (40, 200, 40)
        if fill:
            cv2.rectangle(frame, (x0, y), (x0 + fill, y + bh), color, -1)
        ru = (max(lo, min(hi, REST[sid])) - lo) / float(hi - lo)
        cv2.line(frame, (x0 + int(bw * ru), y), (x0 + int(bw * ru), y + bh), (240, 240, 240), 1)
        cv2.putText(frame, f"{LABELS[sid]} {int(pos)}", (x0, y - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (230, 230, 230), 1)
        y += bh + gap
    if wrist_val is not None:
        cv2.putText(frame, f"palm {wrist_val:.3f}", (x0, y + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 220, 255), 1)


download_model()
landmarker = vision.HandLandmarker.create_from_options(
    vision.HandLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=HAND_FILE),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.55,
        min_tracking_confidence=0.50,
    )
)

cap = open_camera(CAMERA_INDEX)
if not cap.isOpened():
    raise SystemExit("Camera not found. Try CAMERA_INDEX = 1.")
cv2.namedWindow("Hand to robot", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Hand to robot", 1280, 720)

filters = {sid: OneEuro() for sid in SERVO_IDS}
controller = ST3215(PORT)
servos = {}
try:
    for sid in SERVO_IDS:
        try:
            s = controller.wrap_servo(sid)
            torque_on(s)
            set_profile(s, SPEED, ACCEL)
            servos[sid] = s
            print(f"ID {sid}: floor {FLOOR[sid]} rest {REST[sid]} ceil {CEIL[sid]}")
        except ServoNotRespondingError:
            print(f"ID {sid}: NO RESPONSE")
    if not servos:
        raise RuntimeError("No servos.")

    last = clamp_pose(REST)
    dispatch(controller, servos, last)
    time.sleep(REST_SETTLE_S)
    print("Neutral hand, press C. Push hand toward camera to move ID 3.")

    zero = None
    frozen = False
    pending = None
    dt = 1.0 / RATE_HZ
    t_origin = time.perf_counter()

    while True:
        loop = time.perf_counter()
        ok, frame = cap.read()
        if not ok:
            continue
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb))
        ts = int((time.perf_counter() - t_origin) * 1000)
        result = landmarker.detect_for_video(mp_image, ts)
        now = time.perf_counter()

        status = "AT REST"
        wrist_val = None
        if result.hand_landmarks:
            lm = result.hand_landmarks[0]
            draw_hand(frame, lm)
            pending = features(lm) if hand_ok(lm) else None
            if pending:
                wrist_val = pending["wrist"]
            if zero is None:
                status = "press C"
            elif frozen:
                status = "FROZEN"
            elif pending is None:
                status = "EDGE"
            else:
                raw = features_to_targets(pending, zero)
                smooth = {sid: filters[sid](raw[sid], now) for sid in raw}
                last = limit_step(last, smooth)
                dispatch(controller, servos, last)
                status = f"TRACK  3={last[3]}  4={last[4]}"
        else:
            pending = None
            status = "FROZEN" if frozen else "NO HAND"

        draw_bars(frame, last, zero is not None, wrist_val)
        color = (0, 220, 0) if status.startswith("TRACK") else (0, 0, 220)
        cv2.putText(frame, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        cv2.imshow("Hand to robot", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" "):
            frozen = not frozen
        if key == ord("c") and pending is not None:
            zero = dict(pending)
            last = clamp_pose(REST)
            dispatch(controller, servos, last)
            filters = {sid: OneEuro() for sid in SERVO_IDS}
            print(f"Zero palm size = {zero['wrist']:.3f}. Move hand closer to move ID 3.")
        if key == ord("h"):
            last = clamp_pose(REST)
            dispatch(controller, servos, last)

        leftover = dt - (time.perf_counter() - loop)
        if leftover > 0:
            time.sleep(leftover)
finally:
    for s in servos.values():
        torque_off(s)
    cap.release()
    cv2.destroyAllWindows()
    landmarker.close()
    controller.close()
    print("Torque OFF. Closed.")