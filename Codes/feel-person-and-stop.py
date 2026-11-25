"""
LfD playback.
Light touch = load spike above the recent moving average.
On contact: torque ON, hold 2s, resume.
"""

import json
import time
from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM5"
MOTION_FILE = "recorded_motions.json"
SERVO_IDS = [1, 2, 3, 4, 5, 6]

SMOOTH_SPEED = 600
SMOOTH_ACCEL = 25
PLAYBACK_SCALE = 2.0
GOTO_START_TIMEOUT_S = 12.0
SETTLE_TIMEOUT_S = 3.0
CONTACT_PAUSE_S = 2.0

# Light touch: spike over the joint's recent load
TOUCH_DELTA = 55          # lower = more sensitive (try 40 if still stiff)
MIN_TOUCH_LOAD = 70       # ignore encoder noise near 0
CONTACT_HITS = 2
CONTACT_COOLDOWN_S = 0.4
WARMUP_SAMPLES = 10       # learn normal motion load before arming
SKIP_CONTACT_JOINTS = set()
REPLAY_POINTS = 80

JOINT_NAMES = {
    1: "Base", 2: "Shoulder", 3: "Elbow",
    4: "Wrist Pitch", 5: "Wrist Roll", 6: "Gripper",
}


def set_servo_profile_safely(servo, speed, accel):
    try:
        if hasattr(servo.sram, "write_acceleration"):
            servo.sram.write_acceleration(accel)
        if hasattr(servo.sram, "write_running_speed"):
            servo.sram.write_running_speed(speed)
        elif hasattr(servo.sram, "write_goal_speed"):
            servo.sram.write_goal_speed(speed)
    except Exception:
        pass
    try:
        if hasattr(servo.sram, "write_register"):
            servo.sram.write_register(41, accel, 1)
            servo.sram.write_register(46, speed, 2)
    except Exception:
        pass


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
        servo.sram.torque_disable()
    except Exception:
        try:
            servo.sram.write_torque_switch(0)
        except Exception:
            pass


def servo_is_moving(servo):
    for name in ("read_moving", "read_mobile_sign"):
        try:
            fn = getattr(servo.sram, name, None)
            if callable(fn):
                return bool(fn())
        except Exception:
            pass
    return False


def read_load(servo):
    try:
        raw = servo.sram.read_current_load()
        if raw is None:
            return 0
        return abs(int(raw)) & 0x3FF
    except Exception:
        return 0


def read_pose(servos):
    pose = {}
    for sid, servo in servos.items():
        try:
            pose[sid] = int(servo.sram.read_current_location())
        except Exception:
            pass
    return pose


def clean_positions(raw):
    pose = {}
    if not isinstance(raw, dict):
        return pose
    for k, v in raw.items():
        if v is None:
            continue
        try:
            pose[int(k)] = max(0, min(4095, int(round(float(v)))))
        except Exception:
            pass
    return pose


def dispatch_targets(controller, servos, targets):
    live = {sid: pos for sid, pos in targets.items() if sid in servos}
    if not live:
        return
    if hasattr(controller, "sync_write_target_location"):
        try:
            controller.sync_write_target_location(live)
            return
        except Exception as e:
            print(f"Sync write error: {e}")
    if hasattr(controller, "broadcast"):
        try:
            controller.broadcast.sram.sync_write_target_location(live)
            return
        except Exception as e:
            print(f"Broadcast sync write error: {e}")
    for sid, pos in live.items():
        try:
            servos[sid].sram.write_target_location(pos)
        except Exception:
            pass


def load_motions(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    raw_items = data.get("motions", []) if isinstance(data, dict) else data
    motions = []
    for item in raw_items or []:
        if not isinstance(item, dict):
            continue
        frames = []
        for frame in item.get("frames", []):
            if not isinstance(frame, dict):
                continue
            positions = clean_positions(frame.get("positions", {}))
            if positions:
                frames.append({"t": float(frame.get("t", 0.0)), "positions": positions})
        if not frames:
            continue
        frames.sort(key=lambda fr: fr["t"])
        motions.append({
            "name": str(item.get("name", "UNNAMED")),
            "duration_s": float(item.get("duration_s", frames[-1]["t"])),
            "frames": frames,
        })
    return motions


def lerp(a, b, w):
    return a + (b - a) * w


def sample_demo(frames, ids, u):
    t_end = frames[-1]["t"] if frames[-1]["t"] > 0 else 1.0
    t = u * t_end
    if t <= frames[0]["t"]:
        return {sid: frames[0]["positions"].get(sid) for sid in ids}
    if t >= t_end:
        return {sid: frames[-1]["positions"].get(sid) for sid in ids}
    for i in range(1, len(frames)):
        t0, t1 = frames[i - 1]["t"], frames[i]["t"]
        if t1 < t:
            continue
        w = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        pose = {}
        for sid in ids:
            a = frames[i - 1]["positions"].get(sid)
            b = frames[i]["positions"].get(sid)
            if a is None and b is None:
                continue
            pose[sid] = b if a is None else a if b is None else int(round(lerp(a, b, w)))
        return pose
    return {sid: frames[-1]["positions"].get(sid) for sid in ids}


def average_demos(demos, ids, n):
    duration = sum(d["duration_s"] for d in demos) / len(demos)
    path = []
    for i in range(n):
        u = i / (n - 1)
        buckets = {sid: [] for sid in ids}
        for demo in demos:
            for sid, pos in sample_demo(demo["frames"], ids, u).items():
                if pos is not None:
                    buckets[sid].append(pos)
        mean = {sid: int(round(sum(v) / len(v))) for sid, v in buckets.items() if v}
        path.append({"t": u * duration, "positions": mean})
    return path, duration


class ContactGuard:
    def __init__(self, servos):
        self.servos = servos
        self.hits = {sid: 0 for sid in servos}
        self.ema = {sid: 0.0 for sid in servos}
        self.n = {sid: 0 for sid in servos}
        self.cooldown_until = 0.0

    def reset_hits(self):
        self.hits = {sid: 0 for sid in self.servos}
        self.n = {sid: 0 for sid in self.servos}

    def arm_cooldown(self):
        self.hits = {sid: 0 for sid in self.servos}
        self.cooldown_until = time.perf_counter() + CONTACT_COOLDOWN_S

    def check(self):
        if time.perf_counter() < self.cooldown_until:
            return None
        for sid, servo in self.servos.items():
            if sid in SKIP_CONTACT_JOINTS:
                continue
            load = read_load(servo)
            self.n[sid] += 1
            if self.n[sid] == 1:
                self.ema[sid] = float(load)
            else:
                self.ema[sid] = 0.85 * self.ema[sid] + 0.15 * load

            if self.n[sid] < WARMUP_SAMPLES:
                continue

            touched = load >= MIN_TOUCH_LOAD and load >= self.ema[sid] + TOUCH_DELTA
            self.hits[sid] = self.hits[sid] + 1 if touched else 0
            if self.hits[sid] >= CONTACT_HITS:
                return sid, load, int(self.ema[sid])
        return None


def pause_on_contact(controller, servos, guard, hit, phase):
    sid, load, ema = hit
    print(f"\nTOUCH during {phase}: {sid} {JOINT_NAMES.get(sid)} load={load} avg={ema}")
    print(f"Torque ON. Holding {CONTACT_PAUSE_S:.0f}s, then resume...")
    hold = read_pose(servos)
    if hold:
        dispatch_targets(controller, servos, hold)
    time.sleep(CONTACT_PAUSE_S)
    guard.arm_cooldown()
    print("Resuming.")


def wait_with_contact(controller, servos, guard, timeout_s, phase):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        hit = guard.check()
        if hit:
            pause_on_contact(controller, servos, guard, hit, phase)
            t0 = time.time()
            continue
        if not any(servo_is_moving(s) for s in servos.values()):
            time.sleep(0.15)
            return
        time.sleep(0.04)


def play_path(controller, servos, guard, path, duration):
    print("Moving to start...")
    dispatch_targets(controller, servos, path[0]["positions"])
    wait_with_contact(controller, servos, guard, GOTO_START_TIMEOUT_S, "start move")

    print(f"Playing ({duration * PLAYBACK_SCALE:.2f}s)...")
    t0 = time.perf_counter()
    extra = 0.0
    for frame in path:
        wait = frame["t"] * PLAYBACK_SCALE + extra - (time.perf_counter() - t0)
        if wait > 0:
            time.sleep(wait)
        dispatch_targets(controller, servos, frame["positions"])
        hit = guard.check()
        if hit:
            started = time.perf_counter()
            pause_on_contact(controller, servos, guard, hit, "playback")
            extra += time.perf_counter() - started

    wait_with_contact(controller, servos, guard, SETTLE_TIMEOUT_S, "settle")
    print("Finished.")


def list_motions(motions):
    print("\nRecorded motions:")
    for i, m in enumerate(motions, start=1):
        print(f"  {i}. {m['name']}  ({m['duration_s']:.2f}s, {len(m['frames'])} frames)")
    print("  q. quit")


def pick_demos(motions):
    list_motions(motions)
    raw = input("demos> ").strip()
    if raw.lower() in ("q", "quit", "exit", ""):
        return None
    out = []
    for tok in raw.replace(",", " ").split():
        if tok.isdigit() and 1 <= int(tok) <= len(motions):
            out.append(motions[int(tok) - 1])
    return out or None


print(f"Loading {MOTION_FILE}...")
motions = load_motions(MOTION_FILE)
if not motions:
    raise SystemExit("No clips.")

controller = None
servos = {}
try:
    controller = ST3215(PORT)
    for sid in SERVO_IDS:
        try:
            servo = controller.wrap_servo(sid)
            torque_on(servo)
            set_servo_profile_safely(servo, SMOOTH_SPEED, SMOOTH_ACCEL)
            servos[sid] = servo
            print(f"Servo {sid} ({JOINT_NAMES[sid]}): ready")
        except ServoNotRespondingError:
            print(f"Servo {sid}: NO RESPONSE")
    if not servos:
        raise RuntimeError("No servos.")

    guard = ContactGuard(servos)
    print(f"Touch: load ≥ recent avg + {TOUCH_DELTA}")

    while True:
        chosen = pick_demos(motions)
        if not chosen:
            break
        guard.reset_hits()
        print("Averaging:", ", ".join(d["name"] for d in chosen))
        path, duration = average_demos(chosen, list(servos.keys()), REPLAY_POINTS)
        play_path(controller, servos, guard, path, duration)
finally:
    for s in servos.values():
        torque_off(s)
    if controller is not None:
        controller.close()
    print("Connection closed.")