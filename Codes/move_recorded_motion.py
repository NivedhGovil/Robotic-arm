import json
import time

from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM5"
MOTION_FILE = "recorded_motions.json"
SERVO_IDS = [1, 2, 3, 4, 5, 6]

# Same smoothness as your center-move script
SMOOTH_SPEED = 800
SMOOTH_ACCEL = 40

# 2.0 = play at half recorded speed. Raise to 3.0 if still snappy.
PLAYBACK_SCALE = 2.0

GOTO_START_TIMEOUT_S = 12.0

JOINT_NAMES = {
    1: "Base",
    2: "Shoulder",
    3: "Elbow",
    4: "Wrist Pitch",
    5: "Wrist Roll",
    6: "Gripper",
}

controller = ST3215(PORT)


def set_servo_profile_safely(servo, speed, accel):
    """Write speed + accel to the real ST3215 registers."""
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
        if hasattr(servo.sram, "acceleration"):
            servo.sram.acceleration = accel
        if hasattr(servo.sram, "running_speed"):
            servo.sram.running_speed = speed
        if hasattr(servo.sram, "speed"):
            servo.sram.speed = speed
        if hasattr(servo.sram, "moving_speed"):
            servo.sram.moving_speed = speed
    except Exception:
        pass

    try:
        if hasattr(servo.sram, "write_register"):
            servo.sram.write_register(41, accel, 1)
            servo.sram.write_register(46, speed, 2)
        elif hasattr(servo.sram, "set_register"):
            servo.sram.set_register(41, accel)
            servo.sram.set_register(46, speed)
    except Exception:
        pass


def torque_on(servo):
    try:
        if hasattr(servo.sram, "torque_enable"):
            servo.sram.torque_enable()
            return
    except Exception:
        pass
    try:
        servo.sram.write_torque_switch(1)
    except Exception:
        pass


def torque_off(servo):
    try:
        if hasattr(servo.sram, "torque_disable"):
            servo.sram.torque_disable()
            return
    except Exception:
        pass
    try:
        servo.sram.write_torque_switch(0)
    except Exception:
        pass


def servo_is_moving(servo):
    try:
        if hasattr(servo.sram, "read_moving"):
            return bool(servo.sram.read_moving())
        if hasattr(servo.sram, "read_mobile_sign"):
            return bool(servo.sram.read_mobile_sign())
    except Exception:
        pass
    try:
        if hasattr(servo.sram, "read_register"):
            return bool(servo.sram.read_register(66, 1))
    except Exception:
        pass
    return False


def wait_until_still(servos, timeout_s):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if not any(servo_is_moving(s) for s in servos.values()):
            time.sleep(0.2)
            return True
        time.sleep(0.1)
    print("Timeout waiting for motion to finish.")
    return False


def clean_positions(raw):
    pose = {}
    if not isinstance(raw, dict):
        return pose
    for k, v in raw.items():
        if v is None:
            continue
        try:
            sid = int(k)
            pos = int(round(float(v)))
        except Exception:
            continue
        pose[sid] = max(0, min(4095, pos))
    return pose


def dispatch_targets(servos, targets):
    live = {sid: pos for sid, pos in targets.items() if sid in servos}
    if not live:
        return live

    sent = False
    if hasattr(controller, "sync_write_target_location"):
        try:
            controller.sync_write_target_location(live)
            sent = True
        except Exception as e:
            print(f"Sync Write error: {e}")

    if not sent and hasattr(controller, "broadcast"):
        try:
            controller.broadcast.sram.sync_write_target_location(live)
            sent = True
        except Exception as e:
            print(f"Broadcast sync write error: {e}")

    if not sent:
        for sid, pos in live.items():
            try:
                servos[sid].sram.write_target_location(pos)
            except Exception:
                pass
    return live


def load_motions(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    raw_items = data.get("motions", []) if isinstance(data, dict) else data
    if not isinstance(raw_items, list):
        return []

    motions = []
    for item in raw_items:
        if not isinstance(item, dict):
            continue
        frames = []
        for frame in item.get("frames", []):
            if not isinstance(frame, dict):
                continue
            positions = clean_positions(frame.get("positions", {}))
            if not positions:
                continue
            frames.append({
                "t": float(frame.get("t", 0.0)),
                "positions": positions,
            })
        if not frames:
            continue
        frames.sort(key=lambda f: f["t"])
        motions.append({
            "name": str(item.get("name", "UNNAMED")),
            "duration_s": float(item.get("duration_s", frames[-1]["t"])),
            "sample_hz": item.get("sample_hz"),
            "frames": frames,
        })
    return motions


def list_motions(motions):
    print("\nRecorded motions:")
    for i, item in enumerate(motions, start=1):
        print(
            f"  {i}. {item['name']}  "
            f"({item['duration_s']:.2f}s, {len(item['frames'])} frames)"
        )
    print("  q. quit")


def ask_which_motion(motions):
    list_motions(motions)
    while True:
        choice = input("\nPlay which motion? ").strip()
        if choice.lower() in ("q", "quit", "exit"):
            return None
        if choice.isdigit():
            idx = int(choice)
            if 1 <= idx <= len(motions):
                return motions[idx - 1]
            print(f"Pick a number from 1 to {len(motions)}.")
            continue
        key = choice.upper().replace(" ", "_")
        matches = [
            m for m in motions
            if m["name"].upper().replace(" ", "_") == key
        ]
        if matches:
            return matches[0]
        print("Not found. Type a number, a motion name, or q.")


def play_motion(servos, motion):
    frames = [f for f in motion["frames"] if f["positions"]]
    if not frames:
        print("That motion has no usable frames.")
        return

    scaled = motion["duration_s"] * PLAYBACK_SCALE
    print(f"Moving to start of '{motion['name']}' slowly...")
    dispatch_targets(servos, frames[0]["positions"])
    wait_until_still(servos, GOTO_START_TIMEOUT_S)

    print(
        f"Playing '{motion['name']}' smoothly "
        f"({scaled:.2f}s at scale {PLAYBACK_SCALE})..."
    )
    t0 = time.perf_counter()
    for frame in frames:
        wait = (frame["t"] * PLAYBACK_SCALE) - (time.perf_counter() - t0)
        if wait > 0:
            time.sleep(wait)
        dispatch_targets(servos, frame["positions"])

    wait_until_still(servos, 3.0)
    print(f"Finished '{motion['name']}'.")


print(f"Loading motions from {MOTION_FILE}...")
try:
    motions = load_motions(MOTION_FILE)
except FileNotFoundError:
    raise SystemExit(f"No motion file found: {MOTION_FILE}")
except Exception as e:
    raise SystemExit(f"Could not read {MOTION_FILE}: {e}")

if not motions:
    raise SystemExit("Motion file has no usable clips.")

servos = {}
try:
    print("Configuring slow S-curve profile...")
    for sid in SERVO_IDS:
        try:
            servo = controller.wrap_servo(sid)
            torque_on(servo)
            set_servo_profile_safely(servo, SMOOTH_SPEED, SMOOTH_ACCEL)
            curr = servo.sram.read_current_location()
            servos[sid] = servo
            print(
                f"Servo {sid} ({JOINT_NAMES.get(sid, 'Joint')}): "
                f"Current = {curr}"
            )
        except ServoNotRespondingError:
            print(f"Servo {sid}: NO RESPONSE")
        except Exception as e:
            print(f"Servo {sid}: Setup Error -> {e}")

    if not servos:
        raise RuntimeError("No active servos connected.")

    while True:
        chosen = ask_which_motion(motions)
        if chosen is None:
            break
        play_motion(servos, chosen)

finally:
    for servo in servos.values():
        try:
            torque_off(servo)
        except Exception:
            pass
    controller.close()
    print("Connection closed.")