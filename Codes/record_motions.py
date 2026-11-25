import json
import threading
import time
from datetime import datetime

import serial
from serial.tools import list_ports
from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM5"
SERVO_IDS = [1, 2, 3, 4, 5, 6]
SAVE_FILE = "recorded_motions.json"
SAMPLE_HZ = 20

JOINT_NAMES = {
    1: "Base",
    2: "Shoulder",
    3: "Elbow",
    4: "Wrist Pitch",
    5: "Wrist Roll",
    6: "Gripper",
}

stop_flag = threading.Event()


def open_controller(port):
    available = [p.device for p in list_ports.comports()]
    print("Available ports:", available or "(none)")
    if port not in available:
        raise RuntimeError(f"{port} not found. Plug in the adapter and pick a listed port.")
    try:
        return ST3215(port)
    except serial.SerialException as e:
        raise RuntimeError(
            f"Could not open {port}. Close other scripts, unplug/replug USB.\nWindows: {e}"
        ) from e


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


def wait_for_enter_thread():
    input()
    stop_flag.set()


def safe_name(text):
    cleaned = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in text.strip())
    return cleaned.upper() or f"MOTION_{datetime.now().strftime('%H%M%S')}"


def read_positions(controller, servos):
    if hasattr(controller, "broadcast"):
        try:
            raw = controller.broadcast.sram.sync_read_current_location(list(servos.keys()))
            if isinstance(raw, dict) and raw:
                return {int(k): (None if v is None else int(v)) for k, v in raw.items()}
        except Exception:
            pass

    pose = {}
    for sid, servo in servos.items():
        try:
            pose[sid] = int(servo.sram.read_current_location())
        except Exception:
            pose[sid] = None
    return pose


def load_motions(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {"motions": []}
    except FileNotFoundError:
        return {"motions": []}
    except Exception as e:
        print(f"Could not read {path}: {e}")
        return {"motions": []}


def save_library(path, library):
    library["port"] = PORT
    library["saved_at"] = datetime.now().isoformat(timespec="seconds")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(library, f, indent=2)


def list_motions(motions):
    if not motions:
        print("No motions saved yet.")
        return
    print("Saved motions:")
    for i, item in enumerate(motions, start=1):
        dur = item.get("duration_s", 0.0)
        n = len(item.get("frames", []))
        print(f"  {i}. {item.get('name', 'UNNAMED')}  ({dur:.2f}s, {n} frames)")


def upsert_motion(motions, motion):
    name = motion["name"]
    for i, item in enumerate(motions):
        if str(item.get("name", "")).upper() == name:
            motions[i] = motion
            return "updated"
    motions.append(motion)
    return "added"


def record_one(controller, servos):
    stop_flag.clear()
    threading.Thread(target=wait_for_enter_thread, daemon=True).start()

    frames = []
    dt = 1.0 / SAMPLE_HZ
    t0 = time.perf_counter()
    print(f"Recording at {SAMPLE_HZ} Hz. Move the arm, then press Enter to STOP.")

    while not stop_flag.is_set():
        loop_start = time.perf_counter()
        pose = read_positions(controller, servos)
        frames.append({"t": round(loop_start - t0, 4), "positions": pose})
        sleep_for = dt - (time.perf_counter() - loop_start)
        if sleep_for > 0:
            time.sleep(sleep_for)

    duration = frames[-1]["t"] if frames else 0.0
    print(f"Stopped. Captured {len(frames)} frames over {duration:.2f} s.")
    return frames, duration


print("Connecting...")
controller = None
try:
    controller = open_controller(PORT)
    servos = {}
    for sid in SERVO_IDS:
        try:
            servo = controller.wrap_servo(sid)
            torque_off(servo)
            servos[sid] = servo
            print(f"Servo {sid} ({JOINT_NAMES[sid]}): limp / ready")
        except ServoNotRespondingError:
            print(f"Servo {sid}: NO RESPONSE")
        except Exception as e:
            print(f"Servo {sid}: {e}")

    if not servos:
        raise RuntimeError("No active servos available.")

    library = load_motions(SAVE_FILE)
    motions = library.get("motions", [])
    list_motions(motions)

    print("\nHold the arm. Torque is off.")
    print("Commands: Enter = record, list, q = quit")

    while True:
        cmd = input("\nrecord> ").strip().lower()
        if cmd in ("q", "quit", "exit"):
            break
        if cmd == "list":
            list_motions(motions)
            continue
        if cmd not in ("", "r", "rec", "record", "start"):
            print("Press Enter to record, or type list / q.")
            continue

        input("Ready. Press Enter to START this clip...")
        frames, duration = record_one(controller, servos)
        if not frames:
            print("Nothing captured. Try again.")
            continue

        raw_name = input("Name this motion (or q to discard): ").strip()
        if raw_name.lower() in ("q", "quit", "exit", "discard"):
            print("Discarded that clip.")
            continue

        name = safe_name(raw_name)
        motion = {
            "name": name,
            "time": datetime.now().isoformat(timespec="seconds"),
            "sample_hz": SAMPLE_HZ,
            "duration_s": duration,
            "servo_ids": list(servos.keys()),
            "frames": frames,
        }
        action = upsert_motion(motions, motion)
        library["motions"] = motions
        save_library(SAVE_FILE, library)
        print(f"{action.title()} '{name}'. File now has {len(motions)} motion(s).")

    library["motions"] = motions
    save_library(SAVE_FILE, library)
    print(f"\nWrote {len(motions)} motion(s) to {SAVE_FILE}")
    list_motions(motions)

finally:
    if controller is not None:
        controller.close()
    print("Connection closed.")