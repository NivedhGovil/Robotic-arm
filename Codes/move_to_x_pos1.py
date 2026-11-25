import json
import time
from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM7"
POSE_FILE = "recorded_poses.json"

SPEED = 600
ACCEL = 30
MOVE_TIMEOUT_S = 8.0

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


def servo_is_moving(servo):
    for name in ("read_moving", "read_mobile_sign"):
        try:
            fn = getattr(servo.sram, name, None)
            if fn:
                return bool(fn())
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


def dispatch_targets(servos, targets):
    live = {}
    for sid, target in targets.items():
        if sid in servos and target is not None:
            live[int(sid)] = int(target)
    if not live:
        raise RuntimeError("Chosen pose has no usable joint values.")

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
        for sid, target in live.items():
            try:
                servos[sid].sram.write_target_location(target)
            except Exception:
                pass
    return live


def pos_to_deg(pos):
    return pos * 360.0 / 4096.0


def print_pose(pose, title="Pose"):
    print(f"\n{title}")
    for sid in sorted(int(k) for k in pose.keys()):
        pos = pose.get(sid, pose.get(str(sid)))
        name = JOINT_NAMES.get(sid, f"ID {sid}")
        if pos is None:
            print(f"  {sid} {name:12}  NO DATA")
        else:
            print(f"  {sid} {name:12}  {int(pos):5d}  ({pos_to_deg(int(pos)):6.2f}°)")


def load_pose_book(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    book = []
    rest = data.get("rest")
    if isinstance(rest, dict) and rest:
        book.append({
            "name": "REST",
            "positions": {int(k): v for k, v in rest.items()},
        })

    for item in data.get("poses", []):
        name = str(item.get("name", "UNNAMED"))
        positions = item.get("positions", {})
        book.append({
            "name": name,
            "positions": {int(k): v for k, v in positions.items()},
        })

    # keep last save if the same name was recorded twice
    unique = {}
    for item in book:
        unique[item["name"]] = item
    return list(unique.values())


def ask_which_pose(book):
    print("\nSaved positions:")
    for i, item in enumerate(book, start=1):
        print(f"  {i}. {item['name']}")
    print("  q. quit")

    while True:
        choice = input("\nMove to which position? ").strip()
        if choice.lower() in ("q", "quit", "exit"):
            return None

        if choice.isdigit():
            idx = int(choice)
            if 1 <= idx <= len(book):
                return book[idx - 1]
            print(f"Pick a number from 1 to {len(book)}.")
            continue

        matches = [item for item in book if item["name"].lower() == choice.lower()]
        if matches:
            return matches[0]

        print("Not found. Type a number, a pose name, or q.")


print(f"Loading poses from {POSE_FILE}...")
try:
    pose_book = load_pose_book(POSE_FILE)
except FileNotFoundError:
    raise SystemExit(f"No pose file found: {POSE_FILE}\nRun the teach script first.")
except Exception as e:
    raise SystemExit(f"Could not read {POSE_FILE}: {e}")

if not pose_book:
    raise SystemExit("Pose file has no saved positions.")

try:
    servos = {}
    print("Connecting...")
    sample_ids = sorted({sid for item in pose_book for sid in item["positions"]})
    for sid in sample_ids:
        try:
            servo = controller.wrap_servo(sid)
            torque_on(servo)
            set_servo_profile_safely(servo, SPEED, ACCEL)
            curr = servo.sram.read_current_location()
            servos[sid] = servo
            print(f"Servo {sid} ({JOINT_NAMES.get(sid, 'Joint')}): Current = {curr}")
        except ServoNotRespondingError:
            print(f"Servo {sid}: NO RESPONSE")
        except Exception as e:
            print(f"Servo {sid}: Setup Error -> {e}")

    if not servos:
        raise RuntimeError("No active servos available.")

    while True:
        chosen = ask_which_pose(pose_book)
        if chosen is None:
            break

        print_pose(chosen["positions"], f"Moving to {chosen['name']}")
        live_targets = dispatch_targets(servos, chosen["positions"])
        print(f"Sent targets for servos: {sorted(live_targets)}")
        wait_until_still(servos, MOVE_TIMEOUT_S)
        print(f"Arrived at {chosen['name']}.")

finally:
    controller.close()
    print("Connection closed.")