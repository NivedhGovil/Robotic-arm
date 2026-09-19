import json
import os
import time
from datetime import datetime
from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM5"
SAVE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "poses.json")

IDS = [1, 2, 3, 4, 5, 6]

JOINT_NAMES = {
    1: "Base",
    2: "Shoulder",
    3: "Elbow",
    4: "Wrist Pitch",
    5: "Wrist Roll",
    6: "Gripper",
}

SPEED = 600
ACCEL = 30
MOVE_TIMEOUT_S = 8.0
POSITION_TOLERANCE = 8
SETTLE_RETRIES = 2

controller = ST3215(PORT)
servos = {}


# ---------------------------------------------------------------- low level

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


def set_torque(servos, enabled):
    for sid, servo in servos.items():
        if enabled:
            torque_on(servo)
            set_servo_profile_safely(servo, SPEED, ACCEL)
        else:
            torque_off(servo)
    print("Torque ON" if enabled else "Torque OFF", "->", ", ".join(str(s) for s in sorted(servos)))


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


def dispatch_targets(servos, targets):
    """Send every target in one shot so the joints start together."""
    live = {sid: int(targets[sid]) for sid in servos if sid in targets}
    sent = False
    if hasattr(controller, "sync_write_target_location"):
        try:
            controller.sync_write_target_location(live)
            sent = True
        except Exception as e:
            print(f"Sync write error: {e}")
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
            except Exception as e:
                print(f"  ID {sid} move failed: {e}")


def position_error(current, target):
    diff = abs(int(current) - int(target))
    return min(diff, 4096 - diff)


def wait_until_still(servos, timeout_s):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if not any(servo_is_moving(s) for s in servos.values()):
            time.sleep(0.2)
            return True
        time.sleep(0.1)
    return False


def wait_until_reached(servos, targets, timeout_s):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        off = []
        for sid, target in targets.items():
            if sid not in servos:
                continue
            try:
                current = int(servos[sid].sram.read_current_location())
            except Exception:
                off.append(sid)
                continue
            if position_error(current, target) > POSITION_TOLERANCE:
                off.append(sid)
        if not off:
            time.sleep(0.15)
            return True
        time.sleep(0.08)
    return False


# ---------------------------------------------------------------- conversion

def pos_to_deg(pos):
    return pos * 360.0 / 4096.0


def deg_to_pos(degrees):
    return int(round(degrees * 4096.0 / 360.0)) % 4096


# ---------------------------------------------------------------- pose data

def read_pose(servos):
    pose = {}
    for sid in IDS:
        if sid not in servos:
            continue
        try:
            pose[sid] = int(servos[sid].sram.read_current_location())
        except Exception as e:
            print(f"  ID {sid} ({JOINT_NAMES[sid]}) read failed: {e}")
    return pose


def print_pose(pose, title="Current pose"):
    print(f"\n{title}")
    print("-" * 55)
    for sid in IDS:
        name = JOINT_NAMES.get(sid, f"ID {sid}")
        pos = pose.get(sid)
        if pos is None:
            print(f"  {sid} {name:12}  NO DATA")
        else:
            print(f"  {sid} {name:12}  {pos:5d}  ({pos_to_deg(pos):6.2f}°)")
    print("-" * 55)


def pose_as_python(pose, name="RECORDED_POSITION"):
    lines = [f"{name} = {{"]
    for sid in IDS:
        pos = pose.get(sid)
        joint = JOINT_NAMES.get(sid, f"ID {sid}")
        if pos is None:
            lines.append(f"    {sid}: None,  # {joint}: no data")
        else:
            lines.append(f"    {sid}: {pos},  # {joint}: {pos_to_deg(pos):.2f}°")
    lines.append("}")
    return "\n".join(lines)


def is_complete_pose(pose):
    missing = [sid for sid in IDS if pose.get(sid) is None]
    if missing:
        print("\nPOSE INCOMPLETE - missing servo IDs:", ", ".join(str(x) for x in missing))
        print("A complete pose requires all 6 servos.")
        return False
    return True


def safe_name(text):
    cleaned = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in str(text).strip())
    return cleaned.upper() or "POSE"


# ---------------------------------------------------------------- pose file

def load_poses(path):
    if not os.path.exists(path):
        return []
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as e:
        print(f"Could not read {path}: {e}")
        return []

    if isinstance(data, dict) and "poses" in data:
        items = data["poses"]
    elif isinstance(data, dict):
        items = [{"name": k, "positions": v} for k, v in data.items()]
    else:
        return []

    loaded = []
    for item in items:
        if not isinstance(item, dict):
            continue
        raw = item.get("positions", {})
        positions = {}
        if isinstance(raw, dict):
            for k, v in raw.items():
                try:
                    positions[int(k)] = None if v is None else int(v)
                except Exception:
                    pass
        loaded.append({
            "name": safe_name(item.get("name", "UNNAMED")),
            "time": item.get("time"),
            "positions": positions,
        })
    return loaded


def write_pose_file(path, saved):
    payload = {
        "port": PORT,
        "saved_at": datetime.now().isoformat(timespec="seconds"),
        "poses": [
            {
                "name": item["name"],
                "time": item.get("time"),
                "positions": {str(k): v for k, v in item["positions"].items()},
            }
            for item in saved
        ],
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    print(f"Saved pose file: {path}")


def upsert_pose(saved, name, pose):
    record = {
        "name": name,
        "time": datetime.now().isoformat(timespec="seconds"),
        "positions": {int(k): int(v) for k, v in pose.items()},
    }
    for i, item in enumerate(saved):
        if item["name"] == name:
            saved[i] = record
            return "updated"
    saved.append(record)
    return "added"


def find_pose(saved, user_input):
    user_input = str(user_input).strip()
    if user_input.isdigit():
        n = int(user_input)
        if 1 <= n <= len(saved):
            return saved[n - 1]
    wanted = safe_name(user_input)
    for item in saved:
        if item["name"] == wanted:
            return item
    return None


def list_poses(saved):
    if not saved:
        print("\nNo saved poses.")
        return
    print("\nSAVED POSES")
    for i, item in enumerate(saved, start=1):
        count = len([v for v in item["positions"].values() if v is not None])
        status = "COMPLETE" if count == len(IDS) else f"INCOMPLETE ({count}/{len(IDS)})"
        print(f"  {i}. {item['name']} [{status}]")


# ---------------------------------------------------------------- actions

def save_current_pose(saved):
    pose = read_pose(servos)
    print_pose(pose, "CURRENT ROBOT POSE")
    if not is_complete_pose(pose):
        print("Pose was NOT saved. Fix communication with all six servos first.")
        return

    raw = input("\nPose name: ").strip()
    if not raw:
        print("Invalid pose name.")
        return
    name = safe_name(raw)

    if find_pose(saved, name) is not None:
        if not input(f"'{name}' exists. Overwrite? (y/n): ").strip().lower().startswith("y"):
            print("Save cancelled.")
            return

    action = upsert_pose(saved, name, pose)
    write_pose_file(SAVE_FILE, saved)
    print(f"Pose '{name}' {action}. File now has {len(saved)} pose(s).")
    print("\nPaste-ready dict:\n")
    print(pose_as_python(pose, name))


def edit_pose(saved):
    if not saved:
        print("No saved poses.")
        return
    list_poses(saved)
    item = find_pose(saved, input("\nPose to EDIT (number or name): ").strip())
    if item is None:
        print("Pose not found.")
        return

    print_pose(item["positions"], f"CURRENT SAVED POSE: {item['name']}")

    print("\nTEACH MODE - releasing torque. Hold the arm so it does not drop.")
    set_torque(servos, False)
    input("Move the arm to the NEW position, then press ENTER to capture...")

    pose = read_pose(servos)
    print_pose(pose, f"NEW POSE: {item['name']}")
    if not is_complete_pose(pose):
        print("The new pose is incomplete. Changes were NOT saved.")
        return

    if input("\nSave this position as the new pose? (y/n): ").strip().lower().startswith("y"):
        upsert_pose(saved, item["name"], pose)
        write_pose_file(SAVE_FILE, saved)
        print(f"Pose '{item['name']}' updated from live encoder values.")
    else:
        print("Changes discarded.")

    set_torque(servos, False)
    print("Torque remains OFF.")


def delete_pose(saved):
    if not saved:
        print("No saved poses.")
        return
    list_poses(saved)
    item = find_pose(saved, input("\nPose to DELETE (number or name): ").strip())
    if item is None:
        print("Pose not found.")
        return
    if input(f"Delete '{item['name']}' permanently? (y/n): ").strip().lower().startswith("y"):
        saved.remove(item)
        write_pose_file(SAVE_FILE, saved)
        print(f"Pose '{item['name']}' deleted.")
    else:
        print("Delete cancelled.")


def move_to_saved_pose(saved):
    if not saved:
        print("No saved poses.")
        return
    list_poses(saved)
    item = find_pose(saved, input("\nMove to pose (number or name): ").strip())
    if item is None:
        print("Pose not found.")
        return

    target = {sid: int(v) for sid, v in item["positions"].items() if v is not None}
    if not is_complete_pose(target):
        print("This pose is incomplete and will NOT be moved.")
        return

    offline = [sid for sid in IDS if sid not in servos]
    if offline:
        print("Cannot move: these servos are not connected:", ", ".join(str(x) for x in offline))
        return

    print_pose(target, f"TARGET POSE: {item['name']}")

    print("\nEnabling torque...")
    set_torque(servos, True)
    time.sleep(0.2)

    reached = False
    for attempt in range(1, SETTLE_RETRIES + 2):
        dispatch_targets(servos, target)
        wait_until_still(servos, MOVE_TIMEOUT_S)
        if wait_until_reached(servos, target, MOVE_TIMEOUT_S):
            reached = True
            break
        current = read_pose(servos)
        worst = max(
            ((sid, position_error(current[sid], target[sid])) for sid in target if sid in current),
            key=lambda p: p[1],
            default=(None, 0),
        )
        print(f"Attempt {attempt}: not on target yet "
              f"(worst: ID {worst[0]} off by {worst[1]} counts). Re-sending targets.")

    final = read_pose(servos)
    print_pose(final, "FINAL ROBOT POSITION")
    for sid in sorted(target):
        if sid in final:
            err = position_error(final[sid], target[sid])
            if err > POSITION_TOLERANCE:
                print(f"  ID {sid} {JOINT_NAMES[sid]:12} off target by {err} counts "
                      f"({pos_to_deg(err):.2f}°)")

    if reached:
        print("\nPose reached. Torque stays ON so the arm holds the pose.")
        print("Use menu option 1 to release it (hold the arm first).")
    else:
        print("\nThe robot did not reach every target within the timeout.")
        print("Torque stays ON - check for a blocked joint or raise MOVE_TIMEOUT_S.")


# ---------------------------------------------------------------- main

print("Connecting...")
try:
    saved = load_poses(SAVE_FILE)
    if saved:
        print(f"Loaded {len(saved)} existing pose(s) from {SAVE_FILE}:")
        for item in saved:
            print(f"  - {item['name']}")
    else:
        print("No existing poses yet. Starting a new list.")

    for sid in IDS:
        try:
            servo = controller.wrap_servo(sid)
            set_servo_profile_safely(servo, SPEED, ACCEL)
            curr = int(servo.sram.read_current_location())
            servos[sid] = servo
            print(f"Servo {sid} ({JOINT_NAMES[sid]}): connected, current = {curr}")
        except ServoNotRespondingError:
            print(f"Servo {sid} ({JOINT_NAMES[sid]}): NO RESPONSE")
        except Exception as e:
            print(f"Servo {sid} ({JOINT_NAMES[sid]}): Setup Error -> {e}")

    if not servos:
        raise RuntimeError("No active servos available.")

    print("\nPose tool ready.")

    while True:
        print(
            "\n"
            "1  Disable torque (teach mode)\n"
            "2  Enable torque\n"
            "3  Read live pose\n"
            "4  Save current pose\n"
            "5  List saved poses\n"
            "6  Edit saved pose (TEACH MODE)\n"
            "7  Delete saved pose\n"
            "8  Move to saved pose\n"
            "9  Quit"
        )
        choice = input("\nChoice: ").strip().lower()

        if choice == "1":
            print("Hold the arm so it does not drop.")
            set_torque(servos, False)
        elif choice == "2":
            set_torque(servos, True)
        elif choice == "3":
            print_pose(read_pose(servos), "LIVE POSE")
        elif choice == "4":
            save_current_pose(saved)
        elif choice == "5":
            list_poses(saved)
        elif choice == "6":
            edit_pose(saved)
        elif choice == "7":
            delete_pose(saved)
        elif choice == "8":
            move_to_saved_pose(saved)
        elif choice in ("9", "q", "quit", "exit"):
            break
        else:
            print("Invalid choice.")

    write_pose_file(SAVE_FILE, saved)
    print(f"\nWrote {len(saved)} pose(s) to {SAVE_FILE}")

finally:
    try:
        controller.close()
    except Exception:
        pass
    print("\nConnection closed.")
