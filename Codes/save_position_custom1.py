import json
import os
import time
from datetime import datetime
from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM5"
SAVE_FILE = "recorded_poses.json"

REST_POSITIONS = {
    1: 2048,  # Base: 180.00°
    2: 759,   # Shoulder: 66.71°
    3: 3054,  # Elbow: 268.42°
    4: 1926,  # Wrist Pitch: 169.28°
    5: 3385,  # Wrist Roll: 297.51°
    6: 2301,  # Gripper: 202.24°
}

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
    live = {sid: targets[sid] for sid in servos if sid in targets}
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


def pos_to_deg(pos):
    return pos * 360.0 / 4096.0


def read_pose(servos):
    pose = {}
    for sid, servo in servos.items():
        try:
            pose[sid] = int(servo.sram.read_current_location())
        except Exception:
            pose[sid] = None
    return pose


def print_pose(pose, title="Current pose"):
    print(f"\n{title}")
    for sid in REST_POSITIONS:
        pos = pose.get(sid)
        name = JOINT_NAMES.get(sid, f"ID {sid}")
        if pos is None:
            print(f"  {sid} {name:12}  NO DATA")
        else:
            print(f"  {sid} {name:12}  {pos:5d}  ({pos_to_deg(pos):6.2f}°)")


def pose_as_python(pose, name="RECORDED_POSITION"):
    lines = [f"{name} = {{"]
    for sid in REST_POSITIONS:
        pos = pose.get(sid)
        joint = JOINT_NAMES.get(sid, f"ID {sid}")
        if pos is None:
            lines.append(f"    {sid}: None,  # {joint}: no data")
        else:
            lines.append(f"    {sid}: {pos},  # {joint}: {pos_to_deg(pos):.2f}°")
    lines.append("}")
    return "\n".join(lines)


def safe_name(text):
    cleaned = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in text.strip())
    return cleaned.upper() or "POSE"


def load_existing_poses(path):
    if not os.path.exists(path):
        return []
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as e:
        print(f"Could not read {path}: {e}")
        return []

    loaded = []
    for item in data.get("poses", []):
        name = safe_name(str(item.get("name", "UNNAMED")))
        raw = item.get("positions", {})
        positions = {}
        for k, v in raw.items():
            try:
                positions[int(k)] = None if v is None else int(v)
            except Exception:
                pass
        loaded.append({
            "name": name,
            "time": item.get("time"),
            "positions": positions,
        })
    return loaded


def upsert_pose(saved, name, pose):
    record = {
        "name": name,
        "time": datetime.now().isoformat(timespec="seconds"),
        "positions": pose,
    }
    for i, item in enumerate(saved):
        if item["name"] == name:
            saved[i] = record
            return "updated"
    saved.append(record)
    return "added"


def delete_pose(saved, target):
    """Deletes a pose by 1-based index or by exact name."""
    target_clean = target.strip()
    
    # Try deleting by index
    if target_clean.isdigit():
        idx = int(target_clean) - 1
        if 0 <= idx < len(saved):
            removed = saved.pop(idx)
            return removed["name"]
        return None

    # Try deleting by name
    target_name = safe_name(target_clean)
    for i, item in enumerate(saved):
        if item["name"] == target_name:
            removed = saved.pop(i)
            return removed["name"]

    return None


def write_pose_file(path, saved):
    payload = {
        "port": PORT,
        "saved_at": datetime.now().isoformat(timespec="seconds"),
        "rest": REST_POSITIONS,
        "poses": saved,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


print("Connecting...")
try:
    saved = load_existing_poses(SAVE_FILE)
    if saved:
        print(f"Loaded {len(saved)} existing pose(s) from {SAVE_FILE}:")
        for item in saved:
            print(f"  - {item['name']}")
    else:
        print("No existing poses yet. Starting a new list.")

    servos = {}
    for sid, target in REST_POSITIONS.items():
        try:
            servo = controller.wrap_servo(sid)
            torque_on(servo)
            set_servo_profile_safely(servo, SPEED, ACCEL)
            curr = servo.sram.read_current_location()
            servos[sid] = servo
            print(f"Servo {sid}: Current = {curr} -> Rest = {target}")
        except ServoNotRespondingError:
            print(f"Servo {sid}: NO RESPONSE")
        except Exception as e:
            print(f"Servo {sid}: Setup Error -> {e}")

    if not servos:
        raise RuntimeError("No active servos available.")

    print("\nMoving to REST POSITION...")
    dispatch_targets(servos, REST_POSITIONS)
    wait_until_still(servos, MOVE_TIMEOUT_S)
    print("Rest position reached.")

    print("\nReleasing torque. Hold the arm so it does not drop.")
    for servo in servos.values():
        torque_off(servo)

    print("\nTeach mode ready.")
    print("Move the arm, type a name, press Enter.")
    print("Commands:")
    print("  <name>             save / overwrite that pose")
    print("  l                  print live pose")
    print("  list               show all saved names")
    print("  del <name/index>   delete a pose (e.g., 'del 1' or 'del POSE_NAME')")
    print("  q                  quit and write file")

    while True:
        raw = input("\nteach> ").strip()
        if not raw:
            continue

        cmd = raw.lower()
        parts = raw.split(maxsplit=1)

        if cmd in ("q", "quit", "exit"):
            break

        if cmd in ("l", "live", "p", "print"):
            print_pose(read_pose(servos), "Live pose")
            continue

        if cmd == "list":
            if not saved:
                print("No poses saved yet.")
            else:
                for i, item in enumerate(saved, start=1):
                    print(f"  {i}. {item['name']}")
            continue

        # Handle Delete Command
        if parts[0].lower() in ("del", "delete", "remove", "rm"):
            if len(parts) < 2:
                print("Usage: del <number> or del <pose_name>")
            else:
                removed_name = delete_pose(saved, parts[1])
                if removed_name:
                    write_pose_file(SAVE_FILE, saved)
                    print(f"Deleted pose '{removed_name}'. File now has {len(saved)} pose(s).")
                else:
                    print(f"Could not find pose '{parts[1]}'. Use 'list' to see valid poses.")
            continue

        # Save / Overwrite Pose
        pose = read_pose(servos)
        name = safe_name(raw)
        action = upsert_pose(saved, name, pose)
        write_pose_file(SAVE_FILE, saved)
        print_pose(pose, f"{action.title()} '{name}'")
        print(f"File now has {len(saved)} pose(s).")
        print("\nPaste-ready dict:\n")
        print(pose_as_python(pose, name))

    write_pose_file(SAVE_FILE, saved)
    print(f"\nWrote {len(saved)} pose(s) to {SAVE_FILE}")
    for item in saved:
        print(f"  - {item['name']}")

finally:
    controller.close()
    print("Connection closed.")