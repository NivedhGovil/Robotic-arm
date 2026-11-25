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


def set_all_torque(servos, enabled):
    for sid, servo in servos.items():
        if enabled:
            torque_on(servo)
        else:
            torque_off(servo)


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
    start = time.time()

    while time.time() - start < timeout_s:
        if not any(servo_is_moving(s) for s in servos.values()):
            time.sleep(0.2)
            return True

        time.sleep(0.1)

    print("Timeout waiting for motion to finish.")
    return False


def dispatch_targets(servos, targets):
    live = {
        sid: int(targets[sid])
        for sid in servos
        if sid in targets
    }

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
                servos[sid].sram.write_target_location(int(target))
            except Exception as e:
                print(f"ID {sid} move failed: {e}")


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
        name = JOINT_NAMES[sid]

        if pos is None:
            print(f"  ID {sid} {name:12} NO DATA")
        else:
            print(
                f"  ID {sid} {name:12} "
                f"{pos:5d}  ({pos_to_deg(pos):6.2f}°)"
            )


def safe_name(text):
    cleaned = "".join(
        ch if ch.isalnum() or ch == "_" else "_"
        for ch in text.strip()
    )

    return cleaned.upper() or "POSE"


def load_poses():
    if not os.path.exists(SAVE_FILE):
        return []

    try:
        with open(SAVE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as e:
        print(f"Could not read {SAVE_FILE}: {e}")
        return []

    return data.get("poses", [])


def save_file(poses):
    data = {
        "port": PORT,
        "saved_at": datetime.now().isoformat(timespec="seconds"),
        "rest": REST_POSITIONS,
        "poses": poses,
    }

    with open(SAVE_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)


def find_pose(poses, name):
    name = safe_name(name)

    for pose in poses:
        if pose["name"] == name:
            return pose

    return None


def save_current_pose(poses):
    pose = read_pose(servos)
    print_pose(pose, "Saving")

    name = safe_name(input("Pose name: "))

    if not name:
        return

    existing = find_pose(poses, name)

    record = {
        "name": name,
        "time": datetime.now().isoformat(timespec="seconds"),
        "positions": {
            str(k): int(v)
            for k, v in pose.items()
            if v is not None
        },
    }

    if existing:
        existing.update(record)
        print(f"Pose '{name}' overwritten.")
    else:
        poses.append(record)
        print(f"Pose '{name}' saved.")

    save_file(poses)


def list_poses(poses):
    if not poses:
        print("No saved poses.")
        return

    print("\nSAVED POSES")
    print("===========")

    for i, pose in enumerate(poses, 1):
        print(f"{i}. {pose['name']}")


def edit_pose(poses):
    if not poses:
        print("No saved poses.")
        return

    list_poses(poses)

    name = safe_name(input("\nPose to edit: "))
    pose = find_pose(poses, name)

    if pose is None:
        print("Pose not found.")
        return

    positions = {
        int(k): int(v)
        for k, v in pose["positions"].items()
    }

    print_pose(positions, f"Editing '{name}'")

    print("\nEnter a new encoder value.")
    print("Press ENTER to keep the current value.")
    print("Use 'a 180' to enter an angle.")

    for sid in REST_POSITIONS:
        current = positions.get(sid)

        raw = input(
            f"ID {sid} {JOINT_NAMES[sid]} "
            f"[{current}]: "
        ).strip()

        if raw == "":
            continue

        try:
            if raw.lower().startswith("a "):
                degrees = float(raw[2:].strip())
                positions[sid] = int(
                    round(degrees * 4096 / 360.0)
                ) % 4096
            else:
                positions[sid] = int(raw)

        except ValueError:
            print("Invalid value. Keeping old value.")
            continue

    print_pose(positions, f"\nNew '{name}'")

    confirm = input("\nSave these changes? (y/n): ").strip().lower()

    if confirm.startswith("y"):
        pose["positions"] = {
            str(k): int(v)
            for k, v in positions.items()
        }

        pose["time"] = datetime.now().isoformat(timespec="seconds")

        save_file(poses)
        print(f"Pose '{name}' updated.")
    else:
        print("Changes discarded.")


def delete_pose(poses):
    if not poses:
        print("No saved poses.")
        return

    list_poses(poses)

    name = safe_name(input("\nPose to DELETE: "))
    pose = find_pose(poses, name)

    if pose is None:
        print("Pose not found.")
        return

    confirm = input(
        f"Delete '{name}' permanently? (y/n): "
    ).strip().lower()

    if confirm.startswith("y"):
        poses.remove(pose)
        save_file(poses)
        print(f"Pose '{name}' deleted.")
    else:
        print("Delete cancelled.")


def move_to_pose(poses):
    if not poses:
        print("No saved poses.")
        return

    list_poses(poses)

    name = safe_name(input("\nMove to pose: "))
    pose = find_pose(poses, name)

    if pose is None:
        print("Pose not found.")
        return

    target = {
        int(k): int(v)
        for k, v in pose["positions"].items()
    }

    print(f"\nMoving to '{name}'...")

    # Torque MUST be ON while the servo is moving.
    set_all_torque(servos, True)

    time.sleep(0.05)

    for sid, servo in servos.items():
        try:
            set_servo_profile_safely(
                servo,
                SPEED,
                ACCEL
            )
        except Exception:
            pass

    dispatch_targets(servos, target)

    wait_until_still(
        servos,
        MOVE_TIMEOUT_S
    )

    print("\nPose reached.")

    # Release motors after reaching the pose.
    print("Disabling torque...")
    set_all_torque(servos, False)

    print("Torque OFF. Arm is now in teach mode.")


controller = ST3215(PORT)

try:
    print("Connecting...")

    saved = load_poses()

    if saved:
        print(f"Loaded {len(saved)} saved pose(s).")
    else:
        print("No saved poses yet.")

    servos = {}

    for sid, target in REST_POSITIONS.items():
        try:
            servo = controller.wrap_servo(sid)

            torque_on(servo)
            set_servo_profile_safely(
                servo,
                SPEED,
                ACCEL
            )

            curr = int(
                servo.sram.read_current_location()
            )

            servos[sid] = servo

            print(
                f"Servo {sid}: "
                f"Current = {curr} "
                f"-> Rest = {target}"
            )

        except ServoNotRespondingError:
            print(f"Servo {sid}: NO RESPONSE")

        except Exception as e:
            print(f"Servo {sid}: Setup Error -> {e}")

    if not servos:
        raise RuntimeError("No active servos available.")

    print("\nMoving to REST POSITION...")

    dispatch_targets(
        servos,
        REST_POSITIONS
    )

    wait_until_still(
        servos,
        MOVE_TIMEOUT_S
    )

    print("Rest position reached.")

    print("\nDisabling torque...")
    set_all_torque(servos, False)

    print("\nTeach mode ready.")

    while True:
        print(
            "\n"
            "1  Disable torque\n"
            "2  Enable torque\n"
            "3  Read live pose\n"
            "4  Save current pose\n"
            "5  List saved poses\n"
            "6  Edit saved pose\n"
            "7  Delete saved pose\n"
            "8  Move to saved pose\n"
            "9  Quit"
        )

        choice = input("\nChoice: ").strip().lower()

        if choice == "1":
            set_all_torque(servos, False)

        elif choice == "2":
            set_all_torque(servos, True)

        elif choice == "3":
            print_pose(
                read_pose(servos),
                "LIVE POSE"
            )

        elif choice == "4":
            save_current_pose(saved)

        elif choice == "5":
            list_poses(saved)

        elif choice == "6":
            edit_pose(saved)

        elif choice == "7":
            delete_pose(saved)

        elif choice == "8":
            move_to_pose(saved)

        elif choice in ("9", "q", "quit", "exit"):
            break

finally:
    try:
        set_all_torque(servos, False)
    except Exception:
        pass

    controller.close()
    print("\nConnection closed.")