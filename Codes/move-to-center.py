import argparse
import sys
import time
from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM5"

# Joint Target Maps
CENTER_TARGETS = {
    1: 2057,
    2: 2037,
    3: 1972,
    4: 1969,
    5: 3291,
    6: 2306,
}

REST_TARGETS = {
    1: 2057,  # Base stays centered
    2: 759,   # Shoulder minimum rest limit
    3: 869,   # Elbow resting fold limit
    4: 786,   # Wrist rest position
    5: 3291,  # Wrist roll neutral
    6: 2306,  # Gripper neutral
}

# Safety Thresholds
STALL_LOAD_THRESHOLD = 850  # Load threshold
MIN_STEP_PROGRESS = 5       # Minimum ticks moved per interval
TIMEOUT_PER_JOINT = 7.0     # Safety timeout per joint
TOLERANCE = 45              # Target proximity deadband

controller = ST3215(PORT)


def disable_all_torque():
    print("[INIT] Disabling torque across all 6 motors...")
    for sid in range(1, 7):
        try:
            servo = controller.wrap_servo(sid)
            servo.sram.write_torque_switch(0)
        except ServoNotRespondingError:
            print(f"Servo {sid} did not respond during torque disable.")


def emergency_lock_torque(stuck_id):
    print(
        f"\n[SAFETY ABORT] Obstruction or deadlock detected on Motor {stuck_id}!"
    )
    print("Locking all motor torques ON to hold arm position.")
    for sid in range(1, 7):
        try:
            servo = controller.wrap_servo(sid)
            servo.sram.write_torque_switch(1)
        except Exception:
            pass
    controller.close()
    sys.exit(f"Terminated safely at Motor {stuck_id}.")


def move_sequence(targets, mode_name):
    # Sequence: Reverse order 6 -> 1
    reversed_ids = [6, 5, 4, 3, 2, 1]
    print(f"\nStarting {mode_name} sequence: {reversed_ids}...\n")

    for sid in reversed_ids:
        try:
            servo = controller.wrap_servo(sid)
            target_pos = targets[sid]
            start_pos = servo.sram.read_current_location()
            dist_initial = abs(start_pos - target_pos)

            # Pre-check: skip if already within target tolerance
            if dist_initial <= TOLERANCE:
                print(
                    f"Joint {sid} is ALREADY at target: Current={start_pos} | Target={target_pos} | Dist={dist_initial}"
                )
                servo.sram.write_torque_switch(1)
                continue

            print(
                f"Moving Joint {sid}: Current={start_pos} -> Target={target_pos}"
            )

            # Enable torque on active joint and command target position
            servo.sram.write_torque_switch(1)
            servo.sram.write_target_location(target_pos)

            time.sleep(1.0)

            start_time = time.time()
            last_pos = servo.sram.read_current_location()
            stuck_counter = 0

            while True:
                time.sleep(0.12)
                curr_pos = servo.sram.read_current_location()
                curr_load = servo.sram.read_current_load()

                # Calculate load magnitude and movement delta
                effort = curr_load & 0x03FF
                pos_delta = abs(curr_pos - last_pos)
                dist_to_target = abs(curr_pos - target_pos)

                print(
                    f"\r  ID {sid} | Pos: {curr_pos:4d} | Delta: {pos_delta:3d} | Effort: {effort:3d} | Dist: {dist_to_target:4d}  ",
                    end="",
                )
                sys.stdout.flush()

                if dist_to_target <= TOLERANCE:
                    print(
                        f"\n  -> Joint {sid} reached destination successfully.\n"
                    )
                    break

                # Stall check
                if (
                    effort > STALL_LOAD_THRESHOLD
                    and pos_delta < MIN_STEP_PROGRESS
                ):
                    stuck_counter += 1
                    if stuck_counter >= 8:
                        emergency_lock_torque(sid)
                else:
                    stuck_counter = max(0, stuck_counter - 1)

                # Timeout check
                if time.time() - start_time > TIMEOUT_PER_JOINT:
                    print(
                        f"\n\n[TIMEOUT] Joint {sid} did not complete move in time."
                    )
                    emergency_lock_torque(sid)

                last_pos = curr_pos

        except ServoNotRespondingError:
            print(f"\n[ERROR] Servo {sid} failed to respond during movement.")
            emergency_lock_torque(sid)

    print(
        f"\n[COMPLETE] {mode_name} finished. Holding arm with torque ON."
    )
    for sid in range(1, 7):
        try:
            servo = controller.wrap_servo(sid)
            servo.sram.write_torque_switch(1)
        except Exception:
            pass

    controller.close()


# CLI Selection
parser = argparse.ArgumentParser(description="Arm pose controller")
parser.add_argument(
    "--mode", choices=["center", "rest"], help="Pose mode to execute"
)
args = parser.parse_args()

mode = args.mode
if not mode:
    print("Select Arm Mode:")
    print("  1) Centering Pose")
    print("  2) Rest Pose")
    choice = input("Enter choice (1 or 2): ").strip()
    mode = "center" if choice == "1" else "rest" if choice == "2" else None
    if not mode:
        controller.close()
        sys.exit("Invalid choice. Exiting.")

try:
    # Always drop torque at script startup
    disable_all_torque()
    time.sleep(0.5)

    if mode == "center":
        move_sequence(CENTER_TARGETS, "CENTERING")
    else:
        move_sequence(REST_TARGETS, "REST")
except KeyboardInterrupt:
    print("\nScript interrupted by user.")
finally:
    controller.close()