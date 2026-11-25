import argparse
import sys
import time
from python_st3215 import ST3215, ServoNotRespondingError

# Ensure this matches your active port (COM5 was used in Lerobot)
PORT = "COM7"

CENTER_TARGETS = {
    1: 2057,
    2: 2037,
    3: 1972,
    4: 1969,
    5: 3291,
    6: 2306,
}

REST_TARGETS = {
    1: 2057,
    2: 759,
    3: 869,
    4: 786,
    5: 3291,
    6: 2306,
}

# Fine-Tuned Accuracy Parameters
SLOWER_SPEED = 250           # Slow, controlled movement speed
SMOOTH_ACCEL = 10            # Gradual ramp-up/ramp-down to eliminate vibration
ACCURACY_TOLERANCE = 5       # Tight deadband (within 5 counts ~ 0.43 degrees)
SETTLING_TIME = 0.3          # Pause to allow position settling
STALL_LOAD_THRESHOLD = 850
MIN_STEP_PROGRESS = 1
TIMEOUT_PER_JOINT = 15.0     # Extended timeout for micro-adjustments

controller = ST3215(PORT)


def set_servo_speed_and_accel(servo, speed, accel):
    # Set speed (Register 46) and acceleration (Register 47)
    servo.sram.write_data(46, speed.to_bytes(2, byteorder="little"))
    servo.sram.write_data(47, accel.to_bytes(1, byteorder="little"))


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
    reversed_ids = [6, 5, 4, 3, 2, 1]
    print(f"\nStarting high-precision {mode_name} sequence: {reversed_ids}...\n")

    for sid in reversed_ids:
        try:
            servo = controller.wrap_servo(sid)
            target_pos = targets[sid]
            start_pos = servo.sram.read_current_location()
            dist_initial = abs(start_pos - target_pos)

            if dist_initial <= ACCURACY_TOLERANCE:
                print(
                    f"Joint {sid} ALREADY at target: Current={start_pos} | Target={target_pos}"
                )
                servo.sram.write_torque_switch(1)
                continue

            print(
                f"Moving Joint {sid}: Current={start_pos} -> Target={target_pos}"
            )

            # Apply slow speed and smooth acceleration
            set_servo_speed_and_accel(servo, SLOWER_SPEED, SMOOTH_ACCEL)

            # Enable torque and send initial move command
            servo.sram.write_torque_switch(1)
            servo.sram.write_target_location(target_pos)

            start_time = time.time()
            last_pos = servo.sram.read_current_location()
            stuck_counter = 0

            while True:
                time.sleep(0.05)
                curr_pos = servo.sram.read_current_location()
                curr_load = servo.sram.read_current_load()

                effort = curr_load & 0x03FF
                pos_delta = abs(curr_pos - last_pos)
                dist_to_target = abs(curr_pos - target_pos)

                print(
                    f"\r  ID {sid} | Pos: {curr_pos:4d} | Delta: {pos_delta:3d} | Effort: {effort:3d} | Dist: {dist_to_target:4d}  ",
                    end="",
                )
                sys.stdout.flush()

                # High accuracy arrival check
                if dist_to_target <= ACCURACY_TOLERANCE:
                    time.sleep(SETTLING_TIME)
                    final_pos = servo.sram.read_current_location()
                    if abs(final_pos - target_pos) <= ACCURACY_TOLERANCE:
                        print(
                            f"\n  -> Joint {sid} target reached accurately! Final Pos: {final_pos}\n"
                        )
                        break

                # Stall check
                if (
                    effort > STALL_LOAD_THRESHOLD
                    and pos_delta < MIN_STEP_PROGRESS
                ):
                    stuck_counter += 1
                    if stuck_counter >= 12:
                        emergency_lock_torque(sid)
                else:
                    stuck_counter = max(0, stuck_counter - 1)

                # Timeout check
                if time.time() - start_time > TIMEOUT_PER_JOINT:
                    print(
                        f"\n\n[TIMEOUT] Joint {sid} did not reach precise position in time."
                    )
                    emergency_lock_torque(sid)

                last_pos = curr_pos

        except ServoNotRespondingError:
            print(f"\n[ERROR] Servo {sid} failed to respond during movement.")
            emergency_lock_torque(sid)

    print(f"\n[COMPLETE] {mode_name} finished. Holding arm with torque ON.")
    for sid in range(1, 7):
        try:
            servo = controller.wrap_servo(sid)
            servo.sram.write_torque_switch(1)
        except Exception:
            pass

    controller.close()


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