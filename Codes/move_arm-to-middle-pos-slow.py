import time
from arm_connection import ph, pt, bus
from arm_settings import (
    STALL_LOAD_THRESHOLD,
    MIN_STEP_PROGRESS,
    TIMEOUT_PER_JOINT,
    TOLERANCE
)
from arm_safety import emergency_lock_torque


def move_sequence(targets, mode_name):

    motor_order = [6, 5, 4, 3, 2, 1]

    print(f"\nStarting {mode_name} sequence: {motor_order}\n")

    for sid in motor_order:

        target_pos = targets[sid]

        start_pos = ph.read2ByteTxRx(pt, sid, 56)[0]

        distance = abs(start_pos - target_pos)

        if distance <= TOLERANCE:
            print(f"Motor {sid} already at target.")
            ph.write1ByteTxRx(pt, sid, 40, 1)
            continue

        print(f"Moving Motor {sid}: {start_pos} -> {target_pos}")

        # Speed
        ph.write2ByteTxRx(pt, sid, 46, 1200)

        # Acceleration
        ph.write1ByteTxRx(pt, sid, 47, 50)

        # Torque ON
        ph.write1ByteTxRx(pt, sid, 40, 1)

        # Target position
        ph.write2ByteTxRx(pt, sid, 42, target_pos)

        time.sleep(1.0)

        start_time = time.time()
        last_pos = ph.read2ByteTxRx(pt, sid, 56)[0]

        stuck_counter = 0

        while True:

            time.sleep(0.12)

            current_pos = ph.read2ByteTxRx(pt, sid, 56)[0]
            current_load = ph.read2ByteTxRx(pt, sid, 60)[0]

            load = current_load & 0x03FF

            position_change = abs(current_pos - last_pos)
            distance = abs(current_pos - target_pos)

            print(
                f"\rID {sid} | "
                f"Pos: {current_pos:4d} | "
                f"Delta: {position_change:3d} | "
                f"Load: {load:3d} | "
                f"Distance: {distance:4d}",
                end=""
            )

            # Target reached
            if distance <= TOLERANCE:
                print(f"\nMotor {sid} reached target.\n")
                break

            # Stall detection
            if load > STALL_LOAD_THRESHOLD and position_change < MIN_STEP_PROGRESS:

                stuck_counter += 1

                if stuck_counter >= 8:
                    emergency_lock_torque(sid)

            else:
                stuck_counter = max(0, stuck_counter - 1)

            # Timeout
            if time.time() - start_time > TIMEOUT_PER_JOINT:
                print(f"\nMotor {sid} timed out.")
                emergency_lock_torque(sid)

            last_pos = current_pos

    print(f"\n{mode_name} movement complete.")

    for sid in range(1, 7):
        ph.write1ByteTxRx(pt, sid, 40, 1)

    bus.disconnect()

    