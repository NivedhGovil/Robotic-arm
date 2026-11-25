import time
from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM7"

REST_POSITIONS = {
    1: 2048,  # Base: 180.00°
    2: 759,   # Shoulder: 66.71°
    3: 3054,  # Elbow: 268.42°
    4: 1926,  # Wrist Pitch: 169.28°
    5: 3385,  # Wrist Roll: 297.51°
    6: 2301,  # Gripper: 202.24°
}

# Faster, still fluid
# Accel = reg 41 (1 byte), unit 100 steps/s²
# Speed = reg 46 (2 bytes). Do NOT write accel to reg 47.
SPEED = 600
ACCEL = 30
MOVE_TIMEOUT_S = 8.0

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


print("Preparing fluid S-curve move to REST POSITION...")

try:
    servos = {}

    for sid, target in REST_POSITIONS.items():
        try:
            servo = controller.wrap_servo(sid)
            if hasattr(servo.sram, "torque_enable"):
                servo.sram.torque_enable()
            else:
                servo.sram.write_torque_switch(1)

            set_servo_profile_safely(servo, SPEED, ACCEL)
            curr_pos = servo.sram.read_current_location()
            servos[sid] = servo
            print(f"Servo {sid}: Current = {curr_pos} -> Rest = {target}")

        except ServoNotRespondingError:
            print(f"Servo {sid}: NO RESPONSE during initialization.")
        except Exception as e:
            print(f"Servo {sid}: Setup Error -> {e}")

    if not servos:
        raise RuntimeError("No active servos available.")

    print("\nDispatching synchronized rest targets...")
    live_targets = {sid: REST_POSITIONS[sid] for sid in servos}

    sent = False
    if hasattr(controller, "sync_write_target_location"):
        try:
            controller.sync_write_target_location(live_targets)
            sent = True
        except Exception as e:
            print(f"Sync Write error: {e}")

    if not sent and hasattr(controller, "broadcast"):
        try:
            controller.broadcast.sram.sync_write_target_location(live_targets)
            sent = True
        except Exception as e:
            print(f"Broadcast sync write error: {e}")

    if not sent:
        for sid, target in live_targets.items():
            try:
                servos[sid].sram.write_target_location(target)
            except Exception:
                pass

    t0 = time.time()
    while time.time() - t0 < MOVE_TIMEOUT_S:
        if not any(servo_is_moving(s) for s in servos.values()):
            time.sleep(0.2)
            break
        time.sleep(0.1)
    else:
        print("Timeout waiting for motion to finish.")

    print("\nFluid movement to Rest Position complete!")

finally:
    controller.close()
    print("Execution complete. Connection closed.")