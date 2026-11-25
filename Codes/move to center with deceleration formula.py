import time
from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM5"
SERVO_IDS = [1, 2, 3, 4, 5, 6]
TARGET_CENTER = 2048

# Slow + smooth hardware profile
# Speed (reg 46, 2 bytes): steps/s. 50 ≈ 0.73 RPM. 80–150 is slow.
# Accel (reg 41, 1 byte): units of 100 steps/s². 0 = instant. 5–10 = gentle ramp.
SMOOTH_SPEED = 600
SMOOTH_ACCEL = 25
MOVE_TIMEOUT_S = 12.0

controller = ST3215(PORT)


def set_servo_profile_safely(servo, speed, accel):
    """Write speed + accel to the real ST3215 registers."""
    # Named API first
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

    # Raw registers (correct map)
    # 41 = acceleration (1 byte)
    # 46 = running/goal speed (2 bytes)
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
    try:
        if hasattr(servo.sram, "read_moving"):
            return bool(servo.sram.read_moving())
        if hasattr(servo.sram, "read_mobile_sign"):
            return bool(servo.sram.read_mobile_sign())
        if hasattr(servo.sram, "write_register") is False and hasattr(servo.sram, "read_register"):
            return bool(servo.sram.read_register(66, 1))
    except Exception:
        return False
    try:
        if hasattr(servo.sram, "read_register"):
            return bool(servo.sram.read_register(66, 1))
    except Exception:
        return False
    return False


print("Configuring slow S-curve profile...")
try:
    servos = {}
    for sid in SERVO_IDS:
        try:
            servo = controller.wrap_servo(sid)
            if hasattr(servo.sram, "torque_enable"):
                servo.sram.torque_enable()
            else:
                servo.sram.write_torque_switch(1)

            set_servo_profile_safely(servo, SMOOTH_SPEED, SMOOTH_ACCEL)
            curr = servo.sram.read_current_location()
            servos[sid] = servo
            print(f"Servo {sid}: Start = {curr} -> Target = {TARGET_CENTER}")
        except ServoNotRespondingError:
            print(f"Servo {sid}: NO RESPONSE")
        except Exception as e:
            print(f"Servo {sid}: Setup Error -> {e}")

    if not servos:
        raise RuntimeError("No active servos connected.")

    print("\nDispatching slow hardware movement...")
    target_map = {sid: TARGET_CENTER for sid in servos}

    sent = False
    if hasattr(controller, "sync_write_target_location"):
        try:
            controller.sync_write_target_location(target_map)
            sent = True
        except Exception as e:
            print(f"Sync Write error: {e}")

    if not sent and hasattr(controller, "broadcast"):
        try:
            controller.broadcast.sram.sync_write_target_location(target_map)
            sent = True
        except Exception as e:
            print(f"Broadcast sync write error: {e}")

    if not sent:
        for sid, servo in servos.items():
            try:
                servo.sram.write_target_location(TARGET_CENTER)
            except Exception:
                pass

    # Wait until motion finishes (or timeout)
    t0 = time.time()
    while time.time() - t0 < MOVE_TIMEOUT_S:
        if not any(servo_is_moving(s) for s in servos.values()):
            # give the last deceleration a moment
            time.sleep(0.2)
            break
        time.sleep(0.1)
    else:
        print("Timeout waiting for motion to finish.")

    print("\nMovement complete.")
finally:
    controller.close()
    print("Connection closed.")