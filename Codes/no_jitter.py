import time
from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM7"
SERVO_IDS = [1, 2, 3, 4, 5, 6]

TARGET_CENTER = 2048

# Faster speed profile with smooth acceleration curve
# Speed: 500-700 gives fast, responsive motion without losing steps
FAST_SPEED = 600      # Range: 0-4095
# Acceleration: 20-30 provides a smooth hardware S-curve start/stop
HARDWARE_ACCEL = 25   # Range: 0-254

controller = ST3215(PORT)


def set_servo_profile_safely(servo, speed, accel):
    """Sets hardware speed and acceleration registers directly in SRAM."""
    try:
        if hasattr(servo.sram, "speed"):
            servo.sram.speed = speed
        if hasattr(servo.sram, "acceleration"):
            servo.sram.acceleration = accel
        if hasattr(servo.sram, "moving_speed"):
            servo.sram.moving_speed = speed
        if hasattr(servo.sram, "moving_acceleration"):
            servo.sram.moving_acceleration = accel
    except Exception:
        pass

    try:
        if hasattr(servo.sram, "write_register"):
            servo.sram.write_register(46, speed, 2)
            servo.sram.write_register(47, accel, 1)
        elif hasattr(servo.sram, "set_register"):
            servo.sram.set_register(46, speed)
            servo.sram.set_register(47, accel)
    except Exception:
        pass


print(f"Setting fast hardware acceleration profile -> Center ({TARGET_CENTER})...")

try:
    active_servos = []

    # 1. Configuration Pass: Apply hardware acceleration/speed profiles once
    for sid in SERVO_IDS:
        try:
            servo = controller.wrap_servo(sid)
            servo.sram.write_torque_switch(1)

            # Write hardware velocity/acceleration profiles
            set_servo_profile_safely(servo, FAST_SPEED, HARDWARE_ACCEL)

            curr_pos = servo.sram.read_current_location()
            print(f"Servo {sid}: Start = {curr_pos} -> Target = {TARGET_CENTER}")

            active_servos.append(servo)

        except ServoNotRespondingError:
            print(f"Servo {sid}: NO RESPONSE")
        except Exception as e:
            print(f"Servo {sid}: Setup Error -> {e}")

    # 2. Command Pass: Dispatch target position back-to-back to trigger hardware move
    print("\nExecuting fast, fluid hardware move...")
    for servo in active_servos:
        try:
            servo.sram.write_target_location(TARGET_CENTER)
        except Exception as e:
            print(f"Dispatch Error: {e}")

    # Allow time for fast move to finish
    time.sleep(1.2)

finally:
    controller.close()
    print("Execution complete. Connection closed.")