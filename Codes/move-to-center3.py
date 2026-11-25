import time
from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM7"
SERVO_IDS = [1, 2, 3, 4, 5, 6]

# Hardware motion profiles for smooth, jerk-free movement
SLOW_SPEED = 200     # Range: 0-4095
SMOOTH_ACCEL = 10    # Range: 0-254

# Stored Center Position (Midpoint of 0-4095 range)
CENTER_POSITION = 2048

controller = ST3215(PORT)


def set_servo_profile_safely(servo, speed, accel):
    """Sets speed and acceleration safely in SRAM memory across library variants."""
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


print(f"Targeting stored Center Position ({CENTER_POSITION} ticks)...")

try:
    active_servos = []

    # Pass 1: Configure motion profiles & read current state
    for sid in SERVO_IDS:
        try:
            servo = controller.wrap_servo(sid)
            servo.sram.write_torque_switch(1)
            set_servo_profile_safely(servo, SLOW_SPEED, SMOOTH_ACCEL)

            curr_pos = servo.sram.read_current_location()
            print(f"Servo {sid}: Current = {curr_pos} -> Target = {CENTER_POSITION}")

            active_servos.append(servo)

        except ServoNotRespondingError:
            print(f"Servo {sid}: NO RESPONSE")
        except Exception as e:
            print(f"Servo {sid}: Setup Error -> {e}")

    # Pass 2: Dispatch commands back-to-back for synchronized movement
    print("\nDispatching center position commands...")
    for servo in active_servos:
        try:
            servo.sram.write_target_location(CENTER_POSITION)
        except Exception as e:
            print(f"Error dispatching command: {e}")

    # Allow time for firmware trajectory to complete
    time.sleep(2.5)

finally:
    controller.close()
    print("Execution complete. Connection closed.")