import time
from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM7"

# Hardware motion profiles for smooth, jerk-free movement
SLOW_SPEED = 200     # Range: 0-4095
SMOOTH_ACCEL = 10    # Range: 0-254

# Recorded Rest Position Mapping
REST_POSITIONS = {
    1: 2048,  # Base: 180.00°
    2: 759,   # Shoulder: 66.71°
    3: 3054,  # Elbow: 268.42°
    4: 1926,  # Wrist Pitch: 169.28°
    5: 3385,  # Wrist Roll: 297.51°
    6: 2301,  # Gripper: 202.24°
}

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


print("Moving robot arm to recorded REST POSITION...")

try:
    active_moves = {}

    # Pass 1: Configure profiles, enable torque, and prepare target mapping
    for sid, target in REST_POSITIONS.items():
        try:
            servo = controller.wrap_servo(sid)
            servo.sram.write_torque_switch(1)
            set_servo_profile_safely(servo, SLOW_SPEED, SMOOTH_ACCEL)

            curr_pos = servo.sram.read_current_location()
            print(f"Servo {sid}: Current = {curr_pos} -> Target Rest = {target}")

            active_moves[sid] = (servo, target)

        except ServoNotRespondingError:
            print(f"Servo {sid}: NO RESPONSE during configuration.")
        except Exception as e:
            print(f"Servo {sid}: Setup Error -> {e}")

    # Pass 2: Dispatch position commands back-to-back for synchronized movement
    print("\nDispatching rest position commands...")
    for sid, (servo, target) in active_moves.items():
        try:
            servo.sram.write_target_location(target)
        except Exception as e:
            print(f"Servo {sid}: Dispatch Error -> {e}")

    # Allow time for internal hardware trajectory to complete
    time.sleep(2.5)

finally:
    controller.close()
    print("Execution complete. Connection closed.")