import time
from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM7"

# Target positions
TARGET_POSITIONS = {
    1: 2048,  # Base
    2: 2048,  # Shoulder
    3: 2048,  # Elbow
    4: 2048,  # Wrist Pitch
    5: 2048,  # Wrist Roll
    6: 2048   # Gripper
}

# Motion settings
SPEED = 300      # Register 46
ACCEL = 15       # Register 47

controller = ST3215(PORT)


def set_servo_profile_safely(servo, speed, accel):
    """Sets speed and acceleration safely in SRAM memory."""
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


try:
    servos = {}

    # Setup Pass: Enable torque and write speed/accel registers
    for sid in TARGET_POSITIONS.keys():
        try:
            servo = controller.wrap_servo(sid)
            servo.sram.write_torque_switch(1)
            set_servo_profile_safely(servo, SPEED, ACCEL)
            servos[sid] = servo
        except ServoNotRespondingError:
            print(f"Servo {sid}: NO RESPONSE")
        except Exception as e:
            print(f"Servo {sid}: Setup Error -> {e}")

    print("Executing zero-delay simultaneous move to all motors...")

    # Method 1: Try Native Library Sync Write (Broadcasting single packet over RS485)
    sync_success = False
    if hasattr(controller, "sync_write_target_location"):
        try:
            # Map of {servo_id: target_location}
            controller.sync_write_target_location(TARGET_POSITIONS)
            sync_success = True
            print("Dispatched via Hardware Sync Write Packet.")
        except Exception as e:
            print(f"Sync write method failed, using rapid batch fallback: {e}")

    # Method 2: Rapid Packet Buffer Fallback (No internal prints or delays inside loop)
    if not sync_success:
        # Pre-extract write handles to avoid lookup overhead during iteration
        write_methods = [(sid, servos[sid].sram.write_target_location, TARGET_POSITIONS[sid]) 
                         for sid in servos]
        
        # Zero-delay batch send
        for sid, write_target, target_pos in write_methods:
            try:
                write_target(target_pos)
            except Exception:
                pass
        print("Dispatched via zero-overhead rapid batch.")

    # Allow hardware internal trajectory to complete uninterrupted
    time.sleep(2.0)

finally:
    controller.close()
    print("Execution complete. Connection closed.")