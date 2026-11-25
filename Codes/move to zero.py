import time
from python_st3215 import ST3215, ServoNotRespondingError

# Settings
PORT = "COM7"
SERVO_IDS = [1, 2, 3, 4, 5, 6]
TARGET_POSITION = 0  # Absolute zero position
MOVE_DELAY = 1.0     # Time to wait for movement to complete

# Initialize controller
controller = ST3215(PORT)

print("Moving all 6 motors to 0 position...")

try:
    for servo_id in SERVO_IDS:
        try:
            servo = controller.wrap_servo(servo_id)

            # Command motor to move to position 0
            servo.sram.write_target_location(TARGET_POSITION)
            print(f"ID {servo_id}: Sent command to move to 0")

        except ServoNotRespondingError:
            print(f"ID {servo_id}: NO RESPONSE")
        except Exception as e:
            print(f"ID {servo_id}: ERROR -> {e}")

    # Allow time for physical movement
    time.sleep(MOVE_DELAY)

finally:
    # Clean up serial connection
    controller.close()
    print("Execution complete. Serial connection closed.")