from python_st3215 import ST3215, ServoNotRespondingError
import time

# ============================================================
# SETTINGS
# ============================================================

PORT = "COM7"

# IDs you want to check
SERVO_IDS = range(1, 11)

# Small movement around the current position
MOVEMENT = 100

# Time between movements
MOVE_TIME = 1.0

# ============================================================
# CONNECT
# ============================================================

controller = ST3215(PORT)

print("=" * 60)
print("             ST3215 / HS3215 ID FINDER")
print("=" * 60)
print()
print("Watch the robotic arm carefully.")
print("The program will test ONE ID at a time.")
print()

try:

    for servo_id in SERVO_IDS:

        print()
        print("=" * 60)
        print(f"                 TESTING ID {servo_id}")
        print("=" * 60)

        try:
            servo = controller.wrap_servo(servo_id)

            # ------------------------------------------------
            # Read current position
            # ------------------------------------------------

            current = servo.sram.read_current_location()

            print(f"ID {servo_id} RESPONDED")
            print(f"Current position: {current}")
            print()
            print(f">>> WATCH WHICH MOTOR MOVES <<<")
            print(f">>> THIS MOTOR IS ID {servo_id} <<<")
            print()

            # ------------------------------------------------
            # Calculate safe test positions
            # ------------------------------------------------

            position_1 = max(0, current - MOVEMENT)
            position_2 = min(4095, current + MOVEMENT)

            # ------------------------------------------------
            # Move one direction
            # ------------------------------------------------

            servo.sram.write_goal_position(position_1)

            time.sleep(MOVE_TIME)

            # ------------------------------------------------
            # Move back
            # ------------------------------------------------

            servo.sram.write_goal_position(position_2)

            time.sleep(MOVE_TIME)

            # ------------------------------------------------
            # Return to starting position
            # ------------------------------------------------

            servo.sram.write_goal_position(current)

            time.sleep(MOVE_TIME)

            print()
            print(f"ID {servo_id} TEST COMPLETE.")
            print(f"If you saw movement, write down: MOTOR = ID {servo_id}")

        except ServoNotRespondingError:

            print(f"ID {servo_id}: NO RESPONSE")
            print("Skipping this ID.")

        except Exception as e:

            print(f"ID {servo_id}: ERROR")
            print(e)

        # Pause before testing next ID
        print()
        print("Next ID in 2 seconds...")
        time.sleep(2)

finally:

    controller.close()

print()
print("=" * 60)
print("                 SCAN COMPLETE")
print("=" * 60)