from python_st3215 import ST3215, ServoNotRespondingError
import time

# ============================================================
# SETTINGS
# ============================================================

PORT = "COM7"

# IDs discovered in your previous test
SERVO_IDS = [1, 2, 3, 4, 5, 6]

# 90 degrees = 1024 encoder counts
ANGLE_90 = 1024

MOVE_TIME = 1.5
PAUSE_TIME = 2.0


# ============================================================
# CONNECT
# ============================================================

controller = ST3215(PORT)

print("=" * 65)
print("             ST3215 / HS3215 MOTOR ID FINDER")
print("=" * 65)
print()
print("Each responding motor will move approximately 90 degrees.")
print("WATCH THE ARM AND WRITE DOWN WHICH PHYSICAL MOTOR MOVES.")
print()
print("IMPORTANT: Keep hands clear of the arm.")
print()


try:

    for servo_id in SERVO_IDS:

        print()
        print("=" * 65)
        print(f"                     ID {servo_id}")
        print("=" * 65)

        try:

            servo = controller.wrap_servo(servo_id)

            # ------------------------------------------------
            # Read current position
            # ------------------------------------------------

            start_position = servo.sram.read_current_location()

            print(f"ID {servo_id} RESPONDED")
            print(f"Starting position: {start_position}")
            print()
            print(f">>> MOTOR ID {servo_id} IS ABOUT TO MOVE <<<")
            print()
            print("Watch carefully!")
            time.sleep(2)

            # ------------------------------------------------
            # Calculate positions
            # ------------------------------------------------

            position_minus_90 = max(0, start_position - ANGLE_90)
            position_plus_90 = min(4095, start_position + ANGLE_90)

            # ------------------------------------------------
            # Move -90 degrees
            # ------------------------------------------------

            print(f"Moving ID {servo_id}: -90 degrees")

            servo.sram.write_target_location(position_minus_90)

            time.sleep(MOVE_TIME)

            # ------------------------------------------------
            # Move +90 degrees
            # ------------------------------------------------

            print(f"Moving ID {servo_id}: +90 degrees")

            servo.sram.write_target_location(position_plus_90)

            time.sleep(MOVE_TIME)

            # ------------------------------------------------
            # Return to original position
            # ------------------------------------------------

            print("Returning to starting position...")

            servo.sram.write_target_location(start_position)

            time.sleep(MOVE_TIME)

            # ------------------------------------------------
            # Read final position
            # ------------------------------------------------

            final_position = servo.sram.read_current_location()

            print()
            print("------------------------------------------")
            print(f"ID {servo_id} TEST COMPLETE")
            print(f"Starting position : {start_position}")
            print(f"Final position    : {final_position}")
            print("------------------------------------------")
            print()

            print("WRITE DOWN WHICH PHYSICAL MOTOR MOVED.")
            print()

        except ServoNotRespondingError:

            print(f"ID {servo_id}: NO RESPONSE")

        except Exception as e:

            print(f"ID {servo_id}: ERROR")
            print(e)

        print(f"Waiting {PAUSE_TIME} seconds before next ID...")
        time.sleep(PAUSE_TIME)


finally:

    controller.close()


print()
print("=" * 65)
print("                    TEST COMPLETE")
print("=" * 65)