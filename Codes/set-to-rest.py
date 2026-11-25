from python_st3215 import ST3215, ServoNotRespondingError
import time

# ============================================================
# SETTINGS
# ============================================================

PORT = "COM7"

SERVO_IDS = [1, 2, 3, 4, 5, 6]

REST_FILE = "rest-position.txt"


# ============================================================
# POSITION -> DEGREES
# ============================================================

def position_to_degrees(position):
    return position * 360.0 / 4096.0


# ============================================================
# CONNECT
# ============================================================

controller = ST3215(PORT)

print("=" * 65)
print("             ROBOT REST POSITION")
print("=" * 65)

print()
print("Move the complete arm manually to the desired")
print("REST POSITION.")
print()


REST_POSITIONS = {}


try:

    input("Press ENTER when the arm is ready...")

    print()
    print("Reading motor positions...")
    print()

    for servo_id in SERVO_IDS:

        try:

            servo = controller.wrap_servo(servo_id)

            # Read current encoder position
            position = servo.sram.read_current_location()

            angle = position_to_degrees(position)

            REST_POSITIONS[servo_id] = {
                "position": position,
                "degrees": angle
            }

            print(
                f"ID {servo_id}: "
                f"{position} counts "
                f"({angle:.2f}°)"
            )

        except ServoNotRespondingError:

            print(f"ID {servo_id}: NO RESPONSE")

        except Exception as e:

            print(f"ID {servo_id}: ERROR")
            print(f"Error: {e}")

        time.sleep(0.2)


finally:

    controller.close()


# ============================================================
# SAVE REST POSITION
# ============================================================



with open(REST_FILE, "w") as file:

    file.write("ROBOT REST POSITION\n")
    file.write("===================\n\n")

    for servo_id, data in REST_POSITIONS.items():

        file.write(f"ID {servo_id}\n")
        file.write(f"Position: {data['position']} counts\n")
        file.write(
            f"Angle: {data['degrees']:.2f} degrees\n\n"
        )


# ============================================================
# COMPLETE
# ============================================================

print()
print("=" * 65)
print("             REST POSITION SAVED")
print("=" * 65)

print()
print(f"File: {REST_FILE}")