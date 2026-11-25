from python_st3215 import ST3215, ServoNotRespondingError
import time

# ============================================================
# SETTINGS
# ============================================================

# Windows:
PORT = "COM5"

# Linux example:
# PORT = "/dev/ttyUSB0"

# Your 5 servo IDs
SERVO_IDS = [1, 2, 3, 4, 5]

# ST3215 position range
MIN_POSITION = 0
MAX_POSITION = 4095

# ============================================================
# CONVERSION
# ============================================================

def position_to_degrees(position):
    """
    Convert ST3215 encoder position to degrees.

    0    = 0°
    2048 = 180°
    4095 ≈ 360°
    """
    return position * 360.0 / 4096.0


# ============================================================
# CONNECT
# ============================================================

controller = ST3215(PORT)

print("==========================================")
print("      ST3215 MIN/MAX ANGLE TEST")
print("==========================================")
print()

try:

    for servo_id in SERVO_IDS:

        print(f"Testing Servo ID {servo_id}...")
        
        try:
            servo = controller.wrap_servo(servo_id)

            # ------------------------------------------------
            # READ CURRENT POSITION
            # ------------------------------------------------

            current_position = servo.sram.read_current_location()

            print(f"Current position: {current_position}")
            print(
                f"Current angle: "
                f"{position_to_degrees(current_position):.2f}°"
            )

            # ------------------------------------------------
            # DISPLAY THEORETICAL MINIMUM
            # ------------------------------------------------

            print()
            print("Minimum:")
            print(f"Encoder position: {MIN_POSITION}")
            print(
                f"Angle: "
                f"{position_to_degrees(MIN_POSITION):.2f}°"
            )

            # ------------------------------------------------
            # DISPLAY THEORETICAL MAXIMUM
            # ------------------------------------------------

            print()
            print("Maximum:")
            print(f"Encoder position: {MAX_POSITION}")
            print(
                f"Angle: "
                f"{position_to_degrees(MAX_POSITION):.2f}°"
            )

            # ------------------------------------------------
            # RANGE
            # ------------------------------------------------

            print()
            print("Total theoretical rotation:")
            print(
                f"{position_to_degrees(MAX_POSITION):.2f}°"
            )

            print("------------------------------------------")
            print()

        except ServoNotRespondingError:
            print(f"ERROR: Servo {servo_id} did not respond.")
            print("------------------------------------------")
            print()

finally:

    controller.close()

print("Test complete.")