from python_st3215 import ST3215, ServoNotRespondingError
import time

# ============================================================
# SETTINGS
# ============================================================

PORT = "COM7"

SERVO_IDS = [1, 2, 3, 4, 5, 6]


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
print("          MOTOR MINIMUM / MAXIMUM CALIBRATION")
print("=" * 65)

print()
print("For every motor:")
print("1. Move the motor to its MINIMUM physical position.")
print("2. Press ENTER.")
print("3. Move the motor to its MAXIMUM physical position.")
print("4. Press ENTER.")
print()
print("The midpoint will be calculated automatically.")
print()


# ============================================================
# STORE CALIBRATION DATA
# ============================================================

MOTOR_LIMITS = {}


try:

    for servo_id in SERVO_IDS:

        print()
        print("=" * 65)
        print(f"                    MOTOR ID {servo_id}")
        print("=" * 65)

        try:

            servo = controller.wrap_servo(servo_id)

            # ------------------------------------------------
            # MINIMUM
            # ------------------------------------------------

            print()
            print("Move the motor to its MINIMUM physical position.")
            input("Press ENTER when ready...")

            min_position = servo.sram.read_current_location()
            min_angle = position_to_degrees(min_position)

            print()
            print(f"MINIMUM POSITION : {min_position} counts")
            print(f"MINIMUM ANGLE    : {min_angle:.2f}°")


            # ------------------------------------------------
            # MAXIMUM
            # ------------------------------------------------

            print()
            print("Move the motor to its MAXIMUM physical position.")
            input("Press ENTER when ready...")

            max_position = servo.sram.read_current_location()
            max_angle = position_to_degrees(max_position)

            print()
            print(f"MAXIMUM POSITION : {max_position} counts")
            print(f"MAXIMUM ANGLE    : {max_angle:.2f}°")


            # ------------------------------------------------
            # CALCULATE MIDPOINT
            # ------------------------------------------------

            mid_position = (min_position + max_position) / 2
            mid_angle = position_to_degrees(mid_position)


            # ------------------------------------------------
            # STORE DATA IN MEMORY
            # ------------------------------------------------

            MOTOR_LIMITS[servo_id] = {
                "minimum": min_position,
                "maximum": max_position,
                "midpoint": mid_position
            }


            # ------------------------------------------------
            # SHOW RESULT
            # ------------------------------------------------

            print()
            print("------------------------------------------")
            print(f"MOTOR ID {servo_id} CALIBRATION")
            print("------------------------------------------")
            print(f"Minimum : {min_position} counts ({min_angle:.2f}°)")
            print(f"Maximum : {max_position} counts ({max_angle:.2f}°)")
            print(f"Midpoint: {mid_position:.1f} counts ({mid_angle:.2f}°)")
            print("------------------------------------------")


        except ServoNotRespondingError:

            print(f"ID {servo_id}: NO RESPONSE")

        except Exception as e:

            print(f"ID {servo_id}: ERROR")
            print(e)

        time.sleep(0.2)


finally:

    controller.close()


# ============================================================
# PRINT ALL CALIBRATED VALUES
# ============================================================

print()
print("=" * 65)
print("                  CALIBRATED VALUES")
print("=" * 65)

for servo_id, data in MOTOR_LIMITS.items():

    minimum = data["minimum"]
    maximum = data["maximum"]
    midpoint = data["midpoint"]

    print()
    print(f"ID {servo_id}")
    print(f"  Minimum : {minimum} counts ({position_to_degrees(minimum):.2f}°)")
    print(f"  Maximum : {maximum} counts ({position_to_degrees(maximum):.2f}°)")
    print(f"  Midpoint: {midpoint:.1f} counts ({position_to_degrees(midpoint):.2f}°)")


print()
print("=" * 65)
print("                  CALIBRATION COMPLETE")
print("=" * 65)