from python_st3215 import ST3215, ServoNotRespondingError
import time

# ============================================================
# SETTINGS
# ============================================================

PORT = "COM7"
SERVO_IDS = [1, 2, 3, 4, 5, 6]

# Stores the calibrated zero position of each motor
ZERO_POSITIONS = {}


# ============================================================
# POSITION -> DEGREES
# ============================================================

def position_to_degrees(position):
    return position * 360.0 / 4096.0


# ============================================================
# CONNECT
# ============================================================

controller = ST3215(PORT)

print("=" * 60)
print("        ROBOTIC ARM ZERO POSITION CALIBRATION")
print("=" * 60)

print()
print("Move each joint of the arm to your desired ZERO position.")
print("When the position is correct, press Y.")
print()


try:

    for servo_id in SERVO_IDS:

        print()
        print("------------------------------------------")
        print(f"CALIBRATING MOTOR ID {servo_id}")
        print("------------------------------------------")

        try:

            servo = controller.wrap_servo(servo_id)

            # ----------------------------------------------
            # Read current position
            # ----------------------------------------------

            position = servo.sram.read_current_location()
            angle = position_to_degrees(position)

            print(f"Current encoder position : {position}")
            print(f"Current encoder angle    : {angle:.2f}°")
            print()

            # ----------------------------------------------
            # Manually move arm
            # ----------------------------------------------

            print(">>> MOVE THE ARM MANUALLY TO ITS ZERO POSITION <<<")
            print()

            choice = input(
                "Is the arm at the desired ZERO position? (y/n): "
            )

            if choice.lower() == "y":

                # Read position again after manual movement
                zero_position = servo.sram.read_current_location()

                zero_angle = position_to_degrees(zero_position)

                # Save zero position
                ZERO_POSITIONS[servo_id] = zero_position

                print()
                print("ZERO POSITION SAVED")
                print(f"Motor ID       : {servo_id}")
                print(f"Zero encoder   : {zero_position}")
                print(f"Encoder angle  : {zero_angle:.2f}°")

            else:

                print("Zero position NOT saved.")
                print("Repeat this motor when ready.")

        except ServoNotRespondingError:

            print(f"Motor ID {servo_id}: NO RESPONSE")

        except Exception as e:

            print(f"Motor ID {servo_id}: ERROR")
            print(e)

        time.sleep(0.2)


finally:

    controller.close()


# ============================================================
# PRINT ALL ZERO POSITIONS
# ============================================================

print()
print("=" * 60)
print("             SAVED ZERO POSITIONS")
print("=" * 60)

for servo_id, zero_position in ZERO_POSITIONS.items():

    zero_angle = position_to_degrees(zero_position)

    print(
        f"ID {servo_id} : "
        f"{zero_position} counts "
        f"({zero_angle:.2f}° encoder angle)"
    )

print()
print("Calibration complete.")