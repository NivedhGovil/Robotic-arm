from python_st3215 import ST3215, ServoNotRespondingError
import time

# ============================================================
# SETTINGS
# ============================================================

PORT = "COM7"

SERVO_IDS = {
    1: "BASE",
    2: "JOINT 2",
    3: "JOINT 3",
    4: "JOINT 4",
    5: "JOINT 5",
    6: "JOINT 6"
}

# ============================================================
# POSITION CONVERSION
# ============================================================

def position_to_degrees(position):
    return position * 360.0 / 4096.0


# ============================================================
# CONNECT
# ============================================================

controller = ST3215(PORT)

print("=" * 70)
print("              ROBOTIC ARM POSITION MONITOR")
print("=" * 70)
print()
print("NO MOVEMENT COMMANDS ARE SENT.")
print("The program only READS the current motor positions.")
print()
print("Press Ctrl+C to stop.")
print()


try:

    while True:

        print("\033[2J\033[H", end="")  # Clear terminal screen

        print("=" * 70)
        print("              ROBOTIC ARM POSITION MONITOR")
        print("=" * 70)
        print()
        print("Reading current position...")
        print()

        print(
            f"{'ID':<6}"
            f"{'JOINT':<12}"
            f"{'POSITION':<15}"
            f"{'ANGLE':<15}"
            f"{'STATUS'}"
        )

        print("-" * 70)

        for servo_id, joint_name in SERVO_IDS.items():

            try:

                servo = controller.wrap_servo(servo_id)

                # Read current encoder position
                position = servo.sram.read_current_location()

                # Convert to degrees
                angle = position_to_degrees(position)

                print(
                    f"{servo_id:<6}"
                    f"{joint_name:<12}"
                    f"{position:<15}"
                    f"{angle:>8.2f}°{'':<6}"
                    f"OK"
                )

            except ServoNotRespondingError:

                print(
                    f"{servo_id:<6}"
                    f"{joint_name:<12}"
                    f"{'---':<15}"
                    f"{'---':<15}"
                    f"NO RESPONSE"
                )

            except Exception as e:

                print(
                    f"{servo_id:<6}"
                    f"{joint_name:<12}"
                    f"{'---':<15}"
                    f"{'---':<15}"
                    f"ERROR"
                )

        print()
        print("=" * 70)
        print("Position = encoder value")
        print("Angle    = position × 360 / 4096")
        print("=" * 70)

        time.sleep(0.5)


except KeyboardInterrupt:

    print()
    print()
    print("Position monitor stopped.")


finally:

    controller.close()