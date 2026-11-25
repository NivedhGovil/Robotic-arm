from python_st3215 import ST3215, ServoNotRespondingError

# ============================================================
# SETTINGS
# ============================================================

PORT = "COM5"

SERVO_IDS = {
    1: "BASE",
    2: "JOINT 2",
    3: "JOINT 3",
    4: "JOINT 4",
    5: "JOINT 5",
    6: "JOINT 6"
}

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
print("             ROBOTIC ARM CURRENT POSITIONS")
print("=" * 65)
print()
print("READ-ONLY: No movement commands will be sent.")
print()

try:

    for servo_id, joint_name in SERVO_IDS.items():

        try:

            servo = controller.wrap_servo(servo_id)

            # Read current encoder position
            position = servo.sram.read_current_location()

            # Convert to degrees
            angle = position_to_degrees(position)

            print(
                f"ID {servo_id} | "
                f"{joint_name:<8} | "
                f"Position: {position:4d} | "
                f"Angle: {angle:7.2f}°"
            )

        except ServoNotRespondingError:

            print(
                f"ID {servo_id} | "
                f"{joint_name:<8} | "
                f"NO RESPONSE"
            )

        except Exception as e:

            print(
                f"ID {servo_id} | "
                f"{joint_name:<8} | "
                f"ERROR: {e}"
            )

finally:

    controller.close()

print()
print("=" * 65)
print("Position reading complete.")
print("=" * 65)