from python_st3215 import ST3215, ServoNotRespondingError
import time

# ============================================================
# SETTINGS
# ============================================================

# If running Python directly in Windows:
PORT = "COM7"

# If running inside WSL, DO NOT use COM5. 
# You will need the Linux serial device, for example:
# PORT = "/dev/ttyUSB0"

SERVO_IDS = [1, 2, 3, 4, 5, 6]

# ============================================================
# POSITION -> DEGREES
# ============================================================

def position_to_degrees(position):
    return position * 360.0 / 4096.0


# ============================================================
# CONNECT
# ============================================================

print("==========================================")
print("       HS3215 / ST3215 SERVO TEST")
print("==========================================")
print(f"Port: {PORT}")
print()

try:
    controller = ST3215(PORT)
except Exception as e:
    print("FAILED TO OPEN SERIAL PORT")
    print()
    print(f"Error: {e}")
    print()
    print("Check:")
    print("1. Bus Servo Driver is connected.")
    print("2. Correct COM port is being used.")
    print("3. No other program is using the port.")
    print("4. If using WSL, COM5 cannot normally be used directly.")
    raise SystemExit


# ============================================================
# TEST EACH SERVO
# ============================================================

try:

    for servo_id in SERVO_IDS:

        print("------------------------------------------")
        print(f"Testing Servo ID {servo_id}")
        print("------------------------------------------")

        try:
            servo = controller.wrap_servo(servo_id)

            # Read current encoder position
            position = servo.sram.read_current_location()

            # Convert to degrees
            angle = position_to_degrees(position)

            print("STATUS: RESPONDING")
            print(f"Encoder position : {position}")
            print(f"Angle            : {angle:.2f}°")

        except ServoNotRespondingError:

            print("STATUS: NO RESPONSE")
            print(f"Servo ID {servo_id} did not respond.")

        except Exception as e:

            print("STATUS: ERROR")
            print(f"Error: {e}")

        print()

        time.sleep(0.2)

finally:

    controller.close()


print("==========================================")
print("TEST COMPLETE")
print("==========================================")


