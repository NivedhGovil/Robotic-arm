from python_st3215 import ST3215, ServoNotRespondingError
import time

PORT = "COM7"
SERVO_IDS = [1, 2, 3, 4, 5, 6]

controller = ST3215(PORT)

try:

    for servo_id in SERVO_IDS:

        print(f"Testing ID {servo_id}...")

        try:
            servo = controller.wrap_servo(servo_id)

            position = servo.sram.read_current_location()

            print(f"ID {servo_id}: OK")
            print(f"Position: {position}")

        except ServoNotRespondingError:
            print(f"ID {servo_id}: NO RESPONSE")

        except Exception as e:
            print(f"ID {servo_id}: ERROR")
            print(e)

        time.sleep(0.2)

finally:

    controller.close()