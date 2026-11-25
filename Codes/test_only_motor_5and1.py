import time
from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM7"  # Ensure correct port
TEST_ID = 5  # Change to 1 to test Servo 1

controller = ST3215(PORT)

print(f"Testing Servo ID {TEST_ID}...")

try:
    servo = controller.wrap_servo(TEST_ID)
    pos = servo.sram.read_current_location()
    print(f"SUCCESS: Servo {TEST_ID} is ONLINE! Position = {pos}")
except ServoNotRespondingError:
    print(
        f"FAILED: Servo {TEST_ID} is not responding. Check wiring or ID settings."
    )
finally:
    controller.close()