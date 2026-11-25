from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM7"
controller = ST3215(PORT)

print("Scanning for Servo 5 and 6...")

for test_id in [5, 6]:
    try:
        servo = controller.wrap_servo(test_id)
        pos = servo.sram.read_current_location()
        print(f"-> Found Servo ID {test_id}! Current Position: {pos}")
    except ServoNotRespondingError:
        print(f"-> Servo ID {test_id}: NO RESPONSE")

controller.close()