from python_st3215 import ST3215

PORT = "COM7"
SERVO_ID = 1

controller = ST3215(PORT)

try:
    servo = controller.wrap_servo(SERVO_ID)

    print("\nAVAILABLE SRAM METHODS:\n")

    for item in dir(servo.sram):
        if not item.startswith("_"):
            print(item)

finally:
    controller.close()