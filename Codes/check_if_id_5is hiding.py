from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM7"
controller = ST3215(PORT)

print("Scanning bus for connected servo IDs (1 to 10)...")

for test_id in range(1, 11):
    try:
        servo = controller.wrap_servo(test_id)
        pos = servo.sram.read_current_location()
        print(f"-> Found Servo ID {test_id}! Current Position: {pos}")
    except ServoNotRespondingError:
        pass

controller.close()
print("Scan complete.")