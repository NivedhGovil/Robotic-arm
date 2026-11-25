import time
from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM5"

# Initialize controller connection
controller = ST3215(PORT)

# Defined servo IDs (1 through 6)
ids = range(1, 7)

for servo_id in ids:
    try:
        # Access the specific servo instance
        servo = controller.wrap_servo(servo_id)

        # Disable torque by writing 0 to the torque switch register
        servo.sram.write_torque_switch(0)
        print(f"Torque disabled for Servo {servo_id}")

    except ServoNotRespondingError:
        print(f"Servo {servo_id} failed to respond.")

# Clean up serial connection when finished
controller.close()