import time
from python_st3215 import ST3215

# ---------------- SETTINGS ----------------
PORT = "COM7"
SERVO_IDS = [1, 2, 3, 4, 5, 6]


# ---------------- MAIN ----------------

controller = ST3215(PORT)

try:
    print("\nROBOT ARM TEMPERATURE AND LOAD")
    print("==============================\n")

    for servo_id in SERVO_IDS:

        servo = controller.wrap_servo(servo_id)

        # Read temperature
        temperature = servo.sram.read_current_temperature()

        # Read raw load
        load_raw = servo.sram.read_current_load()

        # Remove direction bit
        load = load_raw & 0x3FF

        # Find direction
        if load_raw & 0x400:
            direction = "CCW"
        else:
            direction = "CW"

        # Convert to percentage
        load_percent = load / 1000 * 100

        print(f"ID {servo_id}")
        print(f"  Temperature : {temperature} °C")
        print(f"  Load        : {load_percent:.1f}% ({direction})")
        print(f"  Raw Load    : {load_raw}")
        print()

        time.sleep(0.2)

finally:
    controller.close()
    print("Port closed.")