import time
from python_st3215 import ST3215

# ---------------- SETTINGS ----------------
PORT = "COM5"
SERVO_IDS = [1, 2, 3, 4, 5, 6]

LOAD_REGISTER = 60
TEMPERATURE_REGISTER = 63


# ---------------- MAIN ----------------

controller = ST3215(PORT)

try:
    print("\nROBOT ARM TEMPERATURE AND LOAD\n")
    print("--------------------------------")

    for servo_id in SERVO_IDS:

        servo = controller.wrap_servo(servo_id)

        # Temperature
        temperature = servo.sram.read_byte(TEMPERATURE_REGISTER)

        # Load
        load_raw = servo.sram.read_word(LOAD_REGISTER)

        # Load magnitude = lower 10 bits
        load = load_raw & 0x3FF

        # Load direction = bit 10
        if load_raw & 0x400:
            direction = "CCW"
        else:
            direction = "CW"

        load_percent = load / 1000 * 100

        print(f"ID {servo_id}")
        print(f"  Temperature : {temperature} °C")
        print(f"  Load        : {load_percent:.1f}% ({direction})")
        print()

        time.sleep(0.2)

finally:
    controller.close()
    print("Port closed.")