import math
import time
from python_st3215 import ST3215, ServoNotRespondingError

PORT = "COM7"
SERVO_IDS = [1, 2, 3, 4, 5, 6]

TARGET_CENTER = 2048

# Trajectory Settings
TOTAL_DURATION = 1.5  # Seconds for total travel time (faster & smooth)
UPDATE_HERTZ = 15     # Low frequency (15 Hz) prevents RS485 bus congestion

controller = ST3215(PORT)


def s_curve_step(progress):
    """
    S-Curve (Fluid Acceleration/Deceleration) Formula.
    progress: Float from 0.0 (start) to 1.0 (end).
    Formula: 0.5 * (1 - cos(pi * progress))
    """
    return 0.5 * (1.0 - math.cos(math.pi * progress))


print("Executing S-Curve Fluid Movement to 2048 ticks...")

try:
    servos = {}
    start_positions = {}

    # 1. Read current starting positions for all servos
    for sid in SERVO_IDS:
        try:
            servo = controller.wrap_servo(sid)
            servo.sram.write_torque_switch(1)

            curr = servo.sram.read_current_location()
            start_positions[sid] = curr
            servos[sid] = servo

            print(f"Servo {sid}: Start = {curr} -> Target = {TARGET_CENTER}")
        except ServoNotRespondingError:
            print(f"Servo {sid}: NO RESPONSE")
        except Exception as e:
            print(f"Servo {sid}: Setup Error -> {e}")

    if not servos:
        raise RuntimeError("No active servos available.")

    print("\nRunning S-Curve Trajectory...")

    total_steps = int(TOTAL_DURATION * UPDATE_HERTZ)
    step_time = 1.0 / UPDATE_HERTZ

    # 2. Execute smooth S-Curve loop
    for step in range(1, total_steps + 1):
        loop_start = time.time()

        # Calculate progress from 0.0 to 1.0
        progress = step / float(total_steps)

        # Apply S-curve fluid formula factor
        fluid_factor = s_curve_step(progress)

        # Update each servo along the fluid curve
        for sid, servo in servos.items():
            start_pos = start_positions[sid]
            
            # Interpolated position calculation
            next_pos = int(start_pos + (TARGET_CENTER - start_pos) * fluid_factor)
            next_pos = min(max(next_pos, 0), 4095)

            try:
                servo.sram.write_target_location(next_pos)
            except Exception:
                pass

        # Maintain loop timing
        elapsed = time.time() - loop_start
        if step_time > elapsed:
            time.sleep(step_time - elapsed)

    print("\nS-Curve fluid movement completed successfully!")

finally:
    controller.close()
    print("Connection closed.")