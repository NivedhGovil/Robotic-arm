import json

# ============================================================
# SETTINGS
# ============================================================

JSON_FILE = r"C:\Users\Nivedh Govil\OneDrive\Robotics\motor_limits.json"

# ============================================================
# LOAD JSON FILE
# ============================================================

with open(JSON_FILE, "r") as file:
    MOTOR_LIMITS = json.load(file)


# ============================================================
# PRINT ALL MOTOR VALUES
# ============================================================

print("=" * 60)
print("             LOADED MOTOR LIMITS")
print("=" * 60)

for servo_id, data in MOTOR_LIMITS.items():

    print()
    print(f"Motor ID {servo_id}")

    print(
        f"Minimum : "
        f"{data['minimum']['counts']} counts "
        f"({data['minimum']['degrees']}°)"
    )

    print(
        f"Maximum : "
        f"{data['maximum']['counts']} counts "
        f"({data['maximum']['degrees']}°)"
    )

    print(
        f"Midpoint: "
        f"{data['midpoint']['counts']} counts "
        f"({data['midpoint']['degrees']}°)"
    )


# ============================================================
# EXAMPLES OF ACCESSING INDIVIDUAL VALUES
# ============================================================

# Motor ID 1 midpoint
motor_1_mid = MOTOR_LIMITS["1"]["midpoint"]["counts"]

# Motor ID 1 minimum
motor_1_min = MOTOR_LIMITS["1"]["minimum"]["counts"]

# Motor ID 1 maximum
motor_1_max = MOTOR_LIMITS["1"]["maximum"]["counts"]


print()
print("=" * 60)
print("EXAMPLE: MOTOR 1")
print("=" * 60)

print(f"Minimum : {motor_1_min}")
print(f"Midpoint: {motor_1_mid}")
print(f"Maximum : {motor_1_max}")
import os

print("JSON saved at:")
print(os.path.abspath("motor_limits.json"))
