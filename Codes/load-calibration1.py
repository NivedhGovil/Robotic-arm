import json

with open("motor_limits.json", "r") as file:
    MOTOR_LIMITS = json.load(file)

# Motor 1 midpoint
motor_1_mid = MOTOR_LIMITS["1"]["midpoint"]["counts"]

# Motor 1 minimum
motor_1_min = MOTOR_LIMITS["1"]["minimum"]["counts"]

# Motor 1 maximum
motor_1_max = MOTOR_LIMITS["1"]["maximum"]["counts"]