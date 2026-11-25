from python_st3215 import ST3215

controller = ST3215("COM8")

try:
    servo = controller.wrap_servo(1)

    print("SRAM methods:")
    print(dir(servo.sram))

finally:
    controller.close()