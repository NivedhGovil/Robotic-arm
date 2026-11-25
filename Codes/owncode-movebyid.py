from python_st3215 import ST3215, ServoNotRespondingError
import time
PORT="COM7"
controller = ST3215(PORT)
ids=range(1,5)
position=0
serid=0
servo.sram.torque_disable()


# askid = int(input("What id do you want to move?"))
# askpos = int(input("What is the target position?"))
# servo = controller.wrap_servo(askid) // moves the motor to the position. 
# servo.sram.write_target_location(askpos)
# print(dir(servo.sram))
# print(dir(servo))

