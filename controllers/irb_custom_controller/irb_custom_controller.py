"""irb_custom_controller controller."""

from controller import Robot
import time

robot = Robot()

timestep = int(robot.getBasicTimeStep())

A_motor = robot.getDevice('A motor')
B_motor = robot.getDevice('B motor')
C_motor = robot.getDevice('C motor')
E_motor = robot.getDevice('E motor')
F_motor = robot.getDevice('F motor')

left_finger = robot.getDevice('ROBOTIQ 2F-140 Gripper::left finger joint')
right_finger = robot.getDevice('ROBOTIQ 2F-140 Gripper::right finger joint')

# for i in range(robot.getNumberOfDevices()):
    # device = robot.getDeviceByIndex(i)
    # print(f"Device {i}: {device.getName()}")

# left_finger.setPosition(0)
left_finger.setPosition(0) # max: 0.7
right_finger.setPosition(0)

first = True
# first2 = True

# delay = 10
# previous = time.time()

print("TODO!!!!!")

while robot.step(timestep) != -1:
    if first:
        a, b, c, e, f = 0.24497865810561265, 0.9233073020562608, -0.1268954651608981, 0.7743844198883113, -0.5404195108630617
        print(timestep)
        A_motor.setPosition(a)
        A_motor.setVelocity(0.1)
        B_motor.setPosition(b)
        B_motor.setVelocity(0.1)
        C_motor.setPosition(c)
        C_motor.setVelocity(0.1)
        E_motor.setPosition(e)
        E_motor.setVelocity(0.1)
        F_motor.setPosition(f)
        F_motor.setVelocity(1)
        print(timestep)
        first = False
    # if first2 and time.time() - previous > delay:
        # first2 = False
        # print("grasp")
        # left_finger.setPosition(0.7)
        # B_motor.setPosition(0)
        # B_motor.setVelocity(0.01)      