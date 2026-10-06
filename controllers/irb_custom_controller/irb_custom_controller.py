"""irb_custom_controller controller."""

from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

Amotor = robot.getDevice('A motor')
Bmotor = robot.getDevice('B motor')
Cmotor = robot.getDevice('C motor')
Emotor = robot.getDevice('E motor')
Fmotor = robot.getDevice('F motor')

a, b, c, e, f = -4.987547815814601e-09, 1.0826342348221216, 0.3427160279720831, 2.08, 9.920314781311296e-07

Amotor.setPosition(a)
Bmotor.setPosition(b)
Cmotor.setPosition(c)
Emotor.setPosition(e)
Fmotor.setPosition(f)