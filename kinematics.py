import numpy as np
import scipy.linalg
import scipy.optimize
import matplotlib.pyplot as plt

# https://en.wikipedia.org/wiki/3D_rotation_group
# "Modern robotics"

# 3D rotations

def rot_vec2algebra(x, y, z):
    '''R^3 --> so(3)'''
    return np.asmatrix(f"[0 {-z} {y}; {z} 0 {-x}; {-y} {x} 0]")

def rot_vec2group(x, y, z):
    '''so(3) --> SO(3)'''
    return scipy.linalg.expm(rot_vec2algebra(x, y, z))

def hom_rottrans2group(axis, theta, origin):
    '''axis = unitary vector, origin = translation --> SE(3)'''
    vec = (axis[0]*theta, axis[1]*theta, axis[2]*theta)
    rot = rot_vec2group(*vec)
    return np.asmatrix(f"[{rot[0,0]} {rot[0,1]} {rot[0,2]} {origin[0]};{rot[1,0]} {rot[1,1]} {rot[1,2]} {origin[1]};{rot[2,0]} {rot[2,1]} {rot[2,2]} {origin[2]};0 0 0 1]")

# vec = (0, 0, 1) # x axis
# theta = np.pi/2 # rad

# hom = hom_rottrans2group(vec, theta, (1,2,3))

# IRB 4600/40 without the D joint and approximating the distance of A joint, all values are in mm
# Joint name convention: https://www.cyberbotics.com/doc/guide/irb4600-40?version=R2019b-rev1
# https://library.e.abb.com/public/2de81585a37949b4b1a6b43aaeb79ee3/3HAC032885%20PS%20IRB%204600%20on%20IRC5-en.pdf?x-sign=kEq3EeWxvzlh6yILHMf2ycZTVMQb2VFNue2pEezMygnvEPOM3FiDgij0tbUd4tYR#page=11.23

T0A = lambda theta:hom_rottrans2group((0,0,1), theta, (0, 0, 200))                  # A joint angle
TAB = lambda theta:hom_rottrans2group((0,1,0), theta-np.pi/2, (175, 0, 495))        # B joint anlge
TBC = lambda theta:hom_rottrans2group((0,1,0), theta+np.pi/2, (1095, 0, 0))         # C joint angle
TCE = lambda theta:hom_rottrans2group((0,1,0), theta, (1270, 0, 0))                 # E joint angle
TEF = lambda theta:hom_rottrans2group((1,0,0), theta, (135, 0, 0))                  # F joint angle

bounds = [
    [-3.13, 3.13],
    [-1.56, 2.61],
    [-3.13, 1.30],
    [-2.17, 2.08],
    [-6.97, 6.97],
]

T0F = hom_rottrans2group((0, 0, 0), 0, (1200, 0, 0))                # target

def forward_kinematics(a,b,c,e,f):
    return T0A(a) @ TAB(b) @ TBC(c) @ TCE(e) @ TEF(f)

# print(forward_kinematics(0,0,0,0,0))
# print(forward_kinematics(np.pi,0,0,0,0))
# print(forward_kinematics(0,np.pi/2,0,0,0))

def residual(abcef,target):
    hom = forward_kinematics(*abcef)
    diff = np.asarray(hom - target)
    return (diff ** 2).sum()

# def penalty(a,b,c,e,f):
#     return np.abs(a)+np.abs(b)+np.abs(c)+np.abs(e)+np.abs(f)

def optimize(target, bounds, a0=0, b0=0, c0=0, e0=0, f0=0):
    return scipy.optimize.minimize(lambda x:residual(x,target)#+penalty(*x)
                                   , x0=(a0, b0, c0, e0, f0), bounds=bounds).x

def visualize(group, name=""):
    # in meters
    p = [group[0,3], group[1,3], group[2,3]]
    vecs = []
    for i in range(3):
        vecs.append(p +[group[0, i], group[1, i], group[2, i]])
    X,Y,Z,U,V,W = zip(*vecs)
    ax.quiver(X,Y,Z,U,V,W, length=500, colors=["red", "green", "blue"])
    if name:
        ax.text(x=p[0],y=p[1],z=p[2],s=name)

# TODO: https://matplotlib.org/stable/gallery/widgets/slider_demo.html

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# a = 0
# b = -np.pi/2
# c = 0
# e = np.pi/2
# # d ignored
# f = np.pi/2
a,b,c,e,f = optimize(T0F, bounds)
print(a,b,c,e,f)

visualize(T0F, "{target}")
visualize(T0A(a), "{A}")
visualize(T0A(a) @ TAB(b), "{B}")
visualize(T0A(a) @ TAB(b) @ TBC(c), "{C}")
visualize(T0A(a) @ TAB(b) @ TBC(c) @ TCE(e), "{E}")
visualize(T0A(a) @ TAB(b) @ TBC(c) @ TCE(e) @ TEF(f), "{F}")

ax.set_xlim(-2000, 2000)
ax.set_ylim(-2000, 2000)
ax.set_zlim(0, 2000)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()