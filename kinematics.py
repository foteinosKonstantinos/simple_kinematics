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
    '''R^3 --> SO(3)'''
    return scipy.linalg.expm(rot_vec2algebra(x, y, z))

def hom_rottrans2group(axis, theta, origin):
    '''(axis: unitary vector, theta: in radians, origin: origin position) --> SE(3)'''
    vec = (axis[0]*theta, axis[1]*theta, axis[2]*theta)
    rot = rot_vec2group(*vec)
    return np.asmatrix(f"[{rot[0,0]} {rot[0,1]} {rot[0,2]} {origin[0]};{rot[1,0]} {rot[1,1]} {rot[1,2]} {origin[1]};{rot[2,0]} {rot[2,1]} {rot[2,2]} {origin[2]};0 0 0 1]")

# IRB 4600/40 without the D joint and approximating the distance of A joint with 20 cm, all values are in mm (!)
# with ROBOTIQ 2F-140 Gripper for end-effector
# Joint name convention: https://www.cyberbotics.com/doc/guide/irb4600-40?version=R2019b-rev1
# https://library.e.abb.com/public/2de81585a37949b4b1a6b43aaeb79ee3/3HAC032885%20PS%20IRB%204600%20on%20IRC5-en.pdf?x-sign=kEq3EeWxvzlh6yILHMf2ycZTVMQb2VFNue2pEezMygnvEPOM3FiDgij0tbUd4tYR#page=11.23
# https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_TM_InstructionManual_HTML5_20190503.zip/2F-85_2F-140_TM_InstructionManual_HTML5/Content/6.%20Specifications.htm

# Homogeneous transformations

# All joints are revolute
T0A = lambda theta: hom_rottrans2group((0,0,1), theta, (0, 0, 200))
TAB = lambda theta: hom_rottrans2group((0,1,0), theta-np.pi/2, (175, 0, 495-200))
TBC = lambda theta: hom_rottrans2group((0,1,0), theta+np.pi/2, (1095, 0, 0))
# D joint is ignored
TCE = lambda theta: hom_rottrans2group((0,1,0), theta, (1270, 0, 175))
TEF = lambda theta: hom_rottrans2group((1,0,0), theta, (135, 0, 0))
TFG = hom_rottrans2group((0,1,0),-np.pi/2,(160, 0, 0)) # gripper pose
# Allowed angle ranges (based on Webots)
bounds = [
    [-3.13, 3.13],
    [-1.56, 2.61],
    [-3.13, 1.30],
    # D joint is ignored
    [-2.17, 2.08],
    [-6.97, 6.97],
]

def forward_kinematics(a,b,c,e,f):
    return T0A(a) @ TAB(b) @ TBC(c) @ TCE(e) @ TEF(f) @ TFG

def residual(abcef,target,gamma=1000):
    '''gamma: orientation error weight'''
    hom = forward_kinematics(*abcef)
    diff = np.asarray(hom - target)**2
    return gamma * diff[:3,:3].sum() + diff[:3,3].sum()

def penalty(a,b,c,e,f):
    return np.abs(a)+np.abs(b)+np.abs(c)+np.abs(e)+np.abs(f)

def optimize(target, bounds, gamma=1000, delta=1000, a0=0, b0=0, c0=0, e0=0, f0=0):
    '''gamma: orientation error weight, delta: penalty weight'''
    return scipy.optimize.minimize(lambda x:residual(x,target,gamma)+delta*penalty(*x), x0=(a0, b0, c0, e0, f0), bounds=bounds).x

def visualize(ax, group, name="", arrowl=500):
    p = [group[0,3], group[1,3], group[2,3]]
    vecs = []
    for i in range(3):
        vecs.append(p +[group[0, i], group[1, i], group[2, i]])
    X,Y,Z,U,V,W = zip(*vecs)
    ax.quiver(X,Y,Z,U,V,W, length=arrowl, colors=["red", "green", "blue"])
    if name:
        ax.text(x=p[0],y=p[1],z=p[2],s=name)

def visualize_config(a,b,c,e,f,target=None,name=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    if target is not None:
        visualize(ax, target, "{target}")
    T0Aa = T0A(a)
    visualize(ax, T0Aa, "{A}")
    T0Bab = T0Aa @ TAB(b)
    visualize(ax, T0Bab, "{B}")
    T0Cabc = T0Bab @ TBC(c)
    visualize(ax, T0Cabc, "{C}")
    T0Eabce = T0Cabc @ TCE(e)
    visualize(ax, T0Eabce, "{E}")
    T0Fabce = T0Eabce @ TEF(f)
    visualize(ax, T0Fabce, "{F}")
    T0Gabce = T0Fabce @ TFG
    visualize(ax, T0Gabce, "{gripper}")
    ax.set_xlim(-1000, 2000)
    ax.set_ylim(-2000, 2000)
    ax.set_zlim(0, 2000)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.tight_layout()
    plt.savefig(name+".png")
    plt.show()

# TODO: https://matplotlib.org/stable/gallery/widgets/slider_demo.html

if __name__ == "__main__":

    print("FK in zero configuration: ", forward_kinematics(0,0,0,0,0))
    visualize_config(0,0,0,0,0,None,"Zero configuration")

    target = hom_rottrans2group((0, 0, 1), 0.785398, (1752, -299, 1000))
    a,b,c,e,f = optimize(target, bounds, delta=0)
    print("Approach object: ",a,b,c,e,f)
    visualize_config(a,b,c,e,f,target,"Approach object")

    target = hom_rottrans2group((0, 0, 1), 0.785398, (1752, -299, 790))
    a,b,c,e,f = optimize(target, bounds, delta=0)
    print("Grasp object: ",a,b,c,e,f)
    visualize_config(a,b,c,e,f,target,"Grasp object")

    target = hom_rottrans2group((0, 0, 1), 0, (1590, 720, 1400)) # end-effector in zero configuration
    a,b,c,e,f = optimize(target, bounds, delta=0)
    print("Throw object: ",a,b,c,e,f)
    visualize_config(a,b,c,e,f,target,"Throw object")