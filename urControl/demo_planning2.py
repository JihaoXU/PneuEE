import urx
import numpy as np
from func_transform import transform
from socket import socket
import time

# grinding trajectory
# critical parameters (mm)
n = 4  # path number
z = -30  # absolute height
h = 10.0  # relative height
r = 40.0  # x-axis offset
y0 = 97.0 # initial y
a = 10 / 1000.0  # acc, unit: m/s^2
v = 50 / 1000.0  # vel，unit: m/s
lx = 550.0  # x-axis length
ly = 100.0  # y-axis length
d = ly / (n - 1)  # y-axis gap
# zig-zag trajectory
trajectory = np.zeros(shape=(2 * n + 3, 3), dtype=float)
trajectory[0] = np.array([0, 0, h])
trajectory[1] = np.array([r, y0, h])
trajectory[2] = np.array([r, y0, 0])
for i in range(3, 2 * n + 2):
    if i % 2 == 1:
        trajectory[i, 0] = lx - trajectory[i - 1, 0]
        trajectory[i, 1] = trajectory[i - 1, 1]
    else:
        trajectory[i, 0] = trajectory[i - 1, 0]
        trajectory[i, 1] = trajectory[i - 1, 1] + d
trajectory[2 * n + 2, 0] = trajectory[2 * n + 1, 0]
trajectory[2 * n + 2, 1] = trajectory[2 * n + 1, 1]
trajectory[2 * n + 2, 2] = h
# print(trajectory)

# connect ur_robot
rob = urx.Robot("192.168.0.1")
print("connect ur_robot")


# move to p1
print("moving to p1")
pose = transform(trajectory[1], dz=z)
rob.movel(tpose=pose, acc=a, vel=v)
print("position reached: p1")

# main loop
while True:
    # current position: p1
    button = input("conduct grinding ? >> (y/n) ")
    if button in ['q', 'quit', 'break', 'n', 'N']:
        break

    print("enter the process")
    # move to p2
    print("moving to p2")
    pose = transform(trajectory[2], dz=z)
    rob.movel(tpose=pose, acc=a, vel=v)
    print("position reached: p2")

     # start the main grinding trajectory
    for i in range(3, 2 * n + 2):
        print("moving to p%d" % i)
        pose = transform(trajectory[i], dz=z)
        rob.movel(tpose=pose, acc=a, vel=v)
        print("position reached: p%d" % i)

    # move to p(2n+2)
    print("moving to p%d" % (2 * n + 2))
    pose = transform(trajectory[2 * n + 2], dz=z)
    rob.movel(tpose=pose, acc=a, vel=v)
    print("position reached: p%d" % (2 * n + 2))
    # move to p1
    print("moving to p1")
    pose = transform(trajectory[1], dz=z)
    rob.movel(tpose=pose, acc=a, vel=v)
    print("position reached: p1")

print("quit the process")

# move to p0
print("moving to p0")
pose = transform(trajectory[0], dz=z)
rob.movel(tpose=pose, acc=a, vel=v)
print("position reached: p0")

print("close ur_robot TCP")
rob.close()
