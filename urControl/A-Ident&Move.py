import urx
import numpy as np
from func_transform import transform

# move ur_robot
rob = urx.Robot("192.168.0.1")

pose = transform(np.array([60, 96, 0]), dz=-37)

rob.movep(tpose=pose, acc=0.01, vel=0.02, wait=True)
while 1:
    key = input(">>")
    if key == 'q':
        break
    print(rob.getl())
rob.close()
