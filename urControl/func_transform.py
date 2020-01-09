import numpy as np


def transform(localPoint, dz=47):
    """
    :param localPoint: ndarray, ndim=3,unit:mm
    :param z:scalar,unit:mm
    :return ur_pose: ndarray,ndim=6,unit:m,rad
    """
    localPoint = localPoint / 1000.0
    dz = dz / 1000.0
    dz = max(-0.1, min(0.1, dz))
    # ====identification results======
    originPoint = np.array([-0.031226046015108557, 0.6978767520441147, 0.19248178487431694])
    originPoint[2] += dz

    rotation = np.array([-2.899979803810399, 1.200649182498406, -0.01974949603012923])

    R = np.array([[0.8896838616704904, 0.45653020585359383, -0.0065419741942945794],
                  [-0.4565433497438151, 0.8897009708267738, -0.000593560942632619],
                  [0.0055494222924608655, 0.0035147764041790087, 0.9999784248967817]])

    # test rotation matrix
    # print(np.dot(R,R.T))

    # ur_position
    position = np.dot(R, localPoint) + originPoint

    # ur_pose
    ur_pose = np.zeros(6)
    ur_pose[0:3] = position
    ur_pose[3:] = rotation

    return ur_pose.tolist()


if __name__ == "__main__":
    print(transform(localPoint=np.array([0, 0, 0])))
