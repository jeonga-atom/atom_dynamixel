# #!/usr/bin/env python3

import sys
import numpy as np
import math

np.set_printoptions(suppress=True, precision=4, floatmode='fixed')

sys.path.append("/home/atom/Workspaces/RJG/sdk_challenge/ROBOT/ROBOT_SDK/")
from ketirobotsdk.sdk import *

pose = [-638.300, -338.770, 560.740, 45.5, 178.17, 0.41]  # mm, deg

testRobot = Robot()
# 로그상 m1013 드라이버 + 12345 포트로 연결 성공한 상태였음
testRobot.SetRobotConf(M1013, "192.168.137.101", 12345)

ok = testRobot.RobotConnect()
print("RobotConnect:", ok)
if not ok or not testRobot.IsConnected():
    print("NOT CONNECTED")
    sys.exit(1)

def pose_to_matrix(xyzabc):
    """
    xyzabc = [x(mm), y(mm), z(mm), a(deg), b(deg), c(deg)]
    a,b,c는 ZYX(Euler) 순서로 회전한다고 가정.
    """
    x, y, z, a_deg, b_deg, c_deg = xyzabc
    a, b, c = map(math.radians, [a_deg, b_deg, c_deg])

    # 회전행렬 (Rz * Ry * Rz)
    Rz1 = np.array([
        [ math.cos(a), -math.sin(a), 0],
        [ math.sin(a),  math.cos(a), 0],
        [ 0,            0,           1]
    ])
    Ry  = np.array([
        [ math.cos(b), 0,  math.sin(b)],
        [ 0,           1,  0],
        [-math.sin(b), 0,  math.cos(b)]
    ])
    Rz2 = np.array([
        [ math.cos(c), -math.sin(c), 0],
        [ math.sin(c),  math.cos(c), 0],
        [ 0,            0,           1]
    ])

    R = Rz1 @ Ry @ Rz2  # Z-Y-Z (yaw–pitch–yaw)

    # 변환행렬 4x4
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = np.array([x, y, z])/1000.0

    return np.round(T, 4)
# ---- 예시 ----
T = pose_to_matrix(pose).flatten().astype(float).tolist()

testRobot.SetVelocity(2)  # 속도 (%)
testRobot.movel(0, T)
testRobot.WaitMove()

print(np.round(T, 8))
