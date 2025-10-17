import numpy as np

# 출력 형식을 소수점 형태로 설정
np.set_printoptions(precision=6, suppress=True)

xyzabc = [-672.370, -364.240, 559.420, 45.5, 178.17, 0.41]  # deg와 mm

def xyzabc_to_homogeneous(xyzabc):
    x, y, z, a, b, c = xyzabc
    # 각도를 라디안으로 변환
    a_rad = np.radians(a)
    b_rad = np.radians(b)
    c_rad = np.radians(c)
    ca, sa = np.cos(a_rad), np.sin(a_rad)
    cb, sb = np.cos(b_rad), np.sin(b_rad)
    cc, sc = np.cos(c_rad), np.sin(c_rad)
    
    R = np.array([
        [cc*cb, cc*sb*sa - sc*ca, cc*sb*ca + sc*sa],
        [sc*cb, sc*sb*sa + cc*ca, sc*sb*ca - cc*sa],
        [-sb,   cb*sa,            cb*ca]
    ])
    
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]

    T = np.round(T, decimals=5)

    return T

T = xyzabc_to_homogeneous(xyzabc).flatten()
mat_return = [float(x) for x in T]

print("동차변환 된 Matrix:")
print(mat_return)

movel(base, mat_return)  # ref를 base로
movel(tcp, mat_return)   # ref를 tcp로 