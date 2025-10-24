#!/usr/bin/env python3

# CPU 점유율이 너무 높아 부담되면 sleep(0.01) 정도를 넣음 

import json, os, math
import numpy as np
from pathlib import Path

# 입력/출력
RESULT_PATH = Path("result.json")      # [x, y, z, angle, class]
INTRINSICS_PATH = Path("intrinsics.json")
ROBOT_PATH  = Path("robot.json")       # 1x16 플랫 4x4 행렬 저장

AruCo_x = 0.031041595619171847  # 2025 RB10 예시값
AruCo_y = 0.07023097062483427
AruCo_z = 0.0  # 캘리브레이션 값을 0으로 설정 -> 왜냐면 실제 측정값 + 캘리브레이션 값이 되기 때문에 하나만 써야함

#!/usr/bin/env python3
import json, os, math
import numpy as np
from pathlib import Path

# 입력/출력
RESULT_PATH = Path("result.json")      # [x, y, z, angle, class]
INTRINSICS_PATH = Path("intrinsics.json")
ROBOT_PATH  = Path("robot.json")       # 1x16 플랫 4x4 행렬 저장

# ----- (참고) 아루코 오프셋은 단순식에 이미 녹여 쓰므로 T 행렬은 사용 안 함 -----
# T_cam_to_robot 는 더 이상 좌표 변환에 사용하지 않음 (삭제/주석)
# -------------------------------------------------------------------------------

def is_finite_number(x):
    try:
        return isinstance(x, (int, float)) and math.isfinite(float(x))
    except Exception:
        return False

def load_json_list5(path: Path):
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if (isinstance(data, list) and len(data) == 5 and
            all(is_finite_number(v) for v in data[:4]) and
            (isinstance(data[4], (int, float)) and float(data[4]).is_integer())):
            data[4] = int(data[4])
            return data
    except Exception:
        pass
    return None

def load_intrinsics(path: Path):
    try:
        intr = json.loads(path.read_text(encoding="utf-8"))
        req = ("ppx","ppy","fx","fy")
        if all(k in intr for k in req):
            vals = [intr[k] for k in req]
            if all(is_finite_number(v) for v in vals):
                ppx, ppy, fx, fy = map(float, vals)
                if fx != 0.0 and fy != 0.0:
                    return ppx, ppy, fx, fy
    except Exception:
        pass
    return None

def rotz(deg: float):
    r = np.deg2rad(deg); c, s = np.cos(r), np.sin(r)
    return np.array([[ c,-s, 0],
                     [ s, c, 0],
                     [ 0, 0, 1]], dtype=float)

def make_T(R, t):
    T = np.eye(4, dtype=float)
    T[:3,:3] = R
    T[:3,  3] = t
    return T

# --- 간단식용: 픽셀→카메라(mm) + 로봇(m) 좌표 계산 ---
def pixel_to_cam_mm(u, v, ppx, ppy, fx, fy, z_val):
    """
    u,v: 픽셀 좌표(컬러/정렬 기준)
    z_val: m 또는 mm (10 미만이면 m로 간주)
    return: Xc_mm, Yc_mm, Zc_mm (모두 mm)
    """
    Zc_mm = float(z_val)
    if Zc_mm < 10.0:        # RealSense get_distance()처럼 m로 들어오면 mm로 변환
        Zc_mm *= 1000.0
    Xc_mm = (float(u) - ppx) / fx * Zc_mm
    Yc_mm = (float(v) - ppy) / fy * Zc_mm
    return Xc_mm, Yc_mm, Zc_mm

def compute_pose(result_vec, intrinsics):
    """
    간단 캘리브레이션(네가 쓰던 point()식):
      rob_x = (-Yc + 69.9350)/1000
      rob_y = (-Xc + 32.3049)/1000
      rob_z = (-Zc + 142.4168)/1000
    + 회전은 카메라에서 측정한 angle을 그대로 Rz로 사용
    """
    u_pix, v_pix, z_cam, angle_deg, cls_id = result_vec
    ppx, ppy, fx, fy = intrinsics

    # 1) 픽셀→카메라(mm)
    Xc_mm, Yc_mm, Zc_mm = pixel_to_cam_mm(u_pix, v_pix, ppx, ppy, fx, fy, z_cam)

    # 2) 네가 쓰던 간단 보정식 (mm → m)
    rob_x = (-Yc_mm + AruCo_x)   / 1000.0
    rob_y = (-Xc_mm + AruCo_y)   / 1000.0
    rob_z = (-Zc_mm + AruCo_z)  / 1000.0

    # (필요시 미세오프셋: CAL_DX/DY/DZ_mm 더하고 /1000.0 해도 됨)

    # 3) 회전은 Z(yaw)만 적용
    Rz = rotz(float(angle_deg))

    # 4) 4x4 행렬 구성
    T_target = make_T(Rz, np.array([rob_x, rob_y, rob_z], dtype=float))
    return T_target.flatten().tolist(), int(cls_id)

# --------------- 메인 루프 ---------------
def main():
    print("[watch] 시작")
    last_result_mtime = last_intr_mtime = None
    last_result_value = last_intr_value = None

    # intrinsics 준비
    while True:
        try:
            st = os.stat(INTRINSICS_PATH)
        except FileNotFoundError:
            continue
        intr = load_intrinsics(INTRINSICS_PATH)
        if intr is not None:
            last_intr_mtime = st.st_mtime
            last_intr_value = intr
            print("[ok] intrinsics를 불러오는 중 ...:", intr)
            break

    # 변경 감지 루프
    while True:
        try:
            st_i = os.stat(INTRINSICS_PATH)
            if last_intr_mtime is None or st_i.st_mtime != last_intr_mtime:
                intr = load_intrinsics(INTRINSICS_PATH)
                if intr is not None:
                    last_intr_mtime = st_i.st_mtime
                    if intr != last_intr_value:
                        last_intr_value = intr
                        print("[update] intrinsics.json 바뀜! :", intr)
        except FileNotFoundError:
            pass

        try:
            st_r = os.stat(RESULT_PATH)
            if last_result_mtime is None or st_r.st_mtime != last_result_mtime:
                data = load_json_list5(RESULT_PATH)
                if data is not None:
                    last_result_mtime = st_r.st_mtime
                    if data != last_result_value:
                        last_result_value = data
                        print("[update] result.json 바뀜!:", data)
        except FileNotFoundError:
            pass

        if last_result_value and last_intr_value:
            matrix16, cls_id = compute_pose(last_result_value, last_intr_value)
            out = matrix16 + [cls_id]
            ROBOT_PATH.write_text(json.dumps(out, indent=2), encoding="utf-8")
            print("[write] robot.json updated:", out)
            last_result_value = None  # 동일값 반복 방지

if __name__ == "__main__":
    main()
