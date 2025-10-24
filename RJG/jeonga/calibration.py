#!/usr/bin/env python3
import json, os, math
import numpy as np
from pathlib import Path

# ====== 파일 경로 ======
RESULT_PATH     = Path("result.json")        # [x, y, z, angle, class]
INTRINSICS_PATH = Path("intrinsics.json")    # {ppx, ppy, fx, fy}
ROBOT_PATH      = Path("robot.json")         # [16 floats + class]

# ====== 아루코 기반 오프셋 (m 단위) ======
AruCo_x = 0.03087013459298759
AruCo_y = 0.07023097062483427
AruCo_z = 0.0

# ====== 회전행렬 (두 번째 행렬: 옆면/위쪽 장착용) ======
R_cam_to_robot = np.array([
    [-1, 0, 0],
    [ 0, 0, 1],
    [ 0, 1, 0],
], dtype=float)

t_cam_to_robot = np.array([AruCo_x, AruCo_y, AruCo_z], dtype=float)

T_cam_to_robot = np.eye(4, dtype=float)
T_cam_to_robot[:3, :3] = R_cam_to_robot
T_cam_to_robot[:3,  3] = t_cam_to_robot

# ===========================================================

def rotz(deg: float):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ], dtype=float)

def pixel_to_cam(x, y, z, ppx, ppy, fx, fy):
    """픽셀좌표 + 깊이 → 카메라좌표계 (m)"""
    Xc = (x - ppx) / fx * z
    Yc = (y - ppy) / fy * z
    Zc = z
    return np.array([Xc, Yc, Zc], dtype=float)

def compute_pose(result_vec, intrinsics):
    x_pix, y_pix, z_cam, angle_deg, cls_id = result_vec
    ppx, ppy, fx, fy = intrinsics

    # ① 픽셀 → 카메라
    Pc_cam = pixel_to_cam(x_pix, y_pix, z_cam, ppx, ppy, fx, fy)
    Pc_cam_h = np.append(Pc_cam, 1.0)  # [Xc,Yc,Zc,1]

    # ② 카메라 → 로봇
    Pr_h = T_cam_to_robot @ Pc_cam_h
    Pr = Pr_h[:3]  # [m]

    # ③ 자세 (카메라 기준 yaw → 로봇 프레임 변환)
    Rz_cam = rotz(angle_deg)
    R_target = R_cam_to_robot @ Rz_cam

    # ④ 4x4 변환행렬 구성
    T_target = np.eye(4, dtype=float)
    T_target[:3,:3] = R_target
    T_target[:3, 3] = Pr

    return T_target.flatten().tolist(), int(cls_id)

# ===========================================================

def load_json_list5(path):
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(data, list) and len(data) == 5:
            return data
    except Exception:
        pass
    return None

def load_intrinsics(path):
    try:
        intr = json.loads(path.read_text(encoding="utf-8"))
        ppx, ppy, fx, fy = [float(intr[k]) for k in ("ppx","ppy","fx","fy")]
        return ppx, ppy, fx, fy
    except Exception:
        pass
    return None

# ===========================================================

def main():
    print("[watch] 시작")

    last_result_mtime = None
    last_intr_mtime   = None
    last_result_value = None
    last_intr_value   = None

    # intrinsics 유효할 때까지 대기
    while True:
        try:
            st = os.stat(INTRINSICS_PATH)
            intr = load_intrinsics(INTRINSICS_PATH)
            if intr:
                last_intr_mtime = st.st_mtime
                last_intr_value = intr
                print("[ok] intrinsics 로드:", intr)
                break
        except FileNotFoundError:
            continue

    # 변경 감지 루프 (새 값이 들어올 때만 robot.json 저장)
    while True:
        try:
            st_r = os.stat(RESULT_PATH)
            if last_result_mtime is None or st_r.st_mtime != last_result_mtime:
                data = load_json_list5(RESULT_PATH)
                if data:
                    last_result_mtime = st_r.st_mtime
                    if data != last_result_value:
                        last_result_value = data
                        print("[update] result.json:", data)

                        matrix16, cls_id = compute_pose(data, last_intr_value)
                        out = matrix16 + [cls_id]
                        ROBOT_PATH.write_text(json.dumps(out, indent=2), encoding="utf-8")
                        print("[write] robot.json updated:", out)
                        last_result_value = None
        except FileNotFoundError:
            pass

# ===========================================================
if __name__ == "__main__":
    main()
