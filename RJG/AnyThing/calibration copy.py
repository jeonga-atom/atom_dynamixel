#!/usr/bin/env python3

# CPU 점유율이 너무 높아 부담되면 sleep(0.01) 정도를 넣음 

import json, os, math
import numpy as np
from pathlib import Path

# 입력/출력
RESULT_PATH = Path("result.json")      # [x, y, z, angle, class]
INTRINSICS_PATH = Path("intrinsics.json")
ROBOT_PATH  = Path("robot.json")       # 1x16 플랫 4x4 행렬 저장

# AruCo_x = 0.031041595619171847
# AruCo_y = 0.07023097062483427
AruCo_x = 0.03087013459298759
AruCo_y = 0.08063858165405687
AruCo_z = 0.0  # 캘리브레이션 값을 0으로 설정 -> 왜냐면 실제 측정값 + 캘리브레이션 값이 되기 때문에 하나만 써야함
# AruCo_z = 0.4558679703623054

# 고정 평면(Z_cam) — 깊이 없이 픽셀을 카메라 좌표로 투영할 때 쓰는 스케일
# Z_val   = 0.50  # 현장에 맞게 조정 [m]

# ---- 카메라->로봇 핸드아이 4x4 (m) : 실측값으로 교체 ----
T_cam_to_robot = np.array([
    [-1, 0, 0, AruCo_x],     # t_z -> 캘리브레이션 기준
    [0, 0, 1, AruCo_y],     # t_y
    [0, 1, 0, AruCo_z],     # t_z
    [0, 0, 0,  1.0]
], dtype=float)
# -------------------------------------------------------------

def is_finite_number(x):
    try:
        return isinstance(x, (int, float)) and math.isfinite(float(x))
    except Exception:
        return False

def load_json_list5(path: Path):
    """[x,y,z,angle,class] 형식만 통과. None/NaN/inf/길이오류/타입오류는 None 반환."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if (isinstance(data, list) and len(data) == 5 and
            all(is_finite_number(v) for v in data[:4]) and       # x,y,angle
            (isinstance(data[4], (int, float)) and               # class는 숫자
             float(data[4]).is_integer())):
            # class를 int로 캐스팅
            data[4] = int(data[4])
            return data
    except Exception:
        pass
    return None

def load_intrinsics(path: Path):
    """{ppx, ppy, fx, fy} 형식만 통과. None/NaN/inf/키누락/타입오류는 None."""
    try:
        intr = json.loads(path.read_text(encoding="utf-8"))
        req = ("ppx","ppy","fx","fy")
        if all(k in intr for k in req):
            vals = [intr[k] for k in req]
            if all(is_finite_number(v) for v in vals):
                ppx, ppy, fx, fy = map(float, vals)
                # fx, fy는 0이 되면 안됨
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

def pixel_to_cam_xy(x_pix, y_pix, ppx, ppy, fx, fy, z_cam):
    Xc = (x_pix - ppx) / fx * z_cam
    Yc = (y_pix - ppy) / fy * z_cam
    return Xc, Yc

def compute_pose(result_vec, intrinsics):
    x_pix, y_pix, z_cam, angle_deg, cls_id = result_vec
    ppx, ppy, fx, fy = intrinsics

    # 픽셀 → 카메라 
    Xc, Yc = pixel_to_cam_xy(float(x_pix), float(y_pix), ppx, ppy, fx, fy, z_cam)
    Pc_cam = np.array([Xc, Yc, z_cam, 1.0], dtype=float)

    # 카메라 → 로봇
    Pr = T_cam_to_robot @ Pc_cam

    # 회전(Rz) + z 고정
    Rz = rotz(float(angle_deg))
    t_robot = np.array([Pr[0], Pr[1], Pr[2]], dtype=float)
    T_target = make_T(Rz, t_robot)

    return T_target.flatten().tolist(), int(cls_id)

# --------------- 메인 루프 ---------------
def main():
    print("[watch] 시작")

    last_result_mtime = None
    last_intr_mtime   = None
    last_result_value = None
    last_intr_value   = None

    # 유효한 intrinsics 나올 때까지 대기
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

    # 두 파일 모두 변경/검증
    while True:
        intr_changed = False
        res_changed  = False

        # intrinsics 변경 감지 및 유효성 확인
        try:
            st_i = os.stat(INTRINSICS_PATH)
            if last_intr_mtime is None or st_i.st_mtime != last_intr_mtime:
                intr = load_intrinsics(INTRINSICS_PATH)
                if intr is not None:
                    last_intr_mtime = st_i.st_mtime
                    if intr != last_intr_value:
                        last_intr_value = intr
                        intr_changed = True
                        print("[update] intrinsics.json 바뀜! :", intr)
        except FileNotFoundError:
            pass  # 다음 루프에서 재시도

        # result 변경 감지 및 유효성 확인
        try:
            st_r = os.stat(RESULT_PATH)
            if last_result_mtime is None or st_r.st_mtime != last_result_mtime:
                data = load_json_list5(RESULT_PATH)
                if data is not None:
                    last_result_mtime = st_r.st_mtime
                    if data != last_result_value:
                        last_result_value = data
                        res_changed = True
                        print("[update] result.json 바뀜!:", data)
        except FileNotFoundError:
            pass

        # 갱신 조건:
        #  - result가 새로 왔고 intrinsics도 유효 → 변환/저장
        #  - intrinsics가 바뀌었고, 마지막 유효 result가 있다 → 재계산/저장
        if last_result_value and last_intr_value:
            matrix16, cls_id = compute_pose(last_result_value, last_intr_value)
            out = matrix16 + [cls_id]  # 행렬 뒤에 class 하나만 추가
            ROBOT_PATH.write_text(json.dumps(out, indent=2), encoding="utf-8")
            print("[write] robot.json updated:", out)
            last_result_value = None  # 다음 값 기다림 (동일값 반복 방지)


if __name__ == "__main__":
    main()
