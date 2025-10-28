#!/usr/bin/env python3
import json, os, math
import numpy as np
from pathlib import Path

# ================ json 파일명 확인 =======================
RESULT_PATH      = Path("guide.json")          # ← 비전이 쓰는 파일명과 동일하게!
INTRINSICS_PATH  = Path("intrinsics_guide.json")     # ← 마찬가지
ROBOT_GUIDE_PATH = Path("robot_guide.json")
# ========================================================

AruCo_x = 0.031041595619171847
AruCo_y = 0.07023097062483427
AruCo_z = 0.0

T_cam_to_robot = np.array([
    [-0.707, 0.707,  0, AruCo_x],
    [-0.707, -0.707, 0, AruCo_y],
    [0,      0,      1, AruCo_z],
    [0,      0,      0, 1.0]
], dtype=float)

def is_finite_number(x):
    try:
        return isinstance(x, (int, float)) and math.isfinite(float(x))
    except Exception:
        return False

def safe_json_read(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None

def load_intrinsics(path: Path):
    intr = safe_json_read(path)
    if not isinstance(intr, dict):
        return None
    try:
        ppx, ppy, fx, fy = map(float, (intr["ppx"], intr["ppy"], intr["fx"], intr["fy"]))
    except Exception:
        return None
    if fx == 0.0 or fy == 0.0:
        return None
    if not all(map(is_finite_number, (ppx, ppy, fx, fy))):
        return None
    return ppx, ppy, fx, fy

def load_guides_fixed3(path: Path):
    data = safe_json_read(path)
    if not (isinstance(data, list) and len(data) == 3):
        return None

    out = []
    for it in data:
        if isinstance(it, list) and len(it) == 5:
            x, y, z, ang, cls = it
            out.append([x, y, z, ang, cls])
        else:
            return None
    return out

def is_complete_guides3(guides3):
    if not (isinstance(guides3, list) and len(guides3) == 3):
        return False
    for it in guides3:
        if not (isinstance(it, list) and len(it) == 5):
            return False
        x, y, z, ang, cls = it
        if not all(is_finite_number(v) for v in (x, y, z, ang)):
            return False
        if not (isinstance(cls, (int, float)) and float(cls).is_integer()):
            return False
    return True

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

def pixel_to_cam_xy(x_pix, y_pix, z_cam, ppx, ppy, fx, fy):
    Xc = (x_pix - ppx) / fx * z_cam
    Yc = (y_pix - ppy) / fy * z_cam
    return Xc, Yc

def compute_pose(vec, intr):
    # vec: [x_pix, y_pix, z_cam, angle_deg, cls_id]
    x_pix, y_pix, z_cam, angle_deg, cls_id = vec
    ppx, ppy, fx, fy = intr

    # 필요시 단위 가드(현장에 맞게 사용 결정)
    # if z_cam > 10:  # mm로 올 가능성
    #     z_cam *= 0.001

    Xc, Yc = pixel_to_cam_xy(float(x_pix), float(y_pix), float(z_cam), ppx, ppy, fx, fy)
    Pc = np.array([Xc, Yc, z_cam, 1.0], dtype=float)
    Pr = T_cam_to_robot @ Pc

    Rz = rotz(float(angle_deg))
    t  = np.array([Pr[0], Pr[1], Pr[2]], dtype=float)
    Tm = make_T(Rz, t)
    return Tm.flatten().tolist(), int(cls_id)

def write_once(guides3, intr):
    """
    guides3: 길이 3, 각 원소는
             - [x,y,z,ang,cls] (모두 유효) 또는
             - [None,None,None,None,cls] (좌표 불가) 또는
             - None
    intr: (ppx, ppy, fx, fy)
    """
    outputs = []
    for item in guides3:
        mat16, cls_id = compute_pose(item, intr)
        outputs.append(mat16 + [cls_id])

    # 원자적 저장
    tmp = str(ROBOT_GUIDE_PATH) + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(outputs, f, indent=2)
    os.replace(tmp, ROBOT_GUIDE_PATH)
    print("[done] robot_guide.json saved.")

def main():
    print(f"[guide calib: single-shot] waiting for first refresh of {INTRINSICS_PATH.name} or {RESULT_PATH.name} ...")

    # 1) 두 파일이 '초기 상태'로 존재할 때까지 대기 (파싱까지 성공해야 초기 상태로 인정)
    intr = None
    guides3 = None

    while intr is None:
        try:
            os.stat(INTRINSICS_PATH)
            intr = load_intrinsics(INTRINSICS_PATH)
        except FileNotFoundError:
            pass

    while guides3 is None:
        try:
            os.stat(RESULT_PATH)
            guides3 = load_guides_fixed3(RESULT_PATH)
        except FileNotFoundError:
            pass

    # 초기 mtime 기록
    intr_mtime0   = os.stat(INTRINSICS_PATH).st_mtime
    guides_mtime0 = os.stat(RESULT_PATH).st_mtime
    print("[ok] initial baselines ready.")

    # 2) '둘 중 하나'라도 mtime이 바뀌는 '첫 이벤트'를 기다렸다가 한 번만 처리하고 종료
    while True:
        intr_changed = False
        guides_changed = False

        try:
            m = os.stat(INTRINSICS_PATH).st_mtime
            if m != intr_mtime0:
                intr_changed = True
        except FileNotFoundError:
            pass

        try:
            m = os.stat(RESULT_PATH).st_mtime
            if m != guides_mtime0:
                guides_changed = True
        except FileNotFoundError:
            pass

        # [CHANGED] 변경 없으면 고속 폴링 유지 (sleep 없음)
        if not (intr_changed or guides_changed):
            continue

        # 최신 값 로드
        intr2 = load_intrinsics(INTRINSICS_PATH)
        guides3_2 = load_guides_fixed3(RESULT_PATH)

        # [CHANGED] 기준 mtime을 '현재'로 갱신해 다음 변경 이벤트를 기다리도록
        try: intr_mtime0   = os.stat(INTRINSICS_PATH).st_mtime
        except FileNotFoundError: pass
        try: guides_mtime0 = os.stat(RESULT_PATH).st_mtime
        except FileNotFoundError: pass

        # [CHANGED] 둘 다 파싱 OK + 3개 완성일 때만 실제 저장 후 종료
        if (intr2 is not None) and (guides3_2 is not None) and is_complete_guides3(guides3_2):
            write_once(guides3_2, intr2)
            break

if __name__ == "__main__":
    main()