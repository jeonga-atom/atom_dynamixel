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
    [1, 0, 0, AruCo_x],
    [0, 1, 0, AruCo_y],
    [0, 0, 1, AruCo_z],
    [0, 0, 0, 1.0]
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
            cls_ok = isinstance(cls, (int, float)) and float(cls).is_integer()
            if not cls_ok:
                out.append(None)
                continue
            cls = int(cls)

            ok_vals = all(is_finite_number(v) for v in (x, y, z, ang))
            if ok_vals:
                out.append([float(x), float(y), float(z), float(ang), cls])
            else:
                out.append([None, None, None, None, cls])  # 길이 5 유지
        else:
            out.append(None)

    return out

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
        if item is None:
            outputs.append([None]*17)
            continue

        x, y, z, ang, cls_id = item
        if any(v is None for v in (x, y, z, ang)):
            outputs.append([None]*16 + [cls_id])
            continue

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
            if os.stat(INTRINSICS_PATH).st_mtime != intr_mtime0:
                intr_changed = True
        except FileNotFoundError:
            pass

        try:
            if os.stat(RESULT_PATH).st_mtime != guides_mtime0:
                guides_changed = True
        except FileNotFoundError:
            pass

        if intr_changed or guides_changed:
            # 최신 데이터로 다시 로드 (부분 저장 대비)
            intr2 = load_intrinsics(INTRINSICS_PATH)
            guides3_2 = load_guides_fixed3(RESULT_PATH)

            if (intr2 is not None) and (guides3_2 is not None):
                write_once(guides3_2, intr2)
                break  # 한 번 쓰고 종료
            # 최신 파싱 실패면 다음 루프에서 재시도

if __name__ == "__main__":
    main()
