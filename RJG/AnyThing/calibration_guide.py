#!/usr/bin/env python3
import json, os, math
import numpy as np
from pathlib import Path

# ======= 실제 비전 출력 파일명과 정확히 맞추세요! =======
RESULT_PATH      = Path("guide.json")          # ← 비전이 쓰는 파일명과 동일하게!
INTRINSICS_PATH  = Path("intrinsics.json")     # ← 마찬가지
ROBOT_GUIDE_PATH = Path("robot_guide.json")
# ========================================================

Z_val = 0.50  # m

# 핸드아이 예시(실측으로 교체)
AruCo_x = 35.2245380036407
AruCo_y = -66.81746192800584
AruCo_z = 118.2207832845053
T_cam_to_robot = np.array([
    [-1, 0, 0,  AruCo_x],
    [ 0, 0, 1,  AruCo_y],
    [ 0, 1, 0,  AruCo_z],
    [ 0, 0, 0,  1.0     ]
], dtype=float)

def is_finite_number(x):
    try:
        return isinstance(x, (int, float)) and math.isfinite(float(x))
    except Exception:
        return False

def load_intrinsics(path: Path):
    try:
        intr = json.loads(path.read_text(encoding="utf-8"))
        ppx, ppy, fx, fy = map(float, (intr["ppx"], intr["ppy"], intr["fx"], intr["fy"]))
        if fx != 0.0 and fy != 0.0 and all(map(is_finite_number, (ppx, ppy, fx, fy))):
            return ppx, ppy, fx, fy
    except Exception:
        pass
    return None

def safe_json_read(path: Path):
    """쓰기 도중(부분 저장)이어도 예외 삼키고 None 반환"""
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None

def load_guides(path: Path):
    """
    기대 형식(3개):
      [[x,y,angle,class], [x,y,angle,class], [x,y,angle,class]]
    각 항목은 개별 검증:
      - 세 값(x,y,angle) 중 하나라도 유효하지 않으면 해당 항목은 None 처리
      - class는 정수로 강제 캐스팅 시도, 실패하면 None
    항상 길이 3 리스트 반환: [ [x,y,ang,cls] or None, ... x3 ]
    """
    data = safe_json_read(path)
    if not isinstance(data, list) or len(data) != 3:
        return None

    out = []
    for it in data:
        if (isinstance(it, list) and len(it) == 4):
            x, y, ang, cls = it
            ok_xyz = all(is_finite_number(v) for v in (x, y, ang))
            cls_ok = isinstance(cls, (int, float)) and float(cls).is_integer()
            if ok_xyz and cls_ok:
                out.append([float(x), float(y), float(ang), int(cls)])
            else:
                # 클래스만 맞고 좌표가 None일 수도 있는 경우 허용: [None]*16 + class 로 출력하게 함
                out.append([None, None, None, int(cls)] if cls_ok else None)
        else:
            out.append(None)

    return out if len(out) == 3 else None

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

def compute_pose(vec, intr):
    x_pix, y_pix, angle_deg, cls_id = vec
    ppx, ppy, fx, fy = intr
    Xc, Yc = pixel_to_cam_xy(float(x_pix), float(y_pix), ppx, ppy, fx, fy, Z_val)
    Pc = np.array([Xc, Yc, Z_val, 1.0], dtype=float)
    Pr = T_cam_to_robot @ Pc
    Rz = rotz(float(angle_deg))
    t  = np.array([Pr[0], Pr[1], Z_val], dtype=float)
    Tm = make_T(Rz, t)
    return Tm.flatten().tolist(), int(cls_id)

def main():
    print(f"[guide calib] wait for {INTRINSICS_PATH.name} & {RESULT_PATH.name} ...")

    # 1) intrinsics 대기
    intr = None
    while intr is None:
        try:
            os.stat(INTRINSICS_PATH)
            intr = load_intrinsics(INTRINSICS_PATH)
        except FileNotFoundError:
            pass
    print("[ok] intrinsics loaded:", intr)

    # 2) guide.json 대기
    guides = None
    while guides is None:
        try:
            os.stat(RESULT_PATH)
            guides = load_guides(RESULT_PATH)
        except FileNotFoundError:
            pass
    print("[ok] guide.json loaded:", guides)

    # 3) 변환: 각 항목을 길이 17 리스트(행렬16 + class)로
    outputs = []
    for item in guides:
        if item is None:
            # 완전 무효 → 17개 None
            outputs.append([None]*17)
        elif None in item[:3]:
            # class만 있고 좌표/각도 미유효 → 행렬16 None + class
            cls_id = item[3] if isinstance(item[3], int) else None
            outputs.append([None]*16 + [cls_id])
        else:
            mat16, cls_id = compute_pose(item, intr)
            outputs.append(mat16 + [cls_id])

    # 4) 저장(원자적) 후 종료
    tmp = str(ROBOT_GUIDE_PATH) + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(outputs, f, indent=2)
    os.replace(tmp, ROBOT_GUIDE_PATH)
    print("[done] robot_guide.json saved.")

if __name__ == "__main__":
    main()
