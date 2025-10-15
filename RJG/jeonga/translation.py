import os
import json
import numpy as np
from datetime import datetime

class RobotVisionCoordinator:
    def __init__(self, projection_Z_m: float):
        self.cameraMatrix = None
        self.calibration_T = None
        self.calibration_R = None
        self.projection_Z_m = float(projection_Z_m)

    # ----- NEW: 경로 헬퍼 & 로깅 -----
    @staticmethod
    def abs_path(p): return os.path.abspath(p)

    def log_path(self, label, p):
        ap = self.abs_path(p)
        print(f"[PATH] {label}: {ap}")
        return ap

    # ----- 기존 -----
    def load_calibration(self, calibration_path='calibration.json'):
        calibration_path = self.log_path("calibration.json", calibration_path)
        with open(calibration_path, 'r') as f:
            data = json.load(f)
        T = data.get('calibration_data', data.get('T', None))
        R = data.get('R', None)
        if T is None:
            raise ValueError("calibration.json에 'calibration_data' 또는 'T'가 필요합니다.")
        self.calibration_T = np.array(T, dtype=float).reshape(3,)
        self.calibration_R = None if R is None else np.array(R, dtype=float).reshape(3, 3)
        print(f"[CALIB] T (m): {self.calibration_T}")
        if self.calibration_R is not None:
            print(f"[CALIB] R:\n{self.calibration_R}")
        else:
            print("[CALIB] R 미제공 → translation만 사용")

    # ----- NEW: result.json 파서 (리스트/딕셔너리 모두 허용) -----
    @staticmethod
    def parse_detection_result(obj):
        """
        허용 형태:
          - dict: {"x":.., "y":.., "angle":..}  (추가 키 무시)
          - list/tuple: [x, y, angle, ...] 또는 [x, y, angle]
        """
        if isinstance(obj, dict):
            if not all(k in obj for k in ("x", "y", "angle")):
                raise ValueError("result.json 딕셔너리에 'x','y','angle' 키가 필요합니다.")
            return float(obj["x"]), float(obj["y"]), float(obj["angle"])

        if isinstance(obj, (list, tuple)):
            if len(obj) < 3:
                raise ValueError("result.json 리스트는 [x, y, angle, ...] 형식이어야 합니다.")
            return float(obj[0]), float(obj[1]), float(obj[2])

        raise ValueError("result.json 형식을 인식하지 못했습니다. dict 또는 list 형식이어야 합니다.")

    def pixel_to_camera_xy(self, pixel_x: float, pixel_y: float):
        if self.cameraMatrix is None or self.cameraMatrix.shape != (3, 3):
            raise ValueError("cameraMatrix(3x3)가 설정되지 않았습니다.")
        fx, fy = self.cameraMatrix[0, 0], self.cameraMatrix[1, 1]
        cx, cy = self.cameraMatrix[0, 2], self.cameraMatrix[1, 2]
        if fx == 0 or fy == 0 or self.projection_Z_m == 0:
            raise ValueError("fx/fy/projection_Z_m이 0이면 XY를 계산할 수 없습니다.")
        Z = self.projection_Z_m
        X = (float(pixel_x) - cx) * Z / fx
        Y = (float(pixel_y) - cy) * Z / fy
        cam_pt = np.array([X, Y, Z], dtype=float)
        print(f"[CAM] (X,Y) with fixed Z={Z:.3f} m → {cam_pt}")
        return cam_pt

    def camera_to_robot(self, point_camera):
        if self.calibration_T is None:
            raise ValueError("calibration_T가 설정되지 않았습니다.")
        if self.calibration_R is not None:
            base_pt = self.calibration_R @ point_camera + self.calibration_T
        else:
            base_pt = point_camera + self.calibration_T
        print(f"[BASE] xyz (m): {base_pt}")
        return base_pt

    def create_robot_transform_matrix_xy_yaw(self, x, y, yaw_deg, stay_z):
        ang = np.deg2rad(float(yaw_deg))
        c, s = np.cos(ang), np.sin(ang)
        T = np.array([
            [ c, -s, 0, float(x)],
            [ s,  c, 0, float(y)],
            [ 0,  0, 1, float(stay_z)],
            [ 0,  0, 0, 1.0]
        ], dtype=float)
        return T

    def process_detection_to_robot_command(self, detection_result_obj, stay_z: float):
        px, py, yaw_deg = self.parse_detection_result(detection_result_obj)
        cam_pt = self.pixel_to_camera_xy(px, py)
        base_pt = self.camera_to_robot(cam_pt)
        T = self.create_robot_transform_matrix_xy_yaw(base_pt[0], base_pt[1], yaw_deg, stay_z)
        return T

    def save_robot_command(self, transform_matrix, output_path='robot.json'):
        """transform_matrix(실제 배열) + angle_deg만 저장, 행당 한 줄로 예쁘게"""
        yaw_deg = float(np.rad2deg(np.arctan2(
            transform_matrix[1, 0], transform_matrix[0, 0]
        )))

        # 행 단위 포매팅 (직접 JSON 텍스트 작성)
        def row_to_line(row):
            return "[{}]".format(", ".join(f"{v:.6f}" for v in row))

        lines = [
            "  " + row_to_line(transform_matrix[0]) + ",",
            "  " + row_to_line(transform_matrix[1]) + ",",
            "  " + row_to_line(transform_matrix[2]) + ",",
            "  " + row_to_line(transform_matrix[3])
        ]

        content = "{\n" \
                '  "transform_matrix": [\n' + \
                "\n".join(lines) + "\n" \
                "  ],\n" \
                f'  "angle_deg": {yaw_deg:.6f}\n' \
                "}\n"

        with open(output_path, "w") as f:
            f.write(content)

        # 터미널 출력
        print("\n[ROBOT] Saved robot command:")
        for row in transform_matrix:
            print("  ", " ".join(f"{v: .6f}" for v in row))
        print(f"- angle_deg: {yaw_deg:.3f}\n")

        return {"transform_matrix": transform_matrix.tolist(), "angle_deg": yaw_deg}

# ---------- 실행 예 ----------
def main():
    # 실행/작업 경로 확인
    print(f"[CWD] {os.getcwd()}")

    # 카메라→작업면(고정) 거리. 없으면 대략값(예: 0.55)이라도 넣으세요.
    coordinator = RobotVisionCoordinator(projection_Z_m=0.55)

    # cameraMatrix는 반드시 3x3로 설정
    # 예) fx=fy=600, cx=320, cy=240 (자리표시자; 실제 값으로 교체)
    coordinator.cameraMatrix = np.array([
        [600.0,   0.0, 320.0],
        [  0.0, 600.0, 240.0],
        [  0.0,   0.0,   1.0],
    ], dtype=float)

    coordinator.load_calibration('calibration.json')

    # result.json 읽기 (리스트/딕셔너리 모두 허용)
    result_path = coordinator.log_path("result.json(in)", 'result.json')
    with open(result_path, 'r') as f:
        detection_obj = json.load(f)

    # 로봇이 유지할 Z (현재 TCP Z 등)
    stay_z = 0.500

    T = coordinator.process_detection_to_robot_command(detection_obj, stay_z=stay_z)
    print("\n=== Final 4x4 Transform Matrix (pre-save) ===")
    print(T)

    coordinator.save_robot_command(T, output_path='robot.json')


if __name__ == "__main__":
    main()
