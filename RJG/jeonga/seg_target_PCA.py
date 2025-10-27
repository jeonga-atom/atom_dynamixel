import cv2
import os, math
import numpy as np
import time
import pyrealsense2 as rs
import json
import torch 

from ultralytics import YOLO
from calculate import ObjectManipulator
from pathlib import Path

"""
names: 
    class_id: class_name
    0: circle
    1: circle_guide
    2: corss
    3: cross_guide
    4: square
    5: square_guide
"""
ROBOT_PATH  = Path("robot.json")

AruCo_x = 0.03990034363232551  # 2025 RB10 예시값
AruCo_y = 0.06533827053499407
AruCo_z = 0.0  # 캘리브레이션 값을 0으로 설정 -> 왜냐면 실제 측정값 + 캘리브레이션 값이 되기 때문에 하나만 써야함

# ========== Hand_eye 캘리브레이션 값 (카메라->로봇) ==========
T_cam_to_robot = np.array([
    [-0.707, 0.707,  0,  AruCo_x],     # t_z -> 캘리브레이션 기준
    [-0.707, -0.707, 0,  AruCo_y],     # t_y
    [0,      0,      1,  AruCo_z],     # t_z
    [0,      0,      0,  1.0]
], dtype=float)

def rotz(deg: float):
    r = np.deg2rad(deg)
    c, s = np.cos(r), np.sin(r)
    return np.array([[ c,-s, 0],
                     [ s, c, 0],
                     [ 0, 0, 1]], dtype=float)

def make_T(R, t):
    T = np.eye(4, dtype=float)
    T[:3,:3] = R
    T[:3, 3] = t
    return T

def pixel_to_cam_xy(x_pix, y_pix, ppx, ppy, fx, fy, z_cam):
    Xc = (x_pix - ppx) / fx * z_cam
    Yc = (y_pix - ppy) / fy * z_cam
    return Xc, Yc

def compute_pose(result_vec, intrinsics, T_cam_to_robot):
    """
    result_vec: [cx, cy, depth_value, angle_deg, class_id]
    intrinsics: [ppx, ppy, fx, fy]
    반환: (목표포즈 1x16 리스트, class_id)
    """
    x_pix, y_pix, z_cam, angle_deg, cls_id = result_vec
    ppx, ppy, fx, fy = intrinsics

    # 픽셀 → 카메라 
    Xc, Yc = pixel_to_cam_xy(float(x_pix), float(y_pix), ppx, ppy, fx, fy, float(z_cam))
    Pc_cam = np.array([Xc, Yc, float(z_cam), 1.0], dtype=float)

    # 카메라 → 로봇
    Pr = T_cam_to_robot @ Pc_cam

    # 회전(Rz) + z 고정
    Rz = rotz(float(angle_deg))
    t_robot = np.array([Pr[0], Pr[1], Pr[2]], dtype=float)
    T_target = make_T(Rz, t_robot)

    return T_target.flatten().tolist(), int(cls_id)

def draw_angle_overlay(img, cx, cy, angle_deg, length=80, color=(0,0,255)):
    # OpenCV 이미지 좌표: x→오른쪽(+), y→아래(+)
    # angle_deg: x축과의 각도(도)

    rad = np.deg2rad(angle_deg)
    x2 = int(cx + length * np.cos(rad))
    y2 = int(cy + length * np.sin(rad))

    cv2.circle(img, (int(cx), int(cy)), 5, (0,255,0), -1)        # 중심
    cv2.line(img, (int(cx), int(cy)), (x2, y2), color, 2)        # 각도축
    cv2.putText(img, f"{angle_deg:.1f} deg", (int(cx)+10, int(cy)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def segmentation_calculate(image, model=None, calc=None):
    """
    입력:
        image: 컬러 이미지 (H, W, 3), BGR 포맷 (numpy array)
        model: YOLO 세그멘테이션 모델 (ultralytics YOLO)
    반환:
        (cx, cy, angle, class_name) 픽셀 좌표, 각도, 클래스 이름 또는 None
    """
    if model is None:
        raise ValueError("세그멘테이션을 위해 YOLO 모델이 필요합니다.")

    H, W = image.shape[:2]

    # YOLO 모델로 세그멘테이션 수행
    results = model(source=image, imgsz=1280, device='cuda', conf=0.20, retina_masks=True, verbose=False)
    if not results or results[0].masks is None or results[0].boxes is None:
        return None
    
    # 최고 신뢰도 인덱스 선택 
    confs = results[0].boxes.conf.cpu().numpy()
    i = int(np.argmax(confs))
    conf_top = float(confs[i])

    # 첫 번째 객체 기준 (원하면 신뢰도/클래스 필터링 추가 가능)
    class_id = int(results[0].boxes.cls[i].item())
    # class_name = (model.names[class_id] if hasattr(model, "names") else results[0].names[class_id]).lower()

    # 첫 번째 객체의 마스크와 클래스 이름 가져오기
    mask = results[0].masks.data[i].cpu().numpy()
    mask = (mask > 0).astype(np.uint8)

    # 원본 사이즈로 변환 (최근접)
    if mask.shape != (H, W):
        mask = cv2.resize(mask, (W, H), interpolation=cv2.INTER_NEAREST)

    # 마스크 전처리
    mask = calc.simple_clean_mask(mask, kernel_size=5)

    # 외곽 컨투어만 추출 (전처리된 마스크 사용)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # 중심 좌표와 컨투어 계산
    result = calc.mask_centroid_moments(mask, contours)
    if result is None:
        return None

    cx, cy, largest_contour = result

    if class_id == 0 :
        angle = 0.0

    elif class_id == 2 :    
        # 교차형(corss)은 측정된 각도 +45도 보정  -> 추후에 cross로 이름 수정해야함. 
        # 십자형은 허프+PCA 기반 정밀 각도 추정
        angle = calc.angle_cross_precise(mask)
        angle = calc.normalize_angle_0_90(angle)

        if angle is None:
            # 백업: 기존 minAreaRect 방식
            angle = calc.calculate_contour_angle_cross(largest_contour)
            angle = calc.normalize_angle_0_90(angle)
            print('none')

    elif class_id == 4 :
        # 정사각형은 그허프+PCA 기반 정밀 각도 추정
        angle = calc.angle_square_precise(mask)
        angle = calc.normalize_angle_0_90(angle)

        if angle is None:
            # 백업: 기존 minAreaRect 방식
            angle = calc.calculate_contour_angle_square(largest_contour)
            angle = calc.normalize_angle_0_90(angle)
    else:
        return None
    
    result = [cx, cy, angle, class_id, conf_top]
    return result

def main():
    global model
    global profile
    global calc

    calc = ObjectManipulator()

    t0 = time.time()
    # 모델 로드 
    model_directory = os.environ['HOME'] + '/rjg_ws/runs/segment/train4/weights/last.pt' 
    # print(f"[INFO] Loading model: {model_directory}") 
    model = YOLO(model_directory)

    assert torch.cuda.is_available(), "[ERROR] CUDA(GPU) 미감지"  
    model.to('cuda')                            

    print(f"[INFO] Loading model: {model_directory}") 
    t1 = time.time()

    print(f"전체 걸린 시간: {t1 - t0:.2f}초")

    # RealSense 초기화
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    profile = pipeline.start(config)

    # 카메라 설정 조정
    rs_device = profile.get_device()
    color_sensor = rs_device.query_sensors()[1]
    depth_sensor = profile.get_device().first_depth_sensor()

    # 노출 및 게인 설정 (수동으로 다시 설정) -> 가서 조정해 봐야함.
    color_sensor.set_option(rs.option.enable_auto_exposure, 1)        # 자동 노출 비활성화
    color_sensor.set_option(rs.option.enable_auto_white_balance, 1)   # 자동 화이트 밸런스 비활성화
    # color_sensor.set_option(rs.option.exposure, 30)   # 노출 값 
    # color_sensor.set_option(rs.option.gain, 80)       # 게인 값 
    color_sensor.set_option(rs.option.contrast, 65)     # 명암 대비 값
    color_sensor.set_option(rs.option.saturation, 75)   # 채도 값

    align_to = rs.stream.color
    align = rs.align(align_to)

    # --- 중요: 새 설정 적용될 때까지 프레임 버리기 ---
    for _ in range(2):  # 약 0.1초간 프레임 discard
        frames = pipeline.wait_for_frames()

    # ===== 여기부터 intrinsics.json 저장 =====
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    intrinsics_data = [float(intr.ppx), float(intr.ppy), float(intr.fx), float(intr.fy)]

    print("[INFO] intrinsic 데이터:", intrinsics_data)

    # 이미지 촬영 플래그
    global image_captured
    image_captured = False

    save_directory = '/home/kminseo/LAST/captured'
    file_name_color = 'target.jpg'
    save_path_color = os.path.join(save_directory, file_name_color)
    os.makedirs(save_directory, exist_ok=True)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                print("[WARNING] 컬러 또는 깊이 프레임을 가져오지 못했습니다.")
                continue

            color_image = np.asanyarray(color_frame.get_data())

            if not image_captured:
                cv2.imwrite(save_path_color, color_image)
                
                # # 캡처한 사진 읽기
                image_path_color = '/home/kminseo/LAST/captured/target.jpg'
                image_color = cv2.imread(image_path_color)
                result = segmentation_calculate(image_color, model=model, calc=calc)

                if result is not None:
                    cx, cy, angle, class_id, _ = result
                    depth_value = depth_frame.get_distance(int(cx), int(cy))  # 단위: m

                    print(f"중심 좌표: ({cx}, {cy}), 클래스: {class_id}, 기울기 각도: {angle}도, 깊이: {depth_value} m")

                    # 중심점 + 각도선 시각화
                    draw_angle_overlay(image_color, cx, cy, angle)
                    cv2.imwrite(save_path_color, image_color)
                    
                    # 결과 저장 ([cx, cy, depth_value, angle])
                    result = [cx, cy, depth_value, angle, class_id]
                    pose_flat_1x16, cls_id = compute_pose(result, intrinsics_data, T_cam_to_robot)  # [FIX] result_xyz → result
                    robot_payload = pose_flat_1x16 + [cls_id]  # [FIX] 저장할 payload 정의
                    with open(ROBOT_PATH, 'w', encoding='utf-8') as f:  # [FIX] 상단의 ROBOT_PATH 사용
                        json.dump(robot_payload, f, indent=2)
                
                else:
                    print("[WARNING] 중심 좌표 또는 컨투어 계산 실패")
                    result = [None, None, None, None, None]

                image_captured = True

                t8 = time.time()
                print(f"전체 걸린 시간: {t8 - t7:.2f}초")

                return intrinsics_data, result
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[INFO] 종료 완료.")

if __name__ == "__main__":
    t7 = time.time()
    main()