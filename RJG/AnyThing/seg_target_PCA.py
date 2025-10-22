import cv2
import os
import numpy as np
import time
import pyrealsense2 as rs
import json
import torch 

from ultralytics import YOLO
from calculate import ObjectManipulator

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

# MODEL_PATH = "/home/kminseo/rjg_ws/runs/segment/train/weights/last.pt"
OUT_PATH = "result.json"
OUT_2_PATH = "intrinsics.json"


def draw_angle_overlay(img, cx, cy, angle_deg, length=80, color=(0,0,255)):
    # OpenCV 이미지 좌표: x→오른쪽(+), y→아래(+)
    # angle_deg: x축과의 각도(도)

    rad = np.deg2rad(angle_deg)
    x2 = int(cx + length * np.cos(rad))
    y2 = int(cy + length * np.sin(rad))

    cv2.circle(img, (int(cx), int(cy)), 5, (0,255,0), -1)        # 중심
    cv2.line(img, (int(cx), int(cy)), (x2, y2), color, 2)        # 각도축
    cv2.putText(img, f"{angle_deg:.1f} deg", (int(cx)+10, int(cy)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def segmentation_calculate(image, model=None):
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
    results = model(source=image, imgsz=1280, device='cuda',verbose=False)
    if not results or results[0].masks is None or results[0].boxes is None:
        return None
    
    # 최고 신뢰도 인덱스 선택 
    confs = results[0].boxes.conf.cpu().numpy()
    i = int(np.argmax(confs))
    conf_top = float(confs[i])

    # 첫 번째 객체 기준 (원하면 신뢰도/클래스 필터링 추가 가능)
    class_id = int(results[0].boxes.cls[i].item())
    class_name = (model.names[class_id] if hasattr(model, "names") else results[0].names[class_id]).lower()

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

    if class_name == 'circle':
        angle = 0.0

    elif class_name == 'corss':    
        # 교차형(corss)은 측정된 각도 +45도 보정  -> 추후에 cross로 이름 수정해야함. 
        # 십자형은 허프+PCA 기반 정밀 각도 추정
        angle = calc.angle_cross_precise(mask)
        if angle is None:
            # 백업: 기존 minAreaRect 방식
            angle = calc.calculate_contour_angle_cross(largest_contour)
            angle = calc.normalize_angle_0_90(angle)

    elif class_name == 'square':
        # 정사각형은 그허프+PCA 기반 정밀 각도 추정
        angle = calc.angle_square_precise(mask)
        if angle is None:
            # 백업: 기존 minAreaRect 방식
            angle = calc.calculate_contour_angle_square(largest_contour)
            angle = calc.normalize_angle_0_90(angle)

    else:
        angle = calc.calculate_contour_angle_square(largest_contour)
        angle = calc.normalize_angle_0_90(angle)

    return cx, cy, angle, class_id, conf_top


def main():
    global model
    global profile
    global calc

    calc = ObjectManipulator()

    # 모델 로드 
    model_directory = os.environ['HOME'] + '/ptfile/seg.pt' 
    # print(f"[INFO] Loading model: {model_directory}") 
    model = YOLO(model_directory)

    assert torch.cuda.is_available(), "[ERROR] CUDA(GPU) 미감지"  
    model.to('cuda')                            

    print(f"[INFO] Loading model: {model_directory}") 

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
    color_sensor.set_option(rs.option.enable_auto_exposure, 0)        # 자동 노출 비활성화
    color_sensor.set_option(rs.option.enable_auto_white_balance, 0)   # 자동 화이트 밸런스 비활성화
    color_sensor.set_option(rs.option.exposure, 30)  # 노출 값 
    color_sensor.set_option(rs.option.gain, 128)     # 게인 값 
    color_sensor.set_option(rs.option.saturation, 75)

    align_to = rs.stream.color
    align = rs.align(align_to)

    # --- 중요: 새 설정 적용될 때까지 프레임 버리기 ---
    for _ in range(2):  # 약 0.1초간 프레임 discard
        frames = pipeline.wait_for_frames()

    t9 = time.time()
    # ===== 여기부터 intrinsics.json 저장 =====
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    intrinsics_data = {
        "ppx": float(intr.ppx),
        "ppy": float(intr.ppy),
        "fx":  float(intr.fx),
        "fy":  float(intr.fy),
    }

    # 더 안전한 원자적 저장(임시 파일 → 교체)
    _tmp = OUT_2_PATH + ".tmp"
    with open(_tmp, "w", encoding="utf-8") as f:
        json.dump(intrinsics_data, f, indent=2)
    os.replace(_tmp, OUT_2_PATH)

    print("[INFO] intrinsics.json 저장 완료:", intrinsics_data)

    t10 = time.time()
    print(f"intrinsics 로드에 걸린 시간: {t10 - t9:.2f}초")

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
                result = segmentation_calculate(image_color, model=model)

                if result is not None:
                    cx, cy, angle, class_id, _ = result
                    depth_value = depth_frame.get_distance(int(cx), int(cy))  # 단위: m

                    print(f"중심 좌표: ({cx}, {cy}), 클래스: {class_id}, 기울기 각도: {angle}도, 깊이: {depth_value} m")

                    # 중심점 + 각도선 시각화
                    draw_angle_overlay(image_color, cx, cy, angle)
                    
                    # JSON 파일로 결과 저장 ([cx, cy, depth_value, angle])
                    result_json = [
                        cx if cx is not None else None,
                        cy if cy is not None else None,
                        depth_value if depth_value is not None else None,
                        angle if angle is not None else None,
                        class_id if class_id is not None else None,
                    ]

                    with open(OUT_PATH, 'w') as f:
                        json.dump(result_json, f, indent=4)
                    print(f"[INFO] 중심 좌표, 깊이 값, 기울기 각도를 {OUT_PATH}에 저장했습니다.")
                
                else:
                    print("[WARNING] 중심 좌표 또는 컨투어 계산 실패")
                    result_json = [None, None, None, None]
                    with open(OUT_PATH, 'w') as f:
                        json.dump(result_json, f, indent=4)
                    print(f"[INFO] 실패 결과를 {OUT_PATH}에 저장했습니다.")


                # 중심점이 그려진 이미지 저장
                cv2.imwrite(save_path_color, image_color)

                image_captured = True

                t8 = time.time()
                print(f"전체 걸린 시간: {t8 - t7:.2f}초")

                return result if result is not None else (None, None, None, None)

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[INFO] 종료 완료.")

if __name__ == "__main__":
    t7 = time.time()
    main()