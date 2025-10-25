import cv2
import os
import numpy as np
import time
import pyrealsense2 as rs
import json

from ultralytics import YOLO
from calculate import ObjectManipulator

# names: 
#   0: circle
#   1: circle_guide
#   2: corss
#   3: cross_guide
#   4: square
#   5: square_guide

MODEL_PATH = "/home/kminseo/ptfile/guide.pt"
OUT_PATH = "guide.json"
OUT_2_PATH   = "intrinsics.json"


def draw_angle_overlay(img, cx, cy, angle_deg, length=80, color=(0,0,255)):
    # OpenCV 이미지 좌표: x→오른쪽(+), y→아래(+)
    # angle_deg: x축과의 각도(도)

    rad = np.deg2rad(angle_deg)
    x2 = int(cx + length * np.cos(rad))
    y2 = int(cy + length * np.sin(rad))

    cv2.circle(img, (int(cx), int(cy)), 5, (0,255,0), -1)        # 중심
    cv2.line(img, (int(cx), int(cy)), (x2, y2), color, 2)        # 각도축
    cv2.putText(img, f"{angle_deg:.1f} deg", (int(cx)+10, int(cy)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def segmentation_calculate(image, model=None, guide_classes=(1,3,5), conf_min= 0.5):
    """
    입력:
        image: BGR 이미지 (H,W,3)
        model: Ultralytics YOLO Seg 모델
        guide_classes: 추출할 클래스 id 튜플(또는 리스트)
        conf_min: 최소 confidence
    반환:
        guide_list: [[cx, cy, angle, class_id], ...]  (target 클래스만)
        draw_items:   [(cx, cy, angle), ...]            (시각화용)
    """
    if model is None:
        raise ValueError("세그멘테이션을 위해 YOLO 모델이 필요합니다.")

    H, W = image.shape[:2]

    # YOLO 모델로 세그멘테이션 수행
    results = model(source=image, imgsz=1280, agnostic_nms=True, verbose=False)
    # agnostic_nms=True는 YOLO 모델이 탐지 결과를 “클래스 구분 없이” 정리
    # (NMS, Non-Maximum Suppression)하도록 하는 옵션
    # 같은 물체를 여러 클래스가 동시에 인식하는 문제를 해결해 주는 기능
    if not results or results[0].masks is None or results[0].boxes is None:
        return [], []
    
    boxes = results[0].boxes
    masks = results[0].masks.data.cpu().numpy()     # (N, h, w)
    confs = boxes.conf.cpu().numpy()                # (N,)
    cls_ids = boxes.cls.cpu().numpy().astype(int)   # (N,)

    guide_list = []
    draw_items = []

    # 각 detection 순회 -> class_id 1, 3, 5만 처리하기 위해
    for i in range(len(cls_ids)):
        class_id = int(cls_ids[i])
        conf = float(confs[i])
        
        if class_id not in guide_classes:
            continue
        if conf < conf_min:
            continue

        mask = (masks[i] > 0).astype(np.uint8)

        # 원본 사이즈로 변환 (최근접)
        if mask.shape != (H, W):
            mask = cv2.resize(mask, (W, H), interpolation=cv2.INTER_NEAREST)

        # 마스크 전처리 & 컨투어
        mask = calc.simple_clean_mask(mask, kernel_size=5)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue

        result = calc.mask_centroid_moments(mask, contours)
        if result is None:
            continue

        cx, cy, largest_contour = result

        # guide는 다 사각형이므로 사각형을 계산하는 각도로 계산.
        if class_id == 1 :
            angle = 0.0

        else:
            angle = calc.calculate_contour_angle_square(largest_contour)
            angle = calc.normalize_angle_0_90(angle)

        guide_list.append([cx, cy, angle, class_id])
        draw_items.append((cx, cy, angle))

    return guide_list, draw_items


def main():
    global model
    global profile
    global calc

    calc = ObjectManipulator()

    # 모델 로드
    print("[INFO] Loading model:", MODEL_PATH)
    model_directory = os.environ['HOME'] + '/rjg_ws/runs/segment/train4/weights/last.pt'
    model = YOLO(model_directory)

    # RealSense 초기화
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    profile = pipeline.start(config)

    # 카메라 설정 조정
    device = profile.get_device()
    color_sensor = device.query_sensors()[1]
    depth_sensor = profile.get_device().first_depth_sensor()

    # 노출 및 게인 설정 (수동으로 다시 설정) -> 가서 조정해 봐야함.
    color_sensor.set_option(rs.option.enable_auto_exposure, 1)        # 자동 노출 비활성화
    color_sensor.set_option(rs.option.enable_auto_white_balance, 1)   # 자동 화이트 밸런스 비활성화
    # color_sensor.set_option(rs.option.exposure, 30)   # 노출 값 
    # color_sensor.set_option(rs.option.gain, 80)       # 게인 값 
    # color_sensor.set_option(rs.option.contrast, 65)     # 명암 대비 값
    color_sensor.set_option(rs.option.saturation, 75)   # 채도 값

    align_to = rs.stream.color
    align = rs.align(align_to)

    # --- 프레임 안정화 ---
    for _ in range(2):
        pipeline.wait_for_frames()

    # --- intrinsics.json 저장 (한 번만) ---
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    intrinsics_data = {
        "ppx": float(intr.ppx),
        "ppy": float(intr.ppy),
        "fx":  float(intr.fx),
        "fy":  float(intr.fy),
    }

    tmp_intr = OUT_2_PATH + ".tmp"
    with open(tmp_intr, "w", encoding="utf-8") as f:
        json.dump(intrinsics_data, f, indent=2)
    os.replace(tmp_intr, OUT_2_PATH)
    print("[INFO] intrinsics.json 저장 완료:", intrinsics_data)

    # 이미지 촬영 플래그
    global image_captured
    image_captured = False

    save_directory = '/home/kminseo/LAST/captured'
    file_name_color = 'guide.jpg'
    save_path_color = os.path.join(save_directory, file_name_color)
    os.makedirs(save_directory, exist_ok=True)

    try:
        
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            print("[WARNING] 컬러 또는 깊이 프레임을 가져오지 못했습니다.")

            ordered_guide_json = [
                [None, None, None, None, 1],
                [None, None, None, None, 3],
                [None, None, None, None, 5]
            ]

            with open(OUT_PATH, 'w') as f:
                json.dump(ordered_guide_json, f, indent=4)
            return ordered_guide_json

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        if not image_captured:
            cv2.imwrite(save_path_color, color_image)
            
            # 캡처한 사진 읽기
            image_path_color = '/home/kminseo/Downloads/221.jpg'
            image_color = cv2.imread(image_path_color)
            
            guide_classes = (1, 3, 5)   # 현재 모델은 0,1,2로 되어있음 (1, 3, 5)로 수정 필요
            guide_list, draw_items = segmentation_calculate(image_color, model=model, guide_classes=guide_classes, conf_min=0.25)

            
            value = []
            for (cx, cy, angle, class_id) in guide_list:
                depth_value = depth_frame.get_distance(int(cx), int(cy))  # m
                print(f"중심 좌표: ({cx}, {cy}), 클래스: {class_id}, 기울기 각도: {angle}도, 깊이: {depth_value} m")
                value.append([cx, cy, depth_value, angle, class_id])

            # 중심점 + 각도선 시각화
            for (cx, cy, angle) in draw_items:
                draw_angle_overlay(image_color, cx, cy, angle)

            cv2.imwrite(save_path_color, image_color)


            # JSON 저장 전 정렬 (1 → 3 → 5 순서 고정)
            guide_order = [1, 3, 5] # 1, 3, 5로 수정해야 함.

            ordered_guide_json = []
            
            for cid in guide_order:
                # 해당 클래스의 첫 번째 결과만 (또는 여러 개면 가장 conf 높은 것)
                items = [r for r in value if r[4] == cid]
                if items:
                    ordered_guide_json.append(items[0])  # 여러 개면 첫 번째만

                else:
                    ordered_guide_json.append([None, None, None, None, cid])

            with open(OUT_PATH, 'w') as f:
                json.dump(ordered_guide_json, f, indent=4)
            print(f"[INFO] 중심 좌표, 기울기 각도, 클래스를 {OUT_PATH}에 저장했습니다.")
            

            # 중심점이 그려진 이미지 저장
            cv2.imwrite(save_path_color, image_color)

            image_captured = True
            return ordered_guide_json

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[INFO] 종료 완료.")

if __name__ == "__main__":
    main()