#터미널에서 좌표가 출력될 때 물체가 인식 될 때만 좌표가 출력이 됨, 물체가 인식이 안되면 좌표 출력이 사라짐
import pyrealsense2 as rs
import numpy as np
import cv2
import sys
import time

from ultralytics import YOLO

def main():
    # RealSense 파이프라인 초기화
    pipeline = rs.pipeline()
    config = rs.config()

    # 깊이 및 컬러 스트리밍 활성화
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Color 프레임에 맞춰 정렬
    align_to = rs.stream.color  
    align = rs.align(align_to)

    # YOLO 모델 로드
    model = YOLO('yolov8l-seg.pt')

    # 스트리밍 시작
    pipeline.start(config)

    try:
        while True:
            # 색깔, 거리 프레임 가져오기
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)  # Align 적용된 프레임
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # 깊이 및 컬러 이미지를 NumPy 배열로 변환
            color_image = np.asanyarray(color_frame.get_data())
            # depth_image = np.asanyarray(depth_frame.get_data()) #ndarray(n차원행렬)로 변환, 이번에 뎁스에 대한 프레임을 n차원행렬화, 이렇게 수치화를 시켜야 속도가 향상된다.
            # depth_image = cv2.medianBlur(depth_image.astype(np.float32), 5)  # 잡음 제거

            # YOLO 객체 탐지 수행
            results = model.track(color_image, verbose=False)

            time.sleep(0.005)

            # 터미널 출력할 정보 변수 초기화
            output_text = ""

            # 객체 탐지된 좌표 출력
            for result in results:
                for box, class_id in zip(result.boxes.xyxy.cpu(), result.boxes.cls.cpu()):
                    if int(class_id) == 67:  # 특정 클래스 ID (예: 67)
                        x1, y1, x2, y2 = map(int, box.numpy())
                        

                        # 바운딩 박스 중심 좌표 계산
                        obj_center_x = (x1 + x2) // 2
                        obj_center_y = (y1 + y2) // 2

                        # 객체 중심의 깊이 값 가져오기 (mm 단위)
                        obj_depth_value = depth_frame.get_distance(obj_center_x, obj_center_y) * 1000  # mm 변환
                        # object_depth = np.median(depth_image[y2-30:y2, x1:x2]) #np.median은 객체의 거리에 대한 중간값 
                
                        # 바운딩 박스 및 정보 출력
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (64, 224, 208), 1, cv2.LINE_AA)
                        cv2.putText(color_image, f"({obj_center_x}, {obj_center_y}), Depth: {obj_depth_value:.2f}mm",
                                    (obj_center_x + 10, obj_center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                        cv2.circle(color_image, (obj_center_x, obj_center_y), 2, (0, 255, 0), 2)   #초록색으로 점 표시
                        #점 찍는 법: cv2.circle(image, center, radius, color, thickness)

                        # 터미널 출력값 업데이트
                        output_text = f"Object at ({obj_center_x}, {obj_center_y}), Depth: {obj_depth_value:.2f} mm"

                        # print(f"Object at ({obj_center_x}, {obj_center_y}), Depth: {obj_depth_value:.2f} mm")
                        
                        # 터미널에서 출력값 덮여쓰기, 좌표 값만 바뀌도록
                        # print(f"\rObject at ({obj_center_x}, {obj_center_y}), Depth: {obj_depth_value:.2f} mm", end="")
                        # sys.stdout.flush()  # 출력 즉시 반영

            # 터미널 출력 (덮어쓰기)
            sys.stdout.write(f"\r{output_text.ljust(60)}")  # 길이 고정하여 이전 값 삭제 방지
            sys.stdout.flush()

            # 영상 출력
            cv2.imshow("Color Image", color_image)

            # ESC 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == 27:
                print()  # 종료 시 줄 바꿈 추가
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

