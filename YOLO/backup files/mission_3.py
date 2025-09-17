import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import zmq
import base64

# 토픽 설정
DEFAULT_ROS_Detected = {
    'button': '/object_detection/button',
    'fire': '/object_detection/fire',
    'door': '/object_detection/door',
}

# ZeroMQ 컨텍스트
context = zmq.Context()

# 영상 전송용 PUB 소켓
socket_video = context.socket(zmq.PUB)
socket_video.bind("tcp://*:5555")

# 탐지 결과 발행용 단일 PUB 소켓
socket_detection = context.socket(zmq.PUB)
socket_detection.bind("tcp://*:5556")  # 단일 포트 사용, 필요 시 수정

# 드라이브 토픽 수신용 단일 SUB 소켓
socket_drive = context.socket(zmq.SUB)
socket_drive.connect("tcp://localhost:5557")  # 실제 서버 주소로 수정
for topic in ['button_start', 'fire_start', 'door_start']:
    socket_drive.setsockopt_string(zmq.SUBSCRIBE, topic)

# 모델 로드
button_model = YOLO('/home/kminseo/Documents/mission/Button/runs/detect/train/weights/last.pt')
fire_model = YOLO('/home/kminseo/Documents/mission/Fire/train2/weights/last.pt')
door_model = YOLO('/home/kminseo/Documents/mission/Door/runs/weights/last.pt')

# RealSense 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# Color 프레임에 맞춰 정렬
align_to = rs.stream.color
align = rs.align(align_to)

# 탐지 활성화 플래그
detection_enabled = {
    'button': False,
    'fire': False,
    'door': False
}

# ZMQ 폴러 설정
poller = zmq.Poller()
poller.register(socket_drive, zmq.POLLIN)

try:
    while True:
        # 드라이브 토픽 확인 (논블록킹)
        socks = dict(poller.poll(timeout=10))  # 10ms 타임아웃
        if socket_drive in socks:
            try:
                topic = socket_drive.recv_string()
                
                # 모든 플래그를 먼저 False로 리셋
                detection_enabled = {key: False for key in detection_enabled}

                if topic == "button_start":
                    detection_enabled['button'] = True
                    print("Button detection enabled")

                elif topic == "fire_start":
                    detection_enabled['fire'] = True
                    print("Fire detection enabled")

                elif topic == "door_start":
                    detection_enabled['door'] = True
                    print("Door detection enabled")

            except zmq.ZMQError as e:
                print(f"ZMQ 수신 오류: {str(e)}")

        # 프레임 가져오기
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()

        # 색상 프레임을 numpy 배열로 변환
        color_image = np.asanyarray(color_frame.get_data())

        # 탐지 플래그
        any_detection = False

        # 신규: 프레임당 탐지 추적기
        button_detected = False
        door_detected = False
        fire_off_detected = False
        fire_on_detected = False

        # 버튼 모델 처리
        if detection_enabled['button']:
            results_button = button_model(source=color_image, verbose=False)
            for result in results_button:
                boxes = result.boxes
                for i, box in enumerate(boxes):
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    confidence = box.conf[0].cpu().numpy()
                    class_id = int(box.cls[0].cpu().numpy())

                    if confidence < 0.8:
                        continue

                    any_detection = True

                    # 색상에 따라 바운딩 박스 색상 설정
                    color_map = {0: ("blue", (255, 0, 0)), 1: ("green", (0, 255, 0)), 2: ("yellow", (0, 255, 255))}

                    if class_id in color_map:
                        color, box_color = color_map[class_id]
                    else:
                        color, box_color = "unknown", (0, 0, 255)

                    label = f"id: {class_id} conf: {confidence:.2f} color: {color}"

                    cv2.rectangle(color_image, (x1, y1), (x2, y2), box_color, 2, cv2.LINE_AA)
                    cv2.putText(color_image, label, (x1, y1-10), cv2.FONT_HERSHEY_DUPLEX, 0.5, box_color, 2)
                    
                    socket_detection.send_multipart([DEFAULT_ROS_Detected['button'].encode(), b''])

        # 문 모델 처리
        if detection_enabled['door']:
            results_door = door_model(source=color_image, verbose=False)
            for result in results_door:
                boxes = result.boxes
                masks = result.masks if hasattr(result, 'masks') else None

                if masks is not None:
                    masks = masks.data.cpu().numpy()
                    for i, mask in enumerate(masks):
                        if i < len(boxes):
                            x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
                            confidence = boxes.conf[i].cpu().numpy()
                            class_id = int(boxes.cls[i].cpu().numpy())

                            if confidence < 0.8:
                                continue

                            any_detection = True

                            # 마스크 처리
                            mask_binary = (mask > 0.8).astype(np.uint8)
                            kernel = np.ones((3, 3), np.uint8)
                            mask_binary = cv2.dilate(mask_binary, kernel, iterations=1)
                            mask_binary = cv2.erode(mask_binary, kernel, iterations=1)

                            contours, _ = cv2.findContours(mask_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                            if contours:
                                cv2.drawContours(color_image, contours, -1, (0, 255, 0), 2, cv2.LINE_AA)
                                largest_contour = max(contours, key=cv2.contourArea)
                                rect = cv2.minAreaRect(largest_contour)
                                angle = rect[-1]
                                if angle < -45:
                                    angle = 90 + angle
                                else:
                                    angle = -angle
                                
                                angle_text = f"Angle: {angle:.2f}°" if angle is not None else "Angle: N/A"
                                cv2.putText(color_image, angle_text, (x1, y1 - 30), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255), 2)
            
                            label = f"id: {class_id} conf: {confidence:.2f}"
                            cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0), 2)
                            
                            socket_detection.send_multipart([DEFAULT_ROS_Detected['door'].encode(), b''])

        # 화재 모델 처리
        if detection_enabled['fire']:
            results_fire = fire_model(source=color_image, verbose=False)
            for result in results_fire:
                boxes = result.boxes
                for i, box in enumerate(boxes):
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    confidence = box.conf[0].cpu().numpy()
                    class_id = int(box.cls[0].cpu().numpy())

                    if confidence < 0.8:
                        continue

                    any_detection = True

                    # 화재에 따라 바운딩 박스 색상 설정
                    fire_map = {0: ("off", (255, 255, 255)), 1: ("on", (0, 0, 255))}

                    if class_id in fire_map:
                        fire_status, box_color = fire_map[class_id]
                    else:
                        fire_status, box_color = "unknown", (0, 0, 255)

                    label = f"id: {class_id} conf: {confidence:.2f} fire: {fire_status}"

                    cv2.rectangle(color_image, (x1, y1), (x2, y2), box_color, 2, cv2.LINE_AA)
                    cv2.putText(color_image, label, (x1, y1-10), cv2.FONT_HERSHEY_DUPLEX, 0.5, box_color, 2)
                    
                    # 화재 상태에 따라 다른 토픽으로 발행
                    if fire_status == "off":
                        socket_detection.send_multipart([b'/object_detection/fire/off', b''])
                    elif fire_status == "on":
                        socket_detection.send_multipart([b'/object_detection/fire/on', b''])
                    else:
                        socket_detection.send_multipart([DEFAULT_ROS_Detected['fire'].encode(), b''])
                        
        if not any_detection:
            cv2.putText(color_image, "Looking For", (35, 35), cv2.FONT_HERSHEY_DUPLEX, 1.0, (0, 0, 255), 2)
        
        # base64로 인코딩하여 영상 전송
        _, buffer = cv2.imencode('.jpg', color_image)
        jpg_as_text = base64.b64encode(buffer)
        socket_video.send(jpg_as_text)

finally:
    pipeline.stop()
    cv2.destroyAllWindows()