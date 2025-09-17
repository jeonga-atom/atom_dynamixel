import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import zmq
import base64
import time

# 토픽 설정
DEFAULT_ROS_DETECTED = {
    'button': '/object_detection/button',
    'fire_on': '/object_detection/fire/on',
    'fire_off': '/object_detection/fire/off',
    'door': '/object_detection/door',
}

# ZeroMQ 컨텍스트
context = zmq.Context()

# 영상 전송용 PUB 소켓
socket_video = context.socket(zmq.PUB)
socket_video.bind("tcp://*:5555")

# 탐지 결과 발행용 단일 PUB 소켓
socket_detection = context.socket(zmq.PUB)
socket_detection.bind("tcp://*:5556")

# 드라이브 토픽 수신용 단일 SUB 소켓
socket_drive = context.socket(zmq.SUB)
socket_drive.connect("tcp://localhost:5557")
for topic in ['button_start', 'door_start', 'reset']:
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

# 최종 토픽 단일 발행 추적 (버튼과 문에 사용)
published_once = {
    'button': False,
    'door': False,
}

# 화재 토픽 발행 타이밍 변수
fire_publish_time = 0  # button_start 시점 기록
fire_publish_delay = 10  # 10초 지연
fire_published = False  # 화재 토픽 발행 완료 여부

# ZMQ 폴러 설정
poller = zmq.Poller()
poller.register(socket_drive, zmq.POLLIN)

try:
    while True:
        # 드라이브 토픽 확인 (논블록킹)
        socks = dict(poller.poll(timeout=10))
        if socket_drive in socks:
            try:
                topic = socket_drive.recv_string()
                if topic == "reset":
                    detection_enabled = {key: False for key in detection_enabled}
                    published_once = {key: False for key in published_once}
                    fire_publish_time = 0
                    fire_published = False
                    print("Reset all flags and counters")
                else:
                    detection_enabled = {key: False for key in detection_enabled}
                    published_once = {key: False for key in published_once}
                    fire_publish_time = 0
                    fire_published = False

                    if topic == "button_start" and not published_once['button']:
                        socket_detection.send_multipart([DEFAULT_ROS_DETECTED['button'].encode(), b''])
                        published_once['button'] = True
                        detection_enabled['button'] = True

                        detection_enabled['fire'] = True  # 화재 모델 활성화
                        fire_publish_time = time.time()  # 10초 타이머 시작
                        print(f"Sent {DEFAULT_ROS_DETECTED['button']} and enabled fire detection")
                    
                    elif topic == "door_start" and not published_once['door']:
                        socket_detection.send_multipart([DEFAULT_ROS_DETECTED['door'].encode(), b''])
                        published_once['door'] = True
                        detection_enabled['door'] = True
                        print(f"Sent {DEFAULT_ROS_DETECTED['door']}")

            except zmq.ZMQError as e:
                print(f"ZMQ 수신 오류: {str(e)}")

        # 프레임 가져오기
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        # 탐지 플래그
        any_detection = False

        # 버튼 모델 처리
        if detection_enabled['button']:
            results_button = button_model(source=color_image, verbose=False)
            for result in results_button:
                boxes = result.boxes
                for i, box in enumerate(boxes):
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    confidence = box.conf[0].cpu().numpy()
                    class_id = int(box.cls[0].cpu().numpy())
                    
                    if confidence < 0.85 or class_id not in [0, 1, 2]:
                        continue
                    
                    any_detection = True
                    
                    color_map = {0: ("blue", (255, 0, 0)), 1: ("green", (0, 255, 0)), 2: ("yellow", (0, 255, 255))}
                    color, box_color = color_map.get(class_id, ("unknown", (0, 0, 255)))
                    
                    label = f"id: {class_id} conf: {confidence:.2f} color: {color}"
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), box_color, 2, cv2.LINE_AA)
                    cv2.putText(color_image, label, (x1, y1-10), cv2.FONT_HERSHEY_DUPLEX, 0.5, box_color, 2)

        # 문 모델 처리
        if detection_enabled['door']:
            results_door = door_model(source=color_image, verbose=False)
            for result in results_door:
                boxes = result.boxes
                masks = result.masks
                if masks is not None:
                    masks = masks.data.cpu().numpy()
                    for i, mask in enumerate(masks):
                        if i < len(boxes):
                            x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
                            confidence = boxes.conf[i].cpu().numpy()
                            class_id = int(boxes.cls[i].cpu().numpy())
                            
                            if confidence < 0.85:
                                continue
                            
                            any_detection = True
                            
                            mask_binary = (mask > 0.8).astype(np.uint8)
                            kernel = np.ones((3, 3), np.uint8)
                            mask_binary = cv2.dilate(mask_binary, kernel, iterations=1)
                            mask_binary = cv2.erode(mask_binary, kernel, iterations=1)
                            contours, _ = cv2.findContours(mask_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                            
                            if contours:
                                cv2.drawContours(color_image, contours, -1, (0, 255, 0), 2, cv2.LINE_AA)
                            
                            label = f"id: {class_id} conf: {confidence:.2f}"
                            cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0), 2)

        # 화재 모델 처리 (연속 실행, 시각화만 수행)
        if detection_enabled['fire']:
            results_fire = fire_model(source=color_image, verbose=False)
            for result in results_fire:
                boxes = result.boxes
                for i, box in enumerate(boxes):
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    confidence = box.conf[0].cpu().numpy()
                    class_id = int(box.cls[0].cpu().numpy())

                    if confidence < 0.85 or class_id not in [0, 1]:
                        continue

                    any_detection = True

                    fire_map = {0: ("off", (255, 255, 255)), 1: ("on", (0, 0, 255))}
                    fire_status, box_color = fire_map.get(class_id, ("unknown", (0, 0, 255)))

                    if fire_status == "unknown":
                        continue

                    label = f"id: {class_id} conf: {confidence:.2f} fire: {fire_status}"
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), box_color, 2, cv2.LINE_AA)
                    cv2.putText(color_image, label, (x1, y1-10), cv2.FONT_HERSHEY_DUPLEX, 0.5, box_color, 2)

        # 10초 후 fire_off 토픽 4번 발행
        if detection_enabled['fire'] and not fire_published and fire_publish_time > 0 and time.time() - fire_publish_time >= fire_publish_delay:
            socket_detection.send_multipart([DEFAULT_ROS_DETECTED['fire_off'].encode(), b''])
            socket_detection.send_multipart([DEFAULT_ROS_DETECTED['fire_off'].encode(), b''])
            socket_detection.send_multipart([DEFAULT_ROS_DETECTED['fire_off'].encode(), b''])
            socket_detection.send_multipart([DEFAULT_ROS_DETECTED['fire_off'].encode(), b''])
            
            print(f"Sent {DEFAULT_ROS_DETECTED['fire_off']} 4 times after 10 seconds")
            fire_published = True
            detection_enabled['fire'] = False  # 화재 모델 비활성화

        if not any_detection:
            cv2.putText(color_image, "Looking For", (35, 35), cv2.FONT_HERSHEY_DUPLEX, 1.0, (0, 0, 255), 2)

        # base64로 인코딩하여 영상 전송
        _, buffer = cv2.imencode('.jpg', color_image)
        jpg_as_text = base64.b64encode(buffer)
        socket_video.send(jpg_as_text)

finally:
    socket_detection.close()
    socket_drive.close()
    socket_video.close()
    context.term()
    pipeline.stop()
    cv2.destroyAllWindows()