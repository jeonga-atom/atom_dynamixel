# 영상 받는 쪽
import zmq
import cv2
import base64
import numpy as np

# ZeroMQ 컨텍스트 생성
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://192.168.31.72:5555")  # 송신 측 주소
socket.setsockopt_string(zmq.SUBSCRIBE, "")  # 모든 메시지 수신

while True:
    # base64로 인코딩된 데이터 수신
    jpg_as_text = socket.recv()
    jpg_original = base64.b64decode(jpg_as_text)

    # OpenCV 이미지로 변환
    np_arr = np.frombuffer(jpg_original, dtype=np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    cv2.imshow("Received Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

