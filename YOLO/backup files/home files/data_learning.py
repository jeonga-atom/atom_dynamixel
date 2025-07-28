# #PyTorch와 Ultralytics YOLO모델을 사용하여 GPU환경에서 객체 탐지 모델을 학습시키는 스크립트
# #PyTorch는 GPU와 연동해 딥러닝 모델 학습 및 추론을 지원->동적 계산 그래프 지웑

# import os

# import torch
# import torch.multiprocessing as mp

# from ultralytics import YOLO

# os.environ["CUDA_DEVICE_ORDER"]="PCI_BUS_ID"  # Arrange GPU devices starting from 0
#                                               #CUDA_DEVICE_ORDER: GPU 디바이스를 PCI버스 ID순서대로 정렬하도록 설정
# os.environ["CUDA_VISIBLE_DEVICES"]= "0"  # Set the GPU 0 to use
#                                         #사용할 GPU 지정, 0번 GPU를 사용하도록 지정

# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")   #디바이스 설정, CUDA가 사용 가능한 경우 GPU(CUDA)를 사용하고 그렇지 않으면 CPU를 사용하도록 설정

# print('Device:', device)    #GPU가 정상적으로 설정되었는지 설정
# print('Current cuda device:', torch.cuda.current_device())  #torch.cuda.current_device(): 현재 사용 중인 GPU의 ID를 반환
# print('Count of using GPUs:', torch.cuda.device_count())    #torch.cuda.device_count(): 사용 가능한 GPU의 총 개수를 반환

# if __name__ == '__main__':
#     #mp.set_start_method("spawn")은 멀티프로세싱 시 프로세스 시작 방식을 설정하는 부분, 현재는 사용하지 않음, Python 스크립트를 직접 실행할 때만 코드가 실행되도록 보장.

#     # YOLO모델 로드
#     model = YOLO('yolov8n.pt')  # yolov8x.pt: 사전에 학습된 파일

#     # Train: 학습
#     results = model.train(data='cfg/datasets/SOD_Drone.yaml', epochs=10, imgsz=640, device='cuda')
#     #data:데이터셋 구성 파일 경로. YAML 파일로 데이터셋 경로와 클래스 정보를 정의
#     #epochs: 학습 반복 횟수
#     #imgsz: 입력 이미지 크기(기본값: 640)
#     #device: 학습에 사용할 디바이스(GPU 설정).







# #depth-> realsense 카메라를 사용하여 depth 정보를 얻고 특정 위치(이미지 중앙)의 거리를 측정하는 코드
# import pyrealsense2 as rs   # pyrealsense2 라이브러리를 rs라는 별칭으로 임포트
# pipeline = rs.pipeline()    # rs.pipeline(): RealSense 카메라에서 데이터를 수집하는 기본 파이프라인을 생성-> 파이프라인은 카메라에서 실시간으로 스트리밍, 데이터 흐름을 관리하는 역할, 쉽게 말하면, 카메라로부터 실시간 데이터(예: 깊이 정보, 색상 이미지 등)를 캡처하고, 그 데이터를 파이프라인을 통해 처리하는 역할
# config = rs.config()        # rs.config(): 카메라 스트리밍을 설정하기 위한 구성을 생성
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  
# # 이 줄은 깊이 스트림을 설정, 깊이 스트림: realsense 카메라가 제공하는 3D 깊이 정보를 실시간으로 캡처하는 데이터 스트림으로 카메라와 물체사이의 거리를 측정하여 각 픽셀에 대한 깊이 값을 제공
# # rs.stream.depth: 깊이 카메라 스트림을 활성화
# # 640, 480: 카메라 해상도를 640x480 픽셀로 설정
# # rs.format.z16: 깊이 데이터를 16비트 형식으로 설정, 각 픽셀의 값은 거리 정보를 의미함.
# # 30: 초당 30프레임으로 스트리밍 설정

# pipeline.start(config)  #config(설정된 구성)으로 파이프라인을 시작하여 카메라에서 스트리밍 시작

# frames = pipeline.wait_for_frames()     # 파이프라인이 새 프레임을 받을 때까지 기다린 후, 받은 프레임을 frames 변수에 저장
# depth_frame = frames.get_depth_frame()  # frames.get_depth_frame()는 수신된 프레임에서 깊이 프레임을 추출. 이 깊이 프레임은 각 픽셀의 거리 정보를 포함
# distance = depth_frame.get_distance(320, 240)  # 이미지 중앙 거리-> 640X480 해상도 기준
# print(f"Distance: {distance} meters")   #측정된 거리를 출력. 해당 좌표(이미지 중앙)의 거리를 메터 단위로 출력
# pipeline.stop() #카메라 스트리밍 중지, 자원 해제하여 파이프라인 종료




# #카메라가 작동하는지 확인하는 코드
# import pyrealsense2 as rs
# import numpy as np
# import cv2

# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# pipeline.start(config)

# while True:
#     frames = pipeline.wait_for_frames()
#     depth_frame = frames.get_depth_frame()
#     color_frame = frames.get_color_frame()

#     if not depth_frame or not color_frame:
#         continue

#     depth_image = np.asanyarray(depth_frame.get_data())
#     color_image = np.asanyarray(color_frame.get_data())

#     depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

#     images = np.hstack((color_image, depth_colormap))

#     cv2.imshow('RealSense', images)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# pipeline.stop()



import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

# 깊이 값의 최소, 최대 범위 설정
depth_min = 0    # 가장 가까운 거리 (미터 단위)
depth_max = 4000  # 가장 먼 거리 (단위: 밀리미터)

while True:
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:
        continue

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # 깊이 값을 [0, 255] 범위로 정규화
    depth_normalized = np.clip(depth_image, depth_min, depth_max)
    depth_normalized = 255 - ((depth_normalized - depth_min) / (depth_max - depth_min) * 255).astype(np.uint8)

    # 사용자 정의 색상 맵 생성 (가까운 빨간색, 먼 파란색)
    depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_WINTER)

    # 색상 이미지와 깊이 맵 연결
    images = np.hstack((color_image, depth_colormap))

    cv2.imshow('RealSense', images)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()







## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#####################################################
## librealsense tutorial #1 - Accessing depth data ##
#####################################################

# First import the library
# import pyrealsense2 as rs

# try:
#     # Create a context object. This object owns the handles to all connected realsense devices
#     pipeline = rs.pipeline()

#     # Configure streams
#     config = rs.config()
#     config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

#     # Start streaming
#     pipeline.start(config)

#     while True:
#         # This call waits until a new coherent set of frames is available on a device
#         # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
#         frames = pipeline.wait_for_frames()
#         depth = frames.get_depth_frame()
#         if not depth: continue

#         # Print a simple text-based representation of the image, by breaking it into 10x20 pixel regions and approximating the coverage of pixels within one meter
#         coverage = [0]*64
#         for y in range(480):
#             for x in range(640):
#                 dist = depth.get_distance(x, y)
#                 if 0 < dist and dist < 1:
#                     coverage[x//10] += 1
            
#             if y%20 is 19:
#                 line = ""
#                 for c in coverage:
#                     line += " .:nhBXWW"[c//25]
#                 coverage = [0]*64
#                 print(line)
#     exit(0)
# #except rs.error as e:
# #    # Method calls agaisnt librealsense objects may throw exceptions of type pylibrs.error
# #    print("pylibrs.error was thrown when calling %s(%s):\n", % (e.get_failed_function(), e.get_failed_args()))
# #    print("    %s\n", e.what())
# #    exit(1)
# except Exception as e:
#     print(e)
#     pass









