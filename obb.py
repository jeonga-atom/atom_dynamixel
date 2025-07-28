#OBB- 오리엔티드 바운딩 박스 오브젝트 감지-> 방향성 물체 감지: 물체 감지보다 한 단계 더 나아가 이미지에서 물체를 더 정확하게 찾을 수 있도록 추가 각도를 도입
from ultralytics import YOLO

# Load a model
#model = YOLO("yolo11n-obb.yaml")  # 새로운 YOLO OBB 모델을 YAML 파일로부터 빌드- 새로운 모델을 YAML 설정 파일로 생성
model = YOLO("yolov8n-obb.pt")  # 사전 학습된 YOLO OBB 모델 로드 (학습에 추천됨)
#model = YOLO("yolo11n-obb.yaml").load("yolo11n.pt")  # YAML 설정으로 모델을 빌드하고, 사전 학습된 가중치를 불러오기-새로운 모델 구조에 기존 가중치 적용(전이 학습)

# Train the model
#results = model.train(data="dota8.yaml", epochs=100, imgsz=640) # 모델 학습, dota8.yaml: 데이터셋 경로, 클래스 정의, 학습/검증 데이터 경로 등이 포함된 설정 파일
results = model(source='/home/kminseo/Downloads/tennis.jpg', save=True)



#print('/home/kminseo/runs/obb/train/weights')

#DOTA 데이터 세트
# DOTA-v1: 물체 감지를 위해 방향이 지정된 경계 상자가 있는 포괄적인 항공 이미지 세트를 제공하는 DOTA 데이터 세트의 첫 번째 버전
# DOTA-v1.5: DOTA 데이터 세트의 중간 버전으로, 향상된 개체 감지 작업을 위해 DOTA-v1에 비해 추가 주석과 개선 사항을 제공
# DOTA-v2: 항공 이미지에서 물체 감지를 위한 대규모 데이터 세트(DOTA) 버전 2는 항공 관점에서의 감지를 강조하며 170만 개의 인스턴스와 11,268개의 이미지가 포함된 방향성 경계 상자를 포함
# DOTA8: 전체 DOTA 데이터 세트의 작은 8개 이미지 하위 집합으로, 워크플로 테스트 및 OBB 교육에 대한 지속적 통합(CI) 검사에 적합. ultralytics 리포지토리에 저장


# from ultralytics import YOLO
# import cv2 as cv
# import numpy as np

# # OBB 지원 모델 로드
# model = YOLO("yolov8n-obb.pt")  # 일반 모델이 아닌 OBB 모델로 변경

# cap = cv.VideoCapture(0)

# if not cap.isOpened():
#     print("웹캠을 열 수 없습니다.")
#     exit()

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("프레임을 읽을 수 없습니다.")
#         break

#     # 객체 추적 수행
#     results = model.predict(source=frame, conf=0.7,classes=[67])

#     # OBB 추출 및 시각화
#     for result in results:
#         if hasattr(result, 'obb') and result.obb is not None:
#             for obb in result.obb.xyxy.cpu():  # OBB 좌표 추출
#                 points = obb.numpy().astype(int).reshape(-1, 2)
#                 cv.polylines(frame, [points], isClosed=True, color=(0, 255, 0), thickness=2)

#     cv.imshow("YOLO OBB Tracking", frame)
#     if cv.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv.destroyAllWindows()



#학습 후 결과 확인 방법

# # 학습 결과 시각화
# results.plot()  # results.plot(): 학습 과정의 정확도, 손실 그래프 시각화- 학습 그래프, 손실(loss), mAP 등을 시각화

# # 학습된 모델로 검증 수행
# metrics = model.val()  # model.val(): 검증 데이터셋에서의 성능 평가 (mAP, Precision, Recall 등)

# # 이미지나 비디오에 대한 예측
# predictions = model.predict(source='test_image.jpg', save=True, conf=0.5) #model.predict(): 실제 이미지 또는 영상에 대해 객체 감지 수행
