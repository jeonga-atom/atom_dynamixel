from ultralytics import YOLO

# 사전 학습된 YOLOv8m 모델 로드
model = YOLO("yolov8m-seg.pt")

# 모델 정보 출력 (선택 사항)
model.info()

# 커스텀 데이터셋으로 모델 학습
results = model.train(
    data="/home/kminseo/ptfile/target_guide.v3i.yolov8/data.yaml", 
    epochs=500, 
    imgsz=640,
    device = 'cuda',
    )