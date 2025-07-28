#segment로 웹캠에 표시
from ultralytics import YOLO

def main():
    model = YOLO("yolov8l-seg.pt")
    result = model.track(source=0, show=True)



if __name__ == "__main__":
    main()

