# import cv2
# import supervision as sv
# from ultralytics import YOLO

# image = cv2.imread('/home/kminseo/Downloads/soccccccer.jpg') 
# model = YOLO("yolov8s.pt")
# result = model(image)[0]
# detections = sv.Detections.from_ultralytics(result)

# len(detections)

from ultralytics import YOLO
import cv2 as cv

cap = cv.VideoCapture(6)

def main():
    model=YOLO('yolov8l.pt')
    ret, frame = cap.read() 
    result = model.track(source=frame, show=True)


if __name__ == "__main__":
    main()


