#pose 데이터셋 직접 만들려면 roboflow에서 스켈레톤으로 직접 라벨링해서 다운받아 학습시키기
#pose-human
# import cv2

# from ultralytics import YOLO

# def main():
#     model = YOLO('yolov8m-pose.pt')
#     # tracker = sv.ByteTrack() #트래킹의 세밀함을 위해 supervision의 Bytetrack 사용

#     results = model.track(source=0, show=True, conf=0.5, save=True) #model.track: 객체를 탐지하는 것뿐만 아니라 객체 추적까지 함.

#     for result in results: #단순 이미지 처리가 아닌 영상 처리에서는 for문을 반드시 사용하여 프레임 가공
#         frame = result.orig_img
        
#         cv2.imshow("pose-human", frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()


#pose-hand train
# from ultralytics import YOLO

# # Load a model
# model = YOLO("yolo11n-pose.pt")  # load a pretrained model (recommended for training)

# # Train the model
# results = model.train(data="hand-keypoints.yaml", epochs=30, imgsz=640)


#pose-hand

from ultralytics import YOLO #YOLO를 제공하는 ultralytic로부터 yolo 임포트
import cv2 as cv

model = YOLO("best_posehand.pt") #사용할 yolo의 모델 지정-----> pose, obb, segmentation은 웹캠 여는걸로 pt파일 불러오면 다 됨. 아니면 학습한 pt파일 집어넣기

cap = cv.VideoCapture(4) 

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

while True:
    ret, frame = cap.read()            
    if not ret:                        
        print("프레임을 읽을 수 없습니다.")
        break
    results = model.track(source=frame, persist=True, conf=0.5) 


    cv.imshow("YOLO tracking", results[0].plot()) 
    if cv.waitKey(1) & 0xFF == ord('q'): 
        break

cap.release()          
cv.destroyAllWindows()