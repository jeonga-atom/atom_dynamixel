 
import cv2 as cv
import os

# RealSense 컬러 카메라 장치 번호 (ls /dev/video* 로 확인)
cap = cv.VideoCapture(4)  # 예: /dev/video4
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("RealSense 카메라를 열 수 없습니다.")
    exit()

save_dir = "captures"
os.makedirs(save_dir, exist_ok=True)

counter = 1  # 저장 시작 번호

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    cv.imshow("RealSense Color", frame)

    key = cv.waitKey(1) & 0xFF
    if key == ord('c'):  # 'c' 키로 캡처
        filename = os.path.join(save_dir, f"{counter}.jpg")
        cv.imwrite(filename, frame)
        print(f"저장 완료: {filename}")
        counter += 1
    elif key == ord('q'):  # 'q' 키로 종료
        break

cap.release()
cv.destroyAllWindows()