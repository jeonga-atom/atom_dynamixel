#선호 작품
from ultralytics import YOLO #YOLO를 제공하는 ultralytic로부터 yolo 임포트
import cv2 as cv

model = YOLO("best_obb.pt") #사용할 yolo의 모델 지정

cap = cv.VideoCapture(4) #openCV의 비디오캡처 기능 사용 (0: PC내의 웹캠, 1: 포트를 이용한 외부 웹캠) # 2: sdk, 4: sdk-webcam, 6: webcam

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

while True:
    ret, frame = cap.read()            #cap.read 역시 cap이 cv의 명령어이므로, openCV의 기능임. while문은 동영상형식으로 트래킹하기 위해 필수적임. 이는 yolo가 동영상 그 자체를 트래킹 하는게 아니라, 매 프레임마다 트래킹하기 때문임. 
    if not ret:                        #cap.read()로 비디오캡처 시, 웹캠 비디오를 읽어들임, 그것을 ret과 frame에 할당
        print("프레임을 읽을 수 없습니다.") # 'if not ret:'이라는 것은 ret값이 참/거짓을 가짐을 의미함. 'if not ret:'은 즉, ret이 존재하지 않으면? 0이라면?을 뜻함. 읽어드리지 못했다는 것을 뜻한다.
        break
    results = model.track(source=frame, persist=True) #model.track은 yolo에서 트래킹 하는 역할. 트래킹에 대한 특성을 '()' 내부에 기입하고 그 값을 results에 할당함
                                                                 #source는 트래킹을 어디서 해올것인지 소스를 찾는것, persist는 '유지'라는 뜻으로 프레임 마다의 트래킹 객체의 트래킹 정보를 유지할지의 여부
                                                                 #persist가 True이면 이전 프레임의 객체 정보를 유지해 연속적으로 사용, False이면 객체 정보 유지하지 않음
                                                                 #persist=True 뒤에 show=True를 사용하게 되면 cv.imshow를 사용하지 않고도 트래킹 영상을 띄우게 됨
                                                                 #단 이것에 차이가 있다면 show는 model.track 내부에서 사용하는 명령이므로 yolo가 내부적으로 띄우는 영상인 것이고, imshow를 띄운다면 이는 openCV가 띄우는 영상인것임.
                                                                 #즉, show문은 괄호내에서 제거해도됨, 아니 제거하는게 좋음 (show에 대해 굳이 명시하지 않았으므로 show=False인것임)
                                                                 #conf는 'confidence score'의 약자로 신뢰도를 뜻함. 즉, 어떤 클래스가 인지되었을 때, 그 인지의 믿을만한 정도를 뜻함. 형상이 흐리거나 부정확하면 이 값이 작아짐
                                                                 #comf=0.7은 0.7이상의 신뢰도를 가진 객체만 표기하겠다는 뜻임
    # #바운딩 박스 중간값 표시하기
    # boxes = results[0].boxes.xywh.cpu().detach().numpy()
    # center_point = [(round(xywh[0]), round(xywh[1])) for xywh in boxes]
    # print(center_point)


    cv.imshow("YOLO tracking", results[0].plot()) #results[0]이란 model.track(source=frame) 부분임. 읽어드린 비디오를 트래킹한 것을 화면으로 보이는 것으로 
                                                  #results[0].orig.img는 비디오의 원본 이미지, results[0].plot()은 객체 표시 및 라벨링한 비디오를 보여줌
                                                  #만약 cv.imshow 구문을 쓰지 않고 model.track내부의 show=True를 사용할 시, 화면이 보이나 심각한 프레임 드랍이 생기고, 밑에 사용했던 waitKey 명령어 실행 되지 않음. 즉 q버튼 무반응.
                                                  #cv.imshow()는 내부적으로 C++로 구현되어 있으며, 창 이름을 처리 할때 기본적으로 ASCII와 같은 표준 문자 인코딩 방식을 사용하므로 한글로 창이름을 설정할 시 웹캠이 제대로 작동되지 않음. 즉, ASCII코드로 처리가 되는 영어로 작성해야 함.

    if cv.waitKey(1) & 0xFF == ord('q'): #q버튼 누르면 종료한다는 뜻으로 ord는 유니코드 값이다. q를 눌렀을때의 유니코드 값인 113이 감지되면 브레이크한다는 것.
        break

cap.release()          #브레이크 됐다면 이곳으로 돌아와 cap.release로 openCV에게 요청한 데이터(비디오)를 가져오는 것을 중단하기 바랄때 사용되는 명령어임
cv.destroyAllWindows()



#내 작품
#동영상 tracking
# from ultralytics import YOLO

# model = YOLO('yolov8m.pt') #사용할 yolo 모델 지정(m은 모델 버전)

# results = model.track(source='movie.mp4', show=True, tracker='bytrack.yaml')

# #model.track: 물체 추적 실행
# #source='movie.mp4'는 처리할 비디오 파일 이름(웹캠 사용시 source=0)
# #show=True는 결괄르 화면에 출력하여 시각적으로 확인할 수 있게 설정
# #tracker='bytrack.yaml'는 물체 추적 알고리즘에 사용될 구성 파일 지정

#웹캠
# from ultralytics import YOLO
# import cv2 as cv

# model = YOLO('yolov8n.pt')

# # 웹캠 열기 (0은 기본 웹캠)
# cap = cv.VideoCapture(0)

# # 웹캠이 열리지 않은 경우 처리
# if not cap.isOpened():
#     print("웹캠을 열 수 없습니다. 장치를 확인하세요.")
#     exit()


# while True:
#     ret, frame = cap.read()     #ret 값이 False로 반환되면 웹캠에서 프레임을 가져오지 못했습니다로 뜸, 한마디로 not ret = False
#     if not ret:
#         print("웹캠에서 프레임을 가져오지 못했습니다.")
#         break
#     results = model.track(source=frame, persist=True, conf=0.7)     #model.track 방법은 정적 입력(이미지 파일 또는 비디오 경로)을 나타내며, 프레임 단위의 실시간 입력은 적합하지 않음.
#                                                                     #그래서 나는 model.track이 아닌 model.predict를 사용해 봄.

#     cv.imshow('웹캠 테스트', results[0].plot())
#     # 'q' 키를 누르면 종료
#     if cv.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv.destroyAllWindows()

#웹캠 테스트
# import cv2

# # 카메라 열기
# cap = cv2.VideoCapture(0)

# if not cap.isOpened():
#     print("웹캠을 열 수 없습니다.")
# else:
#     print("웹캠이 성공적으로 열렸습니다.")
#     ret, frame = cap.read()
#     if ret:
#         cv2.imshow("Webcam", frame)
#         cv2.waitKey(0)
#     cap.release()
#     cv2.destroyAllWindows()

