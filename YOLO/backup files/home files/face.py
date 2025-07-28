
#파일이 있는지 확인하는 코드
# import os

# file_name = '/home/kminseo/Downloads/dataset/tedy/1.jpg'
# if os.path.exists(file_name):
#     print("File exists!")
# else:
#     print("File does not exist!")



# #1
# import cv2
# import face_recognition
# import pickle


# #son and tedy's face 10 pictures
# dataset_panths=['/home/kminseo/Downloads/dataset/son/','/home/kminseo/Downloads/dataset/tedy/']
# names=['son', 'tedy']
# number_images=10
# image_type='.jpg'
# encoding_file='encodings.pickle'

# model_method='cnn-gpu'

# knownEncodings=[]
# knownNames=[]

# for (i, dataset_path) in enumerate(dataset_panths):
#     name=names[i]

#     for idx in range(number_images):
#         file_name = dataset_path + str(idx+1) + image_type

#         image = cv2.imread(file_name)
#         rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

#         boxes = face_recognition.face_locations(rgb,
#             model=model_method)
        
#         encodings = face_recognition.face_encodings(rgb, boxes)

#         for encoding in encodings:
#             print(file_name, name, encoding)
#             knownEncodings.append(encoding)
#             knownNames.append(name)

# # pickle파일 형태로 데이터 저장
# data = {"encodings": knownEncodings, "names": knownNames}
# f = open(encoding_file, "wb")
# f.write(pickle.dumps(data))
# f.close()

# #2
# import cv2
# import face_recognition
# import pickle
# import time

# image_file = '/home/kminseo/Downloads/dataset/'
# encoding_file = 'encodings.pickle'
# unknown_name = 'Unknown'

# model_method = 'cnn-gpu'
# tolerance = 0.5

# def detectAndDisplay(image):
#     start_time = time.time()
#     rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

#     # 입력 이미지에서 각 얼굴에 해당하는 box 좌표를 감지하고 얼굴 임베딩 계산
#     boxes = face_recognition.face_locations(rgb,
#         model=model_method)
#     encodings = face_recognition.face_encodings(rgb, boxes)

#     # 감지된 각 얼굴의 이름 목록 초기화
#     names = []

#     # 얼굴 임베딩 반복
#     for encoding in encodings:
#         # 입력 이미지의 각 얼굴과 학습된 데이터 매치
#         matches = face_recognition.compare_faces(data["encodings"],
#             encoding, tolerance)
#         name = unknown_name

#         # 데이터가 매치된 경우
#         if True in matches:
#             # 일치하는 모든 얼굴의 인덱스를 찾고, 얼굴 일치 횟수 계산을 위한 초기화
#             matchedIdxs = [i for (i, b) in enumerate(matches) if b]
#             counts = {}

#             # 일치하는 인덱스를 반복하고, 인식된 각 얼굴의 수 유지
#             for i in matchedIdxs:
#                 name = data["names"][i]
#                 counts[name] = counts.get(name, 0) + 1

#             # 가장 많은 표를 얻은 label 선택
#             name = max(counts, key=counts.get)
        
#         # 이름 목록 업데이트
#         names.append(name)

#     # 인식된 얼굴 반복
#     for ((top, right, bottom, left), name) in zip(boxes, names):
#         # 이미지에 인식된 얼굴의 box를 그림
#         y = top - 15 if top - 15 > 15 else top + 15
#         # 학습된 얼굴(인식한 얼굴)의 경우 녹색 선
#         color = (0, 255, 0)
#         line = 2
#         # 학습되지 않은 얼굴(인식하지 못한 얼굴)의 경우 빨간색 선
#         if(name == unknown_name):
#             color = (0, 0, 255)
#             line = 2
#             name = 'Unknown'
#         # 사각형 그리기
#         cv2.rectangle(image, (left, top), (right, bottom), color, line)
#         y = top - 15 if top - 15 > 15 else top + 15
#         # 텍스트 추가
#         cv2.putText(image, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
#             0.75, color, line)
	
#     end_time = time.time()
#     # 소요시간 체크
#     process_time = end_time - start_time
#     print("=== A frame took {:.3f} seconds".format(process_time))
#     cv2.imshow("Recognition", image)
    
# # 학습된 데이터 load
# data = pickle.loads(open(encoding_file, "rb").read())

# # 테스트할 이미지 load
# image = cv2.imread(image_file)
# detectAndDisplay(image)

# cv2.waitKey(0)
# cv2.destroyAllWindows()

# #2-1
# import os
# import cv2
# import face_recognition
# import pickle
# import time

# # 경로 설정
# dataset_path = '/home/kminseo/Downloads/dataset/'
# encoding_file = 'encodings.pickle'
# unknown_name = 'Unknown'
# model_method = 'cnn-gpu'
# tolerance = 0.5  # 얼굴 매칭 허용 거리

# def detectAndDisplay(image, data):
#     start_time = time.time()
#     rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

#     # 입력 이미지에서 얼굴 위치 및 임베딩 계산
#     boxes = face_recognition.face_locations(rgb, model=model_method)
#     encodings = face_recognition.face_encodings(rgb, boxes)

#     names = []  # 감지된 얼굴 이름 목록

#     for encoding in encodings:
#         matches = face_recognition.compare_faces(data["encodings"], encoding, tolerance)
#         face_distances = face_recognition.face_distance(data["encodings"], encoding)
#         name = unknown_name

#         if True in matches:
#             matchedIdxs = [i for (i, b) in enumerate(matches) if b]
#             counts = {}

#             for i in matchedIdxs:
#                 counts[data["names"][i]] = counts.get(data["names"][i], 0) + 1

#             best_match_name = max(counts, key=counts.get)
#             best_match_index = face_distances.argmin()

#             if face_distances[best_match_index] <= tolerance:
#                 name = best_match_name
#             else:
#                 name = unknown_name

#         names.append(name)

#     for ((top, right, bottom, left), name) in zip(boxes, names):
#         y = top - 15 if top - 15 > 15 else top + 15
#         color = (0, 255, 0) if name != unknown_name else (0, 0, 255)
#         line = 2 if name != unknown_name else 1                     #if unknown, line thickness is 2
#         cv2.rectangle(image, (left, top), (right, bottom), color, line)
#         cv2.putText(image, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, line)

#     end_time = time.time()
#     print("=== A frame took {:.3f} seconds".format(end_time - start_time))
#     cv2.imshow("Recognition", image)

# # 학습된 데이터 로드
# data = pickle.loads(open(encoding_file, "rb").read())

# # 이미지 파일 자동 탐색 및 처리
# for file_name in os.listdir(dataset_path):
#     if file_name.lower().endswith(('.png', '.jpg', '.jpeg')):  # 이미지 파일 필터링
#         file_path = os.path.join(dataset_path, file_name)
#         print(f"Processing file: {file_path}")
#         image = cv2.imread(file_path)

#         if image is None:
#             print(f"Could not load image: {file_path}")
#             continue

#         detectAndDisplay(image, data)
#         cv2.waitKey(0)  # 키 입력 대기 (다음 이미지로 넘어가려면 키 누르기)

# cv2.destroyAllWindows()

#3. automatically find picture - chatgpt
# import os
# import cv2
# import face_recognition
# import pickle
# import time

# # 경로 설정
# dataset_path = '/home/kminseo/Downloads/dataset/'
# encoding_file = 'encodings.pickle'
# unknown_name = 'Unknown'
# model_method = 'cnn-gpu'
# tolerance = 0.5  # 얼굴 매칭 허용 거리

# def detectAndDisplay(image, data):
#     start_time = time.time()
#     rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

#     # 입력 이미지에서 얼굴 위치 및 임베딩 계산
#     boxes = face_recognition.face_locations(rgb, model=model_method)
#     encodings = face_recognition.face_encodings(rgb, boxes)

#     names = []  # 감지된 얼굴 이름 목록

#     for encoding in encodings:
#         matches = face_recognition.compare_faces(data["encodings"], encoding, tolerance)
#         face_distances = face_recognition.face_distance(data["encodings"], encoding)
#         name = unknown_name

#         if True in matches:
#             matchedIdxs = [i for (i, b) in enumerate(matches) if b]
#             counts = {}

#             for i in matchedIdxs:
#                 counts[data["names"][i]] = counts.get(data["names"][i], 0) + 1

#             best_match_name = max(counts, key=counts.get)
#             best_match_index = face_distances.argmin()

#             if face_distances[best_match_index] <= tolerance:
#                 name = best_match_name
#             else:
#                 name = unknown_name

#         names.append(name)

#     for ((top, right, bottom, left), name) in zip(boxes, names):
#         y = top - 15 if top - 15 > 15 else top + 15
#         color = (0, 255, 0) if name != unknown_name else (0, 0, 255)
#         line = 2 if name != unknown_name else 1
#         cv2.rectangle(image, (left, top), (right, bottom), color, line)
#         cv2.putText(image, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, line)

#     end_time = time.time()
#     print("=== A frame took {:.3f} seconds".format(end_time - start_time))
#     cv2.imshow("Recognition", image)

# # 학습된 데이터 로드
# data = pickle.loads(open(encoding_file, "rb").read())

# # 이미지 파일 자동 탐색 및 처리
# for file_name in os.listdir(dataset_path):
#     if file_name.lower().endswith(('.png', '.jpg', '.jpeg')):  # 이미지 파일 필터링
#         file_path = os.path.join(dataset_path, file_name)
#         print(f"Processing file: {file_path}")
#         image = cv2.imread(file_path)

#         if image is None:
#             print(f"Could not load image: {file_path}")
#             continue

#         detectAndDisplay(image, data)
#         cv2.waitKey(0)  # 키 입력 대기 (다음 이미지로 넘어가려면 키 누르기)

# cv2.destroyAllWindows()


#4. My automatically find picture
import os
import cv2
import face_recognition    #얼굴 감지 및 임베딩 계산 위한 라이브러리
import pickle              #데이터 직렬화/역직렬화하여 파일로 저장하거나 불러오는데 사용-> 직렬화: 메모리에 있는 객체 데이터를 이진 형식 또는 첵스트 형식으로 변환하는 과정(데이터 구조를 저장 가능한 포맷 Pickle)으로 변환, 역직렬화: 직렬화된 데이터를 원래의 데이터 구조로 복원하는 과정
import time                #코드 실행 시간 측정, 기록

# 경로 설정
dataset_path = '/home/kminseo/Downloads/dataset/'
encoding_file = 'encodings.pickle'
unknown_name = 'Unknown'
model_method = 'cnn-gpu'   #얼굴 감지 모델 방법(GPU 기반 CNN)
tolerance = 0.5            # 얼굴 매칭 허용 거리

def detectAndDisplay(image):
    start_time = time.time()
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)    # BGR 이미지를 RGB로 변환-> Open CV가 이미지를 RGB형태가 아닌 BGR 형태로 로딩하기 때문(cvtCOLOR를 사용)

    # 입력 이미지에서 얼굴 위치 및 임베딩 계산
    boxes = face_recognition.face_locations(rgb, model=model_method)    #입력 이미지에서 얼굴 위치 감지
    encodings = face_recognition.face_encodings(rgb, boxes)             #감지된 얼굴 위치를 기반으로 얼굴의 임베딩 계산

    names = []  # 감지된 얼굴 이름 목록(저장할 리스트)

    for encoding in encodings:
        matches = face_recognition.compare_faces(data["encodings"], encoding, tolerance)
        face_distances = face_recognition.face_distance(data["encodings"], encoding)    #입력 얼굴과 학습된 얼굴들 간의 거리를 계산
        name = unknown_name

        if True in matches:     #matches는 face_recognition.compare_faces함수가 반환된 리스트
            matchedIdxs = [i for (i, b) in enumerate(matches) if b]     #enumerate(matches): matches 리스트의 각 요소와 해당 인덱스 가져옴
            counts = {}

            for i in matchedIdxs:
                counts[data["names"][i]] = counts.get(data["names"][i], 0) + 1

            best_match_name = max(counts, key=counts.get)   #counts 딕셔너리에서 가장 높은 값을 반환
            best_match_index = face_distances.argmin()      #face_distance: 입력 얼굴 임베딩과 학습된 얼굴 임베딩 간의 유사도 나타내는 리스트 = 값이 작을 수록 두 얼굴이 더 비슷함
                                                            #face_distance.argmin(): 가장 작은 값을 가진 인덱스 반환 = 가장 짧은 거리로 매칭된 얼굴의 인덱스를 찾음
                                                            
            if face_distances[best_match_index] <= tolerance:   #face_distances[best_match_index]: 가장 짧은 거리의 값을 가져옴
                name = best_match_name                          #tolerance 값과 매칭된 거리와 비교하여 결정
            else:
                name = unknown_name

        names.append(name)

    for ((top, right, bottom, left), name) in zip(boxes, names):    #boxes: 얼굴감지함수에서 반환된 얼굴 위치 정보 리스트, 각 위치 좌표
        y = top - 15 if top - 15 > 15 else top + 15                 #텍스트가 얼굴의 상단에 표시되도록 Y좌표 조정, 얼굴의 top값에서 15픽셀 위로 텍스트 표시
        color = (0, 255, 0)     #기본 색상 초록색 (RGB: (0, 255, 0))
        line = 2                ## 기본 선 두께 2

        if (name==unknown_name):    #unkonon이면 색상 빨색으로 변경
            color=(0, 0, 255)
            line=2
            name='Unkown'

        cv2.rectangle(image, (left, top), (right, bottom), color, line) #cv2.rectangle: 얼굴 영역에 사각형을 그리는 OpenCV 함수
        y = top - 15 if top - 15 > 15 else top + 15
        cv2.putText(image, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, line)    #cv2.putText: 얼굴 위에 이름 텍스트를 그리는 OpenCV 함수

    end_time = time.time()  #처리 시간 계산

    process_time = end_time - start_time
    print("=== A frame took {:.3f} seconds".format(end_time - start_time))
    cv2.imshow("Recognition", image)

# 학습된 데이터 로드
data = pickle.loads(open(encoding_file, "rb").read())   


# 이미지 파일 자동 탐색 및 처리-> 이미지 하나만 실행하는 것이 아니라 여러 사진을 인식할 수 있도록 함
for file_name in os.listdir(dataset_path):
    if file_name.lower().endswith(('.png', '.jpg', '.jpeg')):  # 이미지 파일 필터링
        file_path = os.path.join(dataset_path, file_name)      #os.path.join: 디렉토리 경로(dataset_path)와 파일 이름을 결합하여 전체 파일 경로 생성
        print(f"Processing file: {file_path}")
        image = cv2.imread(file_path)                          #cv2.imread: OpenCV를 이용하여 이미지를 읽어옴. 반환값: NumPy 배열 형식

        if image is None:
            print(f"Could not load image: {file_path}")
            continue

        detectAndDisplay(image)     #이전에 정의한 함수로, 입력이미지 RGB로 변환, 얼굴 위치 감지 및 임베딩 생성, 학습된 데이터와 비교하여 이름 매칭, 결과 이미지를 출력 창에 표시
        cv2.waitKey(0)  # 키 입력 대기 (다음 이미지로 넘어가려면 키 누르기)

cv2.destroyAllWindows() #모든 Open CV 창 닫기