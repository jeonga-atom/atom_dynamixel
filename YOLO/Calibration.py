# import numpy as np
# import cv2 as cv
# import glob

# # 종료 조건 설정: epsilon 또는 최대 반복 횟수(30)에 도달하면 알고리즘을 종료
# criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# # 체커보드의 3D 공간 상의 좌표 준비
# # 예: (0,0,0), (1,0,0), (2,0,0), ..., (6,5,0)
# objp = np.zeros((6*7, 3), np.float32)
# objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

# # 모든 이미지에서 객체 좌표와 이미지 좌표를 저장할 배열
# objpoints = []  # 실제 3D 공간에서의 점들
# imgpoints = []  # 이미지 평면에서의 2D 점들

# # 현재 디렉토리에 있는 모든 jpg 이미지를 가져옴
# images = glob.glob('/home/kminseo/*.jpg')

# # 각 이미지를 순회하면서 처리
# for fname in images:
#     # 이미지를 읽고 그레이스케일로 변환
#     img = cv.imread(fname)
#     gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

#     # 체커보드 코너를 찾음
#     ret, corners = cv.findChessboardCorners(gray, (7, 6), None)

#     # 코너를 찾았다면, 객체 좌표와 이미지 좌표를 추가
#     if ret == True:
#         objpoints.append(objp)  # 객체 좌표 추가
#         # 코너 위치를 더 정밀하게 조정
#         corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
#         imgpoints.append(corners2)  # 이미지 좌표 추가

#         # 체커보드 코너를 이미지에 그려서 표시
#         cv.drawChessboardCorners(img, (7, 6), corners2, ret)
#         cv.imshow('img', img)  # 이미지를 창에 표시
#         cv.waitKey(1000)  # 1초 동안 대기

# cv.destroyAllWindows()  # 모든 창 닫기

# # 카메라 캘리브레이션 수행
# ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# # 결과 출력
# print(ret, mtx, dist, rvecs, tvecs, sep='\n')



# #체커보드 촬영
# import cv2
# import datetime

# # 카메라 장치 열기
# cap = cv2.VideoCapture(4)

# # 영상 캡처 루프
# while True:
#     # 프레임 읽기
#     ret, frame = cap.read()
#     if not ret:
#         print("카메라에서 프레임을 가져올 수 없습니다.")
#         break

#     # 프레임을 화면에 표시
#     cv2.imshow("Video", frame)

#     # 키 입력 대기
#     key = cv2.waitKey(1) & 0xFF

#     # 'a' 키를 누르면 프레임 캡처하여 저장
#     if key == ord('a'):
#         # 파일 이름에 현재 날짜와 시간을 추가하여 저장
#         filename = datetime.datetime.now().strftime("capture_%Y%m%d_%H%M%S")
#         cv2.imwrite(f"{filename}.jpg", frame)
#         print(f"{filename} 이미지가 저장되었습니다.")

#     # 'q' 키를 누르면 종료
#     elif key == ord('q'):
#         break

# # 자원 해제
# cap.release()
# cv2.destroyAllWindows()



import cv2
import numpy as np
import os
import glob
import pickle

def calibrate_camera():
    # 체커보드의 차원 정의
    CHECKERBOARD = (7,10)  # 체커보드 행과 열당 내부 코너 수
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # 각 체커보드 이미지에 대한 3D 점 벡터를 저장할 벡터 생성
    objpoints = []
    # 각 체커보드 이미지에 대한 2D 점 벡터를 저장할 벡터 생성
    imgpoints = [] 
    
    # 3D 점의 세계 좌표 정의
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    
    # 주어진 디렉터리에 저장된 개별 이미지의 경로 추출
    images = glob.glob('/home/kminseo/*.jpg')
    
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # 체커보드 코너 찾기
        ret, corners = cv2.findChessboardCorners(gray,
                                               CHECKERBOARD,
                                               cv2.CALIB_CB_ADAPTIVE_THRESH +
                                               cv2.CALIB_CB_FAST_CHECK +
                                               cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            
            # 코너 그리기 및 표시
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(0)
    
    cv2.destroyAllWindows()
    
    # 카메라 캘리브레이션 수행
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints,
                                                      gray.shape[::-1], None, None)
    
    # 결과 출력
    print("Camera matrix : \n")
    print(mtx)
    print("\ndist : \n")
    print(dist)
    print("\nrvecs : \n")
    print(rvecs)
    print("\ntvecs : \n")
    print(tvecs)
    
    # 캘리브레이션 결과를 파일로 저장
    calibration_data = {
        'camera_matrix': mtx,
        'dist_coeffs': dist,
        'rvecs': rvecs,
        'tvecs': tvecs
    }
    
    with open('camera_calibration.pkl', 'wb') as f:
        pickle.dump(calibration_data, f)
    
    return calibration_data

def live_video_correction(calibration_data):
    mtx = calibration_data['camera_matrix']
    dist = calibration_data['dist_coeffs']
    
    cap = cv2.VideoCapture(4)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 프레임 크기 가져오기
        h, w = frame.shape[:2]
        
        # 최적의 카메라 행렬 구하기
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        
        # 왜곡 보정
        dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        
        # ROI로 이미지 자르기
        x, y, w, h = roi
        if all(v > 0 for v in [x, y, w, h]):
            dst = dst[y:y+h, x:x+w]
        
        # 원본과 보정된 이미지를 나란히 표시
        original = cv2.resize(frame, (640, 480))
        corrected = cv2.resize(dst, (640, 480))
        combined = np.hstack((original, corrected))
        
        # 결과 표시
        cv2.imshow('Original | Corrected', combined)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # 이미 캘리브레이션 파일이 있는지 확인
    if os.path.exists('camera_calibration.pkl'):
        print("Loading existing calibration data...")
        with open('camera_calibration.pkl', 'rb') as f:
            calibration_data = pickle.load(f)
            calibration_data = calibrate_camera()
    
    else:
        print("Performing new camera calibration...")
        calibration_data = calibrate_camera()
    
    # 실시간 비디오 보정 실행
    print("Starting live video correction...")
    live_video_correction(calibration_data)
    