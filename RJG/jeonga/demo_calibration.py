import sys
import os
import threading
import socket
import time
#from datetime import datetime
import cv2
#from UI_DEMO_SERVER_210617 import Toplevel1
import tkinter as tk
from PIL import Image
from PIL import ImageTk
import numpy as np
import copy
import json
import datetime
import math

def deproject_pixel_to_point(x, y, depth_raw, K, depth_scale=0.001):
    """
    픽셀(x,y) + depth_raw(z16)을 3D(mm->m)로 역투영
    K: 3x3 cameraMatrix
    depth_scale: 깊이 스케일 (보통 0.001)
    """
    Z = float(depth_raw) * float(depth_scale)
    if Z <= 0: 
        return [0.0, 0.0, 0.0]
    X = (x - K[0, 2]) * Z / K[0, 0]
    Y = (y - K[1, 2]) * Z / K[1, 1]
    return [X, Y, Z]

def project_point_to_pixel(point3d, K):
    """
    3D → 픽셀
    """
    X, Y, Z = point3d
    if Z == 0:
        return [0, 0]
    u = K[0, 0] * X / Z + K[0, 2]
    v = K[1, 1] * Y / Z + K[1, 2]
    return [int(round(u)), int(round(v))]

def get_3dpoints_from_depth(x, y, depth_value, intrinsics):
    """픽셀 좌표(x,y) + 깊이 → 카메라 좌표계 (X,Y,Z) [단위: m]"""
    fx, fy = intrinsics.fx, intrinsics.fy
    cx, cy = intrinsics.ppx, intrinsics.ppy
    if depth_value == 0:
        return [0, 0, 0]
    X = (x - cx) / fx * depth_value
    Y = (y - cy) / fy * depth_value
    Z = depth_value
    return [X, Y, Z]

def get_imagepoints_from_3d(point3d, intrinsics):
    """3D 좌표 (X,Y,Z) → 픽셀 좌표 (u,v)"""
    X, Y, Z = point3d
    fx, fy = intrinsics.fx, intrinsics.fy
    cx, cy = intrinsics.ppx, intrinsics.ppy
    if Z == 0:
        return [0, 0]
    u = (X * fx / Z) + cx
    v = (Y * fy / Z) + cy
    return [u, v]

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..', '..', '..', '..'))  # /root/volume/VISION
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from APP_Calibration.GUI.tkinter.UI_CALIBRATION_DARK import UIApp

class VISION_STATE:
    WAIT_INIT = 0
    COMPLETE_INIT = 1
    REQUEST_RESULT_FROM_ROBOT = 2
    PICK_DETECTING = 3
    READY_TO_SEND_PICKING_DATA = 4
    SET_VISION_ROI = 5
    REQUEST_RELOAD_DETECTOR_SET = 6

def get_vision_state():

    if CUR_VISION_STATE == VISION_STATE.WAIT_INIT:
        return 'VISION_STATE.WAIT_INIT'
    elif CUR_VISION_STATE == VISION_STATE.COMPLETE_INIT:
        return 'VISION_STATE.COMPLETE_INIT'
    elif CUR_VISION_STATE == VISION_STATE.REQUEST_RESULT_FROM_ROBOT:
        return 'VISION_STATE.REQUEST_RESULT_FROM_ROBOT'
    elif CUR_VISION_STATE == VISION_STATE.PICK_DETECTING:
        return 'VISION_STATE.PICK_DETECTING'
    elif CUR_VISION_STATE == VISION_STATE.READY_TO_SEND_PICKING_DATA:
        return 'VISION_STATE.READY_TO_SEND_PICKING_DATA'
    elif CUR_VISION_STATE == VISION_STATE.SET_VISION_ROI:
        return 'VISION_STATE.SET_VISION_ROI'
    elif CUR_VISION_STATE == VISION_STATE.REQUEST_RELOAD_DETECTOR_SET:
        return 'VISION_STATE.REQUEST_RELOAD_DETECTOR_SET'
    else:
        print("not implemented [func: get_vision_state]")
        print("please implement get_vision_state()")


# VISION Program STATE
CUR_VISION_STATE = VISION_STATE.WAIT_INIT
# VIEW MODE CHECK
VIEW_MODE = True
# CAPTURE CAMERA RGB, DEPTH, PICKING AI RESULT
rgb = []
depth = []
result = []
# FINAL PICKING ROBOT POS
PICKING_POS = [0,0,0,0,0,0]
ws_pts = []
client_sock = []

# configurations
gconfigs = []

gnormal_depth_threshold = 780

DEPTH_SCALE = 0.001

camera_ready = False
markerLength = 0.04

BASE_DIR = current_dir  # code 파일이 있는 폴더를 기준으로 저장
RESULT_DIR = os.path.join(BASE_DIR, 'result')
IMAGE_DIR  = os.path.join(BASE_DIR, 'image')

def ensure_dirs(paths):
    for p in paths:
        os.makedirs(p, exist_ok=True)

def showUIImg(cam, Label):

    if cam is not None and len(cam) > 0:
        cam_color_ = copy.deepcopy(cam)
        h=550
        w=int(1280/720*h)
        cam_color_ = cv2.resize(cam_color_, (w,h))
        # cam_color_ = cv2.resize(cam_color_, (1280, 720))
        # cam_color_ = cv2.resize(cam_color_, (640, 360))
        image = Image.fromarray(cam_color_)
        image = ImageTk.PhotoImage(image)

        if Label is None:
            Label = tk.Label(image=image)
            Label.image = image
            Label.pack(side="left")
        else:
            Label.configure(image=image)
            Label.image = image
def point3dinline_byRGB(corners,depth,part1_index,part2_index,cameraMatrix,distCoeffs,flag=0):

    if flag==0:     #right line
        index1=0
        index2=3
    else:           #left line
        index1=1
        index2=2

    corn=corners[part1_index]
    corn_3d1=Aruco_to_3dpoint(corn,markerLength,cameraMatrix,distCoeffs)
    corn1=corn[0, index1]
    corn=corners[part2_index]
    corn_3d2 = Aruco_to_3dpoint(corn, markerLength, cameraMatrix, distCoeffs)
    corn2=corn[0, index2]

    point0=corn_3d1[index1]
    point1=corn_3d1[index2]
    point2=corn_3d2[index1]
    point3=corn_3d2[index2]

    cpoint=[0,0,0]

    cpoint[0]=point3[0]+(point3[0]-point0[0])/10*2
    cpoint[1]=point3[1]+(point3[1]-point0[1])/10*2
    cpoint[2]=point3[2]+(point3[2]-point0[2])/10*2
    cpoint_pixel = project_point_to_pixel(cpoint, cameraMatrix)
    return cpoint_pixel,cpoint,corn1,corn2,point0,point3
def point3dinline(corners, depth, part1_index, part2_index, cameraMatrix, distCoeffs, flag=0):

    if flag == 0:     # right line
        index1 = 0
        index2 = 3
    else:             # left line
        index1 = 1
        index2 = 2

    # (포즈계산은 사용 안 하므로 있어도 무방)
    _ = cv2.aruco.estimatePoseSingleMarkers(corners[part1_index], 40, cameraMatrix, distCoeffs)
    _ = cv2.aruco.estimatePoseSingleMarkers(corners[part2_index], 40, cameraMatrix, distCoeffs)

    x = int(corners[part1_index][0][index1][0]); y = int(corners[part1_index][0][index1][1])
    point0 = deproject_pixel_to_point(x, y, depth[y, x], cameraMatrix, DEPTH_SCALE)

    x = int(corners[part1_index][0][index2][0]); y = int(corners[part1_index][0][index2][1])
    point1 = deproject_pixel_to_point(x, y, depth[y, x], cameraMatrix, DEPTH_SCALE)

    x = int(corners[part2_index][0][index1][0]); y = int(corners[part2_index][0][index1][1])
    point2 = deproject_pixel_to_point(x, y, depth[y, x], cameraMatrix, DEPTH_SCALE)

    x = int(corners[part2_index][0][index2][0]); y = int(corners[part2_index][0][index2][1])
    point3 = deproject_pixel_to_point(x, y, depth[y, x], cameraMatrix, DEPTH_SCALE)

    # 중간 지점 cpoint 계산
    cpoint = [0.0, 0.0, 0.0]
    cpoint[0] = point3[0] + (point3[0] - point0[0]) * 2.0 / 11.0
    cpoint[1] = point3[1] + (point3[1] - point0[1]) * 2.0 / 11.0
    cpoint[2] = point3[2] + (point3[2] - point0[2]) * 2.0 / 11.0

    # 3D -> 2D 투영
    cpoint_pixel = project_point_to_pixel(cpoint, cameraMatrix)
    return cpoint_pixel, cpoint

def safe_imwrite(path, img, why=''):
    ok = cv2.imwrite(path, img)
    if not ok:
        show_log(f'      @SAVE FAIL: {why} -> {path}')
    return ok

def normal_callback(event=None):
    global gnormal_depth_threshold
    show_log('changed normal threshold')
    gnormal_depth_threshold = int(ui_top.normal_depth_threshold.get())

def extract_aruco_line(image,depth,cameraMatrix):
    display_img = copy.deepcopy(image)
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    arucoParams = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    (corners, ids, rejected) = aruco_detector.detectMarkers(display_img)
    if corners.__len__()==0:
        # print("NO aruco")
        show_log('      @ NO detection aruco')
    else:
        flag_corners=(ids.flatten() == 2) | (ids.flatten() == 6) | (ids.flatten() == 1) | (ids.flatten() == 5)
        if np.where(flag_corners==True)[0].__len__()==4:
            show_log('      @ Success detection aruco')
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
            gray = cv2.cvtColor(display_img, cv2.COLOR_BGR2GRAY)
            new_corners = []
            for c in corners:
                _cor = cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), criteria)
                new_corners.append(_cor)
            corners = new_corners
            distCoeffs = np.array([0, 0, 0, 0, 0])
            color_list = [[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0]]

            # 2,6
            part1_index = np.where(ids.flatten() == 2)[0][0]
            part2_index = np.where(ids.flatten() == 6)[0][0]
            cpoint_pixel, cpoint = point3dinline(corners, depth, part1_index, part2_index, cameraMatrix, distCoeffs)
            if  math.isnan(cpoint_pixel[0]):
                cpoint_pixel[0]=0
            else:
                cpoint_pixel[0] = int(cpoint_pixel[0])

            if  math.isnan(cpoint_pixel[1]):
                cpoint_pixel[1]=0
            else:
                cpoint_pixel[1] = int(cpoint_pixel[1])

            cv2.circle(display_img, cpoint_pixel, 1, (0, 0, 0))
            cv2.putText(display_img, str(cpoint_pixel[0]) + " " + str(cpoint_pixel[1]), cpoint_pixel, 1, 1, (0, 255, 0))

            part1_index = np.where(ids.flatten() == 1)[0][0]
            part2_index = np.where(ids.flatten() == 5)[0][0]
            cpoint_pixel2, cpoint2 = point3dinline(corners, depth, part1_index, part2_index, cameraMatrix, distCoeffs,
                                                   flag=1)
            if  math.isnan(cpoint_pixel2[0]):
                cpoint_pixel2[0]=0
            else:
                cpoint_pixel2[0] = int(cpoint_pixel2[0])

            if  math.isnan(cpoint_pixel2[1]):
                cpoint_pixel2[1]=0
            else:
                cpoint_pixel2[1] = int(cpoint_pixel2[1])
            cv2.circle(display_img, cpoint_pixel2, 1, (0, 0, 0))
            cv2.putText(display_img, str(cpoint_pixel2[0]) + " " + str(cpoint_pixel2[1]), cpoint_pixel2, 1, 1, (0, 255, 0))

            # print(cpoint_pixel)
            # print(cpoint_pixel2)
            #
            # print(cpoint)
            # print(cpoint2)
            #
            calibration_2d = ((np.array(cpoint_pixel) + np.array(cpoint_pixel2)) / 2).tolist()
            calibration_2d[0] = int(calibration_2d[0])
            calibration_2d[1] = int(calibration_2d[1])

            cv2.circle(display_img, calibration_2d, 1, (0, 0, 255))
            # cv2.putText(display_img, str(calibration_2d[0]) + " " + str(calibration_2d[1]), calibration_2d, 1, 1, (0,0,255))

            calibration = ((np.array(cpoint2) + np.array(cpoint)) / 2).tolist()

            for index in range(0, corners.__len__()):
                _corner = corners[index]
                id = ids[index]
                _corner = _corner.astype(int)
                if index > 3:
                    index = index % 4
                _color = color_list[index]
                _corner = _corner.reshape(4, 2)
                for _c in _corner:
                    cv2.putText(display_img, str(_c[0]) + " " + str(_c[1]), _c, 1, 1, (0, 255, 0))
                cv2.putText(display_img, str(id), (_corner[0][0], _corner[0][1] + 30), 1, 1, (0, 0, 255))
            return [calibration,display_img]
        else:
            show_log('      @NO detection aruco       aruco 2: '+np.where(ids.flatten() == 2)[0].size+
                    '       aruco 6: '+np.where(ids.flatten() == 6)[0].size+
                     '       aruco 1: ' + np.where(ids.flatten() == 1)[0].size +
                     '       aruco 5: ' + np.where(ids.flatten() == 5)[0].size
                     )
    return [[],[]]

def extract_aruco_line_byRGB(image,depth,cameraMatrix):
    display_img = copy.deepcopy(image)
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    arucoParams = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    (corners, ids, rejected) = aruco_detector.detectMarkers(display_img)
    flag_show=False
    if corners.__len__()==0:
        # print("NO aruco")
        show_log('      @ NO detection aruco')
    else:
        flag_corners=(ids.flatten() == 2) | (ids.flatten() == 6) | (ids.flatten() == 1) | (ids.flatten() == 5)
        if np.where(flag_corners==True)[0].__len__()==4:
            show_log('      @ Success detection aruco')
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
            gray = cv2.cvtColor(display_img, cv2.COLOR_BGR2GRAY)
            new_corners = []
            for c in corners:
                _cor = cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), criteria)
                new_corners.append(_cor)
            corners = new_corners
            distCoeffs = np.array([0, 0, 0, 0, 0])
            color_list = [[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0]]

            # 2,6
            part1_index = np.where(ids.flatten() == 2)[0][0]
            part2_index = np.where(ids.flatten() == 6)[0][0]

            cpoint_pixel,cpoint,corn1,corn2,corn_3d1,corn_3d2=point3dinline_byRGB(corners, depth, part1_index, part2_index, cameraMatrix, distCoeffs)

            if  math.isnan(cpoint_pixel[0]):
                cpoint_pixel[0]=0
            else:
                cpoint_pixel[0] = int(cpoint_pixel[0])

            if  math.isnan(cpoint_pixel[1]):
                cpoint_pixel[1]=0
            else:
                cpoint_pixel[1] = int(cpoint_pixel[1])

            cv2.circle(display_img, cpoint_pixel, 4, (0, 0, 0))
            # cv2.putText(display_img, str(cpoint_pixel[0]) + " " + str(cpoint_pixel[1]), cpoint_pixel, 1, 1, (0, 255, 0))
            cv2.putText(display_img, "p1"+str(corn1[0]) + " " + str(corn1[1]), corn1.astype(int).tolist(), 1, 1, (0, 255, 0))
            cv2.putText(display_img, "p2" + str(corn2[0]) + " " + str(corn2[1]), corn2.astype(int).tolist(), 1, 1, (0, 255, 0))
            if flag_show:
                cv2.imshow("test",display_img)
                cv2.waitKey(0)

            part1_index = np.where(ids.flatten() == 1)[0][0]
            part2_index = np.where(ids.flatten() == 5)[0][0]
            cpoint_pixel2,cpoint2,corn1,corn2,corn_3d1,corn_3d2=point3dinline_byRGB(corners, depth, part1_index, part2_index, cameraMatrix, distCoeffs,
                                                   flag=1)
            if  math.isnan(cpoint_pixel2[0]):
                cpoint_pixel2[0]=0
            else:
                cpoint_pixel2[0] = int(cpoint_pixel2[0])

            if  math.isnan(cpoint_pixel2[1]):
                cpoint_pixel2[1]=0
            else:
                cpoint_pixel2[1] = int(cpoint_pixel2[1])
            cv2.circle(display_img, cpoint_pixel2,4, (0, 0, 0))
            # cv2.putText(display_img, str(cpoint_pixel2[0]) + " " + str(cpoint_pixel2[1]), cpoint_pixel2, 1, 1, (0, 255, 0))
            cv2.putText(display_img, "p1"+str(corn1[0]) + " " + str(corn1[1]), corn1.astype(int).tolist(), 1, 1, (0, 255, 0))
            cv2.putText(display_img, "p2" + str(corn2[0]) + " " + str(corn2[1]), corn2.astype(int).tolist(), 1, 1, (0, 255, 0))
            if flag_show:
                cv2.imshow("test",display_img)
                cv2.waitKey(0)

            # print(cpoint_pixel)
            # print(cpoint_pixel2)
            #
            # print(cpoint)
            # print(cpoint2)
            #
            middle=(np.array(cpoint2) + np.array(cpoint)) / 2
            cpoint_pixel = project_point_to_pixel(middle, cameraMatrix)
            calibration_2d = cpoint_pixel
            calibration_2d[0] = int(calibration_2d[0])
            calibration_2d[1] = int(calibration_2d[1])
            cv2.circle(display_img, calibration_2d, 1, (0, 0, 255))
            # cv2.putText(display_img, str(calibration_2d[0]) + " " + str(calibration_2d[1]), calibration_2d, 1, 1, (0,0,255))

            calibration = middle.tolist()

            for index in range(0, corners.__len__()):
                _corner = corners[index]
                id = ids[index]
                _corner = _corner.astype(int)
                if index > 3:
                    index = index % 4
                _color = color_list[index]
                _corner = _corner.reshape(4, 2)
                for _c in _corner:
                    cv2.putText(display_img, str(_c[0]) + " " + str(_c[1]), _c, 1, 1, (0, 255, 0))
                cv2.putText(display_img, str(id), (_corner[0][0], _corner[0][1] + 30), 1, 1, (0, 0, 255))
            return [calibration,display_img]
        else:
            show_log('      @NO detection aruco       aruco 2: '+np.where(ids.flatten() == 2)[0].size+
                    '       aruco 6: '+np.where(ids.flatten() == 6)[0].size+
                     '       aruco 1: ' + np.where(ids.flatten() == 1)[0].size +
                     '       aruco 5: ' + np.where(ids.flatten() == 5)[0].size
                     )
    return [[],[]]
def save_calibration(calibration):
    show_log('      @ Save calibration')
    calibration_data = {"calibration_data": calibration}
    with open(os.path.join(BASE_DIR, "calibration.json"), "w") as json_file:
        json.dump(calibration_data, json_file)



def calibration_RT(rgb, depth, date_str):
    display_img = copy.deepcopy(rgb)
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    distCoeffs = np.zeros(5, dtype=np.float32)
    # 0) cameraMatrix 준비 확인
    global cameraMatrix
    if 'cameraMatrix' not in globals():
        show_log('      @RT FAIL: cameraMatrix not ready yet (sensor thread not started?).')
        showUIImg(display_img, ui_top.img_tab3_label)
        return None, None
    # 1) 마커 검출
    corners, ids, rejected = cv2.aruco.detectMarkers(display_img, arucoDict)
    if ids is None or len(ids) == 0:
        show_log('      @RT FAIL: no ArUco detected.')
        showUIImg(display_img, ui_top.img_tab3_label)
        return None, None
    # 디버그 표시
    for i in range(len(ids)):
        p = corners[i][0][0]
        cv2.putText(display_img, f"[ {int(ids[i])} ]", p.astype(int).tolist(), 1, 1, (0,0,255))
    # 2) RT는 0~7 ID 8개 전제 → 검증
    required = np.arange(8)
    ids_flat = ids.flatten()
    has = np.isin(required, ids_flat)
    if not has.all():
        show_log(f'      @RT FAIL: need IDs 0~7. detected: {ids_flat.tolist()}')
        showUIImg(display_img, ui_top.img_tab3_label)
        return None, None
    # 3) ID 0~7 순서로 코너 정렬
    #    (ids==k)인 인덱스를 찾아 그 코너를 같은 순서로 담는다
    ordered_corners = []
    for k in required:
        idx = int(np.where(ids_flat == k)[0][0])
        ordered_corners.append(corners[idx])
    # 4) 보드의 월드 좌표(소스) — ID 0~7 순서
    src = np.array([
        [[-11,-12,0],[-7,-12,0],[-7,-8,0],[-11,-8,0]],  # 0
        [[ -5,-12,0],[ -1,-12,0],[ -1,-8,0],[  -5,-8,0]],# 1
        [[  1,-12,0],[  5,-12,0],[  5,-8,0],[   1,-8,0]],# 2
        [[  7,-12,0],[ 11,-12,0],[ 11,-8,0],[   7,-8,0]],# 3
        [[-11, -6,0],[ -7, -6,0],[ -7,-2,0],[ -11,-2,0]],# 4
        [[ -5, -6,0],[ -1, -6,0],[ -1,-2,0],[  -5,-2,0]],# 5
        [[  1, -6,0],[  5, -6,0],[  5,-2,0],[   1,-2,0]],# 6
        [[  7, -6,0],[ 11, -6,0],[ 11,-2,0],[   7,-2,0]],# 7
    ], dtype=float)
    # 5) 각 마커의 3D 코너를 카메라 좌표계로 계산(dst)
    dst_list = []
    markersize = markerLength  # mm 등, 보드 설계 단위와 일치해야 함
    for cor in ordered_corners:
        cor3d = Aruco_to_3dpoint(cor, markersize, cameraMatrix, distCoeffs)  # (4,3)
        dst_list.append(cor3d)
    # 6) estimateAffine3D
    src_flat = src.reshape(-1, 3).astype(np.float32)
    dst_flat = np.array(dst_list, dtype=np.float32).reshape(-1, 3)
    ok, RT, inliers = cv2.estimateAffine3D(src_flat, dst_flat)
    if not ok:
        show_log('      @RT FAIL: estimateAffine3D failed.')
        showUIImg(display_img, ui_top.img_tab3_label)
        return None, None
    R = RT[:, 0:3]
    T = RT[:, 3]
    # 7) 로테이션 벡터 → 도(deg)로
    rvec, _ = cv2.Rodrigues(R)
    anglevector = np.rad2deg(rvec).ravel()
    show_log('      @angle : x: '+str(anglevector[0])+' y: '+str(anglevector[1])+' z: '+str(anglevector[2]))
    show_log('      @calibration  x: '+str(T[0])+' y: '+str(T[1])+' z: '+str(T[2]))
    save_calibration(T.tolist())
    showUIImg(display_img, ui_top.img_tab3_label)
    # 선택: 결과 이미지 저장 전에 폴더 확인
    try:
        os.makedirs('./result', exist_ok=True)
        cv2.imwrite(f'./result/{date_str}pnp_result.png', display_img)
    except Exception as e:
        show_log(f'      @SAVE WARN: {e}')
    return R, T
def calibration_pnp(rgb,depth,date_str):
    show_log('  @ calibration')
    if rgb is None or not hasattr(rgb, 'size') or rgb.size == 0:
        show_log('  @ERROR: RGB image is empty!')
        return
    if depth is None or not hasattr(depth, 'size') or depth.size == 0:
        show_log('  @ERROR: Depth image is empty!')
        return
    RGB=copy.deepcopy(rgb)
    Depth=copy.deepcopy(depth)
    [calibration,display_img]=extract_aruco_line_byRGB(RGB,Depth,cameraMatrix)
    # [calibration,display_img]=extract_aruco_line(rgb,depth,cameraMatrix)
    show_log('      @ calibration ' + " x: "+str(calibration[0])+" y: "+ str(calibration[1])+ " z: "+str(calibration[2]))
    showUIImg(display_img, ui_top.img_tab3_label)
    save_calibration(calibration)
    ensure_dirs([RESULT_DIR, IMAGE_DIR])
    input_path  = os.path.join(RESULT_DIR, f'{date_str}pnp_input.png')
    depth_path  = os.path.join(RESULT_DIR, f'{date_str}pnp_depth.png')
    result_path = os.path.join(RESULT_DIR, f'{date_str}pnp_result.png')

    cv2.imwrite(input_path, RGB[:, :, ::-1])
    cv2.imwrite(depth_path,Depth)
    cv2.imwrite(result_path,display_img)
def calibration(rgb,depth,date_str):
    show_log('  @ calibration')
    if rgb is None or not hasattr(rgb, 'size') or rgb.size == 0:
        show_log('  @ERROR: RGB image is empty!')
        return
    if depth is None or not hasattr(depth, 'size') or depth.size == 0:
        show_log('  @ERROR: Depth image is empty!')
        return
    RGB=copy.deepcopy(rgb)
    Depth=copy.deepcopy(depth)
    [calibration,display_img]=extract_aruco_line(RGB,Depth,cameraMatrix)
    # [calibration,display_img]=extract_aruco_line(rgb,depth,cameraMatrix)
    show_log('      @ calibration ' + " x: "+str(calibration[0])+" y: "+ str(calibration[1])+ " z: "+str(calibration[2]))
    showUIImg(display_img, ui_top.img_tab3_label)
    save_calibration(calibration)


    input_path = './result/' + date_str + "input.png"
    depth_path = './result/' + date_str + "depth.png"
    result_path = './result/' + date_str + "result.png"

    cv2.imwrite(input_path, RGB[:, :, ::-1])
    cv2.imwrite(depth_path,Depth)
    cv2.imwrite(result_path,display_img)
def Aruco_to_3dpoint(corn, markersize, cameraMatrix, distCoeffs):
    pose1 = cv2.aruco.estimatePoseSingleMarkers(corn, markersize, cameraMatrix, distCoeffs)
    rvec = pose1[0]
    tvec = pose1[1]
    mrv, jacobian = cv2.Rodrigues(rvec)
    markercorners = np.matmul(mrv, pose1[2].reshape(4, 3).T).T + np.tile(tvec.flatten(), [4, 1])
    markercorners = markercorners.reshape([4, 3])
    return markercorners

def compare_algorithm():
    """
    현재 카메라에서 캡처된 RGB/Depth 프레임을 이용해
    calibration_RT()와 calibration_pnp()를 테스트합니다.
    """
    global rgb, depth

    # 현재 프레임이 준비되지 않은 경우 예외 처리
    if rgb is None or not hasattr(rgb, 'size') or rgb.size == 0:
        show_log('  @TEST FAIL: current RGB is empty')
        return
    if depth is None or not hasattr(depth, 'size') or depth.size == 0:
        show_log('  @TEST FAIL: current Depth is empty')
        return

    # 현재 프레임 복사
    RGB = copy.deepcopy(rgb)
    Depth = copy.deepcopy(depth)

    # 파일명용 타임스탬프
    date_str = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S_')

    show_log('  @TEST START: using current live frames')

    # Calibration 계산
    calibration_RT(RGB, Depth, date_str)
    calibration_pnp(RGB, Depth, date_str)

    show_log('  @TEST DONE')

def test_algorithm():
    Depth=cv2.imread("./result/2021_08_12_16_28_22_depth.png",-1)
    RGB = cv2.imread("./result/2021_08_12_16_28_22_input.png")

    #detph 이용
    display_img = copy.deepcopy(RGB)
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    arucoParams = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    (corners, ids, rejected) = aruco_detector.detectMarkers(display_img)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    gray = cv2.cvtColor(display_img, cv2.COLOR_BGR2GRAY)
    new_corners = []
    for c in corners:
        _cor = cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), criteria)
        new_corners.append(_cor)
    corners = new_corners
    distCoeffs = np.array([0, 0, 0, 0, 0])
    markersize=markerLength
    for corn in corners:
        pose1 = cv2.aruco.estimatePoseSingleMarkers(corn,markersize, cameraMatrix, distCoeffs)
        rvec=pose1[0]
        tvec = pose1[1]
        mrv, jacobian = cv2.Rodrigues(rvec)
        markercorners =np.matmul(mrv, pose1[2].reshape(4, 3).T).T + np.tile(tvec.flatten(), [4, 1])
        markercorners_rs=[]
        for c in corn[0]:
            x = int(c[0]); y = int(c[1])
            markercorners_rs.append(
                deproject_pixel_to_point(x, y, Depth[y, x], cameraMatrix, DEPTH_SCALE)
            )
        markercorners_rs=np.array(markercorners_rs)
        markercorners=markercorners.reshape([4, 3])
        for index in range(0,4):
            print(markercorners[index],markercorners_rs[index])



def sensor():
    global CUR_VISION_STATE
    global VIEW_MODE
    global rgb, depth, result, ws_pts,cameraMatrix
    global gconfigs
    global rs_sensor
    global camera_ready
    import cv2
    from ketisdk.ketisdk.sensor.realsense_sensor import RSSensor

    # get data from realsene
    rs_sensor = RSSensor()
    rs_sensor.start()
    try:
        # ketisdk가 제공하면 사용
        global DEPTH_SCALE
        DEPTH_SCALE = float(getattr(rs_sensor, 'depth_scale', 0.001))
        show_log(f'      @depth_scale = {DEPTH_SCALE}')
    except Exception:
        pass
    intrinsic_params = rs_sensor.intr_params
    cameraMatrix = np.array(
        [intrinsic_params.fx, 0, intrinsic_params.ppx, 0, intrinsic_params.fy, intrinsic_params.ppy, 0, 0, 1]).reshape(
        3, 3)
    camera_ready = True

    while True:
        rgb, depth = rs_sensor.get_data()
        display_img = copy.deepcopy(rgb)
        display_depth = copy.deepcopy(depth)

        if VIEW_MODE==True :

            if CUR_VISION_STATE != VISION_STATE.READY_TO_SEND_PICKING_DATA and CUR_VISION_STATE != VISION_STATE.SET_VISION_ROI:
                ws_pts_copy = np.array(ws_pts, int)
                display_img = cv2.polylines(display_img, [ws_pts_copy], True, (0, 255, 0), 2)
                #showUIImg(display_img, ui_top.Label1)
                showUIImg(display_img, ui_top.img_tab1_label)

                depth_min = 0
                depth_max = 0
                depth_vis_manual = True


                ## depth disp
                if gconfigs != [] and gconfigs.disp.depth_vis_manual:
                    display_depth[display_depth > gconfigs.disp.depth_vis_max_val] = gconfigs.disp.depth_vis_max_val
                    display_depth -= gconfigs.disp.depth_vis_min_val
                    display_depth = display_depth / gconfigs.disp.depth_vis_max_val
                    display_depth *= 255
                    display_depth = display_depth.astype(np.uint8)

                    depth_min = gconfigs.disp.depth_vis_min_val
                    depth_max = gconfigs.disp.depth_vis_max_val

                else:
                    #print("depth min and max: " + str(display_depth.min()) + ", " + str(display_depth.max()))
                    depth_min = 100#display_depth.min()
                    depth_max = 1500#display_depth.max()
                    display_depth -= depth_min
                    display_depth = display_depth / depth_max
                    display_depth *= 255
                    depth_vis_manual = False

                display_depth = display_depth.astype(np.uint8)
                display_depth = cv2.cvtColor(display_depth, cv2.COLOR_GRAY2BGR)
                depth_disp_text = "Depth Vis Manual: " + str(depth_vis_manual) + " Min: " + str(depth_min) + " Max: " + str(depth_max)
                cv2.putText(display_depth, depth_disp_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1 , (0, 255, 0), 1)

                showUIImg(display_depth, ui_top.img_tab2_label)
                image = copy.deepcopy(display_img)
                if 0:
                    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
                    arucoParams = cv2.aruco.DetectorParameters()
                    aruco_detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
                    (corners, ids, rejected) = aruco_detector.detectMarkers(display_img)
                    if corners.__len__()>0:
                        for index in range(0, ids.__len__()):
                            point1 = corners[index][0][0]
                            cv2.putText(image, "[ " + str(ids[index]) + " ]", point1.astype(int).tolist(), 1, 1,
                                        (0, 0, 255))
                h, w, c = image.shape
                for y in range(0, h, 100):
                    cv2.line(image, [0, y], [w, y], [255, 0, 0])
                showUIImg(image, ui_top.img_tab3_label)

            if result != [] and CUR_VISION_STATE == VISION_STATE.READY_TO_SEND_PICKING_DATA:
                #showUIImg(result[:,:,::-1], ui_top.Label1)
                showUIImg(result[:, :, ::-1], ui_top.img_tab1_label)

            if CUR_VISION_STATE == VISION_STATE.SET_VISION_ROI:
                if ws_pts.__len__() > 0:
                    for pts in ws_pts:
                        cv2.circle(display_img, pts, 2, [0,255,0],2)

                if ws_pts.__len__() > 2:
                    ws_pts_copy = np.array(ws_pts, int)
                    display_img = cv2.polylines(display_img, [ws_pts_copy], True, (0, 255, 0), 2)

                showUIImg(display_img, ui_top.img_tab1_label)
                #showUIImg(display_img, ui_top.img_tab2_label)

def button_HandeyeCalibration():
    global CUR_VISION_STATE
    show_log('  @ClickedButton_HandeyeCalibration')
    calibration(rgb,depth,"1")

def button_HandeyeCalibration_pnp():
    global CUR_VISION_STATE, camera_ready
    if not camera_ready:
        show_log('  @WAIT: camera not ready yet. Please wait until RealSense starts.')
        return
    date_str = datetime.datetime.now().strftime(
        '%Y_%m_%d_%H_%M_%S_')
    show_log('  @ClickedButton_HandeyeCalibration_pnp')
    copy_rgb=copy.deepcopy(rgb)
    copy_depth= copy.deepcopy(depth)

    ensure_dirs(['./result', './image'])
    
    calibration_pnp(copy_rgb,copy_depth,date_str)
    show_log('  @ClickedButton_R,T_pnp')
    calibration_RT(copy_rgb,copy_depth,date_str)
def load_calibration():
    with open(os.path.join(BASE_DIR,'calibration.json')) as json_file:
        return json.load(json_file)["calibration_data"]
    return calibration_data

def button_Calibration():
    global CUR_VISION_STATE
    show_log('  @ClickedButton_load_calibration')
    calibration_data=load_calibration()
    show_log("  @load_calibration :"+str(calibration_data))
def button_test_image():
    global CUR_VISION_STATE
    show_log('  @ClickedButton_test_image')
    # test_algorithm()
    compare_algorithm()
def button_save_image():
    global CUR_VISION_STATE
    show_log('  @ClickedButton_save_image')
    RGB=copy.deepcopy(rgb)
    Depth=copy.deepcopy(depth)
    show_log('  @ClickedButton_4')
    date_str=datetime.datetime.now().strftime(
        '%Y_%m_%d_%H_%M_%S_')
    input_path = './image/' + date_str+ "_input.png"
    depth_path = './image/' + date_str + "_depth.png"

    cv2.imwrite(input_path, rgb[:, :, ::-1])
    cv2.imwrite(depth_path, depth)

def button_setroi():
    global CUR_VISION_STATE
    global ws_pts
    show_log('  @Clicked Button_Setroi')
    ws_pts = []
    SET_CUR_VISION_STATE(VISION_STATE.SET_VISION_ROI)

def click_button_4():
    #global CUR_VISION_STATE
    global rgb, depth
    global robot_point

    show_log('  @ClickedButton_4')
    input_path = '/home/keti_ai/data/200525/' + datetime.now().strftime(
        '%Y_%m_%d_%H_%M_%S_') + "test_input.png"
    depth_path = '/home/keti_ai/data/200525/' + datetime.now().strftime(
        '%Y_%m_%d_%H_%M_%S_') + "test_depth.png"

    cv2.imwrite(input_path, rgb[:, :, ::-1])
    cv2.imwrite(depth_path, depth)

    # for hand writing
    robot_pos = [0, 0, 0, 0, 0, 0]
    robot_pos[0] = ui_top.Entry1.get()
    robot_pos[1] = ui_top.Entry2.get()
    robot_pos[2] = ui_top.Entry3.get()
    robot_pos[3] = ui_top.Entry4.get()
    robot_pos[4] = ui_top.Entry5.get()
    robot_pos[5] = ui_top.Entry6.get()

    with open('/home/keti_ai/data/200525/robot_poits.txt', 'a') as f:
        f.write(str(robot_pos) + '\n')

    SET_CUR_VISION_STATE(VISION_STATE.COMPLETE_INIT)

def click_button_5():
    show_log('  @ClickedButton_5')

def click_button_6():
    '''
    TEMP MODULE :: FOR CALIBRATION

    send_msg = 'getrobotpose@'

    client_sock.sendall(send_msg.encode())

    data = client_sock.recv(1024)

    print(data);;;
    '''
    show_log('  @ClickedButton_6')

def button_saveroi():
    global ws_pts
    global CUR_VISION_STATE
    show_log('  @Clicked Button_Saveroi')

    with open("config_roi.json", "w") as json_file:
        json.dump(ws_pts, json_file)

    SET_CUR_VISION_STATE(VISION_STATE.COMPLETE_INIT)

def button_savebackground():
    global CUR_VISION_STATE
    show_log('  @Clicked Button_Savebackground')
    ## path prob.
    cv2.imwrite('data/bg_depth.png', depth)
    #cv2.imwrite('../data/bg_depth.png', depth)
    SET_CUR_VISION_STATE(VISION_STATE.REQUEST_RELOAD_DETECTOR_SET)

def button_reloadconfigs():
    global CUR_VISION_STATE
    show_log('  @Clicked Button_ReloadConfigs')

    SET_CUR_VISION_STATE(VISION_STATE.REQUEST_RELOAD_DETECTOR_SET)


def button_connections():
    ui_top.button_calibration_depth.configure(command=button_HandeyeCalibration)
    ui_top.button_calibration_pnp.configure(command=button_HandeyeCalibration_pnp)
    ui_top.button_Load_Calibration.configure(command=button_Calibration)
    ui_top.button_Save_image.configure(command=button_save_image)
    ui_top.button_Test_image.configure(command=button_test_image)

pick_count = 0
def mouse_callback(event):
    global pick_count
    global ws_pts
    global CUR_VISION_STATE
    global depth

    if CUR_VISION_STATE == VISION_STATE.SET_VISION_ROI:
        pick_count += 1
        print ("clicked at", event.x, event.y, pick_count)
        ws_pts.append((event.x, event.y))

    else:
        print("clicked at", event.x, event.y)
        xx = int(event.x)
        yy = int(event.y)

        if depth is not None and hasattr(depth, 'size') and depth.size > 0:
            show_log('clicked at  x: ' + str(xx) + ",  y: " + str(yy) + ",  depth: " + str(depth[yy, xx]))
        else:
            show_log('clicked at  x: ' + str(xx) + ",  y: " + str(yy))

def normal_callback(event=None):
    print("changed normal threshold")
    show_log('changed normal threshold')
    tmp_normal_depth_value = ui_top.normal_depth_threshold.get()
    gnormal_depth_threshold = int(tmp_normal_depth_value)

    tmp = 0

def show_log(log_msg):
    ui_top.listbox.insert(tk.END, log_msg + '    ['+str(datetime.datetime.now().strftime('%m-%d %H:%M:%S.%f') + ']'))
    ui_top.listbox.see(ui_top.listbox.size())

def SET_CUR_VISION_STATE(VISION_STATE):
    global CUR_VISION_STATE
    CUR_VISION_STATE = VISION_STATE
    print(get_vision_state())
    show_log('  CUR_VISION_STATE >> ' + get_vision_state())

if __name__=='__main__':


    ui = tk.Tk()
    ui_top = UIApp(ui, "Calibration")

    ### events

    ui_top.img_tab1_label.bind('<Button-1>', mouse_callback)
    ui_top.normal_depth_threshold.bind("<Return>", normal_callback)

    show_log('Main >> Program Started')

    sensor_thread = threading.Thread(target=sensor)
    sensor_thread.daemon = True
    sensor_thread.start()
    show_log('Main >> Sensor Started')

    button_connections()
    SET_CUR_VISION_STATE(VISION_STATE.COMPLETE_INIT)
    show_log('Main >> PROGRAM INIT COMPLETED')

    ui.mainloop()
