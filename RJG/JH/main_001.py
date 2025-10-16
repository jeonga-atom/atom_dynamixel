#!/usr/bin/env python3

# from interface.vision import *  # 비전 서버 통신 함수들을 사용하기 위해 전체 임포트
from interface.gripper import *  # 소켓 기반 그리퍼 제어 함수를 사용하기 위해 전체 임포트
from interface.robot import *  # 로봇 상태 수신 및 명령 송신 클래스를 사용하기 위해 전체 임포트

import sys
sys.path.append("ROBOT/ROBOT_SDK")
# from ketirobotsdk.sdk import *

import copy  # 깊은 복사를 통해 원본 포즈 배열을 보존하기 위해 사용
import math  # 각도 연산과 삼각함수 계산을 위해 사용
import time  # 일정 시간 대기 및 타이밍 제어를 위해 사용(디버깅용)
import numpy as np  # 행렬 연산으로 TCP 회전 행렬을 조작하기 위해 사용

# JSON 파일 데이터 로드를 위한 라이브러리
import json
from pathlib import Path
from typing import List

# JSON 파일에서 반복 사용 포즈/파라미터 로드
DATA_PATH = Path(__file__).resolve().parent / "config" / "ATOMvalues.json"
with DATA_PATH.open("r", encoding="utf-8") as data:
    _DATA = json.load(data)

'''
init_joint          = _DATA["init_joint"]
init_pose           = _DATA["init_pose"]

check_angle_pose    = _DATA["check_angle_pose"]

chuck_up_pose       = _DATA["chuck_up_pose"]
chuck_pose          = _DATA["chuck_pose"]

remove_z            = _DATA["remove_z"]

place_pose          = _DATA["place_pose"]
place_bin_up_pose   = _DATA["place_bin_up_pose"]

place_z             = _DATA["place_z"]
place_obj_offset_x  = _DATA["place_obj_offset_x"]
place_obj_offset_y  = _DATA["place_obj_offset_y"]
'''

# 추가 변수
cls_Targets     = _DATA["cls_Targets"]
cls_Guides      = _DATA["cls_Guides"]

target_camera_pose  = _DATA["target_camera_pose"] # 기존 main.py에서 init_pose와 같음

normal_vel          = _DATA["normal_vel"]
slow_vel            = _DATA["slow_vel"]

forces              = _DATA["forces"]
axis_directions     = _DATA["axis_directions"]

gripper_ready_value     = _DATA["gripper_ready_value"]
gripper_grip0_value     = _DATA["gripper_grip0_value"]
gripper_grip2_value     = _DATA["gripper_grip2_value"]
gripper_grip4_value     = _DATA["gripper_grip4_value"]
gripper_release_value   = _DATA["gripper_release_value"]

target_ready_height     = _DATA["target_ready_height"]
target_go2D_slow_height = _DATA["target_go2D_slow_height"]
target_grip_height      = _DATA["target_grip_height"]
guide_move_height       = _DATA["guide_move_height"]
guide_place_slow_height = _DATA["guide_place_slow_height"]
guide_release_height    = _DATA["guide_release_height"]

Moving_pose         = _DATA["identity_matrix"]          # 전역 변수, 단위 행렬로 초기화

# vision      = Vision(_DATA["vision_connect"][0],           _DATA["vision_connect"][1])
# gripper     = Gripper(_DATA["gripper_connect"][0],         _DATA["gripper_connect"][1])
# robot_info  = RobotInfo(_DATA["robot_info_connect"][0],    _DATA["robot_info_connect"][1])
# robot_cmd   = RobotCmd(_DATA["robot_cmd_connect"][0],      _DATA["robot_cmd_connect"][1])

Init_check      = False                                 # 전역 변수, Gripper 초기화를 위한 변수
Processing_now  = True

result_json_dir = _DATA["result_json_dir"]
guide_json_dir = _DATA["guide_json_dir"]
# target_result   = _DATA["test_result"]

guide_data      = [[], [], []]
tmp_target_data = []
target_data     = []
target_cls      = -1                                    # 전역 변수, -1로 초기화
guide_cls       = -1                                    # 전역 변수, -1로 초기화

code_stop = f"""robot.Stop()"""                         # 코드 종료 시(Ctrl+C) 매니퓰레이터 정지 명령 실행

def T_load_detections(json_path):
    """JSON 파일에서 작업물 감지 결과 배열을 읽어온다."""
    load_json_file = []
    while len(load_json_file) != 4 or (None in load_json_file):
        try:
            with open(json_path, "r", encoding="utf-8") as file:
                load_json_file = json.load(file)
        except FileNotFoundError:
            print("Failed to find json file. Retry to read the file.")
            pass
        except Exception as e:
            print(e)
            pass
        finally:
            print(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()), f"Target data: {load_json_file}")
            pass
    return load_json_file

def G_load_detections(json_path):
    """JSON 파일에서 가이드 감지 결과 배열을 읽어온다."""
    load_json_file = [[], [], []]
    while len(load_json_file) != 3 or (not all(len(data) == 4 for data in load_json_file)):
        try:
            with open(json_path, "r", encoding="utf-8") as file:
                load_json_file = json.load(file)
        except FileNotFoundError:
            print("Failed to find json file. Retry to read the file.")
            pass
        except Exception as e:
            print(e)
            pass
        finally:
            print(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()), 
                  f"Guide data: \n{load_json_file[0]}\n{load_json_file[1]}\n{load_json_file[2]}")
            pass
    return load_json_file

def make_Matrix(x_center: float, y_center: float, z_height: float, angle_deg: float = 0.0) -> List[float]:
    """[x_center, y_center, z_height, angle(도)] 정보를 XY 평면의 4x4 포즈 행렬로 변환한다.

    Args:
        x_center (float): 물체 중심의 X 좌표 (m).
        y_center (float): 물체 중심의 Y 좌표 (m).
        z_height (float): 필요 시 TCP 높이 (m).
        angle_deg (float): Z축 기준 회전 각도(도 단위, 기본값: 0.0도).

    Returns:
        list[float]: 4x4 행렬을 16개 원소 리스트로 평탄화한 값.
    """
    angle = np.deg2rad(angle_deg)  # NumPy를 이용해 도→라디안 변환
    c, s = np.cos(angle), np.sin(angle)  # cos, sin 값 계산

    rotation = np.array(
        [
            [c,     -s,     0.0],
            [s,     c,      0.0],
            [0.0,   0.0,    1.0],
        ],
        dtype=float
    )
    translation = np.array([x_center, y_center, z_height], dtype=float)  # 위치 벡터

    pose = np.eye(4)  # 4x4 단위 행렬 생성
    pose[:3, :3] = rotation  # 회전 부분 채우기
    pose[:3, 3] = translation  # 위치 부분 채우기

    result_Matrix = pose.flatten().tolist()  # 평탄화된 리스트로 반환
    print(f"Moving Matrix: \n{pose}\nRaw data: \n{result_Matrix}")
    return result_Matrix

def Now_robot_pose_Matrix():
    return robot_info.robot_mat

def offset_Matrix(Now_Mat, Destination_Mat):
    np_Now  = np.array(Now_Mat)
    np_Dest = np.array(Destination_Mat)

    np_Now_R    = np_Now[:3, :3]
    np_Dest_R   = np_Dest[:3, :3]

    R_MatMul = np_Now_R @ np_Dest_R
    np_Dest[:3, :3] = R_MatMul
    return np_Dest.flatten().tolist()

def target_camera():
    print(f"\n\nTake a picture.")
    global Init_check
    cmd_data = f"""
    robot.SetVelocity({normal_vel})
    robot.ControlBoxDigitalOut(2)
    robot.movel({target_camera_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)
    # if Init_check:
        # gripper.Release()
    # else:
    #     gripper.Init()
    #     Init_check = True
    pass

def target_ready(_target_data):
    print(f"\n\nReady to go to target.")
    global Moving_pose, target_cls
    print(_target_data)
    x, y, angle, target_cls = _target_data  # [x, y, angle_deg, class]
    Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=target_ready_height, angle_deg=angle)
    cmd_data = f"""
    robot.SetVelocity({normal_vel})
    robot.movel({Moving_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)
    # gripper.Move(gripper_ready_value)
    pass

def target_go(_target_data):
    print(f"\n\nGo to target.")
    global Moving_pose
    x, y, angle, _ = _target_data  # [x, y, angle_deg, class]
    Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=target_go2D_slow_height, angle_deg=angle)
    cmd_data = f"""
    robot.SetVelocity({normal_vel})
    robot.movel({Moving_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)

    print(f"\n\nGoing to target slowly.")
    Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=target_grip_height, angle_deg=angle)
    cmd_data = f"""
    robot.SetVelocity({slow_vel})
    robot.movel({Moving_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)
    pass

def target_grip():
    print(f"\n\nGrip target object.")
    global target_cls
    print(f"Class number: {target_cls}")
    if target_cls == 0:
        print(f"Gripping value: {gripper_grip0_value}")
        # gripper.Move(gripper_grip0_value)
        pass
    elif target_cls == 2:
        print(f"Gripping value: {gripper_grip2_value}")
        # gripper.Move(gripper_grip2_value)
        pass
    elif target_cls == 4:
        print(f"Gripping value: {gripper_grip4_value}")
        # gripper.Move(gripper_grip4_value)
        pass
    else:
        pass
    # # gripper.Grip()
    pass

def target_pickup(_target_data):
    print(f"\n\nPick the object and go up within Z axis.")
    global Moving_pose
    x, y, angle, _ = _target_data  # [x, y, angle_deg, class]
    # Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=target_go2D_slow_height, angle_deg=angle)
    # cmd_data = f"""
    # robot.SetVelocity({slow_vel})
    # robot.movel({Moving_pose})
    # robot.WaitMove()
    # """
    # # robot_cmd.send_cmd(code_str=cmd_data)

    Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=guide_move_height, angle_deg=angle)
    cmd_data = f"""
    robot.SetVelocity({normal_vel})
    robot.movel({Moving_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)
    pass

def guide_move(_guide_data):
    print(f"\n\nMove to guide within X-Y plane.")
    global Moving_pose, target_cls, guide_cls
    print(_guide_data[int(target_cls/2)])
    x, y, angle, guide_cls = _guide_data[int(target_cls/2)]  # [x, y, angle_deg, class]
    print(f"Target class: {target_cls}\nGuide class: {guide_cls}")
    Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=guide_move_height, angle_deg=angle)
    cmd_data = f"""
    robot.SetVelocity({normal_vel})
    robot.movel({Moving_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)
    pass

def guide_place(_guide_data):
    print(f"\n\nGo to guide for placing object.")
    global Moving_pose
    x, y, angle, _ = _guide_data[int(target_cls/2)]  # [x, y, angle_deg, class]
    Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=guide_place_slow_height, angle_deg=angle)
    cmd_data = f"""
    robot.SetVelocity({normal_vel})
    robot.movel({Moving_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)

    # 매니퓰레이터 각 축별 순응 제어(장애물 충돌 시의 강성) 및 힘/토크 제어 설정
    # 함수 자동 기본값 예시: robot.RobotComplianceCtrlOn(stpx = 1500, stpy = 1500, stpz = 1500, strx = 200, stry = 200, strz = 200) - sdk.py 참고.
    cmd_data = f"""
        robot.RobotComplianceCtrlOn()
        robot.RobotSetToolForce({forces}, {axis_directions})
    """
    # robot_cmd.send_cmd(code_str=cmd_data)

    print(f"\n\nGoing slowly to guide for placing object.")
    Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=guide_release_height, angle_deg=angle)
    cmd_data = f"""
    robot.SetVelocity({slow_vel})
    robot.movel({Moving_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)
    pass

def guide_release():
    print(f"\n\nRelease object.")
    print(f"Releasing value: {gripper_release_value}")
    # gripper.Move(gripper_release_value)
    # # gripper.Release()
    pass

def guide_comeback(_guide_data):
    print(f"\n\nGoing up to be ready for next step.")
    global Moving_pose
    cmd_data = f"""
        robot.RobotReleaseForce()
        robot.RobotComplianceCtrlOff()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)

    x, y, angle, _ = _guide_data[int(target_cls/2)]  # [x, y, angle_deg, class]
    Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=guide_move_height, angle_deg=angle)
    cmd_data = f"""
    robot.SetVelocity({normal_vel})
    robot.movel({Moving_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)
    pass

def debugging_delay(delay):
    time.sleep(delay)
    pass

if __name__ == "__main__":
    try:
        target_camera() # 가이드 촬영용 - 추후 3인칭 카메라로 변경 예정
        guide_data = G_load_detections(json_path=guide_json_dir)
        # 가이드 최초 1회 촬영 후 계속 동일 위치값 사용
        while any(guide_data[index][3] != cls_Guides[index] for index in range(3)) or any(None in data for data in guide_data):
            print("Guide objects data does not match the data expected.")
            guide_data = G_load_detections(json_path=guide_json_dir)
            debugging_delay(0.05)
        while Processing_now:
            target_camera() # 작업물 촬영용 - 그리퍼 카메라 사용
            tmp_target_data = T_load_detections(json_path=result_json_dir)
            # 촬영 마칠 때까지 대기 필요, 대기 과정 추가하기
            while tmp_target_data == target_data:
                print("Target object data(coordinate, angle, class) is same as previous one.")
                tmp_target_data = T_load_detections(json_path=result_json_dir)
                debugging_delay(0.05)
            if tmp_target_data[3] in cls_Targets:
                # 물체가 인식된 경우(여기에 매니퓰레이터 이후 동작 함수 넣기)
                print(tmp_target_data, target_data)
                target_data = tmp_target_data
                target_ready(_target_data=target_data)
                target_go(_target_data=target_data)
                target_grip()
                target_pickup(_target_data=target_data)
                # target → guide 전환
                guide_move(_guide_data=guide_data)
                guide_place(_guide_data=guide_data)
                guide_release()
                guide_comeback(_guide_data=guide_data)
                debugging_delay(10)
            else:
                # 물체가 인식되지 않은 경우
                print("Target object was not found.")
                print(tmp_target_data, target_data)
                print("Last detected target object class No.:", target_cls)
                debugging_delay(1)
    except KeyboardInterrupt:
        robot_cmd.send_cmd(code_stop)
        print("Keyboard Ctrl+C detected.")
        pass
    # except Exception as error:
    #     print("Error Exception: ", error)
    finally:
        pass