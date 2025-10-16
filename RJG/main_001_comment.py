#!/usr/bin/env python3

# 이 스크립트는 비전 감지 결과를 바탕으로 로봇과 그리퍼를 제어하는 전체 시퀀스를 테스트한다.

# from interface.vision import *  # 비전 서버 통신 함수들을 사용하기 위해 전체 임포트
from interface.gripper import *  # 소켓 기반 그리퍼 제어 함수를 사용하기 위해 전체 임포트
from interface.robot import *  # 로봇 상태 수신 및 명령 송신 클래스를 사용하기 위해 전체 임포트

import sys
sys.path.append("ROBOT/ROBOT_SDK")  # SDK 모듈 경로를 추가해 추후 import가 가능하도록 준비
# from ketirobotsdk.sdk import *

import copy  # 깊은 복사를 통해 원본 포즈 배열을 보존하기 위해 사용
import math  # 각도 연산과 삼각함수 계산을 위해 사용
import time  # 일정 시간 대기 및 타이밍 제어를 위해 사용(디버깅용)
import numpy as np  # 행렬 연산으로 TCP 회전 행렬을 조작하기 위해 사용

# JSON 파일 데이터 로드를 위한 라이브러리
import json
from pathlib import Path
from typing import Iterable, List

# 작업 반복에 필요한 파라미터가 담긴 JSON 경로 정의
DATA_PATH = Path(__file__).resolve().parent / "config" / "ATOMvalues.json"
with DATA_PATH.open("r", encoding="utf-8") as data:
    _DATA = json.load(data)  # JSON을 한 번 읽어 공유 파라미터 딕셔너리 생성

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

# 추출된 데이터 중 본 시퀀스에서 사용하는 값들을 변수로 풀어둠
cls_Targets     = set(_DATA["cls_Targets"])
cls_Guides      = _DATA["cls_Guides"]

target_camera_pose  = _DATA["target_camera_pose"]  # 기존 main.py에서 init_pose와 동일한 위치

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

Moving_pose         = _DATA["identity_matrix"]  # 전역 이동 포즈: 기본값은 단위 행렬

# vision      = Vision(_DATA["vision_connect"][0],           _DATA["vision_connect"][1])
# gripper     = Gripper(_DATA["gripper_connect"][0],         _DATA["gripper_connect"][1])
# robot_info  = RobotInfo(_DATA["robot_info_connect"][0],    _DATA["robot_info_connect"][1])
# robot_cmd   = RobotCmd(_DATA["robot_cmd_connect"][0],      _DATA["robot_cmd_connect"][1])

Init_check      = False  # 그리퍼 초기화가 끝났는지 추적
Processing_now  = True   # 메인 루프 진행 여부 플래그

result_json_dir = _DATA["result_json_dir"]  # 타겟 감지 결과 JSON 경로
guide_json_dir = _DATA["guide_json_dir"]    # 가이드 감지 결과 JSON 경로
# target_result   = _DATA["test_result"]

guide_data      = [[], [], []]  # 가이드 감지 결과 버퍼 초기화
tmp_target_data = []            # 최신 타겟 결과 버퍼
target_data     = []            # 이전에 처리했던 타겟 결과 기록
target_cls      = -1            # 타겟 클래스 번호 (초기값 -1)
guide_cls       = -1            # 가이드 클래스 번호 (초기값 -1)

code_stop = f"""robot.Stop()"""  # 예외 발생 시 매니퓰레이터를 즉시 정지시키기 위한 명령 문자열

# 반복 사용되는 모듈 함수 바인딩(속도 최적화)
_format_timestamp = time.strftime
_current_localtime = time.localtime
_sleep = time.sleep


def _timestamp() -> str:
    """현재 시각을 포맷팅된 문자열로 반환한다."""
    return _format_timestamp("%Y-%m-%d %H:%M:%S", _current_localtime())


def _has_none(values: Iterable) -> bool:
    """시퀀스에 None이 포함되어 있는지 확인한다."""
    return any(item is None for item in values)


def _guide_index(class_id: int) -> int:
    """타겟 클래스에 해당하는 가이드 인덱스를 계산한다."""
    return max(class_id // 2, 0)


def T_load_detections(json_path):
    """JSON 파일에서 작업물 감지 결과 배열을 읽어온다."""
    load_json_file = []  # 감지 결과 버퍼 초기화
    while True:
        try:
            with open(json_path, "r", encoding="utf-8") as file:
                load_json_file = json.load(file)  # 최신 타겟 감지 데이터를 읽음
        except FileNotFoundError:
            print("Failed to find json file. Retry to read the file.")  # 파일이 없으면 재시도
        except Exception as e:
            print(e)  # 기타 예외도 로그만 남기고 반복 지속
        finally:
            # 현재 시각과 함께 읽은 결과를 출력해 로깅
            print(_timestamp(), f"Target data: {load_json_file}")
        if len(load_json_file) == 4 and not _has_none(load_json_file):
            return load_json_file
        _sleep(0.01)  # 과도한 파일 I/O 요청을 막기 위해 짧은 대기


def G_load_detections(json_path):
    """JSON 파일에서 가이드 감지 결과 배열을 읽어온다."""
    load_json_file = [[], [], []]  # 가이드 감지 결과 버퍼 초기화
    while True:
        try:
            with open(json_path, "r", encoding="utf-8") as file:
                load_json_file = json.load(file)  # 최신 가이드 감지 데이터 로드
        except FileNotFoundError:
            print("Failed to find json file. Retry to read the file.")
        except Exception as e:
            print(e)
        finally:
            # 각 가이드 결과를 줄바꿈해 출력하여 데이터 상태를 눈으로 확인
            print(_timestamp(),
                  f"Guide data: \n{load_json_file[0]}\n{load_json_file[1]}\n{load_json_file[2]}")
        if (len(load_json_file) == 3 and
                all(len(data) == 4 and not _has_none(data) for data in load_json_file)):
            return load_json_file
        _sleep(0.01)  # 반복 중 시스템 부하를 완화하기 위한 짧은 대기


def make_Matrix(x_center: float, y_center: float, z_height: float, angle_deg: float = 0.0) -> List[float]:
    """[x_center, y_center, z_height, angle(도)] 정보를 XY 평면의 4x4 포즈 행렬로 변환한다."""
    angle = math.radians(angle_deg)  # 도→라디안 변환(표준 라이브러리 사용으로 오버헤드 감소)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    pose = [
        [cos_a, -sin_a, 0.0, x_center],
        [sin_a,  cos_a, 0.0, y_center],
        [0.0,    0.0,   1.0, z_height],
        [0.0,    0.0,   0.0, 1.0],
    ]

    result_Matrix = [value for row in pose for value in row]  # 평탄화된 리스트로 반환
    print(f"Moving Matrix: \n{np.array(pose)}\nRaw data: \n{result_Matrix}")
    return result_Matrix


def Now_robot_pose_Matrix():
    """현재 로봇 TCP 포즈 행렬을 반환한다."""
    return robot_info.robot_mat


def offset_Matrix(Now_Mat, Destination_Mat):
    """현재 포즈 대비 목적지 포즈를 보정한 행렬을 계산한다."""
    np_Now = np.array(Now_Mat)
    np_Dest = np.array(Destination_Mat)

    np_Now_R = np_Now[:3, :3]  # 현재 회전 성분 추출
    np_Dest_R = np_Dest[:3, :3]  # 목표 회전 성분 추출

    R_MatMul = np_Now_R @ np_Dest_R  # 회전 행렬을 곱해 누적 회전을 계산
    np_Dest[:3, :3] = R_MatMul  # 계산된 회전으로 목적지 보정
    return np_Dest.flatten().tolist()


def target_camera():
    """타겟 혹은 가이드 촬영을 위해 카메라 위치로 이동한다."""
    print(f"\n\nTake a picture.")
    global Init_check
    cmd_data = f"""
    robot.SetVelocity({normal_vel})
    robot.ControlBoxDigitalOut(2)
    robot.movel({target_camera_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)  # 실제 환경에서는 여기서 촬영 위치로 이동 및 카메라 트리거
    # if Init_check:
    #     gripper.Release()
    # else:
    #     gripper.Init()
    #     Init_check = True
    pass  # 현재는 하드웨어 호출을 생략하여 시퀀스 구조만 유지


def target_ready(_target_data):
    """타겟 상부로 이동해 준비 자세를 잡는다."""
    print(f"\n\nReady to go to target.")
    global Moving_pose, target_cls
    print(_target_data)
    x, y, angle, target_cls = _target_data  # [x, y, angle_deg, class]
    Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=target_ready_height, angle_deg=angle)  # 높은 위치에서 접근 준비
    cmd_data = f"""
    robot.SetVelocity({normal_vel})
    robot.movel({Moving_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)
    # gripper.Move(gripper_ready_value)  # 접근 전에 그리퍼 폭을 준비 자세로 맞춤
    pass


def target_go(_target_data):
    """타겟을 향해 이동하고 집기 높이까지 천천히 접근한다."""
    print(f"\n\nGo to target.")
    global Moving_pose
    x, y, angle, _ = _target_data  # [x, y, angle_deg, class]
    Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=target_go2D_slow_height, angle_deg=angle)  # XY 평면으로 내려옴
    cmd_data = f"""
    robot.SetVelocity({normal_vel})
    robot.movel({Moving_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)

    print(f"\n\nGoing to target slowly.")
    Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=target_grip_height, angle_deg=angle)  # 집기 높이까지 천천히 접근
    cmd_data = f"""
    robot.SetVelocity({slow_vel})
    robot.movel({Moving_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)
    pass


def target_grip():
    """타겟 클래스에 따라 그리퍼를 조절해 파트를 집는다."""
    print(f"\n\nGrip target object.")
    global target_cls
    print(f"Class number: {target_cls}")
    if target_cls == 0:
        print(f"Gripping value: {gripper_grip0_value}")
        # gripper.Move(gripper_grip0_value)  # 0번 클래스용 폭으로 조절
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
    """타겟을 잡은 뒤 안전 높이로 복귀한다."""
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

    Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=guide_move_height, angle_deg=angle)  # 가이드로 이동하기 전 안전 높이 확보
    cmd_data = f"""
    robot.SetVelocity({normal_vel})
    robot.movel({Moving_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)
    pass


def guide_move(_guide_data):
    """타겟 클래스에 대응하는 가이드 상부로 이동한다."""
    print(f"\n\nMove to guide within X-Y plane.")
    global Moving_pose, target_cls, guide_cls
    guide_entry = _guide_data[_guide_index(target_cls)]
    print(guide_entry)
    x, y, angle, guide_cls = guide_entry  # [x, y, angle_deg, class]
    print(f"Target class: {target_cls}\nGuide class: {guide_cls}")
    Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=guide_move_height, angle_deg=angle)  # 가이드 상공으로 이동
    cmd_data = f"""
    robot.SetVelocity({normal_vel})
    robot.movel({Moving_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)
    pass


def guide_place(_guide_data):
    """가이드에 접근해 순응 제어를 설정한 후 천천히 내려 놓는다."""
    print(f"\n\nGo to guide for placing object.")
    global Moving_pose
    x, y, angle, _ = _guide_data[_guide_index(target_cls)]  # [x, y, angle_deg, class]
    Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=guide_place_slow_height, angle_deg=angle)  # 먼저 중간 높이까지 이동
    cmd_data = f"""
    robot.SetVelocity({normal_vel})
    robot.movel({Moving_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)

    # 매니퓰레이터 각 축별 순응 제어(장애물 충돌 시의 강성) 및 힘/토크 제어 설정 예시
    cmd_data = f"""
        robot.RobotComplianceCtrlOn()
        robot.RobotSetToolForce({forces}, {axis_directions})
    """
    # robot_cmd.send_cmd(code_str=cmd_data)

    print(f"\n\nGoing slowly to guide for placing object.")
    Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=guide_release_height, angle_deg=angle)  # 최종 내려놓기 높이로 접근
    cmd_data = f"""
    robot.SetVelocity({slow_vel})
    robot.movel({Moving_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)
    pass


def guide_release():
    """가이드 위치에서 그리퍼를 열어 물체를 내려놓는다."""
    print(f"\n\nRelease object.")
    print(f"Releasing value: {gripper_release_value}")
    # gripper.Move(gripper_release_value)
    # # gripper.Release()
    pass


def guide_comeback(_guide_data):
    """가이드에서 물체를 놓은 뒤 안전 높이로 복귀한다."""
    print(f"\n\nGoing up to be ready for next step.")
    global Moving_pose
    cmd_data = f"""
        robot.RobotReleaseForce()
        robot.RobotComplianceCtrlOff()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)  # 힘 제어 해제 및 순응 제어 종료

    x, y, angle, _ = _guide_data[_guide_index(target_cls)]  # [x, y, angle_deg, class]
    Moving_pose = make_Matrix(x_center=x, y_center=y, z_height=guide_move_height, angle_deg=angle)  # 가이드 상부 안전 높이로 이동
    cmd_data = f"""
    robot.SetVelocity({normal_vel})
    robot.movel({Moving_pose})
    robot.WaitMove()
    """
    # robot_cmd.send_cmd(code_str=cmd_data)
    pass


def debugging_delay(delay):
    """디버깅 시나리오에서 간단한 지연을 추가한다."""
    _sleep(delay)
    pass


if __name__ == "__main__":
    try:
        target_camera()  # 가이드 촬영용 초기 촬영 수행 (향후 3인칭 카메라로 대체 예정)
        guide_data = G_load_detections(json_path=guide_json_dir)  # 가이드 감지 결과를 확보
        # 가이드 최초 1회 촬영 후 계속 동일 위치값 사용
        while any(guide_data[index][3] != cls_Guides[index] for index in range(3)) or any(_has_none(data) for data in guide_data):
            print("Guide objects data does not match the data expected.")  # 감지 결과가 기대와 다르면 재촬영
            guide_data = G_load_detections(json_path=guide_json_dir)
            debugging_delay(0.05)  # 재시도 전 짧은 대기
        while Processing_now:
            target_camera()  # 작업물 촬영용 (그리퍼 카메라)
            tmp_target_data = T_load_detections(json_path=result_json_dir)  # 최신 타겟 데이터를 받아옴
            # 촬영 마칠 때까지 대기 필요, 대기 과정 추가하기
            while tmp_target_data == target_data:
                print("Target object data(coordinate, angle, class) is same as previous one.")  # 이전 데이터와 동일하면 재시도
                tmp_target_data = T_load_detections(json_path=result_json_dir)
                debugging_delay(0.05)  # 인식 결과가 바뀔 때까지 잠시 대기
            if tmp_target_data[3] in cls_Targets:
                # 물체가 인식된 경우(여기에 매니퓰레이터 이후 동작 함수 넣기)
                print(tmp_target_data, target_data)
                target_data = tmp_target_data  # 새 결과를 이전 결과로 업데이트하여 중복 수행 방지
                target_ready(_target_data=target_data)   # 타겟 상부로 이동
                target_go(_target_data=target_data)      # 타겟 접근 및 집기 위치 이동
                target_grip()                            # 타겟 집기 수행
                target_pickup(_target_data=target_data)  # 안전 높이로 복귀
                # target → guide 전환
                guide_move(_guide_data=guide_data)       # 대응 가이드 상부로 이동
                guide_place(_guide_data=guide_data)      # 순응 제어 설정 후 가이드로 접근
                guide_release()                          # 물체 내려놓기
                guide_comeback(_guide_data=guide_data)   # 가이드 상부 안전 높이 복귀
                debugging_delay(10)  # 다음 사이클 전 충분한 대기
            else:
                # 물체가 인식되지 않은 경우
                print("Target object was not found.")
                print(tmp_target_data, target_data)
                print("Last detected target object class No.:", target_cls)
                debugging_delay(1)  # 감지가 다시 이뤄질 때까지 대기
    except KeyboardInterrupt:
        robot_cmd.send_cmd(code_stop)  # 사용자가 Ctrl+C를 입력하면 즉시 정지 명령을 전송
        print("Keyboard Ctrl+C detected.")
        pass
    # except Exception as error:
    #     print("Error Exception: ", error)
    finally:
        pass
