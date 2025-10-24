#!/usr/bin/env python3
# matrix를 받아오는 코드

from datetime import datetime
import json
import time
from pathlib import Path
from typing import List, Sequence, Union
from robot import RobotInfo  # 로봇 상태 수신 및 명령 송신 클래스

import sys
# sys.path.append("/home/atom/Workspaces/RJG/sdk_challenge/ROBOT/ROBOT_SDK/")
# from ketirobotsdk.sdk import *

# # -------------------- 로봇 연결 --------------------
# robot = Robot()
# robot.SetRobotConf(M1013, "192.168.137.101", 12345)
# ok = robot.RobotConnect()
# print("RobotConnect:", ok)
# if not ok or not robot.IsConnected():
#     print("NOT CONNECTED")
#     sys.exit(1)

# -------------------- 로봇 정보 수신 객체 --------------------
robot_info = RobotInfo(addr="192.168.137.101", port=12345)  # 인자 없이 생성 (RobotInfo는 모니터링 스레드 실행용)
# robot_info.addr = "192.168.137.101"
# robot_info.port = 12345
robot_info_OK = robot_info.connected
print("RobotConnect:", robot_info_OK)
# if not robot_info_OK:
#     print("NOT CONNECTED")
#     sys.exit(1)

# 파일명: OS 호환 위해 ':' 대신 '-' 사용
now_time = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
DEFAULT_OUTPUT_PATH = (
    Path(__file__).parent / "pose_json" / "mat" / f"pose_matrix_{now_time}.json"
)

# -------------------- 함수 정의 --------------------
def now_robot_pose_matrix() -> List[float]:
    """현재 수신된 로봇 자세 행렬을 1차원 리스트 형태로 반환한다."""
    return list(robot_info.robot_mat)


def now_robot_pose_joint() -> List[float]:
    """현재 수신된 로봇 자세 조인트를 1차원 리스트 형태로 반환한다."""
    return list(robot_info.robot_jnt)


def now_robot_pose_state() -> List[float]:
    """현재 수신된 로봇 자세 상태를 1차원 리스트 형태로 반환한다."""
    return list(robot_info.robot_state)


def wait_for_robot_pose(data_value: str = "matrix", timeout: float = 5.0, poll_interval: float = 0.05) -> List[float]:
    """
    로봇 제어기에서 유효한 자세 행렬 데이터를 받을 때까지 대기한다.
    timeout 초 안에 데이터를 받지 못하면 TimeoutError 발생.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if robot_info.connected:
            if data_value == "matrix":
                pose = now_robot_pose_matrix()
            elif data_value == "joint":
                pose = now_robot_pose_joint()
            elif data_value == "state":
                pose = now_robot_pose_state()
            else:
                pose = now_robot_pose_matrix()
            
            if pose and any(pose):
                return pose
        time.sleep(poll_interval)
    raise TimeoutError("Timed out while waiting for pose matrix from the robot controller.")


def reshape_pose_matrix(pose: Sequence[float]) -> List[List[float]]:
    """16개의 값을 4x4 행렬 형태의 중첩 리스트로 변환한다."""
    if len(pose) != 16:
        raise ValueError(f"Expected 16 elements for pose matrix, received {len(pose)}")
    return [list(pose[i : i + 4]) for i in range(0, 16, 4)]


def load_existing_records(path: Path) -> List[dict]:
    """이미 존재하는 JSON 파일을 로드하여 리스트 형태로 반환한다."""
    if not path.exists() or path.stat().st_size == 0:
        return []
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return []

    if isinstance(data, list):
        return data
    if isinstance(data, dict):
        return [data]
    return []


def save_pose_matrix(
    output_path: Union[str, Path] = DEFAULT_OUTPUT_PATH,
    timeout: float = 5.0,
    append: bool = True,
) -> dict:
    """
    현재 로봇 자세 행렬을 JSON 파일로 저장한다.
    append 가 True이면 기존 기록에 현재 데이터를 추가하고,
    False이면 현재 데이터만 저장한다.
    """
    pose = wait_for_robot_pose(timeout=timeout)
    payload = {
        "timestamp": datetime.now().isoformat(),
        "pose_matrix": pose,
    }

    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)

    if append:
        records = load_existing_records(path)
        records.append(payload)
        path.write_text(json.dumps(records, indent=2), encoding="utf-8")
    else:
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    return payload


# -------------------- 메인 루프 --------------------
if __name__ == "__main__":
    try:
        result = save_pose_matrix()
        print(f"Pose matrix saved to {DEFAULT_OUTPUT_PATH}")
        print(json.dumps(result, indent=2))
    except TimeoutError as exc:
        print(str(exc))
    finally:
        # 안전하게 연결 종료 (세그폴트 방지)
        try:
            robot_info.close()
        except Exception as e:
            print("Disconnect error:", e)
