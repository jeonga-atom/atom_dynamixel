#!/usr/bin/env python3

"""Simple keyboard teleoperation script for the KETI robot + gripper."""

from __future__ import annotations

import argparse
import json
import math
import os
import select
import signal
import sys
import termios
import tty
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

import numpy as np

from interface.gripper import Gripper

# Resolve repository-relative paths so the script works no matter the CWD.
BASE_DIR = Path(__file__).resolve().parent
SDK_DIR = BASE_DIR / "ROBOT" / "ROBOT_SDK"

if str(SDK_DIR) not in sys.path:
    sys.path.append(str(SDK_DIR))

from ketirobotsdk.sdk import (  # type: ignore  # pylint: disable=wrong-import-position
    Base,
    Indy7,
    M1013,
    RB10,
    TestDummy,
    UR10,
    Robot,
    setLibPath,
)


CONFIG_PATH = BASE_DIR / "config" / "ATOMvalues.json"
DEFAULT_LIB_PATH = SDK_DIR / "ketirobotsdk" / "librobotsdk.so"


ROBOT_TYPE_MAP = {
    "testdummy": TestDummy,
    "rb10": RB10,
    "ur10": UR10,
    "m1013": M1013,
    "indy7": Indy7,
}


def load_config(path: Path) -> Dict:
    """Load project configuration JSON if available."""
    if not path.exists():
        return {}

    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def resolve_robot_type(name: str) -> int:
    """Map a human-readable robot name to the SDK constant."""
    key = name.strip().lower()
    if key not in ROBOT_TYPE_MAP:
        raise ValueError(
            f"Unknown robot type '{name}'. "
            f"Supported: {', '.join(sorted(ROBOT_TYPE_MAP))}"
        )
    return ROBOT_TYPE_MAP[key]


def fetch_current_pose(robot: Robot, fallback: Iterable[float]) -> np.ndarray:
    """Query the robot for the current TCP pose, falling back to a default."""
    try:
        info = robot.RobotInfo()
        mat = np.array(list(info.Mat), dtype=float)
        if not np.allclose(mat, 0.0):
            return mat.reshape((4, 4))
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[warn] Failed to read live pose: {exc}")

    print("[warn] Falling back to configured initial pose.")
    return np.array(list(fallback), dtype=float).reshape((4, 4))


def rotation_matrix_z(angle_rad: float) -> np.ndarray:
    """Generate a rotation matrix about the local Z axis."""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def apply_translation(pose: np.ndarray, delta: Tuple[float, float, float]) -> np.ndarray:
    """Translate the pose in base coordinates."""
    result = pose.copy()
    result[0, 3] += delta[0]
    result[1, 3] += delta[1]
    result[2, 3] += delta[2]
    return result


def apply_wrist_rotation(pose: np.ndarray, angle_deg: float) -> np.ndarray:
    """Rotate the wrist about its local Z axis."""
    angle_rad = math.radians(angle_deg)
    delta = rotation_matrix_z(angle_rad)
    result = pose.copy()
    result[:3, :3] = result[:3, :3] @ delta
    return result


def read_key(fd: int, timeout: float = 0.1) -> Optional[str]:
    """Read a single character without blocking for longer than timeout."""
    ready, _, _ = select.select([fd], [], [], timeout)
    if not ready:
        return None
    return os.read(fd, 1).decode(errors="ignore")  # raw byte -> str


def prompt_float(
    message: str,
    current: float,
    fd: int,
    original_termios: Iterable[float],
) -> float:
    """Temporarily restore canonical mode to prompt the user for a float."""
    termios.tcsetattr(fd, termios.TCSADRAIN, original_termios)
    try:
        raw = input(f"{message} (current {current}): ").strip()
    finally:
        tty.setcbreak(fd)

    if not raw:
        return current

    try:
        return float(raw)
    except ValueError:
        print("[warn] Invalid number, keeping previous value.")
        return current


def prompt_int(
    message: str,
    current: int,
    fd: int,
    original_termios: Iterable[float],
) -> int:
    """Prompt the user for an integer (used for the gripper Move target)."""
    new_value = prompt_float(message, float(current), fd, original_termios)
    return int(round(new_value))


def print_help(move_step: float, rot_step: float, grip_target: int) -> None:
    """Display the key bindings and current parameters."""
    print("\nKeyboard controls (Ctrl+C to exit):")
    print("  w/s : ±X translation (base frame)")
    print("  a/d : ±Y translation (base frame)")
    print("  o/k : ±Z translation (base frame)")
    print("  q/e : wrist CCW/CW about local Z")
    print("  [   : set translation step (meters)")
    print("  ]   : set wrist rotation step (deg)")
    print("  '   : send gripper Move command")
    print("  h   : show this help again")
    print(
        f"  current steps -> move: {move_step:.4f} m, rotate: {rot_step:.2f}°, "
        f"gripper target: {grip_target}"
    )


def connect_gripper(config: Dict) -> Optional[Gripper]:
    """Create a Gripper instance if connection info is present."""
    endpoint = config.get("gripper_connect")
    if not endpoint:
        print("[warn] No gripper_connect entry found in ATOMvalues.json.")
        return None

    host, port = endpoint
    try:
        return Gripper(host, int(port))
    except OSError as exc:
        print(f"[warn] Failed to connect to gripper {endpoint}: {exc}")
        return None


def main(argv: Optional[Iterable[str]] = None) -> int:
    config = load_config(CONFIG_PATH)

    parser = argparse.ArgumentParser(
        description="Keyboard teleoperation using Robot.movel and Gripper.Move.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        "--robot-type",
        default=os.environ.get("KETI_ROBOT_TYPE", "M1013"),
        help="Robot model identifier (TestDummy, RB10, UR10, M1013, Indy7).",
    )
    parser.add_argument(
        "--robot-ip",
        default=os.environ.get("KETI_ROBOT_IP", "192.168.137.101"),
        help="Robot controller IP address.",
    )
    parser.add_argument(
        "--robot-port",
        type=int,
        default=int(os.environ.get("KETI_ROBOT_PORT", "12345")),
        help="Robot controller port.",
    )
    parser.add_argument(
        "--velocity",
        type=float,
        default=float(config.get("normal_vel", 5.0)),
        help="Robot velocity percentage to apply before each movel.",
    )
    parser.add_argument(
        "--move-step",
        type=float,
        default=0.010,
        help="Default Cartesian translation step in meters.",
    )
    parser.add_argument(
        "--rot-step",
        type=float,
        default=5.0,
        help="Default wrist rotation step in degrees.",
    )
    parser.add_argument(
        "--gripper-target",
        type=int,
        default=int(config.get("gripper_ready_value", 1200)),
        help="Initial gripper target value for Move commands.",
    )
    parser.add_argument(
        "--lib",
        type=Path,
        default=DEFAULT_LIB_PATH,
        help="Path to librobotsdk.so.",
    )

    args = parser.parse_args(argv)

    if not args.lib.exists():
        parser.error(f"librobotsdk.so not found at {args.lib}")

    setLibPath(str(args.lib))

    try:
        robot_type = resolve_robot_type(args.robot_type)
    except ValueError as exc:
        parser.error(str(exc))

    robot = Robot()
    robot.SetRobotConf(robot_type, args.robot_ip, int(args.robot_port))
    if not robot.RobotConnect():
        parser.error(
            f"Failed to connect to robot {args.robot_ip}:{args.robot_port} "
            f"({args.robot_type})"
        )

    robot.SetVelocity(args.velocity)

    gripper = connect_gripper(config)

    init_pose = config.get("init_pose", config.get("init_pose"))
    if not init_pose:
        parser.error("ATOMvalues.json must contain init_pose or identity_matrix.")

    current_pose = fetch_current_pose(robot, init_pose)

    move_step = float(args.move_step)
    rot_step = float(args.rot_step)
    gripper_target = int(args.gripper_target)

    fd = sys.stdin.fileno()
    original_termios = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def handle_exit(_signum=None, _frame=None) -> None:
        termios.tcsetattr(fd, termios.TCSADRAIN, original_termios)
        print("\nStopping robot...")
        try:
            robot.Stop()
            robot.WaitMove()
        except Exception:  # pylint: disable=broad-except
            pass
        robot.RobotDisconnect()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_exit)

    print_help(move_step, rot_step, gripper_target)

    try:
        while True:
            key = read_key(fd)
            if key is None:
                continue

            key = key.lower()

            try:
                if key == "w":
                    target = apply_translation(current_pose, (move_step, 0.0, 0.0))
                    robot.movel(Base, target.flatten().tolist())
                elif key == "s":
                    target = apply_translation(current_pose, (-move_step, 0.0, 0.0))
                    robot.movel(Base, target.flatten().tolist())
                elif key == "d":
                    target = apply_translation(current_pose, (0.0, move_step, 0.0))
                    robot.movel(Base, target.flatten().tolist())
                elif key == "a":
                    target = apply_translation(current_pose, (0.0, -move_step, 0.0))
                    robot.movel(Base, target.flatten().tolist())
                elif key == "o":
                    target = apply_translation(current_pose, (0.0, 0.0, move_step))
                    robot.movel(Base, target.flatten().tolist())
                elif key == "k":
                    target = apply_translation(current_pose, (0.0, 0.0, -move_step))
                    robot.movel(Base, target.flatten().tolist())
                elif key == "q":
                    target = apply_wrist_rotation(current_pose, rot_step)
                    robot.movel(Base, target.flatten().tolist())
                elif key == "e":
                    target = apply_wrist_rotation(current_pose, -rot_step)
                    robot.movel(Base, target.flatten().tolist())
                elif key == "[":
                    move_step = prompt_float(
                        "New translation step (meters)",
                        move_step,
                        fd,
                        original_termios,
                    )
                    print_help(move_step, rot_step, gripper_target)
                    continue
                elif key == "]":
                    rot_step = prompt_float(
                        "New wrist rotation step (degrees)",
                        rot_step,
                        fd,
                        original_termios,
                    )
                    print_help(move_step, rot_step, gripper_target)
                    continue
                elif key == "'":
                    gripper_target = prompt_int(
                        "New gripper Move target",
                        gripper_target,
                        fd,
                        original_termios,
                    )
                    if gripper is not None:
                        gripper.Move(gripper_target)
                        print(f"[gripper] Move -> {gripper_target}")
                    else:
                        print("[warn] Gripper not connected.")
                    print_help(move_step, rot_step, gripper_target)
                    continue
                elif key == "h":
                    print_help(move_step, rot_step, gripper_target)
                    continue
                else:
                    continue

                robot.WaitMove()
                current_pose = fetch_current_pose(robot, target.flatten().tolist())
                xyz = current_pose[:3, 3]
                print(
                    f"[movel] pose -> x:{xyz[0]:+.4f} "
                    f"y:{xyz[1]:+.4f} z:{xyz[2]:+.4f}"
                )
            except Exception as exc:  # pylint: disable=broad-except
                print(f"[error] Command failed: {exc}")
                current_pose = fetch_current_pose(robot, current_pose.flatten().tolist())
    finally:
        handle_exit()


if __name__ == "__main__":
    sys.exit(main())
