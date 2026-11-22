"""Acquisition and motor control on the PiPER-enabled LeRobot workstation."""

import time
from dataclasses import dataclass
from importlib import import_module
from pathlib import Path
from threading import Lock

import numpy as np


@dataclass
class ImageFrame:
    rgb: np.ndarray  # rectified uint8 H x W x 3 RGB
    exposure_ns: int  # exposure on the host's monotonic clock
    recorded_ns: int | None = None  # optional source timestamp for recorded data


@dataclass
class ArmState:
    joints_rad: list[float]
    T_base_flange: list  # row-major, meters, maps flange to base
    gripper_width_m: float
    timestamp_ns: int  # matched sample in the exposure clock


def make_lab_robot(settings):
    """Decode the robot options through the workstation's own LeRobot registry."""
    try:
        import draccus
        from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
        from lerobot.robots import (
            RobotConfig,
            bi_piper_follower,  # noqa: F401 - registers the workstation driver
            make_robot_from_config,
            piper_follower,  # noqa: F401
        )
    except ImportError as exc:
        raise RuntimeError(
            "Use the workstation Python environment containing draccus, OpenCV cameras "
            "and the PiPER-enabled LeRobot fork used by deploy/client.py."
        ) from exc
    options = dict(settings.get("robot_options", {}))
    options.update(type=settings["robot_type"], id=settings["robot_id"], cameras=settings["cameras"])
    return make_robot_from_config(draccus.decode(RobotConfig, options))


def driver_info(settings, *, sample=False):
    """List real feature names; optionally read the shape of a single raw observation."""
    robot = make_lab_robot(settings)
    result = {"motor_keys_in_driver_order": list(robot.action_features), "cameras": list(robot.cameras)}
    if sample:
        robot.connect()
        try:
            obs = robot.get_observation()
            result["observation_keys"] = list(obs)
            result["motor_readings"] = {key: float(obs[key]) for key in robot.action_features}
            result["images"] = {
                cam: {"shape": list(np.shape(obs[cam])), "dtype": str(np.asarray(obs[cam]).dtype)}
                for cam in robot.cameras
            }
        finally:
            robot.disconnect()
    return result


def _flange_fk(mode):
    if mode is None:
        return None
    if mode != "piper_urdf":
        raise ValueError("flange_fk must be null or piper_urdf")
    from codeworld.collision import RobotGeometry
    from codeworld.world import read

    from .scene import resolve_asset

    configs = Path(__file__).parent / "data/configs"
    spec = resolve_asset(read(configs / "piper_collision.json")["collision"]["robot_spec"], configs)
    spec["bases"] = {"local": np.eye(4)}
    model = RobotGeometry(spec)

    def forward(joints_rad):
        return model.frames("local", joints_rad, 0.0)[spec["ee_link"]]

    return forward


class LabAdapter:
    """Bridge get_observation/send_action on the lab's existing LeRobot robot."""

    def __init__(self, settings, *, robot=None, site=None):
        from .lerobot_actions import LeRobotActions, MotorLayout
        from .lerobot_observation import LeRobotObservation

        if site is None and settings.get("site_module"):
            site = import_module(settings["site_module"])
        self.robot = make_lab_robot(settings) if robot is None else robot
        layout = MotorLayout(self.robot, settings)
        fk = _flange_fk(settings.get("flange_fk"))
        self.observation = LeRobotObservation(self.robot, settings, site, layout=layout, flange_fk=fk)
        self._site_video = getattr(site, "read_video_frames", None)
        self._video_ready = False
        self.actions = LeRobotActions(
            self.robot, settings, layout=layout, plan_move=getattr(site, "plan_move", None),
            stop=getattr(site, "stop", None), grip_contact=getattr(site, "grip_contact", None),
            flange_fk=fk, observation_metadata=getattr(site, "observation_metadata", None),
        )
        self._lifecycle = Lock()
        self._connected = False
        self._closed = False

    def _connect(self):
        with self._lifecycle:
            if self._closed:
                raise RuntimeError("Lab adapter is closed")
            if not self._connected:
                try:
                    self.robot.connect()
                except BaseException:
                    if getattr(self.robot, "is_connected", False):
                        self.robot.disconnect()
                    raise
                self._connected = True

    def read_images(self, after_ns: int) -> dict[str, ImageFrame]:
        self._connect()
        return self.observation.read_images(after_ns)

    def read_state(self, at_ns: int) -> dict[str, ArmState]:
        return self.observation.read_state(at_ns)

    def read_video_frames(self) -> dict[str, np.ndarray]:
        """Peek at the connected cameras' RGB buffers without consuming an observation."""
        self._connect()
        camera_map = self.observation.camera_map
        deadline = time.monotonic() + self.observation.timeout_s
        while True:
            try:
                if self._site_video is not None:
                    frames = self._site_video(self.robot)
                else:
                    frames = {}
                    for source in camera_map:
                        camera = self.robot.cameras[source]
                        peek = getattr(camera, "read_latest", None)
                        if not callable(peek):
                            raise TypeError(
                                f"Camera {source} has no read_latest(). Provide "
                                "site.read_video_frames(robot) using the driver's frame buffers."
                            )
                        frame = peek()
                        if getattr(camera, "color_mode", None) == "bgr":
                            frame = frame[..., ::-1]
                        frames[source] = frame
                break
            except (RuntimeError, TimeoutError):
                # A camera connected without warmup may not have published its first frame.
                if self._video_ready or time.monotonic() >= deadline:
                    raise
                time.sleep(0.01)

        result = {}
        for source, view in camera_map.items():
            rgb = np.asarray(frames[source])
            if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
                raise ValueError(f"Video camera {source} must return an H×W×3 uint8 RGB image")
            result[view] = rgb.copy()
        self._video_ready = True
        return result

    def execute_action(self, command: dict, check_path=None) -> dict:
        if command["kind"] != "stop":
            self._connect()
        return self.actions.execute_action(command, check_path)

    def close(self):
        with self._lifecycle:
            if self._closed:
                return
            self._closed = True
            if self._connected:
                try:
                    self.actions.execute_action({"kind": "stop"})
                finally:
                    self.robot.disconnect()
                    self._connected = False
