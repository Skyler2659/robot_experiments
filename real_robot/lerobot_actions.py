"""PiPER motor feedback and absolute motor targets on the lab's LeRobot client."""

import threading
import time
from collections.abc import Mapping

import numpy as np
from codeworld.world import rigid
from scipy.spatial.transform import Rotation

from .lerobot_observation import _stamp


def scale_vector(settings, name):
    value = np.asarray(settings.get(name), dtype=float)
    if value.shape != (7,) or not np.isfinite(value).all() or (value <= 0).any():
        raise ValueError(f"driver.{name} needs 7 positive SI-per-driver-unit factors")
    return value


class MotorLayout:
    """Six URDF-order joints followed by total jaw width, named explicitly per arm."""

    def __init__(self, robot, settings):
        self.features = tuple(robot.action_features)
        self.keys = settings.get("motor_keys")
        if not isinstance(self.keys, dict) or not self.keys or set(self.keys) - {"left", "right"}:
            raise ValueError(f"Fill driver.motor_keys from robot.action_features: {self.features}")
        assigned = []
        for arm, keys in self.keys.items():
            if not isinstance(keys, (tuple, list)) or len(keys) != 7:
                raise ValueError(
                    f"driver.motor_keys.{arm} needs six joint keys and one gripper key"
                )
            assigned.extend(keys)
        if len(assigned) != len(set(assigned)) or set(assigned) != set(self.features):
            raise ValueError(f"Map every action feature exactly once: {self.features}")
        self.state_scale = scale_vector(settings, "state_scale")

    def measure(self, obs):
        measured = {}
        for arm, keys in self.keys.items():
            row = np.asarray([obs[key] for key in keys], dtype=float)
            if row.shape != (7,) or not np.isfinite(row).all():
                raise ValueError(f"Invalid motor feedback for {arm}")
            measured[arm] = row * self.state_scale
            if not np.isfinite(measured[arm]).all() or measured[arm][-1] < 0:
                raise ValueError(f"Invalid SI feedback for {arm}")
        return measured

    def encode(self, targets, scale):
        values = {}
        for arm, keys in self.keys.items():
            row = targets[arm] / scale
            if not np.isfinite(row).all():
                raise ValueError(f"Non-finite motor target for {arm}")
            values.update(zip(keys, map(float, row), strict=True))
        return {key: values[key] for key in self.features}


class LeRobotActions:
    """Submit one bounded set of joint targets and confirm using new motor reads."""

    def __init__(
        self,
        robot,
        settings,
        *,
        layout=None,
        plan_move=None,
        stop=None,
        flange_fk=None,
        grip_contact=None,
        observation_metadata=None,
    ):
        self.robot = robot
        self.settings = settings
        self.layout = layout if layout is not None else MotorLayout(robot, settings)
        self.plan_move = plan_move
        self.stop_robot = stop
        self.flange_fk = flange_fk
        self.grip_contact = grip_contact
        self.observation_metadata = observation_metadata
        self.cancelled = threading.Event()
        self.execution = threading.Lock()
        self.dispatch = threading.Lock()
        self.stop_lock = threading.Lock()
        self.issued = False
        self.hold = {}
        self.timeout = float(settings.get("motion_timeout_s", 10))
        self.poll = float(settings.get("poll_interval_s", 0.02))
        self.tolerance = np.asarray(
            [settings.get("joint_tolerance_rad", 0.01)] * 6
            + [settings.get("width_tolerance_m", 0.002)],
            dtype=float,
        )
        self.pose_tolerance = np.asarray(
            [settings.get("flange_tolerance_m", 0.003), settings.get("flange_tolerance_rad", 0.03)],
            dtype=float,
        )
        limits = np.r_[self.timeout, self.poll, self.tolerance, self.pose_tolerance]
        if not np.isfinite(limits).all() or (limits <= 0).any():
            raise ValueError("Motion tolerances, timeout, and polling interval must be positive")

    def _stop(self):
        self.cancelled.set()
        with self.stop_lock:
            # Interrupt a blocked driver write immediately, then wait until no
            # write can still land. The site stop hook is global and idempotent.
            if not self.dispatch.acquire(blocking=False):
                self.stop_robot(self.robot)
                self.dispatch.acquire()
            try:
                if self.issued:
                    self.stop_robot(self.robot)
                    self.issued = False
            finally:
                self.dispatch.release()
        return {"status": "completed"}

    def _feedback(self):
        obs = self.robot.get_observation()
        meta = (
            self.observation_metadata(self.robot, obs)
            if self.observation_metadata
            else obs.get("_embodied2code")
        )
        if not isinstance(meta, Mapping) or not isinstance(meta.get("arms"), Mapping):
            raise TypeError(
                "Motion feedback requires timestamped arm samples in the observation metadata"
            )
        stamps = {}
        for side in self.layout.keys:
            sample = meta["arms"].get(side)
            if not isinstance(sample, Mapping):
                raise TypeError(f"Missing {side} motion feedback timestamp")
            stamps[side] = _stamp(sample, "timestamp_ns", f"{side} motor sample")
        return self.layout.measure(obs), stamps

    def _at_pose(self, joints, target):
        pose = rigid(self.flange_fk(joints))
        distance = np.linalg.norm(target[:3, 3] - pose[:3, 3])
        angle = Rotation.from_matrix(target[:3, :3].T @ pose[:3, :3]).magnitude()
        return distance <= self.pose_tolerance[0] and angle <= self.pose_tolerance[1]

    def execute_action(self, command, check_path=None):
        if command["kind"] == "stop":
            return self._stop()
        with self.execution:
            if self.cancelled.is_set():
                return {"status": "blocked", "message": "Motion was stopped"}
            if self.stop_robot is None:
                return {"status": "blocked", "message": "Configure the site's interruptible stop"}
            return self._execute(command, check_path)

    def _execute(self, command, check_path):
        arm = command["arm"]
        action_scale = scale_vector(self.settings, "action_scale")
        # Both arms seed the first full motor target and collision check.
        requested_ns = time.monotonic_ns()
        read_deadline = time.monotonic() + self.timeout
        while True:
            measured, stamps = self._feedback()
            if min(stamps.values()) > requested_ns:
                break
            if time.monotonic() >= read_deadline:
                raise TimeoutError("Timed out waiting for measured starting poses")
            if self.cancelled.wait(self.poll):
                return {"status": "blocked", "message": "Stopped before dispatch"}
        if arm not in measured:
            raise ValueError(f"Unknown arm {arm}")
        # Initialize from measured SI, then preserve the last commanded hold target
        # of the idle arm (including the gripper target while holding an object).
        targets = {side: self.hold.get(side, row).copy() for side, row in measured.items()}
        target_pose = None
        kind = command["kind"]
        if kind == "grip":
            joints = measured[arm][None, :6]
            width = float(command["gripper_width_m"])
        elif kind == "move":
            if self.plan_move is None:
                return {
                    "status": "blocked",
                    "message": "Configure the site's flange-to-joints planner",
                }
            target_pose = rigid(command["T_base_flange"])
            joints = np.asarray(
                self.plan_move(self.robot, arm, target_pose.copy(), measured), dtype=float
            )
            width = targets[arm][-1]
        else:
            raise ValueError(f"Unknown action {kind}")
        if not joints.size:
            return {"status": "blocked", "message": "Planner returned no waypoints"}
        if joints.ndim != 2 or joints.shape[1] != 6 or not np.isfinite(joints).all():
            raise ValueError("Planner must return finite six-joint waypoints in radians")
        if not np.isfinite(width) or width < 0:
            raise ValueError("Invalid jaw width")
        if (
            target_pose is not None
            and self.flange_fk is not None
            and not self._at_pose(joints[-1], target_pose)
        ):
            return {"status": "blocked", "message": "Planner endpoint misses requested flange pose"}
        if check_path is not None:
            hold = {
                side: {"joints_rad": row[:6].tolist(), "width_m": float(row[-1])}
                for side, row in targets.items()
                if side != arm
            }
            path = [
                {"joints_rad": row.tolist(), "width_m": width, "other_arms": hold} for row in joints
            ]
            path[0]["measured_start"] = {
                side: {"joints_rad": row[:6].tolist(), "width_m": float(row[-1])}
                for side, row in measured.items()
            }
            verdict = check_path(path)
            if verdict["status"] != "CLEAR":
                return {
                    "status": "blocked",
                    "message": f"Controller path check: {verdict['status']}",
                }

        closing = kind == "grip" and width < measured[arm][-1]
        contact = False
        deadline = time.monotonic() + self.timeout
        try:
            for index, q in enumerate(joints):
                targets[arm] = np.r_[q, width]
                packet = self.layout.encode(targets, action_scale)
                with self.dispatch:
                    if self.cancelled.is_set():
                        return {"status": "blocked", "message": "Stopped before dispatch"}
                    if time.monotonic() >= deadline:
                        raise TimeoutError("PiPER motion exceeded its deadline")
                    self.issued = True
                    sent_ns = time.monotonic_ns()
                    self.robot.send_action(packet)
                while not self.cancelled.is_set():
                    feedback, stamps = self._feedback()
                    actual = feedback[arm]
                    if time.monotonic() > deadline:
                        raise TimeoutError("Timed out waiting for PiPER motor feedback")
                    if stamps[arm] <= sent_ns:
                        self.cancelled.wait(self.poll)
                        continue
                    # For a move, a gripper holding an object can be wider than
                    # its commanded target. Only the six joint errors determine arrival.
                    joints_at_target = np.all(np.abs(actual[:6] - q) <= self.tolerance[:6])
                    width_at_target = abs(actual[-1] - width) <= self.tolerance[-1]
                    if joints_at_target and closing and not width_at_target and self.grip_contact:
                        confirmed = self.grip_contact(self.robot, arm, width, feedback)
                        if not isinstance(confirmed, (bool, np.bool_)):
                            raise TypeError("grip_contact must return a boolean")
                        contact = bool(confirmed)
                    arrived = joints_at_target and (kind == "move" or width_at_target or contact)
                    if arrived:
                        final_pose = target_pose is not None and index == len(joints) - 1
                        if (
                            not final_pose
                            or self.flange_fk is None
                            or self._at_pose(actual[:6], target_pose)
                        ):
                            break
                    self.cancelled.wait(self.poll)
                if self.cancelled.is_set():
                    return {"status": "failed", "message": "Stopped during motion"}
            self.hold = targets
            if contact:
                return {
                    "status": "completed",
                    "message": "Site controller confirmed gripper contact",
                }
            return {"status": "completed", "message": "Measured motor feedback reached the target"}
        except BaseException:
            self._stop()
            raise
