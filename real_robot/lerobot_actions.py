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
        plan_line=None,
        plan_home=None,
        stop=None,
        flange_fk=None,
        grip_contact=None,
        observation_metadata=None,
    ):
        self.robot = robot
        self.settings = settings
        self.layout = layout if layout is not None else MotorLayout(robot, settings)
        self.plan_line = plan_line
        self.plan_home = plan_home
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

    def _follows_tcp_line(self, start_q, waypoints, tool, goal_tcp):
        """Sample the planned joint interpolants, including between site waypoints."""
        initial_tcp = rigid(self.flange_fk(start_q)) @ tool
        start = initial_tcp[:3, 3]
        initial_orientation = Rotation.from_matrix(initial_tcp[:3, :3])
        relative_goal = initial_orientation.inv() * Rotation.from_matrix(goal_tcp[:3, :3])
        rotation_vector = relative_goal.as_rotvec()
        delta = goal_tcp[:3, 3] - start
        length = np.linalg.norm(delta)
        direction = delta / length if length else np.zeros(3)
        previous = start_q
        furthest = 0.0
        for target in waypoints:
            steps = max(1, int(np.ceil(np.max(np.abs(target - previous)) / 0.025)))
            for q in np.linspace(previous, target, steps + 1)[1:]:
                tcp = rigid(self.flange_fk(q)) @ tool
                offset = tcp[:3, 3] - start
                distance = float(offset @ direction)
                closest = np.clip(distance, 0., length) * direction
                if (np.linalg.norm(offset - closest) > self.pose_tolerance[0]
                        or distance < furthest - self.pose_tolerance[0]):
                    return False
                if length > 1e-9 or relative_goal.magnitude() < self.pose_tolerance[1]:
                    fraction = np.clip(distance / length, 0., 1.) if length > 1e-9 else 0.
                    expected = initial_orientation * Rotation.from_rotvec(fraction * rotation_vector)
                    if (expected.inv() * Rotation.from_matrix(tcp[:3, :3])).magnitude() > self.pose_tolerance[1]:
                        return False
                furthest = max(furthest, distance)
            previous = target
        return True

    def execute_action(self, command, check_path=None):
        if command["kind"] == "stop":
            return self._stop()
        with self.execution:
            if self.cancelled.is_set():
                return {"status": "blocked", "moved": False, "terminal": True, "message": "Motion was stopped"}
            if self.stop_robot is None:
                return {"status": "blocked", "moved": False, "message": "Configure the site's interruptible stop"}
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
                return {"status": "blocked", "moved": False, "message": "Stopped before dispatch"}
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
            line = command.get("path") == "line"
            planner = self.plan_line if line and self.plan_line else self.plan_move
            if line and (check_path is None or self.flange_fk is None):
                return {"status": "blocked", "moved": False, "message": "Checked line unavailable"}
            if planner is None:
                return {
                    "status": "blocked", "moved": False,
                    "message": "Configure the site's flange-to-joints planner",
                }
            target_pose = rigid(command["T_base_flange"])
            if line:
                tool = rigid(command["T_flange_tcp"])
            if line and self.plan_line:
                planned = planner(self.robot, arm, target_pose.copy(), measured, tool.copy())
            else:
                planned = planner(self.robot, arm, target_pose.copy(), measured)
            joints = np.asarray(planned, dtype=float)
            width = targets[arm][-1]
        elif kind == "home":
            if not (self.plan_home or self.plan_move) or check_path is None or self.flange_fk is None:
                return {"status": "blocked", "moved": False, "message": "Checked home unavailable"}
            goal = np.asarray(command["joints_rad"], float)
            target_pose = rigid(self.flange_fk(goal))
            if self.plan_home:
                planned = self.plan_home(self.robot, arm, goal.copy(), measured)
            else:
                planned = self.plan_move(self.robot, arm, target_pose.copy(), measured)
            joints = np.asarray(planned, float)
            if joints.ndim != 2 or joints.shape[1:] != (6,) or not len(joints) or not np.allclose(
                joints[-1], goal, atol=self.tolerance[0], rtol=0
            ):
                return {"status": "blocked", "moved": False, "message": "Home path misses initial joints"}
            width = targets[arm][-1]
        else:
            raise ValueError(f"Unknown action {kind}")
        if not joints.size:
            return {"status": "blocked", "moved": False, "message": "Planner returned no waypoints"}
        if joints.ndim != 2 or joints.shape[1] != 6 or not np.isfinite(joints).all():
            raise ValueError("Planner must return finite six-joint waypoints in radians")
        if not np.isfinite(width) or width < 0:
            raise ValueError("Invalid jaw width")
        if (
            target_pose is not None
            and self.flange_fk is not None
            and not self._at_pose(joints[-1], target_pose)
        ):
            return {"status": "blocked", "moved": False, "message": "Planner endpoint misses requested flange pose"}
        if kind == "move" and line and not self._follows_tcp_line(
            measured[arm][:6], joints, tool, target_pose @ tool
        ):
            return {"status": "blocked", "moved": False, "message": "Planned TCP leaves line"}
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
                    "status": "blocked", "moved": False,
                    "collision_check": verdict,
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
                        return {"status": "failed", "terminal": True, "moved": bool(index), "message": "Motion stopped"}
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
                    arrived = joints_at_target and (kind != "grip" or width_at_target or contact)
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
                    return {"status": "failed", "terminal": True, "message": "Stopped during motion"}
            self.hold = targets
            displacement = {
                "joint_change_rad": (actual[:6] - measured[arm][:6]).tolist(),
                "width_change_m": float(actual[-1] - measured[arm][-1]),
            }
            # A completed stroke can establish contact with no measurable displacement.
            # Keep that case eligible for the post-action visual review.
            changed = bool(np.any(np.abs(actual - measured[arm]) > self.tolerance))
            return {
                "status": "completed",
                "message": ("Site controller confirmed gripper contact" if contact
                            else "Measured motor feedback reached the target"),
                "moved": True if changed else None,
                "measured_displacement": displacement,
                "contact_confirmed": contact if closing and self.grip_contact else None,
            }
        except BaseException:
            self._stop()
            raise
