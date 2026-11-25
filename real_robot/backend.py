"""Map synchronized hardware samples and blocking motion receipts onto PyRUA."""

import re
import threading
import time
from copy import deepcopy
from functools import partial
from pathlib import Path

import numpy as np
from codeworld.observations import load, register_views
from codeworld.world import atomic, rigid
from scipy.spatial.transform import Rotation


class Backend:
    def __init__(self, adapter, config, output):
        self.adapter = adapter
        self.config = config
        self.output = Path(output).resolve()
        self.frame = config["frame"]
        arms = config.get("arms")
        if arms is None:  # Original single-arm configuration uses its base as W.
            arms = {config["arm"]: {**config, "T_W_base": np.eye(4)}}
        if not arms or set(arms) - {"left", "right"}:
            raise ValueError("Configure the physical arms by name: left and/or right")
        self.arms = tuple(arms)
        self.bases = {}
        self.tools = {}
        self.max_widths = {}
        for arm, spec in arms.items():
            for name in ("T_W_base", "T_flange_tcp", "max_gripper_width_m"):
                if spec.get(name) is None:
                    raise ValueError(f"Fill the measured {arm}.{name} in the site configuration")
            self.bases[arm] = rigid(spec["T_W_base"])
            self.tools[arm] = rigid(spec["T_flange_tcp"])
            width = float(spec["max_gripper_width_m"])
            if not np.isfinite(width) or width <= 0:
                raise ValueError(f"Invalid gripper travel for {arm}")
            self.max_widths[arm] = width
        self.cameras = config["cameras"]
        self.rectification = {}
        for name, camera in self.cameras.items():
            if "raw_distortion" in camera:
                if getattr(adapter, "rectifies_images", False):
                    raise ValueError("Choose site.rectify_image or raw_distortion for " + name)
                import cv2
                size = tuple(camera["image_size"])
                self.rectification[name] = cv2.initUndistortRectifyMap(
                    np.asarray(camera["raw_K"], float), np.asarray(camera["raw_distortion"], float),
                    None, np.asarray(camera["K"], float), size, cv2.CV_32FC1)

        if not self.cameras or not all(re.fullmatch(r"[A-Za-z0-9_-]+", k) for k in self.cameras):
            raise ValueError("Configure named cameras")
        self.last_action = None
        self.initial_joints = {}
        self.step = 0
        self.after_ns = -1
        self.stopped = False
        self.checker = None
        self.packets = {}
        self.lock = threading.Lock()
        self.last = None
        self.video = None

    def start_video(self):
        from .video import VideoRecorder

        video = VideoRecorder(self.output, tuple(self.cameras), self.adapter.read_video_frames,
                              fps=self.config.get("video_fps", 15))
        try:
            video.start()
        except BaseException as exc:
            self._video_report(video, error=exc)
            raise
        self.video = video

    def _video_report(self, video, error=None):
        directory = self.output / "videos"
        directory.mkdir(exist_ok=True)
        report = {"status": "completed" if error is None else "failed",
                  "fps": video.fps, "frames_per_camera": video.frames_written}
        if error is None:
            report["files"] = {name: f"videos/{name}.mp4" for name in self.cameras}
        else:
            report["error"] = f"{type(error).__name__}: {error}"
        atomic(directory / "recording.json", report)

    def finish_video(self):
        video, self.video = self.video, None
        if video is None:
            return
        try:
            video.close()
        except BaseException as exc:
            self._video_report(video, error=exc)
            raise
        self._video_report(video)

    def select_arm(self, arm):
        if arm is None and len(self.arms) == 1:
            return self.arms[0]
        if arm not in self.arms:
            raise ValueError(f"Select an arm from {self.arms}")
        return arm

    def _state_at(self, stamp):
        samples = self.adapter.read_state(stamp)
        if not isinstance(samples, dict):
            samples = {self.select_arm(None): samples}
        if set(samples) != set(self.arms):
            raise ValueError("State must include every configured arm")
        state = {"step": self.step, "timestamp_ns": stamp}
        for arm, sample in samples.items():
            if type(sample.timestamp_ns) is not int:
                raise ValueError("ArmState.timestamp_ns must be integer nanoseconds")
            if abs(sample.timestamp_ns - stamp) > self.config["state_tolerance_ns"]:
                raise ValueError(f"{arm} state is too far from exposure")
            q = np.asarray(sample.joints_rad, float)
            width = float(sample.gripper_width_m)
            if q.shape != (6,) or not np.isfinite(q).all() or not np.isfinite(width) or width < 0:
                raise ValueError(f"Invalid measured joints or gripper width for {arm}")
            flange = rigid(sample.T_base_flange)
            world_flange = self.bases[arm] @ flange
            state[arm] = {
                "timestamp_ns": sample.timestamp_ns,
                "joints": q.tolist(),
                "gripper_width_m": width,
                "gripper": min(width / self.max_widths[arm], 1.0),
                "eef_pos": world_flange[:3, 3].tolist(),
                "eef_quat": Rotation.from_matrix(world_flange[:3, :3]).as_quat()[[3, 0, 1, 2]].tolist(),
                "T_base_flange": flange.tolist(),
                "T_base_tcp": (flange @ self.tools[arm]).tolist(),
                "T_W_flange": world_flange.tolist(),
                "T_W_tcp": (world_flange @ self.tools[arm]).tolist(),
            }
        if len(self.arms) == 1:
            state["timestamp_ns"] = samples[self.arms[0]].timestamp_ns
        return state

    def acquire(self):
        if self.stopped:
            raise RuntimeError("Physical device stopped; no new observation is available")
        try:
            return self._acquire()
        except Exception:
            if self.config["backend"] == "lab" and not self.stopped:
                self.cancel()
            raise

    def _acquire(self):
        if self.video is not None:
            self.video.check()
        images = self.adapter.read_images(self.after_ns)
        if set(images) != set(self.cameras):
            raise ValueError("Image names differ from camera calibration")
        views = []
        states = {}
        for name, calibration in self.cameras.items():
            image = images[name]
            stamp = image.exposure_ns
            if type(stamp) is not int or not self.after_ns < stamp <= time.monotonic_ns():
                raise ValueError(f"Stale or wrong-clock frame: {name}")
            rgb = np.asarray(image.rgb)
            w, h = calibration["image_size"]
            if rgb.dtype != np.uint8 or rgb.shape != (h, w, 3):
                raise ValueError(f"Rectified RGB differs from calibrated dimensions: {name}")
            if name in self.rectification:
                import cv2
                rgb = cv2.remap(rgb, *self.rectification[name], interpolation=cv2.INTER_LINEAR)
            if stamp not in states:
                states[stamp] = self._state_at(stamp)
            mount = calibration["mount"]
            if mount == "fixed":
                if "T_W_camera" in calibration:
                    extrinsic = rigid(calibration["T_W_camera"])
                else:
                    arm = self.select_arm(calibration.get("arm"))
                    extrinsic = self.bases[arm] @ rigid(calibration["T_base_camera"])
            elif mount == "wrist":
                arm = self.select_arm(calibration.get("arm"))
                extrinsic = rigid(states[stamp][arm]["T_W_flange"]) @ rigid(calibration["T_flange_camera"])
            else:
                raise ValueError(f"Unknown camera mount: {mount}")
            views.append(
                {
                    "view_id": name,
                    "image": rgb.copy(),
                    "K": calibration["K"],
                    "T_W_C": extrinsic,
                    "timing": {"exposure_ns": stamp, "recorded_ns": image.recorded_ns},
                }
            )
        if max(states) - min(states) > self.config["view_tolerance_ns"]:
            raise ValueError("Camera exposure skew exceeds the configured tolerance")
        self.last = (views, states[views[0]["timing"]["exposure_ns"]])
        if not self.initial_joints:
            self.initial_joints = {side: self.last[1][side]["joints"][:] for side in self.arms}
        return self.last

    def snapshot(self):
        return self.last or self.acquire()

    def capture_into(self, root, refresh=False):
        views, state = self.acquire() if refresh else self.snapshot()
        p = register_views(
            root,
            views,
            frame=self.frame,
            robot_state=state,
            metadata={
                "host_step": self.step,
                "source": self.config["backend"],
                "image_robot_state": state,
                "clock": "host_monotonic_ns",
            },
        )
        path, packet, _ = load(root, p["capture"])
        self.packets[self.step] = (path.parent, packet)
        return p

    def artifact(self, name, step=-1):
        index = self.step if step == -1 else step
        if index == self.step and index not in self.packets:
            self.capture_into(self.output / "feedback")
        folder, packet = self.packets[index]
        for view in packet["views"]:
            if name == view["view_id"] + ".png":
                return (folder / view["rgb_uri"]).read_bytes()
        raise ValueError(f"Unknown image {name}")

    def check_planned_path(self, waypoints, arm=None):
        """Check the measured start and motor targets, including the other arm’s hold command."""
        arm = self.select_arm(arm)
        if not waypoints:
            raise ValueError("Expected the planned joint/width waypoints")
        def joint_fields(rows):
            fields = {}
            for side, row in rows.items():
                fields[side + "_arm_joint_state"] = row["joints_rad"]
                fields[side + "_ee_joint_state"] = [row["width_m"] / self.max_widths[side]]
            return fields

        start = waypoints[0].get("measured_start")
        if start is None:
            state = self.snapshot()[1]
            start = {s: {"joints_rad": state[s]["joints"], "width_m": state[s]["gripper_width_m"]}
                     for s in self.arms}
        if set(start) != set(self.arms):
            raise ValueError("Planned path must start from the measured state of every arm")
        raw = joint_fields(start)
        actions = []
        for point in waypoints:
            others = point.get("other_arms", {})
            if set(others) - (set(self.arms) - {arm}):
                raise ValueError("Invalid other arm in planned path")
            actions.append(joint_fields({arm: point, **others}))
        return self.checker(raw, actions, phase="planned_hardware_path")

    def set_motion_checker(self, checker):
        self.checker = checker

    def execute_tool(self, tool, args):
        if tool == "view_env_state":
            step = args.get("step", -1)
            if step not in (-1, self.step):
                return deepcopy(self.packets[step][1]["robot_state"])
            return deepcopy(self.snapshot()[1])
        with self.lock:
            if self.stopped or self.step >= self.limit:
                raise RuntimeError("Episode stopped or action budget exhausted")
            if self.video is not None:
                self.video.check()
            arm = self.select_arm(args.get("arm"))
            mode = args.get("path", "auto")
            if tool == "move_pose" and mode == "home":
                command = {"kind": "home", "joints_rad": self.initial_joints[arm]}
            elif tool == "move_pose":
                if "T_W_tcp" in args:
                    tcp = np.linalg.inv(self.bases[arm]) @ rigid(args["T_W_tcp"])
                else:
                    tcp = rigid(args["T_base_tcp"])
                command = {
                    "kind": "move",
                    "T_base_tcp": tcp.tolist(),
                    "T_base_flange": (tcp @ np.linalg.inv(self.tools[arm])).tolist(),
                }
                if mode == "line":
                    command.update(path="line", T_flange_tcp=self.tools[arm].tolist())
                elif mode != "auto":
                    raise ValueError("Unknown motion mode")
            elif tool == "grip":
                command = {"kind": "grip", "gripper_width_m": args["opening"] * self.max_widths[arm]}
            else:
                raise ValueError(f"Unknown tool {tool}")
            command["arm"] = arm
            step = self.step + 1
            receipt_path = self.output / f"action-{step:04}.json"
            atomic(receipt_path, {"step": step, "command": command, "status": "dispatched"})
            try:
                if mode in ("line", "home") and mode not in self.motion_modes():
                    receipt = {"status": "blocked", "moved": False,
                               "message": f"Checked {mode} planner unavailable"}
                else:
                    receipt = self.adapter.execute_action(
                        command, partial(self.check_planned_path, arm=arm) if self.checker else None
                    )
                if receipt["status"] not in {"completed", "blocked", "failed"}:
                    raise ValueError("Unrecognized controller receipt")
                atomic(receipt_path, {"step": step, "command": command, "receipt": receipt})
            except BaseException as exc:
                self.last_action = {"step": step, "arm": arm, "status": "unknown",
                                    "moved": None, "reached": False, "message": str(exc)}
                atomic(
                    receipt_path,
                    {"step": step, "command": command, "status": "unknown", "error": str(exc)},
                )
                self.cancel()
                raise
            finally:
                self.step = step
                self.after_ns = time.monotonic_ns()
                self.last = None
            self.last_action = {
                "step": step, "arm": arm, "moved": receipt.get("moved"), **receipt,
                "reached": receipt["status"] == "completed",
                "blocked": receipt["status"] == "blocked",
            }

            if receipt.get("terminal"):
                self.cancel()
            return deepcopy(self.last_action)

    def motion_modes(self):
        modes = ["auto"]
        actions = getattr(self.adapter, "actions", None)
        if self.checker and actions and actions.flange_fk is not None:
            if actions.plan_line or actions.plan_move:
                modes.append("line")
            if actions.plan_home or actions.plan_move:
                modes.append("home")
        return modes

    def episode_over(self):
        if not self.stopped and self.video is not None:
            try:
                self.video.check()
            except RuntimeError:
                self.cancel()
        return self.stopped

    def cancel(self):
        self.stopped = True
        self.adapter.execute_action({"kind": "stop"})

    def close(self):
        try:
            if not self.stopped:
                self.cancel()
        finally:
            try:
                self.finish_video()
            finally:
                close = getattr(self.adapter, "close", None)
                if close:
                    close()

    def env_steps(self):
        return self.step

    def solved(self):
        return None  # The observer scores the physical outcome.
