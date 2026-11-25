"""Deterministic transport replay. Commands advance a recording, never simulate a policy."""

import time
from pathlib import Path

import numpy as np
from codeworld.world import read
from PIL import Image

from .lab import ArmState, ImageFrame


class ReplayAdapter:
    def __init__(self, settings):
        self.files = [Path(p) for p in settings["records"]]
        self.index = 0
        self.commands = []

    def read_video_frames(self):
        return {name: frame.rgb for name, frame in self.read_images(-1).items()}

    def read_images(self, after_ns):
        path = self.files[self.index]
        stamp = time.monotonic_ns()
        views = {}
        for v in read(path)["views"]:
            with Image.open(path.parent / v["rgb"]) as image:
                views[v["id"]] = ImageFrame(np.array(image.convert("RGB")), stamp, v["exposure_ns"])
        return views

    def read_state(self, at_ns):
        state = read(self.files[self.index])["robot_state"]

        def sample(row):
            return ArmState(row["q_rad"], row["T_base_flange"], row["gripper_width_m"], at_ns)

        if "q_rad" in state:
            return sample(state)
        return {arm: sample(row) for arm, row in state.items()}

    @property
    def exhausted(self):
        return self.index + 1 >= len(self.files)

    def execute_action(self, command, check_path=None):
        self.commands.append(command)
        if command["kind"] == "stop":
            return {"status": "completed"}
        if self.index + 1 >= len(self.files):
            return {"status": "blocked", "message": "End of recording"}
        if check_path is not None:
            raise ValueError("Replay has no planned controller trajectory")
        self.index += 1
        return {"status": "completed", "message": "Advanced the synthetic recording"}
