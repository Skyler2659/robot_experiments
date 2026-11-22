"""Agent-facing API. Physical transport is owned by lab.py."""

import numpy as np
from codeworld.world import rigid
from pyrualean._robot import RobotBase
from pyrualean.robots import RobotAdapter


class PhysicalRobot(RobotBase):
    """Calibrated arms. Positions and orientations use the shared world frame."""

    NAME = "physical"
    FINISH_RECOVERY = False
    ABSENT_MEMBERS = frozenset({"world_map", "done"})
    RESULT_TYPES = ()
    SUMMARY_KEYS = ("status", "message", "reached", "blocked")
    STATEFUL_TOOLS = frozenset({"move_pose", "grip"})

    def __init__(self, backend, **kwargs):
        super().__init__(backend, **kwargs)
        self.CAMERAS = tuple(backend.cameras)
        self.ON_MOTION_IMAGES = tuple(name + ".png" for name in self.CAMERAS)
        self.IMAGE_ARTIFACTS = {(name, "high"): name + ".png" for name in self.CAMERAS}

    def _image_name(self, camera, resolution):
        return self.IMAGE_ARTIFACTS[camera, resolution]

    def _state_from_envelope(self, envelope):
        return envelope

    def _summary_dict(self):
        return self.state()

    @property
    def task(self):
        return self._backend.instruction

    def move_to(self, xyz, rotation=None, *, arm=None):
        """Move the selected TCP to world xyz meters. Specify arm="left" or "right" for two arms."""
        arm = self._backend.select_arm(arm)
        target = np.array(self.state()[arm]["T_W_tcp"])
        position = np.asarray(xyz, float)
        if position.shape != (3,):
            raise ValueError("xyz must contain three coordinates")
        target[:3, 3] = position
        if rotation is not None:
            matrix = np.asarray(rotation, float)
            if matrix.shape != (3, 3):
                raise ValueError("rotation must be a 3x3 matrix")
            target[:3, :3] = matrix
        return self._stateful(
            "move_pose", {"arm": arm, "T_W_tcp": rigid(target).tolist()}, dict
        )

    def set_gripper(self, width_m, *, arm=None):
        """Set the absolute jaw opening in meters. Zero requests a close."""
        arm = self._backend.select_arm(arm)
        width = float(width_m)
        if not 0 <= width <= self._backend.max_widths[arm]:
            raise ValueError("Requested width outside the calibrated range")
        current = self.state()[arm]["gripper_width_m"]
        direction = 1 if width == 0 or width < current else -1
        return self._stateful(
            "grip",
            {
                "arm": arm,
                "gripper": direction,
                "opening": width / self._backend.max_widths[arm],
            },
            dict,
        )


ADAPTER = RobotAdapter(
    name="physical",
    robot_cls=PhysicalRobot,
    host="real_robot.host",
    knowledge="physical",
    knowledge_package="real_robot",
    contract_terms=(("host_name", "robot host"), ("backend_name", "robot backend"),
                    ("final_judge", "an observer"), ("step_unit", "robot actions")),
    blurb="the configured physical PiPER arms with calibrated RGB views",
    guides_subdir="",
    show_examples="robo.show(camera_name)",
)
