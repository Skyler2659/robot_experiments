import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image

from codeworld.observations import load
from real_robot.piper import flange_target, import_capture, read_v2_feedback

EXAMPLE = Path(__file__).resolve().parents[1] / "real_robot/data/examples/replay"


@pytest.mark.parametrize(("stage", "block_x"), [("before", 0.235), ("after", 0.36)])
def test_replayed_calibration_projects_into_recorded_object(tmp_path, stage, block_x):
    result = import_capture(EXAMPLE / f"{stage}.json", tmp_path)
    path, capture, _ = load(tmp_path, result["capture"])
    assert capture["frame"] == "piper_base"
    assert capture["metadata"]["source"] == "synthetic"
    assert [v["view_id"] for v in capture["views"]] == ["fixed", "wrist"]
    for view in capture["views"]:
        camera_point = np.linalg.solve(view["T_W_C"], [block_x, 0, 0.035, 1])
        u, v, depth = np.array(view["K"]) @ camera_point[:3]
        assert depth > 0
        with Image.open(path.parent / view["rgb_uri"]) as image:
            assert image.getpixel((round(u / depth), round(v / depth))) == (173, 69, 45)


def test_target_offset_rotates_with_tool():
    poses = json.loads((EXAMPLE / "tcp_target.json").read_text())
    flange = flange_target(**poses)
    np.testing.assert_allclose(flange[:3, 3], [0.30, 0, 0.22])
    np.testing.assert_allclose(flange @ poses["T_flange_tcp"], poses["T_base_tcp"])


def test_v2_feedback_units_and_frame():
    ns = SimpleNamespace
    joints = ns(**{f"joint_{i}": i * 1000 for i in range(1, 7)})
    pose = ns(X_axis=100000, Y_axis=0, Z_axis=200000, RX_axis=90000, RY_axis=0, RZ_axis=90000)
    arm = ns(
        GetArmJointMsgs=lambda: ns(time_stamp=1.0, joint_state=joints),
        GetArmEndPoseMsgs=lambda: ns(time_stamp=2.0, end_pose=pose),
        GetArmGripperMsgs=lambda: ns(time_stamp=3.0, gripper_state=ns(grippers_angle=40000)),
    )
    state = read_v2_feedback(arm)
    np.testing.assert_allclose(state["q_rad"], np.deg2rad(range(1, 7)))
    assert state["gripper_width_m"] == pytest.approx(0.04)
    np.testing.assert_allclose(
        state["T_base_flange"],
        [[0, 0, 1, 0.1], [1, 0, 0, 0], [0, 1, 0, 0.2], [0, 0, 0, 1]],
        atol=1e-12,
    )
    arm.GetArmJointMsgs = lambda: ns(time_stamp=0.0)
    with pytest.raises(ValueError, match="Waiting"):
        read_v2_feedback(arm)
