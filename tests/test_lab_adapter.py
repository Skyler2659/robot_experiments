"""Offline tests of the lab adapter against the deployment client's LeRobot contract."""

import time

import numpy as np
import pytest

from real_robot.lab import LabAdapter

SIDES = ("left", "right")
CAMERAS = ("top", "l_wrist", "r_wrist")
JOINT_SCALE = np.pi / 180_000
WIDTH_SCALE = 1e-6


def motor_names():
    # Deliberately synthetic feature names: the live workstation supplies its own names.
    return {
        side: {"joints": [f"{side}_q{i}" for i in range(6)], "gripper": f"{side}_jaw"}
        for side in SIDES
    }


class RecordingRobot:
    """Returns raw encoder units and accepts SI motor targets as in the home preset."""

    def __init__(self):
        self.cameras = dict.fromkeys(CAMERAS)
        self.motor_keys = motor_names()
        self.si = {}
        for i, names in enumerate(self.motor_keys.values()):
            self.si.update({key: 0.1 * (1 + j) + i for j, key in enumerate(names["joints"])})
            self.si[names["gripper"]] = 0.02 + 0.01 * i
        # Deliberately different from joint-list order to detect positional slicing.
        self.action_features = dict.fromkeys(reversed(list(self.si)), float)
        self.observations = 0
        self.commands = []
        self.exposure = {}
        self.flange = {}
        self.state_ns = {}
        self.with_metadata = True
        self.follow = True
        self.connected = False

    def connect(self):
        assert not self.connected
        self.connected = True

    def disconnect(self):
        assert self.connected
        self.connected = False

    def get_observation(self):
        assert self.connected
        self.observations += 1
        stamp = time.monotonic_ns()
        self.exposure = {cam: stamp - i for i, cam in enumerate(CAMERAS)}
        self.state_ns = {side: stamp - 10 - i for i, side in enumerate(SIDES)}
        obs = {
            name: np.full((4, 6, 3), i + 5, dtype=np.uint8)
            for i, name in enumerate(CAMERAS)
        }
        for side, names in self.motor_keys.items():
            for key in names["joints"]:
                obs[key] = self.si[key] / JOINT_SCALE
            obs[names["gripper"]] = self.si[names["gripper"]] / WIDTH_SCALE
            flange = np.eye(4)
            flange[0, 3] = self.si[names["joints"][0]]
            self.flange[side] = flange.tolist()
        if self.with_metadata:
            obs["_embodied2code"] = {
                "exposure_ns": self.exposure,
                "arms": {
                    side: {"timestamp_ns": self.state_ns[side], "T_base_flange": self.flange[side]}
                    for side in SIDES
                },
            }
        return obs

    def send_action(self, action):
        assert self.connected
        assert set(action) == set(self.action_features)
        self.commands.append(dict(action))
        if self.follow:
            self.si.update(action)
        return action


class Site:
    def __init__(self):
        self.plans = []
        self.stops = 0

    def plan_move(self, robot, arm, T_base_flange, measured):
        self.plans.append((arm, T_base_flange, measured))
        row = list(measured[arm][:6])
        middle = row.copy()
        goal = row.copy()
        goal[0] = T_base_flange[0][3]
        middle[0] = (row[0] + goal[0]) / 2
        return [middle, goal]

    def stop(self, robot):
        assert robot.connected
        self.stops += 1


@pytest.fixture
def settings():
    return {
        "cameras": {key: {"type": "opencv"} for key in CAMERAS},
        "camera_map": {key: key for key in CAMERAS},
        "motor_keys": {
            arm: [*names["joints"], names["gripper"]] for arm, names in motor_names().items()
        },
        "state_scale": [JOINT_SCALE] * 6 + [WIDTH_SCALE],
        "action_scale": [1.0] * 7,
        "motion_timeout_s": 0.2,
        "joint_tolerance_rad": 0.001,
        "width_tolerance_m": 0.0001,
        "poll_interval_s": 0.002,
    }


def test_lab_capture_uses_one_observation_for_all_images_and_both_states(settings):
    robot = RecordingRobot()
    adapter = LabAdapter(settings, robot=robot)
    try:
        frames = adapter.read_images(after_ns=-1)
        assert set(frames) == set(CAMERAS)
        for i, cam in enumerate(CAMERAS):
            frame = frames[cam]
            assert frame.exposure_ns == robot.exposure[cam]
            np.testing.assert_array_equal(frame.rgb, np.full((4, 6, 3), 5 + i, np.uint8))
            states = adapter.read_state(frame.exposure_ns)
            for side in SIDES:
                keys = robot.motor_keys[side]
                sample = states[side]
                np.testing.assert_allclose(sample.joints_rad, [robot.si[k] for k in keys["joints"]])
                assert sample.gripper_width_m == pytest.approx(robot.si[keys["gripper"]])
                assert sample.timestamp_ns == robot.state_ns[side]
                np.testing.assert_allclose(sample.T_base_flange, robot.flange[side])
        assert robot.observations == 1
        assert robot.commands == []
    finally:
        adapter.close()
    assert not robot.connected


def test_lab_capture_rejects_unstamped_driver_packet(settings):
    robot = RecordingRobot()
    robot.with_metadata = False
    adapter = LabAdapter(settings, robot=robot)
    try:
        with pytest.raises((ValueError, RuntimeError), match="(?i)metadata|timestamp|exposure"):
            adapter.read_images(-1)
        assert robot.commands == []
    finally:
        adapter.close()


def test_lab_planned_move_preserves_inactive_arm_and_checks_before_dispatch(settings):
    robot = RecordingRobot()
    site = Site()
    adapter = LabAdapter(settings, robot=robot, site=site)
    initial = dict(robot.si)
    goal = np.eye(4)
    goal[0, 3] = 0.43
    command = {"kind": "move", "arm": "left", "T_base_flange": goal.tolist(),
               "T_base_tcp": goal.tolist()}
    seen = []

    def check(points):
        assert robot.commands == []
        seen.append(points)
        return {"status": "COLLISION" if len(seen) == 1 else "CLEAR"}

    try:
        assert adapter.execute_action(command, check)["status"] == "blocked"
        assert robot.commands == []
        assert adapter.execute_action(command, check)["status"] == "completed"
        assert len(robot.commands) == 2
        np.testing.assert_allclose(site.plans[-1][1], goal)
        assert site.plans[-1][0] == "left"
        for sent, waypoint in zip(robot.commands, seen[-1], strict=True):
            for name in settings["motor_keys"]["right"]:
                assert sent[name] == pytest.approx(initial[name])
            np.testing.assert_allclose(
                [sent[key] for key in settings["motor_keys"]["left"][:6]], waypoint["joints_rad"]
            )
            assert sent["left_jaw"] == pytest.approx(waypoint["width_m"])
        assert robot.si["left_q0"] == pytest.approx(0.43)
        grip = {"kind": "grip", "arm": "right", "gripper_width_m": 0.047}
        assert adapter.execute_action(grip)["status"] == "completed"
        assert robot.commands[-1]["right_jaw"] == pytest.approx(0.047)
        assert robot.commands[-1]["left_q0"] == pytest.approx(0.43)
        assert adapter.execute_action({"kind": "stop"})["status"] == "completed"
        assert site.stops == 1
    finally:
        adapter.close()
    assert not robot.connected


def test_lab_stop_interrupts_unreached_target(settings):
    from concurrent.futures import ThreadPoolExecutor
    from threading import Event

    robot = RecordingRobot()
    robot.follow = False
    dispatched = Event()
    send = robot.send_action

    def send_and_signal(action):
        result = send(action)
        dispatched.set()
        return result

    robot.send_action = send_and_signal
    site = Site()
    adapter = LabAdapter({**settings, "motion_timeout_s": 1.0}, robot=robot, site=site)
    try:
        with ThreadPoolExecutor(max_workers=1) as pool:
            outcome = pool.submit(
                adapter.execute_action, {"kind": "grip", "arm": "left", "gripper_width_m": 0.06}
            )
            assert dispatched.wait(0.5)
            adapter.execute_action({"kind": "stop"})
            assert site.stops == 1
            assert outcome.result(timeout=0.5)["status"] != "completed"
        assert len(robot.commands) == 1
    finally:
        adapter.close()


def test_incomplete_lab_calibration_never_connects(tmp_path, settings, monkeypatch):
    import json

    from real_robot import host, lab
    from real_robot.__main__ import DATA

    robot = RecordingRobot()
    config = json.loads((DATA / "configs/lab.example.json").read_text())
    config["driver"] = settings
    path = tmp_path / "lab.json"
    path.write_text(json.dumps(config))
    monkeypatch.setattr(lab, "make_lab_robot", lambda _: robot)
    with pytest.raises(ValueError, match="measured"):
        host.boot(host.Cell("pack", "pack"), tmp_path, config_path=path)
    assert not robot.connected
    assert robot.observations == 0 and robot.commands == []


def test_lab_optional_urdf_flange_and_missing_planner(settings):
    import json
    from types import SimpleNamespace

    from real_robot.__main__ import DATA

    robot = RecordingRobot()
    reference = json.loads((DATA / "examples/replay/before.json").read_text())["robot_state"]
    for names in robot.motor_keys.values():
        robot.si.update(zip(names["joints"], reference["q_rad"], strict=True))

    def metadata(robot, obs):
        meta = obs["_embodied2code"]
        for sample in meta["arms"].values():
            del sample["T_base_flange"]
        return meta

    site = SimpleNamespace(observation_metadata=metadata, stop=lambda robot: None)
    adapter = LabAdapter({**settings, "flange_fk": "piper_urdf"}, robot=robot, site=site)
    try:
        frames = adapter.read_images(-1)
        states = adapter.read_state(frames["top"].exposure_ns)
        for side in SIDES:
            # The synthetic reference approximates the delivered URDF link parameters.
            measured = np.asarray(states[side].T_base_flange)
            expected = np.asarray(reference["T_base_flange"])
            assert np.linalg.norm(measured[:3, 3] - expected[:3, 3]) < 0.0005
            np.testing.assert_allclose(measured[:3, :3], expected[:3, :3], atol=0.0005)
        cmd = {"kind": "move", "arm": "left", "T_base_flange": np.eye(4).tolist()}
        receipt = adapter.execute_action(cmd)
        assert receipt["status"] == "blocked"
        assert not robot.commands
    finally:
        adapter.close()


def test_contact_receipt_preserves_held_width_command_in_checked_path(settings):
    robot = RecordingRobot()
    site = Site()
    robot.follow = False  # The left gripper closes onto an object at 0.02 m.
    confirmations = []

    def grip_contact(device, arm, target, measured):
        confirmations.append((arm, target, measured[arm][-1]))
        return arm == "left" and target == 0 and measured["left"][-1] > target

    site.grip_contact = grip_contact
    adapter = LabAdapter(settings, robot=robot, site=site)
    try:
        result = adapter.execute_action({"kind": "grip", "arm": "left", "gripper_width_m": 0.0})
        assert result["status"] == "completed" and "contact" in result["message"]
        assert confirmations == [("left", 0.0, pytest.approx(.02))]
        checked = []

        def check(path):
            checked.extend(path)
            return {"status": "CLEAR"}

        robot.follow = True
        result = adapter.execute_action(
            {"kind": "grip", "arm": "right", "gripper_width_m": .035}, check
        )
        assert result["status"] == "completed"
        assert checked[0]["measured_start"]["left"]["width_m"] == pytest.approx(.02)
        assert checked[0]["other_arms"]["left"]["width_m"] == robot.commands[-1]["left_jaw"] == 0
    finally:
        adapter.close()


def test_move_blocks_when_planner_endpoint_misses_flange(settings):
    robot = RecordingRobot()
    site = Site()
    site.plan_move = lambda device, arm, pose, measured: [measured[arm][:6]]
    adapter = LabAdapter({**settings, "flange_fk": "piper_urdf"}, robot=robot, site=site)
    try:
        goal = np.eye(4)
        goal[0, 3] = .25
        result = adapter.execute_action({"kind": "move", "arm": "right", "T_base_flange": goal})
        assert result["status"] == "blocked" and "endpoint" in result["message"]
        assert not robot.commands
    finally:
        adapter.close()
    assert site.stops == 0


@pytest.mark.parametrize("stale", [False, True], ids=["stalled", "stale-at-target"])
def test_timeout_stops_once_without_extra_motor_or_release_commands(settings, stale):
    robot = RecordingRobot()
    site = Site()
    robot.follow = stale
    read = robot.get_observation

    def sample():
        obs = read()
        if stale and robot.commands:
            # Even an on-target numeric reading cannot acknowledge a new command
            # if its sample predates dispatch.
            obs["_embodied2code"]["arms"]["left"]["timestamp_ns"] = 0
        return obs

    robot.get_observation = sample
    adapter = LabAdapter({**settings, "motion_timeout_s": .04}, robot=robot, site=site)
    command = {"kind": "grip", "arm": "left", "gripper_width_m": .055}
    try:
        with pytest.raises(TimeoutError, match="motor feedback"):
            adapter.execute_action(command)
        assert adapter.execute_action(command)["status"] == "blocked"
        assert len(robot.commands) == 1 and site.stops == 1
        assert robot.connected
    finally:
        adapter.close()
    assert len(robot.commands) == 1 and site.stops == 1 and not robot.connected


def test_lab_host_captures_and_dispatches_checked_bimanual_path(tmp_path, settings, monkeypatch):
    import json

    from real_robot import host, lab
    from real_robot.__main__ import DATA

    cfg = json.loads((DATA / "configs/lab.example.json").read_text())
    cfg["driver"] = {**settings, "site_module": "local_test_controller"}
    site = Site()
    robot = RecordingRobot()
    monkeypatch.setattr(lab, "import_module", lambda name: site)
    monkeypatch.setattr(lab, "make_lab_robot", lambda settings: robot)
    for arm in cfg["arms"].values():
        arm.update(T_W_base=np.eye(4).tolist(), T_flange_tcp=np.eye(4).tolist(),
                   max_gripper_width_m=0.08)
    for camera in cfg["cameras"].values():
        key = "T_W_camera" if camera["mount"] == "fixed" else "T_flange_camera"
        camera.update(image_size=[6, 4], K=[[8, 0, 3], [0, 8, 2], [0, 0, 1]])
        camera[key] = np.eye(4).tolist()
    path = tmp_path / "lab.json"
    path.write_text(json.dumps(cfg))
    backend, _ = host.boot(host.Cell("pack", "pack"), tmp_path, config_path=path)
    try:
        assert {v["view_id"] for v in backend.packets[0][1]["views"]} == set(CAMERAS)
        # The controller has moved since the initial snapshot. Path validation must
        # start from this newer read and account for both arms' commanded targets.
        robot.si["right_q0"] = 0.9
        def checker(raw, actions, **kw):
            assert raw["right_arm_joint_state"][0] == pytest.approx(0.9)
            assert actions[0]["right_arm_joint_state"][0] == pytest.approx(0.9)
            assert actions[0]["left_ee_joint_state"] == pytest.approx([0.04 / 0.08])
            return {"status": "CLEAR"}

        backend.set_motion_checker(checker)
        receipt = backend.execute_tool("grip", {"arm": "left", "opening": 0.5})
        assert receipt["reached"]
        packet = backend.capture_into(tmp_path / "feedback", refresh=True)
        assert packet["robot_state"]["left"]["gripper_width_m"] == pytest.approx(.04)
        assert len(robot.commands) == 1
    finally:
        backend.close()
    assert not robot.connected and site.stops == 1


def test_checked_line_and_home_validate_candidate_before_dispatch(settings):
    robot = RecordingRobot()
    site = Site()
    adapter = LabAdapter(settings, robot=robot, site=site)
    def fk(q):
        pose = np.eye(4)
        pose[:2, 3] = q[:2]
        return pose

    adapter.actions.flange_fk = fk
    try:
        initial = np.array([robot.si[k] for k in settings["motor_keys"]["right"][:6]])
        target = fk(initial)
        target[0, 3] += .04
        command = {"kind": "move", "path": "line", "arm": "right",
                   "T_base_flange": target.tolist(), "T_flange_tcp": np.eye(4).tolist()}
        def detour(device, arm, pose, measured, tool):
            via = measured[arm][:6].copy()
            via[1] += .04
            end = measured[arm][:6].copy()
            end[0] = pose[0, 3]
            return [via, end]

        adapter.actions.plan_line = detour
        clear = lambda path: {"status": "CLEAR"}
        assert adapter.execute_action(command, clear)["moved"] is False
        assert not robot.commands
        adapter.actions.plan_line = None
        collision = {"status": "COLLISION", "hits": [{"first": "right/link7", "second": "tube_rack"}]}
        refusal = adapter.execute_action(command, lambda path: collision)
        assert refusal["collision_check"] == collision and refusal["moved"] is False
        assert not robot.commands
        assert adapter.execute_action(command, clear)["status"] == "completed"
        home = {"kind": "home", "arm": "right", "joints_rad": initial.tolist()}
        assert adapter.execute_action(home, clear)["status"] == "completed"
        actual = [robot.si[k] for k in settings["motor_keys"]["right"][:6]]
        np.testing.assert_allclose(actual, initial)
    finally:
        adapter.close()


def test_site_state_history_is_queried_at_wrist_exposure(settings):
    from real_robot.lab import ArmState

    robot = RecordingRobot()
    site = Site()
    stamps = []
    def measured_at(device, exposure):
        stamps.append(exposure)
        return {side: ArmState([0.] * 6, np.eye(4).tolist(), .02, exposure) for side in SIDES}
    site.read_state_at = measured_at
    adapter = LabAdapter(settings, robot=robot, site=site)
    try:
        images = adapter.read_images(-1)
        for frame in images.values():
            state = adapter.read_state(frame.exposure_ns)
            assert state["right"].timestamp_ns == frame.exposure_ns
        assert stamps == [frame.exposure_ns for frame in images.values()]
    finally:
        adapter.close()
