import json

import numpy as np
import pytest
from codeworld.robot_assets import export_robot
from codeworld.world import read
from pyrualean.play import play_episode

from real_robot import __main__ as cli
from real_robot.__main__ import DATA, demo
from real_robot.host import Cell, boot
from real_robot.scene import resolve_asset

REPLAY = DATA / "configs/replay.json"


@pytest.mark.parametrize(("flags", "expected"), [
    ([], "gpt-6-astra"),
    (["--model", "custom-model"], "custom-model"),
])
def test_run_selects_action_model(tmp_path, monkeypatch, flags, expected):
    def check_episode(cell, out, *, model, **kwargs):
        assert model == expected
        assert kwargs["boot_options"]["record_video"] is True
        (out / "videos").mkdir()
        (out / "videos/recording.json").write_text('{"status": "completed"}')
        return {"status": "completed", "error": None, "finish": {"status": "success"}}

    monkeypatch.setattr(cli, "play_episode", check_episode)
    assert cli.main(["run", "--config", str(REPLAY), "--task", "pack",
                     "--output", str(tmp_path / "episode"), *flags]) == 0


def test_demo_and_capture(tmp_path):
    demo(tmp_path)
    result = read(tmp_path / "result.json")
    assert result["environment_success"] is None
    assert result["interaction"]["updated_count"] == 1
    assert abs(result["updated_item_xyz_m"][0] - 0.36) < 0.015
    assert result["interaction"]["pending_update"] is None
    after = next(
        p
        for p in (tmp_path / "world/observations").glob("*/capture.json")
        if read(p)["metadata"]["host_step"] == 1
    )
    packet = read(after)
    assert [v["view_id"] for v in packet["views"]] == ["fixed", "wrist"]
    src = read(DATA / "examples/replay/after.json")["views"][1]
    np.testing.assert_allclose(
        packet["views"][1]["T_W_C"],
        np.asarray(src["T_base_flange_at_exposure"]) @ src["T_flange_camera"],
    )


def test_state_failure_does_not_reissue_command(tmp_path, monkeypatch):
    host, _ = boot(Cell("test", "test"), tmp_path, config_path=REPLAY)

    def uncertain(cmd, checker=None):
        if cmd["kind"] != "stop":
            raise TimeoutError("Lost acknowledgement")

    monkeypatch.setattr(host.adapter, "execute_action", uncertain)
    with pytest.raises(TimeoutError):
        host.execute_tool("grip", {"opening": 1})
    assert host.stopped
    assert read(tmp_path / "action-0001.json")["status"] == "unknown"
    with pytest.raises(RuntimeError):
        host.execute_tool("grip", {"opening": 1})
    assert host.env_steps() == 1


def test_stale_image_and_state(tmp_path, monkeypatch):
    host, _ = boot(Cell("test", "test"), tmp_path, config_path=REPLAY)
    old = host.adapter.read_images(-1)
    monkeypatch.setattr(host.adapter, "read_images", lambda after_ns: old)
    host.execute_tool("grip", {"opening": 0})
    with pytest.raises(ValueError, match="Stale"):
        host.capture_into(tmp_path / "late", refresh=True)


def test_bimanual_bundled_piper_models(tmp_path):
    lab = DATA / "configs/lab.example.json"
    cfg = read(lab)
    bases = {arm: np.eye(4) for arm in cfg["arms"]}
    bases["left"][0, 3] = 1.5
    spec = resolve_asset(cfg["robot_asset"], lab.parent, bases)
    state = {arm: {"joints": [0.0] * 6, "gripper_width_m": 0.04} for arm in bases}
    output = export_robot(spec, state, tmp_path / "piper.glb")
    import trimesh

    scene = trimesh.load(output, force="scene")
    assert 1.5 < scene.extents[0] < 2.5


def test_agent_entry_with_scripted_transport(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)  # PyRUA runs cells in the episode workspace.
    actions = iter(
        [
            ("python", {"code": "world.read_context(); print(robo.set_gripper(.03))"}),
            (
                "python",
                {
                    "code": "c=world.read_context(); world.commit_step(c['interaction']['pending_update']['capture'],expected_revision=c['revision'],note='recording reviewed',unresolved={'item':'uncertain'})"
                },
            ),
            ("finish", {"status": "stuck", "summary": "fixture playback finished"}),
        ]
    )

    from pyrualean import mcp_server

    server_class = mcp_server.ArmMcpServer

    def check_server(toolkit, **kwargs):
        assert toolkit.finish_recovery is not None
        assert kwargs["annotations"] == {
            "readOnlyHint": False, "destructiveHint": True, "openWorldHint": True
        }
        return server_class(toolkit, **kwargs)

    monkeypatch.setattr(mcp_server, "ArmMcpServer", check_server)

    def transport(url, body, timeout):
        name, args = next(actions)
        tool = {
            "id": "call_" + str(len(body["messages"])),
            "type": "function",
            "function": {"name": name, "arguments": json.dumps(args)},
        }
        return 200, {
            "choices": [
                {
                    "message": {"role": "assistant", "tool_calls": [tool]},
                    "finish_reason": "tool_calls",
                }
            ]
        }

    result = play_episode(
        Cell("pack", "Review the object transfer."),
        tmp_path / "agent",
        arm="cells",
        model="scripted",
        robot="physical",
        runtime="openai",
        base_url="http://unused",
        openai_options={"transport": transport, "retries": 0},
        boot_options={"config_path": str(REPLAY)},
        codeworld_config=str(REPLAY),
        budget_s=30,
    )
    assert result["status"] == "completed", result["error"]
    prompt = (tmp_path / "agent/prompt.txt").read_text()
    assert "an observer from the final scene" in prompt
    assert "The TCP is the grasp center" in prompt
    assert "action budget" in prompt
    assert result["finish"]["status"] == "stuck"
    assert result["codeworld"]["final_interaction"]["updated_count"] == 1
    assert result["environment_success"] is None


def test_bimanual_capture_and_control(tmp_path, monkeypatch):
    import time
    from copy import deepcopy

    from real_robot import host as host_module
    from real_robot.lab import ArmState, ImageFrame
    from real_robot.robot import PhysicalRobot

    cfg = read(DATA / "configs/lab.example.json")
    assert cfg["driver"]["robot_type"] == "bi_piper_follower"
    assert cfg["driver"]["camera_map"] == {k: k for k in ("top", "l_wrist", "r_wrist")}
    left_base = np.array([[0., -1, 0, .7], [1, 0, 0, .2], [0, 0, 1, .1], [0, 0, 0, 1]])
    flange = np.eye(4)
    flange[0, 3] = .21
    tool = np.eye(4)
    tool[2, 3] = .1
    wrist = np.eye(4)
    wrist[1, 3] = .04
    for arm, width in (("left", .07), ("right", .08)):
        cfg["arms"][arm].update(T_flange_tcp=tool.tolist(), max_gripper_width_m=width)
    cfg["arms"]["left"]["T_W_base"] = left_base.tolist()
    for cam in cfg["cameras"].values():
        cam.update(image_size=[8, 6], K=[[10, 0, 4], [0, 10, 3], [0, 0, 1]])
        if cam["mount"] == "wrist":
            cam["T_flange_camera"] = wrist.tolist()
        else:
            cam["T_W_camera"] = np.eye(4).tolist()
    # The workstation can keep measured wrist calibration outside the shared config.
    wrist_calibration = cfg["cameras"]["r_wrist"].copy()
    wrist_calibration.update(raw_K=wrist_calibration["K"], raw_distortion=[0.] * 5)
    (tmp_path / "calibration.json").write_text(json.dumps({"r_wrist": wrist_calibration}))
    cfg["cameras"]["r_wrist"] = {"mount": "wrist", "arm": "right"}
    cfg["camera_calibration_file"] = "calibration.json"
    path = tmp_path / "lab.json"
    path.write_text(json.dumps(cfg))

    class Device:
        def __init__(self, settings):
            self.commands = []
            self.missing_arm = False

        def read_images(self, after_ns):
            now = time.monotonic_ns()
            return {name: ImageFrame(np.full((6, 8, 3), i, np.uint8), now - 2 + i)
                    for i, name in enumerate(cfg["cameras"])}

        def read_state(self, stamp):
            samples = {
                "left": ArmState([.1] * 6, flange.tolist(), .02, stamp),
                "right": ArmState([.2] * 6, np.eye(4).tolist(), .03, stamp),
            }
            if self.missing_arm:
                del samples["left"]
            return samples

        def execute_action(self, command, check_path=None):
            self.commands.append(deepcopy(command))
            if check_path:
                assert check_path([{"joints_rad": [.3] * 6, "width_m": .04}])["status"] == "CLEAR"
            return {"status": "completed"}

    monkeypatch.setattr(host_module, "LabAdapter", Device)
    backend, _ = boot(Cell("pack", "pack"), tmp_path, config_path=path)
    try:
        robot = PhysicalRobot(backend)
        state = robot.state()
        np.testing.assert_allclose(state["left"]["T_W_tcp"], left_base @ flange @ tool)
        np.testing.assert_allclose(state["right"]["T_W_tcp"], tool)
        packet = backend.packets[0][1]
        view_matrices = {row["view_id"]: row["T_W_C"] for row in packet["views"]}
        np.testing.assert_allclose(view_matrices["top"], np.eye(4))
        np.testing.assert_allclose(view_matrices["l_wrist"], left_base @ flange @ wrist)
        np.testing.assert_allclose(view_matrices["r_wrist"], wrist)
        assert list(host_module.dump_card(backend, Cell("pack", "pack"), tmp_path).images) == list(cfg["cameras"])
        for i, name in enumerate(cfg["cameras"]):
            assert (robot.image(name) == i).all()
        with pytest.raises(ValueError, match="Select an arm"):
            robot.move_to([0, 0, 0])
        assert not backend.adapter.commands
        checks = []

        def collision(raw, actions, **kwargs):
            assert raw["left_arm_joint_state"] == [.1] * 6
            assert raw["right_arm_joint_state"] == [.2] * 6
            checks.append(actions)
            return {"status": "CLEAR"}

        target = np.eye(4)
        target[:3, 3] = [.4, .3, .2]
        backend.set_motion_checker(collision)
        for arm, base in (("left", left_base), ("right", np.eye(4))):
            robot.move_to(target[:3, 3], target[:3, :3], arm=arm)
            cmd = backend.adapter.commands[-1]
            assert cmd["arm"] == arm
            np.testing.assert_allclose(base @ cmd["T_base_tcp"], target, atol=1e-12)
            np.testing.assert_allclose(base @ cmd["T_base_flange"] @ tool, target, atol=1e-12)
            assert set(checks[-1][0]) == {arm + "_arm_joint_state", arm + "_ee_joint_state"}
            robot.set_gripper(.04, arm=arm)
            assert backend.adapter.commands[-1] == {"kind": "grip", "gripper_width_m": .04, "arm": arm}
        backend.adapter.missing_arm = True
        with pytest.raises(ValueError, match="every configured arm"):
            backend.acquire()
    finally:
        backend.close()
    assert backend.adapter.commands[-1] == {"kind": "stop"}


def test_refusal_feedback_and_finish_recovery(tmp_path):
    from pyrualean.arms import ArmToolkit
    from real_robot.robot import PhysicalRobot
    from real_robot.scene import prepare

    host, _ = boot(Cell("pack", "audit"), tmp_path, config_path=REPLAY)
    try:
        robot = PhysicalRobot(host)
        bindings, prompt, _ = prepare(REPLAY, robo=robot, robot="physical",
                                      out_dir=tmp_path, scratch=tmp_path / "workspace")
        world = bindings["world"]
        kit = ArmToolkit(robot, arm="cells", workspace=tmp_path / "workspace",
                         bindings=bindings, interaction=world)
        assert "world.preview_plan" in prompt and "image_points" in prompt
        assert "Calibrated RGB views: fixed, wrist" in prompt
        assert kit.execute_tool("finish", {"status": "stuck", "summary": "blocked"}).result[
            "error"] == "early_finish_requires_recovery"
        result = kit.execute_tool("python", {"code":
            "world.read_context(); print(robo.retreat([0, 0, .02])); robo.show('wrist')"}).result
        feedback = result["codeworld"]["physical_action"]
        assert feedback["moved"] is False and feedback["reached"] is False
        assert feedback["measured_T_W_tcp"] and feedback["measurement_step"] == 1
        assert not feedback["pending_update"] and not host.adapter.commands
        host.cancel()
        assert kit.execute_tool("finish", {"status": "stuck", "summary": "stopped"}).result["ok"]
    finally:
        host.close()
