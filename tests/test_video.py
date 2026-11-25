"""Exercise the installed physical run entry point with cameras that change during motion."""

import json
import time
from functools import partial
from threading import Event
from types import SimpleNamespace

import imageio_ffmpeg
import numpy as np
import pytest

from real_robot import __main__ as cli
from real_robot import lab
from real_robot.__main__ import DATA

SIZES = {"top": (64, 48), "l_wrist": (48, 64), "r_wrist": (48, 32)}
FPS = 12


class Camera:
    def __init__(self, robot, name, channel):
        self.robot = robot
        self.size = SIZES[name]
        self.channel = channel

    def image(self):
        width, height = self.size
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        frame[..., self.channel] = self.robot.brightness
        return frame

    def read_latest(self):
        assert self.robot.connected
        if self.robot.video_fault:
            self.robot.fault_seen.set()
            raise OSError("lost video feed")
        if self.channel == 0:
            self.robot.video_samples.append(self.robot.brightness)
        return self.image()


class Device:
    def __init__(self):
        self.connected = False
        self.brightness = 55
        self.video_samples = []
        self.video_fault = False
        self.fault_seen = Event()
        self.commands = []
        self.cameras = {name: Camera(self, name, i) for i, name in enumerate(SIZES)}
        self.motors = {side: [f"{side}{i}" for i in range(7)] for side in ("left", "right")}
        self.values = {name: 0.0 for row in self.motors.values() for name in row}
        self.action_features = dict.fromkeys(self.values, float)

    def connect(self):
        assert not self.connected
        self.connected = True

    def disconnect(self):
        assert self.connected
        self.connected = False

    def show_until_recorded(self, brightness):
        self.brightness = brightness
        deadline = time.monotonic() + 4
        while self.video_samples.count(brightness) < 3:
            assert time.monotonic() < deadline, "video acquisition stalled while the task was busy"
            time.sleep(0.01)

    def get_observation(self):
        assert self.connected
        stamp = time.monotonic_ns()
        return {
            **self.values,
            **{name: camera.image() for name, camera in self.cameras.items()},
            "_embodied2code": {
                "exposure_ns": dict.fromkeys(SIZES, stamp),
                "arms": {
                    arm: {"timestamp_ns": stamp, "T_base_flange": np.eye(4)} for arm in self.motors
                },
            },
        }

    def send_action(self, command):
        assert self.connected
        self.commands.append(command)
        self.show_until_recorded(175)
        self.values.update(command)


@pytest.fixture
def station(tmp_path, monkeypatch):
    device = Device()
    cfg = json.loads((DATA / "configs/lab.example.json").read_text())
    cfg.update(
        frame="piper_base", prior_world=str(DATA / "examples/replay/prior_world"), video_fps=FPS
    )
    cfg["driver"].update(
        motor_keys=device.motors,
        state_scale=[1.0] * 7,
        action_scale=[1.0] * 7,
        site_module="mock_site",
    )
    for arm in cfg["arms"].values():
        arm.update(
            T_W_base=np.eye(4).tolist(), T_flange_tcp=np.eye(4).tolist(), max_gripper_width_m=0.07
        )
    for name, cam in cfg["cameras"].items():
        width, height = SIZES[name]
        cam.update(
            image_size=[width, height], K=[[30, 0, width / 2], [0, 30, height / 2], [0, 0, 1]]
        )
        transform = "T_W_camera" if cam["mount"] == "fixed" else "T_flange_camera"
        cam[transform] = np.eye(4).tolist()
    config = tmp_path / "site.json"
    config.write_text(json.dumps(cfg))
    monkeypatch.chdir(tmp_path)  # play_episode runs agent cells in its workspace.
    monkeypatch.setattr(lab, "make_lab_robot", lambda settings: device)
    monkeypatch.setattr(lab, "import_module", lambda name: SimpleNamespace(stop=lambda robot: None))
    return device, config


def decode(path):
    stream = imageio_ffmpeg.read_frames(str(path), pix_fmt="rgb24")
    try:
        meta = next(stream)
        width, height = meta["size"]
        means = [
            np.frombuffer(raw, np.uint8).reshape(height, width, 3).mean(axis=(0, 1))
            for raw in stream
        ]
        return meta, np.asarray(means)
    finally:
        stream.close()


def run_task(out, config, monkeypatch, transport):
    monkeypatch.setattr(
        cli,
        "play_episode",
        partial(cli.play_episode, openai_options={"transport": transport, "retries": 0}),
    )
    return cli.main(
        [
            "run",
            "--config",
            str(config),
            "--task",
            "pack",
            "--runtime",
            "openai",
            "--base-url",
            "http://unused",
            "--output",
            str(out),
        ]
    )


def response(tool, args, body):
    call = {
        "id": f"call_{len(body['messages'])}",
        "type": "function",
        "function": {"name": tool, "arguments": json.dumps(args)},
    }
    return 200, {
        "choices": [
            {"message": {"role": "assistant", "tool_calls": [call]}, "finish_reason": "tool_calls"}
        ]
    }


def test_run_records_all_cameras_during_decision_and_motion(station, monkeypatch, tmp_path):
    device, config = station
    out = tmp_path / "pack-episode"
    calls = iter(
        [
            (
                115,
                "python",
                {"code": 'world.read_context(); print(robo.set_gripper(.03, arm="right"))'},
            ),
            (
                235,
                "python",
                {
                    "code": "ctx=world.read_context(); world.commit_step("
                    "ctx['interaction']['pending_update']['capture'], expected_revision=ctx['revision'],"
                    "note='reviewed', unresolved={'item':'uncertain'})"
                },
            ),
            (None, "finish", {"status": "success", "summary": "scripted capture completed"}),
        ]
    )

    def transport(url, body, timeout):
        brightness, tool, args = next(calls)
        if brightness:
            device.show_until_recorded(brightness)
        return response(tool, args, body)

    assert run_task(out, config, monkeypatch, transport) == 0, (out / "result.json").read_text()
    assert len(device.commands) == 1 and not device.connected
    counts = []
    for channel, (name, size) in enumerate(SIZES.items()):
        meta, means = decode(out / "videos" / f"{name}.mp4")
        assert meta["size"] == size
        assert meta["fps"] == pytest.approx(FPS)
        assert np.delete(means, channel, axis=1).max() < 12
        levels = means[:, channel]
        assert abs(levels[0] - 55) < 12
        assert abs(levels[-1] - 235) < 12
        for brightness in (115, 175, 235):
            assert (np.abs(levels - brightness) < 12).sum() >= 2
        counts.append(len(means))
    assert min(counts) == max(counts)
    assert json.loads((out / "videos/recording.json").read_text())["status"] == "completed"


def test_failed_scene_setup_finalizes_playable_video(station, monkeypatch, tmp_path):
    from real_robot import host

    device, config = station
    out = tmp_path / "failed-episode"

    def fail_scene(*args, **kwargs):
        device.show_until_recorded(115)
        raise ValueError("bad site calibration")

    monkeypatch.setattr(host, "prepare_scene", fail_scene)
    assert run_task(out, config, monkeypatch, None) == 1
    assert not device.connected and not device.commands
    assert "bad site calibration" in (out / "result.json").read_text()
    for name in SIZES:
        _, frames = decode(out / "videos" / f"{name}.mp4")
        assert len(frames) >= 3


def test_camera_failure_surfaces_at_task_exit(station, monkeypatch, tmp_path):
    device, config = station
    out = tmp_path / "camera-fault"

    def transport(url, body, timeout):
        device.video_fault = True
        assert device.fault_seen.wait(timeout=4)
        return response("finish", {"status": "stuck", "summary": "done"}, body)

    assert run_task(out, config, monkeypatch, transport) == 1
    assert not device.connected
    report = json.loads((out / "videos/recording.json").read_text())
    assert report["status"] == "failed" and "lost video feed" in report["error"]
    assert not (out / "videos/top.mp4").exists()


def test_slow_camera_preserves_elapsed_time_in_mp4(tmp_path):
    from real_robot.video import VideoRecorder

    reads = []
    fourth_read = Event()

    def slow_frame():
        if reads:
            time.sleep(0.16)
        reads.append(time.monotonic())
        if len(reads) == 4:
            fourth_read.set()
        return {"top": np.full((32, 48, 3), 40 * min(5, len(reads)), np.uint8)}

    recorder = VideoRecorder(tmp_path, ["top"], slow_frame, fps=25)
    recorder.start()
    try:
        assert fourth_read.wait(timeout=4)
    finally:
        recorder.close()
    meta, means = decode(tmp_path / "videos/top.mp4")
    assert meta["fps"] == pytest.approx(25)
    duration = len(means) / meta["fps"]
    assert duration >= reads[3] - reads[0] - 1 / meta["fps"]
    assert len(means) > 4
