"""Single-arm PiPER feedback, calibrated RGB import and offline TCP pose conversion."""

import argparse
import json
import tempfile
from copy import deepcopy
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from codeworld.observations import load, register_views
from codeworld.world import read, rigid


def read_v2_feedback(arm):
    """Read the latest feedback from an already connected piper_sdk V2 interface."""
    joints = deepcopy(arm.GetArmJointMsgs())
    pose = deepcopy(arm.GetArmEndPoseMsgs())
    gripper = deepcopy(arm.GetArmGripperMsgs())
    stamps = {
        name: msg.time_stamp
        for name, msg in (("joints", joints), ("flange", pose), ("gripper", gripper))
    }
    if any(not np.isfinite(t) or t <= 0 for t in stamps.values()):
        raise ValueError("Waiting for joint, end-pose and gripper feedback")
    joint_values = [getattr(joints.joint_state, f"joint_{i}") for i in range(1, 7)]
    xyz = [getattr(pose.end_pose, axis + "_axis") for axis in ("X", "Y", "Z")]
    rpy = [getattr(pose.end_pose, axis + "_axis") for axis in ("RX", "RY", "RZ")]
    flange = np.eye(4)
    flange[:3, 3] = np.asarray(xyz) * 1e-6  # SDK reports 0.001 mm.
    flange[:3, :3] = Rotation.from_euler("xyz", np.asarray(rpy) / 1000, degrees=True).as_matrix()
    return {
        "q_rad": np.deg2rad(np.asarray(joint_values) / 1000).tolist(),
        "gripper_width_m": gripper.gripper_state.grippers_angle * 1e-6,
        "T_base_flange": flange.tolist(),
        "sdk_timestamps_s": stamps,
    }


def flange_target(T_base_tcp, T_flange_tcp):
    """Return the flange pose for a requested TCP pose, with no motion command."""
    return rigid(T_base_tcp) @ np.linalg.inv(rigid(T_flange_tcp))


def import_capture(manifest, root):
    """Import one synchronized observation. Paths in the manifest are relative to it."""
    manifest = Path(manifest)
    record = read(manifest)
    if record["source"] not in {"synthetic", "recorded"}:
        raise ValueError("Set source to synthetic or recorded")
    state = record["robot_state"]
    rigid(state["T_base_flange"])
    joints = np.asarray(state["q_rad"], dtype=float)
    if joints.shape != (6,) or not np.isfinite(joints).all():
        raise ValueError("Expected six finite PiPER joint angles in radians")
    width = state["gripper_width_m"]
    if not np.isfinite(width) or width < 0:
        raise ValueError("Expected a non-negative gripper opening in meters")
    views = []
    for entry in record["views"]:
        mount = entry["mount"]
        if mount == "fixed":
            extrinsic = rigid(entry["T_base_camera"])
        elif mount == "wrist":
            extrinsic = rigid(entry["T_base_flange_at_exposure"]) @ rigid(entry["T_flange_camera"])
        else:
            raise ValueError(f"Unknown camera mount: {mount}")
        timestamp = entry["exposure_ns"]
        if type(timestamp) is not int or timestamp < 0:
            raise ValueError("Expected an exposure timestamp in nanoseconds")
        with Image.open(manifest.parent / entry["rgb"]) as image:
            if list(image.size) != entry["image_size"]:
                raise ValueError(f"Calibration size differs from image size for {entry['id']}")
            views.append(
                {
                    "view_id": entry["id"],
                    "image": image.convert("RGB"),
                    "K": entry["K"],
                    "T_W_C": extrinsic,
                    "timing": {"exposure_ns": timestamp},
                }
            )
    return register_views(
        root,
        views,
        frame="piper_base",
        robot_state=state,
        metadata={"source": record["source"], "clock": record["clock"]},
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    capture = commands.add_parser("capture", help="import a synchronized recording")
    capture.add_argument("manifest", type=Path)
    capture.add_argument("root", type=Path, help="observation output directory")
    preview = commands.add_parser("preview", help="convert a TCP target into a flange pose")
    preview.add_argument("poses", type=Path, help="JSON with T_base_tcp and T_flange_tcp")
    replay = commands.add_parser("replay", help="read back an ordered set of offline observations")
    replay.add_argument(
        "manifests", type=Path, nargs="+", help="normalized capture manifests in acquisition order"
    )
    replay.add_argument("--output", type=Path, default=Path("runs/piper-replay"))
    replay.add_argument(
        "--flange-preview", type=Path, help="optional TCP target and tool offset JSON"
    )
    args = parser.parse_args()
    if args.command == "capture":
        packet = import_capture(args.manifest, args.root)
        path, _, _ = load(args.root, packet["capture"])
        print(path)
    elif args.command == "preview":
        poses = read(args.poses)
        target = flange_target(poses["T_base_tcp"], poses["T_flange_tcp"])
        print(
            json.dumps({"frame": "piper_base", "T_base_flange_target": target.tolist()}, indent=2)
        )
    else:
        args.output.mkdir(parents=True, exist_ok=True)
        # Publish a run only after every capture has passed the normal CodeWorld reader.
        with tempfile.TemporaryDirectory(dir=args.output) as staging:
            run = Path(staging) / "episode"
            captures = []
            for manifest in args.manifests:
                result = import_capture(manifest, run)
                _, packet, _ = load(run, result["capture"])
                captures.append(
                    {
                        "manifest": str(manifest.resolve()),
                        "source": packet["metadata"]["source"],
                        "capture": result["capture"],
                        "views": [v["view_id"] for v in packet["views"]],
                    }
                )
            summary = {"mode": "offline_replay", "captures": captures}
            if args.flange_preview:
                poses = read(args.flange_preview)
                summary["T_base_flange_target"] = flange_target(
                    poses["T_base_tcp"], poses["T_flange_tcp"]
                ).tolist()
            (run / "replay.json").write_text(json.dumps(summary, indent=2) + "\n")
            destination = args.output / result["observation_id"]
            run.rename(destination)
        print((destination / "replay.json").resolve())


if __name__ == "__main__":
    main()
