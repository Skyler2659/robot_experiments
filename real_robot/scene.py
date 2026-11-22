"""Prepare calibrated RGB CodeWorld without the simulator's robot, table or fixtures."""

import json
from pathlib import Path

from codeworld.alignment import align_table_height, table_object
from codeworld.fixtures import add_fixed_obstacles
from codeworld.pyrualean import AgentWorld
from codeworld.reconstruction import reconstruct
from codeworld.robot_assets import export_robot
from codeworld.storage import import_world
from codeworld.world import atomic, read


def resolve_asset(spec, base, bases=None):
    resolved = {
        **spec,
        "urdf": str((base / spec["urdf"]).resolve()),
        "packages": {k: str((base / v).resolve()) for k, v in spec.get("packages", {}).items()},
    }
    if bases is not None and "instances" in spec:
        instances = []
        for row in spec["instances"]:
            if "arm" in row:
                row = {**row, "T_W_base": bases[row["arm"]].tolist()}
            instances.append(row)
        resolved["instances"] = instances
    return resolved


def prepare(config_path, *, robo, robot, out_dir, scratch, cancelled=None):
    base = Path(config_path).resolve().parent
    cfg = read(config_path)
    backend = robo._backend
    initial = backend.capture_into(out_dir / "input")
    if "prior_world" in cfg:
        world = import_world(base / cfg["prior_world"], out_dir / "world", frame=backend.frame)
        mode = "imported_prior"
    else:
        asset = resolve_asset(cfg["robot_asset"], base, backend.bases)
        robot_glb = export_robot(asset, initial["robot_state"], out_dir / "known_robot.glb")
        recon = dict(cfg["reconstruction"])
        for key in ("runtime", "codex_home", "codex_config"):
            if recon.get(key):
                recon[key] = str((base / Path(recon[key]).expanduser()).resolve())
        recon.setdefault("view_ids", list(backend.cameras))
        world = reconstruct(
            out_dir / "input",
            initial["capture"],
            robot_glb,
            out_dir / "reconstruction",
            recon,
            cancelled=cancelled,
        )
        mode = "fresh_calibrated_rgb"
    if cfg.get("table_height_m") is not None:
        align_table_height(
            world,
            table_object_id=table_object(world, cfg.get("table_id")),
            target_z_m=cfg["table_height_m"],
            source="lab_measurement",
        )
    add_fixed_obstacles(world, cfg.get("fixed_obstacles", []))
    api = AgentWorld(world, robo, scratch,
                     capture_observation=lambda root: backend.capture_into(root, refresh=True))
    api._perception_mode = "codeworld"
    if cfg.get("collision"):
        spec = resolve_asset(cfg["collision"]["robot_spec"], base)
        spec.setdefault("bases", {name: matrix.tolist() for name, matrix in backend.bases.items()})
        if set(spec["bases"]) != set(backend.arms):
            raise ValueError("Collision model must match configured arms")
        api._enable_collision({**cfg["collision"], "robot_spec": spec})
    state = world.state()
    state["control_policy"] = {"observation_consistency": True, "version": "observation-control-v1"}
    atomic(world.state_path, state)
    cap = backend.capture_into(world.root)
    record = {
        "mode": mode,
        "frame": backend.frame,
        "world": str(world.root),
        "initial_capture": cap["capture"],
        "backend": cfg["backend"],
        "scene_collision_checker": bool(api._collision),
    }
    atomic(out_dir / "integration.json", record)
    prompt = "\nInitial CodeWorld:\n" + json.dumps(api.read_context(), ensure_ascii=False)
    return {"world": api}, prompt, record
