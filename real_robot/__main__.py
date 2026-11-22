"""Launch the recorded integration check or a physical CodeWorld agent episode."""

import argparse
import json
import tempfile
from pathlib import Path

from codeworld.world import atomic, read
from pyrualean.arms import ArmToolkit
from pyrualean.play import play_episode

from .host import Cell, boot
from .robot import PhysicalRobot
from .scene import prepare

DATA = Path(__file__).parent / "data"


def demo(output):
    cfg = DATA / "configs/replay.json"
    cell = Cell("replay", "Verify capture, world query and action feedback using recorded frames.")
    host, _ = boot(cell, output, config_path=cfg)
    try:
        robot = PhysicalRobot(host)
        bindings, _, _ = prepare(
            cfg, robo=robot, robot="physical", out_dir=output, scratch=output / "workspace"
        )
        world = bindings["world"]
        toolkit = ArmToolkit(
            robot, arm="cells", workspace=output / "workspace", bindings=bindings, interaction=world
        )
        cells = [
            (
                "ctx = world.read_context()\ncenter = world.snapshot().position('item')\n"
                "print(robo.move_to(center + np.array([0,0,.12])))"
            ),
            """ctx = world.read_context()
cap = ctx['interaction']['pending_update']['capture']
pixels = {}
for name in ('fixed', 'wrist'):
    rgb = robo.image(name)
    mask = (rgb == [173,69,45]).all(axis=2) | (rgb == [251,205,126]).all(axis=2)
    rows, cols = np.where(mask)
    pixels[name] = [(cols.min()+cols.max())/2, (rows.min()+rows.max())/2]
proposal = {'image_points': {'mode':'translation','points':[{'local_xyz':[0,0,0], 'pixels':pixels}]}}
report = world.commit_step(cap, expected_revision=ctx['revision'], poses={'item':proposal},
                           note='Triangulated the center of the synthetic colored patch')
assert not report['fit_failed'], report
""",
        ]
        for code in cells:
            result = toolkit.execute_tool("python", {"code": code}).result
            if result.get("error") or result.get("codeworld_error"):
                raise RuntimeError(str(result))
        world._before_finish()
        status = world._loop.status()
        if (status["action_count"], status["updated_count"]) != (1, 1):
            raise RuntimeError(f"Incomplete replay: {status}")
        atomic(
            output / "result.json",
            {
                "mode": "synthetic_replay",
                "environment_success": None,
                "interaction": status,
                "commands": host.adapter.commands,
                "updated_item_xyz_m": world.snapshot().position("item").tolist(),
                "tool_turns": toolkit.turns,
            },
        )
    finally:
        host.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest="cmd", required=True)
    info = subs.add_parser("driver-info", help="Inspect the workstation LeRobot feature and camera names")
    info.add_argument("--config", required=True, type=Path)
    info.add_argument("--sample", action="store_true", help="connect and read one raw observation")
    demo_cmd = subs.add_parser("demo")
    demo_cmd.add_argument("--output", type=Path)
    for mode in ("capture", "run"):
        cmd = subs.add_parser(mode)
        cmd.add_argument("--config", required=True, type=Path)
        cmd.add_argument("--output", type=Path)
        if mode == "run":
            cmd.add_argument("--task", choices=("insert", "pack", "tower"), required=True)
            cmd.add_argument("--instruction", help="override the task text for the actual objects")
            cmd.add_argument("--model", default="gpt-6-astra", help="action policy model (default: %(default)s)")
            cmd.add_argument("--runtime", choices=("codex", "openai"), default="codex")
            cmd.add_argument("--base-url")
            cmd.add_argument("--api-key-env")
            cmd.add_argument("--budget-s", type=float, default=5400)
            cmd.add_argument("--max-turns", type=int, default=300)
    args = parser.parse_args(argv)
    if args.cmd == "driver-info":
        from .lab import driver_info

        print(json.dumps(driver_info(read(args.config)["driver"], sample=args.sample), indent=2))
        return 0
    if args.output is None:
        parent = Path("runs").resolve()
        parent.mkdir(exist_ok=True)
        out = Path(tempfile.mkdtemp(prefix=args.cmd + "-", dir=parent))
    else:
        out = args.output.resolve()
        out.mkdir(parents=True, exist_ok=False)
    if args.cmd == "demo":
        demo(out)
    elif args.cmd == "capture":
        host, _ = boot(
            Cell("capture", "Record a synchronized sample"), out, config_path=args.config.resolve()
        )
        # Observation-only mode does not issue a robot command.
        try:
            folder, _ = host.packets[0]
            print(folder / "capture.json")
        finally:
            close = getattr(host.adapter, "close", None)
            if close:
                close()
        return 0
    else:
        path = args.config.resolve()
        task = read(DATA / "configs/tasks.json")[args.task]
        result = play_episode(
            Cell(args.task, args.instruction or task),
            out,
            arm="cells",
            robot="physical",
            boot_options={"config_path": str(path), "record_video": True},
            codeworld_config=str(path),
            model=args.model,
            runtime=args.runtime,
            base_url=args.base_url,
            api_key_env=args.api_key_env,
            budget_s=args.budget_s,
            max_turns=args.max_turns,
        )
        report_path = out / "videos/recording.json"
        if report_path.is_file():
            result["recording"] = read(report_path)
        else:
            result["recording"] = {"status": "unavailable", "error": "No finalized recording report"}
        if result["status"] == "completed" and result["recording"]["status"] != "completed":
            result["status"] = "recording_error"
            result["error"] = {"type": "RecordingError", "message": result["recording"]["error"]}
        atomic(out / "result.json", result)
        print(
            json.dumps({"output": str(out), "status": result["status"], "error": result["error"]})
        )
        return 0 if result["status"] == "completed" and result.get("finish") else 1
    print(out / "result.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
