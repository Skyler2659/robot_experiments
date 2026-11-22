"""Hardware lifecycle shared by replay and agent episodes."""

from dataclasses import dataclass
from pathlib import Path

from codeworld.world import read
from pyrualean.prompt import TaskCard

from .backend import Backend
from .lab import LabAdapter
from .replay import ReplayAdapter
from .scene import prepare as prepare_scene  # noqa: F401 - host hook used by play_episode

INITIAL_SUCCESS = None
TOOL_ANNOTATIONS = {"readOnlyHint": False, "destructiveHint": True, "openWorldHint": True}


class PhysicalTaskCard(TaskCard):
    step_budget_label = "action budget"


@dataclass(frozen=True)
class Cell:
    task: str
    instruction: str
    suite: str = "tabletop"
    seed: int = 0
    cuda_device: int | None = None
    max_episode_steps: int = 200


def boot(cell, out_dir, *, config_path, record_video=False, **kwargs):
    path = Path(config_path).resolve()
    cfg = read(path)
    if cfg["backend"] == "replay":
        settings = dict(cfg["driver"])
        settings["records"] = [path.parent / p for p in settings["records"]]
        device = ReplayAdapter(settings)
    elif cfg["backend"] == "lab":
        device = LabAdapter(cfg.get("driver", {}))
    else:
        raise ValueError("backend must be replay or lab")
    backend = None
    try:
        backend = Backend(device, cfg, out_dir)
        backend.instruction = cell.instruction
        backend.limit = cell.max_episode_steps
        if record_video:
            backend.start_video()
        backend.capture_into(out_dir / "initial")
    except BaseException:
        try:
            if backend is not None:
                backend.finish_video()
        finally:
            close = getattr(device, "close", None)
            if close:
                close()
        raise
    return backend, []


def make_backend(toolkit):
    return toolkit


def dump_card(toolkit, cell, out_dir):
    path, packet = toolkit.packets[0]
    return PhysicalTaskCard(
        suite=cell.suite,
        task=cell.task,
        seed=cell.seed,
        robot="physical",
        task_language=cell.instruction,
        object_names=[],
        max_env_steps=cell.max_episode_steps,
        facts={
            "arms": list(toolkit.arms),
            "world frame": toolkit.frame,
            "maximum jaw opening (m)": toolkit.max_widths,
        },
        images={v["view_id"]: str(path / v["rgb_uri"]) for v in packet["views"]},
    )
