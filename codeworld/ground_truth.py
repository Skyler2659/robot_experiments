"""Adapt native simulator surfaces to the current CodeWorld API and interaction loop."""

import copy
import fcntl
import time
from pathlib import Path

import numpy as np

from .pyrualean import AgentWorld, plain
from .storage import import_world
from .world import atomic, rigid


def packet_from(robo):
    reader = getattr(robo._backend, "ground_truth_packet", None)
    if reader is None:
        raise ValueError("GT CodeWorld needs the RoboDojo native observation adapter")
    packet = reader()
    if packet.get("schema") not in {"embodied2code-gt-v1", "worldrua-gt-v1"} or packet.get("frame") != "pyrualean_world":
        raise ValueError("Unsupported GT packet schema or coordinate frame")
    return packet


def create_world(robo, directory, mode):
    if mode not in {"live_state", "initial_geometry"}:
        raise ValueError("Unknown GT mode")
    packet = packet_from(robo)
    world = import_world(Path(packet["world"]), directory, frame="pyrualean_world")
    validate_packet(world, packet)
    state = world.state()
    for oid, row in state["objects"].items():
        row.update(
            T_W_object=rigid(packet["poses"][oid]).tolist(),
            basis="gt_initial_geometry",
            review={"pose": {"status": "prior", "revision": 0}},
        )
    state["oracle"] = {
        "mode": mode,
        "identity_binding": packet["identity_binding"],
        "initial_native_step": packet["native_step"],
    }
    atomic(world.state_path, state)
    return world


def validate_packet(world, packet):
    if packet["identity_binding"] != world.manifest.get("identity_binding"):
        raise ValueError(
            "GT identity binding changed; refusing to associate poses with other objects"
        )
    if set(packet["poses"]) != set(world.objects()):
        raise ValueError("GT pose IDs differ from the published geometry IDs")
    if packet["frame"] != world.manifest["frame"]:
        raise ValueError("GT coordinate frame changed")
    for transform in packet["poses"].values():
        rigid(transform)


class GroundTruthAgentWorld(AgentWorld):
    """Live GT uses the same queries/collision/commit API with authoritative poses."""

    def _sync(self):
        packet = packet_from(self._robo)
        validate_packet(self._world, packet)
        with (self._world.root / "state/update.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            state = self._world.state()
            previous = state.get("oracle", {})
            if packet["sequence"] < previous.get("sequence", -1) or packet[
                "native_step"
            ] < previous.get("native_step", -1):
                raise ValueError("GT observation predates the accepted native state")
            if previous.get("sequence") == packet["sequence"] and all(
                row.get("basis") == "oracle_state" and row["T_W_object"] == packet["poses"][oid]
                for oid, row in state["objects"].items()
            ):
                return state["revision"]
            from .tracking import correct_pose

            state["revision"] += 1
            robot_state = plain(self._robo.state())
            for oid, transform in packet["poses"].items():
                correct_pose(state, oid, transform, robot_state)
                row = state["objects"][oid]
                row["basis"] = "oracle_state"
                row["oracle_validation"] = {
                    "native_step": packet["native_step"],
                    "sequence": packet["sequence"],
                    "identity_binding": packet["identity_binding"],
                }
                row.setdefault("review", {})["pose"] = {
                    "status": "oracle",
                    "revision": state["revision"],
                }
                row.pop("sensor_validation", None)
                row.pop("pose_refresh", None)
            state["oracle"] = {
                k: packet[k] for k in ("native_step", "sequence", "identity_binding")
            }
            state["oracle"]["mode"] = "live_state"
            state["history"].append(
                {
                    "revision": state["revision"],
                    "kind": "oracle_state",
                    **state["oracle"],
                    "recorded_at_ns": time.time_ns(),
                }
            )
            atomic(self._world.state_path, state)
            return state["revision"]

    def _observe_raw(self):
        # WorldInteraction predicts carried motion first; replace predictions with
        # this native observation BEFORE rendering or calculating the next target.
        self._sync()
        return super()._observe_raw()

    def read_context(self):
        self._sync()
        return super().read_context()

    def _before_action(self, tool, kwargs):
        self._sync()
        return super()._before_action(tool, kwargs)

    def commit_step(
        self,
        capture,
        *,
        poses=None,
        unchanged=None,
        unresolved=None,
        held=None,
        disturbed=None,
        expected_revision,
        note,
        relations=None,
    ):
        if poses or unchanged or unresolved or disturbed:
            raise ValueError(
                "Live GT supplies poses for every object; omit pose/unchanged/unresolved/disturbed mappings"
            )
        self._loop.check_capture(capture)
        state = self._world.state()
        if state["revision"] != expected_revision:
            raise ValueError("Stale state revision; reread and recompute")
        packet = packet_from(self._robo)
        validate_packet(self._world, packet)
        if state.get("oracle", {}).get("sequence") != packet["sequence"]:
            raise ValueError("New GT observation available; observe and read_context again")
        proposals = {
            oid: {
                "T_W_object": copy.deepcopy(transform),
                "time_basis": "current",
                "translation_observed_axes": [True] * 3,
                "orientation_observed_axes": [True] * 3,
            }
            for oid, transform in packet["poses"].items()
        }
        result = super().commit_step(
            capture,
            poses=proposals,
            held=held,
            expected_revision=expected_revision,
            note=note,
            relations=relations,
        )
        result["revision"] = self._sync()
        result["basis"] = "oracle_state"
        result["metric_validation"] = True
        return result

    def _oracle_edit(self, *args, **kwargs):
        raise ValueError(
            "Live GT geometry/poses are supplied by the simulator; use world.fork for hypotheses"
        )

    update = _oracle_edit
    integrate_rgb = _oracle_edit
    update_parts = _oracle_edit
    record_feedback = _oracle_edit

    def publish_parts(self, object_id, parts, **options):
        # Contact regions may partition native faces, but must retain the exact surface.
        import trimesh

        mesh = trimesh.load(
            self._world.root / self._world.object(object_id)["geometry"]["local_mesh"],
            force="mesh",
            process=False,
        )
        faces = []
        for spec in parts.values():
            if set(spec) - {"source_faces", "T_object_part"} or "source_faces" not in spec:
                raise ValueError("GT parts must partition source_faces of the native mesh")
            if not np.allclose(spec.get("T_object_part", np.eye(4)), np.eye(4)):
                raise ValueError("GT source-face parts retain the native object frame")
            faces.extend(spec["source_faces"])
        if sorted(faces) != list(range(len(mesh.faces))):
            raise ValueError("GT parts must cover every source face exactly once")
        return super().publish_parts(object_id, parts, **options)


GT_PROMPT = """
You have `world`, a CodeWorld containing simulator-native object surfaces and current poses
in the same world frame/metres as robo. Object IDs are stable opaque keys: always obtain
IDs from world.objects()/world.object(id). Labels/native_identity describe the binding;
never construct IDs from a display name or list position. Articulated objects are split
into separately moving rigid links, identified by full native prim paths.
Use world.snapshot(), world.mesh(id), world.bounds(id), world.parts(id), world.live,
world.compute(lambda s: ...), and world.target(lambda s: ...) for ordinary Python/NumPy
geometry calculations. world.fork(name) supports hypothetical layouts. The robot's actions
and evaluator remain native PyRUA. No task solution or success flag is supplied by GT.

Read world.read_context() before EACH action. After each action the host synchronizes
ALL object/link poses from the corresponding native observation, including released or
occluded objects. Inspect the RGB and world feedback and commit before acting again:
ctx = world.read_context()
world.commit_step(ctx['interaction']['pending_update']['capture'],
                  held={...}, expected_revision=ctx['revision'], note='scene review')
held maps left/right to a known object ID or None; omit it when unchanged. Pose truth
alone is not a grasp claim. The same measured-motion grasp review as Ours is used.
Omit poses/unchanged/unresolved/disturbed: live GT owns object poses. Optional relations
are agent interpretations. Use hypotheses for planned changes; do not overwrite live GT.
To name contact regions you may publish parts that partition all source_faces exactly once
at identity T_object_part, or use object-local bounds in world.expect_contacts.
RGB can lag the native state on older clients; GT poses refer to the CURRENT native step,
while each RGB view retains its image-time camera calibration.
"""
