"""Compact, current scene input for a host agent; full meshes stay queryable."""

import copy

from .world import read


class SceneContext:
    def __init__(self, world):
        self.world = world
        self._bounds = {}
        self._metadata = {}

    def _object_metadata(self, oid):
        path = self.world.root / self.world.manifest["objects"][oid]
        stat = path.stat()
        version = (stat.st_dev, stat.st_ino, stat.st_mtime_ns, stat.st_ctime_ns, stat.st_size)
        cached = self._metadata.get(oid)
        if cached is None or cached[0] != version:
            cached = (version, read(path))
            self._metadata[oid] = cached
        return cached[1]

    def packet(self, *, detailed=False):
        snapshot = self.world.snapshot()
        state = snapshot._state
        objects = []
        for oid in snapshot.objects():
            meta = self._object_metadata(oid)
            if not detailed and meta.get("role") == "robot_snapshot":
                continue
            current = state["objects"][oid]
            assembly = current.get("assembly")
            # A parameter/review/evidence edit does not move any triangle.
            shape = ({pid: (part["local_mesh"], part.get("sha256"), part["T_object_part"])
                      for pid, part in assembly["parts"].items()} if assembly else
                     meta.get("geometry", {}).get("local_mesh"))
            key = (current["T_W_object"], shape)
            if oid not in self._bounds or self._bounds[oid][0] != key:
                mesh = snapshot.mesh(oid)
                self._bounds[oid] = (
                    key,
                    mesh.bounds.tolist(),
                    mesh.centroid.tolist(),
                    len(mesh.vertices),
                    len(mesh.faces),
                )
            _, bounds, centroid, vertices, faces = self._bounds[oid]
            revision = assembly["geometry_revision"] if assembly else self.world.manifest["geometry_revision"]
            declared = meta.get("declared", {})
            row = {
                "object_id": oid,
                "role": meta.get("role", "rigid_object"),
                "movable": meta.get("movable", declared.get("movable")),
                "T_W_object": current["T_W_object"],
                "bounds_W_m": bounds,
                "basis": current.get("basis"),
            }
            review = current.get("review", {})
            parameters = assembly.get("parameters", {}) if assembly else {}
            if detailed:
                parts = (list(assembly["parts"]) if assembly else
                         [n["name"] for n in meta.get("source_nodes", []) if n.get("name")])
                row.update(
                    label=meta.get("label", oid),
                    centroid_W_m=centroid,
                    geometry={
                        "local_mesh": None if assembly else meta.get("geometry", {}).get("local_mesh"),
                        "representation": "agent_assembly" if assembly else (
                            "native_mesh" if self.world.manifest.get("source") == "ground_truth"
                            else "reconstructed_mesh"),
                        "vertices": vertices, "faces": faces, "geometry_revision": revision,
                    },
                    part_count=len(parts), part_names=parts[:32], part_names_truncated=len(parts) > 32,
                    parameters=parameters, pose_refresh=current.get("pose_refresh"), review=review,
                    sensor_validation=current.get("sensor_validation"),
                    source_unknowns=declared.get("unknowns", []), source_declarations=declared,
                )
                objects.append(copy.deepcopy(row))
            else:
                row["review"] = {key: entry.get("status") for key, entry in review.items()}
                row["geometry_revision"] = revision[:12]
                if declared.get("unknowns"):
                    row["geometry_unknowns"] = declared["unknowns"]
                if current.get("pose_refresh"):
                    row["pose_refresh"] = current["pose_refresh"]
                if parameters:
                    row["parameters"] = parameters
                objects.append(rounded(row))
        return {
            "schema": "codeworld-agent-context-v1",
            "revision": snapshot.revision,
            "geometry_revision": self.world.manifest["geometry_revision"],
            "frame": snapshot.frame,
            "units": "m",
            "objects": objects,
            "attachments": rounded(copy.deepcopy(state.get("attachments", {}))),
            "grasp_review": rounded(copy.deepcopy(state.get("grasp_review", {}))),
            "relations": copy.deepcopy(state.get("relations", {})),
            "recovery_history": copy.deepcopy(state.get("recovery_history", [])),
            "pose_refresh": {oid: copy.deepcopy(row['pose_refresh']) for oid, row in
                             state['objects'].items() if row.get('pose_refresh')},
            "height_alignment": rounded({k: state["height_alignment"][k] for k in
                ("table_object_id", "target_z_m", "delta_z_m", "source")}) if state.get("height_alignment") else None,
            "geometry_accuracy_verified": False,
            "robot_geometry": "initial frozen reference; use robo.state() for live robot",
            "declarations": "source hypotheses, not live measurements",
        }


def rounded(value):
    """Six decimal places for communication only; stored metric data is unchanged."""
    if isinstance(value, float):
        return round(value, 6)
    if isinstance(value, list):
        return [rounded(v) for v in value]
    if isinstance(value, dict):
        return {k: rounded(v) for k, v in value.items()}
    return value
