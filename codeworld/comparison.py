"""Calibrated, CPU geometry rendering for agent visual feedback (no simulator)."""

import hashlib
import io
import time

import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import find_objects

from .observations import load
from .world import atomic


def _clip_near(triangle, near=0.001):
    output = []
    for a, b in zip(triangle, np.roll(triangle, -1, axis=0)):
        if a[2] >= near:
            output.append(a)
        if (a[2] >= near) != (b[2] >= near):
            output.append(a + (b - a) * ((near - a[2]) / (b[2] - a[2])))
    return [np.array([output[0], output[i], output[i + 1]]) for i in range(1, len(output) - 1)]


def _check_deadline(deadline):
    if deadline is not None and time.monotonic() >= deadline:
        raise TimeoutError('CodeWorld comparison rendering exceeded its feedback budget; use current RGB')


def _geometry(snapshot, object_ids, transforms, deadline):
    geometry = []
    for oid in object_ids:
        _check_deadline(deadline)
        mesh = snapshot.mesh(oid)
        vertices = np.asarray(mesh.vertices)
        if transforms and oid in transforms:
            delta = transforms[oid] @ np.linalg.inv(snapshot.transform(oid))
            vertices = vertices @ delta[:3, :3].T + delta[:3, 3]
        seed = hashlib.sha256(oid.encode()).digest()
        color = np.array([65 + b % 150 for b in seed[:3]], float)
        geometry.append((oid, vertices, np.asarray(mesh.faces), color))
    return geometry


def _visible_triangles(triangles):
    inside = triangles[:, :, 2] >= 0.001
    front = inside.all(axis=1)
    crossing = inside.any(axis=1) & ~front
    if not crossing.any():
        return triangles[front]
    # A clipped face can produce two triangles. Preserve face order for equal-depth ties.
    output = []
    for i in np.flatnonzero(front | crossing):
        if front[i]:
            output.append(triangles[i])
        else:
            output.extend(_clip_near(triangles[i]))
    return np.asarray(output).reshape(-1, 3, 3)


def _surface_depth(a, b, c, denominator, z_vertices, x, y):
    u = ((b[..., 0] - x) * (c[..., 1] - y) - (b[..., 1] - y) * (c[..., 0] - x)) / denominator
    v = ((c[..., 0] - x) * (a[..., 1] - y) - (c[..., 1] - y) * (a[..., 0] - x)) / denominator
    t = 1 - u - v
    inside = (u >= -1e-7) & (v >= -1e-7) & (t >= -1e-7)
    inv_z = u / z_vertices[..., 0] + v / z_vertices[..., 1] + t / z_vertices[..., 2]
    z = np.divide(1., inv_z, out=np.full_like(inv_z, np.inf), where=inv_z > 0)
    return inside, z


def _draw_small_faces(lo, hi, pixels, denom, z_vertices, colors, index, canvas, depth, labels):
    width, height = (hi - lo + 1).T
    areas = width * height
    face = np.repeat(np.arange(len(areas)), areas)
    offset = np.arange(len(face)) - (np.cumsum(areas) - areas)[face]
    x = lo[face, 0] + offset % width[face]
    y = lo[face, 1] + offset // width[face]
    a, b, c = pixels[face].transpose(1, 0, 2)
    inside, z = _surface_depth(a, b, c, denom[face], z_vertices[face], x + .5, y + .5)
    hit = inside & (z < depth[y, x])
    if hit.any():
        # Stable ordering retains the first submitted face at equal pixel and depth.
        pixel_id = y[hit] * depth.shape[1] + x[hit]
        order = np.lexsort((z[hit], pixel_id))
        closest = order[np.r_[True, np.diff(pixel_id[order]) != 0]]
        dest = pixel_id[closest]
        depth.flat[dest] = z[hit][closest]
        labels.flat[dest] = index
        canvas.reshape(-1, 3)[dest] = colors[face[hit][closest]]


def _draw_triangles(triangles, k, color, index, canvas, depth, labels, deadline):
    """Rasterize bounded batches of face/pixel pairs, sharing array setup across small faces."""
    if not len(triangles):
        return
    h, w = depth.shape
    pixel = triangles @ k.T
    pixel = pixel[:, :, :2] / pixel[:, :, 2:]
    lo = np.maximum(np.floor(pixel.min(axis=1)).astype(int), [0, 0])
    hi = np.minimum(np.ceil(pixel.max(axis=1)).astype(int), [w - 1, h - 1])
    a, b, c = pixel.transpose(1, 0, 2)
    ab, ac = b - a, c - a
    denom = ab[:, 0] * ac[:, 1] - ab[:, 1] * ac[:, 0]
    use = (hi >= lo).all(axis=1) & (np.abs(denom) >= 1e-9)
    if not use.any():
        return
    lo, hi, denom, triangles = lo[use], hi[use], denom[use], triangles[use]
    pixels, z_vertices = pixel[use], triangles[:, :, 2]
    normal = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    shade = .65 + .35 * np.abs(normal[:, 2]) / np.maximum(np.linalg.norm(normal, axis=1), 1e-12)
    colors = (color * shade[:, None]).astype(np.uint8)

    # Batch small faces, but use direct 2D depth-buffer slices for large rectangles.
    # Both paths submit faces in mesh order so coplanar ties are deterministic.
    large = np.flatnonzero(np.prod(hi - lo + 1, axis=1) > 256)
    start = 0
    for i in np.append(large, len(lo)):
        if i > start:
            group = slice(start, i)
            _draw_small_faces(lo[group], hi[group], pixels[group], denom[group],
                              z_vertices[group], colors[group], index, canvas, depth, labels)
        if i == len(lo):
            break
        _check_deadline(deadline)
        x0, y0 = lo[i]
        x1, y1 = hi[i] + 1
        x = np.arange(x0, x1)[None, :] + .5
        y = np.arange(y0, y1)[:, None] + .5
        inside, z = _surface_depth(*pixels[i], denom[i], z_vertices[i], x, y)
        region = np.s_[y0:y1, x0:x1]
        hit = inside & (z < depth[region])
        depth[region][hit] = z[hit]
        labels[region][hit] = index
        canvas[region][hit] = colors[i]
        start = i + 1


def _render(view, geometry, deadline):
    w, h = view["image_size"]
    canvas = np.full((h, w, 3), 235, np.uint8)
    depth = np.full((h, w), np.inf)
    labels = np.full((h, w), -1, np.int32)
    cam = np.linalg.inv(np.asarray(view["T_W_C"], float))
    k = np.asarray(view["K"], float)
    for index, (_, vertices, faces, color) in enumerate(geometry):
        _check_deadline(deadline)
        vertices = vertices @ cam[:3, :3].T + cam[:3, 3]
        for start in range(0, len(faces), 256):
            _check_deadline(deadline)
            triangles = _visible_triangles(vertices[faces[start:start + 256]])
            _draw_triangles(triangles, k, color, index, canvas, depth, labels, deadline)
    image = Image.fromarray(canvas)
    draw = ImageDraw.Draw(image)
    regions = find_objects(labels + 1, max_label=len(geometry)) if geometry else []
    for index, region in enumerate(regions):
        if region is None:
            continue
        yy, xx = np.where(labels[region] == index)
        if len(xx) > 30:
            center = (region[1].start + int(np.median(xx)), region[0].start + int(np.median(yy)))
            draw.text(center, geometry[index][0], fill="black", stroke_width=1, stroke_fill="white")
    return image, labels


def render(snapshot, view, object_ids, *, transforms=None, deadline=None):
    """Render calibrated geometry with an OpenCV camera and perspective-correct z-buffer."""
    return _render(view, _geometry(snapshot, object_ids, transforms, deadline), deadline)


def compare(world, capture, *, object_ids=None, wall_budget_s=None):
    """Return/save one real-vs-world panel per camera; no automatic pose claim."""
    deadline = None if wall_budget_s is None else time.monotonic() + float(wall_budget_s)
    observation = load(world.root, capture)
    return _compare_loaded(world, observation, object_ids=object_ids, deadline=deadline, capture=capture)


def _compare_loaded(world, observation, *, object_ids=None, deadline=None, capture=None):
    path, packet, _ = observation
    if capture is None:
        capture = str(path.relative_to(world.root))
    snapshot = world.snapshot()
    ids = list(object_ids) if object_ids is not None else [
        oid for oid in snapshot.objects() if world.object_role(oid) != "robot_snapshot"
    ]
    overrides = {}
    from .tracking import arm_pose

    image_state = packet.get("metadata", {}).get("image_robot_state")
    if image_state:
        for arm, held in snapshot._state.get("attachments", {}).items():
            overrides[held["object_id"]] = arm_pose(image_state, arm) @ np.asarray(held["T_E_object"])
    geometry = _geometry(snapshot, ids, overrides, deadline)
    panels, originals, files = [], [], []
    for i, view in enumerate(packet["views"]):
        rgb_path = path.parent / view["rgb_uri"]
        original = rgb_path.read_bytes()
        with Image.open(io.BytesIO(original)) as rgb:
            real = rgb.convert("RGB")
        predicted, _ = _render(view, geometry, deadline)
        w, h = real.size
        panel = Image.new("RGB", (2 * w, h + 24), "white")
        panel.paste(real, (0, 24)); panel.paste(predicted, (w, 24))
        draw = ImageDraw.Draw(panel)
        draw.text((5, 5), view["view_id"] + " | REAL RGB", fill="black")
        draw.text((w + 5, 5), f"CodeWorld r{snapshot.revision} | geometry; robot omitted", fill="black")
        buffer = io.BytesIO(); panel.save(buffer, format="PNG")
        filename = f"comparison_r{snapshot.revision}_{i}.png"
        encoded = buffer.getvalue()
        (path.parent / filename).write_bytes(encoded)
        panels.append(encoded); originals.append(original)
        files.append(str((path.parent / filename).relative_to(world.root)))
    report = {"revision": snapshot.revision, "capture": capture, "files": files,
              "cameras": [v["view_id"] for v in packet["views"]],
              "objects": ids, "camera_match": "recorded_K_and_T_W_C",
              "interpretation": "Compare object pose/shape; ignore lighting and real-robot occlusion."}
    atomic(path.parent / f"comparison_r{snapshot.revision}.json", report)
    return report, {"images": panels, "originals": originals, "camera_names": [c + ".png" for c in report["cameras"]],
                    "labels": [c + "_real_and_codeworld" for c in report["cameras"]]}
