"""Read RGB and both arms from the same LeRobot observation.

The stock deployment client sends pixel and motor arrays only. Acquisition may
augment the original observation with an "_embodied2code" mapping or provide
site.observation_metadata(robot, observation). Its fields are:

    exposure_ns: {device_camera_name: host_monotonic_exposure_ns}
    arms: {side: {timestamp_ns: host_monotonic_sample_ns, T_base_flange: 4x4}}

Arm timestamps refer to the motor feedback in this same observation. Flanges
can also be derived from those joints using an explicitly selected FK model.
"""

import math
import time
from collections.abc import Mapping
from numbers import Integral

import numpy as np
from codeworld.world import rigid

from .lab import ArmState, ImageFrame


def _stamp(mapping, key, label):
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
        raise ValueError(f"Missing or invalid {label}: expected host monotonic nanoseconds")
    value = int(value)
    if value > time.monotonic_ns():
        raise ValueError(f"{label} is in the future on the host monotonic clock")
    return value


class LeRobotObservation:
    def __init__(self, robot, settings, site=None, *, layout, flange_fk=None):
        self.robot = robot
        self.site = site
        self.layout = layout
        self.flange_fk = flange_fk
        self.camera_map = settings.get("camera_map") or {k: k for k in settings["cameras"]}
        if not self.camera_map or len(set(self.camera_map.values())) != len(self.camera_map):
            raise ValueError("Configure a distinct output name for each selected camera")
        if set(self.camera_map) - set(robot.cameras):
            raise ValueError("camera_map contains cameras absent from the LeRobot driver")
        self.timeout_s = float(settings.get("capture_timeout_s", 2.0))
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError("capture_timeout_s must be positive and finite")
        self._states = {}
        self._exposures = set()

    def _read_bundle(self):
        obs = self.robot.get_observation()
        metadata_reader = getattr(self.site, "observation_metadata", None)
        meta = metadata_reader(self.robot, obs) if metadata_reader else obs.get("_embodied2code")
        if meta is None or (isinstance(meta, Mapping) and "exposure_ns" not in meta):
            raise ValueError(
                "Missing exposure_ns. Provide capture-time metadata in observation['_embodied2code'] "
                "or site.observation_metadata(robot, observation)."
            )
        if not isinstance(meta, Mapping) or not isinstance(meta["exposure_ns"], Mapping):
            raise TypeError("Capture metadata and exposure_ns must be mappings")
        exposures = {
            src: _stamp(meta["exposure_ns"], src, f"{src} exposure_ns") for src in self.camera_map
        }
        return obs, meta, exposures

    def read_images(self, after_ns):
        self._exposures.clear()
        deadline = time.monotonic() + self.timeout_s
        while True:
            obs, meta, exposures = self._read_bundle()
            if min(exposures.values()) > after_ns:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("No new LeRobot exposure from every configured camera")
            time.sleep(0.005)

        measured = self.layout.measure(obs)
        arms = meta.get("arms")
        if not isinstance(arms, Mapping) or set(arms) != set(measured):
            raise ValueError("Capture metadata must contain matching arm samples for each side")
        states = {}
        for side, values in measured.items():
            row = arms[side]
            if not isinstance(row, Mapping):
                raise TypeError(f"Expected a capture metadata mapping for {side}")
            timestamp = _stamp(row, "timestamp_ns", f"{side} state timestamp_ns")
            joints = values[:6].tolist()
            flange = row.get("T_base_flange")
            if flange is None:
                if self.flange_fk is None:
                    raise ValueError(f"Missing {side}.T_base_flange or configured flange FK")
                flange = self.flange_fk(joints)
            states[side] = ArmState(joints, rigid(flange).tolist(), float(values[6]), timestamp)

        rectify = getattr(self.site, "rectify_image", None)
        images = {}
        for src, dst in self.camera_map.items():
            rgb = np.asarray(obs[src])
            if rectify is not None:
                rgb = np.asarray(rectify(src, rgb))
            if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
                raise ValueError(f"{src} must supply a rectified HWC uint8 RGB image")
            images[dst] = ImageFrame(rgb.copy(), exposures[src])
        self._states = states
        self._exposures = set(exposures.values())
        return images

    def read_state(self, at_ns):
        if at_ns not in self._exposures:
            raise ValueError("Requested exposure does not belong to the current LeRobot observation")
        # Backend checks each measured timestamp against the requested exposure.
        return self._states
