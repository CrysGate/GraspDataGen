"""Numeric contracts shared by generation, validation, storage and replay."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from graspdatagen.assets import check_cache
from graspdatagen.geometry import FloatArray, IntArray


@dataclass(frozen=True)
class PreparedPair:
    gripper: Path
    object: Path
    definition: dict[str, Any]
    gripper_manifest: dict[str, Any]
    object_manifest: dict[str, Any]
    arrays: dict[str, np.ndarray]

    @classmethod
    def load(cls, gripper: Path, object: Path) -> PreparedPair:
        with np.load(gripper / "definition.npz", allow_pickle=False) as data:
            arrays = {key: data[key] for key in data.files}
        return cls(
            gripper,
            object,
            json.loads((gripper / "definition.json").read_text()),
            check_cache(gripper),
            check_cache(object),
            arrays,
        )


@dataclass(frozen=True)
class CandidateBatch:
    """Object-root/TCP column transforms; SI float64 arrays, stable int64 IDs.

    pregrasp_joints is the complete calibrated state, close_command contains only
    the active joint command. The follower is never independently driven.
    """

    ids: IntArray
    target: FloatArray
    pregrasp: FloatArray
    opening: FloatArray
    contact_width: FloatArray
    pregrasp_joints: FloatArray
    close_command: FloatArray

    def __len__(self) -> int:
        return len(self.ids)

    def select(self, indices: IntArray) -> CandidateBatch:
        return CandidateBatch(*(getattr(self, name)[indices] for name in self.__dataclass_fields__))


STAGES = ("reset", "approach", "close", "gravity_hold", "disturbance", "invert", "inverted_hold")
METRICS = (
    "min_contact_N",
    "max_translation_m",
    "max_rotation_rad",
    "max_contact_loss_s",
    "max_linear_speed_m_s",
    "max_angular_speed_rad_s",
    "max_joint_speed_m_s",
    "final_linear_speed_m_s",
    "final_angular_speed_rad_s",
    "final_joint_speed_m_s",
    "stable_contact_s",
    "max_solver_joint_speed_m_s",
    "final_solver_joint_speed_m_s",
    "max_mimic_error_m",
    "max_limit_error_m",
    "max_palm_contact_N",
    "max_solver_linear_speed_m_s",
    "max_solver_angular_speed_rad_s",
    "final_solver_linear_speed_m_s",
    "final_solver_angular_speed_rad_s",
    "steps",
)
FAILURES = (
    "passed",
    "opening_invalid",
    "approach_collision",
    "no_bilateral_contact",
    "closure_unstable",
    "gravity_slip",
    "disturbance_slip",
    "inversion_drop",
    "solver_invalid",
    "joint_constraint_violation",
)


@dataclass(frozen=True)
class ValidationBatch:
    candidates: CandidateBatch
    # (N, trials), (N, trials, stages); 0=unexecuted, 1=passed, -1=failed.
    failure: IntArray
    stage_status: IntArray
    metrics: FloatArray
    actual_tcp: FloatArray
    actual_joints: FloatArray
    initial_object: FloatArray
    seeds: np.ndarray
    accelerations: FloatArray
    # Sampled real poses for first trial: (frames, N, 4, 4), never a replay controller.
    trace: dict[str, np.ndarray]

    @property
    def passed(self) -> np.ndarray:
        return (self.failure == 0).all(axis=1) & (self.stage_status == 1).all(axis=(1, 2))
