"""Strict YAML boundary for the concrete P1 asset preparation inputs."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def read_mapping(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or not all(isinstance(key, str) for key in data):
        raise ValueError(f"Expected a string-keyed mapping: {path}")
    return data


def fields(data: dict[str, Any], expected: str) -> None:
    if set(data) != set(expected.split()):
        raise ValueError(f"Expected fields {expected}; received {sorted(data)}")


def positive(value: object) -> float:
    """Validate YAML integer counts and floating physical coefficients at the boundary."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"Expected a number, received {value!r}")
    number = float(value)
    if not np.isfinite(number) or number <= 0:
        raise ValueError(f"Expected a finite positive number: {number}")
    return number


def vector(value: list[float], length: int) -> tuple[float, ...]:
    if not isinstance(value, list) or any(
        isinstance(item, bool) or not isinstance(item, (int, float)) for item in value
    ):
        raise ValueError("Expected a YAML sequence of numeric components")
    result = tuple(float(item) for item in value)
    if len(result) != length or not np.isfinite(result).all():
        raise ValueError(f"Expected {length} finite vector components")
    return result


def digest(data: object) -> str:
    return hashlib.sha256(
        json.dumps(data, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


@dataclass(frozen=True)
class MaterialConfig:
    static_friction: float
    dynamic_friction: float
    restitution: float
    source: str

    def __post_init__(self) -> None:
        for friction in (self.static_friction, self.dynamic_friction):
            positive(friction)
        if (
            isinstance(self.restitution, bool)
            or not 0 <= self.restitution <= 1
            or not self.source.strip()
        ):
            raise ValueError("Restitution must be in [0,1] and provenance must be nonempty")


@dataclass(frozen=True)
class ObjectConfig:
    name: str
    source: Path
    root_prim: str
    metadata: Path
    mass_kg: float
    material: MaterialConfig
    snapshot: dict[str, Any]


def load_objects(path: Path) -> list[ObjectConfig]:
    data = read_mapping(path)
    fields(data, "objects")
    result: list[ObjectConfig] = []
    for item in data["objects"]:
        fields(item, "name source root_prim metadata physics")
        physics = item["physics"]
        fields(physics, "mass_kg static_friction dynamic_friction restitution source")
        material = MaterialConfig(**{k: v for k, v in physics.items() if k != "mass_kg"})
        result.append(
            ObjectConfig(
                str(item["name"]),
                Path(item["source"]),
                str(item["root_prim"]),
                Path(item["metadata"]),
                positive(physics["mass_kg"]),
                material,
                item,
            )
        )
    if not result or len({item.name for item in result}) != len(result):
        raise ValueError("Object manifest must have nonempty unique names")
    return result


@dataclass(frozen=True)
class CalibrationConfig:
    samples: int
    steps_per_second: int
    settle_steps: int
    position_tolerance_m: float
    velocity_tolerance_m_s: float
    contact_probe_width_m: float
    contact_probe_depth_m: float
    contact_probe_height_m: float
    contact_probe_mass_kg: float
    hold_steps: int
    minimum_contact_force_N: float
    max_probe_translation_m: float

    def __post_init__(self) -> None:
        for value in (self.samples, self.steps_per_second, self.settle_steps, self.hold_steps):
            if type(value) is not int:
                raise ValueError("Calibration counts must be integers")
        if self.samples < 3 or self.samples % 2 != 1:
            raise ValueError("Calibration needs an odd sample count >= 3")
        for value in vars(self).values():
            positive(value)


@dataclass(frozen=True)
class GripperConfig:
    name: str
    source: Path
    robot_config: Path
    root_prim: str
    base_body: str
    retained_bodies: tuple[str, ...]
    retained_joints: tuple[str, ...]
    tcp_parent_prim: str
    finger_colliders: tuple[str, ...]
    approach_axis_base: tuple[float, ...]
    opening_axis_base: tuple[float, ...]
    approach_interval_m: tuple[float, ...]
    lateral_interval_m: tuple[float, ...]
    normal_alignment_min: float
    material: MaterialConfig
    sensitivity_friction: tuple[float, ...]
    calibration: CalibrationConfig
    robot_snapshot: dict[str, Any]
    snapshot: dict[str, Any]


def load_gripper(path: Path) -> GripperConfig:
    data = read_mapping(path)
    fields(
        data,
        "name robot_config root_prim base_body retained_bodies retained_joints "
        "tcp_parent_prim finger_colliders approach_axis_base opening_axis_base "
        "contact_region material calibration",
    )
    robot_path = Path(data["robot_config"])
    robot = read_mapping(robot_path)
    tcp = robot["kinematics"]["tcp"]
    fields(tcp, "parent_frame position_m orientation_xyzw")
    vector(tcp["position_m"], 3)
    if not np.isclose(np.linalg.norm(vector(tcp["orientation_xyzw"], 4)), 1.0):
        raise ValueError("TCP quaternion must be normalized, in xyzw order")
    # These external nulls mean 'inherit USD'. No nullable values enter resolved drives.
    actuator = robot["actuators"]["gripper"]
    for key in ("stiffness", "damping", "effort_limit_sim", "velocity_limit_sim"):
        if actuator[key] is not None:
            raise ValueError(f"P1 requires the authoritative inherit-USD actuator: {key}")
    approach = vector(data["approach_axis_base"], 3)
    opening = vector(data["opening_axis_base"], 3)
    if not np.allclose([np.linalg.norm(approach), np.linalg.norm(opening)], 1.0):
        raise ValueError("Gripper axes must be unit vectors")
    if abs(np.dot(approach, opening)) > 1e-8:
        raise ValueError("Approach and opening axes must be perpendicular")
    if len(data["finger_colliders"]) != 2 or len(robot["gripper"]["command_joint_names"]) != 1:
        raise ValueError("P1 requires two fingers with one active command joint")
    if data["base_body"] not in data["retained_bodies"]:
        raise ValueError("Base body must be retained")
    if Path(data["tcp_parent_prim"]).name != tcp["parent_frame"]:
        raise ValueError("TCP parent path disagrees with robot configuration")
    region = data["contact_region"]
    fields(region, "approach_interval_m lateral_interval_m normal_alignment_min")
    for key in ("approach_interval_m", "lateral_interval_m"):
        lower, upper = vector(region[key], 2)
        if lower >= upper:
            raise ValueError("Contact intervals must increase")
    if not 0 < region["normal_alignment_min"] <= 1:
        raise ValueError("Invalid contact normal alignment")
    material = data["material"]
    return GripperConfig(
        data["name"],
        Path(robot["usd_path"]),
        robot_path,
        data["root_prim"],
        data["base_body"],
        tuple(data["retained_bodies"]),
        tuple(data["retained_joints"]),
        data["tcp_parent_prim"],
        tuple(data["finger_colliders"]),
        approach,
        opening,
        vector(region["approach_interval_m"], 2),
        vector(region["lateral_interval_m"], 2),
        region["normal_alignment_min"],
        MaterialConfig(**{k: v for k, v in material.items() if k != "sensitivity_friction"}),
        tuple(positive(v) for v in material["sensitivity_friction"]),
        CalibrationConfig(**data["calibration"]),
        robot,
        data,
    )


@dataclass(frozen=True)
class ValidationProfile:
    version: str
    solver_type: str
    external_forces_every_iteration: bool
    solve_articulation_contact_last: bool
    steps_per_second: int
    trials: int
    random_directions: int
    solver_position_iterations: int
    solver_velocity_iterations: int
    contact_buffer_per_env: int
    gpu_found_lost_pairs_capacity: int
    gpu_found_lost_aggregate_pairs_capacity: int
    gpu_total_aggregate_pairs_capacity: int
    gpu_max_rigid_contact_count: int
    approach_s: float
    close_s: float
    hold_s: float
    disturbance_s: float
    recovery_s: float
    invert_s: float
    inverted_hold_s: float
    stable_window_s: float
    contact_loss_s: float
    contact_offset_m: float
    minimum_contact_force_N: float
    max_translation_m: float
    max_rotation_rad: float
    max_linear_speed_m_s: float
    max_angular_speed_rad_s: float
    max_joint_speed_m_s: float
    joint_tolerance_m: float
    approach_translation_m: float
    approach_rotation_rad: float
    command_speed_m_s: float
    disturbance_acceleration_m_s2: float

    def __post_init__(self) -> None:
        if not self.version:
            raise ValueError("Protocol version must be nonempty")
        for name, value in vars(self).items():
            if name not in (
                "version",
                "solver_type",
                "external_forces_every_iteration",
                "solve_articulation_contact_last",
            ):
                positive(value)
        if (
            type(self.external_forces_every_iteration) is not bool
            or type(self.solve_articulation_contact_last) is not bool
        ):
            raise ValueError("Solver flags must be booleans")
        if self.solver_type not in ("TGS", "PGS"):
            raise ValueError("Expected a PhysX TGS or PGS solver")
        if self.solver_type == "PGS" and self.external_forces_every_iteration:
            raise ValueError("PGS does not support external forces on every solver iteration")
        if max(self.solver_position_iterations, self.solver_velocity_iterations) > 255:
            raise ValueError("PhysX solver iteration counts cannot exceed 255")
        for name in (
            "steps_per_second",
            "trials",
            "random_directions",
            "solver_position_iterations",
            "solver_velocity_iterations",
            "contact_buffer_per_env",
            "gpu_found_lost_pairs_capacity",
            "gpu_found_lost_aggregate_pairs_capacity",
            "gpu_total_aggregate_pairs_capacity",
            "gpu_max_rigid_contact_count",
        ):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"{name} must be an integer")
        for name in (
            "approach_s",
            "close_s",
            "hold_s",
            "disturbance_s",
            "recovery_s",
            "invert_s",
            "inverted_hold_s",
            "stable_window_s",
            "contact_loss_s",
        ):
            steps = getattr(self, name) * self.steps_per_second
            if not np.isclose(steps, round(steps)) or steps < 1:
                raise ValueError(f"{name} must contain a positive integral number of steps")
        if self.stable_window_s > min(self.close_s, self.hold_s, self.inverted_hold_s):
            raise ValueError("Stable window exceeds a holding stage")


@dataclass(frozen=True)
class SamplingConfig:
    surface_samples: int
    surface_rounds: int
    rolls_per_contact: int
    normal_alignment_min: float
    clearance_m: float
    pregrasp_distance_m: float
    collision_samples: int
    path_samples: int
    dedup_translation_m: float
    dedup_rotation_rad: float
    dedup_opening_m: float

    def __post_init__(self) -> None:
        for value in vars(self).values():
            positive(value)
        for name in (
            "surface_samples",
            "surface_rounds",
            "rolls_per_contact",
            "collision_samples",
            "path_samples",
        ):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"{name} must be an integer")
        if not 0 < self.normal_alignment_min <= 1 or self.path_samples < 2:
            raise ValueError("Invalid normal alignment or path sample count")


@dataclass(frozen=True)
class PostureConfig:
    """Upright pickup convention, applied before the deliberate holding inversion.

    Each supported gripper supplies its mounting-side up vector in the base frame.
    The object up vector is in its original root frame; initial placement preserves it.
    """

    object_up_axis: tuple[float, ...]
    wrist_up_axes_base: dict[str, tuple[float, ...]]
    max_approach_up_dot: float
    min_wrist_up_dot: float
    bottom_clearance_m: float

    def __post_init__(self) -> None:
        for axis in (self.object_up_axis, *self.wrist_up_axes_base.values()):
            if (
                len(axis) != 3
                or not np.isfinite(axis).all()
                or not np.isclose(np.linalg.norm(axis), 1)
            ):
                raise ValueError("Posture axes must be finite unit vectors")
        if not -1 <= self.max_approach_up_dot <= 0:
            raise ValueError("Pickup approach cannot point upward")
        if not 0 <= self.min_wrist_up_dot < 1:
            raise ValueError("The wrist mounting side must face upward")
        positive(self.bottom_clearance_m)


@dataclass(frozen=True)
class RunConfig:
    grippers: tuple[Path, ...]
    manifest: Path
    cache: Path
    output: Path
    device: int
    environments: int
    seed: int
    target_successes: int
    candidate_budget: int
    time_budget_s: float
    object_position_m: tuple[float, ...]
    sampling: SamplingConfig
    posture: PostureConfig
    validation: ValidationProfile
    snapshot: dict[str, Any]


def load_run(path: Path) -> RunConfig:
    data = read_mapping(path)
    fields(
        data,
        "grippers manifest cache output device environments seed target_successes "
        "candidate_budget time_budget_s object_position_m sampling posture validation",
    )
    for name in ("environments", "target_successes", "candidate_budget"):
        if type(data[name]) is not int or data[name] < 1:
            raise ValueError(f"{name} must be a positive integer")
    for name in ("device", "seed"):
        if type(data[name]) is not int or data[name] < 0:
            raise ValueError(f"{name} must be a nonnegative integer")
    if not data["grippers"] or len(set(data["grippers"])) != len(data["grippers"]):
        raise ValueError("Specify unique gripper configurations")
    posture = data["posture"]
    fields(
        posture,
        "object_up_axis wrist_up_axes_base max_approach_up_dot min_wrist_up_dot bottom_clearance_m",
    )
    return RunConfig(
        tuple(Path(p) for p in data["grippers"]),
        Path(data["manifest"]),
        Path(data["cache"]),
        Path(data["output"]),
        data["device"],
        data["environments"],
        data["seed"],
        data["target_successes"],
        data["candidate_budget"],
        positive(data["time_budget_s"]),
        vector(data["object_position_m"], 3),
        SamplingConfig(**data["sampling"]),
        PostureConfig(
            vector(posture["object_up_axis"], 3),
            {name: vector(axis, 3) for name, axis in posture["wrist_up_axes_base"].items()},
            posture["max_approach_up_dot"],
            posture["min_wrist_up_dot"],
            positive(posture["bottom_clearance_m"]),
        ),
        ValidationProfile(**data["validation"]),
        data,
    )
