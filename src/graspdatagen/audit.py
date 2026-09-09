"""Real-asset validator counterexamples and independent physical-condition reruns."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from graspdatagen.assets import file_hash, write_json
from graspdatagen.config import ValidationProfile, fields, positive, read_mapping
from graspdatagen.geometry import FloatArray
from graspdatagen.records import FAILURES, PreparedPair
from graspdatagen.runtime import GraspScene, PhysxRuntime
from graspdatagen.storage import candidates_from_arrays, load_dataset, result_arrays, write_arrays
from graspdatagen.validation import trial_conditions, validate

CASES = (
    "empty",
    "opening_insufficient",
    "single_finger",
    "wrong_approach",
    "gravity_slip",
    "late_disturbance",
)


class GravitySlipScene(GraspScene):
    """Lose friction after closure; the shared reset discards material/contact caches."""

    def __init__(
        self,
        runtime: PhysxRuntime,
        pair: PreparedPair,
        profile: ValidationProfile,
        friction_scale: float,
    ) -> None:
        super().__init__(runtime, pair, 1, profile)
        self.slip_friction_scale: float = friction_scale

    def reset(self, object_poses: FloatArray, base_poses: FloatArray, joints: FloatArray) -> None:
        # Low friction belongs only to the previous hold, not the next scene's materials.
        self.mass_scale = 1.0
        self.friction_scale = 1.0
        super().reset(object_poses, base_poses, joints)

    def gravity(self, enabled: bool) -> None:
        self.physical_scales(1.0, self.slip_friction_scale if enabled else 1.0)
        super().gravity(enabled)


class SingleFingerScene(GraspScene):
    """Audit fixture: apply lateral load during closure with both jaws held open."""

    def __init__(
        self,
        runtime: PhysxRuntime,
        pair: PreparedPair,
        profile: ValidationProfile,
        acceleration: FloatArray,
    ) -> None:
        super().__init__(runtime, pair, 1, profile)
        self.lateral_acceleration: FloatArray = acceleration
        self.finger_peaks: FloatArray = np.zeros((profile.trials, 2))
        self.trial: int = -1  # First reset starts trial 0.

    def command(self, joints: FloatArray) -> None:
        super().command(joints)
        self.force(self.lateral_acceleration)

    def reset(self, object_poses: FloatArray, base_poses: FloatArray, joints: FloatArray) -> None:
        self.trial += 1
        super().reset(object_poses, base_poses, joints)
        self.force(np.zeros((1, 3)))

    def read(self) -> dict[str, np.ndarray]:
        state = super().read()
        self.finger_peaks[self.trial] = np.maximum(
            self.finger_peaks[self.trial], state["contact"][0, :2]
        )
        return state


def audit(
    runtime: PhysxRuntime, directory: Path, config_path: Path, output: Path, case: str
) -> dict[str, Any]:
    """case='all' includes sensitivity; a named fixture isolates a failed audit rerun."""
    if case != "all" and case not in CASES:
        raise ValueError(f"Unknown audit case: {case}")
    config = read_mapping(config_path)
    fields(
        config,
        "independent_seed late_acceleration_m_s2 slip_friction_scale "
        "single_finger_acceleration_m_s2 physical_variants",
    )
    positive(config["late_acceleration_m_s2"])
    positive(config["slip_friction_scale"])
    positive(config["single_finger_acceleration_m_s2"])
    if type(config["independent_seed"]) is not int or config["independent_seed"] < 0:
        raise ValueError("Audit seed must be a nonnegative integer")
    for variant in config["physical_variants"]:
        fields(variant, "mass_scale friction_scale")
        positive(variant["mass_scale"])
        positive(variant["friction_scale"])
    manifest, arrays = load_dataset(directory)
    from graspdatagen.geometry import pose_matrices

    initial_object = pose_matrices(arrays["pose_world_object_initial_xyz_xyzw"])
    pair = PreparedPair.load(Path(manifest["gripper_cache"]), Path(manifest["object_cache"]))
    profile = ValidationProfile(**manifest["protocol"])
    candidates = candidates_from_arrays(arrays)
    if not len(candidates):
        raise ValueError("The audit requires a successful nominal grasp")
    report: dict[str, Any] = {
        "run": str(directory),
        "run_id": manifest["run_id"],
        "implementation": {p.name: file_hash(p) for p in Path(__file__).parent.glob("*.py")},
        "config": config,
        "case": case,
        "counterexamples": [],
        "sensitivity": [],
    }
    for name in CASES if case == "all" else (case,):
        scene = (
            GravitySlipScene(runtime, pair, profile, config["slip_friction_scale"])
            if name == "gravity_slip"
            else GraspScene(runtime, pair, 1, profile)
        )
        batch = candidates.select(np.array([0], dtype=np.int64))
        acceleration = arrays["disturbance_accelerations_world_m_s2"][:1].copy()
        opening = batch.target[0, :3, :3] @ pair.arrays["opening_axis_tcp"]
        approach = batch.target[0, :3, :3] @ pair.arrays["approach_axis_tcp"]
        if name == "empty":
            shift = opening * (np.linalg.norm(batch.target[0, :3, 3]) + batch.opening[0]) * 2
            batch.target[:, :3, 3] += shift
            batch.pregrasp[:, :3, 3] += shift
        elif name == "opening_insufficient":
            batch.opening[:] = pair.arrays["opening_m"][0]
            batch.pregrasp_joints[:] = pair.arrays["joint_positions_m"][0]
        elif name == "single_finger":
            lateral = initial_object[:1, :3, :3] @ opening
            scene = SingleFingerScene(
                runtime, pair, profile, lateral * config["single_finger_acceleration_m_s2"]
            )
            batch.close_command[:] = batch.pregrasp_joints[:, scene.active_index]
        elif name == "wrong_approach":
            approach_B = pair.arrays["T_B_tcp"][:3, :3] @ pair.arrays["approach_axis_tcp"]
            shift = approach * abs(np.dot(pair.arrays["T_B_tcp"][:3, 3], approach_B))
            batch.target[:, :3, 3] += shift
            batch.pregrasp[:, :3, 3] += shift
        elif name == "late_disturbance":
            direction = initial_object[:1, :3, :3] @ approach
            acceleration[:, :, -1] = direction[:, None] * config["late_acceleration_m_s2"]
        result = validate(
            scene,
            batch,
            initial_object[:1],
            arrays["trial_seed"][:1],
            acceleration,
        )
        expected = {
            "empty": {3},
            "opening_insufficient": {1},
            "single_finger": {3, 4},
            "wrong_approach": {2},
            "gravity_slip": {5},
            "late_disturbance": {6, FAILURES.index("joint_constraint_violation")},
        }[name]
        rejected = not result.passed.any() and set(result.failure.ravel()).issubset(expected)
        if name == "late_disturbance":
            rejected = rejected and bool((result.stage_status[:, :, :4] == 1).all())
            rejected = rejected and bool((result.stage_status[:, :, 5:] == 0).all())
            preceding = (acceleration.shape[2] - 1) * round(
                (profile.disturbance_s + profile.recovery_s) * profile.steps_per_second
            )
            rejected = rejected and bool((result.metrics[:, :, 4, -1] > preceding).all())
        finger_peaks = result.trace["contact"][:, 0, :2].max(axis=0)
        if isinstance(scene, SingleFingerScene):
            rejected = rejected and bool(
                (
                    np.count_nonzero(scene.finger_peaks >= profile.minimum_contact_force_N, axis=1)
                    == 1
                ).all()
            )
            rejected = rejected and bool((result.stage_status[:, :, :2] == 1).all())
            rejected = rejected and bool((result.stage_status[:, :, 2] == -1).all())
            rejected = rejected and bool((result.stage_status[:, :, 3:] == 0).all())
        record = {
            "name": name,
            "correctly_rejected": bool(rejected),
            "trials": [FAILURES[c] for c in result.failure[0]],
            "stage_status": result.stage_status[0].tolist(),
            "metrics": result.metrics[0].tolist(),
            "peak_finger_contact_N": finger_peaks.tolist(),
        }
        if isinstance(scene, SingleFingerScene):
            record["trial_peak_finger_contact_N"] = scene.finger_peaks.tolist()
            record["lateral_acceleration_world_m_s2"] = scene.lateral_acceleration.tolist()
        report["counterexamples"].append(record)
        write_arrays(
            output.parent / (output.stem + "-" + name + ".npz"),
            {
                **result_arrays(result, np.array([0], dtype=np.int64)),
                **{
                    key: value if key == "stage" else value[:, 0]
                    for key, value in result.trace.items()
                },
            },
        )
        write_json(output.with_suffix(".progress.json"), report)
        print(f"P2 audit {name}: {record['trials']}; expected={rejected}", flush=True)
    seeds, acceleration = trial_conditions(candidates, profile, config["independent_seed"])
    variants = config["physical_variants"] if case == "all" else []
    for i, variant in enumerate(variants):
        scene = GraspScene(runtime, pair, len(candidates), profile)
        scene.physical_scales(variant["mass_scale"], variant["friction_scale"])
        result = validate(scene, candidates, initial_object, seeds, acceleration)
        report["sensitivity"].append(
            {
                **variant,
                "passed": int(result.passed.sum()),
                "total": len(candidates),
                "failure_codes": result.failure.tolist(),
            }
        )
        write_arrays(
            output.parent / (output.stem + f"-sensitivity-{i}.npz"),
            result_arrays(result, np.arange(len(candidates), dtype=np.int64)),
        )
        write_json(output.with_suffix(".progress.json"), report)
    report["checks_passed"] = all(item["correctly_rejected"] for item in report["counterexamples"])
    return report
