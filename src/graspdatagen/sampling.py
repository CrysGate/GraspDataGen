"""Area-weighted opposing surface contacts and calibrated collision-aware approach."""

from __future__ import annotations

import time
from dataclasses import asdict
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from graspdatagen.assets import file_hash
from graspdatagen.config import PostureConfig, SamplingConfig, digest
from graspdatagen.geometry import FloatArray, SurfaceQueries, pose_matrices, transform_points
from graspdatagen.records import CandidateBatch, PreparedPair


def gripper_meshes(pair: PreparedPair, joints: FloatArray) -> list[trimesh.Trimesh]:
    """Reconstruct calibrated link translations for the supported prismatic mechanism."""
    measured = pair.definition["calibration"]
    extraction = pair.definition["extraction"]
    active = measured["active_index"]
    commands = pair.arrays["joint_positions_m"][:, active]
    order = np.argsort(commands)
    poses = np.asarray(measured["link_poses_xyzw"])
    measured_order = np.argsort(measured["positions_m"], axis=0)[:, active]
    links = measured["link_names"]
    base_index = links.index(extraction["base_body"])
    transforms: dict[str, FloatArray] = {}
    for i, name in enumerate(links):
        relative = np.linalg.inv(pose_matrices(poses[:, base_index])) @ pose_matrices(poses[:, i])
        if not np.allclose(relative[:, :3, :3], relative[0, :3, :3], atol=1e-5):
            raise ValueError("Sampler requires the calibrated parallel prismatic mechanism")
        transform = relative[0].copy()
        transform[:3, 3] = [
            np.interp(joints[active], commands[order], relative[measured_order, k, 3])
            for k in range(3)
        ]
        transforms[name] = transform @ np.linalg.inv(extraction["source_body_transforms_B"][name])
    meshes: list[trimesh.Trimesh] = []
    with np.load(pair.gripper / "collision_geometry.npz", allow_pickle=False) as geometry:
        for i, path in enumerate(extraction["source_collider_order"]):
            points = transform_points(
                geometry[f"mesh_{i:03d}_vertices_B_m"], transforms[path.split("/")[2]]
            )
            meshes.append(trimesh.Trimesh(points, geometry[f"mesh_{i:03d}_faces"], process=False))
    return meshes


def distinct_indices(poses: FloatArray, openings: FloatArray, config: SamplingConfig) -> np.ndarray:
    accepted: list[int] = []
    rotations = Rotation.from_matrix(poses[:, :3, :3]).as_quat() if len(poses) else []
    for i in range(len(poses)):
        previous = np.asarray(accepted, dtype=np.int64)
        nearby = previous[
            (
                np.linalg.norm(poses[previous, :3, 3] - poses[i, :3, 3], axis=1)
                < config.dedup_translation_m
            )
            & (np.abs(openings[previous] - openings[i]) < config.dedup_opening_m)
        ]
        if len(nearby):
            dots = np.abs(np.asarray(rotations)[nearby] @ rotations[i])
            if (2 * np.arccos(np.clip(dots, 0, 1)) < config.dedup_rotation_rad).any():
                continue
        accepted.append(i)
    return np.asarray(accepted, dtype=np.int64)


def posture_mask(poses: FloatArray, pair: PreparedPair, config: PostureConfig) -> np.ndarray:
    rotation = poses[..., :3, :3]
    approach = rotation @ pair.arrays["approach_axis_tcp"]
    up_tcp = pair.arrays["T_B_tcp"][:3, :3].T @ np.asarray(
        config.wrist_up_axes_base[pair.gripper_manifest["name"]]
    )
    wrist = rotation @ up_tcp
    return (approach @ config.object_up_axis <= config.max_approach_up_dot) & (
        wrist @ config.object_up_axis >= config.min_wrist_up_dot
    )


class Sampler:
    def __init__(
        self,
        pair: PreparedPair,
        config: SamplingConfig,
        posture: PostureConfig,
        seed: int,
        device: str,
        round_index: int,
        cursor: int,
    ) -> None:
        self.pair: PreparedPair = pair
        self.config: SamplingConfig = config
        self.seed: int = seed
        self.posture: PostureConfig = posture
        self.round_index: int = round_index
        self.identity: str = digest(
            {
                "gripper": pair.gripper.name,
                "object": pair.object.name,
                "sampling": asdict(config),
                "posture": asdict(posture),
                "seed": seed,
                "implementation": {
                    name: file_hash(Path(__file__).with_name(name))
                    for name in ("sampling.py", "geometry.py", "kernels.py")
                },
            }
        )
        with np.load(pair.object / "geometry.npz", allow_pickle=False) as data:
            self.surface: trimesh.Trimesh = trimesh.Trimesh(
                data["surface_vertices_m"], data["surface_faces"], process=False
            )
            hulls = [
                trimesh.Trimesh(data[key], data[key.replace("vertices_m", "faces")], process=False)
                for key in data.files
                if key.startswith("hull_") and key.endswith("vertices_m")
            ]
        self.source: SurfaceQueries = SurfaceQueries(self.surface, device)
        self.proxy: SurfaceQueries = SurfaceQueries(trimesh.util.concatenate(hulls), device)
        self.radius: float = float(np.linalg.norm(self.surface.extents))
        self.bottom: float = float((self.surface.vertices @ posture.object_up_axis).min())
        self.samples: list[FloatArray] = []
        for joints in pair.arrays["joint_positions_m"]:
            mesh = trimesh.util.concatenate(gripper_meshes(pair, joints))
            points, _ = trimesh.sample.sample_surface(mesh, config.collision_samples, seed=seed)
            self.samples.append(np.vstack((mesh.vertices, points)))
        self.raw: CandidateBatch = self._contacts()
        self.cursor: int = cursor
        self.rejected: int = 0
        self.consumed: int = 0
        self.posture_rejected: int = 0

    def _contacts(self) -> CandidateBatch:
        c = self.config
        seed = int(digest([self.seed, self.round_index])[:16], 16)
        points, faces = trimesh.sample.sample_surface(self.surface, c.surface_samples, seed=seed)
        normals = self.surface.face_normals[faces]
        epsilon = self.radius * 1e-5
        hit = self.source.rays(points - epsilon * normals, -normals, self.radius)
        widths = hit[:, 0] + epsilon
        valid = (
            (hit[:, 4] > 0)
            & (widths > self.pair.arrays["opening_m"][0])
            & (widths + 2 * c.clearance_m < self.pair.arrays["opening_m"][-1])
            & (np.sum(hit[:, 1:4] * normals, axis=1) < -c.normal_alignment_min)
        )
        indices = np.repeat(np.flatnonzero(valid), c.rolls_per_contact)
        n = normals[indices]
        centers = points[indices] - 0.5 * widths[indices, None] * n
        tangent = np.cross(n, np.eye(3)[np.abs(n).argmin(axis=1)])
        tangent /= np.linalg.norm(tangent, axis=1)[:, None]
        phase = np.random.default_rng(seed).uniform(0, 2 * np.pi, c.surface_samples)
        angles = phase[indices] + np.tile(np.arange(c.rolls_per_contact), valid.sum()) * (
            2 * np.pi / c.rolls_per_contact
        )
        approach = np.cos(angles[:, None]) * tangent + np.sin(angles[:, None]) * np.cross(
            n, tangent
        )
        tcp = self.pair.arrays["T_B_tcp"]
        approach_B = tcp[:3, :3] @ self.pair.arrays["approach_axis_tcp"]
        opening_B = tcp[:3, :3] @ self.pair.arrays["opening_axis_tcp"]
        basis_B = np.column_stack((approach_B, opening_B, np.cross(approach_B, opening_B)))
        rotations = np.stack((approach, n, np.cross(approach, n)), axis=-1) @ basis_B.T
        # Aperture extrema lie at the distal tip on these tapered fingers.
        # Locate the grasp in the contact patch interior, using its actual area.
        patch_centers = []
        for side in range(2):
            patch = trimesh.Trimesh(
                self.pair.arrays[f"contact_patch_{side}_vertices_body_m"],
                self.pair.arrays[f"contact_patch_{side}_faces"],
                process=False,
            )
            transforms = self.pair.arrays["T_B_fingers"][:, side]
            patch_centers.append(
                np.einsum("nij,j->ni", transforms[:, :3, :3], patch.centroid) + transforms[:, :3, 3]
            )
        contact_center = np.mean(patch_centers, axis=0)
        offset = np.column_stack(
            [
                np.interp(widths[indices], self.pair.arrays["opening_m"], contact_center[:, k])
                for k in range(3)
            ]
        )
        base = np.tile(np.eye(4), (len(indices), 1, 1))
        base[:, :3, :3] = rotations
        base[:, :3, 3] = centers - np.einsum("nij,nj->ni", rotations, offset)
        target = base @ tcp
        pregrasp = target.copy()
        pregrasp[:, :3, 3] -= c.pregrasp_distance_m * approach
        ordinal = indices * c.rolls_per_contact + np.tile(
            np.arange(c.rolls_per_contact), valid.sum()
        )
        ids = np.array(
            [int(digest([self.identity, self.round_index, int(i)])[:15], 16) + 1 for i in ordinal],
            dtype=np.int64,
        )
        batch = CandidateBatch(
            ids,
            target,
            pregrasp,
            np.full(len(ids), self.pair.arrays["opening_m"][-1]),
            widths[indices],
            np.tile(self.pair.arrays["joint_positions_m"][-1], (len(ids), 1)),
            np.full(len(ids), self.pair.definition["closed_command_m"]),
        )
        # Shorter COM lever arms reduce the contact torque needed during holds.
        com = np.asarray(self.pair.object_manifest["physical"]["com_pose_xyzw"][:3])
        return batch.select(np.argsort(np.linalg.norm(centers - com, axis=1), kind="stable"))

    def take(self, count: int, deadline: float) -> CandidateBatch:
        """Choose the widest calibrated opening whose entire sampled approach is clear.

        Geometric filtering is conservative screening, not a physical success.
        PhysX checks exact collisions, natural closure and all holding stages.
        """
        accepted: list[CandidateBatch] = []
        total = 0
        c = self.config
        while total < count and time.monotonic() < deadline:
            if self.cursor == len(self.raw):
                if self.round_index + 1 == c.surface_rounds:
                    break
                self.round_index += 1
                self.cursor = 0
                self.raw = self._contacts()
                continue
            batch = self.raw.select(
                np.arange(
                    self.cursor, min(self.cursor + count - total, len(self.raw)), dtype=np.int64
                )
            )
            self.cursor += len(batch)
            self.consumed += len(batch)
            upright = posture_mask(batch.target, self.pair, self.posture)
            self.posture_rejected += int((~upright).sum())
            batch = batch.select(np.flatnonzero(upright))
            base = batch.target @ np.linalg.inv(self.pair.arrays["T_B_tcp"])
            displacement = batch.pregrasp[:, :3, 3] - batch.target[:, :3, 3]
            chosen = np.full(len(batch), -1, dtype=np.int64)
            for state in reversed(range(len(self.samples))):
                pending = np.flatnonzero(
                    (chosen < 0)
                    & (
                        batch.contact_width + 2 * c.clearance_m
                        < self.pair.arrays["opening_m"][state]
                    )
                )
                if not len(pending):
                    continue
                sample = self.samples[state]
                clear = np.ones(len(pending), dtype=np.bool_)
                for fraction in np.linspace(0, 1, c.path_samples):
                    world = np.einsum("nij,pj->npi", base[pending, :3, :3], sample)
                    world += (base[pending, :3, 3] + displacement[pending] * fraction)[:, None, :]
                    clear &= (world @ self.posture.object_up_axis).min(axis=1) >= (
                        self.bottom + self.posture.bottom_clearance_m
                    )
                    distance = self.proxy.distances(world.reshape(-1, 3), self.radius + 1)
                    clear &= distance.reshape(len(pending), -1).min(axis=1) > c.clearance_m
                    if not clear.any():
                        break
                chosen[pending[clear]] = state
            keep = np.flatnonzero(chosen >= 0)
            kept = batch.select(keep)
            kept.opening[:] = self.pair.arrays["opening_m"][chosen[keep]]
            kept.pregrasp_joints[:] = self.pair.arrays["joint_positions_m"][chosen[keep]]
            accepted.append(kept)
            total += len(kept)
            self.rejected += len(batch) - len(kept)
        if not accepted:
            return self.raw.select(np.empty(0, dtype=np.int64))
        return CandidateBatch(
            *(
                np.concatenate([getattr(b, name) for b in accepted])
                for name in CandidateBatch.__dataclass_fields__
            )
        )

    def valid_closures(self, poses: FloatArray, joints: FloatArray) -> np.ndarray:
        """Screen every actual closure, including the full calibrated gripper below the object."""
        valid = posture_mask(poses, self.pair, self.posture)
        active = self.pair.definition["calibration"]["active_index"]
        commands = self.pair.arrays["joint_positions_m"][:, active]
        order = np.argsort(commands)
        bases = poses @ np.linalg.inv(self.pair.arrays["T_B_tcp"])
        for index in np.ndindex(valid.shape):
            if not valid[index]:
                continue
            command = joints[index][active]
            high = int(np.clip(np.searchsorted(commands[order], command), 1, len(order) - 1))
            lower, upper = order[high - 1], order[high]
            blend = np.clip((command - commands[lower]) / (commands[upper] - commands[lower]), 0, 1)
            sample = (1 - blend) * self.samples[lower] + blend * self.samples[upper]
            points = transform_points(sample, bases[index])
            valid[index] &= (points @ self.posture.object_up_axis).min() >= (
                self.bottom + self.posture.bottom_clearance_m
            )
        return np.asarray(valid.all(axis=1))
