"""CPU asset geometry in metres; matrices use column vectors and T_A_B."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import trimesh
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

if TYPE_CHECKING:
    from pxr import Usd


type FloatArray = NDArray[np.float64]
type IntArray = NDArray[np.int64]


def pose_matrices(poses: FloatArray) -> FloatArray:
    """Convert (...,7) xyz/xyzw poses, including empty shards and trial axes."""
    if poses.shape[-1:] != (7,) or not np.isfinite(poses).all():
        raise ValueError("Expected finite xyz/xyzw poses")
    flat = poses.reshape(-1, 7)
    if not np.allclose(np.linalg.norm(flat[:, 3:], axis=1), 1, atol=1e-7):
        raise ValueError("Pose quaternions must be normalized")
    matrices = np.tile(np.eye(4), (len(flat), 1, 1))
    if len(flat):
        matrices[:, :3, :3] = Rotation.from_quat(flat[:, 3:]).as_matrix()
        matrices[:, :3, 3] = flat[:, :3]
    return matrices.reshape((*poses.shape[:-1], 4, 4))


def matrix_poses(matrices: FloatArray) -> FloatArray:
    """Serialize (...,4,4) transforms as xyz/xyzw with canonical quaternion sign."""
    flat = matrices.reshape(-1, 4, 4)
    poses = np.empty((len(flat), 7))
    if len(flat):
        poses[:, :3] = flat[:, :3, 3]
        poses[:, 3:] = Rotation.from_matrix(flat[:, :3, :3]).as_quat(canonical=True)
    return poses.reshape((*matrices.shape[:-2], 7))


class SurfaceQueries:
    """One reusable CUDA BVH per actual source/collision mesh."""

    def __init__(self, mesh: trimesh.Trimesh, device: str) -> None:
        import warp as wp

        self.device: str = device
        self.mesh: wp.Mesh = wp.Mesh(
            points=wp.array(np.asarray(mesh.vertices), dtype=wp.vec3, device=device),
            indices=wp.array(np.asarray(mesh.faces).ravel(), dtype=wp.int32, device=device),
            support_winding_number=True,
        )

    def distances(self, points: FloatArray, max_distance: float) -> FloatArray:
        import warp as wp

        from graspdatagen.kernels import signed_surface_distance

        result = wp.empty(len(points), dtype=wp.float32, device=self.device)
        wp.launch(
            signed_surface_distance,
            dim=len(points),
            inputs=[
                self.mesh.id,
                wp.array(points, dtype=wp.vec3, device=self.device),
                max_distance,
            ],
            outputs=[result],
            device=self.device,
        )
        return result.numpy().astype(np.float64)

    def rays(self, origins: FloatArray, directions: FloatArray, distance: float) -> FloatArray:
        """Return hit distance, normal xyz and hit flag; misses have flag zero."""
        import warp as wp

        from graspdatagen.kernels import surface_rays

        result = wp.zeros((len(origins), 5), dtype=wp.float32, device=self.device)
        wp.launch(
            surface_rays,
            dim=len(origins),
            inputs=[
                self.mesh.id,
                wp.array(origins, dtype=wp.vec3, device=self.device),
                wp.array(directions, dtype=wp.vec3, device=self.device),
                distance,
            ],
            outputs=[result],
            device=self.device,
        )
        return result.numpy().astype(np.float64)


def transform_points(points: FloatArray, transform: FloatArray) -> FloatArray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def relative_transform(prim: Usd.Prim, frame: Usd.Prim) -> FloatArray:
    from pxr import UsdGeom

    cache = UsdGeom.XformCache()
    return np.asarray(
        cache.GetLocalToWorldTransform(prim) * cache.GetLocalToWorldTransform(frame).GetInverse(),
        dtype=np.float64,
    ).T.copy()


def set_transform(prim: Usd.Prim, transform: FloatArray) -> None:
    from pxr import Gf, UsdGeom

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.MakeMatrixXform().Set(Gf.Matrix4d(transform.T.tolist()))


def mesh_in_frame(prim: Usd.Prim, frame: Usd.Prim) -> trimesh.Trimesh:
    """Read composed instances, holes, winding and transforms at default time.

    The supported source assets have triangular faces. Quads are triangulated
    by trimesh; general concave n-gons are rejected rather than silently fanned.
    """
    from pxr import UsdGeom

    mesh = UsdGeom.Mesh(prim)
    if not mesh:
        raise ValueError(f"Expected a mesh: {prim.GetPath()}")
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int64)
    indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
    vertices = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
    if counts.sum() != len(indices) or not np.isin(counts, [3, 4]).all():
        raise ValueError(f"Unsupported or invalid mesh topology: {prim.GetPath()}")
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all():
        raise ValueError(f"Invalid mesh points: {prim.GetPath()}")
    if not len(indices) or indices.min() < 0 or indices.max() >= len(vertices):
        raise ValueError(f"Invalid mesh indices: {prim.GetPath()}")
    holes = set(mesh.GetHoleIndicesAttr().Get() or [])
    if (counts == 3).all() and not holes:
        faces = indices.reshape(-1, 3)
    else:
        polygons = np.split(indices, np.cumsum(counts)[:-1])
        faces = trimesh.geometry.triangulate_quads(
            [polygon for i, polygon in enumerate(polygons) if i not in holes]
        )
    transform = relative_transform(prim, frame)
    if (mesh.GetOrientationAttr().Get() == "leftHanded") != (np.linalg.det(transform[:3, :3]) < 0):
        faces = faces[:, ::-1]
    return trimesh.Trimesh(transform_points(vertices, transform), faces, process=False)


def author_mesh(stage: Usd.Stage, path: str, mesh: trimesh.Trimesh) -> Usd.Prim:
    from pxr import UsdGeom, Vt

    result = UsdGeom.Mesh.Define(stage, path)
    result.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(mesh.vertices, dtype=np.float32)))
    result.CreateFaceVertexCountsAttr([3] * len(mesh.faces))
    result.CreateFaceVertexIndicesAttr(np.asarray(mesh.faces).ravel().tolist())
    result.CreateSubdivisionSchemeAttr("none")
    result.CreateDisplayColorAttr([(0.52, 0.57, 0.60)])
    return result.GetPrim()
