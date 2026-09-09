"""Warp kernels used by asset surface-error measurement."""

# Warp's compiled DSL annotations and query fields are absent from its Python stubs.
# These kernels are type-checked by Warp when compiled in the real preparation run.
# mypy: disable-error-code="valid-type,call-arg,arg-type,attr-defined"

import warp as wp


@wp.kernel
def signed_surface_distance(
    mesh: wp.uint64,
    points: wp.array(dtype=wp.vec3),
    max_distance: float,
    distances: wp.array(dtype=float),
) -> None:
    index = wp.tid()
    point = points[index]
    query = wp.mesh_query_point_sign_winding_number(mesh, point, max_distance)
    distance = float(wp.inf)
    if query.result:
        nearest = wp.mesh_eval_position(mesh, query.face, query.u, query.v)
        distance = wp.length(point - nearest) * query.sign
    distances[index] = distance


@wp.kernel
def surface_rays(
    mesh: wp.uint64,
    origins: wp.array(dtype=wp.vec3),
    directions: wp.array(dtype=wp.vec3),
    distance: float,
    results: wp.array2d(dtype=float),
) -> None:
    i = wp.tid()
    query = wp.mesh_query_ray(mesh, origins[i], directions[i], distance)
    if query.result:
        results[i, 0] = query.t
        results[i, 1] = query.normal[0]
        results[i, 2] = query.normal[1]
        results[i, 3] = query.normal[2]
        results[i, 4] = 1.0


@wp.kernel
def pack_grasp_state(
    objects: wp.array2d(dtype=float),
    links: wp.array3d(dtype=float),
    joints: wp.array2d(dtype=float),
    joint_velocities: wp.array2d(dtype=float),
    velocities: wp.array2d(dtype=float),
    contacts: wp.array3d(dtype=float),
    base: int,
    dofs: int,
    contact_links: int,
    state: wp.array2d(dtype=float),
) -> None:
    env = int(wp.tid())
    for j in range(7):
        state[env, j] = objects[env, j]
        state[env, j + 7] = links[env, base, j]
    for j in range(dofs):
        state[env, 14 + j] = joints[env, j]
        state[env, 14 + dofs + j] = joint_velocities[env, j]
    for j in range(6):
        state[env, 14 + 2 * dofs + j] = velocities[env, j]
    for j in range(contact_links):
        sensor = env * contact_links + j
        own = wp.vec3(contacts[sensor, 0, 0], contacts[sensor, 0, 1], contacts[sensor, 0, 2])
        state[env, 20 + 2 * dofs + j] = wp.length(own)
        other = float(0.0)  # noqa: UP018 - Warp requires a mutable scalar inside this loop.
        if contacts.shape[1] > 1:
            other = wp.length(
                wp.vec3(contacts[sensor, 1, 0], contacts[sensor, 1, 1], contacts[sensor, 1, 2])
            )
        state[env, 20 + 2 * dofs + contact_links + j] = other
