import numpy as np
from numpy.typing import NDArray

from typing import Dict, Union, List

from . import Environment, filter_pointcloud
from .constants import ROBOT_FIRST_JOINT_LOCATIONS, ROBOT_MAX_RADII, POINT_RADIUS
from .transformations import translation_matrix, quaternion_matrix, concatenate_matrices

# Primitive sampling code taken from https://github.com/fishbotics/geometrout


def pos_and_quat_to_matrix(p, xyzw_q):
    T = translation_matrix(p)
    R = quaternion_matrix(xyzw_q)

    return concatenate_matrices(T, R)


def transform_in_place(point_cloud, transformation_matrix):
    homogeneous_xyz = np.concatenate(
        (np.transpose(point_cloud), np.ones((1, point_cloud.shape[0]))), axis = 0
        )
    transformed_xyz = np.dot(transformation_matrix, homogeneous_xyz)
    point_cloud[:, :3] = np.transpose(transformed_xyz[..., :3, :], (1, 0))
    return point_cloud


def sphere_sample_surface(center, radius, num_points, noise):
    points = np.random.uniform(-1.0, 1.0, (num_points, 3))
    for i in range(points.shape[0]):
        nrm = np.linalg.norm(points[i, :])
        points[i, :] /= nrm
    points = radius * points + center
    if noise > 0.0:
        noise = np.random.uniform(-noise, noise, points.shape)
        return points + noise
    return points


def cylinder_surface_area(radius, height):
    return height * 2 * np.pi * radius + 2 * np.pi * radius**2


def cylinder_sample_surface(pose_matrix, radius, height, num_points, noise):
    angles = np.random.uniform(-np.pi, np.pi, num_points)
    circle_points = np.stack((np.cos(angles), np.sin(angles)), axis = 1)
    surface_area = cylinder_surface_area(radius, height)
    probs = np.array(
        [
            np.pi * radius**2 / surface_area,
            height * 2 * np.pi * radius / surface_area,
            np.pi * radius**2 / surface_area,
            ]
        )
    which_surface = np.searchsorted(np.cumsum(probs), np.random.random(num_points), side = "right")
    circle_points[which_surface == 0] *= np.random.uniform(
        0, radius, size = (np.count_nonzero(which_surface == 0), 1)
        )
    circle_points[which_surface == 1] *= radius
    circle_points[which_surface == 2] *= np.random.uniform(
        0, radius, size = (np.count_nonzero(which_surface == 2), 1)
        )
    z = np.ones((num_points, 1))
    z[which_surface == 0] = -height / 2
    z[which_surface == 1] = np.random.uniform(
        -height / 2,
        height / 2,
        size = (np.count_nonzero(which_surface == 1), 1),
        )
    z[which_surface == 2] = height / 2
    surface_points = np.concatenate((circle_points, z), axis = 1)
    transform_in_place(surface_points, pose_matrix)
    noise = 2 * noise * np.random.random_sample(surface_points.shape) - noise
    return surface_points + noise


def cuboid_sample_surface(
    pose_matrix,
    dims,
    num_points,
    noise,
    ):

    random_points = np.random.uniform(-1.0, 1.0, (num_points, 3))
    random_points = random_points * dims / 2
    probs = np.array(
        [
            dims[1] * dims[2],
            dims[1] * dims[2],
            dims[0] * dims[2],
            dims[0] * dims[2],
            dims[0] * dims[1],
            dims[0] * dims[1],
            ]
        )
    probs /= np.sum(probs)
    sides = np.searchsorted(np.cumsum(probs), np.random.random(num_points), side = "right")
    random_points[sides == 0, 0] = dims[0] / 2
    random_points[sides == 1, 0] = -dims[0] / 2
    random_points[sides == 2, 1] = dims[1] / 2
    random_points[sides == 3, 1] = -dims[1] / 2
    random_points[sides == 4, 2] = dims[2] / 2
    random_points[sides == 5, 2] = -dims[2] / 2
    transform_in_place(random_points, pose_matrix)
    noise = 2 * noise * np.random.random_sample(random_points.shape) - noise
    return random_points + noise


def box_to_pc(box, n):
    tf = pos_and_quat_to_matrix(box['position'], box['orientation_quat_xyzw'])
    pc = cuboid_sample_surface(tf, np.array(box['half_extents']) * 2, n, 0)
    return pc


def cylinder_to_pc(cylinder, n):
    tf = pos_and_quat_to_matrix(cylinder['position'], cylinder['orientation_quat_xyzw'])
    pc = cylinder_sample_surface(tf, cylinder['radius'], cylinder['length'], n, 0)
    return pc


def problem_to_pointcloud(problem, n):
    np.random.seed(0)
    return np.vstack(
        [cylinder_to_pc(c, n) for c in problem['cylinder']] + [box_to_pc(box, n) for box in problem['box']]
        )


def problem_dict_to_pointcloud(
        robot: str,
        r_min: float,
        r_max: float,
        problem: Dict[str, List[Dict[str, Union[float, NDArray[np.float32]]]]],
        samples_per_object: int,
        filter_radius: float,
        filter_cull: bool
    ):
    original_pointcloud = problem_to_pointcloud(problem, samples_per_object).tolist()

    filter_origin = [0.0, 0.0, 0.0]
    if robot in ROBOT_FIRST_JOINT_LOCATIONS.keys():
        filter_origin = ROBOT_FIRST_JOINT_LOCATIONS[robot]
    else:
        print(f"Origin location for {robot} not found. Substituting {filter_origin}.")

    filter_cull_radius = 1.4
    if robot in ROBOT_MAX_RADII.keys():
        filter_cull_radius = ROBOT_MAX_RADII[robot]
    else:
        print(f"Max extension radius for {robot} not found. Substituting {filter_cull_radius}m.")

    bbox_lo = np.asarray(filter_origin) - filter_cull_radius
    bbox_hi = np.asarray(filter_origin) + filter_cull_radius
    filtered_pc, filter_time = filter_pointcloud(
        original_pointcloud,
        filter_radius,
        filter_cull_radius,
        filter_origin,
        bbox_lo,
        bbox_hi,
        filter_cull
    )

    env = Environment()
    build_time = env.add_pointcloud(filtered_pc, r_min, r_max, POINT_RADIUS)

    return env, original_pointcloud, filtered_pc, filter_time, build_time
