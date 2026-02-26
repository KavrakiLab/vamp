import numpy as np
import os
from viser_utils import (
    setup_viser_with_robot,
    add_point_cloud,
    )
from pathlib import Path

import vamp
import vamp.pointcloud
from fire import Fire

# Starting configuration
a = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

# Goal configuration
b = [2.35, 1.0, 0.0, -0.8, 0, 2.5, 0.785]

# Problem specification: a list of sphere centers
problem = [
    [0.55, 0, 0.25],
    [0.35, 0.35, 0.25],
    [0, 0.55, 0.25],
    [-0.55, 0, 0.25],
    [-0.35, -0.35, 0.25],
    [0, -0.55, 0.25],
    [0.35, -0.35, 0.25],
    [0.35, 0.35, 0.8],
    [0, 0.55, 0.8],
    [-0.35, 0.35, 0.8],
    [-0.55, 0, 0.8],
    [-0.35, -0.35, 0.8],
    [0, -0.55, 0.8],
    [0.35, -0.35, 0.8],
    ]


def main(
        obstacle_radius: float = 0.2,
        points_per_sphere: int = 2000,
        attachment_radius: float = 0.07,
        attachment_offset: float = 0.02,
        planner: str = "rrtc",
        max_range: float = 2.0,
        origin: tuple = (0.0, 0.0, 0.0),
        workcell_min: tuple = (-2.0, -2.0, -2.0),
        workcell_max: tuple = (2.0, 2.0, 2.0),
        cull: bool = True,
        **kwargs,
    ):

    point_cloud = np.vstack(
        [
            vamp.pointcloud.sphere_sample_surface(center, obstacle_radius, points_per_sphere, 0.0)
            for center in problem
            ]
        ).astype(np.float32)

    (vamp_module, planner_func, plan_settings,
     simp_settings) = (vamp.configure_robot_and_planner_with_kwargs("panda", planner, **kwargs))

    robot_dir = Path(__file__).parents[1] / "resources" / "panda"
    server, robot = setup_viser_with_robot(robot_dir, "panda_spherized.urdf")
    robot.update_cfg(a)

    r_min, r_max = vamp_module.min_max_radii()

    filtered_pc_list, elapsed_ns = vamp.filter_pointcloud(
        point_cloud,
        r_min,  # min_dist
        max_range, # max_range
        list(origin), # origin
        list(workcell_min), # workcell_min
        list(workcell_max), # workcell_max
        cull # cull
    )
    filtered_pc = np.array(filtered_pc_list)
    print(f"Filtered point cloud from {len(point_cloud)} to {len(filtered_pc)} points in {elapsed_ns} ns")

    point_cloud_colors = np.tile([255, 0, 0], (point_cloud.shape[0], 1)) # Original: Red
    filtered_pc_colors = np.tile([0, 255, 0], (filtered_pc.shape[0], 1)) # Filtered: Green

    _problem_point_cloud_handles = add_point_cloud(
        server, point_cloud, point_cloud_colors, prefix = "original_pc"
        )
    _filtered_point_cloud_handles = add_point_cloud(
        server, filtered_pc, filtered_pc_colors, prefix = "filtered_pc"
        )

    # display
    while True:
        continue


if __name__ == "__main__":
    Fire(main)
