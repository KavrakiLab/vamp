import numpy as np
from pathlib import Path
import pandas as pd
import random
import copy
import vamp
from fire import Fire

# Starting configuration
a = [0., -0.785, 0., -2.356, 0., 1.571, 0.785]

# Goal configuration
b = [2.35, 1., 0., -0.8, 0, 2.5, 0.785]

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
    attachment_radius: float = 0.05,
    attachment_offset: float = 0.12,
    visualize: bool = False,
    planner: str = "rrtc",
    **kwargs,
    ):

    (vamp_module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs("panda_attachment", planner, **kwargs)

    attachment = vamp.Attachment(0, 0, attachment_offset, 0, 0, 0, 1)
    attachment.add_spheres([vamp.Sphere([0, 0, 0], attachment_radius)])

    if visualize:
        from vamp import pybullet_interface as vpb

        robot_dir = Path(__file__).parent.parent / 'resources' / 'panda'
        sim = vpb.PyBulletSimulator(
            str(robot_dir / f"panda_spherized.urdf"), vamp.ROBOT_JOINTS['panda'], True
            )

        e = vamp.Environment()
        for sphere in problem:
            e.add_sphere(vamp.Sphere(sphere, obstacle_radius))
            sim.add_sphere(obstacle_radius, sphere)

        attachment_sphere = sim.add_sphere(attachment_radius, [0, 0, 0])

        def callback(configuration):
            output = vamp_module.eefk(configuration)
            attachment.pose(output)
            sphere = attachment.posed_spheres[0]

            sim.update_object_position(attachment_sphere, [sphere.x, sphere.y, sphere.z])

        e.attach(attachment)

        result = planner_func(a, b, e, plan_settings)
        simple = vamp_module.simplify(result.path, e, simp_settings)
        simple.path.interpolate(vamp.panda.resolution())

        sim.animate(simple.path, callback)


if __name__ == "__main__":
    Fire(main)
