from pathlib import Path
from fire import Fire

import vamp

robot_initial_pos = [23.661283493041992, 7.7295989990234375, 1.866841197013855]
goal_pos = [-23.25229835510254, -9.870399475097656, 2.1933717727661133]


def main(
    visualize: bool = False,
    x: float = 20,
    y: float = 20,
    z: float = 1,
    radius: float = 0.1,
    iterations: int = 10000,
    sampler_name: str = "halton",
    ):

    env = vamp.Environment()

    maze = Path(__file__).parent.parent / "resources" / "heightfields" / "maze.png"
    hf = vamp.png_to_heightfield(
        maze,
        (0., 0., 0.),
        (1. / 256 * (2. * x), 1. / 256 * (2. * y), 1. / z),
        )
    env.add_heightfield(hf)

    vamp.sphere.set_lows([-x, -y, 0])
    vamp.sphere.set_highs([x, y, z])
    vamp.sphere.set_radius(radius)

    settings = vamp.PRMSettings(vamp.PRMNeighborParams(vamp.sphere.dimension(), vamp.sphere.space_measure()))
    settings.max_iterations = iterations

    sampler = getattr(vamp.sphere, sampler_name)()
    roadmap = vamp.sphere.roadmap(robot_initial_pos, goal_pos, env, settings, sampler)

    print(f"Created roadmap with {len(roadmap)} nodes in {roadmap.nanoseconds / 1e9} seconds!")

    if visualize:
        from vamp import pybullet_interface as vpb
        pb = vpb.PyBulletSimulator("", [], True)

        pb.draw_roadmap(vamp.sphere.fk, roadmap)

        pb.add_height_map(
            maze, maze, scale = [1. / 256 * (2 * x), 1. / 256 * (2 * y), z], center = [0., 0., z / 2]
            )

        pb.set_camera([x + 1, y + 1, z + 1], [0, 0, 0])

        while True:
            ...


if __name__ == "__main__":
    Fire(main)
