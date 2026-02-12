from pathlib import Path
import vamp
from fire import Fire

# Starting configuration
a = [-1., 1., 0., -0.8, 0, 2.5, 0.785]

# Goal configuration
b = [1., 1., 0., -0.8, 0, 2.5, 0.785]


def main(
    mesh_file: str = None,
    position: tuple = (0.5, 0.0, 0.0),
    scale: float = 0.01,
    decompose: bool = False,
    benchmark: bool = False,
    n_trials: int = 100,
    visualize: bool = True,
    planner: str = "rrtc",
    sampler_name: str = "halton",
    skip_rng_iterations: int = 0,
    **kwargs,
    ):

    if mesh_file is None:
        mesh_path = Path(__file__).parent.parent / "resources" / "objects" / "Stanford_Bunny.stl"
    else:
        mesh_path = Path(mesh_file)

    if not mesh_path.exists():
        raise FileNotFoundError(f"Mesh file not found: {mesh_path}")

    polytopes = vamp.mesh_to_polytopes(
        mesh_path, position=position, scale=scale, convex_decomposition=decompose, name="mesh"
        )

    (vamp_module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs("panda", planner, **kwargs)

    sampler = getattr(vamp_module, sampler_name)()
    sampler.skip(skip_rng_iterations)

    def create_environment():
        e = vamp.Environment()
        for polytope in polytopes:
            e.add_polytope(polytope)
        return e

    if benchmark:
        import pandas as pd

        results = []
        for _ in range(n_trials):
            e = create_environment()

            if vamp.panda.validate(a, e) and vamp.panda.validate(b, e):
                result = planner_func(a, b, e, plan_settings, sampler)
                simple = vamp_module.simplify(result.path, e, simp_settings, sampler)
                results.append(vamp.results_to_dict(result, simple))

        df = pd.DataFrame.from_dict(results)

        # Convert to microseconds
        df["planning_time"] = df["planning_time"].dt.microseconds
        df["simplification_time"] = df["simplification_time"].dt.microseconds

        # Get summary statistics
        stats = df[[
            "planning_time",
            "simplification_time",
            "initial_path_cost",
            "simplified_path_cost",
            "planning_iterations"
            ]].describe()

        print(stats)

    if visualize:
        from vamp import pybullet_interface as vpb

        robot_dir = Path(__file__).parent.parent / 'resources' / 'panda'
        sim = vpb.PyBulletSimulator(str(robot_dir / f"panda_spherized.urdf"), vamp_module.joint_names(), True)

        e = create_environment()
        for polytope in polytopes:
            sim.add_polytope(polytope)

        result = planner_func(a, b, e, plan_settings, sampler)
        simple = vamp_module.simplify(result.path, e, simp_settings, sampler)

        simple.path.interpolate_to_resolution(vamp.panda.resolution())

        sim.animate(simple.path)


if __name__ == "__main__":
    Fire(main)
