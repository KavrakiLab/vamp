# Provided Scripts

All scripts are in the `scripts/` directory.
We provide a simple visualizer build on top of [PyBullet](https://pybullet.org/wordpress/) in `pybullet_visualizer.py`.

## `evaluate_mbm.py` and `visualize_mbm.py`
This pair of scripts evaluates and visualizes results for planning for the UR5, Panda, Fetch, and Baxter robots over the MotionBenchMaker (MBM) [[3]](../README.md#3) dataset.
Before these scripts can be run, the MBM data must be converted from the provided `problems.tar.bz2` that lives in each robot's subfolder (see [Supported Robots](../README.md#supported-robots) for robots and [Datasets](../resources/README.md#datasets) for more information).
Example output of `evaluate_mbm.py` for the Panda arm on an AMD Ryzen 9 7950X:
```
|      |   Planning Time (μs) |   Simplification Time (μs) |   Total Time (μs) |   Planning Iters. |   Time per Iter. (μs) |
|------|----------------------|----------------------------|-------------------|-------------------|-----------------------|
| mean |              76.1989 |                    67.408  |           144.116 |           471.542 |              4.11661  |
| std  |             123.614  |                    51.0165 |           155.289 |          1969.54  |              4.99841  |
| min  |               7      |                     0      |             7     |             0     |              0        |
| 25%  |              21      |                    31      |            60     |            32     |              0.968027 |
| 50%  |              35      |                    57      |            99     |           104     |              2.59259  |
| 75%  |              70      |                    95      |           165     |           294     |              5        |
| 95%  |             343.3    |                   156      |           441.6   |          1891.3   |             14.2331   |
| max  |            1117      |                   359      |          1218     |         42794     |             39.7714   |
|      |    Initial Cost (L2) |       Simplified Cost (L2) |
|------|----------------------|----------------------------|
| mean |              8.65919 |                    5.17621 |
| std  |              3.3625  |                    1.28935 |
| min  |              2.78403 |                    2.78403 |
| 25%  |              6.38067 |                    4.34307 |
| 50%  |              7.76256 |                    4.90312 |
| 75%  |             10.1662  |                    5.67177 |
| 95%  |             15.9367  |                    7.98012 |
| max  |             25.9764  |                   11.7948  |
Solved / Valid / Total # Problems: 699 / 699 / 700
Completed all problems in 99.162 milliseconds
Total time including Python overhead: 210.897 milliseconds
```
This provides summary statistics over the entire MBM dataset: planning time, path simplification time (the post-processing step), total time taken for both planning and simplification, the initial solution cost (path length in configuration space), the simplified solution's cost (also path length), and the number of planning iterations taken by the planner to solve the problem.
All times are reported in **microseconds** unless otherwise noted.

These scripts support standard planner and simplifier configuration arguments (see [Supported Planners](../README.md#supported-planners)).
In addition, they both support the following arguments:
- `problem`: which takes in either a single problem name (e.g., `table_pick`) or a list (e.g., `table_pick,table_under_pick`) to evaluate against a specific set of problems.
- `dataset`: which describes the specific dataset of problems that should be loaded and inspected. See [Datasets](../resources/README.md#supported-planners) for more information.
- `pointcloud`: construct a pointcloud approximation of the problem's scene geometry and plan against this representation using the CAPT datastructure.
- `samples_per_object`: number of samples per each object for pointcloud generation.
- `filter_radius`: pointcloud filtering radius. Will remove all redundant points that are within the specified radius.
- `filter_cull`: remove points from the pointcloud that outside the robot's reachable workspace.

`evaluate_mbm.py` also supports:
- `trials`: the number of times to repeat each test.

`visualize_mbm.py` also supports:
- `index`: the problem index to visualize.
- `display_object_names`: show the names of each object in the collision geometry.

## `sphere_cage_example.py`
Demonstrates planning on the Panda surrounded by spheres.
This script has two components: a benchmark on a number of similar problems (a small variation added to the position of each sphere), and a visualization of a plan on the nominal scene.
This script has the following arguments that affect both visualization and benchmarking:
- `--radius`: radius used for all the spheres in the cage.

This script has the following arguments that only affect benchmarking:
- `--benchmark`: run a benchmark on `--n_trials` problems.
- `--n_trials`: number of trials to run for benchmark.
- `--variation`: maximum variation applied to each sphere when sampling problems.

Visualization is enabled by `--visualize`, which shows the plan with the PyBullet visualizer.

## `attachments.py`
Demonstrates planning with end-effector attachments using the same problem as in `sphere_cage_example.py`.
A single sphere is attached to the end-effector of the Panda and carried through the cage.

This script has the following arguments that affect the placement of the attached sphere.
- `attachment_radius`: Radius of the attached sphere.
- `attachment_offset`: Offset of the attachment transform along the local-frame Z-axis of the end-effector (i.e., its distance from the end-effector).

## `flying_sphere.py`
Demonstrates generating a roadmap over a heightfield for a flying sphere.
The script has the following arguments:
- `--{x, y, z}`: specify the range of each dimension that the configuration space is scaled to.
- `--radius`: radius of the robot.
- `--iterations`: number of iterations to use to generate the roadmap.

Visualization is enabled by `--visualize`, which shows the plan with the PyBullet visualizer.

# Other Scripts

## `visualize_ompl.py`
If you have the [OMPL](https://ompl.kavrakilab.org/) Python Bindings installed (we recommend using the [pre-built wheels](https://github.com/ompl/ompl/releases/tag/prerelease)) you can also test problems against OMPL using PyBullet's collision checking with this script.

The script supports the following arguments:
- `index`: the problem index to evaluate against.
- `dataset`: which describes the specific dataset of problems that should be loaded and inspected. See [Datasets](../resources/README.md#supported-planners) for more information.
- `problem`: which takes in either a single problem name (e.g., `table_pick`) or a list (e.g., `table_pick,table_under_pick`) to evaluate against a specific set of problems.
