# 🧛 Vector-Accelerated Motion Planning (VAMP)

[![arXiv VAMP](https://img.shields.io/badge/arXiv-2309.14545-b31b1b.svg)](https://arxiv.org/abs/2309.14545)
[![arXiv CAPT](https://img.shields.io/badge/arXiv-2406.02807-b31b1b.svg)](https://arxiv.org/abs/2406.02807)
[![Build Check](https://github.com/KavrakiLab/vamp/actions/workflows/build.yml/badge.svg)](https://github.com/KavrakiLab/vamp/actions/workflows/build.yml)
[![Format Check](https://github.com/KavrakiLab/vamp/actions/workflows/format.yml/badge.svg)](https://github.com/KavrakiLab/vamp/actions/workflows/format.yml)

This repository hosts the code for the ICRA 2024 paper [“Motions in Microseconds via Vectorized Sampling-Based Planning”](https://arxiv.org/abs/2309.14545) as well as an implementation of the Collision-Affording Point Tree (CAPT) from the forthcoming RSS 2024 paper [“Collision-Affording Point Trees: SIMD-Amenable Nearest Neighbors for Fast Collision Checking”](http://arxiv.org/abs/2406.02807).

**TL;DR**: By exploiting ubiquitous [CPU SIMD instructions](https://en.wikipedia.org/wiki/Single_instruction,_multiple_data) to accelerate collision checking and forward kinematics (FK), `vamp`'s RRT-Connect [[1]](#1) solves problems for the Franka Emika Panda from the MotionBenchMaker dataset [[3]](#3) at a median speed of 35 microseconds (on one core of a consumer desktop PC).
This approach to hardware-accelerated parallel sampling-based motion planning extends to other planning algorithms without modification (e.g., PRM [[2]](#2)) and also works on low-power systems (e.g., an ARM-based [OrangePi](http://www.orangepi.org/)).
We also accelerate collision checking against pointclouds with a novel spatial data structure, the Collision-Affording Point Tree (CAPT), which has an average query time of less than 10 nanoseconds on 3D scenes composed of thousands of points.

If you found this research useful for your own work, please use the following citation:
```bibtex
@InProceedings{vamp_2024,
  title = {Motions in Microseconds via Vectorized Sampling-Based Planning},
  author = {Thomason, Wil and Kingston, Zachary and Kavraki, Lydia E.},
  booktitle = {IEEE International Conference on Robotics and Automation},
  date = {2024},
  url = {http://arxiv.org/abs/2309.14545},
}
```

If you use CAPTs or the pointcloud collision checking components of this repository, please also use the following citation:
```bibtex
@InProceedings{capt_2024,
  title = {Collision-Affording Point Trees: {SIMD}-Amenable Nearest Neighbors for Fast Collision Checking},
  author = {Ramsey, Clayton W. and Kingston, Zachary and Thomason, Wil and Kavraki, Lydia E.},
  booktitle = {Robotics: Science and Systems},
  date = {2024},
  url = {http://arxiv.org/abs/2406.02807},
  note = {To Appear.}
}
```

## Building and Installing

VAMP requires the following system dependencies:
- [CMake](https://cmake.org/) version 3.16 or greater.
- GCC 8+ or Clang 10+, along with the C++ standard library.
  To install GCC on Ubuntu, `sudo apt install build-essential`.
  To install Clang and its C++ standard library implementation on Ubuntu 22.04, `sudo apt install clang libstdc++6`
- Python development headers for generating Python bindings.
  We support Python 3.8 and above.
  To install on Ubuntu 22.04, `sudo apt install python3-dev`.
- [`Eigen3`](https://eigen.tuxfamily.org/index.php?title=Main_Page) for some vector/matrix operations.
  To install on Ubuntu 22.04, `sudo apt install libeigen3-dev`.

VAMP fetches the following external dependencies via [CPM](https://github.com/cpm-cmake/CPM.cmake):
- [`nanobind`](https://github.com/wjakob/nanobind): for Python bindings
- [`nigh`](https://github.com/KavrakiLab/nigh): a fork of the original [`nigh`](https://github.com/UNC-Robotics/nigh) [[4]](#4) to better use our vector types
- [`pdqsort`](https://github.com/orlp/pdqsort): for fast sorting
- [`SIMDxorshift`](https://github.com/lemire/SIMDxorshift): alternative fast random numbers for x86 machines


Download the code and submodules:
```bash
git clone git@github.com:KavrakiLab/vamp.git
```
### Python
For use through Python, install with `pip`:
```bash
cd vamp
pip install .
```
If you want to install all Python dependencies to run the examples, specify those optional dependencies:
```bash
pip install .[examples,heightmaps]
```
If you have installed the `examples` dependencies, test your installation by running:
```bash
python scripts/sphere_cage_example.py --visualize
```
Which will benchmark a simple scenario of the Franka Emika Panda in a cage of spheres and visualize one of the results.
See the [README in the scripts directory](scripts/README.md) for more details.

### C++
If you wish to extend `vamp` via C++, please build directly with CMake, e.g.:
```
cd vamp
cmake -B build -DCMAKE_BUILD_TYPE=Release .
cmake --build build
```
Please see `CMakeLists.txt` for further build configuration options.

### Docker
We provide example dockerfiles in `docker/` that show installation on Ubuntu 20.04, 22.04, and 24.04.

### Conda/Mamba
Installation in [Conda](https://docs.conda.io/en/latest/)/[Mamba](https://mamba.readthedocs.io/en/latest/index.html) environments is supported.
See the `environment.yaml` file for a basic environment, and see `docker/ubuntu2204-conda.dockerfile` for an example installation.

### Supported Platforms
We currently support x86 CPUs (e.g., Intel, AMD) with the [AVX2 vector instruction set](https://en.wikipedia.org/wiki/Advanced_Vector_Extensions) and ARM CPUs (e.g., Raspberry Pi, Mac M1) with [NEON](https://en.wikipedia.org/wiki/ARM_architecture_family#Advanced_SIMD_(Neon)).
Please see the `docker/` folder for reference installation procedures.

### Using Clang instead of GCC
You can force the use of Clang instead of GCC for compiling VAMP by uncommenting the line at the bottom of the `pyproject.toml` (or setting the corresponding CMake variable for C++ builds):
```toml
[tool.scikit-build.cmake.define]
VAMP_LTO = "ON"
VAMP_FORCE_CLANG = "ON"
```

This may have performance implications for some systems (positive or negative).
We recommend trying both compilers to see which works best for your particular setup.

## Supported Robots
We ship code to do planning for a sphere in $\mathbb{R}^3$ and the UR5, Panda, Fetch, and Baxter models as found in [`robowflex_resources`](https://github.com/KavrakiLab/robowflex_resources) [[5]](#5), as used in the MotionBenchMaker (MBM) [[3]](#3) dataset.
Resources for each robot (URDF, SRDF, meshes, etc.) are all provided in the `resources/` directory under each robot's name.
See the [README](resources/README.md) for more information on the robot models.

The MBM problems for each robot are compressed in `problems.tar.bz2`.
For the UR5, Panda, and Fetch, these problems are the `table_pick`, `table_under_pick`, `box`, `bookshelf_small`, `bookshelf_tall`, `bookshelf_thin`, and `cage` scenarios, each with 100 problems.
For the Baxter, these problems are `bookshelf_tall_both_arms_easy`, `bookshelf_tall_both_arms_medium`, and `bookshelf_tall_both_arms_hard` scenarios, each with 600 problems (note that the difficulty modifier refers to the amount of variation in the scene, not difficulty of finding a problem solution).
These problems can be decompressed into a convenient [pickle](https://docs.python.org/3/library/pickle.html) and JSON format with the script `resources/problem_tar_to_pkl_json.py`, after VAMP has been installed:
```bash
# choose robot name from {ur5, panda, fetch, baxter}
python resources/problem_tar_to_pkl_json.py --robot panda
```
This only needs to be run once.

Each robot in VAMP is provided as a Python submodule (e.g., `vamp.panda`, `vamp.fetch`) and supports the following functions:
- `rrtc`: RRT-Connect. See [Supported Planners](#Supported-Planners)
- `prm`: PRM. See [Supported Planners](#Supported-Planners)
- `roadmap`: returns the constructed roadmap generated by PRM
- `simplify`: simplifies a planned path
- `validate`: checks if a standalone configuration in collision
- `sphere_validity`: returns a list, for each sphere in the robot model, of the names of all objects currently colliding with the sphere
- `fk`: performs FK to compute the locations of all robot collision spheres
- `filter_from_pointcloud`: removes points in the pointcloud that are currently in collision with the robot (i.e., points which probably belong to the robot, if the robot is in a known valid configuration)

For the flying sphere in $\mathbb{R}^3$, additional operations are available to set the domain of the sphere and the radius:
- `vamp.sphere.set_lows()` and `vamp.sphere.set_highs()` to set bounding box of space
- `vamp.sphere.set_radius()` to set the sphere's radius

## Supported Planners
We currently ship two planners:
- `rrtc`, which is an implementation of a dynamic-domain [[6]](#6) balanced [[7]](#7) RRT-Connect [[1]](#1).
- `prm`, which is an implementation of basic PRM [[2]](#2) (i.e., PRM without the bounce heuristic, etc.).

Note that these planners support planning to a set of goals, not just a single goal.
Also, all planners use a multi-dimensional Halton sequence for deterministic planning [[12-13]](#12).

We also ship a number of heuristic simplification routines:
- randomized and deterministic shortcutting [[8, 9]](#8) (`REDUCE` and `SHORTCUT`)
- B-spline smoothing [[10]](#10) (`BSPLINE`)
- randomized perturbation [[11]](#11) (`PERTURB`).
These routines heuristically attempt to shorten the total path length in configuration space.
See the `src/impl/vamp/planning/` folder for more information.

### Planner Configuration and Hyperparameters
We provide a helper function `vamp.configure_robot_and_planner_with_kwargs(robot, planner, **kwargs)` to help configure all the planner and simplification settings that are available.
Scripts that use this helper (`sphere_cage_example.py`, `evaluate_mbm.py`, `visualize_mbm.py`) provide the following arguments:
- `--robot`: Specify the robot to use. See [Supported Robots](#Supported-Robots) for names.
- `--planner`: Planner name, either `rrtc` or `prm`.

Each planner supports a number of settings. Both support the following:
- `--max_iterations`: maximum planner iterations.
- `--max_samples`: maximum samples planner can allocate.
- `--rng_skip_iterations`: skip this many samples from the RNG before planning.

For `rrtc`:
- `--range`: RRT extension range. Set to sensible default for each robot, usually something in [0.5, 2].
- `--dynamic_domain`: `True` or `False`, enables [dynamic domain](https://ieeexplore.ieee.org/document/1570709) sample filtering.
- `--radius`: initial restricted radius for dynamic domain. Usually between [0.5, 5].
- `--alpha`: update parameter to shrink/grow dynamic domain. Usually between [0.00001, 0.01]
- `--min_radius`: minimum radius of dynamic domain. Usually between [0.5, 1]
- `--balance`: `True` or `False`, enables tree balancing.
- `--tree_ratio`: ratio of tree sizes at which trees are swapped. 1 is perfect balancing.
- `--start_tree_first`: `True` or `False`, grow from start tree or goal tree first.
See `rrtc_settings.hh` for more information.

For `prm`, the settings must be configured with a neighbor parameter structure, e.g.:
```py
robot_module = vamp.panda # or other robot submodule
prmstar_params = vamp.PRMNeighborParams(robot_module.dimension(), robot_module.space_measure())
prm_settings = vamp.PRMSettings(prmstar_params)
```
This is handled by default in the configuration function.

For simplification:
- `--simplification_operations`: sequence of shortcutting heuristics to apply each iteration. By default, `[SHORTCUT,BSPLINE]`. Can specify any sequence of the above keys.
- `--max_iterations`: maximum iterations of simplification. If no heuristics do any work, then early terminates from simplification.
- `--bspline_max_steps`: maximum iterations of B-spline smoothing.
- `--bspline_min_change`: minimum change before smoothing is done.
- `--bspline_midpoint_interpolation`: point along each axis B-spline interpolation is done from.
- `--reduce_max_steps`: maximum iterations of randomized vertex reduction.
- `--reduce_max_empty_steps`: maximum no-op iterations of randomized vertex reduction.
- `--reduce_range_ratio`: range from [0, 1] as ratio of entire path that randomized shortcuts are attempted.
- `--perturb_max_steps`: maximum iterations of randomized perturbations.
- `--perturb_max_empty_steps`: maximum no-op iterations of randomized perturbations.
- `--perturb_perturbation_attempts`: maximum number of attempts per iteration of perturbation.
- `--perturb_range`: range vertices are perturbed.
See `simplify_settings.hh` for more information.

## Environment Representation

VAMP currently supports collision checking against primitive models of the environment and pointclouds via CAPTs (see [planned features](#Planned-Features) for forthcoming extensions to meshes, etc.).
Environments (`vamp.Environment`) can be constructed by adding objects (`add_sphere(vamp.Sphere(...))`, etc.).
These objects can be created with the following:
- `vamp.Sphere(position, radius)`: a sphere with position and radius.
- `vamp.Capsule(center, euler_xyz, radius, length)` and `vamp.Capsule(endpoint1, endpoint2, radius)`: a capsule in space, specified by either its frame, radius, and length or by the endpoints and radius.
- `vamp.Cuboid(center, euler_xyz, half_extents)`: a cuboid specified by the frame and then half-extents (radii) along the X, Y, and Z axes in its local frame.
- `vamp.Heightfield` via `vamp.make_heightfield` / `vamp.png_to_heightfield`: a heightfield specified by pixel intensity in an image file, scaled over specified dimensions.
- Pointclouds via `add_pointcloud()` in `vamp.Environment`. This will construct a CAPT from the provided list of points, the minimum and maximum robot sphere radii, and radius for each point in the pointcloud.
See the `src/impl/vamp/collision/` folder for more information.

Some robots (currently, the UR5, Panda, and Fetch) support attaching custom geometry (a collection of spheres) to the end-effector via `vamp.Attachment(relative_position, relative_quaternion_xyzw)`.
Spheres can be added (in the attachment's frame) with `add_sphere(...)`.
The attachment can be added to the environment with `vamp.Environment.attach(...)`, and removed with `vamp.Environment.detach()`.
An example use of attachments with the Panda arm is available in `scripts/attachments.py`.


## Code Overview

The code lives in the `src` folder, split into `impl` (the C++ core) and `vamp` (the Python interface).
Scripts live in the `scripts/` folder; see the [README](scripts/README.md) in that directory for more information.

Inside `impl/vamp`, the code is divided into the following directories:
- `vector.hh` and `vector/`:
  Our abstract SIMD interface that underpins much of the core C++ library.
  The interface for these types is described in `interface.hh`, and the actual implementations of the operations for specific instruction sets are in `avx.hh` (for x86 AVX2) and `neon.hh` (for ARM NEON).
- `bindings/`:
  Python bindings, via [nanobind](https://github.com/wjakob/nanobind).
  The main module is described starting in `python.cc`, with code separated out logically for more efficient compilation.
  `common.hh` is a templated helper that is used to create each robot's submodule.

- `random/`:
  Pseudorandom number generation, e.g., `halton.hh` for the SIMD Halton generator.

- `collision/`:
  Collision checking routines and environment description.
  Primitives are described in `shapes.hh`, the methods to create them in `factory.hh`, the environment in `environment.hh`, and collision checking of spheres against the environment in `validity.hh`.
  CAPTs are implemented in `capt.hh`, with pointcloud filtering in `filter.hh`.

- `planning/`:
  Planning and simplification routines.
  `rrtc.hh` and `rrtc_settings.hh` are for our RRT-Connect implementation.
  `prm.hh` and `roadmap.hh` are for our PRM implementation.
  `simplify.hh` and `simplify_settings.hh` are for simplification heuristics.
  `validate.hh` contains the raked motion validator.

- `robots/`:
  Robot specific code.
  Each named subfolder contains `fk.hh` for each robot, which contains the automatically generated code from the tracing compiler.
  The named `{robot}.hh` folder at the top is a helper struct which maps `fk.hh` routines and other robot-specific information.

## Planned Features
- [ ] Improved API documentation
- [ ] Improved Python API
- [ ] Batch configuration validation
- [ ] Planning subgroups
- [X] Object attachment at end-effector
- [ ] Mesh collision checking
- [X] Pointcloud collision checking
- [ ] Manifold-constrained planning
- [ ] Time-optimal trajectory parameterization
- and more...

## References
- <a id="1">[1]</a> J. J. Kuffner and S. M. LaValle. "RRT-Connect: An efficient approach to single-query path planning". In: IEEE International Conference on Robotics and Automation. Vol. 2. IEEE. 2000, pp. 995–1001.
- <a id="2">[2]</a> L. E. Kavraki, P. Svestka, J.-C. Latombe, and M. H. Overmars. "Probabilistic roadmaps for path planning in high-dimensional configuration spaces". In: IEEE Transations on Robotics and Automation 12.4 (1996), pp. 566–580.
- <a id="3">[3]</a> C. Chamzas, C. Quintero-Pena, Z. Kingston, A. Orthey, D. Rakita, M. Gleicher, M. Toussaint, and L. E. Kavraki. "MotionBenchMaker: A tool to generate and benchmark motion planning datasets". In: IEEE Robotics and Automation Letters 7.2 (2021), pp. 882–889.
- <a id="4">[4]</a> J. Ichnowski and R. Alterovitz. "Concurrent nearest-neighbor searching for parallel sampling-based motion planning in SO(3), SE(3), and Euclidean spaces." Algorithmic Foundations of Robotics. Springer. 2020, pp. 69-85
- <a id="5">[5]</a> Z. Kingston and L. E. Kavraki. "Robowflex: Robot motion planning with MoveIt made easy." In: IEEE/RSJ International Conference on Intelligent Robots and Systems, pp. 3108-3114. IEEE, 2022.
- <a id="6">[6]</a> L. Jaillet, A. Yershova, S. M. La Valle, and T. Siméon. "Adaptive tuning of the sampling domain for dynamic-domain RRTs". In: IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE. 2005, pp. 2851–2856.
- <a id="7">[7]</a> J. J. Kuffner and S. M. LaValle, "An efficient approach to path planning using a balanced bidirectional RRT search". Technical Report, Robotics Institute, Carnegie Mellon University, 2005.
- <a id="8">[8]</a> R. Geraerts and M. H. Overmars. "Creating high-quality paths for motion planning". In: The International Journal of Robotics Research 26.8 (2007), pp. 845–863.
- <a id="9">[9]</a> K. Hauser and V. Ng-Thow-Hing. "Fast smoothing of manipulator trajectories using optimal bounded-acceleration shortcuts". In: IEEE International Conference on Robotics and Automation. IEEE. 2010, pp. 2493–2498.
- <a id="10">[10]</a> J. Pan, L. Zhang, and D. Manocha. "Collision-free and smooth trajectory computation in cluttered environments". In: The International Journal of Robotics Research 31.10 (2012), pp. 1155–1175.
- <a id="11">[11]</a> J. Mainprice, E. Sisbot, L. Jaillet, J. Cortes, R. Alami, T. Simeon "Planning human-aware motions using a sampling-based costmap planner", Robotics and Automation, 2011.
- <a id="12">[12]</a> L. Janson, B. Ichter, and M. Pavone. "Deterministic sampling-based motion planning: Optimality, complexity, and performance". The International Journal of Robotics Research 37.1 (2018): 46-61.
- <a id="13">[13]</a> J. H. Halton. "On the efficiency of certain quasi-random sequences of points in evaluating multi-dimensional integrals". In: Numerische Mathematik 2 (1960), pp. 84–90.
- <a id="14">[14]</a> A. Fishman, A. Murali, C. Eppner, B. Peele, B. Boots, and D. Fox. "Motion policy networks". Conference on Robot Learning, pp. 967-977
