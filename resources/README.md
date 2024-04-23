# VAMP Robot and Problem Resources

## Robots

We ship code to do planning for a sphere in $\mathbb{R}^3$ and the UR5, Panda, Fetch, and Baxter models as found in [`robowflex_resources`](https://github.com/KavrakiLab/robowflex_resources) [[5]](../README.md#5), as used in the MotionBenchMaker (MBM) [[3]](../README.md#3) dataset.
Resources for each robot (URDF, SRDF, meshes, etc.) are all provided in the directory under each robot's name.

To generate code for a new robot, we require a spherical decomposition of the robot's collision geometry.
See the `*_spherized.urdf` files for examples of these spherical decompositions for each robot.
We also use a secondary hierarchical decomposition: see the high-level single sphere URDF used for each robot in the `*_spherized_1.urdf` files.
These decompositions are conservative estimations of the robot's mesh geometry.

## Datasets

### MotionBenchMaker problems
The MBM problems for each robot are compressed in `problems.tar.bz2`.
For the UR5, Panda, and Fetch, these problems are the `table_pick`, `table_under_pick`, `box`, `bookshelf_small`, `bookshelf_tall`, `bookshelf_thin`, and `cage` scenarios, each with 100 problems.
For the Baxter, these problems are `bookshelf_tall_both_arms_easy`, `bookshelf_tall_both_arms_medium`, and `bookshelf_tall_both_arms_hard` scenarios, each with 600 problems (note that the difficulty modifier refers to the amount of variation in the scene, not difficulty of finding a problem solution).
These problems can be decompressed into a convenient [pickle](https://docs.python.org/3/library/pickle.html) and JSON format with the script `resources/problem_tar_to_pkl_json.py`, after VAMP has been installed:
```bash
# choose robot name from {ur5, panda, fetch, baxter}
python resources/problem_tar_to_pkl_json.py --robot panda
```
This only needs to be run once.

### [Robometrics](https://github.com/fishbotics/robometrics)

We also provide support to load datasets from the [Robometrics](https://github.com/fishbotics/robometrics) into the same pickle and JSON format as above.
This requires installation of [robometrics](https://github.com/fishbotics/robometrics) and [geometrout](https://github.com/fishbotics/geometrout).
Robometrics has three datasets available for the Panda robot: `mpinets` (from [Motion Policy Networks](https://mpinets.github.io/) [[14]](../README.md#14), `mbm` (their version of the MotionBenchMaker dataset), and `demo`.
To convert these into a dataset format compatible with our evaluation script (`evaluate_mbm.py`), simply run:
```bash
# choose dataset name from {mpinets, mbm, demo}
python resources/robometrics_to_pkl_json.py --dataset mpinets
```
This only needs to be run once.
This will create the pickle file in the Panda robot directory `robometric_{dataset}_problems.pkl`, e.g., `robometric_mpinets_problems.pkl`.
These files can be passed to the `evaluate_mbm.py` and `visualize_mbm.py` scripts via the `--dataset` flag, e.g., `--dataset robometric_mpinets_problems.pkl`.
See [the script readme](../scripts/README.md) for more information.

:warning: Note that the Panda we ship uses the original MBM finger separation, which is at 6.5cm, while the robometric datasets assume the Panda to have the fingers separated at 2.5cm.
This leads to a number of problems in these datasets to be invalid, mostly in the tabletop domain, but others as well.

# Reference Timings
Here are reference timing outputs for `evaluate_mbm.py` using default planner settings.
The [arXiv submission](https://arxiv.org/abs/2309.14545) does not use dynamic-domain RRT-Connect and uses different default range parameters (e.g., 1.25 for the Panda, 1. for the Fetch), disable this feature with `--dynamic_domain False` and specify range with `--range`.

## `problems.tar.gz`: MotionBenchMaker Reference Timings

### [AMD Ryzen 9 7950X](https://www.amd.com/en/products/cpu/amd-ryzen-9-7950x)

#### UR5
```
|      |   Planning Time (μs) |   Simplification Time (μs) |   Total Time (μs) |   Planning Iters. |   Time per Iter. (μs) |
|------|----------------------|----------------------------|-------------------|-------------------|-----------------------|
| mean |              160.523 |                    65.7484 |           226.794 |           1422.93 |               7.50829 |
| std  |              495.287 |                    42.953  |           507.199 |           3953.06 |               6.50378 |
| min  |                5     |                     0      |             6     |              0    |               0       |
| 25%  |               22     |                    33      |            64     |             57    |               2.5     |
| 50%  |               39     |                    54      |           111.5   |            282.5  |               6.46304 |
| 75%  |               73.25  |                    93      |           174.25  |            788.5  |              10.8164  |
| 95%  |              776.05  |                   146.65   |           836.65  |           7335.65 |              18.7125  |
| max  |             6597     |                   216      |          6758     |          37261    |              45.2946  |
|      |    Initial Cost (L2) |       Simplified Cost (L2) |
|------|----------------------|----------------------------|
| mean |             11.2857  |                    6.765   |
| std  |              4.18102 |                    1.8886  |
| min  |              3.91149 |                    1.9205  |
| 25%  |              8.54816 |                    5.46554 |
| 50%  |             10.1798  |                    6.54791 |
| 75%  |             12.8846  |                    7.97097 |
| 95%  |             19.8162  |                   10.3212  |
| max  |             30.6832  |                   12.7789  |
Solved / Valid / Total # Problems: 608 / 608 / 700
Completed all problems in 137.891 milliseconds
Total time including Python overhead: 236.387 milliseconds
```

#### Panda
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

#### Fetch
```
|      |   Planning Time (μs) |   Simplification Time (μs) |   Total Time (μs) |   Planning Iters. |   Time per Iter. (μs) |
|------|----------------------|----------------------------|-------------------|-------------------|-----------------------|
| mean |              3928.32 |                    267.31  |           4196.13 |          22733.6  |              5.96809  |
| std  |              9163.32 |                    163.063 |           9250.1  |          66200.1  |              5.73922  |
| min  |                30    |                     21     |             85    |             61    |              0.947368 |
| 25%  |               197.25 |                    160     |            408    |            887.75 |              2.46695  |
| 50%  |               828    |                    221     |           1009.5  |           5014.5  |              4.00076  |
| 75%  |              3853.75 |                    331     |           4226.5  |          13919.5  |              7.21323  |
| 95%  |             16250.2  |                    614.95  |          16823.6  |          97944.2  |             17.0086   |
| max  |            107687    |                    978     |         108226    |         881990    |             44.5016   |
|      |    Initial Cost (L2) |       Simplified Cost (L2) |
|------|----------------------|----------------------------|
| mean |             20.1815  |                   10.4456  |
| std  |              7.90254 |                    3.17226 |
| min  |              5.8854  |                    4.18879 |
| 25%  |             14.5473  |                    8.19646 |
| 50%  |             18.7774  |                    9.92887 |
| 75%  |             23.7342  |                   12.0884  |
| 95%  |             35.0012  |                   16.4794  |
| max  |             54.7886  |                   24.4756  |
Solved / Valid / Total # Problems: 668 / 679 / 700
Completed all problems in 2803.015 milliseconds
Total time including Python overhead: 3152.705 milliseconds
```

### [Orange Pi 5B](http://www.orangepi.org/html/hardWare/computerAndMicrocontrollers/details/Orange-Pi-5B.html)

#### UR5
```
|      |   Planning Time (μs) |   Simplification Time (μs) |   Total Time (μs) |   Planning Iters. |   Time per Iter. (μs) |
|------|----------------------|----------------------------|-------------------|-------------------|-----------------------|
| mean |              811.938 |                    678.862 |           1491.31 |           1422.93 |              1.10058  |
| std  |             1867.21  |                    451.052 |           2032.44 |           3953.06 |              1.22912  |
| min  |               52     |                      0     |             53    |              0    |              0        |
| 25%  |              199     |                    352     |            617.75 |             57    |              0.250558 |
| 50%  |              325.5   |                    557.5   |            997.5  |            282.5  |              0.795186 |
| 75%  |              614.25  |                    943.25  |           1608.25 |            788.5  |              1.64015  |
| 95%  |             3333.9   |                   1508.8   |           4061.25 |           7335.65 |              2.70003  |
| max  |            24214     |                   3163     |          25902    |          37261    |             12.6757   |
|      |    Initial Cost (L2) |       Simplified Cost (L2) |
|------|----------------------|----------------------------|
| mean |             11.2808  |                    6.7649  |
| std  |              4.18714 |                    1.8885  |
| min  |              3.91149 |                    1.9205  |
| 25%  |              8.54816 |                    5.46554 |
| 50%  |             10.1798  |                    6.54791 |
| 75%  |             12.8846  |                    7.97097 |
| 95%  |             19.8162  |                   10.3212  |
| max  |             30.6832  |                   12.7789  |
Solved / Valid / Total # Problems: 608 / 608 / 700
Completed all problems in 906.717 milliseconds
Total time including Python overhead: 1202.020 milliseconds
```

#### Panda
```
|      |   Planning Time (μs) |   Simplification Time (μs) |   Total Time (μs) |   Planning Iters. |   Time per Iter. (μs) |
|------|----------------------|----------------------------|-------------------|-------------------|-----------------------|
| mean |              540.868 |                    646.763 |           1188.15 |           471.532 |              0.589179 |
| std  |              734.712 |                    492.967 |           1064.51 |          1970.67  |              0.950798 |
| min  |               58     |                      1     |             59    |             0     |              0        |
| 25%  |              185.5   |                    303.5   |            562    |            32     |              0.110234 |
| 50%  |              288     |                    558     |            881    |           104     |              0.32549  |
| 75%  |              539.5   |                    890     |           1412.5  |           293.5   |              0.628997 |
| 95%  |             1925.1   |                   1539.1   |           3089.8  |          1891.1   |              2.14658  |
| max  |             5251     |                   3259     |           7498    |         42794     |             11.1182   |
|      |    Initial Cost (L2) |       Simplified Cost (L2) |
|------|----------------------|----------------------------|
| mean |              8.65772 |                    5.17413 |
| std  |              3.36504 |                    1.28796 |
| min  |              2.78403 |                    2.78403 |
| 25%  |              6.38199 |                    4.34481 |
| 50%  |              7.76256 |                    4.90296 |
| 75%  |             10.159   |                    5.66869 |
| 95%  |             15.9191  |                    7.96949 |
| max  |             25.9764  |                   11.7948  |
Solved / Valid / Total # Problems: 699 / 699 / 700
Completed all problems in 830.516 milliseconds
Total time including Python overhead: 1167.840 milliseconds
```

#### Fetch
```
|      |   Planning Time (μs) |   Simplification Time (μs) |   Total Time (μs) |   Planning Iters. |   Time per Iter. (μs) |
|------|----------------------|----------------------------|-------------------|-------------------|-----------------------|
| mean |             15697.6  |                    2193.11 |           17891.3 |          22732.6  |              1.32665  |
| std  |             33161.8  |                    1220.46 |           33845.3 |          66200.2  |              1.55456  |
| min  |               213    |                     188    |             555   |             61    |              0.136659 |
| 25%  |              1183.25 |                    1373    |            2913.5 |            887.75 |              0.432675 |
| 50%  |              3945.5  |                    1903    |            5886   |           5014.5  |              0.796061 |
| 75%  |             16678.5  |                    2755.25 |           19367.5 |          13855.8  |              1.56687  |
| 95%  |             64682.3  |                    4600.75 |           67975.8 |          97944.2  |              4.05888  |
| max  |            385787    |                    7648    |          390221   |         881990    |             13.4917   |
|      |    Initial Cost (L2) |       Simplified Cost (L2) |
|------|----------------------|----------------------------|
| mean |              20.1832 |                   10.4427  |
| std  |               7.9018 |                    3.17499 |
| min  |               5.8854 |                    4.18879 |
| 25%  |              14.5473 |                    8.19811 |
| 50%  |              18.7984 |                    9.92887 |
| 75%  |              23.7342 |                   12.0884  |
| 95%  |              35.0012 |                   16.4937  |
| max  |              54.7886 |                   24.4756  |
Solved / Valid / Total # Problems: 679 / 679 / 700
Completed all problems in 11951.364 milliseconds
Total time including Python overhead: 13018.943 milliseconds
```

## Robometrics

All evaluation is done for the Panda robot on a [AMD Ryzen 9 7950X](https://www.amd.com/en/products/cpu/amd-ryzen-9-7950x) CPU.

### Demo Reference Timings

```
|      |   Planning Time (μs) |   Simplification Time (μs) |   Total Time (μs) |   Planning Iters. |   Time per Iter. (μs) |
|------|----------------------|----------------------------|-------------------|-------------------|-----------------------|
| mean |             108      |                    41.6    |          150.2    |            88.4   |             0.632888  |
| std  |              46.5779 |                    17.5014 |           61.6417 |           118.658 |             0.722068  |
| min  |              47      |                    12      |           60      |             3     |             0.0638298 |
| 25%  |              90      |                    42      |          135      |            11     |             0.0916667 |
| 50%  |             108      |                    45      |          162      |            17     |             0.188889  |
| 75%  |             120      |                    53      |          163      |           133     |             1.23148   |
| 95%  |             164      |                    55.4    |          217.4    |           249     |             1.51715   |
| max  |             175      |                    56      |          231      |           278     |             1.58857   |
|      |    Initial Cost (L2) |       Simplified Cost (L2) |
|------|----------------------|----------------------------|
| mean |              7.52892 |                    4.20916 |
| std  |              1.68975 |                    0.93178 |
| min  |              4.86816 |                    3.15659 |
| 25%  |              7.26151 |                    3.65876 |
| 50%  |              7.73403 |                    3.87349 |
| 75%  |              8.38051 |                    4.99107 |
| 95%  |              9.19645 |                    5.2909  |
| max  |              9.40043 |                    5.36586 |
Solved / Valid / Total # Problems: 5 / 5 / 5
Completed all problems in 0.751 milliseconds
Total time including Python overhead: 6.236 milliseconds
```

### Motion Policy Networks Reference Timings
```
|      |   Planning Time (μs) |   Simplification Time (μs) |   Total Time (μs) |   Planning Iters. |   Time per Iter. (μs) |
|------|----------------------|----------------------------|-------------------|-------------------|-----------------------|
| mean |              60.0528 |                    77.0484 |           137.63  |           81.8931 |              0.706297 |
| std  |              91.116  |                   119.659  |           196.073 |          274.129  |              1.33339  |
| min  |               1      |                     0      |             1     |            0      |              0        |
| 25%  |               8      |                     0      |             9     |            0      |              0        |
| 50%  |              25      |                    27      |            57     |            5      |              0.153454 |
| 75%  |              73.75   |                   110      |           191.75  |           70.75   |              0.843918 |
| 95%  |             240.2    |                   306      |           519     |          325.55   |              3.11035  |
| max  |            1145      |                  1081      |          1739     |         5051      |             12.2846   |
|      |    Initial Cost (L2) |       Simplified Cost (L2) |
|------|----------------------|----------------------------|
| mean |             5.44181  |                   3.52765  |
| std  |             3.58686  |                   1.46262  |
| min  |             0.343241 |                   0.343241 |
| 25%  |             3.00382  |                   2.6051   |
| 50%  |             4.20959  |                   3.31663  |
| 75%  |             7.06467  |                   4.17344  |
| 95%  |            13.0685   |                   6.3691   |
| max  |            25.5963   |                  11.8612   |
Solved / Valid / Total # Problems: 1590 / 1593 / 1800
Completed all problems in 218.831 milliseconds
Total time including Python overhead: 14764.528 milliseconds
```

### MotionBenchMaker Reference Timings
```
|      |   Planning Time (μs) |   Simplification Time (μs) |   Total Time (μs) |   Planning Iters. |   Time per Iter. (μs) |
|------|----------------------|----------------------------|-------------------|-------------------|-----------------------|
| mean |              113.079 |                    53.7238 |           167.339 |           180.325 |              0.960203 |
| std  |              183.779 |                    50.9234 |           218.671 |           431.763 |              1.20425  |
| min  |                5     |                     0      |             6     |             0     |              0        |
| 25%  |               27     |                    15.75   |            54     |             4     |              0.153428 |
| 50%  |               55.5   |                    39      |           109     |            34     |              0.584243 |
| 75%  |              111.25  |                    83      |           180     |           130.25  |              1.4212   |
| 95%  |              453.85  |                   155.05   |           621.1   |           895.5   |              3.03767  |
| max  |             1711     |                   355      |          1939     |          5110     |             12.109    |
|      |    Initial Cost (L2) |       Simplified Cost (L2) |
|------|----------------------|----------------------------|
| mean |              8.48574 |                    4.69235 |
| std  |              3.66998 |                    1.35691 |
| min  |              2.68362 |                    2.68362 |
| 25%  |              5.77894 |                    3.77079 |
| 50%  |              7.83714 |                    4.24839 |
| 75%  |             10.4593  |                    5.2624  |
| 95%  |             15.6194  |                    7.7228  |
| max  |             23.4322  |                    9.88084 |
Solved / Valid / Total # Problems: 800 / 800 / 800
Completed all problems in 133.871 milliseconds
Total time including Python overhead: 275.608 milliseconds
```
