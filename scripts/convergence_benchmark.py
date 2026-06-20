#!/usr/bin/env python3
"""Convergence benchmark comparing asymptotically optimal planners on sphere cage.

Runs each planner at increasing iteration budgets and plots cost vs wall-clock time.
Generates two figures:
  1. Deterministic (Halton) sampling — single run per budget.
  2. Multi-seed sampling — Halton with different skip offsets, mean +/- std over seeds.
"""

import numpy as np
import matplotlib.pyplot as plt
from fire import Fire
import vamp

# Sphere cage problem (from sphere_cage_example.py)
START = [0., -0.785, 0., -2.356, 0., 1.571, 0.785]
GOAL = [2.35, 1., 0., -0.8, 0, 2.5, 0.785]
SPHERES = [
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


def make_environment(radius = 0.2):
    e = vamp.Environment()
    for sphere in SPHERES:
        e.add_sphere(vamp.Sphere(sphere, radius))
    return e


def run_planner(planner_name, budget, env, start, goal, skip = 0):
    """Run a planner with a given iteration budget.

    Args:
        skip: Number of Halton samples to skip before planning (acts as seed).

    Returns (plan_time_ms, raw_cost, simp_cost, total_time_ms).
    """
    robot_module, planner_func, plan_settings, simp_settings = \
        vamp.configure_robot_and_planner_with_kwargs("panda", planner_name)

    plan_settings.max_iterations = budget
    plan_settings.max_samples = budget

    if hasattr(plan_settings, 'optimize'):
        plan_settings.optimize = True

    rng = vamp.panda.halton()
    if skip > 0:
        rng.skip(skip)
    result = planner_func(start, goal, env, plan_settings, rng)

    plan_time_ms = result.nanoseconds / 1e6
    raw_cost = result.path.cost() if result.solved else float('inf')

    if result.solved:
        simp_rng = vamp.panda.halton()
        simp_result = robot_module.simplify(result.path, env, simp_settings, simp_rng)
        simp_cost = simp_result.path.cost()
        total_time_ms = plan_time_ms + simp_result.nanoseconds / 1e6
    else:
        simp_cost = float('inf')
        total_time_ms = plan_time_ms

    return plan_time_ms, raw_cost, simp_cost, total_time_ms


def plot_deterministic(planner_list, budgets, env, start, goal, max_time_s, output):
    """Single deterministic Halton run per budget."""
    results = {}

    for planner_name in planner_list:
        print(f"\n=== {planner_name} (deterministic) ===")
        results[planner_name] = {}

        for budget in budgets:
            plan_t, raw_c, simp_c, total_t = run_planner(planner_name, budget, env, start, goal)

            results[planner_name][budget] = (plan_t, raw_c, simp_c, total_t)

            raw_s = f"{raw_c:.4f}" if raw_c < float('inf') else "FAIL"
            simp_s = f"{simp_c:.4f}" if simp_c < float('inf') else "FAIL"
            print(f"  budget={budget:>7d}  {plan_t:>7.0f}ms  raw={raw_s}  simp={simp_s}")

            if plan_t / 1000 > max_time_s:
                print(f"  -> {plan_t/1000:.1f}s > {max_time_s}s limit, stopping")
                break

    fig, axes = plt.subplots(1, 2, figsize = (14, 6))
    colors = plt.cm.tab10.colors

    for idx, planner_name in enumerate(planner_list):
        color = colors[idx % len(colors)]
        data = results[planner_name]

        raw_times = []
        raw_costs = []
        for b in sorted(data.keys()):
            pt, rc, _, _ = data[b]
            if rc < float('inf'):
                raw_times.append(pt)
                raw_costs.append(rc)
        if raw_times:
            axes[0].plot(
                raw_times,
                raw_costs,
                '-o',
                color = color,
                label = planner_name,
                linewidth = 2,
                markersize = 4
                )

        simp_times = []
        simp_costs = []
        for b in sorted(data.keys()):
            _, _, sc, tt = data[b]
            if sc < float('inf'):
                simp_times.append(tt)
                simp_costs.append(sc)
        if simp_times:
            axes[1].plot(
                simp_times,
                simp_costs,
                '-o',
                color = color,
                label = planner_name,
                linewidth = 2,
                markersize = 4
                )

    for ax, title in zip(axes, ["Raw Planner Cost", "After Simplification"]):
        ax.set_xlabel("Time (ms)", fontsize = 12)
        ax.set_ylabel("Path Cost (L2)", fontsize = 12)
        ax.set_title(title, fontsize = 14)
        ax.set_xscale("log")
        ax.legend(fontsize = 11)
        ax.grid(True, alpha = 0.3)

    fig.suptitle("Convergence: Sphere Cage — Deterministic (Halton)", fontsize = 15, y = 1.01)
    plt.tight_layout()
    plt.savefig(output, dpi = 150, bbox_inches = "tight")
    print(f"\nDeterministic plot saved to {output}")
    plt.close(fig)


def plot_multi_seed(planner_list, budgets, env, start, goal, max_time_s, n_seeds, output):
    """Multi-seed runs using Halton with different skip offsets."""
    # Use well-separated skip offsets so Halton subsequences don't overlap
    skip_offsets = [i * 100000 for i in range(n_seeds)]

    # results[planner][budget] = list of (plan_t, raw_c, simp_c, total_t) per seed
    results = {}

    for planner_name in planner_list:
        print(f"\n=== {planner_name} (multi-seed, {n_seeds} seeds) ===")
        results[planner_name] = {}
        stopped_at_budget = None

        for budget in budgets:
            if stopped_at_budget is not None:
                break

            seed_results = []
            for seed_idx, skip in enumerate(skip_offsets):
                plan_t, raw_c, simp_c, total_t = run_planner(
                    planner_name, budget, env, start, goal, skip=skip)
                seed_results.append((plan_t, raw_c, simp_c, total_t))

            results[planner_name][budget] = seed_results

            raw_costs = [r[1] for r in seed_results if r[1] < float('inf')]
            plan_times = [r[0] for r in seed_results]
            n_solved = len(raw_costs)

            if raw_costs:
                print(
                    f"  budget={budget:>7d}  {np.mean(plan_times):>7.0f}ms  "
                    f"raw={np.mean(raw_costs):.4f}±{np.std(raw_costs):.4f}  "
                    f"solved={n_solved}/{n_seeds}"
                    )
            else:
                print(
                    f"  budget={budget:>7d}  {np.mean(plan_times):>7.0f}ms  "
                    f"raw=FAIL  solved=0/{n_seeds}"
                    )

            if np.mean(plan_times) / 1000 > max_time_s:
                print(f"  -> {np.mean(plan_times)/1000:.1f}s > {max_time_s}s limit, stopping")
                stopped_at_budget = budget

    fig, axes = plt.subplots(1, 2, figsize = (14, 6))
    colors = plt.cm.tab10.colors

    for idx, planner_name in enumerate(planner_list):
        color = colors[idx % len(colors)]
        data = results[planner_name]

        # Collect per-budget statistics for raw cost
        raw_mean_times = []
        raw_mean_costs = []
        raw_std_costs = []
        for b in sorted(data.keys()):
            costs = [r[1] for r in data[b] if r[1] < float('inf')]
            times = [r[0] for r in data[b] if r[1] < float('inf')]
            if costs:
                raw_mean_times.append(np.mean(times))
                raw_mean_costs.append(np.mean(costs))
                raw_std_costs.append(np.std(costs))

        if raw_mean_times:
            raw_mean_costs = np.array(raw_mean_costs)
            raw_std_costs = np.array(raw_std_costs)
            raw_mean_times = np.array(raw_mean_times)
            axes[0].plot(
                raw_mean_times,
                raw_mean_costs,
                '-o',
                color = color,
                label = planner_name,
                linewidth = 2,
                markersize = 4
                )
            axes[0].fill_between(
                raw_mean_times,
                raw_mean_costs - raw_std_costs,
                raw_mean_costs + raw_std_costs,
                color = color,
                alpha = 0.15
                )

        # Collect per-budget statistics for simplified cost
        simp_mean_times = []
        simp_mean_costs = []
        simp_std_costs = []
        for b in sorted(data.keys()):
            costs = [r[2] for r in data[b] if r[2] < float('inf')]
            times = [r[3] for r in data[b] if r[2] < float('inf')]
            if costs:
                simp_mean_times.append(np.mean(times))
                simp_mean_costs.append(np.mean(costs))
                simp_std_costs.append(np.std(costs))

        if simp_mean_times:
            simp_mean_costs = np.array(simp_mean_costs)
            simp_std_costs = np.array(simp_std_costs)
            simp_mean_times = np.array(simp_mean_times)
            axes[1].plot(
                simp_mean_times,
                simp_mean_costs,
                '-o',
                color = color,
                label = planner_name,
                linewidth = 2,
                markersize = 4
                )
            axes[1].fill_between(
                simp_mean_times,
                simp_mean_costs - simp_std_costs,
                simp_mean_costs + simp_std_costs,
                color = color,
                alpha = 0.15
                )

    for ax, title in zip(axes, ["Raw Planner Cost", "After Simplification"]):
        ax.set_xlabel("Time (ms)", fontsize = 12)
        ax.set_ylabel("Path Cost (L2)", fontsize = 12)
        ax.set_title(title, fontsize = 14)
        ax.set_xscale("log")
        ax.legend(fontsize = 11)
        ax.grid(True, alpha = 0.3)

    fig.suptitle(f"Convergence: Sphere Cage — {n_seeds} Seeds (Halton skip offsets)", fontsize = 15, y = 1.01)
    plt.tight_layout()
    plt.savefig(output, dpi = 150, bbox_inches = "tight")
    print(f"\nMulti-seed plot saved to {output}")
    plt.close(fig)


def main(
    output: str = "convergence",
    planners: str = "grrtstar,aorrtc,fcit",
    radius: float = 0.2,
    max_time_s: float = 10.,
    n_seeds: int = 5,
    ):
    """Run convergence benchmark.

    Generates two figures:
      {output}_deterministic.png — single Halton run per budget.
      {output}_multi_seed.png   — mean±std over n_seeds Halton skip offsets.

    Args:
        output: Output filename prefix (without extension).
        planners: Comma-separated planner names.
        radius: Sphere obstacle radius.
        max_time_s: Approximate max wall-clock time per planner point (seconds).
        n_seeds: Number of seeds for multi-seed benchmark.
    """
    if isinstance(planners, (list, tuple)):
        planner_list = list(planners)
    else:
        planner_list = [p.strip() for p in planners.split(",")]

    budgets = [
        500,
        1000,
        2000,
        3000,
        5000,
        7500,
        10000,
        15000,
        20000,
        30000,
        50000,
        75000,
        100000,
        150000,
        200000,
        300000,
        500000,
        ]

    env = make_environment(radius)
    start = np.array(START)
    goal = np.array(GOAL)

    assert vamp.panda.validate(start, env), "Start config in collision!"
    assert vamp.panda.validate(goal, env), "Goal config in collision!"

    # Strip extension if user provided one
    if output.endswith('.png'):
        output = output[:-4]

    plot_deterministic(planner_list, budgets, env, start, goal, max_time_s, f"{output}_deterministic.png")

    plot_multi_seed(planner_list, budgets, env, start, goal, max_time_s, n_seeds, f"{output}_multi_seed.png")


if __name__ == "__main__":
    Fire(main)
