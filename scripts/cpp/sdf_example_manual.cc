#include <chrono>
#include <vector>
#include <array>
#include <utility>
#include <iostream>

#include <vamp/collision/factory.hh>
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {
    {0.55, 0, 0.25},
    {0.35, 0.35, 0.25},
    {0, 0.55, 0.25},
    {-0.55, 0, 0.25},
    {-0.35, -0.35, 0.25},
    {0, -0.55, 0.25},
    {0.35, -0.35, 0.25},
    {-0.35, 0.35, 0.25},
    {0.35, 0.35, 0.8},
    {0, 0.55, 0.8},
    {-0.35, 0.35, 0.8},
    {-0.55, 0, 0.8},
    {-0.35, -0.35, 0.8},
    {0, -0.55, 0.8},
    {0.35, -0.35, 0.8},
    {0.55, 0, 0.8},
};

// Radius for obstacle spheres
static constexpr float radius = 0.2;

auto main(int, char **) -> int
{
    // Build sphere cage environment
    EnvironmentInput environment;
    for (const auto &sphere : problem)
    {
        environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    }

    environment.sort();
    auto env_v = EnvironmentVector(environment);

    // Benchmark
    vamp::rng::Halton<Robot> sampler;
    int n_samples = 1000;
    int n_success = 0;
    double total_time_ms = 0.0;
    int total_iter = 0;
    std::cout << "Starting Benchmark with " << n_samples << " samples..." << std::endl;
    int i = 0;
    while (i < n_samples)
    {
        auto q_random = sampler.next();
        // std::cout << "Sample " << i << ": " << q_random << std::endl;
        Robot::ConfigurationBlock<rake> b;
        for (auto k = 0U; k < Robot::dimension; ++k)
        {
            b[k] = q_random.broadcast(k);
        }
        auto valid = Robot::fkcc<rake>(env_v, b);
        if (valid)
        {
            continue;
        }
        i++;

        // Project
        auto start_t = std::chrono::steady_clock::now();

        float current_min_dist = -1e9f;
        int iter = 0;
        const int max_iters = 100;
        float alpha = 0.1f;

        while (current_min_dist < 0 && iter < max_iters)
        {
            // Re-evaluate
            auto res = Robot::sdf_gradient(env_v, b);

            auto dists_arr = res.first.to_array();
            current_min_dist = 1e9f;
            for (auto d : dists_arr)
            {
                if (d < current_min_dist)
                {
                    current_min_dist = d;
                }
            }

            if (current_min_dist >= 0)
            {
                break;
            }

            Robot::ConfigurationBlock<rake> dq_block;
            Robot::d_collision_d_q(b, res.second, dq_block);  // b is already the block for q_new
            std::vector<float> dq(Robot::dimension);
            for (auto k = 0U; k < Robot::dimension; ++k)
            {
                dq[k] = dq_block[k].element(0);
            }

            float dq_norm = 0.0f;
            for (float v : dq)
            {
                dq_norm += v * v;
            }
            dq_norm = std::sqrt(dq_norm);

            if (dq_norm > std::numeric_limits<float>::epsilon())
            {
                for (auto k = 0U; k < Robot::dimension; ++k)
                {
                    b[k] = b[k] + alpha * (dq[k] / dq_norm);
                }
            }
            else
            {
                break;
            }
            iter++;
        }
        auto end_t = std::chrono::steady_clock::now();
        auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(end_t - start_t);
        auto b_final = b;
        valid = Robot::fkcc<rake>(env_v, b_final);
        if (valid)
        {
            n_success++;
        }
        total_time_ms += dur.count() / 1e6;
        total_iter += iter;
    }

    std::cout << "Benchmark Results:" << std::endl;
    std::cout << "Total Samples: " << n_samples << std::endl;
    std::cout << "Successful Projections: " << n_success << " ("
              << (n_samples > 0 ? (100.0 * n_success / n_samples) : 0.0) << "%)" << std::endl;
    if (n_success > 0)
    {
        std::cout << "Average Projection Time: " << (total_time_ms / n_samples) << " ms" << std::endl;
        std::cout << "Average Iterations: " << (total_iter / n_samples) << std::endl;
    }

    return 0;
}
