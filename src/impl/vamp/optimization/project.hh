#pragma once

#include <cmath>
#include <array>
#include <vector>

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>

namespace vamp::optimization
{
    template <typename Robot, std::size_t rake = vamp::FloatVectorWidth>
    inline auto project_to_valid(
        const typename Robot::template ConfigurationBlock<rake> &c_in,
        const vamp::collision::Environment<vamp::FloatVector<rake>> &env,
        float alpha = 0.1f,
        int max_iters = 100) -> typename Robot::template ConfigurationBlock<rake>
    {
        typename Robot::template ConfigurationBlock<rake> b = c_in;
        float current_min_dist = -1e9f;
        int iter = 0;

        while (current_min_dist < 0 && iter < max_iters)
        {
            auto res = Robot::sdf_gradient(env, b);

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

            typename Robot::template ConfigurationBlock<rake> dq_block;
            Robot::d_collision_d_q(b, res.second, dq_block);

            std::array<float, rake> norms;
            for (auto j = 0U; j < rake; j++)
            {
                float sum = 0;
                for (auto i = 0U; i < Robot::dimension; i++)
                {
                    sum += dq_block[i].element(j) * dq_block[i].element(j);
                }
                norms[j] = std::sqrt(sum);
            }
            auto max_norm = *std::max_element(norms.begin(), norms.end());
            std::array<float, rake * Robot::dimension> data;
            if (max_norm > 1e-6f)
            {
                for (auto i = 0U; i < rake; ++i)
                {
                    if (norms[i] > 1e-6f)
                    {
                        for (auto j = 0U; j < Robot::dimension; ++j)
                        {
                            data[j * rake + i] = dq_block[j].element(i) / norms[i] * alpha;
                        }
                    }
                    else
                    {
                        for (auto j = 0U; j < Robot::dimension; ++j)
                        {
                            data[j * rake + i] = 0;
                        }
                    }
                }
                auto dq_block_normalized = typename Robot::template ConfigurationBlock<rake>(data);
                b = b + dq_block_normalized;
            }
            else
            {
                break;
            }
            iter++;
        }
        return b;
    }
}  // namespace vamp::optimization
