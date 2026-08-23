#include <array>
#include <cmath>
#include <random>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include <vamp/collision/capt.hh>

namespace
{
    using vamp::collision::CAPT;
    using vamp::collision::Point;

    // CAPT::collides is documented to report whether a sphere of radius r
    // centered at c intersects any stored point grown by r_point. Brute force
    // over the same input points is the ground truth.
    auto brute_force(const std::vector<Point> &points, const Point &c, float r, float r_point)
        -> bool
    {
        const float threshold = r + r_point;
        for (const auto &p : points)
        {
            const float dx = p[0] - c[0];
            const float dy = p[1] - c[1];
            const float dz = p[2] - c[2];
            if (std::sqrt(dx * dx + dy * dy + dz * dz) <= threshold)
            {
                return true;
            }
        }

        return false;
    }

    struct Counts
    {
        // A false negative is a soundness failure: the tree reports clear where
        // a real overlap exists, so a planner would accept a colliding state.
        int false_negatives = 0;
        int overlaps = 0;
        int simd_disagreements = 0;
    };

    auto check_queries(
        const std::vector<Point> &points,
        float r_min,
        float r_max,
        float r_point,
        const std::vector<std::array<float, 4>> &queries) -> Counts
    {
        const CAPT capt(points, r_min, r_max, r_point);
        REQUIRE(capt.is_valid());

        Counts counts;
        for (const auto &query : queries)
        {
            const Point center{query[0], query[1], query[2]};
            const float r = query[3];

            const bool expected = brute_force(points, center, r, r_point);
            const bool actual = capt.collides(center, r);

            if (expected)
            {
                counts.overlaps++;
                if (not actual)
                {
                    counts.false_negatives++;
                }
            }

            // The SIMD path broadcasts one sphere into every lane, so it must
            // agree with the scalar path.
            using FVectorT = CAPT::FVectorT;
            const std::array<FVectorT, 3> centers = {
                FVectorT::fill(center[0]), FVectorT::fill(center[1]), FVectorT::fill(center[2])};
            if (capt.collides_simd(centers, FVectorT::fill(r)) != actual)
            {
                counts.simd_disagreements++;
            }
        }

        return counts;
    }

    // Panda's sphere-radius bounds with r_point for a 5 cm occupancy grid.
    constexpr float r_min = 0.012F;
    constexpr float r_max = 0.08F;
    constexpr float r_point = 0.05F;
}  // namespace

TEST_CASE("CAPT reports every overlap for a uniform cloud", "[collision][capt]")
{
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> position(-0.6F, 0.6F);
    // Strictly inside [r_min, r_max]: every query is within the documented contract.
    std::uniform_real_distribution<float> radius(r_min + 1e-4F, r_max - 1e-4F);

    std::vector<Point> points;
    points.reserve(700);
    for (auto i = 0u; i < 700u; i++)
    {
        points.push_back({position(rng), position(rng), position(rng)});
    }

    std::vector<std::array<float, 4>> queries;
    queries.reserve(20000);
    for (auto i = 0u; i < 20000u; i++)
    {
        queries.push_back({position(rng), position(rng), position(rng), radius(rng)});
    }

    const auto counts = check_queries(points, r_min, r_max, r_point, queries);

    REQUIRE(counts.overlaps > 0);
    CHECK(counts.false_negatives == 0);
    CHECK(counts.simd_disagreements == 0);
}

TEST_CASE("CAPT reports every overlap for points on cell boundaries", "[collision][capt]")
{
    // A regular grid puts many points exactly on the median split planes, which
    // is where affordance propagation across a split is most easily lost.
    std::vector<Point> points;
    points.reserve(12 * 12 * 12);
    for (auto i = 0u; i < 12u; i++)
    {
        for (auto j = 0u; j < 12u; j++)
        {
            for (auto k = 0u; k < 12u; k++)
            {
                points.push_back(
                    {static_cast<float>(i) * 0.05F,
                     static_cast<float>(j) * 0.05F,
                     static_cast<float>(k) * 0.05F});
            }
        }
    }

    std::mt19937 rng(999);
    std::uniform_real_distribution<float> position(-0.1F, 0.7F);
    std::uniform_real_distribution<float> radius(r_min, r_max);

    std::vector<std::array<float, 4>> queries;
    queries.reserve(20000);
    for (auto i = 0u; i < 20000u; i++)
    {
        queries.push_back({position(rng), position(rng), position(rng), radius(rng)});
    }

    const auto counts = check_queries(points, r_min, r_max, r_point, queries);

    REQUIRE(counts.overlaps > 0);
    CHECK(counts.false_negatives == 0);
    CHECK(counts.simd_disagreements == 0);
}

TEST_CASE("CAPT reports every overlap when r_point is zero", "[collision][capt]")
{
    // With r_point = 0 the affordance band is exactly r_max, so this isolates
    // affordance propagation from the point-radius padding.
    std::mt19937 rng(4242);
    std::uniform_real_distribution<float> position(-0.5F, 0.5F);
    std::uniform_real_distribution<float> radius(0.01F, 0.06F);

    std::vector<Point> points;
    points.reserve(500);
    for (auto i = 0u; i < 500u; i++)
    {
        points.push_back({position(rng), position(rng), position(rng)});
    }

    std::vector<std::array<float, 4>> queries;
    queries.reserve(20000);
    for (auto i = 0u; i < 20000u; i++)
    {
        queries.push_back({position(rng), position(rng), position(rng), radius(rng)});
    }

    const auto counts = check_queries(points, 0.01F, 0.06F, 0.0F, queries);

    REQUIRE(counts.overlaps > 0);
    CHECK(counts.false_negatives == 0);
    CHECK(counts.simd_disagreements == 0);
}

TEST_CASE("CAPT reports every overlap for small clouds", "[collision][capt]")
{
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> position(-0.3F, 0.3F);
    std::uniform_real_distribution<float> radius(r_min, r_max);

    // Small clouds exercise shallow trees, including the single-point tree that
    // needs no test buffer at all.
    for (const auto n : {1u, 2u, 3u, 5u, 8u})
    {
        std::vector<Point> points;
        points.reserve(n);
        for (auto i = 0u; i < n; i++)
        {
            points.push_back({position(rng), position(rng), position(rng)});
        }

        std::vector<std::array<float, 4>> queries;
        queries.reserve(4000);
        for (auto i = 0u; i < 4000u; i++)
        {
            queries.push_back({position(rng), position(rng), position(rng), radius(rng)});
        }

        const auto counts = check_queries(points, r_min, r_max, r_point, queries);

        REQUIRE(counts.overlaps > 0);
        CHECK(counts.false_negatives == 0);
        CHECK(counts.simd_disagreements == 0);
    }
}
