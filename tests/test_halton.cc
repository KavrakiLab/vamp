#include <catch2/catch_test_macros.hpp>

#include <vamp/collision/validity.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/sphere.hh>

using vamp::robots::Sphere;

namespace
{
    // Sphere's configuration space bounds, mirrored from src/impl/vamp/robots/sphere.hh.
    constexpr std::array<float, 3> lows{-10.F, -10.F, 0.F};
    constexpr std::array<float, 3> highs{10.F, 10.F, 5.F};
}  // namespace

TEST_CASE("Halton sequence stays within the robot's configuration bounds", "[random][halton]")
{
    vamp::rng::Halton<Sphere> halton;

    for (auto i = 0U; i < 100U; ++i)
    {
        const auto config = halton.next();
        for (auto d = 0U; d < Sphere::dimension; ++d)
        {
            REQUIRE(config.element(d) >= lows[d]);
            REQUIRE(config.element(d) <= highs[d]);
        }
    }
}

TEST_CASE("Halton sequence is deterministic across independent instances", "[random][halton]")
{
    vamp::rng::Halton<Sphere> a;
    vamp::rng::Halton<Sphere> b;

    for (auto i = 0U; i < 20U; ++i)
    {
        const auto ca = a.next();
        const auto cb = b.next();
        for (auto d = 0U; d < Sphere::dimension; ++d)
        {
            REQUIRE(ca.element(d) == cb.element(d));
        }
    }
}

TEST_CASE("Halton reset() restarts the sequence from the beginning", "[random][halton]")
{
    vamp::rng::Halton<Sphere> halton;

    const auto first = halton.next();
    halton.next();
    halton.next();

    halton.reset();
    const auto after_reset = halton.next();

    for (auto d = 0U; d < Sphere::dimension; ++d)
    {
        REQUIRE(first.element(d) == after_reset.element(d));
    }
}
