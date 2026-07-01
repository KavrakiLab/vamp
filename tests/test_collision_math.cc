#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <vamp/collision/math.hh>
#include <vamp/collision/sphere_sphere.hh>

using Catch::Approx;

TEST_CASE("dot_2 computes 2D dot product", "[collision][math]")
{
    REQUIRE(vamp::collision::dot_2<float>(1.F, 2.F, 3.F, 4.F) == Approx(11.F));
    REQUIRE(vamp::collision::dot_2<float>(0.F, 0.F, 5.F, 5.F) == Approx(0.F));
}

TEST_CASE("dot_3 computes 3D dot product", "[collision][math]")
{
    REQUIRE(vamp::collision::dot_3<float>(1.F, 2.F, 3.F, 4.F, 5.F, 6.F) == Approx(32.F));
}

TEST_CASE("sql2_3 computes squared distance between two points", "[collision][math]")
{
    REQUIRE(vamp::collision::sql2_3<float>(0.F, 0.F, 0.F, 3.F, 4.F, 0.F) == Approx(25.F));
    REQUIRE(vamp::collision::sql2_3<float>(1.F, 1.F, 1.F, 1.F, 1.F, 1.F) == Approx(0.F));
}

TEST_CASE("clamp restricts a value to a range", "[collision][math]")
{
    REQUIRE(vamp::collision::clamp<float>(5.F, 0.F, 10.F) == Approx(5.F));
    REQUIRE(vamp::collision::clamp<float>(-5.F, 0.F, 10.F) == Approx(0.F));
    REQUIRE(vamp::collision::clamp<float>(15.F, 0.F, 10.F) == Approx(10.F));
}

TEST_CASE("sqrt matches std::sqrt for scalars", "[collision][math]")
{
    REQUIRE(vamp::collision::sqrt<float>(4.F) == Approx(2.F));
    REQUIRE(vamp::collision::sqrt<float>(2.F) == Approx(std::sqrt(2.F)));
}

TEST_CASE("sphere_sphere_sql2 is negative for overlapping spheres", "[collision][sphere]")
{
    // Two spheres of radius 1 centered 1 unit apart overlap.
    const float d2 = vamp::collision::sphere_sphere_sql2<float>(0.F, 0.F, 0.F, 1.F, 1.F, 0.F, 0.F, 1.F);
    REQUIRE(d2 < 0.F);
}

TEST_CASE("sphere_sphere_sql2 is positive for separated spheres", "[collision][sphere]")
{
    // Two spheres of radius 1 centered 10 units apart do not overlap.
    const float d2 = vamp::collision::sphere_sphere_sql2<float>(0.F, 0.F, 0.F, 1.F, 10.F, 0.F, 0.F, 1.F);
    REQUIRE(d2 > 0.F);
}

TEST_CASE("sphere_sphere_sql2 is zero for exactly touching spheres", "[collision][sphere]")
{
    const float d2 = vamp::collision::sphere_sphere_sql2<float>(0.F, 0.F, 0.F, 1.F, 2.F, 0.F, 0.F, 1.F);
    REQUIRE(d2 == Approx(0.F).margin(1e-5));
}
