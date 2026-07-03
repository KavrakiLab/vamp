#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <vamp/vector.hh>

using Catch::Approx;
using vamp::FloatVector;
using vamp::FloatVectorWidth;

TEST_CASE("FloatVector round-trips scalar data", "[vector]")
{
    const FloatVector<3> v(std::array<float, 3>{1.F, 2.F, 3.F});
    REQUIRE(v.element(0) == Approx(1.F));
    REQUIRE(v.element(1) == Approx(2.F));
    REQUIRE(v.element(2) == Approx(3.F));
}

TEST_CASE("FloatVector addition and subtraction are elementwise", "[vector]")
{
    const FloatVector<3> a(std::array<float, 3>{1.F, 2.F, 3.F});
    const FloatVector<3> b(std::array<float, 3>{4.F, 5.F, 6.F});

    const auto sum = a + b;
    REQUIRE(sum.element(0) == Approx(5.F));
    REQUIRE(sum.element(1) == Approx(7.F));
    REQUIRE(sum.element(2) == Approx(9.F));

    const auto diff = b - a;
    REQUIRE(diff.element(0) == Approx(3.F));
    REQUIRE(diff.element(1) == Approx(3.F));
    REQUIRE(diff.element(2) == Approx(3.F));
}

TEST_CASE("FloatVector scalar multiplication scales every element", "[vector]")
{
    const FloatVector<3> a(std::array<float, 3>{1.F, -2.F, 3.F});
    const auto scaled = a * 2.F;
    REQUIRE(scaled.element(0) == Approx(2.F));
    REQUIRE(scaled.element(1) == Approx(-4.F));
    REQUIRE(scaled.element(2) == Approx(6.F));
}

TEST_CASE("FloatVector l2_norm computes Euclidean length", "[vector]")
{
    const FloatVector<3> v(std::array<float, 3>{3.F, 4.F, 0.F});
    REQUIRE(v.l2_norm() == Approx(5.F));
    REQUIRE(v.squared_l2_norm() == Approx(25.F));
}

TEST_CASE("FloatVector distance matches l2_norm of the difference", "[vector]")
{
    const FloatVector<3> a(std::array<float, 3>{0.F, 0.F, 0.F});
    const FloatVector<3> b(std::array<float, 3>{3.F, 4.F, 0.F});
    REQUIRE(a.distance(b) == Approx(5.F));
}

TEST_CASE("FloatVector interpolate blends linearly between endpoints", "[vector]")
{
    const FloatVector<3> a(std::array<float, 3>{0.F, 0.F, 0.F});
    const FloatVector<3> b(std::array<float, 3>{10.F, 20.F, 30.F});

    const auto start = a.interpolate(b, 0.F);
    REQUIRE(start.element(0) == Approx(0.F));

    const auto end = a.interpolate(b, 1.F);
    REQUIRE(end.element(1) == Approx(20.F));

    const auto mid = a.interpolate(b, 0.5F);
    REQUIRE(mid.element(0) == Approx(5.F));
    REQUIRE(mid.element(1) == Approx(10.F));
    REQUIRE(mid.element(2) == Approx(15.F));
}

TEST_CASE("FloatVector clamp restricts elements to bounds", "[vector]")
{
    const FloatVector<3> v(std::array<float, 3>{-5.F, 5.F, 15.F});
    const auto clamped = v.clamp(0.F, 10.F);
    REQUIRE(clamped.element(0) == Approx(0.F));
    REQUIRE(clamped.element(1) == Approx(5.F));
    REQUIRE(clamped.element(2) == Approx(10.F));
}

TEST_CASE("FloatVector hsum sums a fully-packed vector", "[vector]")
{
    // Use a full-width vector so there are no padding lanes to worry about.
    std::array<float, FloatVectorWidth> data;
    data.fill(1.F);
    const FloatVector<FloatVectorWidth> v(data);
    REQUIRE(v.hsum() == Approx(static_cast<float>(FloatVectorWidth)));
}

TEST_CASE("FloatVector comparisons report all/any/none over a full-width vector", "[vector]")
{
    std::array<float, FloatVectorWidth> data;
    data.fill(2.F);
    const FloatVector<FloatVectorWidth> v(data);

    REQUIRE((v == 2.F).all());
    REQUIRE((v > 1.F).all());
    REQUIRE((v < 1.F).none());
    REQUIRE((v > 1.F).any());
}
