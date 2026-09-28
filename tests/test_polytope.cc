#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Geometry>

#include <vamp/vector.hh>
#include <vamp/collision/factory.hh>
#include <vamp/collision/gjk.hh>
#include <vamp/collision/sphere_polytope.hh>
#include <vamp/collision/validity.hh>

using Catch::Approx;
using vamp::FloatVector;
using vamp::FloatVectorWidth;
namespace vc = vamp::collision;

namespace
{
    using Vertices = std::vector<std::array<float, 3>>;
    using V = FloatVector<FloatVectorWidth>;

    auto make_polytope(const Vertices &points) -> vc::ConvexPolytope<float>
    {
        std::vector<float> vx, vy, vz;
        for (const auto &[x, y, z] : points)
        {
            vx.push_back(x);
            vy.push_back(y);
            vz.push_back(z);
        }

        // GJK only needs the vertices, so skip the halfspace representation
        return vc::ConvexPolytope<float>(0, {}, {}, {}, {}, points.size(), vx, vy, vz);
    }

    auto box(const Eigen::AlignedBox3f &aabb) -> Vertices
    {
        Vertices corners;
        for (auto i = 0U; i < 8; ++i)
        {
            const Eigen::Vector3f c = aabb.corner(static_cast<Eigen::AlignedBox3f::CornerType>(i));
            corners.push_back({c.x(), c.y(), c.z()});
        }

        return corners;
    }

    auto cube() -> Vertices
    {
        return box({Eigen::Vector3f::Constant(-1.F), Eigen::Vector3f::Constant(1.F)});
    }

    auto tetrahedron() -> Vertices
    {
        return {{0.F, 0.F, 0.F}, {1.F, 0.F, 0.F}, {0.F, 1.F, 0.F}, {0.F, 0.F, 1.F}};
    }

    // Exact squared distance from a point to the convex hull of `points`
    auto sql2_to(const Vertices &points, float x, float y, float z) -> float
    {
        const auto p = make_polytope(points);
        return vc::gjk::sql2(
            p.vx.data(),
            p.vy.data(),
            p.vz.data(),
            p.num_vertices,
            Eigen::Vector3f(x, y, z),
            Eigen::Vector3f(p.vx[0], p.vy[0], p.vz[0]));
    }

    // Collision of a single sphere via the full SIMD path (every lane holds the same sphere)
    auto collides_ball(const vc::ConvexPolytope<V> &p, float x, float y, float z, float r) -> bool
    {
        return not vc::sphere_polytope(p, V::fill(x), V::fill(y), V::fill(z), V::fill(r)).test_zero();
    }

    auto collides_ball(const Vertices &points, float x, float y, float z, float r) -> bool
    {
        return collides_ball(vc::ConvexPolytope<V>(make_polytope(points)), x, y, z, r);
    }
}  // namespace

TEST_CASE("GJK distance from a point inside a cube is zero", "[collision][polytope]")
{
    REQUIRE(sql2_to(cube(), 0.F, 0.F, 0.F) == 0.F);
    REQUIRE(sql2_to(cube(), 0.5F, -0.5F, 0.2F) == 0.F);
}

TEST_CASE("GJK distance to cube face and vertex", "[collision][polytope]")
{
    REQUIRE(sql2_to(cube(), 2.F, 0.F, 0.F) == Approx(1.F));
    REQUIRE(sql2_to(cube(), 2.F, 2.F, 2.F) == Approx(3.F));
}

TEST_CASE("GJK distance to degenerate polytopes", "[collision][polytope]")
{
    REQUIRE(sql2_to({{1.F, 2.F, 3.F}}, 1.F, 2.F, 0.F) == Approx(9.F));
    REQUIRE(sql2_to({{0.F, 0.F, 0.F}, {2.F, 0.F, 0.F}}, 1.F, 1.F, 0.F) == Approx(1.F));

    const Vertices triangle = {{0.F, 0.F, 0.F}, {1.F, 0.F, 0.F}, {0.F, 1.F, 0.F}};
    REQUIRE(sql2_to(triangle, 0.F, 0.F, 1.F) == Approx(1.F));
    REQUIRE(sql2_to(triangle, 0.25F, 0.25F, 1.F) == Approx(1.F));
}

TEST_CASE("GJK distance to tetrahedron", "[collision][polytope]")
{
    REQUIRE(sql2_to(tetrahedron(), 0.25F, 0.25F, 0.25F) == 0.F);
    REQUIRE(sql2_to(tetrahedron(), 10.F, 10.F, 10.F) == Approx(841.F / 3.F));
}

TEST_CASE("GJK distance to no vertices is infinite", "[collision][polytope]")
{
    REQUIRE(
        std::isinf(
            vc::gjk::sql2(nullptr, nullptr, nullptr, 0, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero())));
}

TEST_CASE("Sphere-polytope collision boundaries", "[collision][polytope]")
{
    REQUIRE(collides_ball(cube(), 0.F, 0.F, 0.F, 1e-3F));
    REQUIRE(collides_ball(cube(), 0.5F, -0.5F, 0.2F, 1e-3F));

    REQUIRE(collides_ball(cube(), 2.F, 0.F, 0.F, 1.01F));
    REQUIRE_FALSE(collides_ball(cube(), 2.F, 0.F, 0.F, 0.99F));

    const float vertex_dist = std::sqrt(3.F);
    REQUIRE(collides_ball(cube(), 2.F, 2.F, 2.F, vertex_dist + 0.01F));
    REQUIRE_FALSE(collides_ball(cube(), 2.F, 2.F, 2.F, vertex_dist - 0.01F));

    const float face_dist = std::sqrt(841.F / 3.F);
    REQUIRE(collides_ball(tetrahedron(), 10.F, 10.F, 10.F, face_dist + 0.01F));
    REQUIRE_FALSE(collides_ball(tetrahedron(), 10.F, 10.F, 10.F, face_dist - 0.01F));
}

TEST_CASE(
    "Sphere-polytope catches spheres near an edge but outside the halfspace projection",
    "[collision][polytope]")
{
    // A thin slab rotated 45 degrees about z; a sphere near its long edge is within the OBB of the slab
    // but only touches it through the edge
    const Vertices diamond = {
        {1.F, 0.F, -1.F},
        {0.F, 1.F, -1.F},
        {-1.F, 0.F, -1.F},
        {0.F, -1.F, -1.F},
        {1.F, 0.F, 1.F},
        {0.F, 1.F, 1.F},
        {-1.F, 0.F, 1.F},
        {0.F, -1.F, 1.F},
    };

    // Distance from (0.6, 0.6, 0) to the face x + y = 1 is 0.2 / sqrt(2)
    const float d = 0.2F / std::sqrt(2.F);
    REQUIRE(collides_ball(diamond, 0.6F, 0.6F, 0.F, d + 0.01F));
    REQUIRE_FALSE(collides_ball(diamond, 0.6F, 0.6F, 0.F, d - 0.01F));
}

TEST_CASE("Sphere-polytope with random boxes matches closed-form AABB distance", "[collision][polytope]")
{
    std::mt19937 rng(2024);
    std::uniform_real_distribution<float> center(-5.F, 5.F);
    std::uniform_real_distribution<float> half_width(0.05F, 3.F);
    std::uniform_real_distribution<float> query(-10.F, 10.F);
    std::uniform_real_distribution<float> radius(0.F, 8.F);

    for (auto i = 0U; i < 1000; ++i)
    {
        const Eigen::Vector3f c(center(rng), center(rng), center(rng));
        const Eigen::Vector3f h(half_width(rng), half_width(rng), half_width(rng));
        const Eigen::AlignedBox3f aabb(c - h, c + h);

        const auto corners = box(aabb);
        const float x = query(rng);
        const float y = query(rng);
        const float z = query(rng);
        const float r = radius(rng);

        const float expected = aabb.squaredExteriorDistance(Eigen::Vector3f(x, y, z));
        const float actual = sql2_to(corners, x, y, z);
        REQUIRE(actual == Approx(expected).epsilon(1e-4).margin(1e-4));

        // Skip radii too close to the boundary to be decided robustly in single precision
        if (std::abs(expected - r * r) > 1e-3F * (1.F + expected))
        {
            REQUIRE(collides_ball(corners, x, y, z, r) == (expected < r * r));
        }
    }
}

TEST_CASE("Sphere-polytope SIMD lanes match per-sphere exact distance", "[collision][polytope]")
{
    std::mt19937 rng(99);
    std::uniform_int_distribution<std::size_t> count(1, 20);
    std::uniform_real_distribution<float> coord(-3.F, 3.F);
    std::uniform_real_distribution<float> query(-6.F, 6.F);
    std::uniform_real_distribution<float> radius(0.F, 4.F);

    std::size_t num_colliding = 0;
    std::size_t num_free = 0;
    for (auto i = 0U; i < 1000; ++i)
    {
        Vertices points(count(rng));
        for (auto &p : points)
        {
            p = {coord(rng), coord(rng), coord(rng)};
        }

        const auto polytope = make_polytope(points);
        const vc::ConvexPolytope<V> simd_polytope(polytope);

        alignas(V::S::Alignment) std::array<float, V::num_scalars_rounded> xs, ys, zs, rs;
        bool expected = false;
        bool ambiguous = false;
        for (auto lane = 0U; lane < V::num_scalars_rounded; ++lane)
        {
            xs[lane] = query(rng);
            ys[lane] = query(rng);
            zs[lane] = query(rng);
            rs[lane] = radius(rng);

            const float dist_sq = sql2_to(points, xs[lane], ys[lane], zs[lane]);
            const float r_sq = rs[lane] * rs[lane];
            expected = expected or dist_sq < r_sq;
            ambiguous = ambiguous or std::abs(dist_sq - r_sq) < 1e-3F * (1.F + dist_sq);

            // Every lane should agree with its own single-sphere query
            if (std::abs(dist_sq - r_sq) > 1e-3F * (1.F + dist_sq))
            {
                REQUIRE(
                    collides_ball(simd_polytope, xs[lane], ys[lane], zs[lane], rs[lane]) == (dist_sq < r_sq));
            }
        }

        if (ambiguous)
        {
            continue;
        }

        const bool actual =
            not vc::sphere_polytope(simd_polytope, V(xs.data()), V(ys.data()), V(zs.data()), V(rs.data()))
                    .test_zero();
        REQUIRE(actual == expected);
        (actual ? num_colliding : num_free) += 1;
    }

    // Make sure the fuzzing covers both outcomes
    REQUIRE(num_colliding > 100);
    REQUIRE(num_free > 100);
}

TEST_CASE("GJK converges on coplanar box faces", "[collision][polytope]")
{
    // Regression case from rumple: the query point projects onto the interior of a box face, whose four
    // corners are exactly coplanar
    const Eigen::AlignedBox3f aabb(
        Eigen::Vector3f(-5.368625309902576F, 1.5278480492984203F, -2.156301447748132F),
        Eigen::Vector3f(-1.66844997461571F, 5.422646906343829F, 3.0607879685535417F));
    const float x = -4.743856992006581F;
    const float y = 2.1847375590127527F;
    const float z = -4.426160100672689F;

    REQUIRE(
        sql2_to(box(aabb), x, y, z) ==
        Approx(aabb.squaredExteriorDistance(Eigen::Vector3f(x, y, z))).epsilon(1e-5));
}

TEST_CASE("Polytope min_distance is a lower bound on distance from the origin", "[collision][polytope]")
{
    // A large wall whose nearest vertex is far from the origin, but whose nearest face is close
    const auto wall =
        make_polytope(box({Eigen::Vector3f(0.5F, -10.F, -10.F), Eigen::Vector3f(1.F, 10.F, 10.F)}));
    REQUIRE(wall.min_distance == Approx(0.5F));
}

TEST_CASE("Environment detects polytope collisions past the nearest-vertex cutoff", "[collision][polytope]")
{
    vc::Environment<float> env;
    env.polytopes.emplace_back(
        make_polytope(box({Eigen::Vector3f(0.5F, -10.F, -10.F), Eigen::Vector3f(1.F, 10.F, 10.F)})));
    env.sort();

    const vc::Environment<V> simd_env(env);
    REQUIRE(vamp::sphere_environment_in_collision(simd_env, V::fill(0.4F), V::fill(0.F), V::fill(0.F), 0.2F));
    REQUIRE_FALSE(
        vamp::sphere_environment_in_collision(simd_env, V::fill(0.2F), V::fill(0.F), V::fill(0.F), 0.2F));
}

TEST_CASE("Polytopes built from halfspaces collide correctly", "[collision][polytope]")
{
    // Unit cube [-1, 1]^3 given by its six faces
    const std::vector<std::array<float, 4>> planes = {
        {1.F, 0.F, 0.F, 1.F},
        {-1.F, 0.F, 0.F, 1.F},
        {0.F, 1.F, 0.F, 1.F},
        {0.F, -1.F, 0.F, 1.F},
        {0.F, 0.F, 1.F, 1.F},
        {0.F, 0.F, -1.F, 1.F},
    };

    const vc::ConvexPolytope<V> p(vamp::collision::factory::polytope::from_planes(planes));
    REQUIRE(p.num_vertices == 8);
    REQUIRE(collides_ball(p, 2.F, 0.F, 0.F, 1.01F));
    REQUIRE_FALSE(collides_ball(p, 2.F, 0.F, 0.F, 0.99F));
    REQUIRE(collides_ball(p, 2.F, 2.F, 2.F, std::sqrt(3.F) + 0.01F));
    REQUIRE_FALSE(collides_ball(p, 2.F, 2.F, 2.F, std::sqrt(3.F) - 0.01F));
}

TEST_CASE("Polytope OBB tightly bounds its vertices", "[collision][polytope]")
{
    std::mt19937 rng(7);
    std::uniform_int_distribution<std::size_t> count(1, 30);
    std::uniform_real_distribution<float> coord(-3.F, 3.F);

    for (auto i = 0U; i < 200; ++i)
    {
        Vertices points(count(rng));
        for (auto &p : points)
        {
            p = {coord(rng), coord(rng), coord(rng)};
        }

        const auto obb = make_polytope(points).obb;
        const Eigen::Vector3f center(obb.x, obb.y, obb.z);
        Eigen::Matrix3f axes;
        axes << obb.axis_1_x, obb.axis_2_x, obb.axis_3_x,  //
            obb.axis_1_y, obb.axis_2_y, obb.axis_3_y,      //
            obb.axis_1_z, obb.axis_2_z, obb.axis_3_z;
        const Eigen::Vector3f half(obb.axis_1_r, obb.axis_2_r, obb.axis_3_r);

        REQUIRE(axes.isUnitary(1e-4F));
        REQUIRE(axes.determinant() == Approx(1.F).margin(1e-4));

        // Every vertex lies inside the box, and every face of the box touches some vertex
        Eigen::Vector3f max_extent = Eigen::Vector3f::Constant(-1.F);
        for (const auto &[x, y, z] : points)
        {
            const Eigen::Vector3f local = (axes.transpose() * (Eigen::Vector3f(x, y, z) - center)).cwiseAbs();
            REQUIRE(((local - half).array() <= 1e-4F).all());
            max_extent = max_extent.cwiseMax(local);
        }

        REQUIRE(max_extent.isApprox(half, 1e-4F));
    }
}
