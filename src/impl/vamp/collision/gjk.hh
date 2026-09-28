#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

#include <Eigen/Dense>

// Scalar GJK distance queries between a point and the convex hull of a set of vertices. Ported from
// the `carom-env` polytope implementation in rumple.

namespace vamp::collision::gjk
{
    using Point = Eigen::Vector3f;

    inline constexpr float epsilon = std::numeric_limits<float>::epsilon();

    // The vertex, among those given by parallel coordinate arrays `xs`/`ys`/`zs` of length `n`,
    // furthest in direction `d` (the GJK support function).
    inline auto
    support(const float *xs, const float *ys, const float *zs, std::size_t n, const Point &d) noexcept
        -> Point
    {
        using Coords = Eigen::Map<const Eigen::VectorXf>;
        const auto size = static_cast<Eigen::Index>(n);

        Eigen::Index best;
        (Coords(xs, size) * d.x() + Coords(ys, size) * d.y() + Coords(zs, size) * d.z()).maxCoeff(&best);
        return {xs[best], ys[best], zs[best]};
    }

    // A fixed-capacity stack of up to 4 points (the largest possible simplex in 3D GJK), so that the
    // collision-check hot path never allocates.
    struct Simplex
    {
        std::array<Point, 4> points;
        std::uint8_t len;

        static auto single(const Point &a) noexcept -> Simplex
        {
            return Simplex{{a, a, a, a}, 1};
        }

        static auto triangle(const Point &a, const Point &b, const Point &c) noexcept -> Simplex
        {
            return Simplex{{a, b, c, c}, 3};
        }

        void push(const Point &p) noexcept
        {
            points[len] = p;
            ++len;
        }

        // Whether `p` (numerically) already appears in this simplex.
        [[nodiscard]] auto contains(const Point &p) const noexcept -> bool
        {
            for (auto i = 0U; i < len; ++i)
            {
                if ((p - points[i]).squaredNorm() <= epsilon)
                {
                    return true;
                }
            }

            return false;
        }

        void set1(const Point &a) noexcept
        {
            points[0] = a;
            len = 1;
        }

        void set2(const Point &a, const Point &b) noexcept
        {
            points[0] = a;
            points[1] = b;
            len = 2;
        }
    };

    inline auto closest_on_segment(Simplex &simplex) noexcept -> Point
    {
        const Point a = simplex.points[0];
        const Point b = simplex.points[1];
        const Point ab = b - a;
        const float denom = ab.squaredNorm();
        if (denom <= epsilon)
        {
            simplex.set1(a);
            return a;
        }

        const float t = -a.dot(ab) / denom;
        if (t <= 0.F)
        {
            simplex.set1(a);
            return a;
        }

        if (t >= 1.F)
        {
            simplex.set1(b);
            return b;
        }

        return a + t * ab;
    }

    // Closest point to the origin on triangle `simplex[0..3]`, using the standard Voronoi-region test
    // (Ericson, "Real-Time Collision Detection", `ClosestPtPointTriangle`), specialized to a query
    // point of the origin. Reduces `simplex` to whichever vertex/edge/face is closest.
    inline auto closest_on_triangle(Simplex &simplex) noexcept -> Point
    {
        const Point a = simplex.points[0];
        const Point b = simplex.points[1];
        const Point c = simplex.points[2];

        const Point ab = b - a;
        const Point ac = c - a;

        const float d1 = -ab.dot(a);
        const float d2 = -ac.dot(a);
        if (d1 <= 0.F and d2 <= 0.F)
        {
            simplex.set1(a);
            return a;
        }

        const float d3 = -ab.dot(b);
        const float d4 = -ac.dot(b);
        if (d3 >= 0.F and d4 <= d3)
        {
            simplex.set1(b);
            return b;
        }

        const float vc = d1 * d4 - d3 * d2;
        if (vc <= 0.F and d1 >= 0.F and d3 <= 0.F)
        {
            const float v = d1 / (d1 - d3);
            simplex.set2(a, b);
            return a + v * ab;
        }

        const float d5 = -ab.dot(c);
        const float d6 = -ac.dot(c);
        if (d6 >= 0.F and d5 <= d6)
        {
            simplex.set1(c);
            return c;
        }

        const float vb = d5 * d2 - d1 * d6;
        if (vb <= 0.F and d2 >= 0.F and d6 <= 0.F)
        {
            const float w = d2 / (d2 - d6);
            simplex.set2(a, c);
            return a + w * ac;
        }

        const float va = d3 * d6 - d5 * d4;
        if (va <= 0.F and (d4 - d3) >= 0.F and (d5 - d6) >= 0.F)
        {
            const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            simplex.set2(b, c);
            return b + w * (c - b);
        }

        const float denom = 1.F / (va + vb + vc);
        const float v = vb * denom;
        const float w = vc * denom;
        return a + v * ab + w * ac;
    }

    // Whether the origin and vertex `d` lie on opposite sides of the plane through `a`, `b`, `c` or
    // `d` is coplanar with `a`, `b`, `c`.
    inline auto point_outside_face(const Point &a, const Point &b, const Point &c, const Point &d) noexcept
        -> bool
    {
        const Point normal = (b - a).cross(c - a);
        const float sign_origin = -normal.dot(a);
        const float sign_d = normal.dot(d - a);
        return sign_origin * sign_d <= 0.F;
    }

    // Closest point to the origin on tetrahedron `simplex[0..4]`, following Ericson's
    // `ClosestPtPointTetrahedron`: test each face plane to see whether the origin is outside of it, and
    // if so consider the closest point on that face. Returns `std::nullopt` if the origin is inside all
    // four face planes, i.e. inside the tetrahedron.
    inline auto closest_on_tetrahedron(Simplex &simplex) noexcept -> std::optional<Point>
    {
        const Point a = simplex.points[0];
        const Point b = simplex.points[1];
        const Point c = simplex.points[2];
        const Point d = simplex.points[3];

        float best_sq = std::numeric_limits<float>::max();
        Point best_point = a;
        Simplex best_simplex = Simplex::triangle(a, b, c);
        bool outside_any = false;

        const std::array<std::array<Point, 4>, 4> faces = {{
            {a, b, c, d},
            {a, c, d, b},
            {a, d, b, c},
            {b, d, c, a},
        }};

        for (const auto &[x, y, z, w] : faces)
        {
            if (point_outside_face(x, y, z, w))
            {
                outside_any = true;
                auto face = Simplex::triangle(x, y, z);
                const Point p = closest_on_triangle(face);
                const float sq = p.squaredNorm();
                if (sq < best_sq)
                {
                    best_sq = sq;
                    best_point = p;
                    best_simplex = face;
                }
            }
        }

        if (outside_any)
        {
            simplex = best_simplex;
            return best_point;
        }

        return std::nullopt;
    }

    // Reduce `simplex` to the smallest sub-simplex containing the point on it closest to the origin,
    // and return that closest point. Returns `std::nullopt` if the origin lies inside the simplex (only
    // possible when `simplex` is a tetrahedron), which means the query point is inside the polytope.
    inline auto closest_on_simplex(Simplex &simplex) noexcept -> std::optional<Point>
    {
        switch (simplex.len)
        {
            case 1:
                return simplex.points[0];
            case 2:
                return closest_on_segment(simplex);
            case 3:
                return closest_on_triangle(simplex);
            default:
                return closest_on_tetrahedron(simplex);
        }
    }

    // The core GJK loop, computing the squared distance from the convex hull of the `n` vertices given
    // by parallel coordinate arrays `xs`/`ys`/`zs` to `point`, using `seed` to seed the initial search
    // direction.
    //
    // If `radius_sq` is set, the loop is allowed to return as soon as it can prove that the true squared
    // distance is on one side of `radius_sq` or the other, rather than running to full convergence. The
    // returned value is only guaranteed to be on the correct side of `radius_sq`, not necessarily the
    // exact distance. If `radius_sq` is unset, the loop always converges and returns the exact squared
    // distance.
    //
    // Returns the largest finite float if there are no vertices, since the distance to an empty set is
    // unbounded. This is not infinity because VAMP's Release builds under clang on x86 assume no value is
    // ever infinite (`-fno-honor-infinities`).
    inline auto sql2(
        const float *xs,
        const float *ys,
        const float *zs,
        std::size_t n,
        const Point &point,
        const Point &seed,
        std::optional<float> radius_sq = std::nullopt) noexcept -> float
    {
        if (n == 0)
        {
            return std::numeric_limits<float>::max();
        }

        Point direction = seed - point;
        if (direction.squaredNorm() <= epsilon)
        {
            direction = Point::UnitX();
        }

        // simplex always stores points with `point` subtracted
        auto simplex = Simplex::single(support(xs, ys, zs, n, direction) - point);
        Point closest = simplex.points[0];

        const std::size_t max_iterations = n + 8;
        for (auto iter = 0U; iter < max_iterations; ++iter)
        {
            const float dist_sq = closest.squaredNorm();
            if (dist_sq <= epsilon)
            {
                return 0.F;
            }

            // `closest` is always a point of the polytope, so `dist_sq` is a valid upper bound on the
            // true distance: if it is already within the ball, we are done.
            if (radius_sq and dist_sq <= *radius_sq)
            {
                return dist_sq;
            }

            direction = -closest;
            const Point new_point = support(xs, ys, zs, n, direction) - point;

            const float d_new = new_point.dot(direction);
            const float d_closest = closest.dot(direction);

            // `new_point` is a supporting point of the polytope in `direction`, so the whole polytope
            // lies on its near side of the hyperplane through it: if even that hyperplane is farther
            // from the origin than `r`, the polytope can't collide.
            if (radius_sq and d_new < 0.F and d_new * d_new > *radius_sq * dist_sq)
            {
                return dist_sq;
            }

            // If no candidate point makes enough progress, or the support function just re-found a
            // point already in the simplex (which would make it degenerate), we've converged.
            const float tolerance = epsilon * (1.F + std::abs(d_closest));
            if (d_new - d_closest <= tolerance or simplex.contains(new_point))
            {
                return dist_sq;
            }

            simplex.push(new_point);
            const auto reduced = closest_on_simplex(simplex);
            if (not reduced)
            {
                return 0.F;
            }

            // The check above is only a heuristic lower bound on how much `new_point` could improve the
            // distance, computed from the *old* simplex: on an exactly degenerate (e.g. coplanar)
            // sub-simplex, `closest_on_tetrahedron`'s face tests can end up deterministically dropping
            // the same vertex that floating-point noise in `direction` just as deterministically
            // re-selects, cycling forever without either heuristic above ever firing. Comparing the
            // actual squared distance before and after the simplex update catches that case directly,
            // since a true cycle makes no real progress here even when the heuristic above claims
            // otherwise.
            const float new_dist_sq = reduced->squaredNorm();
            if (dist_sq - new_dist_sq <= tolerance)
            {
                return new_dist_sq;
            }

            closest = *reduced;
        }

        return closest.squaredNorm();
    }
}  // namespace vamp::collision::gjk
