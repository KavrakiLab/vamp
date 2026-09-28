#pragma once

#include <string>
#include <vector>
#include <limits>
#include <cmath>

#include <vamp/vector.hh>
#include <vamp/collision/math.hh>
#include <vamp/collision/gjk.hh>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace vamp::collision
{
    template <typename DataT>
    struct Shape
    {
        std::string name;
        DataT min_distance;

        Shape() = default;

        template <typename OtherDataT>
        explicit Shape(const Shape<OtherDataT> &other) : name(other.name), min_distance(other.min_distance)
        {
        }
    };

    // A cuboid, or rectangular prism, shape oriented in an arbitrary frame. The
    // cube is defined by its center (x, y, z), the three axes defining its frame
    // (axis_1_x, axis_1_y, axis_1_z, etc.), and the half-lengths along each axis
    // (axis_1_r, etc.)
    template <typename DataT>
    struct Cuboid : public Shape<DataT>
    {
        DataT x;
        DataT y;
        DataT z;

        DataT axis_1_x;
        DataT axis_1_y;
        DataT axis_1_z;
        DataT axis_2_x;
        DataT axis_2_y;
        DataT axis_2_z;
        DataT axis_3_x;
        DataT axis_3_y;
        DataT axis_3_z;

        DataT axis_1_r;
        DataT axis_2_r;
        DataT axis_3_r;

        inline constexpr auto compute_min_distance() -> DataT
        {
            auto d_1 = dot_3(-x, -y, -z, axis_1_x, axis_1_y, axis_1_z);
            auto d_2 = dot_3(-x, -y, -z, axis_2_x, axis_2_y, axis_2_z);
            auto d_3 = dot_3(-x, -y, -z, axis_3_x, axis_3_y, axis_3_z);

            auto v_1 = clamp(d_1, -axis_1_r, axis_1_r);
            auto v_2 = clamp(d_2, -axis_2_r, axis_2_r);
            auto v_3 = clamp(d_3, -axis_3_r, axis_3_r);

            auto x_n = x + axis_1_x * v_1 + axis_2_x * v_2 + axis_3_x * v_3;
            auto y_n = y + axis_1_y * v_1 + axis_2_y * v_2 + axis_3_y * v_3;
            auto z_n = z + axis_1_z * v_1 + axis_2_z * v_2 + axis_3_z * v_3;

            return sqrt(x_n * x_n + y_n * y_n + z_n * z_n);
        }

        Cuboid() = default;

        explicit Cuboid(
            DataT x,
            DataT y,
            DataT z,  //
            DataT axis_1_x,
            DataT axis_1_y,
            DataT axis_1_z,  //
            DataT axis_2_x,
            DataT axis_2_y,
            DataT axis_2_z,  //
            DataT axis_3_x,
            DataT axis_3_y,
            DataT axis_3_z,  //
            DataT axis_1_r,
            DataT axis_2_r,
            DataT axis_3_r)
          : Shape<DataT>()
          , x(x)
          , y(y)
          , z(z)
          , axis_1_x(axis_1_x)
          , axis_1_y(axis_1_y)
          , axis_1_z(axis_1_z)
          , axis_2_x(axis_2_x)
          , axis_2_y(axis_2_y)
          , axis_2_z(axis_2_z)
          , axis_3_x(axis_3_x)
          , axis_3_y(axis_3_y)
          , axis_3_z(axis_3_z)
          , axis_1_r(axis_1_r)
          , axis_2_r(axis_2_r)
          , axis_3_r(axis_3_r)
        {
            Shape<DataT>::min_distance = compute_min_distance();
        }

        template <typename OtherDataT>
        explicit Cuboid(const Cuboid<OtherDataT> &other)
          : Shape<DataT>(other)
          , x(other.x)
          , y(other.y)
          , z(other.z)
          , axis_1_x(other.axis_1_x)
          , axis_1_y(other.axis_1_y)
          , axis_1_z(other.axis_1_z)
          , axis_2_x(other.axis_2_x)
          , axis_2_y(other.axis_2_y)
          , axis_2_z(other.axis_2_z)
          , axis_3_x(other.axis_3_x)
          , axis_3_y(other.axis_3_y)
          , axis_3_z(other.axis_3_z)
          , axis_1_r(other.axis_1_r)
          , axis_2_r(other.axis_2_r)
          , axis_3_r(other.axis_3_r)
        {
        }
    };

    // A cylinder shape oriented in an arbitrary frame. The cylinder is defined by
    // two endpoints, which we store as the first point (x1, y1, z1) and the vector
    // from the first point to the second point (xv, yv, zv), and its radius (r)
    template <typename DataT>
    struct Cylinder : public Shape<DataT>
    {
        DataT x1;
        DataT y1;
        DataT z1;

        DataT xv;
        DataT yv;
        DataT zv;

        DataT r;

        // This is the reciprocal of the dot product of the vector between the
        // cylinder's endpoints, i.e. rdv = 1 / ([xv yv zv] . [xv yv zv]) We store it
        // for convenience to make some computations faster
        DataT rdv;

        inline constexpr auto x2() -> DataT
        {
            return x1 + xv;
        }

        inline constexpr auto y2() -> DataT
        {
            return y1 + yv;
        }

        inline constexpr auto z2() -> DataT
        {
            return z1 + zv;
        }

        inline constexpr auto compute_min_distance() -> DataT
        {
            auto dot = clamp(dot_3(-x1, -y1, -z1, xv, yv, zv) * rdv, 0.F, 1.F);

            auto xp = x1 + xv * dot;
            auto yp = y1 + yv * dot;
            auto zp = z1 + zv * dot;

            auto xo = -xp;
            auto yo = -yp;
            auto zo = -zp;

            auto ol = sqrt(dot_3(xo, yo, zo, xo, yo, zo));
            xo = xo / ol;
            yo = yo / ol;
            zo = zo / ol;

            auto ro = clamp(ol, 0.F, r);

            auto xn = xp + ro * xo;
            auto yn = yp + ro * yo;
            auto zn = zp + ro * zo;

            return sqrt(xn * xn + yn * yn + zn * zn);
        }

        Cylinder() = default;

        explicit Cylinder(
            DataT x1,
            DataT y1,
            DataT z1,  //
            DataT xv,
            DataT yv,
            DataT zv,  //
            DataT r,
            DataT rdv)
          : Shape<DataT>(), x1(x1), y1(y1), z1(z1), xv(xv), yv(yv), zv(zv), r(r), rdv(rdv)
        {
            Shape<DataT>::min_distance = compute_min_distance();
        }

        template <typename OtherDataT>
        explicit Cylinder(const Cylinder<OtherDataT> &other)
          : Shape<DataT>(other)
          , x1(other.x1)
          , y1(other.y1)
          , z1(other.z1)
          , xv(other.xv)
          , yv(other.yv)
          , zv(other.zv)
          , r(other.r)
          , rdv(other.rdv)
        {
        }
    };

    template <typename DataT>
    using Capsule = Cylinder<DataT>;

    // A sphere shape, represented as its center (x, y, z) and radius (r)
    template <typename DataT>
    struct Sphere : public Shape<DataT>
    {
        DataT x;
        DataT y;
        DataT z;
        DataT r;

        Sphere() = default;

        explicit Sphere(DataT x, DataT y, DataT z, DataT r) : Shape<DataT>(), x(x), y(y), z(z), r(r)
        {
            Shape<DataT>::min_distance = sqrt(x * x + y * y + z * z) - r;
        }

        template <typename OtherDataT>
        explicit Sphere(const Sphere<OtherDataT> &other)
          : Shape<DataT>(other), x(other.x), y(other.y), z(other.z), r(other.r)
        {
        }
    };

    // A heighfield. Defined by height pixel buffer.
    template <typename DataT>
    struct HeightField : public Shape<DataT>
    {
        DataT x;  // offset
        DataT y;
        DataT z;

        DataT xs;  // scaling factor
        DataT ys;
        DataT zs;

        std::size_t xd;  // image size
        std::size_t yd;

        std::size_t xd2;  // image size half
        std::size_t yd2;

        std::vector<float> data;  // flattened row-major data

        HeightField() = default;

        explicit HeightField(
            DataT x,
            DataT y,
            DataT z,
            DataT xs,
            DataT ys,
            DataT zs,
            std::size_t xd,
            std::size_t yd,
            const std::vector<float> &data)
          : Shape<DataT>()
          , x(x)
          , y(y)
          , z(z)
          , xs(xs)
          , ys(ys)
          , zs(zs)
          , xd(xd)
          , yd(yd)
          , xd2(xd / 2)
          , yd2(yd / 2)
          , data(data)
        {
            Shape<DataT>::min_distance = 0;
        }

        template <typename OtherDataT>
        explicit HeightField(const HeightField<OtherDataT> &other)
          : Shape<DataT>(other)
          , x(other.x)
          , y(other.y)
          , z(other.z)
          , xs(other.xs)
          , ys(other.ys)
          , zs(other.zs)
          , xd(other.xd)
          , yd(other.yd)
          , xd2(other.xd2)
          , yd2(other.yd2)
          , data(other.data)
        {
        }
    };

    // A convex polytope with dual representation:
    // - H-representation: half-planes
    // - V-representation: vertices
    template <typename DataT>
    struct ConvexPolytope : public Shape<DataT>
    {
        // Halfspaces
        std::size_t num_planes;
        std::vector<DataT> nx;  // plane normals
        std::vector<DataT> ny;
        std::vector<DataT> nz;
        std::vector<DataT> d;  // plane offsets (n.x <= d defines interior)

        // Vertices
        std::size_t num_vertices;
        std::vector<float> vx;
        std::vector<float> vy;
        std::vector<float> vz;

        // Oriented bounding box for broadphase
        Cuboid<DataT> obb;

        ConvexPolytope() = default;

        explicit ConvexPolytope(
            std::size_t num_planes,
            const std::vector<DataT> &nx,
            const std::vector<DataT> &ny,
            const std::vector<DataT> &nz,
            const std::vector<DataT> &d,
            std::size_t num_vertices,
            const std::vector<float> &vx,
            const std::vector<float> &vy,
            const std::vector<float> &vz)
          : Shape<DataT>()
          , num_planes(num_planes)
          , nx(nx)
          , ny(ny)
          , nz(nz)
          , d(d)
          , num_vertices(num_vertices)
          , vx(vx)
          , vy(vy)
          , vz(vz)
        {
            obb = compute_obb();
            Shape<DataT>::min_distance = compute_min_distance();
        }

        inline auto compute_min_distance() -> DataT
        {
            // Exact distance from the origin to the polytope.

            if (num_vertices == 0)
            {
                return DataT(std::numeric_limits<float>::max());
            }

            const float dist_sq = gjk::sql2(
                vx.data(),
                vy.data(),
                vz.data(),
                num_vertices,
                Eigen::Vector3f::Zero(),
                Eigen::Vector3f(vx[0], vy[0], vz[0]));
            return DataT(std::sqrt(dist_sq));
        }

        inline auto compute_obb() -> Cuboid<DataT>
        {
            const auto n = static_cast<Eigen::Index>(num_vertices);
            Eigen::Matrix3Xf vertices(3, n);
            vertices.row(0) = Eigen::Map<const Eigen::RowVectorXf>(vx.data(), n);
            vertices.row(1) = Eigen::Map<const Eigen::RowVectorXf>(vy.data(), n);
            vertices.row(2) = Eigen::Map<const Eigen::RowVectorXf>(vz.data(), n);

            // Principal axes of the vertex distribution
            const Eigen::Matrix3Xf centered = vertices.colwise() - vertices.rowwise().mean();
            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(centered * centered.transpose());
            Eigen::Matrix3f axes = solver.eigenvectors();

            if (axes.determinant() < 0)
            {
                axes.col(0) *= -1;
            }

            // Extents of the vertices along each axis
            const Eigen::Matrix3Xf projected = axes.transpose() * vertices;
            const Eigen::Vector3f lo = projected.rowwise().minCoeff();
            const Eigen::Vector3f hi = projected.rowwise().maxCoeff();

            const Eigen::Vector3f center = axes * (lo + hi) * 0.5F;
            const Eigen::Vector3f half = (hi - lo) * 0.5F;
            const Eigen::Vector3f axis_0 = axes.col(0);
            const Eigen::Vector3f axis_1 = axes.col(1);
            const Eigen::Vector3f axis_2 = axes.col(2);

            return Cuboid<DataT>(
                DataT(center.x()),
                DataT(center.y()),
                DataT(center.z()),
                DataT(axis_0.x()),
                DataT(axis_0.y()),
                DataT(axis_0.z()),
                DataT(axis_1.x()),
                DataT(axis_1.y()),
                DataT(axis_1.z()),
                DataT(axis_2.x()),
                DataT(axis_2.y()),
                DataT(axis_2.z()),
                DataT(half.x()),
                DataT(half.y()),
                DataT(half.z()));
        }

        template <typename OtherDataT>
        explicit ConvexPolytope(const ConvexPolytope<OtherDataT> &other)
          : Shape<DataT>(other)
          , num_planes(other.num_planes)
          , nx(other.nx.size())
          , ny(other.ny.size())
          , nz(other.nz.size())
          , d(other.d.size())
          , num_vertices(other.num_vertices)
          , vx(other.vx)
          , vy(other.vy)
          , vz(other.vz)
          , obb(other.obb)
        {
            for (auto i = 0U; i < other.nx.size(); ++i)
            {
                nx[i] = DataT(other.nx[i]);
                ny[i] = DataT(other.ny[i]);
                nz[i] = DataT(other.nz[i]);
                d[i] = DataT(other.d[i]);
            }
        }
    };
}  // namespace vamp::collision
