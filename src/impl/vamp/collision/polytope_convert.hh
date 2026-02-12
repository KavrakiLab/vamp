#pragma once

#include <vector>
#include <array>
#include <stdexcept>
#include <cmath>

extern "C"
{
#include <setoper.h>
#include <cdd.h>
}

namespace vamp::collision
{
    struct PolytopeData
    {
        std::vector<float> nx, ny, nz, d;
        std::vector<float> vx, vy, vz;
    };

    inline void cdd_init()
    {
        static bool initialized = false;
        if (!initialized)
        {
            dd_set_global_constants();
            initialized = true;
        }
    }

    inline auto vertices_to_halfspaces(
        const std::vector<float> &vx,
        const std::vector<float> &vy,
        const std::vector<float> &vz) -> PolytopeData
    {
        cdd_init();

        const std::size_t num_vertices = vx.size();
        if (num_vertices < 4)
        {
            throw std::invalid_argument("Need at least 4 vertices for a 3D convex polytope");
        }

        PolytopeData result;
        result.vx = vx;
        result.vy = vy;
        result.vz = vz;

        // Each row is [1, x, y, z] for a vertex (1 indicates vertex, not ray)
        dd_MatrixPtr vertices_matrix = dd_CreateMatrix(num_vertices, 4);
        vertices_matrix->representation = dd_Generator;
        vertices_matrix->numbtype = dd_Real;

        for (auto i = 0U; i < num_vertices; ++i)
        {
            dd_set_d(vertices_matrix->matrix[i][0], 1.0);
            dd_set_d(vertices_matrix->matrix[i][1], static_cast<double>(vx[i]));
            dd_set_d(vertices_matrix->matrix[i][2], static_cast<double>(vy[i]));
            dd_set_d(vertices_matrix->matrix[i][3], static_cast<double>(vz[i]));
        }

        dd_ErrorType err = dd_NoError;
        dd_PolyhedraPtr poly = dd_DDMatrix2Poly(vertices_matrix, &err);

        if (err != dd_NoError || poly == nullptr)
        {
            dd_FreeMatrix(vertices_matrix);
            if (poly)
                dd_FreePolyhedra(poly);
            throw std::runtime_error("cddlib: Failed to convert vertices to halfspaces");
        }

        dd_MatrixPtr inequalities = dd_CopyInequalities(poly);

        result.nx.reserve(inequalities->rowsize);
        result.ny.reserve(inequalities->rowsize);
        result.nz.reserve(inequalities->rowsize);
        result.d.reserve(inequalities->rowsize);

        for (dd_rowrange i = 0; i < inequalities->rowsize; ++i)
        {
            // cddlib format: b + a1*x1 + a2*x2 + a3*x3 >= 0
            // We want: nx*x + ny*y + nz*z <= d
            // So: -a1*x - a2*y - a3*z <= b
            double b = dd_get_d(inequalities->matrix[i][0]);
            double a1 = dd_get_d(inequalities->matrix[i][1]);
            double a2 = dd_get_d(inequalities->matrix[i][2]);
            double a3 = dd_get_d(inequalities->matrix[i][3]);

            // Normalize the plane equation
            double norm = std::sqrt(a1 * a1 + a2 * a2 + a3 * a3);
            if (norm > 1e-10)
            {
                result.nx.push_back(static_cast<float>(-a1 / norm));
                result.ny.push_back(static_cast<float>(-a2 / norm));
                result.nz.push_back(static_cast<float>(-a3 / norm));
                result.d.push_back(static_cast<float>(b / norm));
            }
        }

        dd_FreeMatrix(inequalities);
        dd_FreePolyhedra(poly);
        dd_FreeMatrix(vertices_matrix);

        return result;
    }

    inline auto halfspaces_to_vertices(
        const std::vector<float> &nx,
        const std::vector<float> &ny,
        const std::vector<float> &nz,
        const std::vector<float> &d) -> PolytopeData
    {
        cdd_init();

        const std::size_t num_planes = nx.size();
        if (num_planes < 4)
        {
            throw std::invalid_argument("Need at least 4 halfspaces for a bounded 3D polytope");
        }

        PolytopeData result;
        result.nx = nx;
        result.ny = ny;
        result.nz = nz;
        result.d = d;

        dd_MatrixPtr ineq_matrix = dd_CreateMatrix(num_planes, 4);
        ineq_matrix->representation = dd_Inequality;
        ineq_matrix->numbtype = dd_Real;

        for (std::size_t i = 0; i < num_planes; ++i)
        {
            dd_set_d(ineq_matrix->matrix[i][0], static_cast<double>(d[i]));
            dd_set_d(ineq_matrix->matrix[i][1], static_cast<double>(-nx[i]));
            dd_set_d(ineq_matrix->matrix[i][2], static_cast<double>(-ny[i]));
            dd_set_d(ineq_matrix->matrix[i][3], static_cast<double>(-nz[i]));
        }

        dd_ErrorType err = dd_NoError;
        dd_PolyhedraPtr poly = dd_DDMatrix2Poly(ineq_matrix, &err);

        if (err != dd_NoError || poly == nullptr)
        {
            dd_FreeMatrix(ineq_matrix);
            if (poly)
                dd_FreePolyhedra(poly);
            throw std::runtime_error("cddlib: Failed to convert halfspaces to vertices");
        }

        dd_MatrixPtr generators = dd_CopyGenerators(poly);

        result.vx.reserve(generators->rowsize);
        result.vy.reserve(generators->rowsize);
        result.vz.reserve(generators->rowsize);

        for (dd_rowrange i = 0; i < generators->rowsize; ++i)
        {
            double type = dd_get_d(generators->matrix[i][0]);
            if (type > 0.5)
            {
                result.vx.push_back(static_cast<float>(dd_get_d(generators->matrix[i][1])));
                result.vy.push_back(static_cast<float>(dd_get_d(generators->matrix[i][2])));
                result.vz.push_back(static_cast<float>(dd_get_d(generators->matrix[i][3])));
            }
        }

        dd_FreeMatrix(generators);
        dd_FreePolyhedra(poly);
        dd_FreeMatrix(ineq_matrix);

        return result;
    }

    inline auto vertices_to_halfspaces(const std::vector<std::array<float, 3>> &vertices) -> PolytopeData
    {
        std::vector<float> vx, vy, vz;
        vx.reserve(vertices.size());
        vy.reserve(vertices.size());
        vz.reserve(vertices.size());

        for (const auto &v : vertices)
        {
            vx.push_back(v[0]);
            vy.push_back(v[1]);
            vz.push_back(v[2]);
        }

        return vertices_to_halfspaces(vx, vy, vz);
    }

    inline auto halfspaces_to_vertices(const std::vector<std::array<float, 4>> &planes) -> PolytopeData
    {
        std::vector<float> nx, ny, nz, d;
        nx.reserve(planes.size());
        ny.reserve(planes.size());
        nz.reserve(planes.size());
        d.reserve(planes.size());

        for (const auto &p : planes)
        {
            nx.push_back(p[0]);
            ny.push_back(p[1]);
            nz.push_back(p[2]);
            d.push_back(p[3]);
        }

        return halfspaces_to_vertices(nx, ny, nz, d);
    }

}  // namespace vamp::collision
