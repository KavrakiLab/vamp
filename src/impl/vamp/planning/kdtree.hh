#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numeric>
#include <optional>
#include <utility>
#include <vector>

#include <vamp/vector.hh>

#ifdef __linux__
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace vamp::planning
{
    // Incremental batched kd-tree over robot configurations.
    //
    // Leaves hold up to leaf_capacity configurations in SoA blocks (dimension-major,
    // FloatVectorWidth lanes) scanned with SIMD row operations. Every node maintains an
    // axis-aligned bounding box, grown along the insertion path and used for traversal
    // pruning, so correctness never depends on the split heuristic. Splits happen when a
    // leaf overflows: median of the widest box axis, ties broken by insertion index.
    //
    // Metric: L2 over the configuration, except quaternion blocks (Robot::so3_offsets)
    // which use the sign-minimized chordal distance min(|a - b|, |a + b|). Because each
    // block contributes an independent term to the squared sum, the minimum over sign
    // choices factorizes per block: box lower bounds and leaf distances take the min of
    // the +/- contributions per block, so one traversal computes the exact metric.
    // Stored quaternions are canonicalized to a fixed hemisphere to keep boxes tight;
    // queries are used as-is since both signs are always evaluated.
    //
    // Deterministic and single-threaded: no RNG, candidate ties broken by lowest index,
    // and pruning is strict (bound > best) so the lowest-index minimum is always found.
    template <typename Robot, std::size_t leaf_capacity = 128>
    struct KDTree
    {
        static constexpr std::size_t dim = Robot::dimension;
        static constexpr std::size_t width = FloatVectorWidth;
        static constexpr std::size_t n_blocks = (leaf_capacity + width - 1) / width;
        static_assert(leaf_capacity % width == 0, "leaf_capacity must be a multiple of the SIMD width");

        using Row = FloatVector<width>;

        static constexpr std::uint32_t nil = std::numeric_limits<std::uint32_t>::max();
        static constexpr float inf = std::numeric_limits<float>::infinity();

        static constexpr std::array<bool, dim> is_quat = []()
        {
            std::array<bool, dim> mask{};
            for (const auto offset : Robot::so3_offsets)
            {
                for (auto j = 0U; j < 4U; ++j)
                {
                    mask[offset + j] = true;
                }
            }

            return mask;
        }();

        static constexpr std::size_t n_base = dim - 4 * Robot::so3_offsets.size();

        static constexpr std::array<std::size_t, n_base == 0 ? 1 : n_base> base_dims = []()
        {
            std::array<std::size_t, n_base == 0 ? 1 : n_base> idx{};
            std::size_t n = 0;
            for (std::size_t j = 0; j < dim; ++j)
            {
                if (not is_quat[j])
                {
                    idx[n++] = j;
                }
            }

            return idx;
        }();

        // Leaf-scan early exit: re-test the partial sums against the live bound every
        // checkpoint base dimensions. No checkpoint fires unless n_base > checkpoint,
        // so low-dim robots pay nothing; high-dim scans skip the bulk of far blocks.
        static constexpr std::size_t checkpoint = 8;

        struct Leaf
        {
            // Zero-initialized so partially filled blocks never read uninitialized lanes.
            alignas(FloatVectorAlignment) std::array<float, n_blocks * dim * width> data = {};
            std::array<std::uint32_t, leaf_capacity> indices = {};
            std::uint32_t count = 0;

            [[nodiscard]] auto coord(std::size_t i, std::size_t j) const noexcept -> float
            {
                return data[((i / width) * dim + j) * width + (i % width)];
            }

            [[nodiscard]] auto row(std::size_t block, std::size_t j) const noexcept -> Row
            {
                return Row(data.data() + (block * dim + j) * width);
            }

            void append(const std::array<float, dim> &q, std::uint32_t index) noexcept
            {
                for (auto j = 0U; j < dim; ++j)
                {
                    data[((count / width) * dim + j) * width + (count % width)] = q[j];
                }

                indices[count] = index;
                count++;
            }
        };

        struct Node
        {
            std::array<float, dim> lo, hi;
            float split = 0.F;
            std::array<std::uint32_t, 2> children = {nil, nil};
            std::uint32_t leaf = nil;
            std::uint32_t axis = 0;
        };

        KDTree()
        {
            init_root();
        }

        [[nodiscard]] auto size() const noexcept -> std::size_t
        {
            return size_;
        }

        // (1 + epsilon)-approximate queries: prune subtrees and leaf blocks that cannot
        // improve the current best by more than the epsilon factor, so any reported
        // neighbor is within (1 + epsilon) of the true nearest distance. Also shrinks
        // the effective radius of bounded queries by the same factor. epsilon = 0 is
        // exact and bit-identical to a tree without this feature.
        void set_epsilon(float epsilon) noexcept
        {
            const auto s = 1.F + std::max(epsilon, 0.F);
            prune_scale_ = 1.F / (s * s);
        }

        void reserve(std::size_t n)
        {
            leaves_.reserve(2 * n / leaf_capacity + 2);
            nodes_.reserve(4 * n / leaf_capacity + 3);
            advise_huge(leaves_.data(), leaves_.capacity() * sizeof(Leaf));
            advise_huge(nodes_.data(), nodes_.capacity() * sizeof(Node));
        }

        void clear()
        {
            nodes_.clear();
            leaves_.clear();
            size_ = 0;
            init_root();
        }

        void insert(std::size_t index, const float *q)
        {
            std::array<float, dim> qa;
            std::copy_n(q, dim, qa.begin());
            canonicalize(qa);

            auto ni = 0U;
            while (true)
            {
                grow(nodes_[ni], qa);

                if (nodes_[ni].leaf == nil)
                {
                    const auto &n = nodes_[ni];
                    ni = n.children[qa[n.axis] < n.split ? 0 : 1];
                    continue;
                }

                if (leaves_[nodes_[ni].leaf].count == leaf_capacity)
                {
                    split_leaf(ni);
                    const auto &n = nodes_[ni];
                    ni = n.children[qa[n.axis] < n.split ? 0 : 1];
                    continue;
                }

                leaves_[nodes_[ni].leaf].append(qa, static_cast<std::uint32_t>(index));
                size_++;
                return;
            }
        }

        [[nodiscard]] auto nearest(const float *q) const noexcept
            -> std::optional<std::pair<std::size_t, float>>
        {
            if (size_ == 0)
            {
                return std::nullopt;
            }

            const auto qp = prepare(q);
            auto best_d2 = inf;
            auto best_index = std::numeric_limits<std::size_t>::max();
            search_one(0, q, qp, best_d2, best_index);
            return std::make_pair(best_index, std::sqrt(best_d2));
        }

        // K-nearest within an optional radius, sorted ascending by (distance, index).
        void nearest(
            std::vector<std::pair<std::size_t, float>> &out,
            const float *q,
            std::size_t k,
            float radius = inf) const
        {
            out.clear();
            if (size_ == 0 or k == 0)
            {
                return;
            }

            const auto qp = prepare(q);
            const auto r2 = (radius == inf) ? inf : radius * radius;
            scratch_.clear();
            search_k(0, q, qp, k, r2);

            std::sort(scratch_.begin(), scratch_.end());
            out.reserve(scratch_.size());
            for (const auto &[d2, index] : scratch_)
            {
                out.emplace_back(index, std::sqrt(d2));
            }
        }

    private:
        using Rows = std::array<Row, dim>;

        void init_root()
        {
            Node root;
            root.lo.fill(inf);
            root.hi.fill(-inf);
            root.leaf = 0;
            nodes_.push_back(root);
            leaves_.emplace_back();
        }

        static void canonicalize(std::array<float, dim> &q) noexcept
        {
            for (const auto offset : Robot::so3_offsets)
            {
                auto sign = 0.F;
                for (auto j = 0U; j < 4U; ++j)
                {
                    const auto v = q[offset + 3 - j];
                    if (v != 0.F)
                    {
                        sign = v;
                        break;
                    }
                }

                if (sign < 0.F)
                {
                    for (auto j = 0U; j < 4U; ++j)
                    {
                        q[offset + j] = -q[offset + j];
                    }
                }
            }
        }

        static void grow(Node &n, const std::array<float, dim> &q) noexcept
        {
            for (auto j = 0U; j < dim; ++j)
            {
                n.lo[j] = std::min(n.lo[j], q[j]);
                n.hi[j] = std::max(n.hi[j], q[j]);
            }
        }

        static auto prepare(const float *q) noexcept -> Rows
        {
            Rows qp;
            for (auto j = 0U; j < dim; ++j)
            {
                qp[j] = Row::fill(q[j]);
            }

            return qp;
        }

        // Next representable float above x (x is a nonnegative squared distance or +inf).
        // Converts the strict skip test "partial > bound" into "partial >= next_up(bound)",
        // which vectorizes with only min/max/hsum.
        static auto next_up(float x) noexcept -> float
        {
            if (not std::isfinite(x))
            {
                return x;
            }

            std::uint32_t u;
            std::memcpy(&u, &x, sizeof(u));
            ++u;
            std::memcpy(&x, &u, sizeof(u));
            return x;
        }

        // Cover the hardware prefetcher's ramp-up at the start of a leaf scan. The near
        // child's lines arrive under the bound computations; the far child's under the
        // whole near-subtree scan. Pruned-leaf prefetches waste a little bandwidth, but
        // the traversal is latency-bound, not bandwidth-bound.
        void prefetch_leaf(const Node &child) const noexcept
        {
#if defined(__GNUC__) or defined(__clang__)
            if (child.leaf != nil)
            {
                const auto *p = reinterpret_cast<const char *>(leaves_[child.leaf].data.data());
                for (auto i = 0UL; i < 8UL; ++i)
                {
                    __builtin_prefetch(p + 64UL * i, 0, 3);
                }
            }
#endif
        }

        // Queries stream most of the tree per call, so back the storage with huge pages
        // where the kernel supports it (no-op unless THP is in madvise/always mode).
        static void advise_huge([[maybe_unused]] void *p, [[maybe_unused]] std::size_t bytes) noexcept
        {
#ifdef __linux__
            static const auto page = static_cast<std::uintptr_t>(sysconf(_SC_PAGESIZE));
            auto lo = reinterpret_cast<std::uintptr_t>(p);
            auto hi = lo + bytes;
            lo = (lo + page - 1) & ~(page - 1);
            hi &= ~(page - 1);
            if (hi > lo)
            {
                madvise(reinterpret_cast<void *>(lo), hi - lo, MADV_HUGEPAGE);
            }
#endif
        }

        // Squared lower bound from the query to the node's box under the chordal metric.
        static auto bound2(const Node &n, const float *q) noexcept -> float
        {
            auto b = 0.F;
            constexpr auto vec_end = (dim / width) * width;
            if constexpr (vec_end > 0)
            {
                auto acc = Row::fill(0.F);
                for (auto j = 0U; j < vec_end; j += width)
                {
                    const Row lo(n.lo.data() + j, false);
                    const Row hi(n.hi.data() + j, false);
                    const Row qv(q + j, false);
                    const auto d = (lo - qv).max(qv - hi).max(0.F);
                    acc = acc + d * d;
                }

                b = acc.hsum();
            }

            for (auto j = vec_end; j < dim; ++j)
            {
                const auto d = std::max(std::max(n.lo[j] - q[j], q[j] - n.hi[j]), 0.F);
                b += d * d;
            }

            // The passes above charged quaternion lanes at +q; swap each block's
            // contribution for the sign-minimized one. May round slightly negative,
            // which only over-visits, never over-prunes.
            for (const auto offset : Robot::so3_offsets)
            {
                auto bp = 0.F, bm = 0.F;
                for (auto j = offset; j < offset + 4; ++j)
                {
                    const auto dp = std::max(std::max(n.lo[j] - q[j], q[j] - n.hi[j]), 0.F);
                    bp += dp * dp;
                    const auto dm = std::max(std::max(n.lo[j] + q[j], -q[j] - n.hi[j]), 0.F);
                    bm += dm * dm;
                }

                b += std::min(bp, bm) - bp;
            }

            return b;
        }

        template <typename Bound, typename Visit>
        void scan_leaf(const Leaf &leaf, const Rows &qp, Bound &&bound, Visit &&visit) const noexcept
        {
            const auto active = (leaf.count + width - 1) / width;
            for (auto block = 0U; block < active; ++block)
            {
                // Skip the block once every lane's partial sum strictly exceeds the live
                // bound: contributions only grow, so no candidate (including index
                // tie-breaks at equal distance) can be lost. A lane is still viable iff
                // partial <= bound, i.e. cutoff - partial > 0 with cutoff = next_up(bound).
                const auto cutoff = Row::fill(next_up(bound()));

                auto base = Row::fill(0.F);
                bool viable = true;
                for (auto i = 0U; i < n_base; ++i)
                {
                    if (i != 0 and (i % checkpoint) == 0 and (cutoff - base).max(0.F).hsum() == 0.F)
                    {
                        viable = false;
                        break;
                    }

                    const auto j = base_dims[i];
                    const auto diff = leaf.row(block, j) - qp[j];
                    base = base + diff * diff;
                }

                if (not viable)
                {
                    continue;
                }

                for (const auto offset : Robot::so3_offsets)
                {
                    auto dp = Row::fill(0.F), dm = Row::fill(0.F);
                    for (auto j = offset; j < offset + 4; ++j)
                    {
                        const auto r = leaf.row(block, j);
                        const auto p = r - qp[j];
                        dp = dp + p * p;
                        const auto m = r + qp[j];
                        dm = dm + m * m;
                    }

                    base = base + dp.min(dm);
                }

                alignas(FloatVectorAlignment) std::array<float, Row::num_scalars_rounded> acc;
                base.to_array(acc);

                const auto lanes = std::min<std::size_t>(width, leaf.count - block * width);
                for (auto lane = 0U; lane < lanes; ++lane)
                {
                    visit(acc[lane], leaf.indices[block * width + lane]);
                }
            }
        }

        void search_one(
            std::uint32_t ni,
            const float *q,
            const Rows &qp,
            float &best_d2,
            std::size_t &best_index) const noexcept
        {
            const auto &n = nodes_[ni];
            if (n.leaf != nil)
            {
                scan_leaf(
                    leaves_[n.leaf],
                    qp,
                    [&best_d2, this] { return best_d2 * prune_scale_; },
                    [&best_d2, &best_index](float d2, std::uint32_t index)
                    {
                        if (d2 < best_d2 or (d2 == best_d2 and index < best_index))
                        {
                            best_d2 = d2;
                            best_index = index;
                        }
                    });
                return;
            }

            const auto &c0 = nodes_[n.children[0]];
            const auto &c1 = nodes_[n.children[1]];
            prefetch_leaf(c0);
            prefetch_leaf(c1);

            const std::array<float, 2> bounds = {bound2(c0, q), bound2(c1, q)};
            const auto near = (bounds[0] <= bounds[1]) ? 0U : 1U;

            if (bounds[near] <= best_d2 * prune_scale_)
            {
                search_one(n.children[near], q, qp, best_d2, best_index);
            }

            if (bounds[1 - near] <= best_d2 * prune_scale_)
            {
                search_one(n.children[1 - near], q, qp, best_d2, best_index);
            }
        }

        void search_k(std::uint32_t ni, const float *q, const Rows &qp, std::size_t k, float r2)
            const noexcept
        {
            const auto &n = nodes_[ni];
            if (n.leaf != nil)
            {
                scan_leaf(
                    leaves_[n.leaf],
                    qp,
                    [this, k, r2]
                    { return ((scratch_.size() < k) ? r2 : scratch_.front().first) * prune_scale_; },
                    [this, k, r2](float d2, std::uint32_t index)
                    {
                        if (d2 > r2)
                        {
                            return;
                        }

                        const auto candidate = std::make_pair(d2, index);
                        if (scratch_.size() < k)
                        {
                            scratch_.push_back(candidate);
                            std::push_heap(scratch_.begin(), scratch_.end());
                        }
                        else if (candidate < scratch_.front())
                        {
                            std::pop_heap(scratch_.begin(), scratch_.end());
                            scratch_.back() = candidate;
                            std::push_heap(scratch_.begin(), scratch_.end());
                        }
                    });
                return;
            }

            const auto &c0 = nodes_[n.children[0]];
            const auto &c1 = nodes_[n.children[1]];
            prefetch_leaf(c0);
            prefetch_leaf(c1);

            const std::array<float, 2> bounds = {bound2(c0, q), bound2(c1, q)};
            const auto near = (bounds[0] <= bounds[1]) ? 0U : 1U;

            const auto limit = [&]()
            { return ((scratch_.size() < k) ? r2 : scratch_.front().first) * prune_scale_; };
            if (bounds[near] <= limit())
            {
                search_k(n.children[near], q, qp, k, r2);
            }

            if (bounds[1 - near] <= limit())
            {
                search_k(n.children[1 - near], q, qp, k, r2);
            }
        }

        void split_leaf(std::uint32_t ni)
        {
            const auto li = nodes_[ni].leaf;

            auto axis = 0U;
            auto best_extent = -inf;
            for (auto j = 0U; j < dim; ++j)
            {
                const auto extent = nodes_[ni].hi[j] - nodes_[ni].lo[j];
                if (extent > best_extent)
                {
                    best_extent = extent;
                    axis = j;
                }
            }

            const Leaf snapshot = leaves_[li];

            std::array<std::uint32_t, leaf_capacity> order;
            std::iota(order.begin(), order.end(), 0U);
            constexpr auto half = leaf_capacity / 2;
            std::nth_element(
                order.begin(),
                order.begin() + half,
                order.end(),
                [&snapshot, axis](std::uint32_t a, std::uint32_t b)
                {
                    const auto ca = snapshot.coord(a, axis);
                    const auto cb = snapshot.coord(b, axis);
                    if (ca != cb)
                    {
                        return ca < cb;
                    }

                    return snapshot.indices[a] < snapshot.indices[b];
                });
            const auto split = snapshot.coord(order[half], axis);

            // Preserve insertion order within each side for determinism.
            std::sort(order.begin(), order.begin() + half);
            std::sort(order.begin() + half, order.end());

            const auto ri = static_cast<std::uint32_t>(leaves_.size());
            leaves_.emplace_back();
            leaves_[li] = Leaf{};

            Node left, right;
            left.lo.fill(inf);
            left.hi.fill(-inf);
            left.leaf = li;
            right.lo.fill(inf);
            right.hi.fill(-inf);
            right.leaf = ri;

            std::array<float, dim> tmp;
            for (auto i = 0U; i < leaf_capacity; ++i)
            {
                const auto pos = order[i];
                for (auto j = 0U; j < dim; ++j)
                {
                    tmp[j] = snapshot.coord(pos, j);
                }

                auto &child = (i < half) ? left : right;
                grow(child, tmp);
                leaves_[child.leaf].append(tmp, snapshot.indices[pos]);
            }

            const auto ci = static_cast<std::uint32_t>(nodes_.size());
            nodes_.push_back(left);
            nodes_.push_back(right);

            auto &parent = nodes_[ni];
            parent.leaf = nil;
            parent.axis = axis;
            parent.split = split;
            parent.children = {ci, ci + 1};
        }

        float prune_scale_ = 1.F;
        std::vector<Node> nodes_;
        std::vector<Leaf> leaves_;
        std::size_t size_ = 0;
        mutable std::vector<std::pair<float, std::uint32_t>> scratch_;
    };
}  // namespace vamp::planning
