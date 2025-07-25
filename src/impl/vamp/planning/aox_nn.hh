#pragma once

// Implementation taken from OMPL, license reproduced here:

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Mark Moll, Bryant Gipson */

#include <algorithm>
#include <functional>
#include <queue>
#include <utility>
#include <unordered_set>

#include <vamp/vector.hh>
#include <vamp/random/rng.hh>

#include <Eigen/Dense>

namespace vamp::planning
{
    template <std::size_t dimension>
    struct GNATNode
    {
        std::size_t index;
        float cost;
        FloatVector<dimension> array;

        // AOX distance function
        inline static auto distance(const GNATNode<dimension> &a, const GNATNode<dimension> &b) -> float
        {
            // Configuration space distance + Cost space distance
            return std::sqrt(std::pow(a.array.distance(b.array), 2) + std::pow(a.cost - b.cost, 2));
        }
    };

    template <std::size_t dimension>
    inline bool operator==(const GNATNode<dimension> &lhs, const GNATNode<dimension> &rhs)
    {
        return lhs.index == rhs.index;
    }

    template <std::size_t dimension>
    inline bool operator!=(const GNATNode<dimension> &lhs, const GNATNode<dimension> &rhs)
    {
        return not(lhs == rhs);
    }

    /** \brief Geometric Near-neighbor Access Tree (GNAT), a data
        structure for nearest neighbor search.

        If GNAT_SAMPLER is defined, then extra code will be enabled to sample
        elements from the GNAT with probability inversely proportial to their
        local density.

        @par External documentation
        S. Brin, Near neighbor search in large metric spaces, in <em>Proc. 21st
        Conf. on Very Large Databases (VLDB)</em>, pp. 574–584, 1995.

        B. Gipson, M. Moll, and L.E. Kavraki, Resolution independent density
         estimation for motion planning in high-dimensional spaces, in
        <em>IEEE Intl. Conf. on Robotics and Automation</em>, 2013.
        [[PDF]](http://www.kavrakilab.org/publications/gipson-moll2013resolution-independent-density.pdf)
    */
    template <typename _T>
    class NearestNeighborsGNAT
    {
    protected:
        // internally, we use a priority queue for nearest neighbors, paired
        // with their distance to the query point
        using NearQueue = std::priority_queue<std::pair<float, const _T *>>;

        // another internal data structure is a priority queue of nodes to
        // check next for possible nearest neighbors
        class Node;
        using NodeDist = std::pair<Node *, float>;

        struct NodeDistCompare
        {
            auto operator()(const NodeDist &n0, const NodeDist &n1) const noexcept -> bool
            {
                return (n0.second - n0.first->maxRadius_) > (n1.second - n1.first->maxRadius_);
            }
        };

        using NodeQueue = std::priority_queue<NodeDist, std::vector<NodeDist>, NodeDistCompare>;

    public:
        using DistanceFunction = std::function<float(const _T &, const _T &)>;

        explicit NearestNeighborsGNAT(
            std::size_t degree = 8,
            std::size_t minDegree = 4,
            std::size_t maxDegree = 12,
            std::size_t maxNumPtsPerLeaf = 50,
            std::size_t removedCacheSize = 500,
            bool rebalancing = false) noexcept
          : degree_(degree)
          , minDegree_(std::min(degree, minDegree))
          , maxDegree_(std::max(maxDegree, degree))
          , maxNumPtsPerLeaf_(maxNumPtsPerLeaf)
          , rebuildSize_(rebalancing ? maxNumPtsPerLeaf * degree : std::numeric_limits<std::size_t>::max())
          , removedCacheSize_(removedCacheSize)
        {
        }

        ~NearestNeighborsGNAT() noexcept
        {
            delete tree_;
        }

        void clear() noexcept
        {
            if (tree_)
            {
                delete tree_;
                tree_ = nullptr;
            }

            size_ = 0;
            removed_.clear();
            if (rebuildSize_ != std::numeric_limits<std::size_t>::max())
            {
                rebuildSize_ = maxNumPtsPerLeaf_ * degree_;
            }
        }

        void add(const _T &data) noexcept
        {
            if (tree_)
            {
                if (isRemoved(data))
                {
                    rebuildDataStructure();
                }
                tree_->add(*this, data);
            }
            else
            {
                tree_ = new Node(degree_, maxNumPtsPerLeaf_, data);
                size_ = 1;
            }
        }

        void add(const std::vector<_T> &data) noexcept
        {
            if (tree_)
            {
                for (const auto &elt : data)
                {
                    add(elt);
                }
            }
            else if (not data.empty())
            {
                tree_ = new Node(degree_, maxNumPtsPerLeaf_, data[0]);
                tree_->data_.insert(tree_->data_.end(), data.begin() + 1, data.end());
                size_ += data.size();

                if (tree_->needToSplit(*this))
                {
                    tree_->split(*this);
                }
            }
        }

        /// \brief Rebuild the internal data structure.
        void rebuildDataStructure() noexcept
        {
            std::vector<_T> lst;
            lst.reserve(size());
            list(lst);
            clear();
            add(lst);
        }

        /// \brief Remove data from the tree.
        /// The element won't actually be removed immediately, but just marked
        /// for removal in the removed_ cache. When the cache is full, the tree
        /// will be rebuilt and the elements marked for removal will actually
        /// be removed.
        auto remove(const _T &data) noexcept -> bool
        {
            if (size_ == 0U)
            {
                return false;
            }

            NearQueue nbhQueue;

            // find data in tree
            const bool isPivot = nearestKInternal(data, 1, nbhQueue);
            const _T *d = nbhQueue.top().second;

            if (*d != data)
            {
                return false;
            }

            removed_.insert(d);
            size_--;

            // if we removed a pivot or if the capacity of removed elements
            // has been reached, we rebuild the entire GNAT
            if (isPivot or removed_.size() >= removedCacheSize_)
            {
                rebuildDataStructure();
            }

            return true;
        }

        auto nearest(const _T &data) const noexcept -> _T
        {
            if (size_)
            {
                NearQueue nbhQueue;
                nearestKInternal(data, 1, nbhQueue);

                if (not nbhQueue.empty())
                {
                    return *nbhQueue.top().second;
                }
            }
        }

        /// Return the k nearest neighbors in sorted order
        void nearestK(const _T &data, std::size_t k, std::vector<_T> &nbh) const noexcept
        {
            nbh.clear();

            if (k != 0 and size_)
            {
                NearQueue nbhQueue;
                nearestKInternal(data, k, nbhQueue);
                postprocessNearest(nbhQueue, nbh);
            }
        }

        /// Return the nearest neighbors within distance \c radius in sorted order
        void nearestR(const _T &data, float radius, std::vector<_T> &nbh) const noexcept
        {
            nbh.clear();

            if (size_)
            {
                NearQueue nbhQueue;
                nearestRInternal(data, radius, nbhQueue);
                postprocessNearest(nbhQueue, nbh);
            }
        }

        auto size() const noexcept -> std::size_t
        {
            return size_;
        }

        void list(std::vector<_T> &data) const noexcept
        {
            data.clear();
            data.reserve(size());

            if (tree_)
            {
                tree_->list(*this, data);
            }
        }

    protected:
        using GNAT = NearestNeighborsGNAT<_T>;

        /// Return true iff data has been marked for removal.
        auto isRemoved(const _T &data) const noexcept -> bool
        {
            return not removed_.empty() and removed_.find(&data) != removed_.end();
        }

        /// \brief Return in nbhQueue the k nearest neighbors of data.
        /// For k=1, return true if the nearest neighbor is a pivot.
        /// (which is important during removal; removing pivots is a
        /// special case).
        auto nearestKInternal(const _T &data, std::size_t k, NearQueue &nbhQueue) const noexcept -> bool
        {
            NodeDist nodeDist;
            NodeQueue nodeQueue;

            const float dist = _T::distance(data, tree_->pivot_);
            bool isPivot = tree_->insertNeighborK(nbhQueue, k, tree_->pivot_, data, dist);
            tree_->nearestK(*this, data, k, nbhQueue, nodeQueue, isPivot);

            while (not nodeQueue.empty())
            {
                dist = nbhQueue.top().first;  // note the difference with nearestRInternal
                nodeDist = nodeQueue.top();
                nodeQueue.pop();

                if (nbhQueue.size() == k and (nodeDist.second > nodeDist.first->maxRadius_ + dist or
                                              nodeDist.second < nodeDist.first->minRadius_ - dist))
                {
                    continue;
                }

                nodeDist.first->nearestK(*this, data, k, nbhQueue, nodeQueue, isPivot);
            }

            return isPivot;
        }

        /// \brief Return in nbhQueue the elements that are within distance radius of data.
        void nearestRInternal(const _T &data, float radius, NearQueue &nbhQueue) const noexcept
        {
            const float dist = radius;  // note the difference with nearestKInternal
            NodeQueue nodeQueue;
            NodeDist nodeDist;

            tree_->insertNeighborR(nbhQueue, radius, tree_->pivot_, _T::distance(data, tree_->pivot_));
            tree_->nearestR(*this, data, radius, nbhQueue, nodeQueue);

            while (not nodeQueue.empty())
            {
                nodeDist = nodeQueue.top();
                nodeQueue.pop();
                if (nodeDist.second > nodeDist.first->maxRadius_ + dist or
                    nodeDist.second < nodeDist.first->minRadius_ - dist)
                {
                    continue;
                }

                nodeDist.first->nearestR(*this, data, radius, nbhQueue, nodeQueue);
            }
        }

        /// \brief Convert the internal data structure used for storing neighbors
        /// to the vector that NearestNeighbor API requires.
        void postprocessNearest(NearQueue &nbhQueue, std::vector<_T> &nbh) const noexcept
        {
            nbh.resize(nbhQueue.size());
            for (auto it = nbh.rbegin(); it != nbh.rend(); it++, nbhQueue.pop())
            {
                *it = *nbhQueue.top().second;
            }
        }

        void kcenters(
            const std::vector<_T> &data,
            std::size_t k,
            std::vector<std::size_t> &centers,
            Eigen::MatrixXf &dists) noexcept
        {
            // array containing the minimum distance between each data point
            // and the centers computed so far
            std::vector<float> minDist(data.size(), std::numeric_limits<float>::infinity());
            centers.clear();
            centers.reserve(k);

            if (static_cast<std::size_t>(dists.rows()) < data.size() or
                static_cast<std::size_t>(dists.cols()) < k)
            {
                dists.resize(std::max(2U * (std::size_t)dists.rows() + 1U, data.size()), k);
            }
            // first center is picked randomly

            centers.emplace_back(rng_.uniform_integer(0, static_cast<int>(data.size()) - 1));

            for (auto i = 1U; i < k; ++i)
            {
                std::size_t ind = 0;
                const _T &center = data[centers[i - 1]];
                float maxDist = -std::numeric_limits<float>::infinity();

                for (auto j = 0U; j < data.size(); ++j)
                {
                    if ((dists(j, i - 1) = _T::distance(data[j], center)) < minDist[j])
                    {
                        minDist[j] = dists(j, i - 1);
                    }

                    // the j-th center is the one furthest away from center 0,..,j-1
                    if (minDist[j] > maxDist)
                    {
                        ind = j;
                        maxDist = minDist[j];
                    }
                }

                // no more centers available
                if (maxDist < std::numeric_limits<float>::epsilon())
                {
                    break;
                }

                centers.emplace_back(ind);
            }

            const _T &center = data[centers.back()];
            const auto i = centers.size() - 1;

            for (auto j = 0U; j < data.size(); ++j)
            {
                dists(j, i) = _T::distance(data[j], center);
            }
        }

        /// The class used internally to define the GNAT.
        class Node
        {
        public:
            /// \brief Construct a node of given degree with at most
            /// \e capacity data elements and with given pivot.
            Node(std::size_t degree, std::size_t capacity, _T pivot)
              : degree_(degree)
              , pivot_(std::move(pivot))
              , minRadius_(std::numeric_limits<float>::infinity())
              , maxRadius_(-minRadius_)
              , minRange_(degree, minRadius_)
              , maxRange_(degree, maxRadius_)
            {
                // The "+1" is needed because we add an element before we check whether to split
                data_.reserve(capacity + 1);
            }

            ~Node()
            {
                for (auto &child : children_)
                {
                    delete child;
                }
            }

            /// \brief Update minRadius_ and maxRadius_, given that an element
            /// was added with distance dist to the pivot.
            void updateRadius(float dist)
            {
                if (minRadius_ > dist)
                {
                    minRadius_ = dist;
                }

                if (maxRadius_ < dist)
                {
                    maxRadius_ = dist;
                }
            }

            /// \brief Update minRange_[i] and maxRange_[i], given that an
            /// element was added to the i-th child of the parent that has
            /// distance dist to this Node's pivot.
            void updateRange(std::size_t i, float dist)
            {
                if (minRange_[i] > dist)
                {
                    minRange_[i] = dist;
                }

                if (maxRange_[i] < dist)
                {
                    maxRange_[i] = dist;
                }
            }

            /// Add an element to the tree rooted at this node.
            void add(GNAT &gnat, const _T &data)
            {
                if (children_.empty())
                {
                    data_.emplace_back(data);
                    gnat.size_++;
                    if (needToSplit(gnat))
                    {
                        if (not gnat.removed_.empty())
                        {
                            gnat.rebuildDataStructure();
                        }
                        else if (gnat.size_ >= gnat.rebuildSize_)
                        {
                            gnat.rebuildSize_ <<= 1;
                            gnat.rebuildDataStructure();
                        }
                        else
                        {
                            split(gnat);
                        }
                    }
                }
                else
                {
                    std::vector<float> dist(children_.size());
                    float minDist = dist[0] = _T::distance(data, children_[0]->pivot_);
                    std::size_t minInd = 0;

                    for (auto i = 1U; i < children_.size(); ++i)
                    {
                        if ((dist[i] = _T::distance(data, children_[i]->pivot_)) < minDist)
                        {
                            minDist = dist[i];
                            minInd = i;
                        }
                    }

                    for (auto i = 0U; i < children_.size(); ++i)
                    {
                        children_[i]->updateRange(minInd, dist[i]);
                    }

                    children_[minInd]->updateRadius(minDist);
                    children_[minInd]->add(gnat, data);
                }
            }

            /// Return true iff the node needs to be split into child nodes.
            auto needToSplit(const GNAT &gnat) const noexcept -> bool
            {
                const auto sz = data_.size();
                return sz > gnat.maxNumPtsPerLeaf_ and sz > degree_;
            }

            /// \brief The split operation finds pivot elements for the child
            /// nodes and moves each data element of this node to the appropriate
            /// child node.
            void split(GNAT &gnat)
            {
                Eigen::MatrixXf dists(data_.size(), degree_);
                std::vector<std::size_t> pivots;

                children_.reserve(degree_);
                gnat.kcenters(data_, degree_, pivots, dists);
                for (const auto &pivot : pivots)
                {
                    children_.emplace_back(new Node(degree_, gnat.maxNumPtsPerLeaf_, data_[pivot]));
                }

                degree_ = pivots.size();  // in case fewer than degree_ pivots were found

                for (auto j = 0U; j < data_.size(); ++j)
                {
                    std::size_t k = 0;
                    for (auto i = 1U; i < degree_; ++i)
                    {
                        if (dists(j, i) < dists(j, k))
                        {
                            k = i;
                        }
                    }

                    auto *child = children_[k];
                    if (j != pivots[k])
                    {
                        child->data_.emplace_back(data_[j]);
                        child->updateRadius(dists(j, k));
                    }

                    for (auto i = 0U; i < degree_; ++i)
                    {
                        children_[i]->updateRange(k, dists(j, i));
                    }
                }

                for (auto &child : children_)
                {
                    // make sure degree lies between minDegree_ and maxDegree_
                    child->degree_ = std::min(
                        std::max(
                            static_cast<std::size_t>((degree_ * child->data_.size()) / data_.size()),
                            gnat.minDegree_),
                        gnat.maxDegree_);

                    // singleton
                    if (child->minRadius_ >= std::numeric_limits<float>::infinity())
                    {
                        child->minRadius_ = child->maxRadius_ = 0.;
                    }
                }

                // this does more than clear(); it also sets capacity to 0 and frees the memory
                std::vector<_T> tmp;
                data_.swap(tmp);

                // check if new leaves need to be split
                for (auto &child : children_)
                {
                    if (child->needToSplit(gnat))
                    {
                        child->split(gnat);
                    }
                }
            }

            /// Insert data in nbh if it is a near neighbor. Return true iff data was added to nbh.
            auto insertNeighborK(NearQueue &nbh, std::size_t k, const _T &data, const _T &key, float dist)
                const noexcept -> bool
            {
                if (nbh.size() < k)
                {
                    nbh.emplace(dist, &data);
                    return true;
                }

                if (dist < nbh.top().first or (dist < std::numeric_limits<float>::epsilon() and data == key))
                {
                    nbh.pop();
                    nbh.emplace(dist, &data);
                    return true;
                }

                return false;
            }

            /// \brief Compute the k nearest neighbors of data in the tree.
            /// For k=1, isPivot is true if the nearest neighbor is a pivot
            /// (which is important during removal; removing pivots is a
            /// special case). The nodeQueue, which contains other Nodes
            /// that need to be checked for nearest neighbors, is updated.
            void nearestK(
                const GNAT &gnat,
                const _T &data,
                std::size_t k,
                NearQueue &nbh,
                NodeQueue &nodeQueue,
                bool &isPivot) const noexcept
            {
                for (const auto &d : data_)
                {
                    if (not gnat.isRemoved(d))
                    {
                        if (insertNeighborK(nbh, k, d, data, _T::distance(data, d)))
                        {
                            isPivot = false;
                        }
                    }
                }

                if (!children_.empty())
                {
                    std::size_t sz = children_.size();
                    std::size_t offset = gnat.offset_++;
                    std::vector<float> distToPivot(sz);
                    std::vector<int> permutation(sz);

                    for (auto i = 0U; i < sz; ++i)
                    {
                        permutation[i] = (i + offset) % sz;
                    }

                    for (auto i = 0U; i < sz; ++i)
                    {
                        if (permutation[i] >= 0)
                        {
                            const auto &child = children_[permutation[i]];
                            distToPivot[permutation[i]] = _T::distance(data, child->pivot_);
                            if (insertNeighborK(nbh, k, child->pivot_, data, distToPivot[permutation[i]]))
                            {
                                isPivot = true;
                            }

                            if (nbh.size() == k)
                            {
                                const auto dist = nbh.top().first;  // note difference with nearestR
                                for (auto j = 0U; j < sz; ++j)
                                {
                                    if (permutation[j] >= 0 and i != j and
                                        (distToPivot[permutation[i]] - dist >
                                             child->maxRange_[permutation[j]] or
                                         distToPivot[permutation[i]] + dist <
                                             child->minRange_[permutation[j]]))
                                    {
                                        permutation[j] = -1;
                                    }
                                }
                            }
                        }
                    }

                    const auto dist = nbh.top().first;
                    for (auto p : permutation)
                    {
                        if (p >= 0)
                        {
                            const auto &child = children_[p];
                            if (nbh.size() < k or (distToPivot[p] - dist <= child->maxRadius_ and
                                                   distToPivot[p] + dist >= child->minRadius_))
                            {
                                nodeQueue.emplace(child, distToPivot[p]);
                            }
                        }
                    }
                }
            }

            /// Insert data in nbh if it is a near neighbor.
            void insertNeighborR(NearQueue &nbh, float r, const _T &data, float dist) const noexcept
            {
                if (dist <= r)
                {
                    nbh.emplace(dist, &data);
                }
            }

            void nearestR(const GNAT &gnat, const _T &data, float r, NearQueue &nbh, NodeQueue &nodeQueue)
                const noexcept
            {
                float dist = r;  // note difference with nearestK

                for (const auto &d : data_)
                {
                    if (not gnat.isRemoved(d))
                    {
                        insertNeighborR(nbh, r, d, _T::distance(data, d));
                    }
                }

                if (not children_.empty())
                {
                    std::size_t sz = children_.size();
                    std::size_t offset = gnat.offset_++;
                    std::vector<float> distToPivot(sz);
                    std::vector<int> permutation(sz);

                    // Not a random permutation, but processing the children in slightly different order is
                    // "good enough" to get a performance boost. A call to std::shuffle takes too long.
                    for (auto i = 0U; i < sz; ++i)
                    {
                        permutation[i] = (i + offset) % sz;
                    }

                    for (auto i = 0U; i < sz; ++i)
                    {
                        if (permutation[i] >= 0)
                        {
                            const auto &child = children_[permutation[i]];
                            distToPivot[permutation[i]] = _T::distance(data, child->pivot_);
                            insertNeighborR(nbh, r, child->pivot_, distToPivot[permutation[i]]);

                            for (auto j = 0U; j < sz; ++j)
                            {
                                if (permutation[j] >= 0 and i != j and
                                    (distToPivot[permutation[i]] - dist > child->maxRange_[permutation[j]] or
                                     distToPivot[permutation[i]] + dist < child->minRange_[permutation[j]]))
                                {
                                    permutation[j] = -1;
                                }
                            }
                        }
                    }

                    for (const auto &p : permutation)
                    {
                        if (p >= 0)
                        {
                            const auto &child = children_[p];

                            if (distToPivot[p] - dist <= child->maxRadius_ and
                                distToPivot[p] + dist >= child->minRadius_)
                            {
                                nodeQueue.emplace(child, distToPivot[p]);
                            }
                        }
                    }
                }
            }

            void list(const GNAT &gnat, std::vector<_T> &data) const noexcept
            {
                if (not gnat.isRemoved(pivot_))
                {
                    data.emplace_back(pivot_);
                }

                for (const auto &d : data_)
                {
                    if (not gnat.isRemoved(d))
                    {
                        data.emplace_back(d);
                    }
                }

                for (const auto &child : children_)
                {
                    child->list(gnat, data);
                }
            }

            std::size_t degree_;
            const _T pivot_;
            float minRadius_;
            float maxRadius_;
            std::vector<float> minRange_;
            std::vector<float> maxRange_;
            std::vector<_T> data_;
            std::vector<Node *> children_;
        };

        Node *tree_{nullptr};
        std::size_t degree_;
        std::size_t minDegree_;
        std::size_t maxDegree_;
        std::size_t maxNumPtsPerLeaf_;
        std::size_t size_{0};
        std::size_t rebuildSize_;
        std::size_t removedCacheSize_;
        std::unordered_set<const _T *> removed_;
        vamp::rng::Distribution rng_;
        mutable std::size_t offset_{0};
    };
}  // namespace vamp::planning
