#pragma once

#include <algorithm>
#include <iostream>
#include <queue>
#include <random>
#include <unordered_set>
#include <utility>

namespace vamp::planning
{
    template <std::size_t dimension>
    struct GNATNode
    {
        int index;
        float cost;
        FloatVector<dimension> array;
    };

    template <std::size_t dimension> inline bool operator==(const GNATNode<dimension>& lhs, const GNATNode<dimension>& rhs) { return lhs.index == rhs.index; }
    template <std::size_t dimension> inline bool operator!=(const GNATNode<dimension>& lhs, const GNATNode<dimension>& rhs) { return !(lhs == rhs); }



    /** \brief Abstract representation of a container that can perform nearest neighbors queries */
    template <typename _T>
    class NearestNeighbors
    {
    public:
        /** \brief The definition of a distance function */
        using DistanceFunction = std::function<float(const _T &, const _T &)>;

        NearestNeighbors() = default;

        virtual ~NearestNeighbors() = default;

        /** \brief Set the distance function to use */
        virtual void setDistanceFunction(const DistanceFunction &distFun)
        {
            distFun_ = distFun;
        }

        /** \brief Get the distance function used */
        const DistanceFunction &getDistanceFunction() const
        {
            return distFun_;
        }

        /** \brief Return true if the solutions reported by this data structure
            are sorted, when calling nearestK / nearestR. */
        virtual bool reportsSortedResults() const = 0;

        /** \brief Clear the datastructure */
        virtual void clear() = 0;

        /** \brief Add an element to the datastructure */
        virtual void add(const _T &data) = 0;

        /** \brief Add a vector of points */
        virtual void add(const std::vector<_T> &data)
        {
            for (const auto &elt : data)
                add(elt);
        }

        /** \brief Remove an element from the datastructure */
        virtual bool remove(const _T &data) = 0;

        /** \brief Get the nearest neighbor of a point */
        virtual _T nearest(const _T &data) const = 0;

        /** \brief Get the k-nearest neighbors of a point
         *
         * All the nearest neighbor structures currently return the neighbors in
         * sorted order, but this is not required.
         */
        virtual void nearestK(const _T &data, std::size_t k, std::vector<_T> &nbh) const = 0;

        /** \brief Get the nearest neighbors of a point, within a specified radius
         *
         * All the nearest neighbor structures currently return the neighbors in
         * sorted order, but this is not required.
         */
        virtual void nearestR(const _T &data, float radius, std::vector<_T> &nbh) const = 0;

        /** \brief Get the number of elements in the datastructure */
        virtual std::size_t size() const = 0;

        /** \brief Get all the elements in the datastructure */
        virtual void list(std::vector<_T> &data) const = 0;

    protected:
        /** \brief The used distance function */
        DistanceFunction distFun_;
    };




    
    /** \brief An instance of this class can be used to greedily select a given
        number of representatives from a set of data points that are all far
        apart from each other. */
    template <typename _T>
    class GreedyKCenters
    {
    public:
        /** \brief The definition of a distance function */
        using DistanceFunction = std::function<float(const _T &, const _T &)>;
        /** \brief A matrix type for storing distances between points and centers */
        using Matrix = Eigen::MatrixXd;

        GreedyKCenters() = default;

        virtual ~GreedyKCenters() = default;

        /** \brief Set the distance function to use */
        void setDistanceFunction(const DistanceFunction &distFun)
        {
            distFun_ = distFun;
        }

        /** \brief Get the distance function used */
        const DistanceFunction &getDistanceFunction() const
        {
            return distFun_;
        }

        /** \brief Greedy algorithm for selecting k centers
            \param data a vector of data points
            \param k the desired number of centers
            \param centers a vector of length k containing the indices into
                data of the k centers
            \param dists a matrix such that dists(i,j) is the distance
                between data[i] and data[center[j]]
        */
        void kcenters(const std::vector<_T> &data, unsigned int k, std::vector<unsigned int> &centers, Matrix &dists)
        {
            // array containing the minimum distance between each data point
            // and the centers computed so far
            std::vector<float> minDist(data.size(), std::numeric_limits<float>::infinity());
            centers.clear();
            centers.reserve(k);
            if ((std::size_t)dists.rows() < data.size() || (std::size_t)dists.cols() < k)
                dists.resize(std::max(2u * (std::size_t)dists.rows() + 1u, data.size()), k);
            // first center is picked randomly

            rng_ = vamp::rng::Distribution();

            centers.push_back(rng_.uniform_integer(0, static_cast<int>(data.size()) - 1));
            for (unsigned i = 1; i < k; ++i)
            {
                unsigned ind = 0;
                const _T &center = data[centers[i - 1]];
                float maxDist = -std::numeric_limits<float>::infinity();

                for (unsigned j = 0; j < data.size(); ++j)
                {
                    if ((dists(j, i - 1) = distFun_(data[j], center)) < minDist[j])
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
                    break;
                centers.push_back(ind);
            }

            const _T &center = data[centers.back()];
            unsigned i = centers.size() - 1;

            for (unsigned j = 0; j < data.size(); ++j)
                dists(j, i) = distFun_(data[j], center);
        }

    protected:
        /** \brief The used distance function */
        DistanceFunction distFun_;

        /** Random number generator used to select first center */
        vamp::rng::Distribution rng_;
    };





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
    class NearestNeighborsGNAT : public NearestNeighbors<_T>
    {
    protected:
        /// \cond IGNORE
        // internally, we use a priority queue for nearest neighbors, paired
        // with their distance to the query point
        using NearQueue = std::priority_queue<std::pair<float, const _T *>>;

        // another internal data structure is a priority queue of nodes to
        // check next for possible nearest neighbors
        class Node;
        using NodeDist = std::pair<Node *, float>;
        struct NodeDistCompare
        {
            bool operator()(const NodeDist &n0, const NodeDist &n1) const
            {
                return (n0.second - n0.first->maxRadius_) > (n1.second - n1.first->maxRadius_);
            }
        };
        using NodeQueue = std::priority_queue<NodeDist, std::vector<NodeDist>, NodeDistCompare>;
        /// \endcond

    public:
        NearestNeighborsGNAT(unsigned int degree = 8, unsigned int minDegree = 4, unsigned int maxDegree = 12,
                             unsigned int maxNumPtsPerLeaf = 50, unsigned int removedCacheSize = 500,
                             bool rebalancing = false
                             )
          : NearestNeighbors<_T>()
          , degree_(degree)
          , minDegree_(std::min(degree, minDegree))
          , maxDegree_(std::max(maxDegree, degree))
          , maxNumPtsPerLeaf_(maxNumPtsPerLeaf)
          , rebuildSize_(rebalancing ? maxNumPtsPerLeaf * degree : std::numeric_limits<std::size_t>::max())
          , removedCacheSize_(removedCacheSize)
        {
        }

        ~NearestNeighborsGNAT() override
        {
            delete tree_;
        }
        /// \brief Set the distance function to use
        void setDistanceFunction(const typename NearestNeighbors<_T>::DistanceFunction &distFun) override
        {
            NearestNeighbors<_T>::setDistanceFunction(distFun);
            pivotSelector_.setDistanceFunction(distFun);
            if (tree_)
                rebuildDataStructure();
        }

        void clear() override
        {
            if (tree_)
            {
                delete tree_;
                tree_ = nullptr;
            }
            size_ = 0;
            removed_.clear();
            if (rebuildSize_ != std::numeric_limits<std::size_t>::max())
                rebuildSize_ = maxNumPtsPerLeaf_ * degree_;
        }

        bool reportsSortedResults() const override
        {
            return true;
        }

        void add(const _T &data) override
        {
            if (tree_)
            {
                if (isRemoved(data))
                    rebuildDataStructure();
                tree_->add(*this, data);
            }
            else
            {
                tree_ = new Node(degree_, maxNumPtsPerLeaf_, data);
                size_ = 1;
            }
        }
        void add(const std::vector<_T> &data) override
        {
            if (tree_)
                NearestNeighbors<_T>::add(data);
            else if (!data.empty())
            {
                tree_ = new Node(degree_, maxNumPtsPerLeaf_, data[0]);
                tree_->data_.insert(tree_->data_.end(), data.begin() + 1, data.end());
                size_ += data.size();
                if (tree_->needToSplit(*this))
                    tree_->split(*this);
            }
        }
        /// \brief Rebuild the internal data structure.
        void rebuildDataStructure()
        {
            std::vector<_T> lst;
            list(lst);
            clear();
            add(lst);
        }
        /// \brief Remove data from the tree.
        /// The element won't actually be removed immediately, but just marked
        /// for removal in the removed_ cache. When the cache is full, the tree
        /// will be rebuilt and the elements marked for removal will actually
        /// be removed.
        bool remove(const _T &data) override
        {
            if (size_ == 0u)
                return false;
            NearQueue nbhQueue;
            // find data in tree
            bool isPivot = nearestKInternal(data, 1, nbhQueue);
            const _T *d = nbhQueue.top().second;
            if (*d != data)
                return false;
            removed_.insert(d);
            size_--;
            // if we removed a pivot or if the capacity of removed elements
            // has been reached, we rebuild the entire GNAT
            if (isPivot || removed_.size() >= removedCacheSize_)
                rebuildDataStructure();
            return true;
        }

        _T nearest(const _T &data) const override
        {
            if (size_)
            {
                NearQueue nbhQueue;
                nearestKInternal(data, 1, nbhQueue);
                if (!nbhQueue.empty())
                    return *nbhQueue.top().second;
            }
            //throw Exception("No elements found in nearest neighbors data structure");
        }

        /// Return the k nearest neighbors in sorted order
        void nearestK(const _T &data, std::size_t k, std::vector<_T> &nbh) const override
        {
            nbh.clear();
            if (k == 0)
                return;
            if (size_)
            {
                NearQueue nbhQueue;
                nearestKInternal(data, k, nbhQueue);
                postprocessNearest(nbhQueue, nbh);
            }
        }

        /// Return the nearest neighbors within distance \c radius in sorted order
        void nearestR(const _T &data, float radius, std::vector<_T> &nbh) const override
        {
            nbh.clear();
            if (size_)
            {
                NearQueue nbhQueue;
                nearestRInternal(data, radius, nbhQueue);
                postprocessNearest(nbhQueue, nbh);
            }
        }

        std::size_t size() const override
        {
            return size_;
        }

        void list(std::vector<_T> &data) const override
        {
            data.clear();
            data.reserve(size());
            if (tree_)
                tree_->list(*this, data);
        }

        /// \brief Print a GNAT structure (mostly useful for debugging purposes).
        friend std::ostream &operator<<(std::ostream &out, const NearestNeighborsGNAT<_T> &gnat)
        {
            if (gnat.tree_)
            {
                out << *gnat.tree_;
                if (!gnat.removed_.empty())
                {
                    out << "Elements marked for removal:\n";
                    for (const auto &elt : gnat.removed_)
                        out << *elt << '\t';
                    out << std::endl;
                }
            }
            return out;
        }

        // for debugging purposes
        void integrityCheck()
        {
            std::vector<_T> lst;
            std::unordered_set<const _T *> tmp;
            // get all elements, including those marked for removal
            removed_.swap(tmp);
            list(lst);
            // check if every element marked for removal is also in the tree
            for (const auto &elt : tmp)
            {
                unsigned int i;
                for (i = 0; i < lst.size(); ++i)
                    if (lst[i] == *elt)
                        break;
                if (i == lst.size())
                {
                    // an element marked for removal is not actually in the tree
                    std::cout << "***** FAIL!! ******\n" << *this << '\n';
                    for (const auto &l : lst)
                        std::cout << l << '\t';
                    std::cout << std::endl;
                }
                assert(i != lst.size());
            }
            // restore
            removed_.swap(tmp);
            // get elements in the tree with elements marked for removal purged from the list
            list(lst);
            if (lst.size() != size_)
                std::cout << "#########################################\n" << *this << std::endl;
            assert(lst.size() == size_);
        }

    protected:
        using GNAT = NearestNeighborsGNAT<_T>;

        /// Return true iff data has been marked for removal.
        bool isRemoved(const _T &data) const
        {
            return !removed_.empty() && removed_.find(&data) != removed_.end();
        }

        /// \brief Return in nbhQueue the k nearest neighbors of data.
        /// For k=1, return true if the nearest neighbor is a pivot.
        /// (which is important during removal; removing pivots is a
        /// special case).
        bool nearestKInternal(const _T &data, std::size_t k, NearQueue &nbhQueue) const
        {
            bool isPivot;
            float dist;
            NodeDist nodeDist;
            NodeQueue nodeQueue;

            dist = NearestNeighbors<_T>::distFun_(data, tree_->pivot_);
            isPivot = tree_->insertNeighborK(nbhQueue, k, tree_->pivot_, data, dist);
            tree_->nearestK(*this, data, k, nbhQueue, nodeQueue, isPivot);
            while (!nodeQueue.empty())
            {
                dist = nbhQueue.top().first;  // note the difference with nearestRInternal
                nodeDist = nodeQueue.top();
                nodeQueue.pop();
                if (nbhQueue.size() == k && (nodeDist.second > nodeDist.first->maxRadius_ + dist ||
                                             nodeDist.second < nodeDist.first->minRadius_ - dist))
                    continue;
                nodeDist.first->nearestK(*this, data, k, nbhQueue, nodeQueue, isPivot);
            }
            return isPivot;
        }
        /// \brief Return in nbhQueue the elements that are within distance radius of data.
        void nearestRInternal(const _T &data, float radius, NearQueue &nbhQueue) const
        {
            float dist = radius;  // note the difference with nearestKInternal
            NodeQueue nodeQueue;
            NodeDist nodeDist;

            tree_->insertNeighborR(nbhQueue, radius, tree_->pivot_,
                                   NearestNeighbors<_T>::distFun_(data, tree_->pivot_));
            tree_->nearestR(*this, data, radius, nbhQueue, nodeQueue);
            while (!nodeQueue.empty())
            {
                nodeDist = nodeQueue.top();
                nodeQueue.pop();
                if (nodeDist.second > nodeDist.first->maxRadius_ + dist ||
                    nodeDist.second < nodeDist.first->minRadius_ - dist)
                    continue;
                nodeDist.first->nearestR(*this, data, radius, nbhQueue, nodeQueue);
            }
        }
        /// \brief Convert the internal data structure used for storing neighbors
        /// to the vector that NearestNeighbor API requires.
        void postprocessNearest(NearQueue &nbhQueue, std::vector<_T> &nbh) const
        {
            nbh.resize(nbhQueue.size());
            for (auto it = nbh.rbegin(); it != nbh.rend(); it++, nbhQueue.pop())
                *it = *nbhQueue.top().second;
        }

        /// The class used internally to define the GNAT.
        class Node
        {
        public:
            /// \brief Construct a node of given degree with at most
            /// \e capacity data elements and with given pivot.
            Node(int degree, int capacity, _T pivot)
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
                    delete child;
            }

            /// \brief Update minRadius_ and maxRadius_, given that an element
            /// was added with distance dist to the pivot.
            void updateRadius(float dist)
            {
                if (minRadius_ > dist)
                    minRadius_ = dist;

                if (maxRadius_ < dist)
                    maxRadius_ = dist;
            }
            /// \brief Update minRange_[i] and maxRange_[i], given that an
            /// element was added to the i-th child of the parent that has
            /// distance dist to this Node's pivot.
            void updateRange(unsigned int i, float dist)
            {
                if (minRange_[i] > dist)
                    minRange_[i] = dist;
                if (maxRange_[i] < dist)
                    maxRange_[i] = dist;
            }
            /// Add an element to the tree rooted at this node.
            void add(GNAT &gnat, const _T &data)
            {
                if (children_.empty())
                {
                    data_.push_back(data);
                    gnat.size_++;
                    if (needToSplit(gnat))
                    {
                        if (!gnat.removed_.empty())
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
                    float minDist = dist[0] = gnat.distFun_(data, children_[0]->pivot_);
                    int minInd = 0;

                    for (unsigned int i = 1; i < children_.size(); ++i)
                        if ((dist[i] = gnat.distFun_(data, children_[i]->pivot_)) < minDist)
                        {
                            minDist = dist[i];
                            minInd = i;
                        }
                    for (unsigned int i = 0; i < children_.size(); ++i)
                        children_[i]->updateRange(minInd, dist[i]);
                    children_[minInd]->updateRadius(minDist);
                    children_[minInd]->add(gnat, data);
                }
            }
            /// Return true iff the node needs to be split into child nodes.
            bool needToSplit(const GNAT &gnat) const
            {
                unsigned int sz = data_.size();
                return sz > gnat.maxNumPtsPerLeaf_ && sz > degree_;
            }
            
            /// \brief The split operation finds pivot elements for the child
            /// nodes and moves each data element of this node to the appropriate
            /// child node.
            void split(GNAT &gnat)
            {
                typename GreedyKCenters<_T>::Matrix dists(data_.size(), degree_);
                std::vector<unsigned int> pivots;

                children_.reserve(degree_);
                gnat.pivotSelector_.kcenters(data_, degree_, pivots, dists);
                for (unsigned int &pivot : pivots)
                {
                    children_.push_back(new Node(degree_, gnat.maxNumPtsPerLeaf_, data_[pivot]));
                }
                degree_ = pivots.size();  // in case fewer than degree_ pivots were found

                for (unsigned int j = 0; j < data_.size(); ++j)
                {
                    unsigned int k = 0;
                    for (unsigned int i = 1; i < degree_; ++i)
                        if (dists(j, i) < dists(j, k))
                            k = i;
                    Node *child = children_[k];
                    if (j != pivots[k])
                    {
                        child->data_.push_back(data_[j]);
                        child->updateRadius(dists(j, k));
                    }
                    for (unsigned int i = 0; i < degree_; ++i)
                        children_[i]->updateRange(k, dists(j, i));
                }

                for (auto &child : children_)
                {
                    // make sure degree lies between minDegree_ and maxDegree_
                    child->degree_ =
                        std::min(std::max((unsigned int)((degree_ * child->data_.size()) / data_.size()),
                                          gnat.minDegree_),
                                 gnat.maxDegree_);
                    // singleton
                    if (child->minRadius_ >= std::numeric_limits<float>::infinity())
                        child->minRadius_ = child->maxRadius_ = 0.;
                }

                // this does more than clear(); it also sets capacity to 0 and frees the memory
                std::vector<_T> tmp;
                data_.swap(tmp);

                // check if new leaves need to be split
                for (auto &child : children_)
                    if (child->needToSplit(gnat))
                        child->split(gnat);
            }

            /// Insert data in nbh if it is a near neighbor. Return true iff data was added to nbh.
            bool insertNeighborK(NearQueue &nbh, std::size_t k, const _T &data, const _T &key, float dist) const
            {
                if (nbh.size() < k)
                {
                    nbh.emplace(dist, &data);
                    return true;
                }
                if (dist < nbh.top().first || (dist < std::numeric_limits<float>::epsilon() && data == key))
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
            void nearestK(const GNAT &gnat, const _T &data, std::size_t k, NearQueue &nbh, NodeQueue &nodeQueue,
                          bool &isPivot) const
            {
                for (const auto &d : data_)
                    if (!gnat.isRemoved(d))
                    {
                        if (insertNeighborK(nbh, k, d, data, gnat.distFun_(data, d)))
                            isPivot = false;
                    }
                if (!children_.empty())
                {
                    float dist;
                    Node *child;
                    std::size_t sz = children_.size(), offset = gnat.offset_++;
                    std::vector<float> distToPivot(sz);
                    std::vector<int> permutation(sz);
                    for (unsigned int i = 0; i < sz; ++i)
                        permutation[i] = (i + offset) % sz;

                    for (unsigned int i = 0; i < sz; ++i)
                        if (permutation[i] >= 0)
                        {
                            child = children_[permutation[i]];
                            distToPivot[permutation[i]] = gnat.distFun_(data, child->pivot_);
                            if (insertNeighborK(nbh, k, child->pivot_, data, distToPivot[permutation[i]]))
                                isPivot = true;
                            if (nbh.size() == k)
                            {
                                dist = nbh.top().first;  // note difference with nearestR
                                for (unsigned int j = 0; j < sz; ++j)
                                    if (permutation[j] >= 0 && i != j &&
                                        (distToPivot[permutation[i]] - dist > child->maxRange_[permutation[j]] ||
                                         distToPivot[permutation[i]] + dist < child->minRange_[permutation[j]]))
                                        permutation[j] = -1;
                            }
                        }

                    dist = nbh.top().first;
                    for (auto p : permutation)
                        if (p >= 0)
                        {
                            child = children_[p];
                            if (nbh.size() < k || (distToPivot[p] - dist <= child->maxRadius_ &&
                                                   distToPivot[p] + dist >= child->minRadius_))
                                nodeQueue.emplace(child, distToPivot[p]);
                        }
                }
            }
            /// Insert data in nbh if it is a near neighbor.
            void insertNeighborR(NearQueue &nbh, float r, const _T &data, float dist) const
            {
                if (dist <= r)
                    nbh.emplace(dist, &data);
            }
            /// \brief Return all elements that are within distance r in nbh.
            /// The nodeQueue, which contains other Nodes that need to
            /// be checked for nearest neighbors, is updated.
            void nearestR(const GNAT &gnat, const _T &data, float r, NearQueue &nbh, NodeQueue &nodeQueue) const
            {
                float dist = r;  // note difference with nearestK

                for (const auto &d : data_)
                    if (!gnat.isRemoved(d))
                        insertNeighborR(nbh, r, d, gnat.distFun_(data, d));
                if (!children_.empty())
                {
                    Node *child;
                    std::size_t sz = children_.size(), offset = gnat.offset_++;
                    std::vector<float> distToPivot(sz);
                    std::vector<int> permutation(sz);
                    // Not a random permutation, but processing the children in slightly different order is
                    // "good enough" to get a performance boost. A call to std::shuffle takes too long.
                    for (unsigned int i = 0; i < sz; ++i)
                        permutation[i] = (i + offset) % sz;

                    for (unsigned int i = 0; i < sz; ++i)
                        if (permutation[i] >= 0)
                        {
                            child = children_[permutation[i]];
                            distToPivot[permutation[i]] = gnat.distFun_(data, child->pivot_);
                            insertNeighborR(nbh, r, child->pivot_, distToPivot[permutation[i]]);
                            for (unsigned int j = 0; j < sz; ++j)
                                if (permutation[j] >= 0 && i != j &&
                                    (distToPivot[permutation[i]] - dist > child->maxRange_[permutation[j]] ||
                                     distToPivot[permutation[i]] + dist < child->minRange_[permutation[j]]))
                                    permutation[j] = -1;
                        }

                    for (auto p : permutation)
                        if (p >= 0)
                        {
                            child = children_[p];
                            if (distToPivot[p] - dist <= child->maxRadius_ &&
                                distToPivot[p] + dist >= child->minRadius_)
                                nodeQueue.emplace(child, distToPivot[p]);
                        }
                }
            }


            void list(const GNAT &gnat, std::vector<_T> &data) const
            {
                if (!gnat.isRemoved(pivot_))
                    data.push_back(pivot_);
                for (const auto &d : data_)
                    if (!gnat.isRemoved(d))
                        data.push_back(d);
                for (const auto &child : children_)
                    child->list(gnat, data);
            }

            friend std::ostream &operator<<(std::ostream &out, const Node &node)
            {
                out << "\ndegree:\t" << node.degree_;
                out << "\nminRadius:\t" << node.minRadius_;
                out << "\nmaxRadius:\t" << node.maxRadius_;
                out << "\nminRange:\t";
                for (auto minR : node.minRange_)
                    out << minR << '\t';
                out << "\nmaxRange: ";
                for (auto maxR : node.maxRange_)
                    out << maxR << '\t';
                out << "\npivot:\t" << node.pivot_;
                out << "\ndata: ";
                for (auto &data : node.data_)
                    out << data << '\t';
                out << "\nthis:\t" << &node;
                out << "\nchildren:\n";
                for (auto &child : node.children_)
                    out << child << '\t';
                out << '\n';
                for (auto &child : node.children_)
                    out << *child << '\n';
                return out;
            }

            /// Number of child nodes
            unsigned int degree_;
            /// Data element stored in this Node
            const _T pivot_;
            /// Minimum distance between the pivot element and the elements stored in data_
            float minRadius_;
            /// Maximum distance between the pivot element and the elements stored in data_
            float maxRadius_;
            /// \brief The i-th element in minRange_ is the minimum distance between the
            /// pivot and any data_ element in the i-th child node of this node's parent.
            std::vector<float> minRange_;
            /// \brief The i-th element in maxRange_ is the maximum distance between the
            /// pivot and any data_ element in the i-th child node of this node's parent.
            std::vector<float> maxRange_;
            /// \brief The data elements stored in this node (in addition to the pivot
            /// element). An internal node has no elements stored in data_.
            std::vector<_T> data_;
            /// \brief The child nodes of this node. By definition, only internal nodes
            /// have child nodes.
            std::vector<Node *> children_;
        };

        /// \brief The data structure containing the elements stored in this structure.
        Node *tree_{nullptr};
        /// The desired degree of each node.
        unsigned int degree_;
        /// \brief After splitting a Node, each child Node has degree equal to
        /// the default degree times the fraction of data elements from the
        /// original node that got assigned to that child Node. However, its
        /// degree can be no less than minDegree_.
        unsigned int minDegree_;
        /// \brief After splitting a Node, each child Node has degree equal to
        /// the default degree times the fraction of data elements from the
        /// original node that got assigned to that child Node. However, its
        /// degree can be no larger than maxDegree_.
        unsigned int maxDegree_;
        /// \brief Maximum number of elements allowed to be stored in a Node before
        /// it needs to be split into several nodes.
        unsigned int maxNumPtsPerLeaf_;
        /// \brief Number of elements stored in the tree.
        std::size_t size_{0};
        /// \brief If size_ exceeds rebuildSize_, the tree will be rebuilt (and
        /// automatically rebalanced), and rebuildSize_ will be floatd.
        std::size_t rebuildSize_;
        /// \brief Maximum number of removed elements that can be stored in the
        /// removed_ cache. If the cache is full, the tree will be rebuilt with
        /// the elements in removed_ actually removed from the tree.
        std::size_t removedCacheSize_;
        /// \brief The data structure used to split data into subtrees.
        GreedyKCenters<_T> pivotSelector_;
        /// \brief Cache of removed elements.
        std::unordered_set<const _T *> removed_;

        /// \cond IGNORE
        // used to cycle through children of a node in different orders
        mutable std::size_t offset_{0};
        /// \endcond
    };
}