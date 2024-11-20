#include "kdiTree.cuh"

#define WINDOW_SIZE 3

namespace CUDA
{
    __device__ __host__ kdiTree::kdiTree() {}
    __device__ __host__ kdiTree::~kdiTree() {}

    void kdiTree::Init(Eigen::Vector3f* points, size_t numberOfPoints)
    {
        this->points = points;
        this->numberOfPoints = numberOfPoints;

        visited = 0;
        cudaMallocManaged(&nodes, sizeof(kdiNode) * numberOfPoints);
        cudaMallocManaged(&kdDistances, sizeof(float));
        cudaMallocManaged(&kdFound, sizeof(kdiNode));
        cudaMallocManaged(&kdRoot, sizeof(kdiNode) * numberOfPoints);
        cudaMallocManaged(&kdQuery, sizeof(kdiNode) * numberOfPoints);
        cudaMallocManaged(&VisitedNodes, sizeof(kdiNode) * numberOfPoints);
        //cudaMalloc(&kdDistances, sizeof(float));
        //cudaMalloc(&kdFound, sizeof(kdiNode));
        //cudaMalloc(&kdRoot, sizeof(kdiNode) * numberOfPoints);
        //cudaMalloc(&kdQuery, sizeof(kdiNode) * numberOfPoints);
        //cudaMalloc(&VisitedNodes, sizeof(kdiNode) * numberOfPoints);
        cudaDeviceSynchronize();

        for (size_t i = 0; i < numberOfPoints; i++)
        {
            nodes[i].index = i;
        }
    }

    void kdiTree::Reset()
    {
        for (size_t i = 0; i < numberOfPoints; i++)
        {
            nodes[i].index = i;
        }
    }

    void kdiTree::Free(void)
    {
        cudaDeviceSynchronize();
        cudaFree(nodes);
        cudaFree(kdDistances);
        cudaFree(kdFound);
        cudaFree(kdRoot);
        cudaFree(kdQuery);
        cudaFree(VisitedNodes);
    }

    __device__ __host__ inline float kdiTree::dist(kdiNode* a, kdiNode* b, int dim)
    {
        float t, d = 0;
        while (dim--) {
            t = points[a->index][dim] - points[b->index][dim];
            d += t * t;
        }
        return d;
    }

    __device__ __host__ inline float kdiTree::dist(kdiNode* a, const Eigen::Vector3f& query, int dim)
    {
        float t, d = 0;
        while (dim--) {
            t = points[a->index][dim] - query[dim];
            d += t * t;
        }
        return d;
    }

    inline void kdiTree::swap(kdiNode* x, kdiNode* y)
    {
        size_t tmp;
        tmp = x->index;
        x->index = y->index;
        y->index = tmp;
    }

    kdiNode* kdiTree::findMedian(kdiNode* start, kdiNode* end, int idx)
    {
        if (end <= start) return NULL;
        if (end == start + 1)
            return start;

        kdiNode* p, * store, * md = start + (end - start) / 2;
        float pivot;
        while (1) {
            pivot = points[md->index][idx];

            swap(md, end - 1);
            for (store = p = start; p < end; p++) {
                if (points[p->index][idx] < pivot) {
                    if (p != store)
                        swap(p, store);
                    store++;
                }
            }
            swap(store, end - 1);

            /* median has duplicate values */
            if (points[store->index][idx] == points[md->index][idx])
                return md;

            if (store > md) end = store;
            else        start = store;
        }
    }

    kdiNode* kdiTree::buildTree(kdiNode* t, int len, int i, int dim)
    {
        kdiNode* n;

        if (!len) return 0;

        if ((n = findMedian(t, t + len, i))) {
            i = (i + 1) % dim;
            n->left = buildTree(t, n - t, i, dim);
            n->right = buildTree(n + 1, t + len - (n + 1), i, dim);
        }
        return n;
    }

    __device__ __host__ void kdiTree::findNearest(
        kdiNode* root,
        kdiNode* nd,
        int i,
        int dim,
        kdiNode** best,
        float* best_dist)
    {
        float d, dx, dx2;

        if (!root) return;
        d = dist(root, nd, dim);
        dx = points[root->index][i] - points[nd->index][i];
        dx2 = dx * dx;

        visited++;
        // std::cout << "RootID: "<< root->id  << ", ndID: "<< nd->id  << ", d: " << d << std::endl;
        if (!*best || d < *best_dist) {
            *best_dist = d;
            *best = root;
        }

        /* if chance of exact match is high */
        if (!*best_dist) return;

        if (++i >= dim) i = 0;

        findNearest(dx > 0 ? root->left : root->right, nd, i, dim, best, best_dist);
        if (dx2 >= *best_dist) return;
        findNearest(dx > 0 ? root->right : root->left, nd, i, dim, best, best_dist);
    }

    __device__ __host__ void kdiTree::findKNearest(
        kdiNode* root,
        kdiNode* nd,
        int i,
        int dim,
        kdiNode** best,
        float* best_dist,
        kdiNode* VisitedNodes)
    {
        float d, dx, dx2;

        if (!root) return;
        d = dist(root, nd, dim);
        dx = points[root->index][i] - points[nd->index][i];
        dx2 = dx * dx;

        VisitedNodes[visited] = *root;
        VisitedNodes[visited].distance = d;
        visited++;

        // std::cout << "RootID: "<< root->id  << ", ndID: "<< nd->id  << ", d: " << d << std::endl;

        if (!*best || d < *best_dist) {
            *best_dist = d;
            *best = root;
        }

        /* if chance of exact match is high */
        if (!*best_dist) return;

        if (++i >= dim) i = 0;

        findKNearest(dx > 0 ? root->left : root->right, nd, i, dim, best, best_dist, VisitedNodes);
        if (dx2 >= *best_dist) return;
        findKNearest(dx > 0 ? root->right : root->left, nd, i, dim, best, best_dist, VisitedNodes);
    }

    __device__ __host__ void kdiTree::findKNearest(
        kdiNode* root,
        const Eigen::Vector3f& query,
        int i,
        int dim,
        kdiNode** best,
        float* best_dist,
        kdiNode* VisitedNodes)
    {
        float d, dx, dx2;

        if (!root) return;
        d = dist(root, query, dim);
        dx = points[root->index][i] - query[i];
        dx2 = dx * dx;

        VisitedNodes[visited] = *root;
        VisitedNodes[visited].distance = d;
        visited++;

        // std::cout << "RootID: "<< root->id  << ", ndID: "<< nd->id  << ", d: " << d << std::endl;

        if (!*best || d < *best_dist) {
            *best_dist = d;
            *best = root;
        }

        /* if chance of exact match is high */
        if (!*best_dist) return;

        if (++i >= dim) i = 0;

        findKNearest(dx > 0 ? root->left : root->right, query, i, dim, best, best_dist, VisitedNodes);
        if (dx2 >= *best_dist) return;
        findKNearest(dx > 0 ? root->right : root->left, query, i, dim, best, best_dist, VisitedNodes);
    }

    __device__ __host__ void kdiTree::findKNN(kdiNode& targetNode)
    {
        this->visited = 0;
        this->kdFound = 0;

        this->findKNearest(this->kdRoot, &targetNode, 0, 3, &this->kdFound, this->kdDistances, VisitedNodes);

        this->sortNodes(this->visited);
    }
    
    __device__ __host__ void kdiTree::findKNN(const Eigen::Vector3f& query)
    {
        this->visited = 0;
        this->kdFound = 0;

        this->findKNearest(this->kdRoot, query, 10, 3, &this->kdFound, this->kdDistances, VisitedNodes);

        this->sortNodes(this->visited);
    }

    __device__ __host__ inline void kdiTree::sortNodes(int visitedNum)
    {
        for (int idx = 0; idx < visitedNum; idx++) {
            // Shift values (and indexes) higher
            kdiNode tmpNodes;
            int j = idx;
            // Store current distance and associated nIdx
            kdiNode currNodes = VisitedNodes[j];
            while (j > 0 && VisitedNodes[j - 1].distance > currNodes.distance) {

                tmpNodes = VisitedNodes[j - 1];

                VisitedNodes[j - 1] = currNodes;

                VisitedNodes[j] = tmpNodes;

                --j;
            }
        }
    }
}
