#include "SVO.cuh"


#define MAX_LEVELS 13

namespace Algorithm
{
    __device__ int ComputeChildIndex(Eigen::Vector3f point, int currentNodeIdx, float3 rootMin, float3 rootMax) {
        // Compute child index based on which sub-cube contains the point
        // Simplified logic for calculating voxel index within the octree region
        float3 center = make_float3((rootMin.x + rootMax.x) * 0.5f,
            (rootMin.y + rootMax.y) * 0.5f,
            (rootMin.z + rootMax.z) * 0.5f);

        int index = 0;
        if (point.x() >= center.x) index |= 1;
        if (point.y() >= center.y) index |= 2;
        if (point.z() >= center.z) index |= 4;

        return index;
    }

    __global__ void InsertPoints(OctreeNode* octreeNodes, Eigen::Vector3f* points, int numOfPoints, float3 rootMin, float3 rootMax, int* nextFreeNodeIdx) {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (idx >= numOfPoints) return;

        Eigen::Vector3f point = points[idx];
        int currentNodeIdx = 0;  // Start at root

        float3 minBoundary = rootMin;
        float3 maxBoundary = rootMax;

        for (int level = 0; level < MAX_LEVELS; ++level) {
            int childIdx = ComputeChildIndex(point, currentNodeIdx, minBoundary, maxBoundary);

            if (octreeNodes[currentNodeIdx].children[childIdx] == -1) {
                // Allocate new node (atomic operation to avoid race conditions)
                int newChildIdx = atomicAdd(nextFreeNodeIdx, 1);
                if (newChildIdx >= numOfPoints * MAX_LEVELS) {
                    printf("Octree is full. Cannot allocate new node.\n");
                        return;
                }

                octreeNodes[newChildIdx].isLeafNode = true;
                octreeNodes[newChildIdx].pointIdx = idx;
                for (int i = 0; i < 8; ++i) {
                    octreeNodes[newChildIdx].children[i] = -1;
                }

                octreeNodes[currentNodeIdx].children[childIdx] = newChildIdx;
            }

            // Move to the child node
            currentNodeIdx = octreeNodes[currentNodeIdx].children[childIdx];

            // Update bounding box
            float3 center = make_float3((minBoundary.x + maxBoundary.x) * 0.5f,
                (minBoundary.y + maxBoundary.y) * 0.5f,
                (minBoundary.z + maxBoundary.z) * 0.5f);

            if (childIdx & 1) minBoundary.x = center.x; else maxBoundary.x = center.x;
            if (childIdx & 2) minBoundary.y = center.y; else maxBoundary.y = center.y;
            if (childIdx & 4) minBoundary.z = center.z; else maxBoundary.z = center.z;
        }
    }

	void Test(const std::vector<Eigen::Vector3f>& inputPoints)
	{
        // Allocate GPU memory
        Eigen::Vector3f* d_points;
        cudaMallocManaged(&d_points, inputPoints.size() * sizeof(Eigen::Vector3f));

        OctreeNode* d_octreeNodes;
        cudaMallocManaged(&d_octreeNodes, inputPoints.size() * MAX_LEVELS * sizeof(OctreeNode));

        // Initialize octree root
        OctreeNode root;
        root.isLeafNode = false;
        root.pointIdx = -1;
        for (int i = 0; i < 8; ++i) {
            root.children[i] = -1;
        }
        d_octreeNodes[0] = root;

        int* d_nextFreeNodeIndex;
        int h_nextFreeNodeIndex = 1;
        cudaMallocManaged(&d_nextFreeNodeIndex, sizeof(int));
        *d_nextFreeNodeIndex = h_nextFreeNodeIndex;

        // Define root bounds
        float3 rootMin = make_float3(-1.0f, -1.0f, -1.0f);
        float3 rootMax = make_float3(1.0f, 1.0f, 1.0f);

        nvtxRangePushA("Octree");

        // Launch CUDA kernel to insert points into the octree
        int blockSize = 256;
        int numBlocks = (inputPoints.size() + blockSize - 1) / blockSize;
        InsertPoints << <numBlocks, blockSize >> > (d_octreeNodes, d_points, inputPoints.size(), rootMin, rootMax, d_nextFreeNodeIndex);
        cudaDeviceSynchronize();

        nvtxRangePop();

        // Cleanup
        cudaFree(d_points);
        cudaFree(d_octreeNodes);
        cudaFree(d_nextFreeNodeIndex);
	}
}