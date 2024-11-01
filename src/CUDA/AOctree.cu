#include "AOctree.cuh"

#define MAX_LEVELS 13

namespace AOctree
{
 //   __device__ int ComputeChildIndex(Eigen::Vector3f point, int currentNodeIdx, float3 rootMin, float3 rootMax) {
 //       // Compute child index based on which sub-cube contains the point
 //       // Simplified logic for calculating voxel index within the octree region
 //       float3 center = make_float3((rootMin.x + rootMax.x) * 0.5f,
 //           (rootMin.y + rootMax.y) * 0.5f,
 //           (rootMin.z + rootMax.z) * 0.5f);

 //       int index = 0;
 //       if (point.x() >= center.x) index |= 1;
 //       if (point.y() >= center.y) index |= 2;
 //       if (point.z() >= center.z) index |= 4;

 //       return index;
 //   }

 //   __global__ void InsertPoints(AOctreeNode* octreeNodes, Eigen::Vector3f* points, int numOfPoints, float3 rootMin, float3 rootMax, int* nextFreeNodeIdx) {
 //       int idx = blockIdx.x * blockDim.x + threadIdx.x;
 //       if (idx >= numOfPoints) return;

 //       Eigen::Vector3f point = points[idx];
 //       int currentNodeIdx = 0;  // Start at root

 //       float3 minBoundary = rootMin;
 //       float3 maxBoundary = rootMax;

 //       for (int level = 0; level < MAX_LEVELS; ++level) {
 //           int childIdx = ComputeChildIndex(point, currentNodeIdx, minBoundary, maxBoundary);

 //           if (octreeNodes[currentNodeIdx].children[childIdx] == -1) {
 //               // Allocate new node (atomic operation to avoid race conditions)
 //               int newChildIdx = atomicAdd(nextFreeNodeIdx, 1);
 //               if (newChildIdx >= numOfPoints * MAX_LEVELS) {
 //                   printf("Octree is full. Cannot allocate new node.\n");
 //                       return;
 //               }

 //               octreeNodes[newChildIdx].isLeafNode = true;
 //               octreeNodes[newChildIdx].pointIdx = idx;
 //               for (int i = 0; i < 8; ++i) {
 //                   octreeNodes[newChildIdx].children[i] = -1;
 //               }

 //               octreeNodes[currentNodeIdx].children[childIdx] = newChildIdx;
 //           }

 //           // Move to the child node
 //           currentNodeIdx = octreeNodes[currentNodeIdx].children[childIdx];

 //           // Update bounding box
 //           float3 center = make_float3((minBoundary.x + maxBoundary.x) * 0.5f,
 //               (minBoundary.y + maxBoundary.y) * 0.5f,
 //               (minBoundary.z + maxBoundary.z) * 0.5f);

 //           if (childIdx & 1) minBoundary.x = center.x; else maxBoundary.x = center.x;
 //           if (childIdx & 2) minBoundary.y = center.y; else maxBoundary.y = center.y;
 //           if (childIdx & 4) minBoundary.z = center.z; else maxBoundary.z = center.z;
 //       }
 //   }

	//void TestA(const std::vector<Eigen::Vector3f>& inputPoints)
	//{
 //       // Allocate GPU memory
 //       Eigen::Vector3f* d_points;
 //       cudaMallocManaged(&d_points, inputPoints.size() * sizeof(Eigen::Vector3f));

 //       AOctreeNode* d_AOctreeNodes;
 //       cudaMallocManaged(&d_AOctreeNodes, inputPoints.size() * MAX_LEVELS * sizeof(AOctreeNode));

 //       // Initialize octree root
 //       AOctreeNode root;
 //       root.isLeafNode = false;
 //       root.pointIdx = -1;
 //       for (int i = 0; i < 8; ++i) {
 //           root.children[i] = -1;
 //       }
 //       d_AOctreeNodes[0] = root;

 //       int* d_nextFreeNodeIndex;
 //       int h_nextFreeNodeIndex = 1;
 //       cudaMallocManaged(&d_nextFreeNodeIndex, sizeof(int));
 //       *d_nextFreeNodeIndex = h_nextFreeNodeIndex;

 //       // Define root bounds
 //       float3 rootMin = make_float3(-1.0f, -1.0f, -1.0f);
 //       float3 rootMax = make_float3(1.0f, 1.0f, 1.0f);

 //       nvtxRangePushA("AOctree");

 //       // Launch CUDA kernel to insert points into the octree
 //       int blockSize = 256;
 //       int numBlocks = (inputPoints.size() + blockSize - 1) / blockSize;
 //       InsertPoints << <numBlocks, blockSize >> > (d_AOctreeNodes, d_points, inputPoints.size(), rootMin, rootMax, d_nextFreeNodeIndex);
 //       cudaDeviceSynchronize();

 //       nvtxRangePop();

 //       // Cleanup
 //       cudaFree(d_points);
 //       cudaFree(d_AOctreeNodes);
 //       cudaFree(d_nextFreeNodeIndex);
	//}

    struct AOctreeInfo
    {
        Eigen::Vector3f min = Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX);
        Eigen::Vector3f max = Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX);
        int maxDepth = 13;
        size_t nodeCount = 1;
    };

    __device__ int GetChildIndex(
        const Eigen::Vector3f& parentMin,
        const Eigen::Vector3f& parentMax,
        const Eigen::Vector3f& position,
        Eigen::Vector3f& childMin,
        Eigen::Vector3f& childMax)
    {
        float cx = (parentMin.x() + parentMax.x()) * 0.5f;
        float cy = (parentMin.y() + parentMax.y()) * 0.5f;
        float cz = (parentMin.z() + parentMax.z()) * 0.5f;

        int index = 0;
        childMin = parentMin;
        childMax = parentMax;

        if (position.x() >= cx) { index |= 1; childMin.x() = cx; }
        else { childMax.x() = cx; }

        if (position.y() >= cy) { index |= 2; childMin.y() = cy; }
        else { childMax.y() = cy; }

        if (position.z() >= cz) { index |= 4; childMin.z() = cz; }
        else { childMax.z() = cz; }

        return index;
    }

    __device__ size_t NewNode(AOctreeInfo& info) {
        return atomicAdd(&info.nodeCount, 1);
    }

    __device__ inline void LockNode(AOctreeNode* node) {
       // printf("Attempting to Lock\n");
        while (atomicCAS(&(node->lock), 0, 1) != 0) {}
        printf("Lock Acquired\n");
    }

    __device__ inline void UnlockNode(AOctreeNode* node) {
        atomicExch(&(node->lock), 0);
    }

    __global__ void Kernel_BuildOctree(
        AOctreeInfo info,
        AOctreeNode* nodes,
        size_t rootIndex,
        Eigen::Vector3f* points,
        size_t numberOfPoints)
    {
        size_t threadid = blockIdx.x * blockDim.x + threadIdx.x;
        if (threadid >= numberOfPoints) return;

        Eigen::Vector3f point = points[threadid];
        int currentDepth = 0;
        Eigen::Vector3f currentMin = info.min;
        Eigen::Vector3f currentMax = info.max;
        size_t currentNodeIndex = rootIndex;

        while (true) {
            AOctreeNode* currentNode = &nodes[currentNodeIndex];
            Eigen::Vector3f childMin, childMax;

            int childIndex = GetChildIndex(currentMin, currentMax, point, childMin, childMax);

            if (currentDepth == info.maxDepth) {
                // Atomically assign the point to the leaf node
                atomicCAS(&(currentNode->pointIdx), UINT64_MAX, threadid);  // Only assign if pointIdx is UINT64_MAX
                break;
            }

            // Atomically assign a new child node if unassigned
            if (atomicCAS(&(currentNode->children[childIndex]), UINT64_MAX, NewNode(info)) == UINT64_MAX) {
                // New node assigned by this thread
                printf("Thread %lu: New child created at depth %d\n", threadid, currentDepth);
            }

            // Traverse to the assigned or newly created child node
            currentNodeIndex = currentNode->children[childIndex];
            currentMin = childMin;
            currentMax = childMax;
            currentDepth++;
        }
        printf("Thread %lu: Finished at depth %d\n", threadid, currentDepth);  // Confirm completion
    }

    void TestB(const std::vector<Eigen::Vector3f>& inputPoints)
    {
        Eigen::Vector3f* d_points;
        cudaMallocManaged(&d_points, inputPoints.size() * sizeof(Eigen::Vector3f));
        cudaMemcpy(d_points, inputPoints.data(), inputPoints.size() * sizeof(Eigen::Vector3f), cudaMemcpyDeviceToHost);

        AOctreeNode* d_AOctreeNodes;
        cudaMallocManaged(&d_AOctreeNodes, inputPoints.size() * MAX_LEVELS * sizeof(AOctreeNode));
        cudaMemset(d_AOctreeNodes, 0, inputPoints.size() * MAX_LEVELS * sizeof(AOctreeNode));

        AOctreeInfo info;
        info.min = Eigen::Vector3f(-25.0f, -25.0f, -25.0f);
        info.max = Eigen::Vector3f(25.0f, 25.0f, 25.0f);
        info.maxDepth = 13;

        nvtxRangePushA("Octree");

        int mingridsize;
        int threadblocksize;
        cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, Kernel_BuildOctree, 0, 0);

        int gridsize = (inputPoints.size() + threadblocksize - 1) / threadblocksize;
        Kernel_BuildOctree<<<gridsize, threadblocksize>>>(info, d_AOctreeNodes, 0, d_points, inputPoints.size());

        cudaError_t err = cudaDeviceSynchronize();  // Synchronize to catch any runtime errors
        if (err != cudaSuccess) {
            printf("CUDA Error: %s\n", cudaGetErrorString(err));
        }

        nvtxRangePop();
    }

    struct Triangle {
        int a_idx, b_idx, c_idx;  // Indices in the points array

        // Default constructor
        __host__ __device__ Triangle() : a_idx(-1), b_idx(-1), c_idx(-1) {}

        // Parameterized constructor
        __host__ __device__ Triangle(int a, int b, int c) : a_idx(a), b_idx(b), c_idx(c) {}

        // Method to check if a point is inside the circumcircle
        __device__ bool containsPointInCircumcircle(const float* points, int p_idx) {
            float ax = points[a_idx * 3], ay = points[a_idx * 3 + 1], az = points[a_idx * 3 + 2];
            float bx = points[b_idx * 3], by = points[b_idx * 3 + 1], bz = points[b_idx * 3 + 2];
            float cx = points[c_idx * 3], cy = points[c_idx * 3 + 1], cz = points[c_idx * 3 + 2];

            float px = points[p_idx * 3], py = points[p_idx * 3 + 1], pz = points[p_idx * 3 + 2];

            float axp = ax - px, ayp = ay - py, azp = az - pz;
            float bxp = bx - px, byp = by - py, bzp = bz - pz;
            float cxp = cx - px, cyp = cy - py, czp = cz - pz;

            float det = axp * (byp * czp - bzp * cyp) - ayp * (bxp * czp - bzp * cxp) + azp * (bxp * cyp - byp * cxp);
            return det > 0;
        }
    };

    // Function to convert std::vector<Eigen::Vector3f> to a flat float array and transfer it to device
    void convertAndTransferPoints(const std::vector<Eigen::Vector3f>& inputPoints, float*& d_points) {
        int num_points = inputPoints.size();
        float* h_points = new float[num_points * 3];

        for (int i = 0; i < num_points; ++i) {
            h_points[i * 3] = inputPoints[i].x();
            h_points[i * 3 + 1] = inputPoints[i].y();
            h_points[i * 3 + 2] = inputPoints[i].z();
        }

        cudaMalloc((void**)&d_points, num_points * 3 * sizeof(float));
        cudaMemcpy(d_points, h_points, num_points * 3 * sizeof(float), cudaMemcpyHostToDevice);
        delete[] h_points;
    }

    // Kernel to initialize a super-triangle (a very large encompassing triangle)
    __global__ void initializeSuperTriangle(Triangle* triangles, int num_points) {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (idx == 0) {
            // Using large indices to represent a super-triangle that encompasses all points
            triangles[0] = Triangle(num_points, num_points + 1, num_points + 2);
        }
    }

    // Kernel to perform Delaunay insertion
    __global__ void delaunayInsert(Triangle* triangles, float* points, int num_points, int* triangle_count) {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (idx >= num_points) return;

        bool isTriangleInvalid[1024];  // Temporary array to mark invalid triangles
        for (int i = 0; i < *triangle_count; ++i) {
            isTriangleInvalid[i] = false;
        }

        // Iterate over each triangle and check if the new point is in its circumcircle
        for (int i = 0; i < *triangle_count; ++i) {
            if (triangles[i].containsPointInCircumcircle(points, idx)) {
                isTriangleInvalid[i] = true;
            }
        }

        __syncthreads();  // Ensure all threads have completed circumcircle checks

        // Add new triangles for each invalid triangle
        for (int i = 0; i < *triangle_count; ++i) {
            if (isTriangleInvalid[i]) {
                int a = triangles[i].a_idx;
                int b = triangles[i].b_idx;
                int c = triangles[i].c_idx;

                int new_idx = atomicAdd(triangle_count, 3);

                triangles[new_idx] = Triangle(a, b, idx);    // Triangle with vertices a, b, and new point
                triangles[new_idx + 1] = Triangle(b, c, idx);  // Triangle with vertices b, c, and new point
                triangles[new_idx + 2] = Triangle(c, a, idx);  // Triangle with vertices c, a, and new point

                isTriangleInvalid[i] = false;
            }
        }
    }

    void TestC(const std::vector<Eigen::Vector3f>& inputPoints)
    {
        TestB(inputPoints);
        return;

        float* d_points;
        Triangle* d_triangles;
        int* d_triangle_count;
        int num_points = inputPoints.size();
        int max_triangles = num_points * 6;  // Rough estimate of maximum triangles (to ensure enough memory)

        // Convert and transfer input points to device
        convertAndTransferPoints(inputPoints, d_points);

        // Allocate device memory for triangles and triangle count
        cudaMalloc((void**)&d_triangles, max_triangles * sizeof(Triangle));
        cudaMalloc((void**)&d_triangle_count, sizeof(int));
        cudaMemset(d_triangle_count, 1, sizeof(int));  // Initialize triangle count to 1 (the super triangle)

        // Initialize the super triangle
        initializeSuperTriangle << <1, 1 >> > (d_triangles, num_points);

        // Launch kernel to perform Delaunay insertion
        int threadsPerBlock = 256;
        int blocks = (num_points + threadsPerBlock - 1) / threadsPerBlock;
        delaunayInsert << <blocks, threadsPerBlock >> > (d_triangles, d_points, num_points, d_triangle_count);

        // Copy the resulting triangles back to the host
        Triangle* h_triangles = new Triangle[max_triangles];
        cudaMemcpy(h_triangles, d_triangles, max_triangles * sizeof(Triangle), cudaMemcpyDeviceToHost);

        // Print the resulting triangles
        int triangle_count;
        cudaMemcpy(&triangle_count, d_triangle_count, sizeof(int), cudaMemcpyDeviceToHost);
        std::cout << "Number of triangles: " << triangle_count << "\n";
        for (int i = 0; i < triangle_count; ++i) {
            std::cout << "Triangle " << i << ": " << h_triangles[i].a_idx << ", "
                << h_triangles[i].b_idx << ", " << h_triangles[i].c_idx << "\n";
        }

        // Free device memory
        cudaFree(d_points);
        cudaFree(d_triangles);
        cudaFree(d_triangle_count);
        delete[] h_triangles;
    }

    struct OctreeNode {
        float min_bound[3]; // Node 공간의 최소 좌표
        float max_bound[3]; // Node 공간의 최대 좌표
        int depth;          // 트리의 깊이
        int children[8];    // 자식 노드의 인덱스 (없는 경우 -1)

        __device__ __host__ OctreeNode() {
            for (int i = 0; i < 8; ++i) children[i] = -1;
            printf("-1\n");
        }
    };

    __global__ void InitializeNodes(OctreeNode* nodes, int num_points)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (idx >= num_points) return;

        nodes[idx].min_bound[0] = FLT_MAX;
        nodes[idx].min_bound[1] = FLT_MAX;
        nodes[idx].min_bound[2] = FLT_MAX;

        nodes[idx].max_bound[0] = -FLT_MAX;
        nodes[idx].max_bound[1] = -FLT_MAX;
        nodes[idx].max_bound[2] = -FLT_MAX;

        nodes[idx].depth = 0;
        nodes[idx].children[0] = -1;
        nodes[idx].children[1] = -1;
        nodes[idx].children[2] = -1;
        nodes[idx].children[3] = -1;
        nodes[idx].children[4] = -1;
        nodes[idx].children[5] = -1;
        nodes[idx].children[6] = -1;
        nodes[idx].children[7] = -1;
    }

    __global__ void buildOctreeKernel(OctreeNode* nodes, float* points, int num_points, int max_depth, int* num_nodes) {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (idx >= num_points) return;

        float* point = &points[idx * 3]; // 포인트의 x, y, z 접근

        OctreeNode* current = &nodes[0]; // 루트 노드에서 시작
        for (int d = 0; d < max_depth; ++d) {

            float mid[3] = { (current->min_bound[0] + current->max_bound[0]) / 2.0f,
                            (current->min_bound[1] + current->max_bound[1]) / 2.0f,
                            (current->min_bound[2] + current->max_bound[2]) / 2.0f };

            int child_index = (point[0] > mid[0]) * 1 + (point[1] > mid[1]) * 2 + (point[2] > mid[2]) * 4;

            if (current->children[child_index] == -1) {
                int new_index = atomicAdd(num_nodes, 1); // num_nodes는 GPU에서 안전하게 증가
                current->children[child_index] = new_index;

                nodes[new_index].min_bound[0] = (child_index & 1) ? mid[0] : current->min_bound[0];
                nodes[new_index].min_bound[1] = (child_index & 2) ? mid[1] : current->min_bound[1];
                nodes[new_index].min_bound[2] = (child_index & 4) ? mid[2] : current->min_bound[2];

                nodes[new_index].max_bound[0] = (child_index & 1) ? current->max_bound[0] : mid[0];
                nodes[new_index].max_bound[1] = (child_index & 2) ? current->max_bound[1] : mid[1];
                nodes[new_index].max_bound[2] = (child_index & 4) ? current->max_bound[2] : mid[2];
                nodes[new_index].depth = current->depth + 1;
            }

            // 하위 노드로 이동
            current = &nodes[current->children[child_index]];
        }
    }

    void Test(const std::vector<Eigen::Vector3f>& inputPoints)
    {
        int num_points = inputPoints.size();
        int max_depth = 13; // 최대 트리 깊이 설정
        int max_nodes = inputPoints.size();

        // GPU 메모리에 포인트 데이터 복사
        float* d_points;
        cudaMalloc((void**)&d_points, inputPoints.size() * sizeof(float));
        cudaMemcpy(d_points, inputPoints.data(), inputPoints.size() * sizeof(float), cudaMemcpyHostToDevice);

        // Octree 노드 메모리 할당
        OctreeNode* d_nodes;
        cudaMalloc((void**)&d_nodes, max_nodes * sizeof(OctreeNode));

        // GPU에서 사용할 num_nodes 변수 초기화
        int initial_num_nodes = 1; // 루트 노드
        int* d_num_nodes;
        cudaMalloc((void**)&d_num_nodes, sizeof(int));
        cudaMemcpy(d_num_nodes, &initial_num_nodes, sizeof(int), cudaMemcpyHostToDevice);

        nvtxRangePushA("OCTREE");

        // CUDA 커널 실행
        int threadsPerBlock = 256;
        int blocksPerGrid = (num_points + threadsPerBlock - 1) / threadsPerBlock;
        InitializeNodes << <blocksPerGrid, threadsPerBlock >> > (d_nodes, num_points);
        buildOctreeKernel << <blocksPerGrid, threadsPerBlock >> > (d_nodes, d_points, num_points, max_depth, d_num_nodes);

        nvtxRangePop();

        // 결과 가져오기
        int num_nodes;
        cudaMemcpy(&num_nodes, d_num_nodes, sizeof(int), cudaMemcpyDeviceToHost);
        std::cout << "Total nodes created: " << num_nodes << std::endl;

        // 메모리 해제
        cudaFree(d_points);
        cudaFree(d_nodes);
        cudaFree(d_num_nodes);
    }
}
