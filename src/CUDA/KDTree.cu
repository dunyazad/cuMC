#include "KDTree.cuh"

#define WINDOW_SIZE 3

namespace Algorithm
{
	KDTreeNode::KDTreeNode()
	{
	}

	KDTreeNode::~KDTreeNode()
	{
	}

	KDTree::KDTree()
	{
	}

	KDTree::~KDTree()
	{
	}
}

namespace CUDA
{
    void Test()
    {
        printf("Test\n");
    }

    // Comparator for Eigen::Vector3f based on the specified axis (0 = x, 1 = y, 2 = z)
    __device__ void Comparator(float* arr, int i, int j, bool direction, int axis) {
        // Compare based on the specified axis
        if (direction == (arr[i * 3 + axis] > arr[j * 3 + axis])) {
            // Swap all three components of the vector (x, y, z)
            for (int k = 0; k < 3; k++) {
                float temp = arr[i * 3 + k];
                arr[i * 3 + k] = arr[j * 3 + k];
                arr[j * 3 + k] = temp;
            }
        }
    }

    // CUDA kernel for the bitonic merge step
    __global__ void bitonicMerge(float* arr, int n, int size, int direction, int axis) {
        unsigned int idx = threadIdx.x + blockIdx.x * blockDim.x;
        unsigned int halfSize = size / 2;

        if (idx < halfSize) {
            int i = idx;
            int j = i + halfSize;

            Comparator(arr, i, j, direction, axis);
        }
    }

    // CUDA kernel for the bitonic sort step
    __global__ void bitonicSort(float* arr, int n, int size, bool direction, int axis) {
        unsigned int idx = threadIdx.x + blockIdx.x * blockDim.x;

        for (int k = 2; k <= size; k <<= 1) {
            for (int j = k / 2; j > 0; j >>= 1) {
                int ixj = idx ^ j;
                if (ixj > idx) {
                    Comparator(arr, idx, ixj, direction, axis);
                }
                __syncthreads();
            }
        }
    }

    void BitonicSortGPU(Eigen::Vector3f* arr, int n, int axis) {
        int numFloats = n * 3; // Each Eigen::Vector3f has 3 floats (x, y, z)
        size_t size = numFloats * sizeof(float);

        // Allocate an array of floats for copying the Vector3f array to the GPU
        float* floatArr = new float[numFloats];

        // Copy Vector3f array into a float array
        for (int i = 0; i < n; i++) {
            floatArr[i * 3 + 0] = arr[i].x(); // x-component
            floatArr[i * 3 + 1] = arr[i].y(); // y-component
            floatArr[i * 3 + 2] = arr[i].z(); // z-component
        }

        // Allocate memory on the device (GPU)
        float* d_arr;
        cudaMalloc((void**)&d_arr, size);

        // Copy the float array from host (CPU) to device (GPU)
        cudaMemcpy(d_arr, floatArr, size, cudaMemcpyHostToDevice);

        // Kernel execution configuration
        dim3 blockDim(512);            // Number of threads per block
        dim3 gridDim((n + 511) / 512); // Number of blocks

        /*chrono::steady_clock::time_point start, end;
        chrono::nanoseconds result;
        start = chrono::steady_clock::now();*/
        nvtxRangePushA("BitonicSort");

        // Perform bitonic sort on GPU
        for (int size = 2; size <= n; size <<= 1) {
            for (int direction = size / 2; direction > 0; direction >>= 1) {
                bitonicMerge << <gridDim, blockDim >> > (d_arr, n, size, direction, axis);
            }
        }

        // Synchronize the device
        cudaDeviceSynchronize();

        // Copy the sorted float array from device back to host
        cudaMemcpy(floatArr, d_arr, size, cudaMemcpyDeviceToHost);

        // Copy the sorted float array back into the original Vector3f array
        for (int i = 0; i < n; i++) {
            arr[i].x() = floatArr[i * 3 + 0]; // x-component
            arr[i].y() = floatArr[i * 3 + 1]; // y-component
            arr[i].z() = floatArr[i * 3 + 2]; // z-component
        }

        // Free GPU memory
        cudaFree(d_arr);
        delete[] floatArr;

        /*end = chrono::steady_clock::now();
        result = end - start;
        cout << "CUDA Bitonic Sort (axis " << axis << "): " << result.count() << "ns" << endl;*/

        nvtxRangePop();
    }

    //bool isPowerOfTwo(int n) {
    //    return (n && !(n & (n - 1)));
    //}

    //int nextPowerOfTwo(int n) {
    //    return pow(2, ceil(log2(n)));
    //}

    //__device__ void swap(float& a, float& b) {
    //    float tmp = a;
    //    a = b;
    //    b = tmp;
    //}

    //__global__ void bitonicSort(float* data, int numberOfPoints, int step, int stage) {
    //    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    //    if (idx >= numberOfPoints)
    //        return;

    //    int ixj = idx ^ step;

    //    if (ixj > idx) {
    //        if ((idx & stage) == 0) {
    //            if (data[idx] > data[ixj]) {
    //                swap(data[idx], data[ixj]);
    //            }
    //        }
    //        else {
    //            if (data[idx] < data[ixj]) {
    //                swap(data[idx], data[ixj]);
    //            }
    //        }
    //    }
    //}

    //std::vector<Eigen::Vector3f> BitonicSort(Eigen::Vector3f* points, int numberOfPoints)
    //{
    //    int nextPOT = nextPowerOfTwo(numberOfPoints);

    //    float* d_data;
    //    cudaMalloc(&d_data, sizeof(Eigen::Vector3f) * nextPOT);
    //    cudaMemset(&d_data, FLT_MAX, sizeof(Eigen::Vector3f) * nextPOT);
    //    cudaMemcpy(d_data, points, sizeof(Eigen::Vector3f) * numberOfPoints, cudaMemcpyHostToDevice);

    //    dim3 blocks((nextPOT + 255) / 256);
    //    dim3 threads(256);

    //    nvtxRangePushA("Bitonic Sort");

    //    for (int stage = 2; stage <= nextPOT; stage <<= 1) {
    //        for (int step = stage >> 1; step > 0; step >>= 1) {
    //            bitonicSort << <blocks, threads >> > (d_data, nextPOT, step, stage);
    //            cudaDeviceSynchronize();

    //            //printf("step : %d\n", step);
    //        }
    //    }

    //    nvtxRangePop();

    //    std::vector<Eigen::Vector3f> result(nextPOT);
    //    cudaMemcpy(result.data(), d_data, sizeof(Eigen::Vector3f) * nextPOT, cudaMemcpyDeviceToHost);

    //    cudaFree(d_data);

    //    return result;
    //}

    //__device__ void bubbleSort(float* window, int windowSize) {
    //    for (int i = 0; i < windowSize - 1; i++) {
    //        for (int j = 0; j < windowSize - i - 1; j++) {
    //            if (window[j] > window[j + 1]) {
    //                float temp = window[j];
    //                window[j] = window[j + 1];
    //                window[j + 1] = temp;
    //            }
    //        }
    //    }
    //}

    //// CUDA kernel to perform median filtering on an Eigen::Vector3f array
    //__global__ void medianFilter3D(Eigen::Vector3f* input, Eigen::Vector3f* output, int width, int height) {
    //    // Compute the x, y index of the current thread
    //    int x = blockIdx.x * blockDim.x + threadIdx.x;
    //    int y = blockIdx.y * blockDim.y + threadIdx.y;

    //    // Ensure the thread is within the bounds of the array
    //    if (x >= width || y >= height) {
    //        return;
    //    }

    //    // Define the window size for median filtering
    //    const int halfWindowSize = WINDOW_SIZE / 2;

    //    // Initialize arrays to hold the window values for x, y, z components
    //    float windowX[WINDOW_SIZE * WINDOW_SIZE];
    //    float windowY[WINDOW_SIZE * WINDOW_SIZE];
    //    float windowZ[WINDOW_SIZE * WINDOW_SIZE];

    //    int windowIndex = 0;

    //    // Loop through the window around the current pixel
    //    for (int dy = -halfWindowSize; dy <= halfWindowSize; ++dy) {
    //        for (int dx = -halfWindowSize; dx <= halfWindowSize; ++dx) {
    //            int nx = min(max(x + dx, 0), width - 1); // Clamp to array bounds
    //            int ny = min(max(y + dy, 0), height - 1); // Clamp to array bounds

    //            // Load the window values for each component (x, y, z)
    //            Eigen::Vector3f neighbor = input[ny * width + nx];

    //            if (FLT_MAX == neighbor.x() || FLT_MAX == neighbor.y() || FLT_MAX == neighbor.z())
    //                continue;

    //            windowX[windowIndex] = neighbor.x();
    //            windowY[windowIndex] = neighbor.y();
    //            windowZ[windowIndex] = neighbor.z();
    //            windowIndex++;
    //        }
    //    }

    //    // Sort each window to find the median
    //    bubbleSort(windowX, WINDOW_SIZE * WINDOW_SIZE);
    //    bubbleSort(windowY, WINDOW_SIZE * WINDOW_SIZE);
    //    bubbleSort(windowZ, WINDOW_SIZE * WINDOW_SIZE);

    //    // Store the median values in the output array
    //    output[y * width + x] = Eigen::Vector3f(windowX[WINDOW_SIZE * WINDOW_SIZE / 2],
    //        windowY[WINDOW_SIZE * WINDOW_SIZE / 2],
    //        windowZ[WINDOW_SIZE * WINDOW_SIZE / 2]);
    //}

    //std::vector<Eigen::Vector3f> CUDA::DoFilter(Eigen::Vector3f* points)
    //{
    //    std::vector<Eigen::Vector3f> result;

    //    int width = 256;
    //    int height = 480;

    //    // Create an Eigen array to hold the input data
    //    Eigen::Matrix<Eigen::Vector3f, Eigen::Dynamic, Eigen::Dynamic> input(height, width);
    //    Eigen::Matrix<Eigen::Vector3f, Eigen::Dynamic, Eigen::Dynamic> output(height, width);

    //    // Initialize the input array with some data (this should be replaced with actual data)
    //    for (int y = 0; y < height; ++y) {
    //        for (int x = 0; x < width; ++x) {
    //            ///input(y, x) = Eigen::Vector3f(x, y, x + y); // Example values
    //            input(y, x) = points[y * 256 + x];
    //        }
    //    }

    //    // Allocate device memory for input and output arrays
    //    Eigen::Vector3f* d_input;
    //    Eigen::Vector3f* d_output;
    //    cudaMalloc(&d_input, width * height * sizeof(Eigen::Vector3f));
    //    cudaMalloc(&d_output, width * height * sizeof(Eigen::Vector3f));

    //    // Copy input data to device
    //    cudaMemcpy(d_input, input.data(), width * height * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice);

    //    // Define block and grid sizes
    //    dim3 blockDim(16, 16);
    //    dim3 gridDim((width + blockDim.x - 1) / blockDim.x, (height + blockDim.y - 1) / blockDim.y);

    //    // Launch the kernel
    //    medianFilter3D << <gridDim, blockDim >> > (d_input, d_output, width, height);

    //    // Copy the output data back to the host
    //    cudaMemcpy(output.data(), d_output, width * height * sizeof(Eigen::Vector3f), cudaMemcpyDeviceToHost);

    //    // Free device memory
    //    cudaFree(d_input);
    //    cudaFree(d_output);

    //    // Print the result for testing (this can be replaced with further processing)
    //    for (int y = 0; y < height; ++y) {
    //        for (int x = 0; x < width; ++x) {
    //            Eigen::Vector3f p = output(y, x).transpose();

    //            if (p.x() == FLT_MAX || p.y() == FLT_MAX || p.z() == FLT_MAX)
    //                continue;

    //            result.push_back(p);
    //            //std::cout << p << " ";
    //        }
    //        //std::cout << std::endl;
    //    }

    //    return result;
    //}

//#define CUDA_CALL(x) do { if((x) != cudaSuccess) { \
//    printf("Error at %s:%d\n",__FILE__,__LINE__); return EXIT_FAILURE;}} while(0)

#define CUDA_CALL(x)

    __global__ void mergeSortKernel(const Eigen::Vector3f* data, int* indices, int size, int totalSize) {
        int tid = threadIdx.x + blockDim.x * blockIdx.x;
        int left = 2 * tid * size;
        int mid = left + size;
        int right = left + 2 * size;

        if (left >= totalSize) {
            return;
        }

        if (mid > totalSize) {
            mid = totalSize;
        }

        if (right > totalSize) {
            right = totalSize;
        }

        int i = left;
        int j = mid;
        int k = 0;

        extern __shared__ int temp[];

        while (i < mid && j < right) {
            if (data[indices[i]].x() <= data[indices[j]].x()) {
                temp[k++] = indices[i++];
            }
            else {
                temp[k++] = indices[j++];
            }
        }

        while (i < mid) {
            temp[k++] = indices[i++];
        }

        while (j < right) {
            temp[k++] = indices[j++];
        }

        for (int l = 0; l < (right - left); ++l) {
            indices[left + l] = temp[l];
        }
    }

    void cudaMergeSort(Eigen::Vector3f* data, int* indices, int size) {
        int* d_indices = nullptr;
        Eigen::Vector3f* d_data = nullptr;

        CUDA_CALL(cudaMalloc((void**)&d_data, size * sizeof(Eigen::Vector3f)));
        CUDA_CALL(cudaMemcpy(d_data, data, size * sizeof(Eigen::Vector3f), cudaMemcpyHostToDevice));

        CUDA_CALL(cudaMalloc((void**)&d_indices, size * sizeof(int)));
        CUDA_CALL(cudaMemcpy(d_indices, indices, size * sizeof(int), cudaMemcpyHostToDevice));

        int blockSize = 256;
        int gridSize = (size + blockSize - 1) / blockSize;

        nvtxRangePushA("Merge Sort");

        for (int width = 1; width < size; width *= 2) {
            mergeSortKernel << <gridSize, blockSize, blockSize * sizeof(int) >> > (d_data, d_indices, width, size);
            CUDA_CALL(cudaDeviceSynchronize());
        }

        nvtxRangePop();

        CUDA_CALL(cudaMemcpy(indices, d_indices, size * sizeof(int), cudaMemcpyDeviceToHost));

        CUDA_CALL(cudaFree(d_indices));
        CUDA_CALL(cudaFree(d_data));
    }

}