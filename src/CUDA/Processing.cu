#include "Processing.cuh"

namespace Processing
{
	__global__ void Kernel_ClearGrid(float* grid, size_t gridWidth, size_t gridHeight)
	{
		size_t threadid = blockIdx.x * blockDim.x + threadIdx.x;
		if (threadid > gridWidth * gridHeight - 1) return;

		grid[threadid] = FLT_MAX;
	}

	__global__ void Kernel_ClearBuffer_Vector3f(Eigen::Vector3f* points, size_t numberOfPoints)
	{
		size_t threadid = blockIdx.x * blockDim.x + threadIdx.x;
		if (threadid > numberOfPoints - 1) return;

		points[threadid] = Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX);
	}

	__global__ void Kernel_ClearBuffer_UINT32(uint32_t* elements, size_t numberOfElements)
	{
		size_t threadid = blockIdx.x * blockDim.x + threadIdx.x;
		if (threadid > numberOfElements - 1) return;

		elements[threadid] = UINT32_MAX;
	}

	__global__ void Kernel_DownSample(
		Eigen::Vector3f* inputPoints,
		size_t numberOfInputPoints,
		Eigen::Vector3f inputPointsMin,
		Eigen::Vector3f inputPointsMax,
		float sampleSize,
		float* grid,
		int gridWidth,
		int gridHeight)
	{
		size_t threadid = blockIdx.x * blockDim.x + threadIdx.x;
		if (threadid > numberOfInputPoints - 1) return;

		auto& p = inputPoints[threadid];

		size_t xIndex = (size_t)floorf((p.x() - inputPointsMin.x()) / sampleSize);
		size_t yIndex = (size_t)floorf((p.y() - inputPointsMin.y()) / sampleSize);

		size_t gridIndex = yIndex * gridWidth + xIndex;
		if (FLT_MAX != grid[gridIndex])
		{
			auto distance = p.z() - grid[gridIndex];
			if (fabs(distance) > sampleSize * 10.0f)
			{
				grid[gridIndex] = max(grid[gridIndex], p.z());
			}
			else
			{
				grid[gridIndex] = (grid[gridIndex] + p.z()) * 0.5f;
			}
		}
		else
		{
			grid[gridIndex] = p.z();
		}
	}

	__device__ void bubbleSort(float* window, int size) {
		// A simple bubble sort to sort the elements in the window
		for (int i = 0; i < size - 1; i++) {
			for (int j = 0; j < size - i - 1; j++) {
				if (window[j] > window[j + 1]) {
					float temp = window[j];
					window[j] = window[j + 1];
					window[j + 1] = temp;
				}
			}
		}
	}

	__global__ void Kernel_MedianFilter(
		float* grid, size_t gridWidth, size_t gridHeight,
		float sampleSize, int indexOffset, Eigen::Vector3f gridMin)
	{
		int threadid = blockIdx.x * blockDim.x + threadIdx.x;
		if (threadid > gridWidth * gridHeight - 1) return;

		int xIndex = threadid % gridWidth;
		int yIndex = threadid / gridWidth;

		float sx = gridMin.x() + (float)xIndex * sampleSize;
		float sy = gridMin.y() + (float)yIndex * sampleSize;

		if (FLT_MAX != grid[threadid])
		{
			float neighborValues[100];

			float sz = grid[threadid];

			float z = 0;
			int count = 0;

			int offset = indexOffset;
			int currentOffset = 0;
			while (currentOffset <= offset)
			{
				for (int y = -currentOffset; y <= currentOffset; y++)
				{
					if (0 > yIndex + y) continue;

					for (int x = -currentOffset; x <= currentOffset; x++)
					{
						if (0 > xIndex + x) continue;

						if ((x == -currentOffset || x == currentOffset) ||
							(y == -currentOffset || y == currentOffset))
						{
							//printf("%d, %d\n", xIndex + x, yIndex + y);

							auto nix = xIndex + x;
							auto niy = yIndex + y;

							if (FLT_MAX != grid[niy * gridWidth + nix])
							{
								float nx = gridMin.x() + (float)(nix)*sampleSize;
								float ny = gridMin.y() + (float)(niy)*sampleSize;
								float nz = grid[niy * gridWidth + nix];

								neighborValues[count] = nz;

								z += nz;

								count++;
							}
						}
					}
				}
				currentOffset++;
			}

			if (0 < count)
			{
				//grid[threadid] = z / (float)count;

				bubbleSort(neighborValues, count);
				grid[threadid] = neighborValues[count / 2];
				//printf("count : %d\n", count);
			}
		}
	}

	__global__ void Kernel_GetPoints(float* grid, int gridWidth, int gridHeight,
		float sampleSize, Eigen::Vector3f gridMin, Eigen::Vector3f* resultPoints, size_t* numberOfResultPoints)
	{
		size_t threadid = blockIdx.x * blockDim.x + threadIdx.x;
		if (threadid > gridWidth * gridHeight - 1) return;

		int xIndex = threadid % gridWidth;
		int yIndex = threadid / gridWidth;

		float x = gridMin.x() + (float)xIndex * sampleSize;
		float y = gridMin.y() + (float)yIndex * sampleSize;

		if (FLT_MAX != grid[threadid])
		{
			auto index = atomicAdd(numberOfResultPoints, 1);
			resultPoints[index] = Eigen::Vector3f(x, y, grid[threadid]);
		}
	}

	__device__ const int marchingSquaresTable[16][4] = {
		{-1, -1, -1, -1}, {0, 3, -1, -1}, {0, 1, -1, -1}, {1, 3, -1, -1},
		{1, 2, -1, -1}, {0, 1, 2, 3}, {0, 2, -1, -1}, {2, 3, -1, -1},
		{2, 3, -1, -1}, {0, 2, -1, -1}, {0, 1, 2, 3}, {1, 2, -1, -1},
		{1, 3, -1, -1}, {0, 1, -1, -1}, {0, 3, -1, -1}, {-1, -1, -1, -1}
	};

	__device__ Eigen::Vector3f interpolate_OLD(
		const Eigen::Vector3f& p0, const Eigen::Vector3f& p1,
		float v0, float v1, float isoValue)
	{
		if (FLT_MAX == v0) return p1;
		if (FLT_MAX == v1) return p0;
		if (FLT_MAX == p0.z()) return p1;
		if (FLT_MAX == p1.z()) return p0;

		float t = (isoValue - v0) / (v1 - v0);
		if (0 > t) t = 0;
		if (1 < t) t = 1;
		return p0 + t * (p1 - p0);
	}

	__device__ Eigen::Vector3f interpolate(float ratio, const Eigen::Vector3f& p0, const Eigen::Vector3f& p1)
	{
		if (FLT_MAX == p0.z()) return p1;
		if (FLT_MAX == p1.z()) return p0;

		if (0 > ratio) ratio = 0;
		if (1 < ratio) ratio = 1;
		return p0 + ratio * (p1 - p0);
	}

	__global__ void Kernel_MarchingSquares_OLD(
		float* grid, int gridWidth, int gridHeight,
		float sampleSize, Eigen::Vector3f gridMin, float isoValue,
		Eigen::Vector3f* vertices, size_t* vertexCount,
		unsigned int* indices, size_t* indexCount,
		Eigen::Vector3f* resultPoints, size_t* numberOfResultPoints)
	{
		int threadid = blockIdx.x * blockDim.x + threadIdx.x;
		if (threadid > gridWidth * gridHeight - 1) return;

		int xIndex = threadid % gridWidth;
		int yIndex = threadid / gridWidth;

		// Get the four corners of the cell.
		int index0 = yIndex * gridWidth + xIndex;
		int index1 = index0 + 1;
		int index2 = (yIndex + 1) * gridWidth + xIndex + 1;
		int index3 = index2 - 1;

		// Corner points of the cell.
		float v0 = grid[index0];
		float v1 = grid[index1];
		float v2 = grid[index2];
		float v3 = grid[index3];

		if (FLT_MAX == v0) return;
		//if (FLT_MAX == v1) return;
		//if (FLT_MAX == v2) return;
		//if (FLT_MAX == v3) return;

		float x = gridMin.x() + (float)xIndex * sampleSize;
		float y = gridMin.y() + (float)yIndex * sampleSize;
		float z = v0;

		// Determine the cell type by setting bits based on the isoValue comparison.
		int cellType = 0;
		//if (v0 < isoValue) cellType |= 1;
		//if (v1 < isoValue) cellType |= 2;
		//if (v2 < isoValue) cellType |= 4;
		//if (v3 < isoValue) cellType |= 8;
		if (FLT_MAX != v0) cellType |= 1;
		if (FLT_MAX != v1) cellType |= 2;
		if (FLT_MAX != v2) cellType |= 4;
		if (FLT_MAX != v3) cellType |= 8;

		// Retrieve edges to interpolate from the lookup table.
		const int* edges = marchingSquaresTable[cellType];
		if (edges[0] == -1) return; // No triangles for this cell.

		printf("threadid : %d\n", threadid);

		// Compute the interpolated vertices.
		Eigen::Vector3f p[4];
		p[0] = { gridMin.x() + xIndex * sampleSize, gridMin.y() + yIndex * sampleSize, v0 };
		p[1] = { gridMin.x() + (xIndex + 1) * sampleSize, gridMin.y() + yIndex * sampleSize, v1 };
		p[2] = { gridMin.x() + (xIndex + 1) * sampleSize, gridMin.y() + (yIndex + 1) * sampleSize, v2 };
		p[3] = { gridMin.x() + xIndex * sampleSize, gridMin.y() + (yIndex + 1) * sampleSize, v3 };

		Eigen::Vector3f interpolatedPoints[4];
		int interpolatedVertexIndices[4];

		for (int i = 0; i < 4 && edges[i] != -1; ++i) {
			// Perform interpolation for each edge
			int e0 = edges[i];
			int e1 = (e0 + 1) % 4; // Assuming edge connectivity follows p[0,1,2,3]

			interpolatedPoints[i] = interpolate_OLD(p[e0], p[e1], v0, v1, isoValue);

			// Atomic add to get a unique index for this vertex.
			interpolatedVertexIndices[i] = atomicAdd(vertexCount, 1);
			vertices[interpolatedVertexIndices[i]] = interpolatedPoints[i];

			auto ri = atomicAdd(numberOfResultPoints, 1);
			resultPoints[ri] = interpolatedPoints[i];
			resultPoints[ri].z() = v0;
		}

		// Generate indices for triangles
		size_t localIndexCount = atomicAdd(indexCount, 6); // Maximum of 6 indices (2 triangles).
		if (edges[0] != -1 && edges[1] != -1 && edges[2] != -1) {
			indices[localIndexCount] = interpolatedVertexIndices[0];
			indices[localIndexCount + 1] = interpolatedVertexIndices[1];
			indices[localIndexCount + 2] = interpolatedVertexIndices[2];
		}
		if (edges[2] != -1 && edges[3] != -1) {
			indices[localIndexCount + 3] = interpolatedVertexIndices[2];
			indices[localIndexCount + 4] = interpolatedVertexIndices[3];
			indices[localIndexCount + 5] = interpolatedVertexIndices[0];
		}
	}

	__global__ void Kernel_MarchingSquares(
		float* grid, int gridWidth, int gridHeight,
		float sampleSize, Eigen::Vector3f gridMin, float isoValue,
		Eigen::Vector3f* vertices, size_t* vertexCount,
		unsigned int* indices, size_t* indexCount,
		Eigen::Vector3f* resultPoints, size_t* numberOfResultPoints)
	{
		int threadid = blockIdx.x * blockDim.x + threadIdx.x;
		if (threadid > gridWidth * gridHeight - 1) return;

		int xIndex = threadid % gridWidth;
		int yIndex = threadid / gridWidth;

		// Get the four corners of the cell.
		int index0 = yIndex * gridWidth + xIndex;
		int index1 = index0 + 1;
		int index2 = (yIndex + 1) * gridWidth + xIndex + 1;
		int index3 = index2 - 1;

		// Corner points of the cell.
		float v0 = grid[index0];
		float v1 = grid[index1];
		float v2 = grid[index2];
		float v3 = grid[index3];

		if (FLT_MAX == v0) return;
		//if (FLT_MAX == v1) return;
		//if (FLT_MAX == v2) return;
		//if (FLT_MAX == v3) return;

		float x = gridMin.x() + (float)xIndex * sampleSize;
		float y = gridMin.y() + (float)yIndex * sampleSize;
		float z = v0;

		// Determine the cell type by setting bits based on the isoValue comparison.
		int cellType = 0;
		//if (v0 < isoValue) cellType |= 1;
		//if (v1 < isoValue) cellType |= 2;
		//if (v2 < isoValue) cellType |= 4;
		//if (v3 < isoValue) cellType |= 8;
		if (FLT_MAX != v0) cellType |= 1;
		if (FLT_MAX != v1) cellType |= 2;
		if (FLT_MAX != v2) cellType |= 4;
		if (FLT_MAX != v3) cellType |= 8;

		// Retrieve edges to interpolate from the lookup table.
		const int* edges = marchingSquaresTable[cellType];
		if (edges[0] == -1) return; // No triangles for this cell.

		printf("threadid : %d\n", threadid);

		// Compute the interpolated vertices.
		Eigen::Vector3f p[4];
		p[0] = { gridMin.x() + xIndex * sampleSize, gridMin.y() + yIndex * sampleSize, v0 };
		p[1] = { gridMin.x() + (xIndex + 1) * sampleSize, gridMin.y() + yIndex * sampleSize, v1 };
		p[2] = { gridMin.x() + (xIndex + 1) * sampleSize, gridMin.y() + (yIndex + 1) * sampleSize, v2 };
		p[3] = { gridMin.x() + xIndex * sampleSize, gridMin.y() + (yIndex + 1) * sampleSize, v3 };

		Eigen::Vector3f interpolatedPoints[4];
		int interpolatedVertexIndices[4];

		for (int i = 0; i < 4 && edges[i] != -1; ++i) {
			// Perform interpolation for each edge
			int e0 = edges[i];
			int e1 = (e0 + 1) % 4; // Assuming edge connectivity follows p[0,1,2,3]

			//interpolatedPoints[i] = interpolate(p[e0], p[e1], v0, v1, isoValue);
			interpolatedPoints[i] = interpolate(v1 - v0, p[e0], p[e1]);

			// Atomic add to get a unique index for this vertex.
			interpolatedVertexIndices[i] = atomicAdd(vertexCount, 1);
			vertices[interpolatedVertexIndices[i]] = interpolatedPoints[i];

			auto ri = atomicAdd(numberOfResultPoints, 1);
			resultPoints[ri] = interpolatedPoints[i];
			resultPoints[ri].z() = v0;
		}

		// Generate indices for triangles
		size_t localIndexCount = atomicAdd(indexCount, 6); // Maximum of 6 indices (2 triangles).
		if (edges[0] != -1 && edges[1] != -1 && edges[2] != -1) {
			indices[localIndexCount] = interpolatedVertexIndices[0];
			indices[localIndexCount + 1] = interpolatedVertexIndices[1];
			indices[localIndexCount + 2] = interpolatedVertexIndices[2];
		}
		if (edges[2] != -1 && edges[3] != -1) {
			indices[localIndexCount + 3] = interpolatedVertexIndices[2];
			indices[localIndexCount + 4] = interpolatedVertexIndices[3];
			indices[localIndexCount + 5] = interpolatedVertexIndices[0];
		}
	}

	void PatchProcessor::Initialize(size_t gridWidth, size_t gridHeight, size_t numberOfPoints, CUstream_st* stream)
	{
		this->gridWidth = gridWidth;
		this->gridHeight = gridHeight;
		this->stream = stream;

		if (nullptr != stream)
		{
			checkCudaErrors(cudaMallocAsync(&grid, sizeof(float) * gridWidth * gridHeight, stream));
			checkCudaErrors(cudaStreamSynchronize(stream));
		}
		else
		{
			checkCudaErrors(cudaMalloc(&grid, sizeof(float) * gridWidth * gridHeight));
			checkCudaErrors(cudaDeviceSynchronize());
		}

		ClearGrid();

		AllocateBuffers(numberOfPoints);
	}

	void PatchProcessor::Terminate()
	{
		if (nullptr != stream)
		{
			checkCudaErrors(cudaFreeAsync(grid, stream));
		}
		else
		{
			checkCudaErrors(cudaFree(grid));
		}

		DeallocatedBuffers();
	}

	void PatchProcessor::Prepare(Eigen::Vector3f* inputPoints, size_t numberOfInputPoints)
	{
		this->sampleSize = sampleSize;

		if (0 == h_numberOfInputPoints)
		{
			AllocateBuffers(numberOfInputPoints);
		}
		else if (h_numberOfInputPoints < numberOfInputPoints)
		{
			DeallocatedBuffers();
			AllocateBuffers(numberOfInputPoints);
		}

		h_numberOfInputPoints = numberOfInputPoints;

		if (nullptr != stream)
		{
			cudaMemcpyAsync(d_inputPoints, inputPoints, sizeof(Eigen::Vector3f) * h_numberOfInputPoints, cudaMemcpyHostToDevice, stream);
			checkCudaErrors(cudaStreamSynchronize(stream));
		}
		else
		{
			cudaMemcpy(d_inputPoints, inputPoints, sizeof(Eigen::Vector3f) * h_numberOfInputPoints, cudaMemcpyHostToDevice);
			checkCudaErrors(cudaDeviceSynchronize());
		}

		ClearGrid();
	}

	void PatchProcessor::ClearGrid()
	{
		int mingridsize;
		int threadblocksize;
		checkCudaErrors(cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, Kernel_ClearGrid, 0, 0));
		auto gridsize = (gridWidth * gridHeight - 1) / threadblocksize;

		if (nullptr != stream)
		{
			Kernel_ClearGrid << <gridsize, threadblocksize, 0, stream >> > (grid, gridWidth, gridHeight);
			checkCudaErrors(cudaStreamSynchronize(stream));
		}
		else
		{
			Kernel_ClearGrid << <gridsize, threadblocksize >> > (grid, gridWidth, gridHeight);
			checkCudaErrors(cudaDeviceSynchronize());
		}
	}

	void PatchProcessor::ClearBuffers()
	{
		h_numberOfTriangleVertices = gridWidth * gridHeight * 2;
		h_numberOfTriangleIndices = h_numberOfTriangleVertices * 3;

		int mingridsize;
		int threadblocksize;
		checkCudaErrors(cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, Kernel_ClearBuffer_Vector3f, 0, 0));
		auto gridsize = (gridWidth * gridHeight - 1) / threadblocksize;

		if (nullptr != stream)
		{
			Kernel_ClearBuffer_Vector3f << <gridsize, threadblocksize, 0, stream >> > (d_inputPoints, h_numberOfInputPoints);
			Kernel_ClearBuffer_Vector3f << <gridsize, threadblocksize, 0, stream >> > (d_resultPoints, h_numberOfInputPoints);
			Kernel_ClearBuffer_Vector3f << <gridsize, threadblocksize, 0, stream >> > (d_triangleVertices, h_numberOfTriangleVertices);
			Kernel_ClearBuffer_UINT32 << <gridsize, threadblocksize, 0, stream >> > (d_triangleIndices, h_numberOfTriangleIndices);
			checkCudaErrors(cudaStreamSynchronize(stream));
		}
		else
		{
			Kernel_ClearBuffer_Vector3f << <gridsize, threadblocksize >> > (d_inputPoints, h_numberOfInputPoints);
			Kernel_ClearBuffer_Vector3f << <gridsize, threadblocksize >> > (d_resultPoints, h_numberOfInputPoints);
			Kernel_ClearBuffer_Vector3f << <gridsize, threadblocksize, 0, stream >> > (d_triangleVertices, h_numberOfTriangleVertices);
			Kernel_ClearBuffer_UINT32 << <gridsize, threadblocksize, 0, stream >> > (d_triangleIndices, h_numberOfTriangleIndices);
			checkCudaErrors(cudaDeviceSynchronize());
		}
	}

	void PatchProcessor::AllocateBuffers(size_t numberOfPoints)
	{
		h_numberOfInputPoints = numberOfPoints;
		h_inputPoints = new Eigen::Vector3f[h_numberOfInputPoints];

		h_numberOfResultPoints = h_numberOfInputPoints;
		h_resultPoints = new Eigen::Vector3f[h_numberOfResultPoints];

		h_numberOfTriangleVertices = gridWidth * gridHeight * 2;
		h_triangleVertices = new Eigen::Vector3f[h_numberOfTriangleVertices];

		h_numberOfTriangleIndices = h_numberOfTriangleVertices * 3;
		h_triangleIndices = new uint32_t[h_numberOfTriangleIndices];

		if (nullptr != stream)
		{
			checkCudaErrors(cudaMallocAsync(&d_numberOfInputPoints, sizeof(size_t), stream));
			checkCudaErrors(cudaMallocAsync(&d_inputPoints, sizeof(Eigen::Vector3f) * h_numberOfInputPoints, stream));
			checkCudaErrors(cudaMallocAsync(&d_numberOfResultPoints, sizeof(size_t), stream));
			checkCudaErrors(cudaMallocAsync(&d_resultPoints, sizeof(Eigen::Vector3f) * h_numberOfResultPoints, stream));
			checkCudaErrors(cudaMallocAsync(&d_numberOfTriangleVertices, sizeof(size_t), stream));
			checkCudaErrors(cudaMallocAsync(&d_triangleVertices, sizeof(Eigen::Vector3f) * h_numberOfTriangleVertices, stream));
			checkCudaErrors(cudaMallocAsync(&d_numberOfTriangleIndices, sizeof(size_t), stream));
			checkCudaErrors(cudaMallocAsync(&d_triangleIndices, sizeof(uint32_t) * h_numberOfTriangleIndices, stream));

			//checkCudaErrors(cudaMemcpyAsync(d_numberOfResultPoints, h_numberOfResultPoints, sizeof(size_t), cudaMemcpyHostToDevice, stream));
			cudaMemsetAsync(d_numberOfResultPoints, 0, sizeof(size_t), stream);

			checkCudaErrors(cudaStreamSynchronize(stream));
		}
		else
		{
			checkCudaErrors(cudaMalloc(&d_numberOfInputPoints, sizeof(size_t)));
			checkCudaErrors(cudaMalloc(&d_inputPoints, sizeof(Eigen::Vector3f) * h_numberOfInputPoints));
			checkCudaErrors(cudaMalloc(&d_numberOfResultPoints, sizeof(size_t)));
			checkCudaErrors(cudaMalloc(&d_resultPoints, sizeof(Eigen::Vector3f) * h_numberOfResultPoints));
			checkCudaErrors(cudaMalloc(&d_numberOfTriangleVertices, sizeof(size_t)));
			checkCudaErrors(cudaMalloc(&d_triangleVertices, sizeof(Eigen::Vector3f) * h_numberOfTriangleVertices));
			checkCudaErrors(cudaMalloc(&d_numberOfTriangleIndices, sizeof(size_t)));
			checkCudaErrors(cudaMalloc(&d_triangleIndices, sizeof(uint32_t) * h_numberOfTriangleIndices));

			//checkCudaErrors(cudaMemcpy(d_numberOfResultPoints, h_numberOfResultPoints, sizeof(size_t), cudaMemcpyHostToDevice));
			cudaMemset(d_numberOfResultPoints, 0, sizeof(size_t));

			checkCudaErrors(cudaDeviceSynchronize());
		}

		ClearBuffers();
	}

	void PatchProcessor::DeallocatedBuffers()
	{
		if (nullptr != stream)
		{
			checkCudaErrors(cudaFreeAsync(d_inputPoints, stream));
			checkCudaErrors(cudaFreeAsync(d_numberOfInputPoints, stream));
			checkCudaErrors(cudaFreeAsync(d_resultPoints, stream));
			checkCudaErrors(cudaFreeAsync(d_numberOfResultPoints, stream));
			checkCudaErrors(cudaFreeAsync(d_triangleVertices, stream));
			checkCudaErrors(cudaFreeAsync(d_numberOfTriangleVertices, stream));
			checkCudaErrors(cudaFreeAsync(d_triangleIndices, stream));
			checkCudaErrors(cudaFreeAsync(d_numberOfTriangleIndices, stream));

			checkCudaErrors(cudaStreamSynchronize(stream));
		}
		else
		{
			checkCudaErrors(cudaFree(d_inputPoints));
			checkCudaErrors(cudaFree(d_numberOfInputPoints));
			checkCudaErrors(cudaFree(d_resultPoints));
			checkCudaErrors(cudaFree(d_numberOfResultPoints));
			checkCudaErrors(cudaFree(d_triangleVertices));
			checkCudaErrors(cudaFree(d_numberOfTriangleVertices));
			checkCudaErrors(cudaFree(d_triangleIndices));
			checkCudaErrors(cudaFree(d_numberOfTriangleIndices));

			checkCudaErrors(cudaDeviceSynchronize());
		}

		if (nullptr != h_inputPoints) delete[] h_inputPoints;
		if (nullptr != h_resultPoints) delete[] h_resultPoints;
		if (nullptr != h_triangleVertices) delete[] h_triangleVertices;
		if (nullptr != h_resultPoints) delete[] h_triangleIndices;

		h_numberOfInputPoints = 0;
		h_numberOfResultPoints = 0;
		h_numberOfTriangleVertices = 0;
		h_numberOfTriangleIndices = 0;
	}

	void PatchProcessor::DownSample(
		Eigen::Vector3f* inputPoints,
		size_t numberOfInputPoints,
		float sampleSize,
		const Eigen::Vector3f& inputPointsMin,
		const Eigen::Vector3f& inputPointsMax)
	{
		Prepare(inputPoints, numberOfInputPoints);

		{
			int mingridsize;
			int threadblocksize;
			checkCudaErrors(cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, Kernel_DownSample, 0, 0));
			auto gridsize = (h_numberOfInputPoints - 1) / threadblocksize;

			if (nullptr != stream)
			{
				Kernel_DownSample << <gridsize, threadblocksize, 0, stream >> > (
					d_inputPoints,
					numberOfInputPoints,
					inputPointsMin,
					inputPointsMax,
					sampleSize,
					grid,
					gridWidth,
					gridHeight);

				checkCudaErrors(cudaStreamSynchronize(stream));
			}
			else
			{
				Kernel_DownSample << <gridsize, threadblocksize >> > (
					d_inputPoints,
					numberOfInputPoints,
					inputPointsMin,
					inputPointsMax,
					sampleSize,
					grid,
					gridWidth,
					gridHeight);

				checkCudaErrors(cudaDeviceSynchronize());
			}
		}

		{
			if (nullptr != stream)
			{
				checkCudaErrors(cudaMemsetAsync(d_numberOfResultPoints, 0, sizeof(size_t), stream));

				checkCudaErrors(cudaStreamSynchronize(stream));
			}
			else
			{
				checkCudaErrors(cudaMemset(d_numberOfResultPoints, 0, sizeof(size_t)));

				checkCudaErrors(cudaDeviceSynchronize());
			}

			int mingridsize;
			int threadblocksize;
			checkCudaErrors(cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, Kernel_GetPoints, 0, 0));
			auto gridsize = (gridWidth * gridHeight - 1) / threadblocksize;

			if (nullptr != stream)
			{
				Kernel_GetPoints << <gridsize, threadblocksize, 0, stream >> > (
					grid,
					gridWidth,
					gridHeight,
					sampleSize,
					inputPointsMin,
					d_resultPoints,
					d_numberOfResultPoints);

				checkCudaErrors(cudaStreamSynchronize(stream));
			}
			else
			{
				Kernel_GetPoints << <gridsize, threadblocksize >> > (
					grid,
					gridWidth,
					gridHeight,
					sampleSize,
					inputPointsMin,
					d_resultPoints,
					d_numberOfResultPoints);

				checkCudaErrors(cudaDeviceSynchronize());
			}

			if (nullptr != stream)
			{
				checkCudaErrors(cudaMemcpyAsync(&h_numberOfResultPoints, d_numberOfResultPoints, sizeof(size_t), cudaMemcpyDeviceToHost, stream));
				checkCudaErrors(cudaMemcpyAsync(h_resultPoints, d_resultPoints, sizeof(Eigen::Vector3f) * h_numberOfResultPoints, cudaMemcpyDeviceToHost, stream));

				checkCudaErrors(cudaStreamSynchronize(stream));
			}
			else
			{
				cudaMemcpy(&h_numberOfResultPoints, d_numberOfResultPoints, sizeof(size_t), cudaMemcpyDeviceToHost);
				cudaMemcpy(h_resultPoints, d_resultPoints, sizeof(Eigen::Vector3f) * h_numberOfResultPoints, cudaMemcpyDeviceToHost);

				checkCudaErrors(cudaDeviceSynchronize());
			}
		}
	}

	void PatchProcessor::MedianFilter(
		Eigen::Vector3f* inputPoints,
		size_t numberOfInputPoints,
		float sampleSize,
		const Eigen::Vector3f& inputPointsMin,
		const Eigen::Vector3f& inputPointsMax)
	{
		Prepare(inputPoints, numberOfInputPoints);

		{
			int mingridsize;
			int threadblocksize;
			checkCudaErrors(cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, Kernel_DownSample, 0, 0));
			auto gridsize = (h_numberOfInputPoints - 1) / threadblocksize;

			if (nullptr != stream)
			{
				Kernel_DownSample << <gridsize, threadblocksize, 0, stream >> > (
					d_inputPoints,
					numberOfInputPoints,
					inputPointsMin,
					inputPointsMax,
					sampleSize,
					grid,
					gridWidth,
					gridHeight);

				checkCudaErrors(cudaStreamSynchronize(stream));
			}
			else
			{
				Kernel_DownSample << <gridsize, threadblocksize >> > (
					d_inputPoints,
					numberOfInputPoints,
					inputPointsMin,
					inputPointsMax,
					sampleSize,
					grid,
					gridWidth,
					gridHeight);

				checkCudaErrors(cudaDeviceSynchronize());
			}
		}

		{
			int mingridsize;
			int threadblocksize;
			checkCudaErrors(cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, Kernel_MedianFilter, 0, 0));
			auto gridsize = (gridWidth * gridHeight - 1) / threadblocksize;

			if (nullptr != stream)
			{
				Kernel_MedianFilter << <gridsize, threadblocksize, 0, stream >> > (
					grid,
					gridWidth,
					gridHeight,
					sampleSize,
					3,
					inputPointsMin);

				checkCudaErrors(cudaStreamSynchronize(stream));
			}
			else
			{
				Kernel_MedianFilter << <gridsize, threadblocksize >> > (
					grid,
					gridWidth,
					gridHeight,
					sampleSize,
					3,
					inputPointsMin);

				checkCudaErrors(cudaDeviceSynchronize());
			}
		}

		{
			if (nullptr != stream)
			{
				checkCudaErrors(cudaMemsetAsync(d_numberOfResultPoints, 0, sizeof(size_t), stream));

				checkCudaErrors(cudaStreamSynchronize(stream));
			}
			else
			{
				checkCudaErrors(cudaMemset(d_numberOfResultPoints, 0, sizeof(size_t)));

				checkCudaErrors(cudaDeviceSynchronize());
			}

			int mingridsize;
			int threadblocksize;
			checkCudaErrors(cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, Kernel_GetPoints, 0, 0));
			auto gridsize = (gridWidth * gridHeight - 1) / threadblocksize;

			if (nullptr != stream)
			{
				Kernel_GetPoints << <gridsize, threadblocksize, 0, stream >> > (
					grid,
					gridWidth,
					gridHeight,
					sampleSize,
					inputPointsMin,
					d_resultPoints,
					d_numberOfResultPoints);

				checkCudaErrors(cudaStreamSynchronize(stream));
			}
			else
			{
				Kernel_GetPoints << <gridsize, threadblocksize >> > (
					grid,
					gridWidth,
					gridHeight,
					sampleSize,
					inputPointsMin,
					d_resultPoints,
					d_numberOfResultPoints);

				checkCudaErrors(cudaDeviceSynchronize());
			}

			if (nullptr != stream)
			{
				cudaMemcpyAsync(&h_numberOfResultPoints, d_numberOfResultPoints, sizeof(size_t), cudaMemcpyDeviceToHost, stream);
				cudaMemcpyAsync(h_resultPoints, d_resultPoints, sizeof(Eigen::Vector3f) * h_numberOfResultPoints, cudaMemcpyDeviceToHost, stream);

				checkCudaErrors(cudaStreamSynchronize(stream));
			}
			else
			{
				cudaMemcpy(&h_numberOfResultPoints, d_numberOfResultPoints, sizeof(size_t), cudaMemcpyDeviceToHost);
				cudaMemcpy(h_resultPoints, d_resultPoints, sizeof(Eigen::Vector3f) * h_numberOfResultPoints, cudaMemcpyDeviceToHost);

				checkCudaErrors(cudaDeviceSynchronize());
			}
		}
	}

	void PatchProcessor::MarchingSquare(
		Eigen::Vector3f* inputPoints,
		size_t numberOfInputPoints,
		float sampleSize,
		const Eigen::Vector3f& inputPointsMin,
		const Eigen::Vector3f& inputPointsMax)
	{
		Prepare(inputPoints, numberOfInputPoints);

		{
			int mingridsize;
			int threadblocksize;
			checkCudaErrors(cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, Kernel_DownSample, 0, 0));
			auto gridsize = (h_numberOfInputPoints - 1) / threadblocksize;

			if (nullptr != stream)
			{
				Kernel_DownSample << <gridsize, threadblocksize, 0, stream >> > (
					d_inputPoints,
					numberOfInputPoints,
					inputPointsMin,
					inputPointsMax,
					sampleSize,
					grid,
					gridWidth,
					gridHeight);

				checkCudaErrors(cudaStreamSynchronize(stream));
			}
			else
			{
				Kernel_DownSample << <gridsize, threadblocksize >> > (
					d_inputPoints,
					numberOfInputPoints,
					inputPointsMin,
					inputPointsMax,
					sampleSize,
					grid,
					gridWidth,
					gridHeight);

				checkCudaErrors(cudaDeviceSynchronize());
			}
		}

		{
			if (nullptr != stream)
			{
				checkCudaErrors(cudaMemsetAsync(d_numberOfTriangleVertices, 0, sizeof(size_t), stream));
				checkCudaErrors(cudaMemsetAsync(d_numberOfTriangleIndices, 0, sizeof(size_t), stream));

				checkCudaErrors(cudaMemsetAsync(d_numberOfResultPoints, 0, sizeof(size_t), stream));

				checkCudaErrors(cudaStreamSynchronize(stream));
			}
			else
			{
				checkCudaErrors(cudaMemset(d_numberOfTriangleVertices, 0, sizeof(size_t)));
				checkCudaErrors(cudaMemset(d_numberOfTriangleIndices, 0, sizeof(size_t)));

				checkCudaErrors(cudaMemset(d_numberOfResultPoints, 0, sizeof(size_t)));

				checkCudaErrors(cudaDeviceSynchronize());
			}

			int mingridsize;
			int threadblocksize;
			checkCudaErrors(cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, Kernel_MarchingSquares, 0, 0));
			auto gridsize = (gridWidth * gridHeight - 1) / threadblocksize;

			if (nullptr != stream)
			{
				Kernel_MarchingSquares << <gridsize, threadblocksize, 0, stream >> > (
					grid,
					gridWidth,
					gridHeight,
					sampleSize,
					inputPointsMin,
					0.0f,
					d_triangleVertices,
					d_numberOfTriangleVertices,
					d_triangleIndices,
					d_numberOfTriangleIndices,
					d_resultPoints,
					d_numberOfResultPoints);

				checkCudaErrors(cudaStreamSynchronize(stream));
			}
			else
			{
				Kernel_MarchingSquares << <gridsize, threadblocksize >> > (
					grid,
					gridWidth,
					gridHeight,
					sampleSize,
					inputPointsMin,
					0.0f,
					d_triangleVertices,
					d_numberOfTriangleVertices,
					d_triangleIndices,
					d_numberOfTriangleIndices,
					d_resultPoints,
					d_numberOfResultPoints);

				checkCudaErrors(cudaDeviceSynchronize());
			}

			if (nullptr != stream)
			{
				checkCudaErrors(cudaMemcpyAsync(&h_numberOfTriangleVertices, d_numberOfTriangleVertices, sizeof(size_t), cudaMemcpyDeviceToHost, stream));
				checkCudaErrors(cudaMemcpyAsync(h_triangleVertices, d_triangleVertices, sizeof(Eigen::Vector3f) * h_numberOfTriangleVertices, cudaMemcpyDeviceToHost, stream));

				checkCudaErrors(cudaMemcpyAsync(&h_numberOfTriangleIndices, d_numberOfTriangleIndices, sizeof(size_t), cudaMemcpyDeviceToHost, stream));
				checkCudaErrors(cudaMemcpyAsync(h_triangleIndices, d_triangleIndices, sizeof(Eigen::Vector3f) * h_numberOfTriangleIndices, cudaMemcpyDeviceToHost, stream));

				checkCudaErrors(cudaStreamSynchronize(stream));
			}
			else
			{
				checkCudaErrors(cudaMemcpy(&h_numberOfTriangleVertices, d_numberOfTriangleVertices, sizeof(size_t), cudaMemcpyDeviceToHost));
				checkCudaErrors(cudaMemcpy(h_triangleVertices, d_triangleVertices, sizeof(Eigen::Vector3f) * h_numberOfTriangleVertices, cudaMemcpyDeviceToHost));

				checkCudaErrors(cudaMemcpy(&h_numberOfTriangleIndices, d_numberOfTriangleIndices, sizeof(size_t), cudaMemcpyDeviceToHost));
				checkCudaErrors(cudaMemcpy(h_triangleIndices, d_triangleIndices, sizeof(Eigen::Vector3f) * h_numberOfTriangleIndices, cudaMemcpyDeviceToHost));

				checkCudaErrors(cudaDeviceSynchronize());
			}

			if (nullptr != stream)
			{
				checkCudaErrors(cudaMemcpyAsync(&h_numberOfResultPoints, d_numberOfResultPoints, sizeof(size_t), cudaMemcpyDeviceToHost, stream));
				checkCudaErrors(cudaMemcpyAsync(h_resultPoints, d_resultPoints, sizeof(Eigen::Vector3f) * h_numberOfResultPoints, cudaMemcpyDeviceToHost, stream));

				checkCudaErrors(cudaStreamSynchronize(stream));
			}
			else
			{
				cudaMemcpy(&h_numberOfResultPoints, d_numberOfResultPoints, sizeof(size_t), cudaMemcpyDeviceToHost);
				cudaMemcpy(h_resultPoints, d_resultPoints, sizeof(Eigen::Vector3f) * h_numberOfResultPoints, cudaMemcpyDeviceToHost);

				checkCudaErrors(cudaDeviceSynchronize());
			}
		}
	}
}
