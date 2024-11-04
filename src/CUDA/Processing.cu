#include "Processing.cuh"

namespace Processing
{
	__global__ void Kernel_ClearGrid(float* grid, size_t gridWidth, size_t gridHeight)
	{
		size_t threadid = blockIdx.x * blockDim.x + threadIdx.x;
		if (threadid > gridWidth * gridHeight - 1) return;

		grid[threadid] = FLT_MAX;
	}

	__global__ void Kernel_ClearBuffer(Eigen::Vector3f* points, size_t numberOfPoints)
	{
		size_t threadid = blockIdx.x * blockDim.x + threadIdx.x;
		if (threadid > numberOfPoints - 1) return;

		points[threadid] = Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX);
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
								float nx = gridMin.x() + (float)(nix) * sampleSize;
								float ny = gridMin.y() + (float)(niy) * sampleSize;
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

	__global__ void Kernel_GetPoints(float* grid, size_t gridWidth, size_t gridHeight,
		float sampleSize, Eigen::Vector3f gridMin, Eigen::Vector3f* resultPoints, size_t* numberOfResultPoints)
	{
		size_t threadid = blockIdx.x * blockDim.x + threadIdx.x;
		if (threadid > gridWidth * gridHeight - 1) return;

		size_t xIndex = threadid % gridWidth;
		size_t yIndex = threadid / gridWidth;

		float x = gridMin.x() + (float)xIndex * sampleSize;
		float y = gridMin.y() + (float)yIndex * sampleSize;

		if (FLT_MAX != grid[threadid])
		{
			auto index = atomicAdd(numberOfResultPoints, 1);
			resultPoints[index] = Eigen::Vector3f(x, y, grid[threadid]);
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
		int mingridsize;
		int threadblocksize;
		checkCudaErrors(cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, Kernel_ClearBuffer, 0, 0));
		auto gridsize = (gridWidth * gridHeight - 1) / threadblocksize;

		if (nullptr != stream)
		{
			Kernel_ClearBuffer << <gridsize, threadblocksize, 0, stream >> > (d_inputPoints, h_numberOfInputPoints);
			Kernel_ClearBuffer << <gridsize, threadblocksize, 0, stream >> > (d_resultPoints, h_numberOfInputPoints);
			checkCudaErrors(cudaStreamSynchronize(stream));
		}
		else
		{
			Kernel_ClearBuffer << <gridsize, threadblocksize>> > (d_inputPoints, h_numberOfInputPoints);
			Kernel_ClearBuffer << <gridsize, threadblocksize>> > (d_resultPoints, h_numberOfInputPoints);
			checkCudaErrors(cudaDeviceSynchronize());
		}
	}

	void PatchProcessor::AllocateBuffers(size_t numberOfPoints)
	{
		h_numberOfInputPoints = numberOfPoints;
		h_inputPoints = new Eigen::Vector3f[h_numberOfInputPoints];

		h_numberOfResultPoints = h_numberOfInputPoints;
		h_resultPoints = new Eigen::Vector3f[h_numberOfResultPoints];

		if (nullptr != stream)
		{
			checkCudaErrors(cudaMallocAsync(&d_numberOfInputPoints, sizeof(size_t), stream));
			checkCudaErrors(cudaMallocAsync(&d_inputPoints, sizeof(Eigen::Vector3f) * h_numberOfInputPoints, stream));
			checkCudaErrors(cudaMallocAsync(&d_numberOfResultPoints, sizeof(size_t), stream));
			checkCudaErrors(cudaMallocAsync(&d_resultPoints, sizeof(Eigen::Vector3f) * h_numberOfResultPoints, stream));

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

			checkCudaErrors(cudaStreamSynchronize(stream));
		}
		else
		{
			checkCudaErrors(cudaFree(d_inputPoints));
			checkCudaErrors(cudaFree(d_numberOfInputPoints));
			checkCudaErrors(cudaFree(d_resultPoints));
			checkCudaErrors(cudaFree(d_numberOfResultPoints));

			checkCudaErrors(cudaDeviceSynchronize());
		}

		if (nullptr != h_inputPoints) delete[] h_inputPoints;
		if (nullptr != h_resultPoints) delete[] h_resultPoints;

		h_numberOfInputPoints = 0;
		h_numberOfResultPoints = 0;
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

}
