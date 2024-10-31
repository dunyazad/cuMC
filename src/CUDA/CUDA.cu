#include "CUDA.cuh"

struct VolumInfo
{
	float voxelSize = 0.1f;
	float minX = -25.0f;
	float minY = -25.0f;
	float minZ = -25.0f;
	size_t volumeSizeX = 500;
	size_t volumeSizeY = 500;
	size_t volumeSizeZ = 500;
	size_t numberOfVoxels = 500 * 500 * 500;
};

__global__ void TestFill(VolumInfo info, Voxel* voxels, float cx, float cy, float cz, float radius)
{
	size_t threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid > info.numberOfVoxels - 1) return;

	auto indexZ = threadid / (info.volumeSizeX * info.volumeSizeY);
	auto indexY = (threadid % (info.volumeSizeX * info.volumeSizeY)) / info.volumeSizeX;
	auto indexX = (threadid % (info.volumeSizeX * info.volumeSizeY)) % info.volumeSizeX;

	auto x = info.minX + (float)indexX * info.voxelSize;
	auto y = info.minY + (float)indexY * info.voxelSize;
	auto z = info.minZ + (float)indexZ * info.voxelSize;

	auto dx = x - cx;
	auto dy = y - cy;
	auto dz = z - cz;

	voxels[threadid].x = x;
	voxels[threadid].y = y;
	voxels[threadid].z = z;
	voxels[threadid].value = sqrtf(dx * dx + dy * dy + dz * dz);

	//printf("voxel : %f, %f, %f, %f\n", voxels[threadid].x, voxels[threadid].y, voxels[threadid].z, voxels[threadid].value);
}

Voxel* CUDATest()
{
	VolumInfo info
	{
		0.1f,
		-25.0f, -25.0f, -25.0f,
		500, 500, 500,
		500 * 500 * 500
	};

	Voxel* grid = nullptr;
	cudaMallocManaged(&grid, sizeof(Voxel) * info.volumeSizeX * info.volumeSizeY * info.volumeSizeZ);

	nvtxRangePushA("TestFill");

	int minGridSize;
	int threadBlockSize;
	cudaOccupancyMaxPotentialBlockSize(&minGridSize, &threadBlockSize, TestFill, 0, 0);
	int gridSize = (info.numberOfVoxels + threadBlockSize - 1) / threadBlockSize;

	TestFill << <gridSize, threadBlockSize, 0, 0>>> (info, grid, 0.0f, 0.0f, 0.0f, 10.0f);

	cudaDeviceSynchronize();

	nvtxRangePop();

	return grid;
}