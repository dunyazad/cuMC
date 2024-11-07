#include "CUDA.cuh"

namespace CUDA
{
	__global__ void Kernel_InitializeCache(Voxel* cache, int3 cacheSize)
	{
		int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
		int yIndex = blockIdx.y * blockDim.y + threadIdx.y;
		int zIndex = blockIdx.z * blockDim.z + threadIdx.z;

		int index = zIndex * cacheSize.z * cacheSize.y + yIndex * cacheSize.x + xIndex;
		if (index > cacheSize.x * cacheSize.y * cacheSize.z - 1) return;

		cache[index].tsdfValue = FLT_MAX;
		cache[index].weight = 0.0f;
		cache[index].normal = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
		cache[index].color = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
	}

	__device__ void UpdateVoxel(
		int3 index,
		Voxel* cache,
		int3 cacheSize,
		float voxelSize,
		Eigen::Vector3f cacheMin,
		Eigen::Matrix4f transform,
		uint32_t numberOfPoints,
		Eigen::Vector3f* points,
		Eigen::Vector3f* normals,
		Eigen::Vector3f* colors)
	{

	}

	__global__ void Kernel_Integrate(
		Voxel* cache,
		int3 cacheSize,
		float voxelSize,
		Eigen::Vector3f cacheMin,
		Eigen::Matrix4f transform,
		uint32_t numberOfPoints,
		Eigen::Vector3f* points,
		Eigen::Vector3f* normals,
		Eigen::Vector3f* colors)
	{
		uint32_t threadid = blockIdx.x * blockDim.x + threadIdx.x;
		if (threadid > numberOfPoints - 1) return;

		if (nullptr == cache) return;
		if (nullptr == points) return;

		auto& p = points[threadid];

		float offsetX = p.x() - cacheMin.x();
		float offsetY = p.y() - cacheMin.y();
		float offsetZ = p.z() - cacheMin.z();

		if (0 > offsetX || 0 > offsetY || 0 > offsetZ) return;

		int xIndex = (int)floorf(offsetX / voxelSize);
		int yIndex = (int)floorf(offsetY / voxelSize);
		int zIndex = (int)floorf(offsetZ / voxelSize);

		if (xIndex < 0 || xIndex >= cacheSize.x || yIndex < 0 || yIndex >= cacheSize.y || zIndex < 0 || zIndex >= cacheSize.z) return;

		auto vp = Eigen::Vector3f(
			cacheMin.x() + xIndex * cacheSize.x,
			cacheMin.y() + yIndex * cacheSize.y,
			cacheMin.z() + zIndex * cacheSize.z);

		for (size_t i = 0; i < 10 * 10 * 10; i++)
		{
			int index = zIndex * cacheSize.z * cacheSize.y + yIndex * cacheSize.x + xIndex;
			if (0.0f == cache[index].weight)
			{
				cache[index].tsdfValue = 0.0f;
				cache[index].weight = 1.0f;
			}
			else
			{
				float newTsdfValue = 0.0f; // Calculate based on points and cache[index].tsdfValue
				cache[index].tsdfValue = (cache[index].tsdfValue * cache[index].weight + newTsdfValue) / (cache[index].weight + 1);
				cache[index].weight += 1.0f;
			}
		}
	}

	cuCache::cuCache(int xLength, int yLength, int zLength, float voxelSize)
		: voxelSize(voxelSize)
	{
		cacheSize = make_int3(xLength, yLength, zLength);
		cudaMallocManaged(&cache, sizeof(Voxel) * xLength * yLength * zLength);

		ClearCache();
	}

	cuCache::~cuCache()
	{
		cudaFree(cache);
	}

	void cuCache::ClearCache()
	{
		dim3 blockDim(8, 8, 8);
		dim3 gridDim(
			(cacheSize.x + blockDim.x - 1) / blockDim.x,
			(cacheSize.y + blockDim.y - 1) / blockDim.y,
			(cacheSize.z + blockDim.z - 1) / blockDim.z);

		Kernel_InitializeCache << < gridDim, blockDim >> > (cache, cacheSize);
	}

	void cuCache::Integrate(
		const Eigen::Vector3f& cacheMin,
		const Eigen::Matrix4f& transform,
		uint32_t numberOfPoints,
		Eigen::Vector3f* points,
		Eigen::Vector3f* normals,
		Eigen::Vector3f* colors)
	{
		nvtxRangePushA("Integrate");

		this->cacheMin = cacheMin;

		int threadblocksize = 512;
		uint32_t gridsize = (numberOfPoints - 1) / threadblocksize;
		Kernel_Integrate << < gridsize, threadblocksize >> > (
			cache, cacheSize, voxelSize, cacheMin, transform, numberOfPoints, points, normals, colors);

		cudaDeviceSynchronize();

		nvtxRangePop();
	}
}