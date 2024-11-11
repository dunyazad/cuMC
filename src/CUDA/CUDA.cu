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

	__global__ void Kernel_Integrate(
		Voxel* cache,
		int3 cacheSize,
		float voxelSize,
		float tsdfThreshold,
		Eigen::Vector3f cacheMin,
		Eigen::Vector3f camPos,
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

		auto& op = points[threadid];
		auto tp = transform * Eigen::Vector4f(op.x(), op.y(), op.z(), 1.0f);
		auto p = Eigen::Vector3f(tp.x(), tp.y(), tp.z());

		auto dp = norm3df(p.x() - camPos.x(), p.y() - camPos.y(), p.z() - camPos.z());

		float offsetX = p.x() - cacheMin.x();
		float offsetY = p.y() - cacheMin.y();
		float offsetZ = p.z() - cacheMin.z();

		int xIndex = (int)floorf(offsetX / voxelSize);
		int yIndex = (int)floorf(offsetY / voxelSize);
		int zIndex = (int)floorf(offsetZ / voxelSize);

		//if (xIndex < 0 || xIndex >= cacheSize.x || yIndex < 0 || yIndex >= cacheSize.y || zIndex < 0 || zIndex >= cacheSize.z)	return;

		int offset = (int)ceilf(tsdfThreshold / voxelSize);
		for (int iz = zIndex - offset; iz <= zIndex + offset; iz++)
		{
			if (iz < 0 || iz > cacheSize.z) continue;
			for (int iy = yIndex - offset; iy <= yIndex + offset; iy++)
			{
				if (iy < 0 || iy > cacheSize.y) continue;
				for (int ix = xIndex - offset; ix <= xIndex + offset; ix++)
				{
					if (ix < 0 || ix > cacheSize.x) continue;

					auto vp = Eigen::Vector3f(
						cacheMin.x() + ix * voxelSize,
						cacheMin.y() + iy * voxelSize,
						cacheMin.z() + iz * voxelSize);

					auto distance = norm3df(vp.x() - p.x(), vp.y() - p.y(), vp.z() - p.z());

					auto dvp = norm3df(vp.x() - camPos.x(), vp.y() - camPos.y(), vp.z() - camPos.z());

					if (0 > dp - dvp) distance = -distance;

					if (tsdfThreshold > fabsf(distance))
					{
						//printf("distance : %f\n", distance);

						int index = iz * cacheSize.z * cacheSize.y + iy * cacheSize.x + ix;
						if (0.0f == cache[index].weight)
						{
							cache[index].tsdfValue = distance;
							cache[index].weight = 1.0f;
						}
						else
						{
							float newTsdfValue = distance;
							float tsdfValue = (cache[index].tsdfValue * cache[index].weight + newTsdfValue) / (cache[index].weight + 1);
							if (fabsf(tsdfValue) < fabsf(cache[index].tsdfValue))
								cache[index].tsdfValue = tsdfValue;
							cache[index].weight += 1.0f;
						}
					}
				}
			}
		}
	}

	cuCache::cuCache(int xLength, int yLength, int zLength, float voxelSize)
		: voxelSize(voxelSize)
	{
		cacheSize = make_int3(xLength, yLength, zLength);
		cudaMallocManaged(&cache, sizeof(Voxel) * xLength * yLength * zLength);
		//cudaMalloc(&cache, sizeof(Voxel) * xLength * yLength * zLength);

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
		const Eigen::Vector3f& camPos,
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
			cache, cacheSize, voxelSize, tsdfThreshold, cacheMin, camPos, transform, numberOfPoints, points, normals, colors);

		cudaDeviceSynchronize();

		nvtxRangePop();
	}

	void cuCache::Serialize(Voxel* h_voxels)
	{
		cudaMemcpy(h_voxels, cache, sizeof(Voxel) * cacheSize.x * cacheSize.y * cacheSize.z, cudaMemcpyDeviceToHost);
		cudaDeviceSynchronize();
	}
}