#pragma once

#include "CUDA_Common.cuh"

namespace CUDA
{
	class Voxel
	{
	public:
		float tsdfValue = FLT_MAX;
		float weight = 0.0f;
		Eigen::Vector3f normal = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
		Eigen::Vector3f color = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
	};

	class cuCache
	{
	public:
		cuCache(int xLength = 200, int yLength = 200, int zLength = 200, float voxelSize = 0.1f);
		~cuCache();

		void ClearCache();

		void Integrate(
			const Eigen::Vector3f& cacheMin,
			const Eigen::Matrix4f& transform,
			uint32_t numberOfPoints,
			Eigen::Vector3f* points,
			Eigen::Vector3f* normals,
			Eigen::Vector3f* colors);

	//private:
		int3 cacheSize = make_int3(200, 200, 200);
		float voxelSize = 0.1f;
		Eigen::Vector3f cacheMin = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
		Voxel* cache = nullptr;
	};

}