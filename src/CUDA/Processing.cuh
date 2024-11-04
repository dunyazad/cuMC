#pragma once

#include "CUDA_Common.cuh"

namespace Processing
{
	struct PointCloud
	{
		Eigen::Vector3f* points;
		size_t numberOfPoints;
		Eigen::Vector3f pointMin;
		Eigen::Vector3f pointMax;
	};

	PointCloud DownSample(PointCloud pointCloud, float sampleSize);

	class PatchProcessor
	{
	public:
		void Initialize(size_t gridWidth, size_t gridHeight, size_t numberOfPoints, CUstream_st* stream = nullptr);
		void Terminate();

		void Prepare(Eigen::Vector3f* inputPoints, size_t numberOfInputPoints);

		void ClearGrid();
		void ClearBuffers();

		void AllocateBuffers(size_t numberOfPoints);
		void DeallocatedBuffers();
		
		void DownSample(
			Eigen::Vector3f* inputPoints,
			size_t numberOfInputPoints,
			float sampleSize,
			const Eigen::Vector3f& inputPointsMin,
			const Eigen::Vector3f& inputPointsMax);

		void MedianFilter(
			Eigen::Vector3f* inputPoints,
			size_t numberOfInputPoints,
			float sampleSize,
			const Eigen::Vector3f& inputPointsMin,
			const Eigen::Vector3f& inputPointsMax);

	//protected:
		CUstream_st* stream = nullptr;
		int gridWidth = 200;
		int gridHeight = 200;
		float sampleSize = 0.1f;
		float* grid = nullptr;

		size_t h_numberOfInputPoints = 0;
		Eigen::Vector3f* h_inputPoints = nullptr;

		size_t* d_numberOfInputPoints = nullptr;
		Eigen::Vector3f* d_inputPoints = nullptr;

		size_t h_numberOfResultPoints = 0;
		Eigen::Vector3f* h_resultPoints = nullptr;

		size_t* d_numberOfResultPoints = nullptr;
		Eigen::Vector3f* d_resultPoints = nullptr;
	};
}