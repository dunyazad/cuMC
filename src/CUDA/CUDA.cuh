#pragma once

#include "CUDA_Common.cuh"

struct Voxel
{
	float x;
	float y;
	float z;

	float value;
};

Voxel* CUDATest();
