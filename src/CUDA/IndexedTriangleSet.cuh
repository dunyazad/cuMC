#pragma once

#include "CUDA_Common.cuh"

namespace CUDA
{
	struct Mesh
	{
		float3* vertices;
		uint64_t numberOfVertices;
		uint3* indices;
		uint64_t numberOfTriangles;
	};

	Mesh AllocateMesh(float* vertices, uint64_t numberOfVertices, uint32_t* indices, uint64_t numberOfTriangles);
	void DeallocMesh(Mesh* mesh);
}
