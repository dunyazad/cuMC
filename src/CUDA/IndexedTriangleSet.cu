#include "IndexedTriangleSet.cuh"

namespace CUDA
{
	Mesh AllocateMesh(float* vertices, uint64_t numberOfVertices, uint32_t* indices, uint64_t numberOfTriangles)
	{
		Mesh mesh;
		mesh.numberOfVertices = numberOfVertices;
		cudaMallocManaged(&mesh.vertices, sizeof(float) * numberOfVertices * 3);
		cudaMemcpy(mesh.vertices, vertices, sizeof(float) * numberOfVertices * 3, cudaMemcpyHostToDevice);

		mesh.numberOfTriangles = numberOfTriangles;
		cudaMallocManaged(&mesh.indices, sizeof(uint32_t) * numberOfTriangles * 3);
		cudaMemcpy(mesh.indices, indices, sizeof(uint32_t) * numberOfTriangles * 3, cudaMemcpyHostToDevice);

		cudaDeviceSynchronize();

		return mesh;
	}

	void DeallocMesh(Mesh* mesh)
	{
		cudaFree(mesh->vertices);
		cudaFree(mesh->indices);

		cudaDeviceSynchronize();
	}
}
