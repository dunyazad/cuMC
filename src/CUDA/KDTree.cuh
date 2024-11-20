#pragma once

#include <stdio.h>
#include <iostream>
#include <vector>

#include <cuda_runtime.h>
#include <nvtx3/nvToolsExt.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

namespace CUDA
{
	// KDtree dimension
#define KNN 		4
#define MAX_DIM 	3
#define expWeight 	false
#define NODE_NUM    30000000
#define dgw 		1
#define EPSILLON  	1e-4

	struct kdNode {
		int id = -1;
		float x[MAX_DIM] = { 0.0f, 0.0f , 0.0f };
		kdNode* left = nullptr;
		kdNode* right = nullptr;
		float distance = FLT_MAX;
	};

	class kdTree {
	public:
		__device__ __host__ kdTree();
		__device__ __host__ virtual ~kdTree();

		void init(int querySize);
		void Free(void);
		
		__device__ __host__ inline float dist(struct kdNode* a, struct kdNode* b, int dim);
		
		inline void swap(struct kdNode* x, struct kdNode* y);
		
		kdNode* findMedian(struct kdNode* start, struct kdNode* end, int idx);
		
		kdNode* buildTree(struct kdNode* t, int len, int i, int dim);
		
		__device__ __host__ void findNearest(
			struct kdNode* root,
			struct kdNode* nd,
			int i,
			int dim,
			struct kdNode** best,
			float* best_dist);
		
		__device__ 	__host__ void findKNearest(
			struct kdNode* root,
			struct kdNode* nd,
			int i,
			int dim,
			struct kdNode** best,
			float* best_dist,
			struct kdNode* VisitedNodes);
		
		__device__ __host__ void findKNN(struct kdNode& targetNode);
		__device__ __host__ inline void sortNodes(int visitedNum);

		int visited;
		float* kdDistnaces;
		kdNode* kdRoot = nullptr;
		kdNode* kdQuery = nullptr;
		kdNode* kdFound = nullptr;
		kdNode* VisitedNodes = nullptr;
	};
}

namespace CUDA
{
	void Test();

	//std::vector<Eigen::Vector3f> DoFilter(Eigen::Vector3f* points);

	//std::vector<Eigen::Vector3f> BitonicSort(Eigen::Vector3f* points, int numberOfPoints);

	void BitonicSortGPU(Eigen::Vector3f* arr, int n, int axis);

	void cudaMergeSort(Eigen::Vector3f* data, int* indices, int size);
}