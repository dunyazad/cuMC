#pragma once

#include "CUDA_Common.cuh"

namespace Algorithm
{
	struct kdiNode {
		size_t index = UINT64_MAX;
		kdiNode* left = nullptr;
		kdiNode* right = nullptr;
		float distance = -FLT_MAX;
	};

	class kdiTree {
	public:
		__device__ __host__ kdiTree();
		__device__ __host__ virtual ~kdiTree();

		void Init(Eigen::Vector3f* points, size_t numberOfPoints);
		void Reset();
		void Free(void);
		
		__device__ __host__ inline float dist(kdiNode* a, kdiNode* b, int dim);
		__device__ __host__ inline float dist(kdiNode* a, const Eigen::Vector3f& query, int dim);
		
		inline void swap(kdiNode* x, kdiNode* y);
		
		kdiNode* findMedian(kdiNode* start, kdiNode* end, int idx);
		
		kdiNode* buildTree(kdiNode* t, int len, int i, int dim);
		
		__device__ __host__ void findNearest(
			kdiNode* root,
			kdiNode* nd,
			int i,
			int dim,
			kdiNode** best,
			float* best_dist);

		__device__ 	__host__ void findKNearest(
			kdiNode* root,
			kdiNode* nd,
			int i,
			int dim,
			kdiNode** best,
			float* best_dist,
			kdiNode* VisitedNodes);

		__device__ 	__host__ void findKNearest(
			kdiNode* root,
			const Eigen::Vector3f& query,
			int i,
			int dim,
			kdiNode** best,
			float* best_dist,
			kdiNode* VisitedNodes);
		
		__device__ __host__ void findKNN(kdiNode& targetNode);
		__device__ __host__ void findKNN(const Eigen::Vector3f& query);
		__device__ __host__ inline void sortNodes(int visitedNum);

		Eigen::Vector3f* points = nullptr;
		size_t numberOfPoints = 0;

		int visited;
		float* kdDistances;
		kdiNode* nodes = nullptr;
		kdiNode* kdRoot = nullptr;
		kdiNode* kdQuery = nullptr;
		kdiNode* kdFound = nullptr;
		kdiNode* VisitedNodes = nullptr;
	};
}
