#pragma once

#include <Common.h>

namespace Algorithm
{

	class KDTreeNode
	{
	public:
		KDTreeNode(unsigned int point_index) : point_index(point_index) {}

		inline unsigned int GetPointIndex() const { return point_index; }

	private:
		unsigned int point_index = UINT_MAX;
		KDTreeNode* left = nullptr;
		KDTreeNode* right = nullptr;

	public:
		friend class KDTree;
	};

	class KDTree
	{
	public:
		KDTree() : points(nullptr) {}
		KDTree(const float* points) : points(points) {}
		~KDTree() { Clear(); }

		void Clear()
		{
			if (nullptr != root)
			{
				ClearRecursive(root);
				root = nullptr;
			}

			nearestNeighborNode = nullptr;
			nearestNeighbor = UINT_MAX;
			nearestNeighborDistance = FLT_MAX;
		}

		void Insert(unsigned int point_index)
		{
			root = InsertRecursive(root, point_index, 0);
		}

		unsigned int FindNearestNeighbor(const float* query) const
		{
			nearestNeighbor = root->point_index;
			auto dx = points[root->point_index * 3 + 0] - query[0];
			auto dy = points[root->point_index * 3 + 1] - query[1];
			auto dz = points[root->point_index * 3 + 2] - query[2];
			nearestNeighborDistance = dx * dx + dy * dy + dz * dz;
			nearestNeighborNode = nullptr;
			FindNearestNeighborRecursive(root, query, 0);
			return nearestNeighbor;
		}

		KDTreeNode* FindNearestNeighborNode(const float* query) const
		{
			if (nullptr == root)
				return nullptr;

			nearestNeighbor = UINT_MAX;
			nearestNeighborDistance = FLT_MAX;
			nearestNeighborNode = nullptr;
			FindNearestNeighborRecursive(root, query, 0);
			return nearestNeighborNode;
		}

		vector<unsigned int> RangeSearchSquaredDistance(const float* query, float squaredRadius) const
		{
			vector<unsigned int> result;
			RangeSearchRecursive(root, query, squaredRadius, result, 0);
			return result;
		}

		inline bool IsEmpty() const { return nullptr == root; }

		inline void SetPoints(const float* points) { this->points = points; }

	private:
		const float* points;

		KDTreeNode* root = nullptr;
		mutable KDTreeNode* nearestNeighborNode = nullptr;
		mutable unsigned int nearestNeighbor = UINT_MAX;
		mutable float nearestNeighborDistance = FLT_MAX;

		void ClearRecursive(KDTreeNode* node)
		{
			if (nullptr != node->left)
			{
				ClearRecursive(node->left);
			}

			if (nullptr != node->right)
			{
				ClearRecursive(node->right);
			}

			delete node;
		}

		KDTreeNode* InsertRecursive(KDTreeNode* node, unsigned int point_index, int depth)
		{
			if (node == nullptr)
			{
				auto newNode = new KDTreeNode(point_index);
				return newNode;
			}

			int currentDimension = depth % 3;
			float pointValue = points[point_index * 3 + currentDimension];
			float nodeValue = points[node->point_index * 3 + currentDimension];

			if (pointValue < nodeValue)
			{
				node->left = InsertRecursive(node->left, point_index, depth + 1);
			}
			else
			{
				node->right = InsertRecursive(node->right, point_index, depth + 1);
			}

			return node;
		}

		void FindNearestNeighborRecursive(KDTreeNode* node, const float* query, int depth) const
		{
			if (node == nullptr)
			{
				return;
			}

			int currentDimension = depth % 3;
			auto dx = points[node->point_index * 3 + 0] - query[0];
			auto dy = points[node->point_index * 3 + 1] - query[1];
			auto dz = points[node->point_index * 3 + 2] - query[2];
			float nodeDistance = sqrtf(dx * dx + dy * dy + dz * dz);
			if (nodeDistance < nearestNeighborDistance)
			{
				nearestNeighborNode = node;
				nearestNeighbor = node->point_index;
				nearestNeighborDistance = nodeDistance;
			}

			auto queryValue = query[currentDimension];
			auto nodeValue = points[node->point_index * 3 + currentDimension];

			KDTreeNode* closerNode = (queryValue < nodeValue) ? node->left : node->right;
			KDTreeNode* otherNode = (queryValue < nodeValue) ? node->right : node->left;

			FindNearestNeighborRecursive(closerNode, query, depth + 1);

			float planeDistance = queryValue - nodeValue;
			if (planeDistance * planeDistance < nearestNeighborDistance) {
				FindNearestNeighborRecursive(otherNode, query, depth + 1);
			}
		}

		void RangeSearchRecursive(KDTreeNode* node, const float* query, float squaredRadius, std::vector<unsigned int>& result, int depth) const
		{
			if (node == nullptr)
			{
				return;
			}

			auto dx = points[node->point_index * 3 + 0] - query[0];
			auto dy = points[node->point_index * 3 + 1] - query[1];
			auto dz = points[node->point_index * 3 + 2] - query[2];
			float nodeDistance = sqrtf(dx * dx + dy * dy + dz * dz);

			if (nodeDistance <= squaredRadius)
			{
				result.push_back(node->point_index);
			}

			int currentDimension = depth % 3;
			float queryValue = query[currentDimension];
			float nodeValue = points[node->point_index * 3 + currentDimension];

			KDTreeNode* closerNode = (queryValue < nodeValue) ? node->left : node->right;
			KDTreeNode* otherNode = (queryValue < nodeValue) ? node->right : node->left;

			RangeSearchRecursive(closerNode, query, squaredRadius, result, depth + 1);

			float distanceToPlane = queryValue - nodeValue;
			if (distanceToPlane * distanceToPlane <= squaredRadius)
			{
				RangeSearchRecursive(otherNode, query, squaredRadius, result, depth + 1);
			}
		}
	};

}