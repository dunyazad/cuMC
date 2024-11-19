#include "SVO.h"

#include <Debugging/VisualDebugging.h>

namespace SVO
{
	std::vector<Eigen::Vector3f> GetVoxelCorners(const Eigen::Vector3f& center, float halfSize)
	{
		return {
			center + Eigen::Vector3f(-halfSize, -halfSize, -halfSize),
			center + Eigen::Vector3f(halfSize, -halfSize, -halfSize),
			center + Eigen::Vector3f(-halfSize,  halfSize, -halfSize),
			center + Eigen::Vector3f(halfSize,  halfSize, -halfSize),
			center + Eigen::Vector3f(-halfSize, -halfSize,  halfSize),
			center + Eigen::Vector3f(halfSize, -halfSize,  halfSize),
			center + Eigen::Vector3f(-halfSize,  halfSize,  halfSize),
			center + Eigen::Vector3f(halfSize,  halfSize,  halfSize)
		};
	}

	void InitializeNode(SVONode* node, const Eigen::Vector3f& center, float size)
	{
		node->isLeaf = false;
		node->parentIndex = Max.U64;
		node->voxelIndex = Max.U64;
		node->childIndex[0] = Max.U64;
		node->childIndex[1] = Max.U64;
		node->childIndex[2] = Max.U64;
		node->childIndex[3] = Max.U64;
		node->childIndex[4] = Max.U64;
		node->childIndex[5] = Max.U64;
		node->childIndex[6] = Max.U64;
		node->childIndex[7] = Max.U64;
		node->center = center;
		node->size = size;
		node->tsdfValue = Max.FLOAT;
		node->occupied = false;
		node->weight = 0.0f;
		node->updatedCount = 0;
	}

	void InitializeSVO(Octree* octree, size_t numberOfNodes)
	{
		octree->nodes = new SVONode[numberOfNodes];
		octree->numberOfNodes = numberOfNodes;
		octree->nodeBufferIndex = 0;
		octree->rootIndex = Max.U64;
		octree->maxDepth = 0;
	}

	void IntegratePointCloud(Octree* octree, const Eigen::Vector3f& center, const Eigen::Matrix4f& transform, const Eigen::Vector3f* pointCloud, size_t numberOfPoints, float rootVoxelSize, int maxDepth)
	{
		for (size_t i = 0; i < numberOfPoints; ++i)
		{
			const Eigen::Vector3f& patchPoint = pointCloud[i];
			Eigen::Vector4f tpp = transform * Eigen::Vector4f(patchPoint.x(), patchPoint.y(), patchPoint.z(), 1.0f);
			Eigen::Vector3f point(tpp.x(), tpp.y(), tpp.z());

			size_t currentIndex = octree->rootIndex;

			if (currentIndex == Max.U64)
			{
				octree->rootIndex = 0;
				InitializeNode(&octree->nodes[octree->rootIndex], center, rootVoxelSize);
				currentIndex = octree->rootIndex;
			}

			int currentDepth = 0;

			while (!octree->nodes[currentIndex].isLeaf)
			{
				if (currentDepth >= maxDepth)
				{
					octree->nodes[currentIndex].isLeaf = true;
					break;
				}

				Eigen::Vector3f nodeCenter = octree->nodes[currentIndex].center;
				float currentVoxelSize = octree->nodes[currentIndex].size;

				int childIdx = GetChildIndex(point, nodeCenter);

				if (octree->nodes[currentIndex].childIndex[childIdx] == Max.U64)
				{
					octree->nodes[currentIndex].childIndex[childIdx] = ++octree->nodeBufferIndex;

					float halfSize = currentVoxelSize / 2.0f;
					Eigen::Vector3f childCenter = nodeCenter;

					childCenter.x() += (childIdx & 1) ? halfSize / 2.0f : -halfSize / 2.0f;
					childCenter.y() += (childIdx & 2) ? halfSize / 2.0f : -halfSize / 2.0f;
					childCenter.z() += (childIdx & 4) ? halfSize / 2.0f : -halfSize / 2.0f;

					//printf("octree->nodeBufferIndex : %llu, halfSize : %f\n", octree->nodeBufferIndex, halfSize);

					InitializeNode(&octree->nodes[octree->nodeBufferIndex], childCenter, halfSize);
				}

				currentIndex = octree->nodes[currentIndex].childIndex[childIdx];
				currentDepth++;

				if (currentDepth > octree->maxDepth)
				{
					octree->maxDepth = currentDepth;
					//printf("octree->maxDepth : %llu\n", octree->maxDepth);
				}
			}

			UpdateNodeWithSDF(octree->nodes[currentIndex], point);
		}
	}

	void IntegratePointCloudWithTSDF(Octree* octree, const Eigen::Vector3f& center, const Eigen::Matrix4f& transform, const Eigen::Vector3f* pointCloud, size_t numberOfPoints, float rootVoxelSize, int maxDepth, float truncationDistance)
	{
		for (size_t i = 0; i < numberOfPoints; ++i)
		{
			const Eigen::Vector3f& patchPoint = pointCloud[i];
			Eigen::Vector4f tpp = transform * Eigen::Vector4f(patchPoint.x(), patchPoint.y(), patchPoint.z(), 1.0f);
			Eigen::Vector3f point(tpp.x(), tpp.y(), tpp.z());

			size_t currentIndex = octree->rootIndex;

			if (currentIndex == Max.U64)
			{
				octree->rootIndex = 0;
				InitializeNode(&octree->nodes[octree->rootIndex], center, rootVoxelSize);
				currentIndex = octree->rootIndex;
			}

			int currentDepth = 0;

			while (!octree->nodes[currentIndex].isLeaf)
			{
				if (currentDepth >= maxDepth)
				{
					octree->nodes[currentIndex].isLeaf = true;
					break;
				}

				Eigen::Vector3f nodeCenter = octree->nodes[currentIndex].center;
				float currentVoxelSize = octree->nodes[currentIndex].size;

				int childIdx = GetChildIndex(point, nodeCenter);

				if (octree->nodes[currentIndex].childIndex[childIdx] == Max.U64)
				{
					octree->nodes[currentIndex].childIndex[childIdx] = ++octree->nodeBufferIndex;

					float halfSize = currentVoxelSize / 2.0f;
					Eigen::Vector3f childCenter = nodeCenter;

					childCenter.x() += (childIdx & 1) ? halfSize / 2.0f : -halfSize / 2.0f;
					childCenter.y() += (childIdx & 2) ? halfSize / 2.0f : -halfSize / 2.0f;
					childCenter.z() += (childIdx & 4) ? halfSize / 2.0f : -halfSize / 2.0f;

					InitializeNode(&octree->nodes[octree->nodeBufferIndex], childCenter, halfSize);
				}

				currentIndex = octree->nodes[currentIndex].childIndex[childIdx];
				currentDepth++;
			}

			float sdfValue = ComputeTSDFValue(octree->nodes[currentIndex], point, truncationDistance);
			float weight = 1.0f;
			UpdateNodeWithTSDF(octree->nodes[currentIndex], point, sdfValue, weight);
		}
	}

	void IntegratePointCloudWithNeighborUpdate(Octree* octree, const Eigen::Vector3f& center, const Eigen::Matrix4f& transform, const Eigen::Vector3f* pointCloud, size_t numberOfPoints, float rootVoxelSize, int maxDepth, float truncationDistance)
	{
		for (size_t i = 0; i < numberOfPoints; ++i)
		{
			const Eigen::Vector3f& patchPoint = pointCloud[i];
			Eigen::Vector4f tpp = transform * Eigen::Vector4f(patchPoint.x(), patchPoint.y(), patchPoint.z(), 1.0f);
			Eigen::Vector3f point(tpp.x(), tpp.y(), tpp.z());

			size_t currentIndex = octree->rootIndex;

			if (currentIndex == Max.U64)
			{
				octree->rootIndex = 0;
				InitializeNode(&octree->nodes[octree->rootIndex], center, rootVoxelSize);
				currentIndex = octree->rootIndex;
			}

			int currentDepth = 0;

			while (!octree->nodes[currentIndex].isLeaf)
			{
				if (currentDepth >= maxDepth)
				{
					octree->nodes[currentIndex].isLeaf = true;
					break;
				}

				Eigen::Vector3f nodeCenter = octree->nodes[currentIndex].center;
				float currentVoxelSize = octree->nodes[currentIndex].size;

				int childIdx = GetChildIndex(point, nodeCenter);

				if (octree->nodes[currentIndex].childIndex[childIdx] == Max.U64)
				{
					octree->nodes[currentIndex].childIndex[childIdx] = ++octree->nodeBufferIndex;

					float halfSize = currentVoxelSize / 2.0f;
					Eigen::Vector3f childCenter = nodeCenter;

					childCenter.x() += (childIdx & 1) ? halfSize / 2.0f : -halfSize / 2.0f;
					childCenter.y() += (childIdx & 2) ? halfSize / 2.0f : -halfSize / 2.0f;
					childCenter.z() += (childIdx & 4) ? halfSize / 2.0f : -halfSize / 2.0f;

					InitializeNode(&octree->nodes[octree->nodeBufferIndex], childCenter, halfSize);
				}

				currentIndex = octree->nodes[currentIndex].childIndex[childIdx];
				currentDepth++;
			}

			float sdfValue = ComputeTSDFValue(octree->nodes[currentIndex], point, truncationDistance);
			float weight = 1.0f;
			UpdateNodeWithTSDF(octree->nodes[currentIndex], point, sdfValue, weight);

			UpdateNeighboringNodesWithTSDF(octree, currentIndex, point, truncationDistance, weight, maxDepth, currentDepth);
		}
	}

	int GetChildIndex(const Eigen::Vector3f& point, const Eigen::Vector3f& nodeCenter)
	{
		int childIndex = 0;

		if (point.x() >= nodeCenter.x()) childIndex |= 1;
		if (point.y() >= nodeCenter.y()) childIndex |= 2;
		if (point.z() >= nodeCenter.z()) childIndex |= 4;

		return childIndex;
	}

	void UpdateNodeWithSDF(SVONode& node, const Eigen::Vector3f& point)
	{
		float tsdfValue = ComputeSDFValue(node, point);
		//if (tsdfValue != 0.0f)
		//{
		//	printf("tsdfValue : %f\n", tsdfValue);
		//}

		if (node.updatedCount > 0) {
			node.tsdfValue = (node.tsdfValue * node.updatedCount + tsdfValue) / (node.updatedCount + 1);
		}
		else {
			node.tsdfValue = tsdfValue;
		}
		node.updatedCount++;

		node.occupied = (node.tsdfValue < 0.0f);

		//if (node.occupied)
		//{
		//	printf("OK\n");
		//}
	}

	void UpdateNodeWithTSDF(SVONode& node, const Eigen::Vector3f& point, float sdfValue, float weight) {
		// TSDF ������Ʈ
		float totalWeight = node.weight + weight;
		node.tsdfValue = (node.tsdfValue * node.weight + sdfValue * weight) / totalWeight;
		node.weight = totalWeight;
		node.occupied = (node.tsdfValue < 0.0f && std::abs(node.tsdfValue) < node.size);

		// �� �ڳ��� SDF ���� ������Ʈ
		float halfSize = node.size / 2.0f;
		std::vector<Eigen::Vector3f> corners = {
			node.center + Eigen::Vector3f(-halfSize, -halfSize, -halfSize),
			node.center + Eigen::Vector3f(halfSize, -halfSize, -halfSize),
			node.center + Eigen::Vector3f(-halfSize, halfSize, -halfSize),
			node.center + Eigen::Vector3f(halfSize, halfSize, -halfSize),
			node.center + Eigen::Vector3f(-halfSize, -halfSize, halfSize),
			node.center + Eigen::Vector3f(halfSize, -halfSize, halfSize),
			node.center + Eigen::Vector3f(-halfSize, halfSize, halfSize),
			node.center + Eigen::Vector3f(halfSize, halfSize, halfSize)
		};

		// �� �ڳʿ� ���� �־��� ����Ʈ�κ����� �Ÿ��� �������� SDF ���� ������Ʈ
		for (int i = 0; i < 8; ++i) {
			float distance = (corners[i] - point).norm();
			float cornerSDF = distance - halfSize;

			if (node.cornerSDFs[i] == Max.FLOAT) {
				// �ڳʰ� �ʱ�ȭ���� ���� ��� �ʱⰪ ����
				node.cornerSDFs[i] = cornerSDF;
			}
			else {
				// ����ġ�� �ݿ��Ͽ� ���� SDF ���� �����Ͽ� ������Ʈ
				node.cornerSDFs[i] = (node.cornerSDFs[i] * node.weight + cornerSDF * weight) / totalWeight;
			}
		}
	}

	void EnsureNeighboringNodesExist(Octree* octree, size_t nodeIndex, const Eigen::Vector3f& point, float truncationDistance, int maxDepth, int currentDepth)
	{
		if (nodeIndex == Max.U64 || currentDepth > maxDepth)
		{
			return;
		}

		std::queue<std::pair<size_t, int>> nodesToVisit;
		nodesToVisit.push({ nodeIndex, currentDepth });

		while (!nodesToVisit.empty())
		{
			auto [currentIndex, currentDepth] = nodesToVisit.front();
			nodesToVisit.pop();

			SVONode& currentNode = octree->nodes[currentIndex];

			float halfSize = currentNode.size / 2.0f;

			for (int i = 0; i < 8; ++i)
			{
				size_t& childIdx = currentNode.childIndex[i];
				Eigen::Vector3f childCenter = currentNode.center;

				childCenter.x() += (i & 1) ? halfSize / 2.0f : -halfSize / 2.0f;
				childCenter.y() += (i & 2) ? halfSize / 2.0f : -halfSize / 2.0f;
				childCenter.z() += (i & 4) ? halfSize / 2.0f : -halfSize / 2.0f;

				float distanceToChild = (point - childCenter).norm();

				if (distanceToChild < truncationDistance)
				{
					if (childIdx == Max.U64)
					{
						if (currentDepth < maxDepth)
						{
							childIdx = ++octree->nodeBufferIndex;
							InitializeNode(&octree->nodes[childIdx], childCenter, halfSize);
						}
						else if (currentDepth == maxDepth)
						{
							childIdx = ++octree->nodeBufferIndex;
							InitializeNode(&octree->nodes[childIdx], childCenter, halfSize);
						}
					}

					if (childIdx != Max.U64 && currentDepth < maxDepth)
					{
						nodesToVisit.push({ childIdx, currentDepth + 1 });
					}
				}
			}
		}
	}

	void UpdateNeighboringNodesWithTSDF(Octree* octree, size_t nodeIndex, const Eigen::Vector3f& point, float truncationDistance, float weight, int maxDepth, int currentDepth)
	{
		EnsureNeighboringNodesExist(octree, nodeIndex, point, truncationDistance, maxDepth, currentDepth);

		std::queue<size_t> nodesToVisit;
		nodesToVisit.push(nodeIndex);

		while (!nodesToVisit.empty())
		{
			size_t currentIndex = nodesToVisit.front();
			nodesToVisit.pop();

			SVONode& currentNode = octree->nodes[currentIndex];

			float sdfValue = ComputeTSDFValue(currentNode, point, truncationDistance);
			UpdateNodeWithTSDF(currentNode, point, sdfValue, weight);

			if (!currentNode.isLeaf)
			{
				for (int i = 0; i < 8; ++i)
				{
					size_t& childIdx = currentNode.childIndex[i];
					if (childIdx != Max.U64 && (currentDepth + 1) <= maxDepth)
					{
						nodesToVisit.push(childIdx);
					}
				}
			}
		}
	}

	float ComputeSDFValue(const SVONode& node, const Eigen::Vector3f& point) {
		Eigen::Vector3f toPoint = point - node.center;
		float halfSize = node.size / 2.0f;

		// �� �࿡�� �Ÿ� ���̸� ���
		float dx = std::max(0.0f, std::abs(toPoint.x()) - halfSize);
		float dy = std::max(0.0f, std::abs(toPoint.y()) - halfSize);
		float dz = std::max(0.0f, std::abs(toPoint.z()) - halfSize);

		float sdfValue = std::sqrt(dx * dx + dy * dy + dz * dz);

		// ����Ʈ�� �ڽ� ���ο� �ִ� ��� ������ ����
		if (std::abs(toPoint.x()) < halfSize &&
			std::abs(toPoint.y()) < halfSize &&
			std::abs(toPoint.z()) < halfSize) {
			sdfValue = -sdfValue;
		}

		return sdfValue;
	}

	float ComputeTSDFValue(const SVONode& node, const Eigen::Vector3f& point, float truncationDistance)
	{
		Eigen::Vector3f toPoint = point - node.center;
		float distance = toPoint.norm();

		float halfSize = node.size / 2.0f;
		float sdfValue = distance - halfSize;

		return std::min(std::max(sdfValue, -truncationDistance), truncationDistance);
	}

	void TraverseOctree(Octree* octree, size_t nodeIndex, int currentDepth, const std::function<void(const SVONode&, int)>& operation)
	{
		if (nodeIndex == Max.U64)
		{
			return;
		}

		const SVONode& currentNode = octree->nodes[nodeIndex];

		operation(currentNode, currentDepth);

		if (currentNode.isLeaf)
		{
			return;
		}

		for (int i = 0; i < 8; ++i)
		{
			if (currentNode.childIndex[i] != Max.U64)
			{
				TraverseOctree(octree, currentNode.childIndex[i], currentDepth + 1, operation);
			}
		}
	}

	void TraverseOctree(Octree* octree, const std::function<void(const SVONode&, int)>& operation)
	{
		if (octree->rootIndex != Max.U64)
		{
			TraverseOctree(octree, octree->rootIndex, 0, operation);
		}
		else
		{
			std::cout << "The octree is empty." << std::endl;
		}
	}

	void NearestNeighborDFS(Octree* octree, size_t nodeIndex, const Eigen::Vector3f& queryPoint, NearestNeighborResult& result) {
		if (nodeIndex == Max.U64) {
			return;
		}

		const SVONode& currentNode = octree->nodes[nodeIndex];

		float distSquaredToCenter = (queryPoint - currentNode.center).squaredNorm();

		if (distSquaredToCenter > result.distanceSquared) {
			return;
		}

		if (currentNode.isLeaf && currentNode.occupied) {
			if (distSquaredToCenter < result.distanceSquared) {
				result.distanceSquared = distSquaredToCenter;
				result.nearestPoint = currentNode.center;
			}
			return;
		}

		std::vector<std::pair<size_t, float>> childrenDistances;
		for (int i = 0; i < 8; ++i) {
			if (currentNode.childIndex[i] != Max.U64) {
				const SVONode& childNode = octree->nodes[currentNode.childIndex[i]];
				float childDistSquared = (queryPoint - childNode.center).squaredNorm();
				childrenDistances.emplace_back(currentNode.childIndex[i], childDistSquared);
			}
		}

		std::sort(childrenDistances.begin(), childrenDistances.end(),
			[](const std::pair<size_t, float>& a, const std::pair<size_t, float>& b) {
				return a.second < b.second;
			});

		for (const auto& child : childrenDistances) {
			NearestNeighborDFS(octree, child.first, queryPoint, result);
		}
	}

	void GetCornerSDFValues(const SVONode& node, std::array<float, 8>& cornerSDFs) {
		float halfSize = node.size / 2.0f;
		const Eigen::Vector3f center = node.center;

		std::vector<Eigen::Vector3f> corners = {
			center + Eigen::Vector3f(-halfSize, -halfSize, -halfSize),
			center + Eigen::Vector3f(halfSize, -halfSize, -halfSize),
			center + Eigen::Vector3f(-halfSize,  halfSize, -halfSize),
			center + Eigen::Vector3f(halfSize,  halfSize, -halfSize),
			center + Eigen::Vector3f(-halfSize, -halfSize,  halfSize),
			center + Eigen::Vector3f(halfSize, -halfSize,  halfSize),
			center + Eigen::Vector3f(-halfSize,  halfSize,  halfSize),
			center + Eigen::Vector3f(halfSize,  halfSize,  halfSize)
		};

		for (int i = 0; i < 8; ++i) {
			Eigen::Vector3f toCorner = corners[i] - center;
			float distance = toCorner.norm();
			cornerSDFs[i] = node.tsdfValue - distance;
			//std::cout << "Corner [" << i << "] Position: " << corners[i].transpose()
			//	<< ", SDF Value: " << cornerSDFs[i] << std::endl;
		}
	}

	void MarchingCubes(const SVONode& node, std::vector<Triangle>& triangles) {
		const std::array<float, 8>& cornerSDFs = node.cornerSDFs;

		// �� �ڳ��� SDF ���� Ȯ���ϰ�, cubeIndex�� ����
		int cubeIndex = 0;
		for (int i = 0; i < 8; ++i) {
			if (cornerSDFs[i] < 0.0f) {
				cubeIndex |= (1 << i);
			}
		}

		// �ﰢ���� ���� ���
		if (MARCHING_CUBES_TABLE[cubeIndex] == 0) {
			return;
		}

		float halfSize = node.size / 2.0f;
		const auto corners = GetVoxelCorners(node.center, halfSize);

		Eigen::Vector3f vertList[12];

		// �� ������ ���� �����Ͽ� �ﰢ���� ���ؽ��� ���
		for (int i = 0; i < 12; ++i) {
			if (MARCHING_CUBES_TABLE[cubeIndex] & (1 << i)) {
				int idx1 = edgeConnection[i][0];
				int idx2 = edgeConnection[i][1];

				const Eigen::Vector3f& p1 = corners[idx1];
				const Eigen::Vector3f& p2 = corners[idx2];
				float val1 = cornerSDFs[idx1];
				float val2 = cornerSDFs[idx2];

				// ������ ���� ������ �߰� ���� ���
				float t = val1 / (val1 - val2);
				vertList[i] = p1 + t * (p2 - p1);
			}
		}

		// �ﰢ�� ����
		for (int i = 0; TRIANGLE_TABLE[cubeIndex][i] != -1; i += 3) {
			Triangle tri;
			tri.vertices[0] = vertList[TRIANGLE_TABLE[cubeIndex][i]];
			tri.vertices[1] = vertList[TRIANGLE_TABLE[cubeIndex][i + 1]];
			tri.vertices[2] = vertList[TRIANGLE_TABLE[cubeIndex][i + 2]];
			triangles.push_back(tri);

			//// ������ �ﰢ�� �ð�ȭ
			//std::cout << "Generating triangle with vertices: "
			//	<< tri.vertices[0].transpose() << ", "
			//	<< tri.vertices[1].transpose() << ", "
			//	<< tri.vertices[2].transpose() << std::endl;

			VisualDebugging::AddTriangle("Triangles", tri.vertices[0], tri.vertices[1], tri.vertices[2], Color4::White);
		}
	}

	void ExtractTrianglesFromOctree(Octree* octree, std::vector<Triangle>& triangles)
	{
		TraverseOctree(octree, [&](const SVONode& node, int depth)
			{
				if (node.isLeaf && node.occupied)
				{
					MarchingCubes(node, triangles);
				}
			});
	}
}