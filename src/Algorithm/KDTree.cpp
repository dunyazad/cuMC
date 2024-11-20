#include <Algorithm/KDTree.h>

namespace Algorithm
{

	float GetDistanceSquared(const float* points, unsigned int point_index, const float* query)
	{
		float distanceSquared = 0.0f;
		for (int i = 0; i < 3; ++i) {
			float diff = points[point_index * 3 + i] - query[i];
			distanceSquared += diff * diff;
		}
		return distanceSquared;
	}

	float GetDistance(const float* points, unsigned int point_index, const float* query)
	{
		auto dx = points[point_index * 3] - query[0];
		auto dy = points[point_index * 3 + 1] - query[1];
		auto dz = points[point_index * 3 + 2] - query[2];
		return sqrtf(dx * dx + dy * dy + dz * dz);
	}

	Eigen::AlignedBox3f TransformAABB(const Eigen::AlignedBox3f& aabb, const Eigen::Matrix4f& transform)
	{
		Eigen::AlignedBox3f transformedAABB(
			Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX), Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX));
		for (size_t i = 0; i < 8; i++)
		{
			const auto& corner = aabb.corner((Eigen::AlignedBox3f::CornerType)i);
			Eigen::Vector4f homogenousCorner(corner.x(), corner.y(), corner.z(), 1.0f);
			Eigen::Vector4f transformedCorner = transform * homogenousCorner;
			transformedAABB.extend(transformedCorner.head<3>());
		}

		return transformedAABB;
	}

	bool Intersects(const Eigen::AlignedBox3f& a, const Eigen::AlignedBox3f& b, const Eigen::Matrix4f& transformB)
	{
		return a.intersects(TransformAABB(b, transformB));
	}

}
