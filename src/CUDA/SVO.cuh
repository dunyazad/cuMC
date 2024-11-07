#pragma once

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>

#include <cuda_runtime.h>
#include <nvtx3/nvToolsExt.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

namespace Algorithm
{
    class OctreeNode
    {
    public:
        bool isLeafNode;
        int children[8];

        float tsdf;    // TSDF value
        float weight;  // Accumulated weight
    };

    typedef size_t OctreeNodePtr;
    const OctreeNodePtr InvalidPointer = UINT64_MAX;

    class Octree
    {
    public:
        OctreeNodePtr root = InvalidPointer;
        int maxDepth = 13;

        Eigen::Vector3f min = Eigen::Vector3f(-250.0f, -250.0f, -250.0f);
        Eigen::Vector3f max = Eigen::Vector3f(250.0f, 250.0f, 250.0f);

        void Insert(const Eigen::Vector3f& position)
        {
            auto nodePtr = root;
            if (InvalidPointer != root)
            {

            }
            else
            {

            }
        }

        OctreeNodePtr NewNode()
        {

        }
    };

	void Test(const std::vector<Eigen::Vector3f>& inputPoints);
}