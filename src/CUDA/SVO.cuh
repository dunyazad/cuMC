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
    struct OctreeNode {
        bool isLeafNode;
        int children[8];
        int pointIdx;
    };

	void Test(const std::vector<Eigen::Vector3f>& inputPoints);
}