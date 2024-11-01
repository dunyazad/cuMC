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

namespace AOctree
{
    struct AOctreeNode {
        size_t children[8] = { UINT64_MAX, UINT64_MAX, UINT64_MAX, UINT64_MAX, UINT64_MAX, UINT64_MAX, UINT64_MAX, UINT64_MAX };
        size_t pointIdx = UINT64_MAX;
        int lock = 0;
    };

	void Test(const std::vector<Eigen::Vector3f>& inputPoints);
}