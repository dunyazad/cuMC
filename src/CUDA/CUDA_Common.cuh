#pragma once

#include <stdio.h>
#include <iostream>
#include <vector>

#include <cuda_runtime.h>
#include <cooperative_groups.h>
#include <nvtx3/nvToolsExt.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>