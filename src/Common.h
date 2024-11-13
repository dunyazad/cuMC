#pragma once

#include <stdHeaderFiles.h>
#include <vtkHeaderFiles.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

#include <Max.h>

typedef unsigned char ubyte;

namespace Time
{
	chrono::steady_clock::time_point Now();

    uint64_t Microseconds(chrono::steady_clock::time_point& from, chrono::steady_clock::time_point& now);

    chrono::steady_clock::time_point End(chrono::steady_clock::time_point& from, const string& message = "");

    string DateTime();
}

Eigen::Matrix4f vtkToEigen(const vtkMatrix4x4* vtkMat);
vtkSmartPointer<vtkMatrix4x4> eigenToVtk(const Eigen::Matrix4f& eigenMat);