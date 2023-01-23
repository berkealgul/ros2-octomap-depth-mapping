#pragma once
#ifndef CUDA_PROJ_HPP
#define CUDA_PROJ_HPP

#include <cuda_runtime.h>


namespace octomap_depth_mapping
{

__global__ void project_kernel(ushort* depth, double* pc, int width, int padding,
    double fx, double fy, double cx, double cy,
    double r1, double r2, double r3,
    double r4, double r5, double r6,
    double r7, double r8, double r9,
    double t1, double t2, double t3);

} // octomap_depth_mapping

#endif // CUDA_PROJ_HPP
