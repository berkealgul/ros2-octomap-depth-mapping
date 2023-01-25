#pragma once
#ifndef CUDA_PROJ_HPP
#define CUDA_PROJ_HPP

#include <cuda_runtime.h>

namespace octomap_depth_mapping
{

void project_depth_img(ushort*, double*, int, int,
    dim3, dim3,
    double, double, double, double,
    double, double, double,
    double, double, double,
    double, double, double,
    double, double, double);

__global__ void project_kernel(ushort*, double*, int, int,
    double, double, double, double,
    double, double, double,
    double, double, double,
    double, double, double,
    double, double, double);

} // octomap_depth_mapping

#endif // CUDA_PROJ_HPP
