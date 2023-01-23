#pragma once
#ifndef CUDA_PROJ_HPP
#define CUDA_PROJ_HPP

#include <cuda_runtime.h>

namespace octomap_depth_mapping
{

// template <typename T> // ushort for mono16, uchar for mono8
// // @param depth: depth image
// // @param pc: projected point cloud
// // @param params: array of parameters = [fx, fy, cx, cy, r1, r[1-9], t[1-3]]
// __global__ void project_kernel(ushort* depth, ushort* pc, double* params);




} // octomap_depth_mapping

#endif // CUDA_PROJ_HPP