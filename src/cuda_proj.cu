#include "cuda_proj.hpp" 
#include "depth_conversions.hpp"

#include <stdio.h>

namespace octomap_depth_mapping
{

void project_depth_img(ushort* depth, double* pc, int width, int padding,
    double fx, double fy, double cx, double cy,
    double r1, double r2, double r3,
    double r4, double r5, double r6,
    double r7, double r8, double r9,
    double t1, double t2, double t3)
{
    dim3 block(16, 16);
    //dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
    dim3 grid((width + block.x - 1) / block.x, (480 + block.y - 1) / block.y);
    
    project_kernel<<<grid, block>>>(depth, pc, width, padding,
        fx, fy, cx, cy,
        r1, r2, r3,
        r4, r5, r6,
        r7, r8, r9,
        t1, t2, t3);

    cudaDeviceSynchronize();
}

__global__ void project_kernel(ushort* depth, double* pc, int width, int padding,
    double fx, double fy, double cx, double cy,
    double r1, double r2, double r3,
    double r4, double r5, double r6,
    double r7, double r8, double r9,
    double t1, double t2, double t3)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    const int j = blockIdx.y * blockDim.y + threadIdx.y;

    // if(i % padding != 0 || j % padding != 0)
    //     return;

    const int idx = i + width * j;

    double d;
    depth_to_meters(depth[idx], d);

    // 3d coordinates
    double x = (i - cx) * d / fx;
    double y = (j - cy) * d / fy;
    double z = d;

    // apply transform to point
    x = r1*x + r2*x + r3*x + t1;
    y = r4*y + r5*y + r6*y + t2;
    z = r7*z + r8*z + r9*z + t3;

    pc[idx] = x;
    pc[idx+1] = y;
    pc[idx+2] = z;

    printf("i: %d, j: %d, x: %f, y: %f, z: %f\n", i,j,x,y,z);
}


} // octomap_depth_mapping