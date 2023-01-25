#include "cuda_proj.hpp" 
#include "depth_conversions.hpp"


namespace octomap_depth_mapping
{

void project_depth_img(ushort* depth, double* pc, int width, int padding,
    dim3 grid_, dim3 block_,
    double fx, double fy, double cx, double cy,
    double r1, double r2, double r3,
    double r4, double r5, double r6,
    double r7, double r8, double r9,
    double t1, double t2, double t3)
{
    dim3 grid, block;
    block.x = 32;
    block.y = 32;
    grid.x = (width + block.x - 1) / block.x;
    grid.y = (480 + block.y - 1) / block.y;

    project_kernel<<<grid, block>>>(depth, pc, width, padding,
        fx, fy, cx, cy,
        r1, r2, r3,
        r4, r5, r6,
        r7, r8, r9,
        t1, t2, t3);

    //std::cout << "project_kernel: " << cudaGetErrorString(cudaGetLastError()) << std::endl;
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

    if(i % padding != 0 || j % padding != 0) { return; }

    const int idx = i + width * j;
    const int pidx = 3*((i/padding) + (width/padding) * (j/padding));

    double d;
    depth_to_meters(depth[idx], d);

    if(d == 0)
    {
        pc[pidx] = 0;
        pc[pidx+1] = 0;
        pc[pidx+2] = 0;
        return;
    }

    double x = (i - cx) * d / fx;
    double y = (j - cy) * d / fy;

    pc[pidx]   = r1*x + r2*y + r3*d + t1;
    pc[pidx+1] = r4*x + r5*y + r6*d + t2;
    pc[pidx+2] = r7*x + r8*y + r9*d + t3;
}

} // octomap_depth_mapping
