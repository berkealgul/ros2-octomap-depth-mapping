#pragma once
#ifndef DEPTH_CONVERSIONS_HPP
#define DEPTH_CONVERSIONS_HPP

#ifdef CUDA
#include <cuda_runtime.h>
#endif

namespace octomap_depth_mapping
{

#ifdef CUDA
// kinect v2 cuda
__device__ __forceinline__ void depth_to_meters(ushort raw_depth, double& depth) 
{
    if(raw_depth > 6408)
    {
        depth = ((2.5-0.9)/(15800.0-6408.0))*raw_depth;
    }        
    else
        depth = 0;
}

// tum dataset cuda
// __device__ __forceinline__ void depth_to_meters(ushort raw_depth, double& depth) 
// {
//     depth = raw_depth / 5000.0;      
// }
#else
// kinect v2
inline double depth_to_meters(ushort raw_depth) 
{
    if(raw_depth > 6408)
    {
        return ((2.5-0.9)/(15800.0-6408.0))*raw_depth;
    }        

    return 0;
}

// tum dataset
// inline double depth_to_meters(ushort raw_depth) 
// {
//     return raw_depth / 5000.0;      
// }
#endif

} // octomap_depth_mapping

#endif // DEPTH_CONVERSIONS_HPPs
