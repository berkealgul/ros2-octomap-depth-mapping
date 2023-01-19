#pragma once
#ifndef DEPTH_CONVERSIONS_HPP
#define DEPTH_CONVERSIONS_HPP

namespace octomap_depth_mapping
{

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


} // octomap_depth_mapping

#endif // DEPTH_CONVERSIONS_HPPs