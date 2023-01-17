#ifndef DEPTH_CONVERSIONS_HPP
#define DEPTH_CONVERSIONS_HPP

namespace octomap_depth_mapping
{

// note: may need to add (double) to line 12 again if values are incorrect
double depth_to_meters(ushort raw_depth) 
{
    if(raw_depth > 6408)
    {
        return ((2.5-0.9)/(15800.0-6408.0))*raw_depth;
    }        

    return 0;
}


} // octomap_depth_mapping

#endif // DEPTH_CONVERSIONS_HPPs