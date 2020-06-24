#pragma once

#include "main/tools/intervals/intervals.h"


namespace motion_planner
{
    namespace config_space
    {
        namespace graph
        {
            namespace generation
            {
                class UniformConfigGen;
            }
        }
    }
}


class motion_planner::config_space::graph::generation::UniformConfigGen
{
public:

    explicit UniformConfigGen( 
        IntervalsAllDims ranges = tools::RangeLinksMotion::get() ) :
        m_ranges( ranges )
    {}

    Point generate() const;

    void changeRanges( const IntervalsAllDims & newRanges );

private:

    IntervalsAllDims m_ranges;
};
