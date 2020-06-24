#pragma once

#include <cinttypes>


#include "../i_nodes_list.h"
#include "../i_edges_list.h"
#include "main/path/path_def.h"


namespace motion_planner
{
    namespace config_space
    {
        namespace graph
        {
            class IShortestPathFinder;
        }
    }
}


class motion_planner::config_space::graph::IShortestPathFinder
{

public:

    virtual ~IShortestPathFinder() {}

    virtual uint16_t findShortestPath( motion_planner::path::Path & pathData,
        const INodesList & nodesList, const IEdgesList & edgesList ) = 0;

};
