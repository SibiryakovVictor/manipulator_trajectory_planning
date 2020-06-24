#pragma once

#include "typedefs.h"



namespace motion_planner
{
    namespace config_space
    {
        namespace graph
        {
            class IEdgesList;
        }
    }
}


class motion_planner::config_space::graph::IEdgesList
{

public:

    virtual ~IEdgesList() {}

    virtual NodeId getNodeId( NodeId nodePos, EdgeId edgePos ) const = 0;

};

