#pragma once

#include "typedefs.h"
#include "main/config_space/point/point.h"


namespace motion_planner
{
    namespace config_space
    {
        namespace graph
        {
            class INodesList;
        }
    }
}

class motion_planner::config_space::graph::INodesList
{

public:

    virtual ~INodesList() {}

    virtual uint16_t getNodesAmount() const = 0;

    virtual NodeId getPosLastNode() const = 0;

    virtual bool hasNodeFreeEdge( NodeId nodePos ) const = 0;

    virtual uint8_t getNodesInArea( NodeId nodePos ) const = 0;

    virtual uint8_t getAmountFreeEdges( NodeId nodesPos ) const = 0;

    virtual bool isNodeInserted( NodeId nodePos ) const = 0;

    virtual const config_space::Point & getNodeConfig( NodeId nodePos ) const = 0;

};

