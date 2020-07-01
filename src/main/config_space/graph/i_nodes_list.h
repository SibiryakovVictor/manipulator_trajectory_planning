/**************************************************************************************************
Описание

Определение интерфейса, получающего требуемую информацию о вершине на запрашиваемой позиции

Разработчик: Сибиряков Виктор
Заметки
**************************************************************************************************/


#pragma once

#include "graph_typedefs.h"
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

    virtual NodeId getNodesAmount() const = 0;

    virtual bool hasNodeFreeEdge( NodeId nodePos ) const = 0;

    virtual EdgeId getAmountFreeEdges( NodeId nodesPos ) const = 0;

    virtual EdgeId getNodeAmountEdges( NodeId nodePos ) const = 0;

    virtual const Point & getNodeConfig( NodeId nodePos ) const = 0;

    // возвращает число ближайших соседей вершины, которое определяется 
    // работой класса Est (est.h)
    virtual NodeId getNodesInArea( NodeId nodePos ) const = 0;
};

