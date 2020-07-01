/**************************************************************************************************
Описание

Определение интерфейса, возвращающего позицию вершины на другом конце 
запрашиваемого ребра вершины на запрашиваемой позиции

Разработчик: Сибиряков Виктор
Заметки
**************************************************************************************************/



#pragma once

#include "graph_typedefs.h"



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

    virtual NodeId getNodePos( NodeId nodePos, EdgeId edgePos ) const = 0;

};
