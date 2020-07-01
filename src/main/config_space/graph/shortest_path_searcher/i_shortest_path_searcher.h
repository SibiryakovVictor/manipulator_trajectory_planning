/***********************************************************************************************************************************
Описание

Интерфейс поисковика кратчайшего пути, использующего интерфейсы списков вершин и рёбер для получения инфы
о графе, массив секций, передающийся по ссылке для занесения результата, и возвращаемое значение в качестве
количества секций найденного пути.

Разработчик: Сибиряков Виктор
Заметки
***********************************************************************************************************************************/


#pragma once

#include <cinttypes>


#include "../i_nodes_list.h"
#include "../i_edges_list.h"
#include "main/path/path_typedefs.h"



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

    virtual path::PathElemId findShortestPath( path::Path & pathData, const INodesList & nodesList,
                                               const IEdgesList & edgesList ) = 0;

};

