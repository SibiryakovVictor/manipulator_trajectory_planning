/**************************************************************************************************
Описание

Определение типов структур данных, работающим с путем

Разработчик: Сибиряков Виктор
Заметки
**************************************************************************************************/



#pragma once

#include "main/config_space/graph/graph_typedefs.h"
#include "path_parameters.h"




namespace motion_planner
{
    namespace path
    {
        typedef uint8_t PathElemId;

        struct PathElement
        {
            config_space::graph::NodeId point = config_space::graph::empty_node;

            config_space::graph::EdgeId segment = config_space::graph::empty_edge;
            
            PathElement()
            {}
            
            explicit PathElement( config_space::graph::NodeId nodePos, 
                                  config_space::graph::EdgeId edgePos ):
                point( nodePos ),
                segment( edgePos )
            {}

            bool areEquals( const PathElement & pe )
            {
                return ( point == pe.point ) && ( segment == pe.segment );
            }
        };

        const PathElement empty_path_elem = PathElement( config_space::graph::empty_node, 
                                                         config_space::graph::empty_edge );

        struct Path
        {
            PathElement sections[ path::length_limit ];
        };
    }
}

