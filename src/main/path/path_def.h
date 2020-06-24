#pragma once

#include "main/config_space/graph/typedefs.h"
#include "path_parameters.h"




namespace motion_planner
{
    namespace path
    {

        struct PathElement
        {
            NodeId point = UINT16_MAX;

            EdgeId segment = UINT8_MAX;
            
            explicit PathElement()
            {}
            
            explicit PathElement( NodeId p, EdgeId s ) :
                point( p ),
                segment( s )
            {}

            bool areEquals( const PathElement & pe )
            {
                return ( point == pe.point ) && ( segment == pe.segment );
            }
        };

        struct Path
        {
            PathElement sections[ path::length_limit ];
        };

    }
}

