#pragma once

#include "main/env/obstacles/obstacles.h"
#include "main/env/manipulator/manipulator.h"


namespace motion_planner
{
    namespace coll
    {
        struct CheckPair
        {
            uint16_t body1 = 0;
            uint16_t body2 = 0;
            
            CheckPair()
            {}
            
            explicit CheckPair( uint16_t b1, uint16_t b2 ) :
                body1( b1 ),
                body2( b2 )
            {}
                
        };

        static const uint16_t pairsAmount =
            manip::links_amount * obst::obst_amount;

        static const uint16_t obsts = env::Obstacles::biasObstId;
        static const uint16_t links = env::Manipulator::biasLinkId;
        static const uint16_t groups = env::Manipulator::biasGroupId;

        static const coll::CheckPair list_check_pairs[ pairsAmount ] = {
        CheckPair( links + 1, obsts + 2 ), CheckPair( links + 1, obsts + 3 ),
        CheckPair( groups + 1, obsts + 0 ), CheckPair( groups + 1, obsts + 1 ), 
        CheckPair( groups + 1, obsts + 2 ), CheckPair( groups + 1, obsts + 3 ),
        CheckPair( links + 4, obsts + 0 ), CheckPair( links + 4, obsts + 1 ),
        CheckPair( links + 4, obsts + 2 ), CheckPair( links + 4, obsts + 3 ),
        CheckPair( links + 5, obsts + 0 ), CheckPair( links + 5, obsts + 1 ),
        CheckPair( links + 5, obsts + 2 ), CheckPair( links + 5, obsts + 3 ),
        CheckPair( links + 5, obsts + 4 ), CheckPair( links + 5, obsts + 5 ),
        CheckPair( links + 5, links + 1 ), CheckPair( links + 5, links + 2 ),
        CheckPair( links + 6, links + 4 ),
        CheckPair( groups + 2, obsts + 0 ), CheckPair( groups + 2, obsts + 1 ),
        CheckPair( groups + 2, obsts + 2 ), CheckPair( groups + 2, obsts + 3 ),
        CheckPair( groups + 2, obsts + 4 ), CheckPair( groups + 2, obsts + 5 ),
        CheckPair( groups + 2, links + 1 ), CheckPair( groups + 2, links + 2 ),
        CheckPair( links + 8, links + 4 ),
        CheckPair( groups + 3, obsts + 0 ), CheckPair( groups + 3, obsts + 1 ),
        CheckPair( groups + 3, obsts + 2 ), CheckPair( groups + 3, obsts + 3 ),
        CheckPair( groups + 3, obsts + 4 ), CheckPair( groups + 3, obsts + 5 ),
        CheckPair( groups + 3, links + 1 ), CheckPair( groups + 3, links + 2 ),
        CheckPair( links + 10, links + 4 ), CheckPair( links + 10, links + 8 )
        };

        static const uint16_t list_config_change[ motion_planner::conf_space_dims + 1 ] = {
            0, 2, 6, 10, 18, 19, 28, 38
        };
    }
}
