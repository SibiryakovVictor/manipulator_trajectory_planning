#include "main/config_space/point/point.h"


namespace motion_planner
{
    
    struct TaskPlanning
    {
        config_space::Point start;
        config_space::Point goal;
        
        explicit TaskPlanning( config_space::Point s, config_space::Point g ) :
            start( s ),
            goal( g )
        {}
    };
    
}
