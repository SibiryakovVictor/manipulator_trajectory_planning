#pragma once

#include <cmath>


#include "main/tools/float_operations/float_operations.h"
#include "main/config_space/conf_space_dims.h"


namespace motion_planner
{
    namespace path
    {
        const uint16_t length_limit = 60;

        const float req_prec = 1.f;    

        const float traj_prec = 
            std::sqrtf( std::powf( flt_op::cvtDegToRad( req_prec ), 2.f ) / float( conf_space_dims ) );
        const float check_prec = traj_prec / 2.f;
    }
}

