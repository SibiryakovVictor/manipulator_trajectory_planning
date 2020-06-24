#pragma once

#include "main/env/env_def.h"
#include "main/env/primitives/obb/obb.h"
#include "main/env/primitives/attached_detail/attached_detail.h"
#include "main/env/manipulator/manip_parameters.h"
#include "main/env/obstacles/obst_parameters.h"


namespace motion_planner
{
    namespace env
    {
        struct ManipInitInfo
        {
            struct LinkInitInfo
            {
                Obb body_ = Obb();

                Position mountPos_ = Eigen::Vector3f::Zero();

                uint8_t axisRot_ = 2;

                uint16_t groupId_ = 0;

                uint16_t mountId_ = 0;

                EulerAngles anglesOrient_ = EulerAngles( { 0.f, 0.f, 0.f } );
                bool isAngleSet_[ 3 ] = { false, false, false };
                bool areAnglesUsed_ = false;

                explicit LinkInitInfo( 
                    Obb::Sizes sizes,
                    Position centerPos,
                    Position mountPos,
                    uint16_t mountId,
                    uint8_t axis,
                    uint16_t id,
                    const float (&angles)[ 3 ],
                    const bool (&isAngleSet)[ 3 ],
                    bool areAnglesUsed
                ) :
                    body_( sizes, centerPos ),
                    mountPos_( mountPos ),
                    mountId_( mountId ),
                    axisRot_( axis ),
                    groupId_( id ),
                    areAnglesUsed_( areAnglesUsed ),
                    anglesOrient_( { angles[ 0 ], angles[ 1 ], angles[ 2 ] } )
                {
                    std::copy( isAngleSet, isAngleSet + 3, isAngleSet_ );
                }

                explicit LinkInitInfo()
                {}
            };

            LinkInitInfo linksData[ manip::links_amount ];
        };

        struct ObstInitInfo;
    }
}



struct motion_planner::env::ObstInitInfo
{
    AttachedDetail obsts[ obst::obst_amount ];
};
