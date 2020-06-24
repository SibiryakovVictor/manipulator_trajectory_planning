#pragma once

#include "main/env/env_def.h"

namespace motion_planner
{
    namespace env
    {
        uint8_t cvtAxisToId( Axis axis );

        const Eigen::Vector3f & getUnitAxis( Axis axis );
        const Eigen::Vector3f & getUnitAxis( uint8_t axis );


        Eigen::Matrix3f getRotMat( const float & angleRad, const Eigen::Vector3f & rotAxis );
        Eigen::Matrix3f getRotMat( const float & angleRad, Eigen::Vector3f && rotAxis );


        void rotateVector( Position & pos, const float & angleRad, 
            const Eigen::Vector3f & rotAxis );
        Eigen::Vector3f rotateVector( const Position & pos, const float & angleRad, 
            const Eigen::Vector3f & rotAxis );
        Eigen::Vector3f rotateVector( Position && pos, const float & angleRad, 
            const Eigen::Vector3f & rotAxis );


        void rotateOrient( Orient & orient, const float & angleRad,
            const Eigen::Vector3f & rotAxis );
        Orient rotateOrient( const env::Orient & orient, 
            const float & angleRad, const Eigen::Vector3f & rotAxis );
        Orient rotateOrient( env::Orient && orient, 
            const float & angleRad, const Eigen::Vector3f & rotAxis );
    }
}

