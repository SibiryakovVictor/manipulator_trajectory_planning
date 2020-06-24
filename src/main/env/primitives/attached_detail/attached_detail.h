#pragma once

#include "main/env/primitives/obb/obb.h"


namespace motion_planner
{
    namespace env
    {
        class AttachedDetail;
    }
}


class motion_planner::env::AttachedDetail : public motion_planner::env::Obb
{

public:

    void rotateAroundAnchor( float angle, const Eigen::Vector3f & rotAxis );
    void rotateAroundAnchor( float angle, Eigen::Vector3f && rotAxis );

    void resetPosition();

    explicit AttachedDetail()
    {}

    explicit AttachedDetail( const Obb & obbObj );
    explicit AttachedDetail( Obb && obbObj );

    explicit AttachedDetail( const Obb & obbObj, const Position & mountPos );
    explicit AttachedDetail( Obb && obbObj, Position && mountPos );

    AttachedDetail & operator=( const Obb & obbObj );

private:

    Position m_mountPos = Obb::getCenterPos();

    Position m_initCenterPos = Obb::getCenterPos();

};


