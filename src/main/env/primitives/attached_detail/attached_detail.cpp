#include "attached_detail.h"
#include "main/env/motion_common/motion_common.h"


using namespace motion_planner::env;




void AttachedDetail::rotateAroundAnchor( float angle, Eigen::Vector3f && rotAxis )
{
    auto && rotMat = getRotMat( angle, rotAxis );

    Obb::setCenterPos( m_mountPos + rotMat * ( m_initCenterPos - m_mountPos ) );

    Obb::setOrient( rotMat );
}

void AttachedDetail::rotateAroundAnchor( float angle, const Eigen::Vector3f & rotAxis )
{
    auto && rotMat = getRotMat( angle, rotAxis );

    Obb::setCenterPos( m_mountPos + rotMat * ( m_initCenterPos - m_mountPos ) );

    Obb::setOrient( rotMat );
}




void AttachedDetail::resetPosition()
{
    Obb::setCenterPos( m_initCenterPos );

    Obb::setOrient( Eigen::Matrix3f::Identity() );
}



AttachedDetail & AttachedDetail::operator=( const Obb & obbObj )
{
    Obb::operator=( obbObj );

    return *this;
}



AttachedDetail::AttachedDetail( const Obb & obbObj ) :
    Obb( obbObj )
{}

AttachedDetail::AttachedDetail( Obb && obbObj ) :
    Obb( obbObj )
{}

AttachedDetail::AttachedDetail( const Obb & obbObj, const Position & mountPos ) :
    Obb( obbObj ), m_mountPos( mountPos )
{}

AttachedDetail::AttachedDetail( Obb && obbObj, Position && mountPos ) :
    Obb( obbObj ), m_mountPos( mountPos )
{}




