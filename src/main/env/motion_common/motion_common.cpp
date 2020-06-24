#include "motion_common.h"

using namespace motion_planner::env;



const Eigen::Vector3f & motion_planner::env::getUnitAxis( Axis axis )
{

    static const Eigen::Vector3f vuX = Eigen::Vector3f::UnitX();
    static const Eigen::Vector3f vuY = Eigen::Vector3f::UnitY();
    static const Eigen::Vector3f vuZ = Eigen::Vector3f::UnitZ();

    switch ( axis )
    {

    case ( Axis::X ) :
    {
        return vuX;
    }
    case ( Axis::Y ) :
    {
        return vuY;
    }
    default :
    {
        return vuZ;
    }

    }

}



const Eigen::Vector3f & motion_planner::env::getUnitAxis( uint8_t axis )
{

    static const Eigen::Vector3f unitX = Eigen::Vector3f::UnitX();
    static const Eigen::Vector3f unitY = Eigen::Vector3f::UnitY();
    static const Eigen::Vector3f unitZ = Eigen::Vector3f::UnitZ();

    switch ( axis )
    {

    case ( 0 ) :
    {
        return unitX;
    }
    case ( 1 ) :
    {
        return unitY;
    }
    default :
    {
        return unitZ;
    }

    }

}



uint8_t motion_planner::env::cvtAxisToId( Axis axis )
{

    switch ( axis )
    {

    case ( Axis::X ) :
    {
        return 0;
    }
    case ( Axis::Y ) :
    {
        return 1;
    }
    default :
    {
        return 2;
    }

    }

}



Eigen::Matrix3f motion_planner::env::getRotMat( const float & angleRad, const Eigen::Vector3f & rotAxis )
{
    return Eigen::AngleAxis< float >( angleRad, rotAxis ).toRotationMatrix();
}
Eigen::Matrix3f motion_planner::env::getRotMat( const float & angleRad, Eigen::Vector3f && rotAxis )
{
    return Eigen::AngleAxis< float >( angleRad, rotAxis ).toRotationMatrix();
}




void motion_planner::env::rotateVector( Position & pos, const float & angleRad, 
    const Eigen::Vector3f & rotAxis )
{
    pos = getRotMat( angleRad, rotAxis ) * pos;
}
Eigen::Vector3f motion_planner::env::rotateVector( const Position & pos, const float & angleRad, 
    const Eigen::Vector3f & rotAxis )
{
    return getRotMat( angleRad, rotAxis ) * pos;
}
Eigen::Vector3f motion_planner::env::rotateVector( Position && pos, const float & angleRad, 
    const Eigen::Vector3f & rotAxis )
{
    return getRotMat( angleRad, rotAxis ) * pos;
}



void motion_planner::env::rotateOrient( Orient & orient, const float & angleRad,
    const Eigen::Vector3f & rotAxis )
{
    orient = getRotMat( angleRad, rotAxis ) * orient;
}
Orient motion_planner::env::rotateOrient( const Orient & orient, const float & angleRad, 
    const Eigen::Vector3f & rotAxis )
{
    return getRotMat( angleRad, rotAxis ) * orient;
}
Orient motion_planner::env::rotateOrient( Orient && orient, const float & angleRad, 
    const Eigen::Vector3f & rotAxis )
{
    return getRotMat( angleRad, rotAxis ) * orient;
}