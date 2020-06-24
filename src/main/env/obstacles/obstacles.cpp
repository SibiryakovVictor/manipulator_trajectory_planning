#include "obstacles.h"

using namespace motion_planner::env;
using namespace motion_planner;


const Obb * Obstacles::begin() const
{
    return m_obstInfo.obsts;
}




const Obb * Obstacles::end() const
{
    return m_obstInfo.obsts + obst::obst_amount;
}





const Obb & Obstacles::getBody( uint16_t obstBiasedId ) const
{
    return m_obstInfo.obsts[ obstBiasedId - biasObstId ];
}




const Position & Obstacles::getObstPos( uint16_t obstId ) const
{
    return m_obstInfo.obsts[ obstId ].getCenterPos();
}




const Orient & Obstacles::getObstOrient( uint16_t obstId ) const
{
    return m_obstInfo.obsts[ obstId ].getOrient();
}




const Obb::Sizes & Obstacles::getObstSizes( uint16_t obstId ) const
{
    return m_obstInfo.obsts[ obstId ].getSizes();
}




void Obstacles::changeBody( uint16_t obstBiasedId, const env::Obb & body )
{
    m_obstInfo.obsts[ obstBiasedId - biasObstId ] = body;
}



void Obstacles::rotate( uint16_t obstBiasedId, float angle, Eigen::Vector3f axis )
{
    m_obstInfo.obsts[ obstBiasedId - biasObstId ].rotateAroundAnchor( angle, axis );
}
