#include "collision_table.h"

using namespace motion_planner::env;
using namespace motion_planner::coll;


void CollisionTable::fillTable()
{
    std::copy( coll::list_check_pairs, coll::list_check_pairs + coll::pairsAmount, m_table );
}



void CollisionTable::activatePair( uint16_t pairIndex )
{
    m_isPairActivated[ pairIndex ] = true;
}



void CollisionTable::deactivatePair( uint16_t pairIndex )
{
    m_isPairActivated[ pairIndex ] = false;
}



CheckPair CollisionTable::getPair( uint16_t pairIndex ) const
{
    return m_table[ pairIndex ];
}



bool CollisionTable::isPairActivated( uint16_t pairIndex ) const
{
    return m_isPairActivated[ pairIndex ];
}

