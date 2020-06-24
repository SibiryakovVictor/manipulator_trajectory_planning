#pragma once

#include "collision_check_pairs.h"



namespace motion_planner
{
    namespace env
    {
        class CollisionTable;
    }
}


class motion_planner::env::CollisionTable
{
public:

    explicit CollisionTable()
    {
        fillTable();

        std::fill( m_isPairActivated, m_isPairActivated + coll::pairsAmount, true );
    }

    void activatePair( uint16_t pairIndex );

    void deactivatePair( uint16_t pairIndex );

    coll::CheckPair getPair( uint16_t pairIndex ) const;

    bool isPairActivated( uint16_t pairIndex ) const;


private:

    coll::CheckPair m_table[ coll::pairsAmount ];

    bool m_isPairActivated[ coll::pairsAmount ] { true };

    void fillTable();

};

