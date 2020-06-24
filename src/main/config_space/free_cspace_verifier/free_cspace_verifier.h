#pragma once


#include "main/env/manipulator/manipulator.h"
#include "main/env/obstacles/obstacles.h"
#include "main/env/collision_detection/collision_detector.h"
#include "main/env/collision_detection/collision_check_pairs.h"
#include "main/env/collision_detection/collision_table.h"
#include "main/config_space/segment/segment.h"



namespace motion_planner
{
    class ConfigManager;

    namespace config_space
    {
        class FreeCSpaceVerifier;
    }
}


class motion_planner::config_space::FreeCSpaceVerifier
{
public:

    friend class motion_planner::ConfigManager;

    explicit FreeCSpaceVerifier( env::Manipulator & manip, env::Obstacles & obst ) :
        m_manip( manip ),
        m_obst( obst )
    {}

    bool isConfigInFreeCSpace( const config_space::Point & config ); 

    bool isSegmentInFreeCSpace( const config_space::Point & start, const config_space::Point & end );


private:

    env::Manipulator & m_manip;

    env::Obstacles & m_obst;

    coll::CollisionDetector m_collDetect;

    env::CollisionTable m_collTable;


    bool isSegmentCollided_Precision( const config_space::Segment & segment, const float prec );
    bool isSegmentCollided_Points( const config_space::Segment & segment, const float pointsCheck );


    uint8_t checkSegmentRoughPhase( const config_space::Segment & segment );


    bool checkManipVsManip( uint16_t id1, uint16_t id2 ) const;
    bool checkManipVsObst( uint16_t manipId, uint16_t obstId ) const;

    bool checkGroupVsLink( uint16_t groupId, uint16_t linkId ) const;
    bool checkGroupVsGroup( uint16_t groupId1, uint16_t groupId2 ) const;
    bool checkGroupVsObst( uint16_t groupId, uint16_t obstId ) const;

    uint16_t moveToNextGroup( uint16_t pairPos, uint16_t confPos ) const;

};

