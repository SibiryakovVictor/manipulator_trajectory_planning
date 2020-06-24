#include "free_cspace_verifier.h"
#include "main/tools/float_operations/float_operations.h"
#include "main/path/path_parameters.h"

#include <cfloat>

using namespace motion_planner::config_space;
using namespace motion_planner::path;
using namespace motion_planner::env;
using namespace motion_planner::coll;




bool FreeCSpaceVerifier::isSegmentInFreeCSpace( const Point & start, const Point & end )
{
    Segment segment( start, end );

    auto resultRough = checkSegmentRoughPhase( segment );

    if ( resultRough == 2 )
    {
        return false;
    }

    if ( resultRough == 0 )
    {
        return true;
    }

    return 
        ( ! isSegmentCollided_Precision( segment, segment.calcParamStep( path::check_prec ) ) );
}




uint8_t FreeCSpaceVerifier::checkSegmentRoughPhase( const Segment & segment )
{
    static const auto stages = 2;
    static const unsigned amount_points[ stages ] = { 4, 15 };
    static const auto seg_threshold_check = path::check_prec * 0.5f;

    for ( auto stage = 0; stage != stages; stage++ )
    {
        if ( isSegmentCollided_Points( segment, amount_points[ stage ] ) )
        {
            return 2;
        }

        if ( ( segment.getMaxInterval() / amount_points[ stage ] ) < seg_threshold_check )
        {
            return 0;
        }
    }

    return 1;
}



bool FreeCSpaceVerifier::isSegmentCollided_Points( const Segment & segment, const float pointsCheck )
{
    return isSegmentCollided_Precision( segment, 1.f / pointsCheck );
}



bool FreeCSpaceVerifier::isSegmentCollided_Precision( const Segment & segment, const float prec )
{
    auto step = prec;

    while ( step < 1.f )
    {
        auto point( segment.getPoint( step ) );

        if ( ! isConfigInFreeCSpace( point ) )
        {
            return true;
        }

        step += prec;
    }

    return false;
}




bool FreeCSpaceVerifier::isConfigInFreeCSpace( const Point & conf )
{
    static const auto threshold_config_change = path::check_prec * 0.5f;

    static config_space::Point prevConfig( config_space::PointData( FLT_MAX ) );
    static bool isPrevResultCorrect = false;

    m_manip.reset();

    uint8_t configPos = 0;

    uint16_t manip = 0;

    uint16_t pairPos = 0;

    auto isDiffFromPrev = false;

    while ( configPos < conf_space_dims )
    {
        m_manip.setConfigCurrentLink( conf[ configPos ] );

        if ( isPrevResultCorrect && ( ! isDiffFromPrev ) )
        {
            if ( std::fabsf( prevConfig[ configPos ] - conf[ configPos ] ) < 
                threshold_config_change )
            {
                pairPos = moveToNextGroup( pairPos, configPos );

                configPos++;

                m_manip.moveGroupCounter();

                continue;
            }
            else
            {
                isDiffFromPrev = true;
            }
        }

        configPos++;

        while ( pairPos != list_config_change[ configPos ] )
        {
            if ( ! m_collTable.isPairActivated( pairPos ) )
            {
                pairPos++;

                continue;
            }

            auto pair = m_collTable.getPair( pairPos );

            bool checkResult = true;
            if ( pair.body2 >= env::Obstacles::biasObstId )
            {
                checkResult = checkManipVsObst( pair.body1, pair.body2 );
            }
            else
            {
                checkResult = checkManipVsManip( pair.body1, pair.body2 );
            }


            if ( ! checkResult )
            {
                isPrevResultCorrect = false;
                return false;
            }

            pairPos++;
        }

        m_manip.moveGroupCounter();
    }

    prevConfig = conf;
    isPrevResultCorrect = true;
    return true;
}




bool FreeCSpaceVerifier::checkManipVsManip( uint16_t id1, uint16_t id2 ) const
{
    const auto & body1 = m_manip.getBody( id1 );
    const auto & body2 = m_manip.getBody( id2 );

    if ( ! m_collDetect.areObjectsCollided( body1, body2 ) )
    {
        return true;
    }

    bool isId1Link = id1 < env::Manipulator::biasGroupId;
    bool isId2Link = id2 < env::Manipulator::biasGroupId;

    if ( isId1Link && isId2Link )
    {
        return false;
    }

    if ( ( ! isId1Link ) && isId2Link )
    {
        return checkGroupVsLink( id1, id2 );
    }

    if ( isId1Link && ( ! isId2Link ) )
    {
        return checkGroupVsLink( id2, id1 );
    }

    return checkGroupVsGroup( id1, id2 );
}




bool FreeCSpaceVerifier::checkManipVsObst( uint16_t manipId, uint16_t obstId ) const
{
    const auto & body1 = m_manip.getBody( manipId );
    const auto & body2 = m_obst.getBody( obstId );

    if ( ! m_collDetect.areObjectsCollided( body1, body2 ) )
    {
        return true;
    }

    bool isLink = manipId < env::Manipulator::biasGroupId;

    if ( isLink )
    {
        return false;
    }

    return checkGroupVsObst( manipId, obstId );
}



bool FreeCSpaceVerifier::checkGroupVsLink( uint16_t groupId, uint16_t linkId ) const
{
    auto firstLinkGroup = m_manip.getFirstLinkComplex( groupId );
    auto linkNextGroup = firstLinkGroup + m_manip.getLinkType( firstLinkGroup );

    const auto & linkBody = m_manip.getBody( linkId ); 

    for ( auto link = firstLinkGroup; link != linkNextGroup; link++ )
    {
        if ( m_collDetect.areObjectsCollided( m_manip.getBody( link + m_manip.biasLinkId ), linkBody ) )
        {
            return false;
        }
    }

    return true;
}




bool FreeCSpaceVerifier::checkGroupVsGroup( uint16_t groupId1, uint16_t groupId2 ) const
{
    auto firstLinkGroup1 = m_manip.getFirstLinkComplex( groupId1 );
    auto firstLinkGroup2 = m_manip.getFirstLinkComplex( groupId2 );

    auto linkNextGroup1 = firstLinkGroup1 + m_manip.getLinkType( firstLinkGroup1 );
    auto linkNextGroup2 = firstLinkGroup2 + m_manip.getLinkType( firstLinkGroup2 );

    for ( auto links1 = firstLinkGroup1; links1 != linkNextGroup1; links1++ )
    {
        const auto & body1 = m_manip.getBody( links1 + m_manip.biasLinkId );

        for ( auto links2 = firstLinkGroup2; links2 != linkNextGroup2; links2++ )
        {
            const auto & body2 = m_manip.getBody( links2 + m_manip.biasLinkId );

            if ( m_collDetect.areObjectsCollided( body1, body2 ) )
            {
                return false;
            }
        }
    }

    return true;
}



bool FreeCSpaceVerifier::checkGroupVsObst( uint16_t groupId, uint16_t obstId ) const
{
    static const auto biasLinkId = m_manip.biasLinkId;

    auto firstLinkGroup = m_manip.getFirstLinkComplex( groupId );
    auto linkNextGroup = firstLinkGroup + m_manip.getLinkType( firstLinkGroup );

    const auto & obstBody = m_obst.getBody( obstId );

    for ( auto link = firstLinkGroup; link != linkNextGroup; link++ )
    {
        if ( m_collDetect.areObjectsCollided( 
            m_manip.getBody( link + m_manip.biasLinkId ), obstBody ) )
        {
            return false;
        }
    }

    return true;
}



uint16_t FreeCSpaceVerifier::moveToNextGroup( uint16_t pairPos, uint16_t confPos ) const
{
    confPos++;

    if ( confPos == conf_space_dims )
    {
        return pairsAmount;
    }

    return list_config_change[ confPos ];
}

