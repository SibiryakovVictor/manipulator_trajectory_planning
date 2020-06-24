#include "est.h"

#include "main/tools/float_operations/float_operations.h"


using namespace motion_planner::config_space::graph;
using namespace motion_planner;



bool Est::isConnectionSuccessful()
{
    bool isStartListChanged = ( m_startCheckPos != m_posListEndStart );
    bool isGoalListChanged = ( m_goalCheckPos != m_posListEndGoal );

    if ( ( ! isStartListChanged ) && ( ! isGoalListChanged ) )
    {
        return false;
    }

    if ( isGoalListChanged && 
        areTreesConnected( 0, m_posListEndStart, m_goalCheckPos, m_posListEndGoal ) )
    {
        m_goalCheckPos = m_posListEndGoal;
        m_startCheckPos = m_posListEndStart;

        return true;
    }

    if ( isStartListChanged &&
        areTreesConnected( m_startCheckPos, m_posListEndStart, 0, m_posListEndGoal ) )
    {
        m_goalCheckPos = m_posListEndGoal;
        m_startCheckPos = m_posListEndStart;

        return true;
    }
   
    m_goalCheckPos = m_posListEndGoal;
    m_startCheckPos = m_posListEndStart;

    return false;
}



void Est::expansion()
{
    const auto nodeListEnd = getListEnd();

    for ( NodeId pos = 0; pos != nodeListEnd; pos++ )
    {
        if ( calcProbabilityNode( m_listCompStatus[ m_treeId ][ pos ] ) 
            < flt_op::getRandomFloat( 0.005f, 0.995f ) )
        {
            continue;
        }

        sampleNodesInArea( m_listCompStatus[ m_treeId ][ pos ] );
    }

    changeGrowingTree();
}




void Est::sampleNodesInArea( NodeId refNodeId )
{
    const auto & refConfig = m_mapFreeCSpace.getNodeConfig( refNodeId );

    for ( uint16_t sample = 0; sample != samples_amount; sample++ )
    {
        if ( ! m_mapFreeCSpace.hasNodeFreeEdge( refNodeId ) )
        {
            break;
        }

        auto genConfig( generateNearConfig( refConfig ) );

        tryConnectSample( genConfig, refNodeId, refConfig );
    }
}




float Est::calcProbabilityNode( NodeId node ) const
{
    return 1.f / static_cast< float >( m_mapFreeCSpace.getNodesInArea( node ) );
}




void Est::changeGrowingTree()
{
    if ( m_treeId == start_comp )
    {
        m_treeId = goal_comp;
    }
    else
    {
        m_treeId = start_comp;
    }
}




config_space::Point Est::generateNearConfig( const config_space::Point & refConfig ) const
{
    config_space::Point genConfig;

    do
    {
        genConfig = m_generator.generate();

    } while ( config_space::Point::calcDistNoSqrt( genConfig, refConfig ) > m_radiusGen );

    return genConfig;
}




float Est::calcProbabilityGenConfig( const config_space::Point & config )
{
    static const float threshold_low = 0.001f;

    const auto listEnd = getListEnd();

    uint16_t amountNn = 1;

    for ( uint16_t listPos = 0; listPos != listEnd; listPos++ )
    {
        const auto nodePos = m_listCompStatus[ m_treeId ][ listPos ];

        auto distance = config_space::Point::calcDistNoSqrt( 
            config, m_mapFreeCSpace.getNodeConfig( nodePos ) );

        if ( ( distance < m_radiusGen ) && ( distance > threshold_low ) )
        {
            amountNn++;
        }
    }

    return 1.f / static_cast< float >( amountNn );
}




uint16_t & Est::getListEnd()
{
    if ( m_treeId == start_comp )
    {
        return m_posListEndStart;
    }

    return m_posListEndGoal;
}




void Est::addToList( NodeId nodePos )
{
    auto & listEnd = getListEnd();

    m_listCompStatus[ m_treeId ][ listEnd ] = nodePos; 

    listEnd++;
}




void Est::reset()
{
    m_posConnectGoal = goal_node_pos;

    m_posListEndStart = 1;
    m_posListEndGoal = 1;

    m_startCheckPos = 0;
    m_goalCheckPos = 0;

    m_treeId = start_comp;

    m_listCompStatus[ start_comp ][ 0 ] = 0;
    m_listCompStatus[ goal_comp ][ 0 ] = 1;
}




void Est::tryConnectSample( const config_space::Point & sampleConf, NodeId refNodeId,
                            const config_space::Point & refConfig )
{
    static const float threshold_low = 0.001f;

    const auto listEnd = getListEnd();

    uint16_t amountNn = 0;

    uint16_t listNn[ capacity / 5 ] { 0 };
    uint16_t listNnEnd = 0;

    for ( uint16_t listPos = 0; listPos != listEnd; listPos++ )
    {
        if ( listNnEnd == ( capacity / 5 ) )
        {
            break;
        }

        const auto nodePos = m_listCompStatus[ m_treeId ][ listPos ];

        auto distance = config_space::Point::calcDistNoSqrt( 
            sampleConf, m_mapFreeCSpace.getNodeConfig( nodePos ) );

        if ( ( distance < m_radiusGen ) && ( distance > threshold_low ) )
        {
            amountNn++;

            listNn[ listNnEnd ] = nodePos;

            listNnEnd++;
        }
    }

    const auto probabilityToConnect = 1.f / static_cast< float >( amountNn );

    if ( probabilityToConnect < flt_op::getRandomFloat( 0.005f, 0.995f ) )
    {
        return;
    }

    if ( ( ! m_mapValidator.isConfigInFreeCSpace( sampleConf ) ) ||
        ( ! m_mapValidator.isSegmentInFreeCSpace( refConfig, sampleConf ) ) )
    {
        return;
    }

    auto nodePos = m_mapFreeCSpace.insertNode( sampleConf );
    m_mapFreeCSpace.addEdge( nodePos, refNodeId );
    m_mapFreeCSpace.setNodesInArea( nodePos, amountNn );

    addToList( nodePos );

    for ( uint16_t listNnPos = 0; listNnPos != listNnEnd; listNnPos++ )
    {
        m_mapFreeCSpace.increaseNodesInArea( listNn[ listNnPos ] );
    }
}




bool Est::areTreesConnected( uint16_t listStartBegin, uint16_t listStartEnd,
                             uint16_t listGoalBegin, uint16_t listGoalEnd ) const
{
    for ( uint16_t startListPos = listStartBegin; startListPos != listStartEnd; startListPos++ )
    {
        const auto nodePosStart = m_listCompStatus[ start_comp ][ startListPos ];

        if ( ! m_mapFreeCSpace.hasNodeFreeEdge( nodePosStart ) )
        {
            continue;
        }

        const auto & confPosStart = m_mapFreeCSpace.getNodeConfig( nodePosStart );

        for ( uint16_t goalListPos = listGoalBegin; goalListPos != listGoalEnd; goalListPos++ )
        {
            const auto nodePosGoal = m_listCompStatus[ goal_comp ][ goalListPos ];
            
            if ( ! m_mapFreeCSpace.hasNodeFreeEdge( nodePosStart ) )
            {
                break;
            }

            if ( ! m_mapFreeCSpace.hasNodeFreeEdge( nodePosGoal ) )
            {
                continue;
            }

            const auto & confPosGoal = m_mapFreeCSpace.getNodeConfig( nodePosGoal );

            if ( config_space::Point::calcDistNoSqrt( confPosStart, confPosGoal ) > m_radiusConn )
            {
                continue;
            }

            if ( ! m_mapValidator.isSegmentInFreeCSpace( confPosStart, confPosGoal ) )
            {
                continue;
            }

            m_mapFreeCSpace.addEdge( nodePosStart, nodePosGoal );

            return true;
        }
    }

    return false;
}

