#include "path_finder.h"

using namespace motion_planner::path;
using namespace motion_planner;



uint16_t ShortestPathSearcher::findShortestPath()
{
    return m_searchAlgorithm.findShortestPath( m_pathSections, m_nodes, m_edges );
}



float ShortestPathSearcher::calcSumPathLength() const
{
    auto sumLength = 0.f;

    for ( uint16_t curElem = 0; m_pathSections.sections[ curElem ].segment != UINT8_MAX; curElem++ )
    {
        sumLength += config_space::Point::calcDistance( 
            m_nodes.getNodeConfig( m_pathSections.sections[ curElem ].point ),
            m_nodes.getNodeConfig( m_pathSections.sections[ curElem + 1 ].point ) );
    }

    return sumLength;
}



void ShortestPathSearcher::reset()
{
    m_refPathLength = config_space::Point::calcDistance( 
        m_nodes.getNodeConfig( config_space::graph::start_node_pos ),
        m_nodes.getNodeConfig( config_space::graph::goal_node_pos ) );
}



float ShortestPathSearcher::getRefPathLength() const
{
    return m_refPathLength;
}

