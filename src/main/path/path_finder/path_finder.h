#pragma once


#include "main/config_space/graph/shortest_path_finder/a_star/a_star.h"


namespace motion_planner
{
    namespace path
    {
        class ShortestPathSearcher;
    }
}


class motion_planner::path::ShortestPathSearcher
{
public:

    explicit ShortestPathSearcher( path::Path & pathSections,
                                   const config_space::graph::INodesList & nodesList,
                                   const config_space::graph::IEdgesList & edgesList ) :
        m_pathSections( pathSections ),
        m_nodes( nodesList ),
        m_edges( edgesList )
    {}

    uint16_t findShortestPath();

    float calcSumPathLength() const;

    float getRefPathLength() const;

    void reset();

private:

    config_space::graph::Astar m_searchAlgorithm;

    path::Path & m_pathSections; 

    const config_space::graph::INodesList & m_nodes;
    const config_space::graph::IEdgesList & m_edges;

    float m_refPathLength = 0.f;

};

