#pragma once

#include "main/config_space/graph/graph.h"
#include "main/path/path_traversal/path_traversal.h"
#include "main/path/path_finder/path_finder.h"
#include "main/config_space/free_cspace_verifier/free_cspace_verifier.h"
#include "main/est/est.h"
#include "main/env/manipulator/manipulator.h"
#include "main/env/obstacles/obstacles.h"

#include "main/tools/config_manager/config_manager.h"


namespace motion_planner
{
    class MotionPlanner;
}


class motion_planner::MotionPlanner
{
public:

    enum class SearchResult { SUCCESS, FAILURE, START_COLLIDED, GOAL_COLLIDED };

    ConfigManager _config = ConfigManager( m_manipulator, m_obstacles, m_mapValidator );

    explicit MotionPlanner( float step ) :
        m_pathTraversal( m_mapFreeCSpace, step )
    {}

    void findPath( config_space::Point startConfig, 
                   config_space::Point goalConfig );

    SearchResult getSearchResult() const;

    bool isPathPassed() const;

    config_space::Point getPointTraj() const;

    void resetStepPath() const;

private:

    env::Manipulator m_manipulator;
    env::Obstacles m_obstacles;

    config_space::graph::Graph m_mapFreeCSpace;

    path::PathTraversal m_pathTraversal;

    path::ShortestPathSearcher m_pathSearcher = path::ShortestPathSearcher( 
        m_pathTraversal.getPathStorage(), m_mapFreeCSpace, m_mapFreeCSpace );

    config_space::FreeCSpaceVerifier m_mapValidator = 
        config_space::FreeCSpaceVerifier( m_manipulator, m_obstacles );

    Est m_planner = Est( m_mapFreeCSpace, m_mapValidator );

    SearchResult m_searchResult = SearchResult::FAILURE;

    uint32_t m_timeSolving_mSec = 0;

    void setup( const config_space::Point & start, const config_space::Point & goal );

};

