#include "motion_planner.h"
#include "main/env/env_def.h"

using namespace motion_planner;


void MotionPlanner::findPath( config_space::Point startConfig, 
                              config_space::Point goalConfig )
{
    if ( ! m_mapValidator.isConfigInFreeCSpace( startConfig ) )
    {
        m_searchResult = SearchResult::START_COLLIDED;
        return;
    }

    if ( ! m_mapValidator.isConfigInFreeCSpace( goalConfig ) )
    {
        m_searchResult = SearchResult::GOAL_COLLIDED;
        return;
    }

    setup( startConfig, goalConfig );

    do
    {
        m_planner.expansion();

    } while ( ! m_planner.isConnectionSuccessful() );

    m_pathTraversal.reset();

    auto pointsInPath = m_pathSearcher.findShortestPath();
    m_pathTraversal.setPointsAmount( pointsInPath );

    m_pathTraversal.resetPointGetter();

    m_searchResult = SearchResult::SUCCESS;
}




MotionPlanner::SearchResult MotionPlanner::getSearchResult() const
{
    return m_searchResult;
}




config_space::Point MotionPlanner::getPointTraj() const
{
    return m_pathTraversal.getNextPointPath();
}



void MotionPlanner::resetStepPath() const
{
    m_pathTraversal.resetPointGetter();
}



bool MotionPlanner::isPathPassed() const
{
    return m_pathTraversal.isPathPassed();
}




void MotionPlanner::setup( const config_space::Point & start, const config_space::Point & goal )
{
    m_mapFreeCSpace.reset();

    m_mapFreeCSpace.insertNode( start );
    m_mapFreeCSpace.insertNode( goal );

    m_pathSearcher.reset();

    m_planner.reset();

    m_searchResult = SearchResult::FAILURE;
}

