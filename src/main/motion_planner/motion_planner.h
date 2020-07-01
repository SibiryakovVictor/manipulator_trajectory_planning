/**************************************************************************************************
Описание

Класс, реализующий планирование траектории движения манипулятора

Разработчик: Сибиряков Виктор
Заметки
* конструктор по умолчанию создаёт планировщик с точностью траектории, заданной в "path/path_parameters.h"
* получение результата о планировании - getSearchResult, isPathFound
* !!! ВАЖНО !!!: после вызова методов 'changeTraj...' при желании использовать траекторию для тех же
старта и цели ОБЯЗАТЕЛЬНО повторить для них поиск траектории через findPath
* константными являются все методы, которые не меняют найденную траекторию
**************************************************************************************************/


#pragma once

#include "main/config_space/graph/graph.h"
#include "main/path/path_generator/path_generator.h"
#include "main/path/path_searcher/path_searcher.h"
#include "main/config_space/free_cspace_validator/free_cspace_validator.h"
#include "main/est/est.h"
#include "main/env/manipulator/manipulator_controller.h"
#include "main/env/obstacles/obstacles.h"

#include "main/tools/configurator/configurator.h"


namespace motion_planner
{
    class MotionPlanner;
}


class motion_planner::MotionPlanner
{
public:

    enum class SearchResult { SUCCESS, 
                              LIMIT_NODES_REACHED,
                              START_COLLIDED,
                              GOAL_COLLIDED,
                              TRAJ_PREC_CHANGED };


    Configurator settings = Configurator( m_manip, m_obst, m_mapValidator, m_planner );

    explicit MotionPlanner()
    {}

    explicit MotionPlanner( float trajPrec_Degree ) :
        m_pathGenerator( m_mapFreeCSpace, trajPrec_Degree ),
        m_mapValidator( m_manip, m_obst, trajPrec_Degree / path::ratio_traj_prec_check )
    {}

    void findPath( config_space::Point startConfig, config_space::Point goalConfig );


    bool isPathFound() const;
    SearchResult getSearchResult() const;

    bool isPathPassed() const;

    config_space::Point getPointTraj() const;

    void resetPointGetter() const;

    void changeTrajStep( float trajStep_Degree );
    float getTrajStep_Degree() const;

    void changeTrajCheckStep( float checkTrajStep_Degree );
    float getTrajCheckStep_Degree() const;


private:

    env::ManipulatorController m_manip;
    env::Obstacles m_obst;

    config_space::graph::Graph m_mapFreeCSpace;

    path::PathGenerator m_pathGenerator = 
        path::PathGenerator( m_mapFreeCSpace, path::traj_prec_Degrees );

    path::ShortestPathSearcher m_pathSearcher = path::ShortestPathSearcher( 
        m_pathGenerator.getPathStorage(), m_mapFreeCSpace, m_mapFreeCSpace );

    config_space::FreeCSpaceValidator m_mapValidator = 
        config_space::FreeCSpaceValidator( m_manip, m_obst );

    Est m_planner = Est( m_mapFreeCSpace, m_mapValidator );

    SearchResult m_searchResult = SearchResult::LIMIT_NODES_REACHED;


    void setup( const config_space::Point & start, const config_space::Point & goal );
};
