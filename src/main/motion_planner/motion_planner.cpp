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


#include "motion_planner.h"
#include "main/env/env_def.h"

using namespace motion_planner;



/**************************************************************************************************
Описание:
осуществляет поиск траектории для заданных стартовой и целевой конфигураций
Аргументы:
* startConfig: стартовая конфигурация
* goalConfig: целевая конфигурация
Возврат:
**************************************************************************************************/
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

        m_planner.connection();

    } while ( ( ! m_planner.areTreesConnected() ) && ( ! m_mapFreeCSpace.isLimitNodesReached() ) );


    if ( m_mapFreeCSpace.isLimitNodesReached() )
    {
        m_searchResult = SearchResult::LIMIT_NODES_REACHED;

        return;
    }


    m_pathGenerator.reset();

    auto pointsInPath = m_pathSearcher.findShortestPath();
    m_pathGenerator.setPointsAmount( pointsInPath );

    m_pathGenerator.resetPointGetter();

    m_searchResult = SearchResult::SUCCESS;
}




void MotionPlanner::setup( const config_space::Point & start, const config_space::Point & goal )
{
    m_mapFreeCSpace.reset();

    m_mapFreeCSpace.addNode( start );
    m_mapFreeCSpace.addNode( goal );

    m_pathSearcher.reset();

    m_planner.reset();

    m_searchResult = SearchResult::LIMIT_NODES_REACHED;
}



/**************************************************************************************************
Описание:
проверяет, был ли найден путь с последнего вызова findPath
Аргументы:
Возврат: результат о том, был ли найден путь
**************************************************************************************************/
bool MotionPlanner::isPathFound() const
{
    return m_searchResult == SearchResult::SUCCESS;
}



/**************************************************************************************************
Описание:
возвращает статус последнего вызова метода findPath 
Возврат: статус последнего вызова метода findPath 
**************************************************************************************************/
MotionPlanner::SearchResult MotionPlanner::getSearchResult() const
{
    return m_searchResult;
}



/**************************************************************************************************
Описание:
возвращает следующую точку траектории, начиная со стартовой и заканчивая целевой
Аргументы:
Возврат: конфигурация как точка траектории
**************************************************************************************************/
config_space::Point MotionPlanner::getPointTraj() const
{
    return m_pathGenerator.getNextPointPath();
}



/**************************************************************************************************
Описание:
сбрасывает состояние генератора пути, в результате метод getPointTraj начнёт 
выдавать точки траектории с начала
Аргументы:
Возврат:
**************************************************************************************************/
void MotionPlanner::resetPointGetter() const
{
    m_pathGenerator.resetPointGetter();
}



/**************************************************************************************************
Описание:
проверяет, была ли достигнута последняя точка построенной траектории в результате вызовов
метода getPointTraj
Аргументы:
Возврат: результат проверки на достижение последней точки построенной траектории
**************************************************************************************************/
bool MotionPlanner::isPathPassed() const
{
    return m_pathGenerator.isPathPassed();
}



/**************************************************************************************************
Описание:
изменяет шаг траектории
Аргументы:
* trajStep_Degree: максимальное евклидово расстояние между точками траектории
Возврат:
**************************************************************************************************/
void MotionPlanner::changeTrajStep( float trajStep_Degree )
{
    m_pathGenerator.changeStep( trajStep_Degree );

    m_mapValidator.setSegCheckPrec( trajStep_Degree / path::ratio_traj_prec_check );

    m_searchResult = SearchResult::TRAJ_PREC_CHANGED;
}



/**************************************************************************************************
Описание:
изменяет шаг проверки траектории на наличие пересечений
Аргументы:
* checkTrajStep_Degree: максимальное евклидово расстояние между проверяемыми точками траектории
Возврат:
**************************************************************************************************/
void MotionPlanner::changeTrajCheckStep( float checkTrajStep_Degree )
{
    m_mapValidator.setSegCheckPrec( checkTrajStep_Degree );

    m_searchResult = SearchResult::TRAJ_PREC_CHANGED;
}



/**************************************************************************************************
Описание:
возвращает шаг траектории
Аргументы:
* checkTrajStep_Degree: максимальное евклидово расстояние между проверяемыми точками траектории
Возврат: шаг траектории как максимальное евклидово расстояние между точками траектории
**************************************************************************************************/
float MotionPlanner::getTrajStep_Degree() const
{
    return m_pathGenerator.getStep_Degree();
}



/**************************************************************************************************
Описание:
возвращает шаг проверки траектории на наличие пересечений
Аргументы:
Возврат: шаг проверки траектории как максимальное евклидово расстояние 
между проверяемыми точками траектории
**************************************************************************************************/
float MotionPlanner::getTrajCheckStep_Degree() const
{
    return m_mapValidator.getSegCheckPrec_Degree();
}
