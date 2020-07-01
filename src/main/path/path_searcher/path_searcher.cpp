#include "path_searcher.h"

using namespace motion_planner::path;
using namespace motion_planner;



/**************************************************************************************************
Описание:
выполняет поиск кратчайшего пути и возвращает количество секций найденного пути;
секции найденного пути помещаются в переданную по ссылке структуру Path при инициализации
Аргументы:
Возврат: количество секций найденного пути
**************************************************************************************************/
uint16_t ShortestPathSearcher::findShortestPath()
{
    return m_searchAlgorithm.findShortestPath( m_pathSections, m_nodes, m_edges );
}



/**************************************************************************************************
Описание:
рассчитывает длину найденного пути как сумму длин его рёбер
Аргументы:
Возврат: количество секций найденного пути
**************************************************************************************************/
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



/**************************************************************************************************
Описание:
сохраняет в поле m_refPathLength длину отрезка, соединяющего стартовую и целевую конфигурации
Аргументы:
Возврат:
**************************************************************************************************/
void ShortestPathSearcher::reset()
{
    m_refPathLength = config_space::Point::calcDistance( 
        m_nodes.getNodeConfig( config_space::graph::start_node_pos ),
        m_nodes.getNodeConfig( config_space::graph::goal_node_pos ) );
}



/**************************************************************************************************
Описание:
возвращает длину отрезка, соединяющего стартовую и целевую конфигурации
Аргументы:
Возврат: длина отрезка, соединяющего стартовую и целевую конфигурации
**************************************************************************************************/
float ShortestPathSearcher::getRefPathLength() const
{
    return m_refPathLength;
}
