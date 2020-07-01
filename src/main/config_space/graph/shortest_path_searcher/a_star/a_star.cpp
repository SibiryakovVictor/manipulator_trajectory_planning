/**************************************************************************************************
Описание

Реализация алгоритма поиска кратчайшего пути по графу A*.

Разработчик: Сибиряков Виктор
Заметки
1) Используемая функция стоимости перехода calcCostNodesConfigs из одной вершины в другую сильно зависит 
от результата работы функции calcDistCoeffs, который, в свою очередь, зависит от диапазонов движения
звеньев, указанных в константе realRangesMotion в файле "tools/intervals.h". То есть меньший диапазон
даёт меньший вклад в результат функции, следовательно, кратчайший путь определяется также с точки
зрения выполнения большей части перемещения звеньями с меньшей подвижностью.
2) Функция стоимости перехода из текущей вершины в целевую - calcCostNodesConfigs 
**************************************************************************************************/



#include "a_star.h"
#include "main/config_space/point/point.h"
#include "main/tools/intervals/intervals.h"


using namespace motion_planner::path;
using namespace motion_planner::config_space::graph;
using namespace motion_planner::config_space;
using namespace motion_planner;



/**************************************************************************************************
Описание: 
находит кратчайший путь от стартовой вершины (под позицией config_space::graph::start_node_pos
в списке вершин) до целевой (под позицией config_space::graph::goal_node_pos в списке вершин)
(под позицией 
Аргументы:
* pathData: массив секций найденного пути; первая секция содержит позицию стартовой вершины, 
последняя (непустая) - позицию целевой вершины;
* nodesList: интерфейс для получения инфы о вершинах (реализован, например, 
классом config_space::graph::Graph);
* edgesList: интерфейс для получения вершины на другом конце ребра с помощью вершины и номера её ребра
(реализован, например, классом config_space::graph::Graph)
Возврат: количество секций найденного пути (включая старт и цель); 
если путь не найден, то возвращается 0
Замечания:
**************************************************************************************************/
path::PathElemId Astar::findShortestPath( path::Path & pathData, const INodesList & nodesList,
										  const IEdgesList & edgesList )
{
    static const float cost_threshold_zero = 0.0001f;

	float costToNodes[ nodes_limit ] { 0.f };

	path::PathElement previousInPath[ nodes_limit ];

	Astar::PriorityQueue openSet;
	openSet.putNodePriority( NodePriority( start_node_pos, 0.f ) );

	const auto & goalConfig = nodesList.getNodeConfig( goal_node_pos );

	while ( ! openSet.isEmpty() )
	{
		auto curNodePos = openSet.getMostPriorityNodePos();

		if ( curNodePos == goal_node_pos )
		{
			break;
		}

		const auto & curNodeConfig = nodesList.getNodeConfig( curNodePos );

		for ( EdgeId curEdge = 0; curEdge != edges_limit; curEdge++ )
		{
			auto curNeighborPos = edgesList.getNodePos( curNodePos, curEdge );

			if ( curNeighborPos == empty_node )
			{
				continue;
			}

			const auto & curNeighborConfig = nodesList.getNodeConfig( curNeighborPos );

			auto neighborCost = costToNodes[ curNodePos ] + 
				calcCostNodesConfigs( curNodeConfig, curNeighborConfig );

			if ( ( costToNodes[ curNeighborPos ] < cost_threshold_zero ) || 
				( neighborCost < costToNodes[ curNeighborPos ] ) )
			{
				costToNodes[ curNeighborPos ] = neighborCost;

				auto priority = neighborCost + 
					calcCostNodesConfigs( curNeighborConfig, goalConfig );

				openSet.putNodePriority( NodePriority( curNeighborPos, priority ) );

				previousInPath[ curNeighborPos ] = path::PathElement( curNodePos, curEdge );
			}
		}
	}


	if ( previousInPath[ goal_node_pos ].point == empty_node )
	{
		return 0;
	}


	return invertPath( pathData, previousInPath );
}




/**************************************************************************************************
Описание:
рассчитывает стоимость перехода между конфигурациями
Аргументы:
* config1: первая конфигурация
* config2: вторая конфигурация
Возврат: стоимость перехода между конфигурациями
Замечания: 
по сути рассчитывает евклидово расстояние, только не извлекает корень из конечного результата, и
домножает расстояние между проекциями на коэффициент m_distCoeff, определяемый в calcDistCoeff
**************************************************************************************************/
float Astar::calcCostNodesConfigs( const Point & config1, const Point & config2 )
{
	auto dim = 0;

	return std::inner_product( config1.begin(), config1.end(), config2.begin(), 0.f,
		[]( float arg, float result ){ return result + arg; },
		[ this, &dim ](float arg1, float arg2) 
		{
			float diff = ( arg1 - arg2 ) * m_distCoeff[ dim ];

			dim++;

			return diff * diff; 
		} );
}




/**************************************************************************************************
Описание:
располагает путь в результирующем массиве в верном порядке - от старта к цели
Аргументы:
* result: результирующий массив секций найденного пути
* prevInPath: массив предыдущих в пути вершин (заполнен в процессе работы метода findShortestPath)
Возврат: тот же, что и у findShortestPath
**************************************************************************************************/
path::PathElemId Astar::invertPath( Path & result, PathElement (&prevInPath)[ nodes_limit ] )
{
	path::PathElemId pointCounter = 0;

	NodeId nodePos = 1;

	while ( nodePos != start_node_pos )
	{
		result.sections[ pointCounter ] = prevInPath[ nodePos ];

		pointCounter++;

		nodePos = prevInPath[ nodePos ].point;
	}

	std::reverse( result.sections, result.sections + pointCounter );

	result.sections[ pointCounter ] = PathElement( goal_node_pos, empty_edge );
	pointCounter++;

	return pointCounter;
}




/**************************************************************************************************
Описание:
рассчитывает множитель, учитываемый при расчете функции стоимости перехода (метод calcCostNodesConfigs)
Аргументы:
Возврат:
**************************************************************************************************/
void Astar::calcDistCoeffs()
{
	for ( auto dim = 0; dim != conf_space_dims; dim++ )
	{
		const auto & interval = tools::RangeLinksMotion::get()[ dim ];

		m_distCoeff[ dim ] = std::fabsf( interval.gethb() - interval.getlb() );
	}
}

