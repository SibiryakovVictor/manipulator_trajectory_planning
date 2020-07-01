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



#pragma once

#include <algorithm>
#include <cfloat>


#include "../i_shortest_path_searcher.h"
#include "main/config_space/graph/i_nodes_list.h"
#include "main/config_space/graph/i_edges_list.h"
#include "main/path/path_typedefs.h"
#include "main/config_space/graph/graph_parameters.h"



namespace motion_planner
{
	namespace config_space
	{
		namespace graph
		{
			class Astar;
		}
	}
}


class motion_planner::config_space::graph::Astar : public IShortestPathFinder
{

public:

    explicit Astar()
	{
		calcDistCoeffs();
	}

    path::PathElemId findShortestPath( path::Path & pathData, const INodesList & nodesList, 
									   const IEdgesList & edgesList );

private:

	struct NodePriority
	{
		NodeId nodePos = empty_node;

		float priority = FLT_MAX;
        
        explicit NodePriority()
        {}
        
        NodePriority( uint16_t pos, float prior ) :
            nodePos( pos ),
            priority( prior )
		{}
	};

	class PriorityQueue
	{

	public:

		explicit PriorityQueue()
		{}

		bool isEmpty() const;

		void putNodePriority( Astar::NodePriority nodeInfo );

		NodeId getMostPriorityNodePos();

	private:

		class PriorityComparator
		{

		public:

			bool operator()( const Astar::NodePriority & n1, 
						     const Astar::NodePriority & n2 ) const;

		};

		Astar::NodePriority m_queueContainer[ nodes_limit ];

		NodeId m_containerPos = 0;					
	};

	float m_distCoeff[ conf_space_dims ] { 0.f };

	float calcCostNodesConfigs( const Point & config1, const Point & config2 );

	path::PathElemId invertPath( path::Path & result, 
								 path::PathElement (&prevInPath)[ nodes_limit ] );

	void calcDistCoeffs();

};
