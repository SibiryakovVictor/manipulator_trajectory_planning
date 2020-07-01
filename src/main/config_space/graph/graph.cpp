/**************************************************************************************************
Описание

Реализация графа для хранения вершин и рёбер деревьев алгоритма EST.

Разработчик: Сибиряков Виктор
Заметки
* максимальное количество вершин графа - параметр nodes_limit в "config_space/graph/graph_parameters.h"
* максимальное количество рёбер у вершин - параметр edges_limit в "config_space/graph/graph_parameters.h"
**************************************************************************************************/



#include "graph.h"

#include <algorithm>

using namespace motion_planner::config_space::graph;
using namespace motion_planner::config_space;



/**************************************************************************************************
Описание:
присваивает конфигурацию свободной вершине на позиции m_nodesAmount
Аргументы:
* config: конфигурация
Возврат: позиция вершины, которой была назначена конфигурация
Замечания:
**************************************************************************************************/
NodeId Graph::addNode( const Point & config )
{

	if ( getNodesAmount() > ( nodes_limit - 1 ) )
	{
		while ( true )
		{
			; // в графе больше нет свободных мест для вершин
		}
	}
	
	m_nodesStorage[ m_nodesAmount ].config = config;

	auto addedNodePos = m_nodesAmount;

	m_nodesAmount++;

	return addedNodePos;
}




/**************************************************************************************************
Описание:
заносит идентификаторы звеньев в список их рёбер на свободные позиции в списке
Аргументы:
* node1Pos: позиция первой вершины в списке вершин
* node2Pos: позиция второй вершины в списке вершин
Возврат:
Замечания:
**************************************************************************************************/
void Graph::addEdge( NodeId node1Pos, NodeId node2Pos )
{

	Node & node1 =  m_nodesStorage[ node1Pos ];
	Node & node2 =  m_nodesStorage[ node2Pos ];


	if ( ( node1.amountEdgesConnected == edges_limit ) ||
		( node2.amountEdgesConnected == edges_limit ) )
	{
		while ( true )
		{
			; // одна из вершин уже не имеет свободных рёбер
		}
	}


	auto freeEdgeNode1 = node1.amountEdgesConnected;
	auto freeEdgeNode2 = node2.amountEdgesConnected;

	node1.edges[ freeEdgeNode1 ] = node2Pos;
	node1.amountEdgesConnected++;

	node2.edges[ freeEdgeNode2 ] = node1Pos;
	node2.amountEdgesConnected++;
}




/**************************************************************************************************
Описание:
обнуляет все рёбра у вершин и их количество ближайших соседей, а также количество вершин в графе и
свободную позицию
Аргументы:
Возврат:
Замечания:
**************************************************************************************************/
void Graph::reset()
{
	std::for_each( m_nodesStorage, m_nodesStorage + m_nodesAmount, 
		[]( Node & node)
		{
			node.reset();
		} );

	m_nodesAmount = 0;
}



bool Graph::isLimitNodesReached() const
{
	return m_nodesAmount > ( nodes_limit - 2 );
}



void Graph::increaseNodesInArea( NodeId nodePos )
{
	m_nodesStorage[ nodePos ].nodesInArea++;
}




NodeId Graph::getNodesInArea( NodeId nodePos ) const
{
	return m_nodesStorage[ nodePos ].nodesInArea;
}




void Graph::setNodesInArea( NodeId nodePos, uint16_t nodesInArea )
{
	m_nodesStorage[ nodePos ].nodesInArea = nodesInArea;
}




const Point & Graph::getNodeConfig( NodeId nodePos ) const
{
	return m_nodesStorage[ nodePos ].config;
}




NodeId Graph::getNodesAmount() const
{
	return m_nodesAmount;
}




bool Graph::hasNodeFreeEdge( NodeId nodePos ) const
{
	return ( m_nodesStorage[ nodePos ].amountEdgesConnected < edges_limit );
}




EdgeId Graph::getAmountFreeEdges( NodeId nodePos ) const
{
	return ( edges_limit - m_nodesStorage[ nodePos ].amountEdgesConnected );
}




NodeId Graph::getNodePos( NodeId nodePos, EdgeId edgePos ) const
{
	return m_nodesStorage[ nodePos ].edges[ edgePos ];
}




EdgeId Graph::getNodeAmountEdges( NodeId nodePos ) const
{
	return m_nodesStorage[ nodePos ].amountEdgesConnected;
}

