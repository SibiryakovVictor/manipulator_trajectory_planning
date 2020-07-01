/**************************************************************************************************
Описание

Реализация очереди с приоритетом для алгоритма A*.

Разработчик: Сибиряков Виктор
Заметки
Поддерживает первый элемент в очереди максимальным с помощью make_heap
Получает максимальный элемент в очереди с помощью pop_heap
**************************************************************************************************/


#include "a_star.h"

using namespace motion_planner::config_space;
using namespace motion_planner::config_space::graph;



bool Astar::PriorityQueue::isEmpty() const
{
	return ( ! m_containerPos );
}



void Astar::PriorityQueue::putNodePriority( Astar::NodePriority nodeInfo )
{
	m_queueContainer[ m_containerPos ] = nodeInfo;

	m_containerPos++;

	std::make_heap( m_queueContainer, m_queueContainer + m_containerPos,
		PriorityComparator() );
}



NodeId Astar::PriorityQueue::getMostPriorityNodePos()
{
	std::pop_heap( m_queueContainer, m_queueContainer + m_containerPos,
		PriorityComparator() );

	m_containerPos--;

	return m_queueContainer[ m_containerPos ].nodePos;
}




bool Astar::PriorityQueue::PriorityComparator::operator()( const Astar::NodePriority & n1, 
														   const Astar::NodePriority & n2 ) const
{
	return ( n1.priority > n2.priority );
}

