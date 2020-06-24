#include "graph.h"

#include <algorithm>

using namespace motion_planner::config_space;
using namespace graph;




Graph::Graph() :
	m_nodesStorage( new Node[ capacity ] )
{}



Graph::~Graph()
{
	delete[] m_nodesStorage;
}



NodeId Graph::insertNode( const Point & config )
{

	if ( getNodesAmount() > ( capacity - 2 ) )
	{

		while ( true )
		{
			; /* в графе больше нет свободных мест для вершин */
		}

	}
	
	m_nodesStorage[ m_freeNodePos ].m_config = config;

	m_nodesStorage[ m_freeNodePos ].m_isNodeInserted = true;

	NodeId addedNodePos = m_freeNodePos;

	m_nodesInvolved++;

	updateAvailableIndex();

	verifyLastPos( addedNodePos );

	return addedNodePos;
}



void Graph::decreaseFreeEdges( uint16_t nodePos )
{

	m_nodesStorage[ nodePos ].m_curFreeEdge++;

}



const Point & Graph::getNodeConfig( NodeId nodePos ) const
{
	return m_nodesStorage[ nodePos ].m_config;
}




uint16_t Graph::getNodesAmount() const
{
	return m_nodesInvolved;
}




bool Graph::hasNodeFreeEdge( NodeId nodePos ) const
{
	return ( m_nodesStorage[ nodePos ].m_curFreeEdge < edges_limit );
}




uint8_t Graph::getAmountFreeEdges( NodeId nodePos ) const
{
	return ( edges_limit - m_nodesStorage[ nodePos ].m_amountEdgesConnected );
}



uint16_t Graph::getPosLastNode() const
{
	return m_lastNodePos;
}



NodeId Graph::getNodeId( NodeId nodePos, EdgeId edgePos ) const
{
	return m_nodesStorage[ nodePos ].edges[ edgePos ];
}




bool Graph::isNodeInserted( NodeId nodePos ) const
{
	return m_nodesStorage[ nodePos ].m_isNodeInserted;
}




bool Graph::isNodeInGraph( uint16_t nodePos ) const
{
	return ( m_nodesStorage[ nodePos ].m_amountEdgesConnected != 0 );  
}




void Graph::reset()
{
	const auto lastPos = getPosLastNode();

	for ( NodeId curNodePos = 0; curNodePos != lastPos + 1; curNodePos++ )
	{
		m_nodesStorage[ curNodePos ].resetFull();
	}

	m_freeNodePos = 0;

	m_nodesInvolved = 0;

	m_lastNodePos = 0;
}




uint16_t Graph::getNodeAmountEdges( NodeId nodePos ) const
{
	return m_nodesStorage[ nodePos ].m_amountEdgesConnected;
}




void Graph::verifyFreeEdgePos( NodeId nodePos, EdgeId testingEdge )
{

	Node & node = m_nodesStorage[ nodePos ];

	if ( testingEdge < node.m_curFreeEdge )
	{
		node.m_curFreeEdge = testingEdge;
	}

}



void Graph::findFreeEdgePos( NodeId nodePos )
{
	static const uint8_t border = edges_limit - 2;


	Node & node = m_nodesStorage[ nodePos ];	

	if ( node.m_curFreeEdge > border )
	{
		node.m_curFreeEdge = edges_limit;

		return;
	}


	uint8_t possiblePos = node.m_curFreeEdge + 1;

	while ( possiblePos != edges_limit )
	{

		if ( node.edges[ possiblePos ] == UINT16_MAX )
		{
			node.m_curFreeEdge = possiblePos;

			return;
		}

		possiblePos++;

	}

	node.m_curFreeEdge = edges_limit;

}


void Graph::addEdge( NodeId node1Pos, NodeId node2Pos )
{

	Node & node1 =  m_nodesStorage[ node1Pos ];
	Node & node2 =  m_nodesStorage[ node2Pos ];


	if ( ( node1.m_curFreeEdge == edges_limit ) ||
		( node2.m_curFreeEdge == edges_limit ) )
	{
		while ( true )
		{
			; /* один из узлов уже имеет максимальное количество рёбер */
		}
	}

	
	uint8_t freeEdgeNode1 = node1.m_curFreeEdge;
	uint8_t freeEdgeNode2 = node2.m_curFreeEdge;

	node1.edges[ freeEdgeNode1 ] = node2Pos;
	node1.m_amountEdgesConnected++;
	node2.m_orderConnection[ freeEdgeNode2 ] = freeEdgeNode1;

	node2.edges[ freeEdgeNode2 ] = node1Pos;
	node2.m_amountEdgesConnected++;
	node1.m_orderConnection[ freeEdgeNode1 ] = freeEdgeNode2;

	findFreeEdgePos( node1Pos );
	findFreeEdgePos( node2Pos );

}



void Graph::removeEdge( NodeId nodePos, EdgeId edgePos )
{

	Node & node = m_nodesStorage[ nodePos ];

	EdgeId edgeNeighbor = node.m_orderConnection[ edgePos ];

	NodeId neighborPos = node.edges[ edgePos ];

	Node & nodeNeighbor = m_nodesStorage[ neighborPos ]; 

	node.edges[ edgePos ] = UINT16_MAX;
	node.m_amountEdgesConnected--;
	node.m_orderConnection[ edgePos ] = UINT8_MAX;

	if ( ( node.m_amountEdgesConnected == 0 ) && ( nodePos > 1 ) )
	{
		resetNodeNotConn( nodePos );
	}
	else
	{
		verifyFreeEdgePos( nodePos, edgePos );
	}

	nodeNeighbor.edges[ edgeNeighbor ] = UINT16_MAX; 
	nodeNeighbor.m_amountEdgesConnected--;
	nodeNeighbor.m_orderConnection[ edgeNeighbor ] = UINT8_MAX;

	if ( ( nodeNeighbor.m_amountEdgesConnected == 0 ) && ( neighborPos > 1 ) )
	{
		resetNodeNotConn( neighborPos );
	}
	else
	{
		verifyFreeEdgePos( neighborPos, edgeNeighbor );
	}

}



void Graph::removeNode( NodeId nodePos )
{

	Node & node = m_nodesStorage[ nodePos ];

	if ( isNodeInGraph( nodePos ) )
	{
		for ( uint8_t curEdge = 0; curEdge != edges_limit; curEdge++ )
		{
			NodeId neighborId = node.edges[ curEdge ];

			if ( neighborId  != UINT16_MAX )
			{
				removeEdge( nodePos, curEdge );
			}
		}

		node.m_amountEdgesConnected = 0;
	}

	if ( isNodeInserted( nodePos ) )
	{
		resetNodeNotConn( nodePos );
	}

}



void Graph::updateAvailableIndex()
{

	static const uint16_t indexBorder = capacity - 2;

	if ( m_freeNodePos > indexBorder )
	{
		m_freeNodePos = capacity - 1;
		return;
	}

	uint16_t possiblePos = m_freeNodePos + 1;

	while ( possiblePos < ( indexBorder + 1 ) )
	{

		if ( ! isNodeInserted( possiblePos ) )
		{
			m_freeNodePos = possiblePos;

			return;
		}

		possiblePos++;

	}

	m_freeNodePos = possiblePos;

}



void Graph::verifyAvailableIndex( NodeId testingNodePos )
{

	if ( testingNodePos < m_freeNodePos )
	{
		m_freeNodePos = testingNodePos;
	}

}



void Graph::resetNodeNotConn( NodeId nodePos )
{
	m_nodesStorage[ nodePos ].resetNotConn();

	m_nodesInvolved--;

	verifyAvailableIndex( nodePos );

	updateLastPos( nodePos );
}



void Graph::verifyLastPos( NodeId nodePos )
{
	if ( nodePos > m_lastNodePos )
	{
		m_lastNodePos = nodePos;
	}
}



void Graph::updateLastPos( NodeId nodePos )
{
	if ( nodePos != m_lastNodePos )
	{
		return;
	}

	m_lastNodePos = 0;

	while ( nodePos != 0 )
	{
		nodePos--;

		if ( isNodeInserted( nodePos ) )
		{
			m_lastNodePos = nodePos;

			break;
		}
	}
}



void Graph::increaseNodesInArea( NodeId node )
{
	m_nodesStorage[ node ].m_nodesInArea++;
}



uint8_t Graph::getNodesInArea( NodeId nodePos ) const
{
	return m_nodesStorage[ nodePos ].m_nodesInArea;
}



void Graph::setNodesInArea( NodeId node, uint16_t nodesInArea )
{
	m_nodesStorage[ node ].m_nodesInArea = nodesInArea;
}

