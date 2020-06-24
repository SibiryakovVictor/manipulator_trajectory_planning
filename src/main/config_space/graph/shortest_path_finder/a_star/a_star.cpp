#include "a_star.h"

#include "main/config_space/point/point.h"
#include "main/tools/intervals/intervals.h"

using namespace motion_planner;
using namespace motion_planner::config_space;
using namespace motion_planner::config_space::graph;



uint16_t Astar::findShortestPath( motion_planner::path::Path & pathData,
	const INodesList & nodesList,
	const IEdgesList & edgesList )
{
	static const float cost_threshold_zero = 0.001f;

	float costToNodes[ capacity ] { 0.f };

	path::PathElement previousInPath[ capacity ];

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
			auto curNeighborPos = edgesList.getNodeId( curNodePos, curEdge );

			if ( curNeighborPos == UINT16_MAX )
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


	if ( previousInPath[ goal_node_pos ].point == UINT16_MAX )
	{
		return 0;
	}


	return invertPath( pathData, previousInPath );
}




float Astar::calcCostNodesConfigs( const config_space::Point & config1, 
	const config_space::Point & config2 )
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




uint16_t Astar::invertPath( motion_planner::path::Path & result,
	motion_planner::path::PathElement (&prevInPath)[ capacity ] )
{
	uint16_t pointCounter = 0;

	NodeId nodePos = 1;

	while ( nodePos != start_node_pos )
	{
		result.sections[ pointCounter ] = prevInPath[ nodePos ];

		pointCounter++;

		nodePos = prevInPath[ nodePos ].point;
	}

	std::reverse( result.sections, result.sections + pointCounter );

	result.sections[ pointCounter ] = path::PathElement( goal_node_pos, UINT8_MAX );
	pointCounter++;

	return pointCounter;
}



void Astar::calcDistCoeffs()
{
	for ( uint16_t dim = 0; dim != conf_space_dims; dim++ )
	{
		const auto & interval = tools::RangeLinksMotion::get()[ dim ];

		m_distCoeff[ dim ] = std::fabsf( interval.gethb() - interval.getlb() );
	}
}

