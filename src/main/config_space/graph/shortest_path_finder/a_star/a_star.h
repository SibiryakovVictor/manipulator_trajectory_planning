#pragma once

#include <algorithm>
#include <cfloat>


#include "../i_shortest_path_finder.h"
#include "main/config_space/graph/i_nodes_list.h"
#include "main/config_space/graph/i_edges_list.h"
#include "main/path/path_def.h"
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

    uint16_t findShortestPath( motion_planner::path::Path & pathData,
		const INodesList & nodesList, const IEdgesList & edgesList );

private:

	struct NodePriority
	{
		uint16_t nodePos = UINT16_MAX;

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

		uint16_t getMostPriorityNodePos();

	private:

		class PriorityComparator
		{

		public:

			bool operator()( const Astar::NodePriority & n1, 
				const Astar::NodePriority & n2 ) const;

		};

		Astar::NodePriority m_queueContainer[ capacity ];

		uint16_t m_containerPos = 0;					
	};

	float m_distCoeff[ conf_space_dims ] { 0.f };

	float calcCostNodesConfigs( const Point & config1, const Point & config2 );

	uint16_t invertPath( motion_planner::path::Path & result,
		motion_planner::path::PathElement (&prevInPath)[ capacity ] );

	void calcDistCoeffs();

};

