/**************************************************************************************************
Описание

Реализация графа для хранения вершин и рёбер деревьев алгоритма EST.

Разработчик: Сибиряков Виктор
Заметки
* максимальное количество вершин графа - параметр nodes_limit в "config_space/graph/graph_parameters.h"
* максимальное количество рёбер у вершин - параметр edges_limit в "config_space/graph/graph_parameters.h"
**************************************************************************************************/



#pragma once

#include "graph_parameters.h"
#include "i_nodes_list.h"
#include "i_edges_list.h"


namespace motion_planner
{
    namespace config_space
    {
        namespace graph
        {
            class Graph;
        }
    }
}


class motion_planner::config_space::graph::Graph : public INodesList, public IEdgesList
{

public:

    explicit Graph()
    {}

    NodeId addNode( const config_space::Point & config );

    void addEdge( NodeId node1Pos, NodeId node2Pos );

    void reset();

    bool isLimitNodesReached() const;

    void setNodesInArea( NodeId nodePos, uint16_t nodesInArea );

    void increaseNodesInArea( NodeId nodePos );

    
    // INodesList
    const Point & getNodeConfig( NodeId nodePos ) const final;

    NodeId getNodesInArea( NodeId nodePos ) const final;

    NodeId getNodesAmount() const final;

    bool hasNodeFreeEdge( NodeId nodePos ) const final;

    EdgeId getNodeAmountEdges( NodeId nodePos ) const final;

    EdgeId getAmountFreeEdges( NodeId nodesPos ) const final;


    // IEdgesList
    NodeId getNodePos( NodeId nodePos, EdgeId edgePos ) const final;

private:

    struct Node
    {
        config_space::Point config;

        NodeId edges[ edges_limit ];

        EdgeId amountEdgesConnected = 0;

        NodeId nodesInArea = 1;

        Node()
        {
            std::fill( edges, edges + edges_limit, empty_node );
        }

        void reset()
        {
            std::fill( edges, edges + amountEdgesConnected, empty_node );

            nodesInArea = 1;

            amountEdgesConnected = 0;
        }
    };

    Node m_nodesStorage[ nodes_limit ];

    NodeId m_nodesAmount = 0;

};
