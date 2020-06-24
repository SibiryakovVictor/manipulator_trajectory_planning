#pragma once

#include "typedefs.h"
#include "graph_parameters.h"
#include "main/config_space/point/point.h"

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

    explicit Graph();
    ~Graph();

    NodeId insertNode( const config_space::Point & config );
    void removeNode( NodeId nodePos );

    void addEdge( NodeId node1, NodeId node2 );
    void removeEdge( NodeId node, EdgeId edge );

    void setNodesInArea( NodeId node, uint16_t nodesInArea );

    void increaseNodesInArea( NodeId node );

    uint16_t getNodeAmountEdges( NodeId nodePos ) const;

    void reset();

    /* INodesList */
    const Point & getNodeConfig( NodeId nodePos ) const final;

    uint16_t getNodesAmount() const final;

    NodeId getPosLastNode() const final;

    uint8_t getNodesInArea( NodeId nodePos ) const final;

    bool hasNodeFreeEdge( NodeId nodePos ) const final;

    uint8_t getAmountFreeEdges( NodeId nodesPos ) const final;

    bool isNodeInserted( NodeId nodePos ) const final;

    /* IEdgesList */
    NodeId getNodeId( NodeId nodePos, EdgeId edgePos ) const final;


    

private:

    struct Node
    {

        config_space::Point m_config;

        uint8_t m_orderConnection[ edges_limit ];

        NodeId edges[ edges_limit ];

        uint16_t m_curFreeEdge = 0;

        uint16_t m_amountEdgesConnected = 0;

        uint8_t m_nodesInArea = 1;

        bool m_isNodeInserted = false;

        Node()
        {
            std::fill( edges, edges + edges_limit, UINT16_MAX );

            std::fill( m_orderConnection, 
                m_orderConnection + edges_limit, UINT8_MAX );
        }

        void resetFull()
        {

            std::fill( edges, edges + edges_limit, UINT16_MAX );

            std::fill( m_orderConnection, m_orderConnection + edges_limit,
                UINT8_MAX );

            m_nodesInArea = 1;

            m_curFreeEdge = 0;

            m_amountEdgesConnected = 0;

            m_isNodeInserted = false;

        }

        void resetNotConn()
        {
            m_curFreeEdge = 0;

            m_isNodeInserted = false;
        }

    };

    Node * m_nodesStorage;

    uint16_t m_freeNodePos = 0;

    uint16_t m_lastNodePos = 0;

    uint16_t m_nodesInvolved = 0;

    void updateAvailableIndex();
    void updateLastPos( NodeId nodePos );

    void verifyAvailableIndex( NodeId testingNodePos );
    void verifyLastPos( NodeId nodePos );

    void decreaseFreeEdges( uint16_t nodePos );

    bool isNodeInGraph( uint16_t nodePos ) const;

    void verifyFreeEdgePos( NodeId nodePos, EdgeId testingEdge );

    void findFreeEdgePos( NodeId nodePos );

    void resetNodeNotConn( NodeId nodePos );

};
