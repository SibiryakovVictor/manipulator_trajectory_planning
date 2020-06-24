#pragma once

#include "main/config_space/graph/graph.h"
#include "main/config_space/free_cspace_verifier/free_cspace_verifier.h"
#include "main/config_space/uniform_config_gen/uniform_config_gen.h"



namespace motion_planner
{
    class Est;
}


class motion_planner::Est
{

public:

    explicit Est( config_space::graph::Graph & mapFreeCSpace, 
                  config_space::FreeCSpaceVerifier & mapValidator ) :
        m_mapFreeCSpace( mapFreeCSpace ),
        m_mapValidator( mapValidator )
    {
        m_listCompStatus[ config_space::graph::start_node_pos ][ 0 ] = 0;
        m_listCompStatus[ config_space::graph::goal_node_pos ][ 0 ] = 1;
    }

    void expansion();

    bool isConnectionSuccessful();

    void reset();

private:

    static const uint16_t samples_amount = 4;

    const float aver_ref_length = 5.266f;
    const float max_ref_length = 9.710f;
    
    const float init_radius_gen = aver_ref_length;
    const float init_radius_conn = max_ref_length;
    static const uint16_t sample_nn_limit = 6;

    float m_radiusGen = std::powf( init_radius_gen, 2.0f );
    float m_radiusConn = std::powf( init_radius_conn, 2.0f );

    uint16_t m_posConnectGoal = config_space::graph::goal_node_pos;

    config_space::graph::Graph & m_mapFreeCSpace;

    config_space::FreeCSpaceVerifier & m_mapValidator;

    config_space::graph::generation::UniformConfigGen m_generator;

    uint16_t m_listCompStatus[ 2 ][ config_space::graph::capacity ] { { 0 }, { 0 } };
    uint16_t m_posListEndStart = 1;
    uint16_t m_posListEndGoal = 1;

    uint16_t m_treeId = config_space::graph::start_comp;

    uint16_t m_startCheckPos = 0;
    uint16_t m_goalCheckPos = 0;

    uint16_t & getListEnd();

    float calcProbabilityNode( NodeId node ) const;

    void changeGrowingTree();

    void sampleNodesInArea( NodeId refNodeId );

    void tryConnectSample( const config_space::Point & sampleConf, NodeId refNodeId,
                           const config_space::Point & refConfig );

    config_space::Point generateNearConfig( const config_space::Point & refConfig ) const;

    float calcProbabilityGenConfig( const config_space::Point & config );

    void addToList( NodeId nodePos );

    bool areTreesConnected( uint16_t listStartBegin, uint16_t listStartEnd,
                            uint16_t listGoalBegin, uint16_t listGoalEnd ) const;

};

