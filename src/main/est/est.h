/**************************************************************************************************
Описание

Класс, реализующий алгоритм планирования движения EST

Разработчик: Сибиряков Виктор
Заметки
* радиус r поиска ближайших соседей от опорной вершины - поле m_radiusGen
* расстояние d между парой вершин, принадлежащих разным деревьям, 
на котором деревья могут быть соединены - поле m_radiusConn
* если траектория получается длинной, попробовать снизить значения 
* если траектория ищется долго, попробовать увеличить значения

!!! ОЧЕНЬ ВАЖНО !!!
величины вышеописанных параметров всегда задаются квадратом желаемого значения, поскольку
при расчете всех расстояний используется метод Point::calcDistNoSqrt (config_space/point/point.cpp),
поэтому для этого используются приватные методы calcRadiusGen и calcRadiusConn, соответственно
желаемые значения ПО УМОЛЧАНИЮ лучше вписывать в инициализацию поля так:
float m_radiusGen = calcRadiusGen( init_radius_gen );
а чтобы менять значения в процессе работы - использовать методы класса Configurator,
в эти методы ОБЯЗАТЕЛЬНО передавать именно желаемое значение, а не его квадрат.
* Configurator - дружественный класс, чтобы менять параметры алгоритма EST.
**************************************************************************************************/



#pragma once

#include "main/config_space/graph/graph.h"
#include "main/config_space/free_cspace_validator/free_cspace_validator.h"
#include "main/config_space/uniform_config_gen/uniform_config_gen.h"




namespace motion_planner
{
    class Est;

    class Configurator;
}


class motion_planner::Est
{

public:

    friend class motion_planner::Configurator;

    explicit Est( config_space::graph::Graph & mapFreeCSpace, 
                  config_space::FreeCSpaceValidator & mapValidator ) :
        m_mapFreeCSpace( mapFreeCSpace ),
        m_mapValidator( mapValidator )
    {
        m_listCompStatus[ config_space::graph::start_node_pos ][ 0 ] = 0;
        m_listCompStatus[ config_space::graph::goal_node_pos ][ 0 ] = 1;
    }

    void expansion();

    void connection();

    bool areTreesConnected() const;

    void reset();

private:

    uint8_t m_samplesAmount = 4;

    const float aver_ref_length = 5.266f;
    const float max_ref_length = 9.710f;
    
    const float init_radius_gen = aver_ref_length;
    const float init_radius_conn = max_ref_length;

    float m_radiusGen = calcRadiusGen( init_radius_gen );
    float m_radiusConn = calcRadiusConn( init_radius_conn );

    config_space::graph::Graph & m_mapFreeCSpace;

    config_space::FreeCSpaceValidator & m_mapValidator;

    config_space::graph::generation::UniformConfigGen m_generator;

    bool m_flagTreesConnected = false;

    uint16_t m_listCompStatus[ 2 ][ config_space::graph::nodes_limit ] { { 0 }, { 0 } };
    uint16_t m_posListEndStart = 1;
    uint16_t m_posListEndGoal = 1;

    uint16_t m_treeId = config_space::graph::start_comp;

    uint16_t m_startCheckPos = 0;
    uint16_t m_goalCheckPos = 0;

    uint16_t & getListEnd();

    float calcProbabilityNode( config_space::graph::NodeId node ) const;

    void changeGrowingTree();

    void sampleNodesInArea( config_space::graph::NodeId refNodeId );

    void tryConnectSample( const config_space::Point & sampleConf,
                           const config_space::Point & refConfig,
                           config_space::graph::NodeId refNodeId );

    config_space::Point generateNearConfig( const config_space::Point & refConfig ) const;

    float calcProbabilityGenConfig( const config_space::Point & config );

    void addToList( config_space::graph::NodeId nodePos );

    bool tryConnectTrees( uint16_t listStartBegin, uint16_t listStartEnd,
                          uint16_t listGoalBegin, uint16_t listGoalEnd ) const;



    float calcRadiusGen( float radiusGen ) const;
    float calcRadiusConn( float radiusConn ) const;

};
