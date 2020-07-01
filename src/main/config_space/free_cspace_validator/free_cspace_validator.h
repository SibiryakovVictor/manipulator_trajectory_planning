/**************************************************************************************************
Описание

Реализация класса, проверяющего принадлежность точек и отрезков конфигурационного пространства 
к его свободным регионам (к тем, в которых не происходит пересечений с препятствиями и звеньев между собой)

Разработчик: Сибиряков Виктор
Заметки
* евклидово расстояние между проверяемыми точками отрезка по умолчанию: 
константа check_traj_prec_Degrees в "path/path_parameters.h", но можно задать с помощью
setSegCheckPrec (в градусах);
* Configurator - дружественный класс, чтобы рулить таблицей сталкиваемых объектов
**************************************************************************************************/



#pragma once


#include "main/env/manipulator/manipulator_controller.h"
#include "main/env/obstacles/obstacles.h"
#include "main/env/collision_detection/collision_detector.h"
#include "main/env/collision_detection/collision_check_pairs.h"
#include "main/env/collision_detection/collision_table.h"
#include "main/config_space/segment/segment.h"
#include "main/path/path_parameters.h"




namespace motion_planner
{
    class Configurator;

    namespace config_space
    {
        class FreeCSpaceValidator;
    }
}


class motion_planner::config_space::FreeCSpaceValidator
{
public:

    friend class motion_planner::Configurator;

    explicit FreeCSpaceValidator( env::ManipulatorController & manip, 
                                  env::Obstacles & obst,
                                  float segCheckPrec_Degree = path::check_traj_prec_Degrees ) :
        m_manip( manip ),
        m_obst( obst ),
        m_segDimStep_Rad( path::calcMaxDimValue_Rad( segCheckPrec_Degree ) )
    {
        fillListPosConfigChange();
    }

    bool isConfigInFreeCSpace( const config_space::Point & config ); 

    bool isSegmentInFreeCSpace( const config_space::Point & start, const config_space::Point & end );

    void setSegCheckPrec( float segCheckPrec_Degree );

    float getSegCheckPrec_Degree() const;


private:

    env::ManipulatorController & m_manip;

    env::Obstacles & m_obst;

    coll::CollisionDetector m_collDetect;

    env::CollisionTable m_collTable;

    float m_segDimStep_Rad = path::calcMaxDimValue_Rad( 0.5f );

    int m_listPosConfigChange[ conf_space_dims + 1 ] { 0 };

    bool checkSegmentRoughPhase( const config_space::Segment & segment );


    bool isSegmentCollided_Precision( const config_space::Segment & segment, float prec );
    bool isSegmentCollided_Points( const config_space::Segment & segment, uint16_t pointsCheck );



    bool checkManipVsManip( uint16_t id1, uint16_t id2 ) const;
    bool checkManipVsObst( uint16_t manipId, uint16_t obstId ) const;

    bool checkGroupVsLink( uint16_t groupId, uint16_t linkId ) const;
    bool checkGroupVsGroup( uint16_t groupId1, uint16_t groupId2 ) const;
    bool checkGroupVsObst( uint16_t groupId, uint16_t obstId ) const;

    uint16_t moveToNextGroup( uint16_t pairPos, uint16_t confPos ) const;



    void fillListPosConfigChange();
};

