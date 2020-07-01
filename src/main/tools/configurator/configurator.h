/**************************************************************************************************
Описание

Класс, позволяющий настраивать объекты планировщика MotionPlanner

Разработчик: Сибиряков Виктор
Заметки
**************************************************************************************************/


#pragma once

#include "main/env/manipulator/manipulator_controller.h"
#include "main/env/obstacles/obstacles.h"
#include "main/config_space/free_cspace_validator/free_cspace_validator.h"
#include "main/est/est.h"


namespace motion_planner
{
    class Configurator;
}

class motion_planner::Configurator
{

public:

    explicit Configurator( env::ManipulatorController & manip,
                           env::Obstacles & obst,
                           config_space::FreeCSpaceValidator & validator,
                           Est & planner ) :
        m_manip( manip ),
        m_obst( obst ),
        m_validator( validator ),
        m_planner( planner )
    {}


    void changeObjectBody( uint16_t objId, env::Obb body );

    void rotateObst( uint16_t obstId, float angle_Rad, Eigen::Vector3f axis );

    void activateCollPair( uint16_t pairIndex );
    void deactivateCollPair( uint16_t pairIndex );

    void changeEstRadiusGen( float radiusGen );
    void changeEstRadiusConn( float radiusConn );

private:

    env::ManipulatorController & m_manip;
    env::Obstacles & m_obst;

    config_space::FreeCSpaceValidator m_validator;

    Est & m_planner;

};
