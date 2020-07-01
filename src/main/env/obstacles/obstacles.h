/**************************************************************************************************
Описание

Описывает контейнер препятствий

Разработчик: Сибиряков Виктор
Заметки
* Configurator - дружественный класс, чтобы менять ограничивающий объем звена и осуществлять
поворот вокруг заданной оси на заданный угол;
**************************************************************************************************/


#pragma once

#include "main/env/env_def.h"
#include "main/env/info_loader/info_loader.h"
#include "main/env/primitives/obb/obb.h"



namespace motion_planner
{
    class Configurator;

    namespace env
    {
        class Obstacles;
    }
}


class motion_planner::env::Obstacles
{
public:

    friend class motion_planner::Configurator;

    static const uint16_t biasObstId = UINT16_MAX / 2;

    explicit Obstacles( ObstInitInfo && objects = load::getObstInitInfo() ) :
        m_obstInfo( objects )
    {}

    const Obb & getBody( uint16_t obstBiasedId ) const;


private:

    ObstInitInfo m_obstInfo;

    void changeBody( uint16_t obstBiasedId, const Obb & body );

    void rotate( uint16_t obstBiasedId, float angle, Eigen::Vector3f axis );
};
